// Live arm tracking off the page's main thread. The page keeps one of these workers per camera, so the cameras'
// frames are fetched, decoded and searched for tags at the same time on separate CPU cores; the joint fit, which
// needs all cameras, then runs in one of them.
//   { type: 'tags', cam: { id, url, cal }, start, quick, w, h }  one snapshot (GET only), the arm tags and the cubes in it
//   { type: 'solve', views: [{ id, cal, tags }], commanded, last, useCorners }  js/armsolve.js solveArm
//   { type: 'cubes', views: [{ cal, detections }], arm }  the cubes in 3D from all cameras (cubetrack.js trackCubes)
//   { type: 'gap', cal, angles, gaps, w, h }  the jaw opening scores on this worker's last frame (armsolve.js jawProfile)
// Every message carries the page's current tag layout and mat offset (both can change in the page).

import { ARM_TAGS, MAT } from './config.js';
import { findTags, solveArm, jawProfile, gapScores, screwScores } from './armsolve.js';
import { detectCubes } from './detect.js';
import { trackCubes } from './cubetrack.js';

self.onmessage = async ({ data: q }) => {
  const t0 = performance.now();
  for (const t of q.layout) Object.assign(ARM_TAGS.find(x => x.key === t.key) || {}, t);
  MAT.sToEdge = q.d;
  if (q.type === 'tags') {
    const c = q.cam;
    try {
      const r = await fetch(c.url, { cache: 'no-store' });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const blob = await r.blob();
      const bmp = await createImageBitmap(blob);
      const canvas = new OffscreenCanvas(bmp.width, bmp.height), g = canvas.getContext('2d', { willReadFrequently: true });
      g.drawImage(bmp, 0, 0); bmp.close();
      const img = g.getImageData(0, 0, canvas.width, canvas.height);
      self.lastImg = img;                                 // kept for the jaw-opening check after the fit
      const t1 = performance.now();
      const tags = findTags(img, c.cal, q.start, { quick: q.quick, w: q.w, h: q.h });
      const t2 = performance.now();
      const cubes = q.cubes ? detectCubes(img, c.cal, { frameW: q.w, frameH: q.h }) : [];
      self.postMessage({ seq: q.seq, id: c.id, blob, tags, cubes, ms: { frame: Math.round(t1 - t0), tags: Math.round(t2 - t1), cubes: Math.round(performance.now() - t2) } });
    } catch (e) { self.postMessage({ seq: q.seq, id: c.id, blob: null, tags: {}, cubes: [], error: e.message, ms: {} }); }
  } else if (q.type === 'cubes') {
    const r = trackCubes(q.views, q.arm);
    self.postMessage({ seq: q.seq, cubes: [...r], onArm: r.onArm, ms: Math.round(performance.now() - t0) });   // (an array's extra fields do not survive postMessage)
  } else if (q.type === 'gap') {
    let r = { scores: q.gaps.map(() => null), mids: q.gaps.map(() => null), pxPerCm: 0 };
    if (self.lastImg) {
      const p = jawProfile(self.lastImg, q.cal, q.angles, { w: q.w, h: q.h }); r = { ...gapScores(p, q.gaps), pxPerCm: p.pxPerCm };
      // the screw heads near the gripper (shown in the camera views; not yet used for the gap: the model's screw layout is rough)
      const s = screwScores(self.lastImg, q.cal, q.angles, q.gaps, { w: q.w, h: q.h }); r.screws = { spots: s.spots, side: s.side, shifts: s.shifts };
    }
    self.postMessage({ seq: q.seq, ...r });
  } else if (q.type === 'solve') {
    const solved = solveArm(q.views, { commanded: q.commanded, last: q.last, useCorners: q.useCorners });
    self.postMessage({ seq: q.seq, solved, ms: { fit: Math.round(performance.now() - t0) } });
  }
};
