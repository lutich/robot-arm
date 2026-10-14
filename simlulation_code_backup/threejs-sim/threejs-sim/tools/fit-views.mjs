// Fits the camera and the arm pose of each reference photo to the points marked in it.
//
//   node tools/fit-views.mjs                   fit every view in tools/photo-views.json that has "points", print the result
//   node tools/fit-views.mjs --only park-side  views whose photo name contains "park-side"
//   node tools/fit-views.mjs --joint           photos of the same pose (same "pose", no "counts") share one set of
//                                              joint angles and one mat offset d; each keeps its own camera
//   node tools/fit-views.mjs --save            write camera, angles and fit back into photo-views.json
//
// A view's "points" maps keypoint names (js/viewfit.js KEYPOINTS) to pixels [u, v] or [u, v, weight] in the
// photo. "fit" sets the options: free (joints to fit, default s1..s4), fitD (mat offset), prior (deg),
// fov / fovSigma (prior on the vertical field of view, deg), start (angles to start from), d (mat offset),
// skip (a reason: the view keeps its hand-set camera and is left out of every fit).
// The start is the view's current camera and the angles of its pose's counts.
// Then render with: node tools/compare.mjs --keypoints. Procedure: docs/model-photo-comparison.md.

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { SERVOS } from '../js/config.js';
import { anglesFromCounts } from '../js/kinematics.js';
import { fitView, fitViews, gapForCounts } from '../js/viewfit.js';

const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const only = opt('--only', '');
const save = args.includes('--save');
const viewsFile = path.resolve(opt('--views', path.join(ROOT, 'tools', 'photo-views.json')));
const data = JSON.parse(fs.readFileSync(viewsFile, 'utf8'));

// photo size from the JPEG header (SOF marker), no image library needed
function jpegSize(file) {
  const b = fs.readFileSync(file);
  for (let i = 2; i < b.length;) {
    const m = b[i + 1], len = b.readUInt16BE(i + 2);
    if (m >= 0xc0 && m <= 0xcf && ![0xc4, 0xc8, 0xcc].includes(m)) return { height: b.readUInt16BE(i + 5), width: b.readUInt16BE(i + 7) };
    i += 2 + len;
  }
  throw new Error(`no size in ${file}`);
}

const POSES = { HOME: SERVOS.map(s => s.home), PARK: SERVOS.map(s => s.park) };
const photoSize = v => jpegSize(path.join(ROOT, '..', 'docs', 'images', 'photos', v.photo));
const fovPrior = o => o.fov ? [o.fov, o.fovSigma ?? 8] : null;
const worstOf = res => Object.entries(res).map(([n, e]) => [n, Math.hypot(...e)]).sort((a, b) => b[1] - a[1]).slice(0, 4)
  .map(([n, e]) => `${n} ${e.toFixed(0)}`).join(', ');
const selected = data.views.filter(v => v.points && v.photo.includes(only) && !v.fit?.skip);
for (const v of data.views) if (v.fit?.skip && v.photo.includes(only)) console.log(`${v.photo}: skipped, ${v.fit.skip}`);

if (args.includes('--joint')) {
  const groups = {};
  for (const v of selected) if (!v.counts) (groups[v.pose || 'HOME'] ??= []).push(v);
  for (const [pose, vs] of Object.entries(groups)) {
    const counts = POSES[pose];
    const list = vs.map(v => ({ points: v.points, ...photoSize(v), camera: v.camera, fovPrior: fovPrior(v.fit || {}) }));
    const r = fitViews(list, { init: { angles: anglesFromCounts(counts), gap: gapForCounts(counts[5]) }, fitD: true });
    console.log(`${pose}, ${vs.length} photos: angles ${JSON.stringify(r.angles)}, d ${r.d}`);
    vs.forEach((v, k) => {
      const o = r.views[k];
      console.log(`  ${v.photo}: rms ${o.rms} px, fov ${o.camera.fov}, pos ${o.camera.pos}; worst: ${worstOf(o.residuals)}`);
      if (save) { v.camera = o.camera; v.angles = r.angles; v.fit = { ...(v.fit || {}), rms: o.rms, joint: true, dFit: r.d }; }
    });
  }
} else {
  for (const v of selected) {
    const { width, height } = photoSize(v);
    const counts = v.counts || POSES[v.pose || 'HOME'];
    const o = v.fit || {};
    const init = { camera: v.camera, angles: { ...anglesFromCounts(counts), ...(o.start || {}) }, gap: gapForCounts(counts[5]), d: o.d };
    const r = fitView(v.points, { width, height, init, free: o.free, fitD: o.fitD, prior: o.prior ?? 60, fovPrior: fovPrior(o) });
    console.log(`${v.photo}: rms ${r.rms} px over ${Object.keys(r.residuals).length} points, fov ${r.camera.fov}, pos ${r.camera.pos}, ` +
      `angles ${JSON.stringify(r.angles)}${o.fitD ? `, d ${r.d}` : ''}\n  worst: ${worstOf(r.residuals)}`);
    if (save) {
      const { joint, dFit, ...rest } = o;
      v.camera = r.camera; v.angles = r.angles;
      v.fit = { ...rest, rms: r.rms, ...(o.fitD ? { dFit: r.d } : {}) };
    }
  }
}
if (save) {
  // one view per line, as the file was written by hand
  const lines = data.views.map(v => '    ' + JSON.stringify(v));
  fs.writeFileSync(viewsFile, `{\n  "about": ${JSON.stringify(data.about)},\n  "views": [\n${lines.join(',\n')}\n  ]\n}\n`);
  console.log('saved', path.relative(process.cwd(), viewsFile));
}
