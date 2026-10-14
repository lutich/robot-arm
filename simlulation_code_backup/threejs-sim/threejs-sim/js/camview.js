// Camera tab: what each camera (config.js CAMERAS: the OAK and the webcam) sees in the simulation,
// the real frame on top of it, its calibration from the printed mat markers, cube recognition, and
// tracking the real arm by the AprilTags on it, seen by both cameras (armtrack.js).
//
// Read only: the real frames come from a saved file, a file you pick, or GETs of the controller's
// camera API (/api/camera/* for the OAK, /api/camera2/* for the webcam). Nothing is ever sent.

import * as THREE from 'three';
import { CAMERA, CAMERAS, ARM_TAGS, ARM_PARTS, MAT } from './config.js';
import { calibrate, project, MARKER_CORNERS, SAVED_FRAME_CORNERS, markerCentre } from './calib.js';
import { detectCubes } from './detect.js';
import { detectMarkers } from './aruco.js';
import { calFromLookAt, detectArmTags, triangulate, fitArmPose, tagPoints, projectBoard, boardToTable } from './armtrack.js';
import { locateCubes } from './locate3d.js';
import { fitPartPose, learnTagLayout } from './rigidtrack.js';
import { findTags, solveArm, pickGap, gripperScrews } from './armsolve.js';
import { recalibrate, RECAL_PARAMS, layoutFor, applyRecalibration } from './recalib.js';
import { fitJointPair, estimateJointAxis } from './jointfit.js';
import { armShapes, LINK } from './collision.js';
import { fk, point } from './kinematics.js';
import { CUBE } from './config.js';

const W = CAMERA.w, H = CAMERA.h;
const COLOR_CSS = { orange: '#e8792b', blue: '#2f6fd6', yellow: '#d9a800', green: '#2f9b47', red: '#d23b3b' };
const $ = id => document.getElementById(id);
const deg = r => r * 180 / Math.PI;
const partCss = link => '#' + ARM_PARTS[link].colour.toString(16).padStart(6, '0');
const tagById = id => ARM_TAGS.find(t => String(t.id) === String(id));

/** Table frame (mm) -> board frame (cm). */
export function tableToBoard([X, Y, Z], d = MAT.sToEdge) { return [-Y / 10, X / 10 + d, Z / 10]; }
const tableDirToBoard = v => [-v[1], v[0], v[2]];

/** Place a three.js camera at a calibrated pinhole camera (R, t in table mm) with its exact intrinsics. */
export function applyPinhole(cam3, cal, width, height, d = MAT.sToEdge) {
  const C = tableToBoard(cal.centre, d);
  const x = tableDirToBoard(cal.R[0]), y = tableDirToBoard(cal.R[1]), z = tableDirToBoard(cal.R[2]);
  const m = new THREE.Matrix4().makeBasis(new THREE.Vector3(...x), new THREE.Vector3(...y).negate(), new THREE.Vector3(...z).negate());
  cam3.position.set(...C);
  cam3.quaternion.setFromRotationMatrix(m);
  // scale K by the height, keep the principal point offset; a wider canvas just shows more
  const k = height / H, fx = cal.f * k, fy = cal.f * k, cx = width / 2 + (cal.cx - W / 2) * k, cy = cal.cy * k;
  const n = 1, f = 2000;
  cam3.projectionMatrix.set(
    2 * fx / width, 0, 1 - 2 * cx / width, 0,
    0, 2 * fy / height, 2 * cy / height - 1, 0,
    0, 0, -(f + n) / (f - n), -2 * f * n / (f - n),
    0, 0, -1, 0);
  cam3.projectionMatrixInverse.copy(cam3.projectionMatrix).invert();
  cam3.updateMatrixWorld(true);
}

/** Pairs from the corners vision/blocks.py found in the saved frame. */
function savedPairs() {
  const pts = [];
  for (const id of Object.keys(SAVED_FRAME_CORNERS)) MARKER_CORNERS[id].forEach((p, k) => pts.push({ id: Number(id), k, X: [p[0], p[1], 0], uv: SAVED_FRAME_CORNERS[id][k] }));
  return pts;
}

/** The default camera: fitted to the saved frame's marker corners (same as vision/blocks.py). */
export function defaultCalibration() {
  const p = savedPairs();
  return calibrate(p.map(q => q.X), p.map(q => q.uv));
}

/**
 * Wire up the camera tab for every camera in config.js CAMERAS.
 * ctx: { scene, cals: { [id]: cal }, hideForCamera(fn), linkBase(), onApply(cal, id), onArmTrack(result | null),
 *        armAngles(), log, toast, useDetections(list), trackDetections(list) }
 * Each camera has its own frame element, source, calibration and detections; the selector shows one.
 * Arm tracking reads the AprilTags in every camera that has a frame and combines them (armtrack.js).
 */
export function initCameraTab(ctx) {
  const glCanvas = document.createElement('canvas');
  const renderer = new THREE.WebGLRenderer({ canvas: glCanvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(1);
  renderer.setSize(960, 540, false);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  const cam3 = new THREE.PerspectiveCamera();
  const fullscreen = el => (document.fullscreenElement === el ? document.exitFullscreen() : el.requestFullscreen?.()).catch?.(() => {});
  // ---- one state and one stage per camera, side by side (the first stage is in the HTML)
  const states = {};
  CAMERAS.forEach((c, i) => {
    let stage = $('cam-stage-0');
    if (i > 0) {
      stage = document.createElement('div'); stage.className = 'cam-stage';
      stage.innerHTML = '<canvas width="960" height="540"></canvas><img alt="" style="opacity:0.5"><canvas></canvas>';
      $('cam-stages').append(stage);
    }
    const [sim, overlay] = stage.querySelectorAll('canvas'), img = stage.querySelector('img');
    overlay.width = 1280; overlay.height = 720;
    const cap = document.createElement('div'); cap.className = 'cam-cap'; cap.textContent = c.name; stage.append(cap);
    // options inside the view, so they work in full screen too: they flip the Camera tab's checkboxes
    const bar = document.createElement('div'); bar.className = 'cam-bar';
    const opt = (label, id, title) => { const b = document.createElement('button'); b.textContent = label; b.title = title; b.dataset.for = id; b.onclick = e => { e.stopPropagation(); const c = $(id); c.checked = !c.checked; c.dispatchEvent(new Event('change')); syncBars(); }; return b; };
    bar.append(
      opt('Sim', 'cam-show-sim', 'Simulated view under the frame'), opt('Mat', 'cam-show-grid', 'Mat grid and axes'), opt('Marks', 'cam-show-marks', 'Markers and model tags'),
      opt('Tags', 'cam-show-parts', 'Tags coloured by part'), opt('Parts', 'cam-show-bodies', 'Estimated parts as wireframes'), opt('Follow', 'cam-arm-ghost', 'Draw the arm at the tracked pose (while tracking)'),
      opt('Screws', 'cam-show-screws', 'Screw heads found near the gripper (cyan) and the model screws (live tracking)'), cleanButton());
    const op = document.createElement('input'); op.type = 'range'; op.min = 0; op.max = 100; op.value = 50; op.title = 'Real frame opacity'; op.className = 'cam-op';
    op.oninput = e => { e.stopPropagation(); $('cam-opacity').value = op.value; $('cam-opacity').dispatchEvent(new Event('input')); };
    op.onclick = e => e.stopPropagation();
    const tr = document.createElement('button'); tr.textContent = 'Track'; tr.title = 'Track the arm in this frame'; tr.onclick = e => { e.stopPropagation(); trackArm(); };
    const fsb = document.createElement('button'); fsb.textContent = '⛶'; fsb.title = 'Full screen (Esc to leave)';
    fsb.onclick = e => { e.stopPropagation(); fullscreen(stage); };
    bar.append(op, tr, fsb);
    stage.append(bar);
    states[c.id] = { cam: c, stage, img, sim, simG: sim.getContext('2d'), overlay, g: overlay.getContext('2d'), cal: ctx.cals[c.id],
      points: c.id === 'oak' ? savedPairs() : [], source: c.id === 'oak' ? 'saved' : 'live', markerQuads: {}, detections: [], tags: {}, videoTimer: null, frames: 0 };
  });
  let S = states[CAMERAS[0].id];
  $('cam-select').innerHTML = CAMERAS.map(c => `<option value="${c.id}">${c.name}</option>`).join('');
  const apiUrl = (st, file) => `${ctx.linkBase()}${st.cam.api}/${file}`;

  let picking = null;                    // marker id waiting for a click
  // the camera views render when the Camera tab is open or when they fill the main view ("Cameras")
  const visible = () => !$('tab-camera').hidden || !$('cam-main').hidden;

  // ---- frame sources (all read only), per camera
  //   saved  the OAK's frame from 2026-10-12 in the repo
  //   live   one snapshot per "Refresh" (GET <api>/snapshot.jpg)
  //   video  the camera's MJPEG stream (GET <api>/stream.mjpg) for live tracking; closed when you pick
  //          another source, hide the page, or after VIDEO_LIMIT_MIN minutes
  //   file   an image from disk
  const VIDEO_LIMIT_MIN = 10;
  const isVideo = st => st.img.src.includes('stream.mjpg');
  // "Clean": all marks off in the camera views (the frame and the simulated view stay); again: back as they were
  const MARK_IDS = ['cam-show-grid', 'cam-show-marks', 'cam-show-parts', 'cam-show-bodies', 'cam-show-screws'];
  let cleanSaved = null;
  function cleanButton() {
    const b = document.createElement('button'); b.textContent = 'Clean'; b.title = 'Hide all marks in the camera views (click again to bring them back)';
    b.onclick = e => {
      e.stopPropagation();
      if (cleanSaved) { MARK_IDS.forEach(id => { $(id).checked = cleanSaved[id]; }); cleanSaved = null; }
      else { cleanSaved = Object.fromEntries(MARK_IDS.map(id => [id, $(id).checked])); MARK_IDS.forEach(id => { $(id).checked = false; }); }
      for (const c of document.querySelectorAll('.cam-bar button.clean')) c.classList.toggle('on', !!cleanSaved);
      syncBars(); draw();
    };
    b.className = 'clean';
    return b;
  }
  const stopVideo = (reason, st = S) => {
    if (!st.videoTimer && !isVideo(st)) return;
    clearTimeout(st.videoTimer); st.videoTimer = null;
    st.img.removeAttribute('src');                      // closes the stream (and the proxy's request to the Pi)
    if (!Object.values(states).some(isVideo)) stopTracking();
    if (reason) { ctx.log(`${st.cam.name}: video stopped (${reason})`); if (st === S) $('cam-status').textContent = `Video stopped: ${reason}`; }
  };
  const loadSource = () => {
    stopVideo(S.source === 'video' ? 'source changed' : '');
    S.source = $('cam-source').value;
    $('cam-refresh').disabled = !(S.source === 'live' || S.source === 'video');
    $('cam-refresh').textContent = S.source === 'video' ? 'Restart video' : 'Refresh snapshots';
    if (S.source === 'saved') {
      if (S.cam.id === 'oak') { S.img.src = 'ref/camera-2026-10-12.jpg'; S.points = savedPairs(); }
      else { ctx.toast('There is no saved frame for this camera'); }
    } else if (S.source === 'live') { S.points = []; refreshLive(); }
    else if (S.source === 'video') { S.points = []; startVideo(); }
    else if (S.source === 'file') { S.points = []; $('cam-file').click(); }
    draw();
  };
  const startVideo = async (st = S) => {
    await readStatus(st);
    st.img.src = `${apiUrl(st, 'stream.mjpg')}?t=${Date.now()}`;
    // the video's load event is not reliable for MJPEG; calibrate from a frame shortly after it starts
    if ($('cam-auto-on').checked) setTimeout(() => { if (st.source === 'video') autoCalibrate({ quiet: true, announce: true, st }); }, 1500);
    ctx.log(`${st.cam.name}: video stream opened (GET only, stops after ${VIDEO_LIMIT_MIN} min)`);
    clearTimeout(st.videoTimer);
    st.videoTimer = setTimeout(() => stopVideo(`${VIDEO_LIMIT_MIN} min limit`, st), VIDEO_LIMIT_MIN * 60000);
  };
  document.addEventListener('visibilitychange', () => { if (document.hidden) for (const st of Object.values(states)) if (st.source === 'video') stopVideo('page hidden', st); });
  // Two GETs per refresh: the camera's status and one frame. Reading starts nothing and sends nothing.
  const refreshLive = async (st = S) => {
    if (st.source === 'video') return startVideo(st);
    if (st.source !== 'live') return;
    await readStatus(st);
    st.img.src = `${apiUrl(st, 'snapshot.jpg')}?t=${Date.now()}`;
    ctx.log(`${st.cam.name}: status and one snapshot read (GET only)`);
  };
  async function readStatus(st = S) {
    const out = text => { if (st === S) $('cam-status').textContent = text; };
    try {
      const r = await fetch(apiUrl(st, 'status'), { cache: 'no-store' });
      if (!r.ok) throw new Error(`HTTP ${r.status}. For the real cameras start the page server with --camera http://10.102.4.103:8765 (npm run lab)`);
      const s = await r.json();
      if (s.source === 'mock') s.source = 'mock camera (the saved frame; for the real one: npm run lab)';
      const f = s.latest_frame || {};
      out(s.enabled === false ? 'Camera disabled on the controller'
        : `${s.available ? 'Camera active' : 'Camera not delivering frames'} · ${s.source || ''} · ${(f.delivered_fps ?? 0).toFixed(1)} fps · ` +
          `frame ${Math.round(s.frame_age_ms ?? 0)} ms old · exposure ${f.time_us ?? '?'} µs, ISO ${f.iso ?? '?'}, focus ${f.lens_position ?? '?'}` +
          (f.captured_at ? ` · ${new Date(f.captured_at).toLocaleTimeString()}` : ''));
    } catch (e) { out(`Camera status unavailable: ${e.message}`); }
  }
  $('cam-source').onchange = loadSource;
  // "Refresh snapshot" reads every camera, not only the selected one (one status + one frame GET each);
  // a camera with its video running keeps it. "Restart video" restarts only the selected camera's stream.
  $('cam-refresh').onclick = () => {
    if (S.source === 'video') return startVideo(S);
    for (const st of Object.values(states)) {
      if (st.source === 'video' && isVideo(st)) continue;
      st.source = 'live'; st.points = [];
      refreshLive(st);
    }
  };
  $('cam-file').onchange = e => { const f = e.target.files[0]; if (f) S.img.src = URL.createObjectURL(f); };
  for (const st of Object.values(states)) {
    st.img.onerror = () => { if (st.img.getAttribute('src')) ctx.toast(`${st.cam.name}: no frame (live needs the page served by server.mjs --camera … or --controller …)`); };
    // A new real frame (snapshot or image file): find the markers and refit that camera right away,
    // so the overlay and the camera in the 3D model follow a camera that has been moved.
    st.img.onload = () => {
      st.frames++;
      if (st.source !== 'video' && $('cam-auto-on').checked && !(live && st.liveCal)) { if (live) st.liveCal = true; setTimeout(() => autoCalibrate({ quiet: true, announce: true, st }), 50); }
    };
  }
  $('cam-opacity').oninput = () => { for (const st of Object.values(states)) st.img.style.opacity = $('cam-opacity').value / 100; };
  for (const id of ['cam-show-sim', 'cam-show-marks', 'cam-show-grid']) $(id).onchange = () => { for (const st of Object.values(states)) st.sim.style.visibility = $('cam-show-sim').checked ? 'visible' : 'hidden'; draw(); };

  // ---- switching the camera shown
  function select(id) {
    S = states[id];
    for (const st of Object.values(states)) st.stage.classList.toggle('sel', st === S);
    $('cam-select').value = id;
    $('cam-title').textContent = S.cam.name;
    $('cam-source').value = S.source;
    $('cam-refresh').disabled = !(S.source === 'live' || S.source === 'video');
    $('cam-refresh').textContent = S.source === 'video' ? 'Restart video' : 'Refresh snapshots';
    $('cam-status').textContent = S.frames ? '' : 'No frame yet for this camera: pick a source.';
    picking = null; $('cam-pick-hint').textContent = '';
    showDetections(0);
    update();
  }
  $('cam-select').onchange = () => select($('cam-select').value);

  // ---- picking marker centres on the frame
  const nextPick = () => {
    const have = new Set(S.points.filter(p => p.k === undefined).map(p => p.id));
    picking = [0, 1, 2, 3, 4, 5].find(id => !have.has(id)) ?? null;
    $('cam-pick-hint').textContent = picking === null ? 'All six marker centres set. Press Solve.' : `Click the centre of marker ID ${picking} (highlighted in green).`;
    draw();
  };
  $('cam-pick').onclick = () => { S.points = S.points.filter(p => p.k === undefined); nextPick(); };
  $('cam-skip').onclick = () => { if (picking !== null) { S.points.push({ id: picking, skipped: true }); nextPick(); } };
  $('cam-clear').onclick = () => { S.points = []; picking = null; $('cam-pick-hint').textContent = ''; draw(); };
  $('cam-use-saved').onclick = () => { if (S.cam.id !== 'oak') { ctx.toast('The saved corners are from the OAK'); return; } $('cam-source').value = 'saved'; loadSource(); };
  // a click on a camera selects it for the controls below; while picking, it sets a marker centre
  for (const st of Object.values(states)) st.overlay.addEventListener('click', e => {
    if (st !== S) { select(st.cam.id); return; }
    if (picking === null) return;
    const r = st.overlay.getBoundingClientRect();
    const uv = [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H];
    const [X, Y] = markerCentre(picking);
    S.points.push({ id: picking, X: [X, Y, 0], uv });
    nextPick();
  });

  // ---- solve and apply
  const usable = (st = S) => st.points.filter(p => p.X && p.uv);
  const apply = (st, cal) => { st.cal = cal; ctx.onApply(cal, st.cam.id); };
  $('cam-solve').onclick = () => {
    const p = usable();
    try {
      const cal = calibrate(p.map(q => q.X), p.map(q => q.uv), { fixPrincipal: p.length < 10 });
      apply(S, cal);
      ctx.log(`${S.cam.name} solved from ${p.length} points: f ${cal.f.toFixed(0)} px, ${cal.rms.toFixed(2)} px rms`, 'ok');
      update();
    } catch (e) { ctx.toast(`Calibration failed: ${e.message}`); }
  };
  $('cam-reset').onclick = () => {
    const c = S.cam;
    apply(S, c.id === 'oak' ? defaultCalibration() : calFromLookAt(c.posMm, c.lookAtMm, c.fPx, c.w, c.h)); update();
  };
  $('cam-copy').onclick = async () => {
    const cal = S.cal;
    const j = JSON.stringify({ camera: S.cam.id, f_px: +cal.f.toFixed(2), cx: +cal.cx.toFixed(2), cy: +cal.cy.toFixed(2), R: cal.R.map(r => r.map(v => +v.toFixed(6))), t_mm: cal.t.map(v => +v.toFixed(2)), centre_mm: cal.centre.map(v => +v.toFixed(1)), rms_px: +(cal.rms || 0).toFixed(3) }, null, 1);
    try { await navigator.clipboard.writeText(j); ctx.toast('Camera parameters copied'); } catch { console.log(j); }
  };

  // ---- grabbing a camera's current frame (works for a still image and for the MJPEG video alike)
  const grabCanvas = document.createElement('canvas');
  function grabFrame(st = S) {
    const w = st.img.naturalWidth, h = st.img.naturalHeight;
    if (!w || !h || !st.img.getAttribute('src')) return null;
    grabCanvas.width = w; grabCanvas.height = h;
    const g2 = grabCanvas.getContext('2d', { willReadFrequently: true });
    g2.drawImage(st.img, 0, 0);
    try { return g2.getImageData(0, 0, w, h); } catch { return null; }   // cross-origin frames cannot be read
  }

  // ---- trackers: functions run on every grabbed frame of the camera shown while tracking is on;
  // multi-camera trackers (the arm tags) run once per round over all cameras.
  const trackers = new Map();
  const addTracker = (name, fn) => { trackers.set(name, fn); };
  let trackTimer = null, trackFrames = 0, trackMs = 0, live = false;   // live: startLive() below
  // rounds as a chain: the next one starts the set time after the last one ended, so slow rounds never pile up
  function scheduleTrack() { clearTimeout(trackTimer); trackTimer = setTimeout(() => { trackOnce(); if (trackTimer) scheduleTrack(); }, Number($('cam-track-rate').value)); }
  function stopTracking() { clearTimeout(trackTimer); trackTimer = null; if ($('cam-track')) $('cam-track').checked = false; if (!live) ctx.clearArmTrack?.(); }
  function trackOnce() {
    const img = grabFrame();
    const t0 = performance.now();
    if (img) for (const [name, fn] of trackers) { try { fn(img, S.cal, Date.now()); } catch (e) { console.warn(name, e); } }
    if ($('cam-track-arm').checked) trackArm({ quiet: true });
    trackMs = performance.now() - t0; trackFrames++;
    $('cam-track-stats').textContent = `tracking: ${trackFrames} rounds, ${trackMs.toFixed(0)} ms per round`;
  }
  $('cam-track').onchange = () => {
    if (!$('cam-track').checked) return stopTracking();
    trackFrames = 0;
    scheduleTrack();
  };
  $('cam-track-rate').onchange = () => { if (trackTimer) scheduleTrack(); };

  // ---- live tracking: every few seconds one snapshot from each camera (GET only); each camera's frame is decoded
  // and searched for tags in its own Web Worker (separate CPU cores, the page never waits), the joint fit runs in
  // one of them, and the model follows. Ends after LIVE_LIMIT_MIN, when the page is hidden, or with stopLive().
  const LIVE_LIMIT_MIN = 10;
  let workers = null, liveSeq = 0, liveTimer = null, liveUntil = 0, liveGap = null;
  // a calibration as plain data for the workers; the camera centre (table mm) is needed by the cube search
  const plainCal = c => ({ R: c.R, t: c.t, f: c.f, cx: c.cx, cy: c.cy, rms: c.rms,
    centre: c.centre || [0, 1, 2].map(i => -(c.R[0][i] * c.t[0] + c.R[1][i] * c.t[1] + c.R[2][i] * c.t[2])) });
  const tagLayout = () => ARM_TAGS.map(({ key, centre, normal, up, size }) => ({ key, centre, normal, up, size }));
  const liveRate = () => Number($('live-rate')?.value || 500);
  function ask(worker, msg) {
    return new Promise(res => {
      const on = ({ data }) => { if (data.seq !== msg.seq) return; worker.removeEventListener('message', on); res(data); };
      worker.addEventListener('message', on); worker.postMessage(msg);
    });
  }
  async function liveRound() {
    const seq = ++liveSeq, t0 = performance.now(), list = Object.values(states);
    const base = { seq, layout: tagLayout(), d: MAT.sToEdge };
    const start = ctx.trackStart?.() ?? ctx.armAngles(), quick = !!ctx.trackStart?.() && lastTagViews >= 2;
    // all cameras at once, one worker each
    const replies = await Promise.all(list.map((st, i) => ask(workers[i], { ...base, type: 'tags', start, quick, w: W, h: H, cubes: true,
      cam: { id: st.cam.id, url: new URL(`${apiUrl(st, 'snapshot.jpg')}?t=${Date.now()}`, location.href).href, cal: plainCal(st.cal) } })));
    if (!live || seq !== liveSeq) return;
    // the frames that were tracked are the ones shown
    list.forEach((st, i) => {
      const r = replies[i];
      st.tags = r.tags;
      if (r.blob) { if (st.liveUrl) URL.revokeObjectURL(st.liveUrl); st.liveUrl = URL.createObjectURL(r.blob); st.img.src = st.liveUrl; }
    });
    const views = list.filter(st => Object.keys(st.tags).length).map(st => ({ st, cal: st.cal, tags: st.tags }));
    const s = await ask(workers[0], { ...base, type: 'solve', views: views.map(v => ({ cal: plainCal(v.cal), tags: v.tags })),
      commanded: ctx.armAngles(), last: ctx.trackStart?.(), useCorners: $('cam-arm-corners').checked });
    if (!live || seq !== liveSeq) return;
    showArm(views, s.solved, { quiet: true });
    // the cubes in 3D at their real height, the blobs on the tracked arm left out
    list.forEach((st, i) => { st.detections = replies[i].cubes || []; });
    const arm = s.solved.result?.angles ?? ctx.trackStart?.();
    // (in a worker: the joint fit of several cubes can take half a second)
    const cr = await ask(workers[0], { ...base, type: 'cubes', views: list.map(st => ({ cal: plainCal(st.cal), detections: st.detections })), arm: arm ? { angles: arm } : null });
    if (!live || seq !== liveSeq) return;
    cubes3d = cr.cubes; cubes3d.onArm = cr.onArm;
    ctx.onLiveCubes?.(cubes3d);
    showDetections(0);
    // the jaw opening: the model's fingers for several gaps against each frame (in the workers), smoothed over rounds
    if (s.solved.result) {
      const gaps = Array.from({ length: 19 }, (_, i) => i * 0.25), holding = cubes3d.some(c => c.where === 'gripper');
      const per = await Promise.all(list.map((st, i) => ask(workers[i], { ...base, type: 'gap', cal: plainCal(st.cal), angles: s.solved.result.angles, gaps, w: W, h: H })));
      if (!live || seq !== liveSeq) return;
      const g = pickGap(gaps, per, { holding });
      if (g.cameras) liveGap = liveGap == null ? g.gap : 0.5 * liveGap + 0.5 * g.gap;   // smoothed over rounds
      else if (g.gap != null) liveGap = g.gap;
      // for the Screws view: the screw heads found, and the model's screws at the current gap (with that camera's best shift)
      list.forEach((st, i) => {
        const sc = per[i].screws; if (!sc || liveGap == null) { st.screws = null; return; }
        const k = gaps.reduce((b, g, j) => Math.abs(g - liveGap) < Math.abs(gaps[b] - liveGap) ? j : b, 0), sh = sc.shifts[k], hand = fk(s.solved.result.angles).hand;
        const P = p => { const q = projectBoard(st.cal, point(hand, p)); return [q[0] + sh[0], q[1] + sh[1]]; }, m = gripperScrews(liveGap, sc.side);
        st.screws = { spots: sc.spots, fixed: m.fixed.map(P), moving: m.moving.map(P) };
      });
      ctx.onGap?.({ gap: liveGap, source: g.cameras ? `finger bars in ${g.cameras} camera${g.cameras > 1 ? 's' : ''}` : g.gap != null ? 'cube width' : liveGap != null ? 'last estimate' : 'unknown', holding });
    }
    trackFrames++;
    const errs = replies.filter(r => r.error).map(r => `${r.id}: ${r.error}`);
    const stats = `round ${trackFrames}: ${(performance.now() - t0).toFixed(0)} ms (frame + tags ${replies.map(r => `${r.id} ${(r.ms.frame ?? 0) + (r.ms.tags ?? 0)}`).join(' · ')} ms in parallel, cubes ${replies.map(r => r.ms.cubes ?? 0).join(' · ')} ms, fit ${s.ms.fit} ms, ${quick ? 'near the arm' : 'whole frames'})${errs.length ? ' · ' + errs.join(', ') : ''}`;
    $('cam-track-stats').textContent = `live ${stats}`;
    ctx.onLiveStats?.(stats);
  }
  function liveLoop() {
    if (!live) return;
    if (Date.now() > liveUntil) { stopLive(`${LIVE_LIMIT_MIN} min limit`); return; }
    const t0 = Date.now();
    liveRound().catch(e => ctx.log(`live tracking: ${e.message}`, 'err'))
      .finally(() => { if (live) liveTimer = setTimeout(liveLoop, Math.max(100, liveRate() - (Date.now() - t0))); });
  }
  async function startLive() {
    await activate();
    workers ??= Object.values(states).map(() => new Worker(new URL('./trackworker.js', import.meta.url), { type: 'module' }));
    for (const st of Object.values(states)) { if (st.source === 'video') stopVideo('', st); st.source = 'live'; st.points = []; st.liveCal = false; }
    $('cam-source').value = 'live'; $('cam-refresh').disabled = false; $('cam-refresh').textContent = 'Refresh snapshots';
    $('cam-arm-ghost').checked = true; $('cam-arm-ghost').dispatchEvent(new Event('change'));
    trackFrames = 0; lastTagViews = 0; liveGap = null; live = true; liveUntil = Date.now() + LIVE_LIMIT_MIN * 60000;
    ctx.onLive?.(true);
    ctx.log(`live tracking on: one snapshot per camera every ${liveRate() / 1000} s (read only), tags found in ${workers.length} worker threads`, 'ok');
    liveLoop();
  }
  function stopLive(reason = '') {
    clearTimeout(liveTimer); liveTimer = null; liveSeq++;
    if (!live) return;
    live = false; ctx.clearArmTrack?.(); ctx.onLive?.(false);
    if (reason) ctx.log(`live tracking stopped (${reason})`);
  }
  document.addEventListener('visibilitychange', () => { if (document.hidden && live) stopLive('page hidden'); });

  // ---- recalibration from captures (docs/recalibration.md, js/recalib.js): docs/images/calibration/* through the server
  let recal = null;
  async function imageData(url) {
    const im = new Image(); im.src = url; await im.decode();
    const c = document.createElement('canvas'); c.width = im.naturalWidth; c.height = im.naturalHeight;
    const g = c.getContext('2d', { willReadFrequently: true }); g.drawImage(im, 0, 0);
    return g.getImageData(0, 0, c.width, c.height);
  }
  /** A camera calibration from the mat markers in one frame (as "Auto-calibrate"), or null with too few. */
  function calFromFrame(img) {
    const { markers } = detectMarkers(img), sx = W / img.width, sy = H / img.height, X = [], uv = [];
    for (const [id, q] of Object.entries(markers)) if (MARKER_CORNERS[id]) q.forEach(([x, y], k) => { X.push([...MARKER_CORNERS[id][k], 0]); uv.push([x * sx, y * sy]); });
    if (X.length < 8) return null;
    const cal = calibrate(X, uv, { fixPrincipal: X.length < 12 });
    return cal.rms > 4 ? null : cal;
  }
  async function runRecalibration() {
    const out = $('recal-out'), tick = () => new Promise(r => setTimeout(r, 0));
    $('recal-use').disabled = $('recal-copy').disabled = true;
    out.textContent = 'Loading the captures…';
    let list;
    try { list = await (await fetch('/calibration/index.json', { cache: 'no-store' })).json(); } catch (e) { out.textContent = `No capture list (${e.message}): the page must be served by server.mjs`; return; }
    const captures = [], notes = [];
    for (const c of list) {
      if (!c.info?.target) { notes.push(`${c.name}: no target in info.json, left out`); continue; }
      const views = [];
      for (const file of ['oak.jpg', 'webcam.jpg'].filter(f => c.files.includes(f))) {
        const img = await imageData(`/calibration/${c.name}/${file}`);
        const cal = calFromFrame(img);
        if (!cal) { notes.push(`${c.name} ${file}: too few mat markers`); continue; }
        // the tags: searched (also enlarged) around and above the target spot, and in the whole frame
        const [u, v] = project(cal, c.info.target.at), sx = W / img.width, sy = H / img.height;
        const roi = [(u - 0.25 * W) / sx, (v - 0.5 * H) / sy, (u + 0.25 * W) / sx, (v + 0.1 * H) / sy];
        const found = detectArmTags(img, undefined, { roi });
        views.push({ cal, file, tags: Object.fromEntries(Object.entries(found).map(([id, t]) => [id, { corners: t.corners.map(([x, y]) => [x * sx, y * sy]) }])) });
      }
      captures.push({ name: c.name, views, target: c.info.target });
      out.textContent = `Loaded ${captures.length} capture(s)…`; await tick();
    }
    if (!captures.length) { out.innerHTML = `No capture with a target.<br>${notes.join('<br>')}`; return; }
    const fit = [...document.querySelectorAll('#recal-params input:checked')].map(i => i.value);
    out.textContent = `Fitting ${fit.length} value(s) and the joint angles of ${captures.length} captures…`; await tick();
    const t0 = performance.now();
    recal = recalibrate(captures, { fit });
    const mean = l => { const v = l.map(x => x.gripMm).filter(x => x != null); return v.length ? (v.reduce((s, x) => s + x, 0) / v.length).toFixed(1) : '–'; };
    out.innerHTML = `<table class="params"><tr><th>Capture</th><th>Tags (camera: ids)</th><th>Spot</th><th>Grip error before</th><th>after</th><th>Tag fit after</th></tr>` +
      captures.map((c, i) => `<tr><td>${c.name}</td><td>${c.views.map(v => `${v.file.split('.')[0]}: ${Object.keys(v.tags).join(' ') || '–'}`).join(' · ')}</td><td>${c.target.spot ?? c.target.at.join(' / ')}, level ${c.target.level ?? 0}</td>` +
        `<td>${recal.before[i]?.gripMm ?? '–'} mm</td><td><b>${recal.after[i]?.gripMm ?? '–'} mm</b></td><td>${recal.after[i]?.tagPx ?? '–'} px</td></tr>`).join('') +
      `<tr><th colspan="3">mean</th><th>${mean(recal.before)} mm</th><th>${mean(recal.after)} mm</th><th></th></tr></table>` +
      `<table class="params"><tr><th>Value</th><th>now</th><th>fitted</th><th>change</th></tr>` +
      RECAL_PARAMS.map(p => `<tr><td title="${p.what}">${p.key}${fit.includes(p.key) ? '' : ' (kept)'}</td><td>${(recal.values[p.key] - recal.change[p.key]).toFixed(2)} ${p.unit}</td><td>${recal.values[p.key].toFixed(2)} ${p.unit}</td><td>${recal.change[p.key] >= 0 ? '+' : ''}${recal.change[p.key].toFixed(2)}</td></tr>`).join('') + '</table>' +
      `<div>${recal.iterations} iterations, ${(performance.now() - t0).toFixed(0)} ms. Use the values only if most captures get better and the changes are plausible (a few mm).</div>` +
      (notes.length ? `<div>${notes.join('<br>')}</div>` : '');
    $('recal-use').disabled = $('recal-copy').disabled = false;
    ctx.log(`recalibration from ${captures.length} captures: grip error ${mean(recal.before)} -> ${mean(recal.after)} mm (mean)`, 'ok');
  }
  $('recal-run').onclick = () => runRecalibration().catch(e => { $('recal-out').textContent = `Recalibration failed: ${e.message}`; });
  $('recal-use').onclick = () => {
    if (!recal) return;
    applyRecalibration(recal.values);
    ctx.onRecalibrated?.();
    ctx.log('recalibrated values in use for this page (tracking, grip length, mat offset); copy them into config.js to keep them', 'ok');
    update();
  };
  $('recal-copy').onclick = () => {
    if (!recal) return;
    const text = `// docs/recalibration.md, ${new Date().toISOString().slice(0, 10)}: fitted from ${recal.after.length} captures\n` +
      `// ARM.G_BLOCK = ${recal.values.grip};  MAT.sToEdge = ${recal.values.d};\n// ARM_TAGS centres:\n` +
      layoutFor(recal.values).map(l => `//   ${l.key} (${l.id}): centre: [${l.centre.join(', ')}]`).join('\n');
    try { navigator.clipboard.writeText(text); } catch { /* shown below */ }
    $('recal-out').insertAdjacentHTML('beforeend', `<pre class="api">${text}</pre>`);
  };

  // ---- automatic calibration from the ArUco markers in a camera's current frame
  function autoCalibrate({ quiet = false, announce = false, st = S } = {}) {
    const img = grabFrame(st);
    if (!img) { if (!quiet) ctx.toast('No readable frame'); return false; }
    const t0 = performance.now();
    const { markers } = detectMarkers(img);
    const ms = performance.now() - t0;
    const sx = W / img.width, sy = H / img.height;                      // calibration uses the 1280 x 720 grid
    st.markerQuads = Object.fromEntries(Object.entries(markers).map(([id, q]) => [id, q.map(([x, y]) => [x * sx, y * sy])]));
    const ids = Object.keys(st.markerQuads).map(Number);
    const hint = text => { if (st === S) $('cam-pick-hint').textContent = text; };
    if (ids.length < 2) {
      if (!quiet) ctx.toast(`Only ${ids.length} marker(s) found; at least 2 are needed (4 or more spread over the mat are better)`);
      hint(`Markers found: ${ids.join(', ') || 'none'} (${ms.toFixed(0)} ms). Too few to calibrate.`);
      draw(); return false;
    }
    st.points = [];
    for (const id of ids) MARKER_CORNERS[id].forEach((p, k) => st.points.push({ id, k, X: [p[0], p[1], 0], uv: st.markerQuads[id][k] }));
    try {
      const next = calibrate(st.points.map(q => q.X), st.points.map(q => q.uv), { fixPrincipal: ids.length < 3 });
      if (next.rms > 4) throw new Error(`fit too poor (${next.rms.toFixed(1)} px)`);
      apply(st, next);
      hint(`Markers found: ${ids.sort().join(', ')} in ${ms.toFixed(0)} ms. Camera fitted to ${st.points.length} corners, ${next.rms.toFixed(2)} px rms.`);
      if (!quiet || announce) ctx.log(`${st.cam.name} calibrated from markers ${ids.join(', ')}: f ${next.f.toFixed(0)} px, ${next.rms.toFixed(2)} px rms, camera at ${next.centre.map(v => v.toFixed(0)).join(' / ')} mm (mat frame); 3D model updated`, 'ok');
      update();
      return true;
    } catch (e) { if (!quiet) ctx.toast(`Calibration failed: ${e.message}`); draw(); return false; }
  }
  $('cam-auto').onclick = () => autoCalibrate();
  $('cam-auto-all').onclick = () => {
    const done = Object.values(states).filter(st => autoCalibrate({ quiet: true, announce: true, st }));
    ctx.toast(done.length ? `Calibrated: ${done.map(st => st.cam.name).join(', ')}` : 'No camera has a frame with enough markers');
  };
  let lastKeep = 0;
  addTracker('camera', () => {
    if (!$('cam-keep').checked || Date.now() - lastKeep < 5000) return null;
    lastKeep = Date.now();
    for (const st of Object.values(states)) autoCalibrate({ quiet: true, st });
    return true;
  });

  // ---- cube recognition: in every camera with a frame; cubes seen by two or more cameras are located in
  // 3D by intersecting the viewing rays (locate3d.js), so stacked and held cubes get their real height.
  // With one camera only, the cubes are assumed to lie on the mat (detect.js).
  let cubes3d = [];
  function findCubes() {
    const t0 = performance.now(), views = [];
    for (const st of Object.values(states)) {
      const img = grabFrame(st);
      st.detections = img ? detectCubes(img, st.cal) : [];
      if (img) views.push({ cal: st.cal, detections: st.detections });
    }
    cubes3d = locateCubes(views);
    return performance.now() - t0;
  }
  addTracker('cubes', () => {
    findCubes();
    showDetections(0);
    if ($('cam-track-sim').checked) {
      if (cubes3d.length) ctx.trackCubes3d(cubes3d); else ctx.trackDetections(S.detections.filter(d => d.ok));
    }
    return cubes3d.length ? cubes3d : S.detections;
  });
  $('cam-detect').onclick = () => {
    if (!Object.values(states).some(st => grabFrame(st))) { ctx.toast(S.img.naturalWidth ? 'This frame cannot be read (cross-origin)' : 'No frame loaded'); return; }
    showDetections(findCubes());
  };
  const resting = c => Math.abs(c.X[2] - (c.level * CUBE * 10 + CUBE * 5)) < 6;   // within 6 mm of a stack level
  function showDetections(ms) {
    const det = S.detections;
    if (cubes3d.length) {
      $('cam-detections').innerHTML = `<div><b>In 3D, from ${Math.max(...cubes3d.map(c => c.cameras))} cameras:</b></div>` + cubes3d.map(c =>
        `<div><b style="color:${COLOR_CSS[c.colour]}">${c.colour}</b> board x ${c.board[0].toFixed(1)} · y ${c.board[1].toFixed(1)} · z ${c.board[2].toFixed(1)} cm → ${resting(c) ? `level ${c.level}` : '<span class="warn">between levels: in the gripper or tilted?</span>'} · rays meet within ${c.gap.toFixed(1)} mm</div>`).join('') +
        (ms ? `<div>${ms.toFixed(0)} ms</div>` : '');
    } else {
      $('cam-detections').innerHTML = det.length
        ? `<div><b>${S.cam.name} alone (cubes assumed on the mat):</b></div>` + det.map(d => {
            const [x, y] = tableToBoard([d.X, d.Y, 0]);
            return `<div><b style="color:${COLOR_CSS[d.colour]}">${d.colour}</b> mat X ${d.X.toFixed(0)} · Y ${d.Y.toFixed(0)} mm → board x ${x.toFixed(1)} · y ${y.toFixed(1)} cm, yaw ${d.yaw.toFixed(0)}° ${d.ok ? '' : `<span class="warn">size ${(d.areaRatio * 100).toFixed(0)} % of a cube on the mat: stacked, in the gripper or partly hidden?</span>`}</div>`;
          }).join('') + (ms ? `<div>${ms.toFixed(0)} ms</div>` : '')
        : 'No cubes found.';
    }
    $('cam-detect-use').disabled = !(cubes3d.some(resting) || det.some(d => d.ok));
    draw();
  }
  $('cam-detect-use').onclick = () => {
    if (cubes3d.length) ctx.useCubes3d(cubes3d.filter(resting));
    else ctx.useDetections(S.detections.filter(d => d.ok));
  };

  // ---- arm tracking: the AprilTags on the arm in every camera with a frame, combined (armtrack.js)
  let lastArm = null, parts = {}, lastViews = [], pair = null;   // last joint fit, rigid part fits and the views they came from
  let lastTagViews = 0;   // tag views found in the last round; live rounds search only near the arm while that is >= 2
  // "Track the arm" (and Track continuously): the frames shown now, in the page
  function trackArm({ quiet = false } = {}) {
    const start = ctx.trackStart?.() ?? ctx.armAngles();
    const views = [];
    for (const st of Object.values(states)) {
      const img = grabFrame(st);
      st.tags = img ? findTags(img, st.cal, start, { w: W, h: H }) : {};
      if (Object.keys(st.tags).length) views.push({ st, cal: st.cal, tags: st.tags });
    }
    const solved = solveArm(views, { commanded: ctx.armAngles(), last: ctx.trackStart?.(), useCorners: $('cam-arm-corners').checked });
    return showArm(views, solved, { quiet });
  }
  /** Shows a solved round: the model follows it, the camera overlays and the Arm tracking box. */
  function showArm(views, { tri, parts: p, pair: pr, result, notes }, { quiet = false } = {}) {
    parts = p; pair = pr;
    const lines = Object.values(states).map(st => `${st.cam.name}: ${st.img.getAttribute('src') ? (Object.keys(st.tags).length ? `tags ${Object.keys(st.tags).join(', ')}` : 'no tags') : 'no frame'}`).concat(notes);
    lastArm = result;
    lastTagViews = views.reduce((n, x) => n + Object.keys(x.tags).length, 0);
    lastViews = views;
    // how far the arm model (joint chain) is from each part as its tags alone put it
    const fit = result ? fk(result.angles) : null;
    const partLines = Object.values(parts).map(p => {
      let dev = '';
      if (fit) { const q = fit[p.link]; let tr = 0; for (let i = 0; i < 9; i++) tr += q.R[i] * p.R[i];
        dev = ` · arm fit ${Math.hypot(...q.p.map((v, i) => v - p.p[i])).toFixed(1)} cm, ${(Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI).toFixed(1)}° away`; }
      return `<div><b style="color:${partCss(p.link)}">${ARM_PARTS[p.link].name}</b>: ${p.views} tag view${p.views > 1 ? 's' : ''} from ${p.cameras} camera${p.cameras > 1 ? 's' : ''}, ${p.rms.toFixed(1)} px${dev}${Object.entries(p.shifts).filter(([, k]) => k).map(([id, k]) => ` · tag ${id} stuck on turned ${k * 90}°`).join('')}</div>`;
    });
    const camName = st => st.cam.name.split(' ')[0];
    ctx.onArmTrack(result, Object.values(states).map(st => ({ name: camName(st), ids: Object.keys(st.tags || {}) })),
      result ? result.residuals.map(x => ({ id: x.id, cam: camName(views[x.cam].st), err: x.err })) : []);
    const a0 = ctx.armAngles();
    const pairLine = pair ? `<div><b>Wrist + gripper together</b> (6 + 1 numbers, ${pair.views} tag views from ${pair.cameras} camera${pair.cameras > 1 ? 's' : ''}): S5 = ${pair.angle.toFixed(1)}° about the model axis, ${pair.rms.toFixed(1)} px${pair.rms > 3 * Math.max(parts.wrist?.rms || 0, parts.hand?.rms || 0, 1) ? ' — much worse than the parts alone: the tag anchors or the S5 axis are off; record poses and fit the axis' : ''}</div>` : '';
    $('cam-arm-out').innerHTML = legend() + lines.map(l => `<div>${l}</div>`).join('') + partLines.join('') + pairLine + recLine() +
      Object.entries(tri).map(([id, t]) => `<div>tag ${id} from ${views.filter(x => x.tags[id]).length} cameras: x ${t.board[0].toFixed(1)} · y ${t.board[1].toFixed(1)} · z ${t.board[2].toFixed(1)} cm (${t.rms.toFixed(1)} px)</div>`).join('') +
      (result ? `<table class="params"><tr><th></th>${['S1', 'S2', 'S3', 'S4', 'S5'].map(s => `<th>${s}</th>`).join('')}</tr>
        <tr><th>tracked</th>${['s1', 's2', 's3', 's4', 's5'].map(j => `<td>${result.free.includes(j) ? `${result.angles[j].toFixed(1)}°` : '–'}</td>`).join('')}</tr>
        <tr><th>commanded</th>${['s1', 's2', 's3', 's4', 's5'].map(j => `<td>${a0[j].toFixed(1)}°</td>`).join('')}</tr>
        <tr><th>difference</th>${['s1', 's2', 's3', 's4', 's5'].map(j => `<td>${result.free.includes(j) ? `${(result.angles[j] - a0[j]).toFixed(1)}°` : '–'}</td>`).join('')}</tr></table>
        <div>${result.rms.toFixed(1)} px rms over ${result.residuals.length} tag views${result.residuals.length < 4 ? ' (few views: S5 and the joints along the line of sight are weakly determined)' : ''}</div>` : '<div>Not enough tags to fit the arm: each camera needs a frame, and together they must see at least two tag views.</div>');
    if (!quiet && !result) ctx.toast('No arm pose: see the Arm tracking box');
    draw();
    return result;
  }
  $('cam-arm').onclick = () => trackArm();
  // ---- joint axis calibration: record poses (both parts seen), then fit the S5 axis (jointfit.js)
  const records = [];
  const recLine = () => `<div>${records.length} pose${records.length === 1 ? '' : 's'} recorded for the S5 axis${records.length ? ` (S5 commanded: ${records.map(r => r.commanded.toFixed(0)).join(', ')}°)` : ''}.</div>`;
  $('cam-arm-record').onclick = () => {
    trackArm({ quiet: true });
    if (!parts.wrist || !parts.hand) { ctx.toast('Both the wrist and the gripper need a tag in view to record'); return; }
    records.push({ parent: parts.wrist, child: parts.hand, commanded: ctx.armAngles().s5 });
    ctx.log(`pose ${records.length} recorded for the S5 axis (S5 commanded ${ctx.armAngles().s5.toFixed(1)}°)`, 'ok');
    trackArm({ quiet: true });
  };
  $('cam-arm-clear').onclick = () => { records.length = 0; trackArm({ quiet: true }); };
  $('cam-arm-axis').onclick = () => {
    try {
      const r = estimateJointAxis('s5', records);
      for (const t of r.layout) Object.assign(ARM_TAGS.find(x => x.key === t.key), { centre: t.centre, normal: t.normal, up: t.up });
      const text = JSON.stringify(r.layout.map(({ key, id, link, centre, normal, up }) => ({ key, id, link, centre, normal, up })), null, 1);
      try { navigator.clipboard.writeText(text); } catch { /* shown below */ }
      $('cam-arm-out').insertAdjacentHTML('beforeend', `<div><b>S5 axis from ${records.length} poses</b> (spread ${r.spread}°): ${r.model.axisDeg}° and ${r.model.offset} cm from the model axis, circle fit ${r.fit} cm.
        ${r.servo ? `Measured S5 = ${r.servo.slope} × commanded + ${r.servo.offset}° (slope near 1: direction right; the offset is the error of S5's angle at HOME).` : ''}
        Corrected tag layout used now and copied for config.js ARM_TAGS:</div><pre class="api">${text}</pre>`);
      ctx.log(`S5 axis fitted from ${records.length} poses: ${r.model.offset} cm, ${r.model.axisDeg}° from the model; tag layout corrected`, 'ok');
      update();
    } catch (e) { ctx.toast(e.message); }
  };
  // Measures where each seen tag sits on its part, relative to the part's fitted pose, and uses that from
  // now on (this browser session). Copy the numbers into config.js ARM_TAGS to keep them.
  $('cam-arm-learn').onclick = () => {
    if (!Object.keys(parts).length) { ctx.toast('Track the arm first'); return; }
    const out = {};
    for (const p of Object.values(parts)) Object.assign(out, learnTagLayout(p.link, lastViews, p, p.shifts));
    for (const [id, l] of Object.entries(out)) Object.assign(tagById(id), l);
    const text = JSON.stringify(out, null, 1);
    try { navigator.clipboard.writeText(text); } catch { /* shown below */ }
    $('cam-arm-out').insertAdjacentHTML('beforeend', `<div><b>Tag layout measured</b> (used now; copied for config.js ARM_TAGS):</div><pre class="api">${text}</pre>`);
    ctx.log(`tag layout measured for tags ${Object.keys(out).join(', ')}`, 'ok');
  };
  /** Table: tag, where it is, the rigid part, what moves it, which cameras saw it in the last round. */
  function legend() {
    const seen = t => Object.values(states).filter(st => t.id != null && st.tags[t.id]).map(st => st.cam.name.split(' ')[0]).join(', ') || '–';
    return `<table class="params legend"><tr><th>Tag</th><th>On</th><th>Part</th><th>Moved by</th><th>Seen by</th></tr>${ARM_TAGS.map(t =>
      `<tr><td><b style="color:${partCss(t.link)}">${t.id ?? 'reprint'}</b></td><td>${t.on}</td><td style="color:${partCss(t.link)}">${ARM_PARTS[t.link].name}</td><td>${ARM_PARTS[t.link].joints} (last: ${ARM_PARTS[t.link].last})</td><td>${seen(t)}</td></tr>`).join('')}</table>`;
  }
  $('cam-arm-out').innerHTML = legend();
  $('cam-show-parts').onchange = () => draw();
  $('cam-show-screws').onchange = () => draw();

  // ---- drawing (the camera shown)
  /** A collision shape (collision.js) as a wireframe through the projection P. */
  function wire(g, sh, P) {
    const seg = (a, b) => { const p = P(a), q = P(b); g.beginPath(); g.moveTo(...p); g.lineTo(...q); g.stroke(); };
    if (sh.kind === 'box') {
      const R = sh.R, h = sh.h, C = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
        const v = [sx * h[0], sy * h[1], sz * h[2]];
        C.push([0, 1, 2].map(i => sh.c[i] + R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2]));
      }
      for (const [a, b] of [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]]) seg(C[a], C[b]);
    } else if (sh.kind === 'capsule') {
      const d = sh.b.map((v, i) => v - sh.a[i]), L = Math.hypot(...d), u = d.map(v => v / L);
      const t = Math.abs(u[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
      const e1 = [u[1] * t[2] - u[2] * t[1], u[2] * t[0] - u[0] * t[2], u[0] * t[1] - u[1] * t[0]], n1 = Math.hypot(...e1);
      const x = e1.map(v => v / n1), y = [u[1] * x[2] - u[2] * x[1], u[2] * x[0] - u[0] * x[2], u[0] * x[1] - u[1] * x[0]];
      for (const k of [0, 1, 2, 3]) { const a = k * Math.PI / 2, o = x.map((v, i) => sh.r * (Math.cos(a) * v + Math.sin(a) * y[i])); seg(sh.a.map((v, i) => v + o[i]), sh.b.map((v, i) => v + o[i])); }
    }
  }
  function draw() { for (const st of Object.values(states)) drawOne(st); params(); }
  function drawOne(st) {
    const g = st.g;
    const cal = st.cal;
    g.clearRect(0, 0, W, H);
    const P = X => project(cal, X);
    if ($('cam-show-grid').checked) {
      // the mat's 50 mm grid and outline, reprojected with the current camera
      g.lineWidth = 1.2; g.strokeStyle = 'rgba(14,138,119,0.55)';
      for (let X = 0; X <= 360; X += 50) { const a = P([X, -140, 0]), b = P([X, 140, 0]); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); }
      for (let Y = -140; Y <= 140; Y += 50) { const a = P([0, Y, 0]), b = P([360, Y, 0]); g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); }
      // table axes at the origin: X red, Y green, Z blue (100 mm)
      const o = P([0, 0, 0]);
      for (const [v, c] of [[[100, 0, 0], '#e0281e'], [[0, 100, 0], '#1f9d4a'], [[0, 0, 100], '#2f6fd6']]) {
        const e = P(v); g.strokeStyle = c; g.lineWidth = 3; g.beginPath(); g.moveTo(...o); g.lineTo(...e); g.stroke();
      }
    }
    if ($('cam-show-marks').checked) {
      for (const id of [0, 1, 2, 3, 4, 5]) {
        const q = MARKER_CORNERS[id].map(p => P([p[0], p[1], 0]));
        g.lineWidth = (st === S && picking === id) ? 4 : 2; g.strokeStyle = (st === S && picking === id) ? '#19d36b' : 'rgba(255,170,0,0.95)';
        g.beginPath(); q.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke();
        const c = P([...markerCentre(id), 0]);
        g.fillStyle = '#ffb000'; g.font = 'bold 20px system-ui'; g.fillText(`ID ${id}`, c[0] + 22, c[1] - 18);
      }
      // measured points (circles) and their reprojection error (red line, 5x)
      for (const p of usable(st)) {
        const r = P(p.X);
        g.strokeStyle = '#e0281e'; g.lineWidth = 2;
        g.beginPath(); g.moveTo(...p.uv); g.lineTo(p.uv[0] + 5 * (r[0] - p.uv[0]), p.uv[1] + 5 * (r[1] - p.uv[1])); g.stroke();
        g.strokeStyle = '#ffffff'; g.lineWidth = 2; g.beginPath(); g.arc(...p.uv, 6, 0, 2 * Math.PI); g.stroke();
        g.strokeStyle = '#2f6fd6'; g.beginPath(); g.moveTo(r[0] - 6, r[1]); g.lineTo(r[0] + 6, r[1]); g.moveTo(r[0], r[1] - 6); g.lineTo(r[0], r[1] + 6); g.stroke();
      }
      // the arm tags where the model puts them (commanded pose), in the colour of the rigid part they sit on;
      // a dashed line joins the tags of one part (they move as one object)
      const a = ctx.armAngles();
      const showParts = $('cam-show-parts').checked;
      for (const link of Object.keys(ARM_PARTS)) {
        const cs = ARM_TAGS.filter(t => t.link === link).map(t => projectBoard(cal, tagPoints(t, a).centre));
        if (showParts && cs.length > 1) { g.strokeStyle = partCss(link); g.lineWidth = 1.5; g.setLineDash([5, 4]); g.beginPath(); cs.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.stroke(); g.setLineDash([]); }
      }
      for (const t of ARM_TAGS) {
        const tp = tagPoints(t, a), q = tp.corners.map(c => projectBoard(cal, c));
        g.strokeStyle = partCss(t.link); g.lineWidth = 2; g.setLineDash(t.id == null ? [3, 3] : []);
        g.beginPath(); q.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke(); g.setLineDash([]);
        const c = projectBoard(cal, tp.centre); g.fillStyle = partCss(t.link); g.font = 'bold 15px system-ui';
        if (showParts) g.fillText(`${t.id ?? '?'}`, c[0] - 8, c[1] - 14);   // model position: part colour, id only
      }
    }
    // markers as detected in the frame (green), with their first corner marked
    for (const [id, q] of Object.entries(st.markerQuads)) {
      g.strokeStyle = '#19d36b'; g.lineWidth = 2.5; g.beginPath(); q.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke();
      g.fillStyle = '#19d36b'; g.beginPath(); g.arc(...q[0], 4, 0, 2 * Math.PI); g.fill();
      g.font = 'bold 13px system-ui'; g.fillText(`${id}`, q[2][0] + 6, q[2][1] + 16);
    }
    // each rigid part where its tags alone put it: its shapes as a wireframe, in the part's colour
    // screw heads found near the gripper (cyan rings) and the model's screws: fixed (yellow +), on the fingers (orange x)
    if ($('cam-show-screws').checked && st.screws) {
      g.lineWidth = 1.5; g.strokeStyle = '#22d3ee';
      for (const [u, v] of st.screws.spots) { g.beginPath(); g.arc(u, v, 5, 0, 2 * Math.PI); g.stroke(); }
      const mark = ([u, v], c, x) => { g.strokeStyle = c; g.beginPath(); if (x) { g.moveTo(u - 4, v - 4); g.lineTo(u + 4, v + 4); g.moveTo(u + 4, v - 4); g.lineTo(u - 4, v + 4); } else { g.moveTo(u - 5, v); g.lineTo(u + 5, v); g.moveTo(u, v - 5); g.lineTo(u, v + 5); } g.stroke(); };
      st.screws.fixed.forEach(p => mark(p, '#facc15', false)); st.screws.moving.forEach(p => mark(p, '#f97316', true));
    }
    if ($('cam-show-bodies').checked) for (const p of Object.values(parts)) {
      const frames = fk(ctx.armAngles()); frames[p.link] = { R: p.R, p: p.p };
      const ids = p.link === 'wrist' ? [LINK.wrist] : [LINK.hand, LINK.fingerL];
      g.strokeStyle = partCss(p.link); g.lineWidth = 2;
      for (const sh of armShapes(frames, 0).filter(x => ids.includes(x.link))) wire(g, sh, P3 => projectBoard(cal, P3));
      const c = projectBoard(cal, p.p);
      g.fillStyle = partCss(p.link); g.font = 'bold 15px system-ui'; g.fillText(`${p.link === 'wrist' ? 'wrist' : 'gripper'} (${p.rms.toFixed(1)} px)`, c[0] + 16, c[1] + 4);
    }
    // arm tags as detected (cyan) and where the tracked pose puts them (white cross)
    for (const [id, t] of Object.entries(st.tags)) {
      g.strokeStyle = '#19c8e0'; g.lineWidth = 3; g.beginPath(); t.corners.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke();
      const tg = tagById(id);
      g.fillStyle = '#19c8e0'; g.font = 'bold 16px system-ui'; g.fillText(tg ? `${id}` : `${id}?`, t.corners[2][0] + 4, t.corners[2][1] + 14);
      if (lastArm?.points[id]) {
        const c = projectBoard(cal, lastArm.points[id]);
        g.strokeStyle = '#fff'; g.lineWidth = 2; g.beginPath(); g.moveTo(c[0] - 7, c[1]); g.lineTo(c[0] + 7, c[1]); g.moveTo(c[0], c[1] - 7); g.lineTo(c[0], c[1] + 7); g.stroke();
      }
    }
    for (const c of cubes3d) {
      const h = CUBE * 5, a = (c.yaw || 0) * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
      const corner = (sx, sy, sz) => P([c.X[0] + h * (sx * ca - sy * sa), c.X[1] + h * (sx * sa + sy * ca), c.X[2] + sz * h]);
      const q = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
      const lo = q.map(([x, y]) => corner(x, y, -1)), hi = q.map(([x, y]) => corner(x, y, 1));
      g.strokeStyle = COLOR_CSS[c.colour]; g.lineWidth = 2; g.beginPath();
      for (let i = 0; i < 4; i++) { const j = (i + 1) % 4; g.moveTo(...lo[i]); g.lineTo(...lo[j]); g.moveTo(...hi[i]); g.lineTo(...hi[j]); g.moveTo(...lo[i]); g.lineTo(...hi[i]); }
      g.stroke();
      const m = P(c.X); g.fillStyle = '#fff'; g.font = 'bold 16px system-ui'; g.fillText(`${c.colour} z ${(c.X[2] / 10).toFixed(1)}`, m[0] + 10, m[1]);
    }
    for (const d of cubes3d.length ? [] : st.detections) {
      g.strokeStyle = d.ok ? COLOR_CSS[d.colour] : '#888'; g.lineWidth = 3; g.setLineDash(d.ok ? [] : [6, 4]);
      const [a, b, c2, e] = d.bbox; g.strokeRect(a - 4, b - 4, c2 - a + 8, e - b + 8); g.setLineDash([]);
      const p = project(cal, [d.X, d.Y, 0]);
      g.fillStyle = '#fff'; g.beginPath(); g.arc(p[0], p[1], 5, 0, 2 * Math.PI); g.fill();
      g.font = 'bold 18px system-ui'; g.fillStyle = '#fff'; g.fillText(d.colour + (d.ok ? '' : ' ?'), a, b - 10);
    }
  }

  function params() {
    const cal = S.cal, C = cal.centre, Cb = tableToBoard(C);
    const fwd = cal.R[2];                                   // optical axis in the table frame
    const pitch = deg(Math.asin(-fwd[2])), yaw = deg(Math.atan2(fwd[1], fwd[0]));
    const roll = deg(Math.atan2(cal.R[0][2], -cal.R[1][2]));
    const rows = [
      ['Camera', S.cam.name, `${S.cam.api}/*`],
      ['Focal length', `${cal.f.toFixed(1)} px`, `field of view ${(2 * deg(Math.atan(W / 2 / cal.f))).toFixed(1)}° × ${(2 * deg(Math.atan(H / 2 / cal.f))).toFixed(1)}°`],
      ['Principal point', `${cal.cx.toFixed(1)}, ${cal.cy.toFixed(1)} px`, `image centre is 640, 360`],
      ['Position, mat frame', `X ${C[0].toFixed(1)} · Y ${C[1].toFixed(1)} · Z ${C[2].toFixed(1)} mm`, 'X away from the arm, Y to its left'],
      ['Position, board frame', `x ${Cb[0].toFixed(1)} · y ${Cb[1].toFixed(1)} · z ${Cb[2].toFixed(1)} cm`, `with S1 → mat edge d = ${MAT.sToEdge} cm`],
      ['Looking', `down ${pitch.toFixed(1)}°, heading ${yaw.toFixed(1)}°, roll ${roll.toFixed(1)}°`, 'heading 180° = straight back at the arm'],
      ['Fit', Number.isFinite(cal.rms) ? `${cal.rms.toFixed(2)} px rms` : 'not calibrated yet (first guess)', `${usable().length} points`],
    ];
    $('cam-params').innerHTML = rows.map(([a, b, c]) => `<tr><th>${a}</th><td>${b}</td><td class="note">${c}</td></tr>`).join('') +
      `<tr><th>Homography</th><td colspan="2"><code>${cal.H ? cal.H.map(r => r.map(v => v.toPrecision(4)).join('  ')).join('<br>') : '–'}</code></td></tr>`;
  }

  // ---- rendering the simulated camera images (one renderer, copied into each camera's stage)
  let dirty = true;
  function render() {
    if (!visible() || !dirty) return;
    dirty = false;
    for (const st of Object.values(states)) {
      applyPinhole(cam3, st.cal, 960, 540);
      ctx.hideForCamera(() => renderer.render(ctx.scene, cam3));
      st.simG.drawImage(glCanvas, 0, 0, 960, 540);
    }
  }
  function update() { dirty = true; draw(); render(); }

  // The first time the tab opens: a live snapshot of each camera when the real cameras are connected
  // (one read each, only now), otherwise the OAK's saved frame. Nothing is loaded before the tab is opened.
  let activated = false;
  async function activate() {
    if (activated) return;
    activated = true;
    let camera = null;
    try { camera = (await (await fetch('/local/info', { cache: 'no-store' })).json()).camera; } catch { /* static server */ }
    const real = camera && camera !== 'mock';
    for (const st of Object.values(states)) {
      st.source = real ? 'live' : st.cam.id === 'oak' ? 'saved' : 'file';
      if (st.source === 'live') refreshLive(st);
      else if (st.source === 'saved') { st.img.src = 'ref/camera-2026-10-12.jpg'; st.points = savedPairs(); }
    }
    select(S.cam.id);
  }
  // "Cameras" in the main view: the stages move there (and back), with the frames and overlays they have
  const home = $('cam-stages').parentElement, homeNext = $('cam-stages').nextSibling;
  function showInMain(on) {
    $('cam-main').hidden = !on;
    if (on) $('cam-main').append($('cam-stages')); else home.insertBefore($('cam-stages'), homeNext);
    $('view-cams').classList.toggle('on', on);
    if (on) activate();
    update();
  }
  $('view-cams').onclick = () => showInMain($('cam-main').hidden);
  function syncBars() { for (const b of document.querySelectorAll('.cam-bar button[data-for]')) b.classList.toggle('on', $(b.dataset.for).checked); for (const r of document.querySelectorAll('.cam-op')) r.value = $('cam-opacity').value; }
  for (const id of ['cam-show-sim', 'cam-show-grid', 'cam-show-marks', 'cam-show-parts', 'cam-show-bodies', 'cam-show-screws', 'cam-arm-ghost', 'cam-opacity']) $(id).addEventListener(id === 'cam-opacity' ? 'input' : 'change', syncBars);
  syncBars();
  $('cams-full').onclick = () => fullscreen($('cam-stages'));
  document.addEventListener('fullscreenchange', () => update());
  /**
   * The cubes the cameras see now, into the simulation: a fresh snapshot of every live camera (one GET each),
   * then cubes located in 3D from both cameras (or on the mat from one) and placed at their stack levels.
   */
  async function syncCubes() {
    await activate();
    // always an up-to-date frame: a fresh snapshot of every camera (one GET each), or its running video
    const stale = [];
    await Promise.all(Object.values(states).map(st => {
      if (st.source === 'video' && st.img.getAttribute('src')) return null;
      st.source = 'live'; if (st === S) $('cam-source').value = 'live';
      return new Promise(res => {
        let finished = false;
        const done = ok => {
          if (finished) return; finished = true;
          st.img.removeEventListener('load', onLoad); st.img.removeEventListener('error', onErr);
          if (!ok) stale.push(st.cam.name);
          res();
        };
        const onLoad = () => done(true), onErr = () => done(false);
        st.img.addEventListener('load', onLoad); st.img.addEventListener('error', onErr);
        refreshLive(st); setTimeout(() => done(false), 6000);
      });
    }));
    await new Promise(r => setTimeout(r, 300));
    findCubes();
    showDetections(0);
    const note = stale.length ? ` (no fresh frame from ${stale.join(', ')}: its last frame was used)` : ' (fresh frames)';
    const resting3d = cubes3d.filter(resting);
    if (resting3d.length) { ctx.useCubes3d(resting3d); return `${resting3d.length} cubes from ${Math.max(...cubes3d.map(c => c.cameras))} cameras${note}`; }
    const flat = S.detections.filter(d => d.ok);
    if (flat.length) { ctx.useDetections(flat); return `${flat.length} cubes from ${S.cam.name}, assumed on the mat${note}`; }
    throw new Error(stale.length === Object.keys(states).length ? 'No camera could be read: start the server with --camera http://10.102.4.103:8765' : `No cubes found in the camera frames${note}`);
  }
  return {
    syncCubes, showInMain, update, render, markDirty: () => { dirty = true; }, get calibration() { return S.cal; }, get detections() { return S.detections; },
    calibrationOf: id => states[id].cal, addTracker, grabFrame, stopVideo, activate, trackArm, select, startLive, stopLive, get live() { return live; },
    /** Show an image (URL) as a camera's frame, like "Image file…" (for tests and tools). */
    loadFrame: (id, url) => { const st = states[id]; st.source = 'file'; st.img.src = url; if (st === S) $('cam-source').value = 'file'; },
  };
}
