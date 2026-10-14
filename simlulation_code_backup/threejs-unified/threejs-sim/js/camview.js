// Camera tab: what the OAK camera sees in the simulation, with a reference frame on top,
// and the calibration of the camera from the printed mat markers.
//
// Reference frames come only from the repository or a local file selected by the user.

import * as THREE from 'three';
import { CAMERA, MAT } from './config.js';
import { calibrate, project, MARKER_CORNERS, SAVED_FRAME_CORNERS, markerCentre } from './calib.js';
import { detectCubes } from './detect.js';
import { detectMarkers } from './aruco.js';

const W = CAMERA.w, H = CAMERA.h;
const COLOR_CSS = { orange: '#e8792b', blue: '#2f6fd6', yellow: '#d9a800', green: '#2f9b47', red: '#d23b3b' };
const $ = id => document.getElementById(id);
const deg = r => r * 180 / Math.PI;

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
 * Wire up the camera tab. ctx: { scene, hideForCamera(fn), onApply(cal), log, toast }.
 */
export function initCameraTab(ctx) {
  const simCanvas = $('cam-sim'), overlay = $('cam-overlay'), photo = $('cam-photo');
  const renderer = new THREE.WebGLRenderer({ canvas: simCanvas, antialias: true });
  renderer.setPixelRatio(1);
  renderer.setSize(960, 540, false);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  const cam3 = new THREE.PerspectiveCamera();
  overlay.width = 1280; overlay.height = 720;
  const g = overlay.getContext('2d');

  let cal = ctx.initial;
  let points = savedPairs();            // { id, k?, X, uv }
  let picking = null;                    // marker id waiting for a click
  let source = 'saved';

  const visible = () => !$('tab-camera').hidden;

  // ---- local frame sources
  //   saved  the frame from 2026-10-12 in the repo
  //   file   an image from disk
  const setPhoto = url => { photo.src = url; };
  const loadSource = () => {
    source = $('cam-source').value;
    if (source === 'saved') { setPhoto('ref/camera-2026-10-12.jpg'); points = savedPairs(); }
    else if (source === 'file') { points = []; $('cam-file').click(); }
    draw();
  };
  $('cam-source').onchange = loadSource;
  $('cam-file').onchange = e => { const f = e.target.files[0]; if (f) setPhoto(URL.createObjectURL(f)); };
  photo.onerror = () => ctx.toast('The selected reference image could not be loaded');
  // A new reference frame: find markers and refit the camera right away.
  photo.onload = () => {
    if ($('cam-auto-on').checked) setTimeout(() => autoCalibrate({ quiet: true, announce: true }), 50);
  };
  $('cam-opacity').oninput = () => { photo.style.opacity = $('cam-opacity').value / 100; };
  for (const id of ['cam-show-sim', 'cam-show-marks', 'cam-show-grid']) $(id).onchange = () => { simCanvas.style.visibility = $('cam-show-sim').checked ? 'visible' : 'hidden'; draw(); };

  // ---- picking marker centres on the frame
  const nextPick = () => {
    const have = new Set(points.filter(p => p.k === undefined).map(p => p.id));
    picking = [0, 1, 2, 3, 4, 5].find(id => !have.has(id)) ?? null;
    $('cam-pick-hint').textContent = picking === null ? 'All six marker centres set. Press Solve.' : `Click the centre of marker ID ${picking} (highlighted in green).`;
    draw();
  };
  $('cam-pick').onclick = () => { points = points.filter(p => p.k === undefined); nextPick(); };
  $('cam-skip').onclick = () => { if (picking !== null) { points.push({ id: picking, skipped: true }); nextPick(); } };
  $('cam-clear').onclick = () => { points = []; picking = null; $('cam-pick-hint').textContent = ''; draw(); };
  $('cam-use-saved').onclick = () => { $('cam-source').value = 'saved'; loadSource(); };
  overlay.addEventListener('click', e => {
    if (picking === null) return;
    const r = overlay.getBoundingClientRect();
    const uv = [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H];
    const [X, Y] = markerCentre(picking);
    points.push({ id: picking, X: [X, Y, 0], uv });
    nextPick();
  });

  // ---- solve and apply
  const usable = () => points.filter(p => p.X && p.uv);
  $('cam-solve').onclick = () => {
    const p = usable();
    try {
      const fixPrincipal = p.length < 10;
      cal = calibrate(p.map(q => q.X), p.map(q => q.uv), { fixPrincipal });
      ctx.onApply(cal);
      ctx.log(`camera solved from ${p.length} points: f ${cal.f.toFixed(0)} px, ${cal.rms.toFixed(2)} px rms`, 'ok');
      update();
    } catch (e) { ctx.toast(`Calibration failed: ${e.message}`); }
  };
  $('cam-reset').onclick = () => { cal = defaultCalibration(); ctx.onApply(cal); update(); };
  $('cam-copy').onclick = async () => {
    const j = JSON.stringify({ f_px: +cal.f.toFixed(2), cx: +cal.cx.toFixed(2), cy: +cal.cy.toFixed(2), R: cal.R.map(r => r.map(v => +v.toFixed(6))), t_mm: cal.t.map(v => +v.toFixed(2)), centre_mm: cal.centre.map(v => +v.toFixed(1)), rms_px: +cal.rms.toFixed(3) }, null, 1);
    try { await navigator.clipboard.writeText(j); ctx.toast('Camera parameters copied'); } catch { console.log(j); }
  };

  // ---- grabbing the current reference frame
  const grabCanvas = document.createElement('canvas');
  function grabFrame() {
    const w = photo.naturalWidth, h = photo.naturalHeight;
    if (!w || !h) return null;
    grabCanvas.width = w; grabCanvas.height = h;
    const g2 = grabCanvas.getContext('2d', { willReadFrequently: true });
    g2.drawImage(photo, 0, 0);
    try { return g2.getImageData(0, 0, w, h); } catch { return null; }   // cross-origin frames cannot be read
  }

  // ---- trackers: functions run on every grabbed frame while tracking is on.
  // Built in: the cube detector. Future trackers (e.g. an ArUco marker on the gripper for the
  // HOME-angle calibration) register with addTracker(name, (image, camera, time) => result).
  const trackers = new Map();
  const addTracker = (name, fn) => { trackers.set(name, fn); };
  let trackTimer = null, trackFrames = 0, trackMs = 0;
  function stopTracking() { clearInterval(trackTimer); trackTimer = null; if ($('cam-track')) $('cam-track').checked = false; }
  function trackOnce() {
    const img = grabFrame();
    if (!img) return;
    const t0 = performance.now();
    const results = {};
    for (const [name, fn] of trackers) { try { results[name] = fn(img, cal, Date.now()); } catch (e) { results[name] = { error: e.message }; } }
    trackMs = performance.now() - t0; trackFrames++;
    $('cam-track-stats').textContent = `tracking: ${trackFrames} frames, ${trackMs.toFixed(0)} ms per frame`;
  }
  $('cam-track').onchange = () => {
    if (!$('cam-track').checked) return stopTracking();
    trackFrames = 0;
    trackTimer = setInterval(trackOnce, Number($('cam-track-rate').value));
  };
  $('cam-track-rate').onchange = () => { if (trackTimer) { clearInterval(trackTimer); trackTimer = setInterval(trackOnce, Number($('cam-track-rate').value)); } };

  // ---- automatic calibration from the ArUco markers in the current frame
  let markerQuads = {};
  function autoCalibrate({ quiet = false, announce = false } = {}) {
    const img = grabFrame();
    if (!img) { if (!quiet) ctx.toast('No readable frame'); return false; }
    const t0 = performance.now();
    const { markers } = detectMarkers(img);
    const ms = performance.now() - t0;
    const sx = W / img.width, sy = H / img.height;                      // calibration uses the 1280 x 720 grid
    markerQuads = Object.fromEntries(Object.entries(markers).map(([id, q]) => [id, q.map(([x, y]) => [x * sx, y * sy])]));
    const ids = Object.keys(markerQuads).map(Number);
    if (ids.length < 2) {
      if (!quiet) ctx.toast(`Only ${ids.length} marker(s) found; at least 2 are needed (4 or more spread over the mat are better)`);
      $('cam-pick-hint').textContent = `Markers found: ${ids.join(', ') || 'none'} (${ms.toFixed(0)} ms). Too few to calibrate.`;
      draw(); return false;
    }
    points = [];
    for (const id of ids) MARKER_CORNERS[id].forEach((p, k) => points.push({ id, k, X: [p[0], p[1], 0], uv: markerQuads[id][k] }));
    try {
      const next = calibrate(points.map(q => q.X), points.map(q => q.uv), { fixPrincipal: ids.length < 3 });
      if (next.rms > 4) throw new Error(`fit too poor (${next.rms.toFixed(1)} px)`);
      cal = next;
      ctx.onApply(cal);
      $('cam-pick-hint').textContent = `Markers found: ${ids.sort().join(', ')} in ${ms.toFixed(0)} ms. Camera fitted to ${points.length} corners, ${cal.rms.toFixed(2)} px rms.`;
      if (!quiet || announce) ctx.log(`camera calibrated from markers ${ids.join(', ')}: f ${cal.f.toFixed(0)} px, ${cal.rms.toFixed(2)} px rms, camera at ${cal.centre.map(v => v.toFixed(0)).join(' / ')} mm (mat frame); 3D model updated`, 'ok');
      update();
      return true;
    } catch (e) { if (!quiet) ctx.toast(`Calibration failed: ${e.message}`); draw(); return false; }
  }
  $('cam-auto').onclick = () => autoCalibrate();
  let lastKeep = 0;
  addTracker('camera', () => {
    if (!$('cam-keep').checked || Date.now() - lastKeep < 5000) return null;
    lastKeep = Date.now();
    return autoCalibrate({ quiet: true });
  });

  // ---- cube recognition (read only: works on the frame shown above)
  let detections = [];
  addTracker('cubes', (img, c) => {
    detections = detectCubes(img, c);
    showDetections(0);
    if ($('cam-track-sim').checked) ctx.trackDetections(detections.filter(d => d.ok));
    return detections;
  });
  $('cam-detect').onclick = () => {
    const img = grabFrame();
    if (!img) { ctx.toast(photo.naturalWidth ? 'This frame cannot be read (cross-origin)' : 'No frame loaded'); return; }
    const t0 = performance.now();
    detections = detectCubes(img, cal);
    showDetections(performance.now() - t0);
  };
  function showDetections(ms) {
    $('cam-detections').innerHTML = detections.length
      ? detections.map(d => {
          const [x, y] = tableToBoard([d.X, d.Y, 0]);
          return `<div><b style="color:${COLOR_CSS[d.colour]}">${d.colour}</b> mat X ${d.X.toFixed(0)} · Y ${d.Y.toFixed(0)} mm → board x ${x.toFixed(1)} · y ${y.toFixed(1)} cm, yaw ${d.yaw.toFixed(0)}° ${d.ok ? '' : `<span class="warn">size ${(d.areaRatio * 100).toFixed(0)} % of a cube on the mat: in the gripper or partly hidden?</span>`}</div>`;
        }).join('') + (ms ? `<div>${ms.toFixed(0)} ms</div>` : '')
      : 'No cubes found on the mat.';
    $('cam-detect-use').disabled = !detections.some(d => d.ok);
    draw();
  }
  $('cam-detect-use').onclick = () => { ctx.useDetections(detections.filter(d => d.ok)); };

  // ---- drawing
  function draw() {
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
        g.lineWidth = picking === id ? 4 : 2; g.strokeStyle = picking === id ? '#19d36b' : 'rgba(255,170,0,0.95)';
        g.beginPath(); q.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke();
        const c = P([...markerCentre(id), 0]);
        g.fillStyle = '#ffb000'; g.font = 'bold 20px system-ui'; g.fillText(`ID ${id}`, c[0] + 22, c[1] - 18);
      }
      // measured points (circles) and their reprojection error (red line, 5x)
      for (const p of usable()) {
        const r = P(p.X);
        g.strokeStyle = '#e0281e'; g.lineWidth = 2;
        g.beginPath(); g.moveTo(...p.uv); g.lineTo(p.uv[0] + 5 * (r[0] - p.uv[0]), p.uv[1] + 5 * (r[1] - p.uv[1])); g.stroke();
        g.strokeStyle = '#ffffff'; g.lineWidth = 2; g.beginPath(); g.arc(...p.uv, 6, 0, 2 * Math.PI); g.stroke();
        g.strokeStyle = '#2f6fd6'; g.beginPath(); g.moveTo(r[0] - 6, r[1]); g.lineTo(r[0] + 6, r[1]); g.moveTo(r[0], r[1] - 6); g.lineTo(r[0], r[1] + 6); g.stroke();
      }
    }
    // markers as detected in the frame (green), with their first corner marked
    for (const [id, q] of Object.entries(markerQuads)) {
      g.strokeStyle = '#19d36b'; g.lineWidth = 2.5; g.beginPath(); q.forEach((p, i) => (i ? g.lineTo(...p) : g.moveTo(...p))); g.closePath(); g.stroke();
      g.fillStyle = '#19d36b'; g.beginPath(); g.arc(...q[0], 4, 0, 2 * Math.PI); g.fill();
      g.font = 'bold 16px system-ui'; g.fillText(`found ${id}`, q[2][0] + 6, q[2][1] + 16);
    }
    for (const d of detections) {
      g.strokeStyle = d.ok ? COLOR_CSS[d.colour] : '#888'; g.lineWidth = 3; g.setLineDash(d.ok ? [] : [6, 4]);
      const [a, b, c2, e] = d.bbox; g.strokeRect(a - 4, b - 4, c2 - a + 8, e - b + 8); g.setLineDash([]);
      const p = project(cal, [d.X, d.Y, 0]);
      g.fillStyle = '#fff'; g.beginPath(); g.arc(p[0], p[1], 5, 0, 2 * Math.PI); g.fill();
      g.font = 'bold 18px system-ui'; g.fillStyle = '#fff'; g.fillText(d.colour + (d.ok ? '' : ' ?'), a, b - 10);
    }
    params();
  }

  function params() {
    const C = cal.centre, Cb = tableToBoard(C);
    const fwd = cal.R[2];                                   // optical axis in the table frame
    const pitch = deg(Math.asin(-fwd[2])), yaw = deg(Math.atan2(fwd[1], fwd[0]));
    const roll = deg(Math.atan2(cal.R[0][2], -cal.R[1][2]));
    const rows = [
      ['Focal length', `${cal.f.toFixed(1)} px`, `field of view ${(2 * deg(Math.atan(W / 2 / cal.f))).toFixed(1)}° × ${(2 * deg(Math.atan(H / 2 / cal.f))).toFixed(1)}°`],
      ['Principal point', `${cal.cx.toFixed(1)}, ${cal.cy.toFixed(1)} px`, `image centre is 640, 360`],
      ['Position, mat frame', `X ${C[0].toFixed(1)} · Y ${C[1].toFixed(1)} · Z ${C[2].toFixed(1)} mm`, 'X away from the arm, Y to its left'],
      ['Position, board frame', `x ${Cb[0].toFixed(1)} · y ${Cb[1].toFixed(1)} · z ${Cb[2].toFixed(1)} cm`, `with S1 → mat edge d = ${MAT.sToEdge} cm`],
      ['Looking', `down ${pitch.toFixed(1)}°, heading ${yaw.toFixed(1)}°, roll ${roll.toFixed(1)}°`, 'heading 180° = straight back at the arm'],
      ['Fit', `${cal.rms.toFixed(2)} px rms`, `${usable().length} points`],
    ];
    $('cam-params').innerHTML = rows.map(([a, b, c]) => `<tr><th>${a}</th><td>${b}</td><td class="note">${c}</td></tr>`).join('') +
      `<tr><th>Homography</th><td colspan="2"><code>${cal.H ? cal.H.map(r => r.map(v => v.toPrecision(4)).join('  ')).join('<br>') : '–'}</code></td></tr>`;
  }

  // ---- rendering the simulated camera image
  let dirty = true;
  function render() {
    if (!visible() || !dirty) return;
    dirty = false;
    applyPinhole(cam3, cal, 960, 540);
    ctx.hideForCamera(() => renderer.render(ctx.scene, cam3));
  }
  function update() { dirty = true; draw(); render(); }

  // Load the repository reference only when the tab is opened.
  let activated = false;
  function activate() {
    if (activated) return;
    activated = true;
    $('cam-source').value = 'saved';
    loadSource();
  }
  return { update, render, markDirty: () => { dirty = true; }, get calibration() { return cal; }, addTracker, grabFrame, activate };
}
