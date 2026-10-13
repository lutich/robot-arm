// "Arm simulation" section of the manual control page. It draws what the mock controller's
// arm simulation reports (GET /api/sim/model and /api/sim/world): the synthetic arm in
// Preview, or the real arm while the mock follows a real controller (--follow). Every number
// comes from the API. A view only: it sends nothing.

import * as THREE from 'three';
import { createView } from './render.js';
import { MAT, CAMERA } from './config.js';
import { fk, anglesFromCounts } from './kinematics.js';
import { makeMaterials, buildArm, buildTable, buildCamera } from './model/index.js';
import { initCameraTab, defaultCalibration, applyPinhole, tableToBoard } from './camview.js';

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => v.toFixed(d).replace('-0.0', '0.0');
const TRACE_SECONDS = 20;
const CAM_KEY = 'roboter-arm.sim.camera.v1';

/** ctx: { onStatus(label, ok), notice(text, error) }. Returns { setOpen(open), onState(state) }. */
export function start(ctx) {
  let needsRender = true, last = performance.now(), frameId = null;

  // ---------------- renderer ----------------
  const canvas = $('sim-canvas');
  const view = createView(canvas);
  const { scene, camera, controls } = view;
  camera.position.set(-30, 50, 30);
  controls.target.set(0, 11, 7);
  controls.update();

  const M = makeMaterials();
  const table = buildTable(M); scene.add(table);
  const backdrop = table.getObjectByName('backdrop');
  const arm = buildArm(M); scene.add(arm.root);

  // The OAK camera, fitted to the mat markers (camview.js); the fit is kept per browser.
  CAMERA.calib = defaultCalibration();
  try { const c = JSON.parse(localStorage.getItem(CAM_KEY)); if (c && c.R && c.f) CAMERA.calib = c; } catch { /* default fit */ }
  let oak = null;
  const camPose = () => {
    const cal = CAMERA.calib, C = cal.centre, z = cal.R[2];
    const s = -C[2] / z[2];                                   // optical axis meets the mat
    return { pos: tableToBoard(C), target: tableToBoard([C[0] + s * z[0], C[1] + s * z[1], 0]) };
  };
  const hideForCamera = fn => { const v = oak.visible; oak.visible = false; fn(); oak.visible = v; };
  function placeMatAndCamera() {
    table.userData.placeMat(MAT.sToEdge);
    if (oak) scene.remove(oak);
    const p = camPose(); oak = buildCamera(M, p.pos, p.target);
    scene.add(oak);
  }
  placeMatAndCamera();

  // ---------------- collision shape overlay ----------------
  const rotation = R => new THREE.Matrix4().set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);
  const shapeGroup = new THREE.Group(); scene.add(shapeGroup);
  const shapeMeshes = new Map();
  const wireOk = new THREE.LineBasicMaterial({ color: 0x0e8a77, transparent: true, opacity: 0.7 });
  const wireHit = new THREE.LineBasicMaterial({ color: 0xe0281e });
  const wirePress = new THREE.LineBasicMaterial({ color: 0xe0a01e });
  const unitBox = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
  function updateShapes(shapes, contacts, showAll) {
    const hits = new Map();             // name -> pressing only?
    for (const k of contacts) for (const n of [k.a, k.b]) hits.set(n, (hits.get(n) ?? true) && k.pressing);
    const used = new Set();
    for (const s of shapes) {
      const hit = hits.has(s.name);
      if (!showAll && !hit) continue;
      used.add(s.name);
      let m = shapeMeshes.get(s.name);
      if (!m) {
        if (s.kind === 'box') m = new THREE.LineSegments(unitBox, wireOk);
        else {
          const len = Math.hypot(s.b[0] - s.a[0], s.b[1] - s.a[1], s.b[2] - s.a[2]);
          m = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.CapsuleGeometry(s.r, len, 4, 10), 30), wireOk);
        }
        shapeMeshes.set(s.name, m); shapeGroup.add(m);
      }
      m.visible = true;
      m.material = !hit ? wireOk : hits.get(s.name) ? wirePress : wireHit;
      if (s.kind === 'box') {
        m.position.set(...s.c); m.scale.set(2 * s.h[0], 2 * s.h[1], 2 * s.h[2]);
        m.setRotationFromMatrix(rotation(s.R));
      } else {
        m.position.set((s.a[0] + s.b[0]) / 2, (s.a[1] + s.b[1]) / 2, (s.a[2] + s.b[2]) / 2);
        m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(s.b[0] - s.a[0], s.b[1] - s.a[1], s.b[2] - s.a[2]).normalize());
      }
    }
    for (const [n, m] of shapeMeshes) if (!used.has(n)) m.visible = false;
  }

  // ---------------- path: gripper trace and the demo's positions ----------------
  const line = color => { const l = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color })); l.frustumCulled = false; scene.add(l); return l; };
  const traceLine = line(0x2f6fd6);
  const demoLine = line(0xff7a1a);
  const demoDots = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xff7a1a, size: 0.6 }));
  demoDots.frustumCulled = false; scene.add(demoDots);
  const trace = [];                      // [time, x, y, z]
  let demoKey = '';
  const setPoints = (obj, pts) => { obj.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3)); obj.geometry.computeBoundingSphere(); };
  function showDemo(positions) {
    const key = JSON.stringify(positions.map(p => p.counts));
    if (key === demoKey) return;
    demoKey = key;
    const pts = [];
    for (const p of positions) {
      const c = [0, 1, 2, 3, 4, 5].map(i => p.counts[i] ?? p.counts[String(i)]);
      if (c.every(v => typeof v === 'number')) pts.push(...fk(anglesFromCounts(c)).grasp);
    }
    setPoints(demoLine, pts); setPoints(demoDots, pts);
    needsRender = true;
  }
  const showPath = () => { traceLine.visible = demoLine.visible = demoDots.visible = $('sim-show-path').checked; needsRender = true; };
  $('sim-show-path').onchange = showPath;
  showPath();

  // ---------------- views ----------------
  const VIEWS = {
    orbit: () => { camera.fov = 36; camera.position.set(-30, 50, 30); controls.target.set(0, 11, 7); },
    top: () => { camera.fov = 36; camera.position.set(0, 15.5, 95); controls.target.set(0, 15.5, 0); },
    side: () => { camera.fov = 36; camera.position.set(75, 12, 14); controls.target.set(0, 12, 12); },
  };
  let oakView = false;
  function applyOakView() {
    const r = canvas.getBoundingClientRect();
    applyPinhole(camera, CAMERA.calib, r.width, r.height);
    controls.target.set(...camPose().target);
  }
  function leaveOakView() {
    if (!oakView) return;
    oakView = false; controls.enabled = true;
    camera.fov = 36; camera.updateProjectionMatrix();
    document.querySelectorAll('[data-sim-view]').forEach(x => x.classList.toggle('on', x.dataset.simView === 'orbit'));
  }
  document.querySelectorAll('[data-sim-view]').forEach(b => b.onclick = () => {
    document.querySelectorAll('[data-sim-view]').forEach(x => x.classList.toggle('on', x === b));
    if (b.dataset.simView === 'oak') { oakView = true; controls.enabled = false; applyOakView(); }
    else { leaveOakView(); VIEWS[b.dataset.simView](); camera.updateProjectionMatrix(); controls.update(); }
    needsRender = true;
  });
  canvas.addEventListener('pointerdown', () => { if (oakView) { leaveOakView(); controls.update(); } }, true);
  const resize = () => { view.resize(); if (oakView) applyOakView(); needsRender = true; };
  new ResizeObserver(resize).observe(canvas.parentElement);

  // ---------------- simulation from the API ----------------
  let world = null, open = false, timer = null, flight = false, lastSeq = 0;
  let pose = null, gap = 4.5;

  function log(text, kind = '') {
    const li = document.createElement('li');
    li.textContent = text;
    if (kind) li.className = kind;
    $('sim-log').prepend(li);
    while ($('sim-log').children.length > 40) $('sim-log').lastChild.remove();
  }
  const list = ks => ks.map(k => `${k.a} × ${k.b} ${fmt(-k.sep * 10)} mm`).join(', ');

  async function poll() {
    if (flight) return;
    flight = true;
    try {
      const r = await fetch('/api/sim/world', { cache: 'no-store' });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      world = await r.json();
      if (!pose) pose = world.counts.slice();
      for (const e of world.events) if (e.seq > lastSeq) log(e.text, e.kind === 'warn' || e.kind === 'collision' ? 'warn' : '');
      lastSeq = Math.max(lastSeq, ...world.events.map(e => e.seq));
      const now = performance.now() / 1000;
      const last = trace[trace.length - 1];
      if (!last || Math.hypot(last[1] - world.grasp[0], last[2] - world.grasp[1], last[3] - world.grasp[2]) > 0.05) trace.push([now, ...world.grasp]);
      while (trace.length && trace[0][0] < now - TRACE_SECONDS) trace.shift();
      setPoints(traceLine, trace.flatMap(t => t.slice(1)));
      updatePanel();
      needsRender = true;
    } catch (e) {
      ctx.onStatus('Unavailable', false);
      $('sim-readout').textContent = `Simulation unavailable: ${e.message}`;
    } finally {
      flight = false;
    }
  }

  function updatePanel() {
    const w = world;
    const label = w.stopped_by ? 'Simulated collision' : w.follow_error ? 'Real controller unreachable'
      : w.mode === 'following' ? 'Following the real arm' : 'Synthetic · Preview';
    ctx.onStatus(label, !w.stopped_by && !w.follow_error);
    $('sim-mode').textContent = w.mode === 'following'
      ? `Following ${w.following}: the real arm's commands. The model is not calibrated, so contacts are only shown.`
        + (w.follow_error ? ` Last read failed: ${w.follow_error}` : '')
      : 'Preview: the synthetic arm. Moves that would collide are stopped.';
    $('sim-readout').textContent = `Cube-centre point x ${fmt(w.grasp[0])} · y ${fmt(w.grasp[1])} · z ${fmt(w.grasp[2])} cm · `
      + `tool ${fmt(w.tool.tilt, 0)}° off vertical · jaw gap ${fmt(w.gap, 2)} cm`;
    const hard = w.contacts.filter(k => !k.pressing), pressing = w.contacts.filter(k => k.pressing);
    $('sim-contacts').hidden = !w.contacts.length && !w.stopped_by;
    $('sim-contacts').textContent = w.stopped_by ? `Stopped: ${w.stopped_by}` : [
      hard.length ? `${w.resting ? 'Resting on the base' : 'Contact'}: ${list(hard)}` : '',
      pressing.length ? `Pressing (the servo stops short of its command): ${list(pressing)}` : '',
    ].filter(Boolean).join(' — ');
    $('sim-contacts').classList.toggle('error', !!w.stopped_by || (hard.length > 0 && !w.resting));
  }

  // ---------------- camera & vision ----------------
  const camTab = initCameraTab({
    scene, initial: CAMERA.calib, hideForCamera, toast: text => ctx.notice(text, true),
    log: (text, kind) => log(text, kind === 'ok' ? 'ok' : ''),
    linkBase: () => '',
    visible: () => open && $('sim-camera').open,
    onApply: cal => {
      CAMERA.calib = cal;
      try { localStorage.setItem(CAM_KEY, JSON.stringify({ R: cal.R, t: cal.t, f: cal.f, cx: cal.cx, cy: cal.cy, centre: cal.centre, rms: cal.rms, H: cal.H })); } catch { /* not saved */ }
      placeMatAndCamera(); if (oakView) applyOakView(); needsRender = true;
    },
  });
  $('sim-camera').addEventListener('toggle', () => { if ($('sim-camera').open) { camTab.activate(); camTab.update(); } else camTab.stopVideo('section closed'); });

  // ---------------- loop ----------------
  controls.addEventListener('change', () => { needsRender = true; });
  $('sim-show-shapes').onchange = () => { needsRender = true; };

  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (world && pose) {
      // ease toward the 10 Hz pose of the simulation
      let moved = false;
      pose = pose.map((p, i) => { const q = p + (world.counts[i] - p) * Math.min(1, dt * 14); if (Math.abs(q - p) > 1e-3) moved = true; return q; });
      const g = gap + (world.gap - gap) * Math.min(1, dt * 14);
      if (Math.abs(g - gap) > 1e-4) moved = true;
      gap = g;
      if (moved) needsRender = true;
    }
    if (!oakView && controls.update()) needsRender = true;
    if (needsRender && world) {
      arm.setPose(anglesFromCounts(pose), gap);
      // resting on the base is expected, like pressing: amber, not red
      updateShapes(world.shapes, world.contacts.map(k => ({ ...k, pressing: k.pressing || world.resting })), $('sim-show-shapes').checked);
      if (backdrop) backdrop.visible = camera.position.y > backdrop.position.y + 2;
      if (oakView) hideForCamera(view.render); else view.render();
      camTab.markDirty(); camTab.render();
      needsRender = false;
    }
    frameId = open ? requestAnimationFrame(frame) : null;
  }

  function setOpen(value) {
    open = value;
    clearInterval(timer);
    if (open) {
      poll();
      timer = setInterval(poll, 100);
      resize();
      if (frameId === null) { last = performance.now(); frameId = requestAnimationFrame(frame); }
    } else camTab.stopVideo('section closed');
  }

  return {
    setOpen,
    // the loaded demo's positions, from the /api/state the page already reads
    onState(state) { if (state?.demo) showDemo(state.demo.positions || []); },
  };
}
