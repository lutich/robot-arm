// Page glue: rendering, joint control, tasks, scene editing, live link, calibration.

import * as THREE from 'three';
import { createView } from './render.js';
import { SERVOS, MAT, CAMERA, CAMERAS, CUBE, COLOURS, PLAN, CONTROLLER } from './config.js';
import { calFromLookAt } from './armtrack.js';
import { buildWebcam } from './model/tags.js';
import { buildTagOverlay, buildSeenOverlay, label } from './tagoverlay.js';
import { cubeText } from './cubetrack.js';
import { fk, anglesFromCounts, countsFromAngles, ikTopDown, guardViolations, camToBoard, toolYawAndTilt, Unreachable } from './kinematics.js';
import { armShapes } from './collision.js';
import { SimController, ControllerError } from './controller.js';
import { World } from './world.js';
import { cubeTilt, cubeYaw } from './physics.js';
import { planTransfer, verify, toMoves, parseCommand, PlanError } from './planner.js';
import { LiveLink } from './live.js';
import { SimArm, runCalls, movesToCalls } from './armapi.js';
import { makeMaterials, buildArm, buildTable, buildCamera, buildCube } from './model/index.js';
import { initCameraTab, defaultCalibration, applyPinhole, tableToBoard } from './camview.js';
import { initDemo } from './demo.js';
import { initScenarios, TAUGHT } from './scenarios.js';

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => v.toFixed(d).replace('-0.0', '0.0');

// ---------------- calibration (editable, saved per browser) ----------------

const CAL_KEY = 'pxsim.calibration.v1';
const DEFAULT_CAL = { servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge };
function applyCal(cal) {
  cal.servos.forEach((c, i) => Object.assign(SERVOS[i], c));
  MAT.sToEdge = cal.d;
}
try { const saved = JSON.parse(localStorage.getItem(CAL_KEY)); if (saved?.servos?.length === 5) applyCal(saved); } catch { /* defaults */ }
const currentCal = () => ({ servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge });

// ---------------- sim state ----------------

const ctl = new SimController();
const world = new World();
const DEFAULT_SCENE = [['orange', 10.8, 14.0, 0, 3], ['blue', -8, 12, 0, 0], ['yellow', 3, 10, 0, 0], ['green', -4, 17, 0, 10]];
function resetScene() {
  world.cubes = []; world.holding = null;
  for (const [c, x, y, l, yaw] of DEFAULT_SCENE) world.addCube(c, c, x, y, l, yaw);
}
resetScene();

// ---------------- renderer ----------------

const canvas = $('canvas');
const view = createView(canvas);
const { renderer, scene, camera, controls } = view;
camera.position.set(-30, 50, 30);
controls.target.set(0, 11, 7);
controls.update();

const M = makeMaterials();
const table = buildTable(M); scene.add(table);
const backdrop = table.getObjectByName('backdrop');
const arm = buildArm(M); scene.add(arm.root);

// ghost of the real arm (live mirror)
const ghostMat = new THREE.MeshStandardMaterial({ color: 0x18a38f, transparent: true, opacity: 0.16, depthWrite: false });
const ghost = buildArm(new Proxy({}, { get: () => ghostMat }));
ghost.root.traverse(o => { o.castShadow = false; o.receiveShadow = false; });
for (const c of ghost.root.children) if (c.isMesh) c.visible = false;   // the base never moves; draw only the moving links
ghost.root.visible = false; scene.add(ghost.root);

// The cameras (config.js CAMERAS): each fitted to the mat markers in its own frames (camview.js); the
// fits are kept per browser. Before a fit: the OAK's fit to the saved frame, the webcam's first guess.
const CAM_KEY = id => (id === 'oak' ? 'pxsim.camera.v1' : `pxsim.camera.${id}.v1`);
const camCals = {};
for (const c of CAMERAS) {
  camCals[c.id] = c.id === 'oak' ? defaultCalibration() : calFromLookAt(c.posMm, c.lookAtMm, c.fPx, c.w, c.h);
  try { const k = JSON.parse(localStorage.getItem(CAM_KEY(c.id))); if (k && k.R && k.f) camCals[c.id] = k; } catch { /* default */ }
}
CAMERA.calib = camCals.oak;
const camBodies = {};
function camPose(cal) {
  const C = cal.centre, z = cal.R[2];
  const s = z[2] < -0.05 ? -C[2] / z[2] : 500;              // where the optical axis meets the mat (or 50 cm ahead)
  return { pos: tableToBoard(C), target: tableToBoard([C[0] + s * z[0], C[1] + s * z[1], C[2] + s * z[2]]) };
}
/** Render with the cameras' own bodies hidden (they would block a camera's view). */
const hideForCamera = fn => {
  const was = Object.values(camBodies).map(b => b.visible);
  Object.values(camBodies).forEach(b => { b.visible = false; });
  fn();
  Object.values(camBodies).forEach((b, i) => { b.visible = was[i]; });
};
function placeCamera(c) {
  if (camBodies[c.id]) scene.remove(camBodies[c.id]);
  const cal = camCals[c.id], p = camPose(cal);
  const body = c.id === 'oak' ? buildCamera(M, p.pos, p.target) : buildWebcam(M, p.pos, p.target);
  // viewing pyramid: rays through the four image corners down to the desk (at most 1,5 m)
  const C = tableToBoard(cal.centre), pts = [];
  for (const [u, v] of [[0, 0], [c.w, 0], [c.w, c.h], [0, c.h]]) {
    const d = [(u - cal.cx) / cal.f, (v - cal.cy) / cal.f, 1];
    const wd = [0, 1, 2].map(i => cal.R[0][i] * d[0] + cal.R[1][i] * d[1] + cal.R[2][i] * d[2]);   // ray in the mat frame
    const sc = wd[2] < -1e-6 ? Math.min((-10 - cal.centre[2]) / wd[2], 1500 / Math.hypot(...wd)) : 1500 / Math.hypot(...wd);   // desk is 10 mm below the mat
    pts.push(tableToBoard(cal.centre.map((x, i) => x + sc * wd[i])));
  }
  const seg = [];
  for (let i = 0; i < 4; i++) seg.push(...C, ...pts[i], ...pts[i], ...pts[(i + 1) % 4]);
  const frustum = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(seg, 3)),
    new THREE.LineBasicMaterial({ color: c.id === 'oak' ? 0x0e8a77 : 0x6b4fd6, transparent: true, opacity: 0.55 }));
  frustum.name = `${c.id}-frustum`;
  body.add(frustum);
  body.name = c.id;
  camBodies[c.id] = body;
  scene.add(body);
}
function placeMatAndCamera() {
  table.userData.placeMat(MAT.sToEdge);
  for (const c of CAMERAS) placeCamera(c);
}
placeMatAndCamera();

// which tag sits on which rigid part, and what moves it (toggle "Tags & parts")
const tagOverlay = buildTagOverlay();
tagOverlay.group.visible = false; scene.add(tagOverlay.group);
// the tags the cameras see right now, on the arm model (while the arm is drawn at its tracked pose)
const seenOverlay = buildSeenOverlay();
seenOverlay.group.visible = false; scene.add(seenOverlay.group);

// cubes
const cubeMeshes = new Map();
function syncCubeMeshes() {
  for (const [id, m] of cubeMeshes) if (!world.cube(id)) { scene.remove(m); cubeMeshes.delete(id); }
  for (const c of world.cubes) {
    let m = cubeMeshes.get(c.id);
    if (!m) { m = buildCube(c.colour); cubeMeshes.set(c.id, m); scene.add(m); }
    m.position.set(...c.p);
    if (c.R) m.setRotationFromMatrix(new THREE.Matrix4().set(c.R[0], c.R[1], c.R[2], 0, c.R[3], c.R[4], c.R[5], 0, c.R[6], c.R[7], c.R[8], 0, 0, 0, 0, 1));
    else { m.rotation.set(0, 0, c.yaw * Math.PI / 180); }
  }
}

// collision shape overlay
const shapeGroup = new THREE.Group(); scene.add(shapeGroup);
const shapeMeshes = new Map();
const wireOk = new THREE.LineBasicMaterial({ color: 0x0e8a77, transparent: true, opacity: 0.7 });
const wireHit = new THREE.LineBasicMaterial({ color: 0xe0281e });
const unitBox = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
function updateShapes(shapes, hitNames, showAll) {
  const used = new Set();
  for (const s of shapes) {
    const hit = hitNames.has(s.name);
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
    m.material = hit ? wireHit : wireOk;
    if (s.kind === 'box') {
      m.position.set(...s.c); m.scale.set(2 * s.h[0], 2 * s.h[1], 2 * s.h[2]);
      const R = s.R;
      m.setRotationFromMatrix(new THREE.Matrix4().set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1));
    } else {
      m.position.set((s.a[0] + s.b[0]) / 2, (s.a[1] + s.b[1]) / 2, (s.a[2] + s.b[2]) / 2);
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(s.b[0] - s.a[0], s.b[1] - s.a[1], s.b[2] - s.a[2]).normalize());
    }
  }
  for (const [n, m] of shapeMeshes) if (!used.has(n)) m.visible = false;
}

// planned path
const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff7a1a }));
pathLine.frustumCulled = false; scene.add(pathLine);
const pathDots = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xff7a1a, size: 0.45 }));
pathDots.frustumCulled = false; scene.add(pathDots);
function showPath(steps, start) {
  const pts = [], dots = [];
  let prev = start;
  for (const s of steps) {
    for (let k = 1; k <= 6; k++) {
      const c = prev.map((v, i) => v + (s.counts[i] - v) * k / 6);
      pts.push(...fk(anglesFromCounts(c)).grasp);
    }
    dots.push(...fk(anglesFromCounts(s.counts)).grasp);
    prev = s.counts;
  }
  pathLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  pathDots.geometry.setAttribute('position', new THREE.Float32BufferAttribute(dots, 3));
  pathLine.geometry.computeBoundingSphere();
  needsRender = true;
}

// ---------------- views ----------------

const VIEWS = {
  orbit: () => { camera.fov = 36; camera.position.set(-30, 50, 30); controls.target.set(0, 11, 7); },
  top: () => { camera.fov = 36; camera.position.set(0, 15.5, 95); controls.target.set(0, 15.5, 0); },
  side: () => { camera.fov = 36; camera.position.set(75, 12, 14); controls.target.set(0, 12, 12); },
  oak: () => { oakView = 'oak'; controls.enabled = false; applyOakView(); },
  webcam: () => { oakView = 'webcam'; controls.enabled = false; applyOakView(); },
};
let oakView = false;
function applyOakView() {
  const r = canvas.getBoundingClientRect();
  const cal = camCals[oakView];
  applyPinhole(camera, cal, r.width, r.height);
  controls.target.set(...camPose(cal).target);
}
function leaveOakView() {
  if (!oakView) return;
  oakView = false; controls.enabled = true;
  camera.fov = 36; camera.updateProjectionMatrix();
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x.dataset.view === 'orbit'));
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x === b));
  if (b.dataset.view !== 'oak' && b.dataset.view !== 'webcam') { leaveOakView(); VIEWS[b.dataset.view](); camera.updateProjectionMatrix(); controls.update(); }
  else VIEWS[b.dataset.view]();
  needsRender = true;
});
// dragging in the OAK view leaves it and orbits from where the camera is
canvas.addEventListener('pointerdown', () => { if (oakView) { leaveOakView(); controls.update(); } }, true);

function resize() { view.resize(); if (oakView) applyOakView(); needsRender = true; }
new ResizeObserver(resize).observe(canvas.parentElement);

// ---------------- resizable sidebar ----------------

{
  const KEY = 'pxsim.panelWidth', panel = $('panel'), split = $('splitter');
  const setW = w => { const px = Math.round(Math.max(300, Math.min(innerWidth * 0.75, w))); document.documentElement.style.setProperty('--panel-w', px + 'px'); return px; };
  try { const w = Number(localStorage.getItem(KEY)); if (w) setW(w); } catch { /* default width */ }
  split.addEventListener('pointerdown', e => {
    try { split.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    split.classList.add('drag'); document.body.classList.add('resizing');
    const move = ev => setW(innerWidth - ev.clientX - split.offsetWidth / 2);
    const up = () => {
      split.removeEventListener('pointermove', move); split.classList.remove('drag'); document.body.classList.remove('resizing');
      try { localStorage.setItem(KEY, String(panel.getBoundingClientRect().width)); } catch { /* not saved */ }
    };
    split.addEventListener('pointermove', move);
    split.addEventListener('pointerup', up, { once: true });
  });
  split.addEventListener('dblclick', () => { document.documentElement.style.removeProperty('--panel-w'); try { localStorage.removeItem(KEY); } catch { /* ignore */ } });
}

// ---------------- log and toast ----------------

function log(text, cls = '') {
  const d = document.createElement('div');
  d.className = cls;
  d.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
  $('log').prepend(d);
  while ($('log').childNodes.length > 300) $('log').lastChild.remove();
}
let toastTimer = null;
function toast(text, ms = 4000) {
  $('toast').textContent = text; $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { $('toast').hidden = true; }, ms);
}
ctl.on((kind, detail) => {
  if (kind === 'stop') log(`STOP: ${detail}`, 'err');
  else if (kind === 'power') log(`power ${detail}`);
});

// ---------------- joints panel ----------------

const jointRows = [];
for (const s of SERVOS) {
  const row = document.createElement('div');
  row.className = 'joint';
  row.innerHTML = `<div class="name">${s.name}<small>${s.joint}</small></div>
    <input type="range" min="${s.low}" max="${s.high}" step="1">
    <input type="number" min="${s.low}" max="${s.high}" step="1">
    <button title="jog −">−</button><button title="jog +">+</button><div class="deg"></div>`;
  const [range, num] = row.querySelectorAll('input');
  const [minus, plus] = row.querySelectorAll('button');
  const r = { range, num, deg: row.querySelector('.deg'), dragging: false };
  range.addEventListener('pointerdown', () => { r.dragging = true; });
  range.addEventListener('input', () => { num.value = range.value; });
  range.addEventListener('change', () => { r.dragging = false; command('move', s.ch, Number(range.value)); });
  num.addEventListener('change', () => command('move', s.ch, Math.round(Number(num.value))));
  minus.onclick = () => command('jog', s.ch, -1);
  plus.onclick = () => command('jog', s.ch, 1);
  $('joints').append(row);
  jointRows.push(r);
}
$('step').onchange = () => { ctl.settings.step_size = Math.max(1, Math.round(Number($('step').value))); };

function updateJointPanel(counts) {
  const a = anglesFromCounts(counts);
  const deg = [a.s1, a.s2, a.s3, a.s4, a.s5];
  counts.forEach((c, i) => {
    const r = jointRows[i];
    if (!r.dragging && document.activeElement !== r.num) { r.range.value = Math.round(c); r.num.value = Math.round(c); }
    r.deg.textContent = i < 5 ? `model angle ${fmt(deg[i])}°` : `gap ${fmt(world.gap, 2)} cm${world.holding ? ` · holding ${world.holding.id}` : ''}`;
  });
}

// The page's own simulated arm behind the controller's API (armapi.js): used only without server.mjs.
const simArm = new SimArm(ctl);

/** Sim check of one joint move before sending it anywhere: does the swept path hit something? */
function moveIsClear(from, to) {
  const snap = world.snapshot();
  let hit = null;
  for (let k = 1; k <= 12 && !hit; k++) {
    world.step(0, from.map((v, i) => v + (to[i] - v) * k / 12));
    if (world.contacts.length) hit = world.contacts[0];
  }
  world.restore(snap);
  world.step(0, from);
  return hit;
}

async function command(kind, ch, value) {
  try {
    const cur = serverMode() ? LiveLink.counts(link.state) : ctl.commands;
    const target = kind === 'jog' ? cur[ch] + value * ctl.settings.step_size : value;
    await send([{ op: 'move', ch, target, label: `move ${SERVOS[ch].name} → ${target}` }]);
  } catch (e) { toast(e.message); log(e.message, 'err'); }
}

const poseCmd = name => async () => {
  try { await send([{ op: 'go_pose', name, label: `go ${name}` }]); }
  catch (e) { toast(e.message); log(e.message, 'err'); }
};
$('go-home').onclick = poseCmd('HOME');
$('go-park').onclick = poseCmd('PARK');
$('power-on').onclick = () => {
  if (serverMode()) powerOn();
  else { ctl.rearm(); lastContacts = new Set(world.contacts.map(k => `${k.a} × ${k.b}`)); }
};
$('estop').onclick = async () => {
  demo.stop();
  simArm.abort = true;
  if (link) link.abort = true;
  ctl.stop('emergency stop button');
  if (link) { try { await link.emergencyStop(); log('STOP sent to the active backend', 'err'); } catch (e) { log(`stop failed: ${e.message}`, 'err'); } }
};

// ---------------- task panel ----------------

let current = null; // { plan, report, moves, start, mode }

const startCounts = () => (serverMode() ? LiveLink.counts(link.state) : ctl.commands.slice());

function describe(report) {
  const parts = [];
  parts.push(report.ok ? '<span class="ok">✓ checked in simulation</span>' : '<span class="bad">✗ does not work in simulation</span>');
  parts.push(`${fmt(report.duration, 0)} s`);
  if (report.contacts.length) parts.push(`<span class="bad">${report.contacts.map(c => `${c.a} × ${c.b} (${fmt(-c.sep * 10, 1)} mm, at "${c.step}")`).join('; ')}</span>`);
  const warns = report.events.filter(e => e.kind === 'warn').map(e => e.text);
  if (warns.length) parts.push(`<span class="warn">${warns.join('; ')}</span>`);
  if (!report.contacts.length && !report.ok) parts.push(`<span class="bad">cube ends ${fmt(report.offset, 2)} cm from the target</span>`);
  return parts.join(' · ');
}

function renderPlan() {
  const { plan, report, mode, notes } = current;
  const grasp = report.events.find(e => e.kind === 'grasp');
  const rows = plan.steps.map((s, i) => `<tr data-i="${i}"><td>${s.label}</td>${s.counts.map(c => `<td>${c}</td>`).join('')}</tr>`).join('');
  $('plan-out').innerHTML = `<div>${plan.steps.length} poses, hover ${plan.info.hover} cm${plan.info.vias ? `, ${plan.info.vias} transit via${plan.info.vias > 1 ? 's' : ''} around stacks` : ''}, ${mode === 'moves' ? `${current.moves.length} single-joint moves` : 'demo playback'}.</div>
    <div>${describe(report)}</div>
    ${grasp ? `<div class="note">${grasp.text}</div>` : ''}
    ${notes?.length ? `<div class="warn">${notes.join('; ')}</div>` : ''}
    <div class="steps"><table><tr><th>Step</th>${SERVOS.map(s => `<th>${s.name}</th>`).join('')}</tr>${rows}</table></div>`;
  $('run-sim').disabled = false;
  $('export-plan').disabled = false;
  updateRealReason();
}

function makePlan() {
  try {
    if (serverMode()) $('exec-mode').value = 'moves';   // the robot API moves one joint at a time
    const mode = $('exec-mode').value;
    const speed = Math.max(1, Math.min(100, Number($('demo-speed').value) || 20));
    const start = startCounts();
    const parsed = parseCommand($('cmd').value, world);
    const plan = planTransfer(world, parsed, start, { home: ctl.poses.HOME, step: mode === 'moves' ? 0.25 : PLAN.descentStep });
    let moves = null, notes = null;
    if (mode === 'moves') ({ moves, notes } = toMoves(world, plan, start));
    const report = verify(world, plan, start, { mode, speed, moves });
    current = { plan, report, moves, notes, start, mode, speed, sim: 'none' };   // sim: none | running | done | failed
    renderPlan();
    showPath(plan.steps, start);
    log(`plan "${$('cmd').value}": ${plan.steps.length} poses, ${report.ok ? 'OK' : 'FAILS'} in sim`, report.ok ? 'ok' : 'err');
  } catch (e) {
    current = null;
    $('plan-out').innerHTML = `<span class="bad">${e.message}</span>`;
    $('run-sim').disabled = $('export-plan').disabled = true;
    updateRealReason();
    if (!(e instanceof PlanError)) console.error(e);
  }
}
$('plan-btn').onclick = makePlan;
$('cmd').addEventListener('keydown', e => { if (e.key === 'Enter') makePlan(); });
$('exec-mode').onchange = () => { if (current) makePlan(); };

$('run-sim').onclick = async () => {
  if (!current) return;
  try {
    if (!serverMode()) {   // local simulation only
      if (ctl.commands.some((c, i) => c !== current.start[i])) { makePlan(); if (!current) return; }
      world.events = [];
      if (current.mode === 'moves') ctl.runMoves(current.moves, current.moves.map(m => m.label));
      else ctl.runPoses(current.plan.steps.map(s => s.counts), current.speed, current.plan.steps.map(s => s.label));
      return;
    }
    if (LiveLink.counts(link.state).some((c, i) => c !== current.start[i])) { makePlan(); if (!current) return; }
    if (!current.report.ok) { toast('The plan fails in the simulation; not sent'); return; }
    const calls = movesToCalls(current.moves);
    if (onReal()) {
      let at = current.start.slice();
      const rows = current.moves.map(m => { at = at.slice(); at[m.ch] = m.target; return [m.label, ...at]; });
      if (!await confirmReal(`Run "${$('cmd').value}" on the real arm?`, `${calls.length} single-joint moves, computed and checked in the simulation.`, [['now', ...current.start], ...rows])) return;
    }
    await send(calls);
    log(`plan "${$('cmd').value}" done on the ${onReal() ? 'real arm' : 'shared simulation'}`, 'ok');
  } catch (e) { toast(e.message); log(e.message, 'err'); }
};

$('export-plan').onclick = async () => {
  if (!current) return;
  const json = JSON.stringify({ name: $('cmd').value, units: 'PCA9685_counts', nominal_frequency_hz: 60,
    positions: current.plan.steps.map(s => ({ name: s.label, counts: Object.fromEntries(s.counts.map((c, i) => [String(i), c])) })) }, null, 1);
  try { await navigator.clipboard.writeText(json); toast('Plan copied as demo JSON'); } catch { console.log(json); toast('Clipboard blocked; JSON printed to the console'); }
};

$('goto-xyz').onclick = () => {
  try {
    const s5 = anglesFromCounts(ctl.commands).s5;
    const a = ikTopDown(Number($('gx').value), Number($('gy').value), Number($('gz').value), { s5Near: s5 });
    const c = countsFromAngles(a, ctl.commands[5]);
    const bad = guardViolations(c); if (bad.length) throw new Error(bad.join(', '));
    if (!serverMode()) { ctl.runPoses([c], Math.max(1, Number($('demo-speed').value) || 20), ['go to x y z']); return; }
    const start = LiveLink.counts(link.state);
    send(movesToCalls(toMoves(world, { steps: [{ label: 'go to x y z', counts: c }] }, start).moves)).catch(e => toast(e.message));
  } catch (e) { toast(e instanceof Unreachable ? `Unreachable: ${e.message}` : e.message); }
};

// the team's hand-taught demo (Tools)
$('taught-demo').onclick = () => {
  if (serverMode()) { $('scn-select').value = 'taught'; $('scn-select').dispatchEvent(new Event('change')); toast('Scenario "taught pick and place" selected: ↺ Reset, then ▶ Run'); return; }
  try { ctl.demo_load(TAUGHT.name, TAUGHT.positions); ctl.settings.demo_speed = Number($('demo-speed').value) || 40; ctl.demo_run(); showPath(TAUGHT.positions, ctl.commands.slice()); }
  catch (e) { toast(e.message); }
};

// ---------------- demonstration (demo.js) ----------------

const demo = initDemo({
  ctl, world, $, log, toast, renderCubes, showPath, moveIsClear, liveControl: () => liveControl(),
  setPathVisible: on => { pathLine.visible = pathDots.visible = on; needsRender = true; },
  resetView: () => { leaveOakView(); camera.position.set(-34, 44, 30); controls.target.set(0, 11, 6); controls.update(); },
});

// ---------------- scene panel ----------------

let placing = null;
function renderCubes() {
  $('cubes').innerHTML = '';
  for (const c of world.cubes) {
    const row = document.createElement('div');
    row.className = 'cube-row';
    const lvl = world.levelOf(c);
    row.innerHTML = `<span class="dot" style="background:#${new THREE.Color(COLOURS[c.colour] ?? 0x999999).getHexString()}"></span>
      <span><b>${c.id}</b> <span class="pos">x ${fmt(c.p[0])} · y ${fmt(c.p[1])} · ${c.state === 'held' ? 'in gripper' : c.p[2] < -0.2 ? 'off the board' : cubeTilt(c) > 20 ? 'toppled' : `level ${lvl}`}${c.state === 'awake' ? ' · moving' : ''}</span></span>
      <button title="Click on the mat to drop it there" class="${placing === c.id ? 'on' : ''}">⌖</button><button title="Remove">×</button>`;
    const [pick, rm] = row.querySelectorAll('button');
    pick.onclick = () => { placing = placing === c.id ? null : c.id; renderCubes(); };
    rm.onclick = () => {
      if (serverMode()) { setScene(sceneOf(c.id)).catch(e => toast(e.message)); return; }
      if (world.holding?.id === c.id) world.holding = null; world.cubes = world.cubes.filter(k => k !== c); world.dirty = true; renderCubes(); needsRender = true;
    };
    $('cubes').append(row);
  }
}
$('add-cube').onclick = () => {
  const colour = $('new-colour').value;
  if (serverMode()) { setScene([...sceneOf(), [colour, Number($('nx').value), Number($('ny').value), 0, 0]]).catch(e => toast(e.message)); return; }
  let id = colour, n = 2;
  while (world.cube(id)) id = `${colour}${n++}`;
  world.addCube(id, colour, Number($('nx').value), Number($('ny').value), 0, 0);
  world.cube(id).p[2] = 30;
  renderCubes();
};
$('reset-scene').onclick = () => {
  if (serverMode()) { setScene(DEFAULT_SCENE).catch(e => toast(e.message)); return; }
  resetScene(); renderCubes(); needsRender = true;
};

const ray = new THREE.Raycaster();
let downAt = null;
canvas.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', e => {
  if (!placing || !downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  const r = canvas.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), camera);
  const hit = new THREE.Vector3();
  if (!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit)) return;
  const c = world.cube(placing);
  if (c && c.state !== 'held' && serverMode()) setScene([...sceneOf(c.id), [c.colour, hit.x, hit.y, 0, cubeYaw(c), c.id]]).catch(e => toast(e.message));
  else if (c && c.state !== 'held') { c.p = [hit.x, hit.y, 12]; c.v = [0, 0, 0]; c.w = [0.4, 0.3, 0]; c.state = 'awake'; }
  placing = null; renderCubes();
});

// ---------------- arm server: one robot API, switchable backend ----------------
//
// The page is a client of server.mjs like any other program: every command is a call of the robot's
// low-level API (/api/move, /api/go_pose, ...), answered by the backend that is active for everybody:
// the shared simulated arm, or the real arm (each motion call simulated first on the server and
// blocked if it collides). Header switch = POST /api/backend. The 3D view mirrors the server: arm
// pose from GET /api/state, cubes and grasp events from GET /sim/world. Without server.mjs (page
// served elsewhere) the page falls back to its own simulation.

let link = null;
let backend = { backend: 'none' };
let scenarios = null;   // scenarios.js, set up further down
const mirror = { position: null, phase: null, backend: 'sim', seen: new Set(), off: false };
const serverMode = () => backend.backend === 'arm-server' && !!link?.state;
/** The view shows the server's arm and cubes (not while the local physics show runs). */
const mirroring = () => serverMode() && !demo.running;
const onReal = () => serverMode() && mirror.backend === 'real';
/** Kept for demo.js / scenarios.js: true when commands go to the real arm. */
const liveControl = onReal;

function connect() {
  if (link) return;
  link = new LiveLink('');
  link.on((state, err) => {
    if (err) { $('live-status').textContent = `error: ${err}`; updateLiveUi(); return; }
    const c = LiveLink.counts(state);
    mirror.position = state.position || c; mirror.phase = state.phase; mirror.backend = state.backend || 'real';
    if (state.outputs_off && !mirror.off) { log(`${mirror.backend === 'real' ? 'real arm' : 'simulation'} stopped: ${state.error || 'outputs off'}`, 'err'); toast(`Stopped: ${state.error || 'outputs off'}. "Re-arm" powers it on (PARK → HOME).`); }
    mirror.off = !!state.outputs_off;
    if (state.joints) state.joints.forEach((j, i) => { ctl.poses.HOME[i] = j.home; ctl.poses.PARK[i] = j.park; });
    if (mirroring()) { ctl.commands = c.slice(); ctl.outputsOff = !!state.outputs_off; ctl.armed = !state.outputs_off; ctl.error = state.error || null; }
    $('live-status').textContent = `${mirror.backend === 'real' ? `real arm (${state.mode})` : 'shared simulation'}, ${state.outputs_off ? 'outputs off' : 'armed'}${state.busy ? ', moving' : ''}`;
    updateLiveUi(); needsRender = true;
  });
  link.start(100);   // same machine: cheap; the server reads the real controller only when the real backend is active
}

/** Cubes, gripper and events of the server's simulation into the local world (no local physics). */
function applyWorld(ws) {
  const ids = new Set(ws.cubes.map(c => c.id));
  world.cubes = world.cubes.filter(c => ids.has(c.id));
  for (const s of ws.cubes) {
    const c = world.cube(s.id) || world.addCube(s.id, s.colour, s.p[0], s.p[1], 0, 0);
    c.p = s.p; c.R = s.R; c.state = s.state; c.v = [0, 0, 0]; c.w = [0, 0, 0];
  }
  world.holding = ws.holding ? { id: ws.holding, width: ws.gap } : null;
  world.gap = ws.gap;
  for (const e of ws.events || []) {
    const key = `${e.t}|${e.text}`;
    if (mirror.seen.has(key)) continue;
    mirror.seen.add(key); if (mirror.seen.size > 200) mirror.seen.delete(mirror.seen.values().next().value);
    log(e.text, e.kind === 'warn' ? 'warn' : e.kind === 'grasp' ? 'ok' : '');
  }
  needsRender = true;
}
(async function pollWorld() {
  if (mirroring()) {
    try { const r = await fetch('/sim/world', { cache: 'no-store' }); if (r.ok) applyWorld(await r.json()); } catch { /* next time */ }
  }
  setTimeout(pollWorld, 100);
})();

/** Replace the cubes of the shared simulation: [[colour, x, y, level, yaw, id], ...]. */
async function setScene(cubes) {
  if (!serverMode()) {
    world.cubes = []; world.holding = null;
    for (const [c, x, y, l = 0, yaw = 0, id] of cubes) world.addCube(id || c, c, x, y, l, yaw);
    world.dirty = true; renderCubes(); needsRender = true; return;
  }
  const r = await fetch('/sim/scene', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ cubes }) });
  const j = await r.json();
  if (!r.ok) throw new Error(j.error);
  applyWorld(j); renderCubes();
}
/** The current cubes as a scene list (held cubes stay out). */
const sceneOf = (except = null) => world.cubes.filter(c => c.state !== 'held' && c.id !== except)
  .map(c => [c.colour, c.p[0], c.p[1], Math.max(0, world.levelOf(c)), cubeYaw(c), c.id]);

/** Run API calls on the active backend; a blocked call (409) stops the rest. */
async function send(calls) {
  const hud = (i, c) => { $('hud').textContent = `${calls.length > 1 ? `${i + 1}/${calls.length} ` : ''}${c.label}`; };
  if (!serverMode()) return runCalls(simArm, calls, { onStep: hud });
  if (onReal()) log(`real arm: ${calls.length === 1 ? calls[0].label : `${calls.length} calls`}`, 'warn');
  return runCalls(link, calls, { onStep: hud });
}

async function setTarget(target) {
  try {
    const r = await fetch('/api/backend', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ target }) });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error);
    Object.assign(backend, j);
    await link?.read();
    log(`backend: ${target === 'real' ? `REAL ARM ${j.real}` : 'shared simulation'}`, target === 'real' ? 'warn' : '');
  } catch (e) { toast(e.message); log(e.message, 'err'); }
  updateLiveUi();
}

function updateLiveUi() {
  const real = onReal(), off = !!link?.state?.outputs_off;
  $('target-sim').classList.toggle('on', !real);
  $('target-real').classList.toggle('on', real);
  $('target-real').disabled = !backend.realAvailable;
  $('target-real').title = backend.realAvailable ? `Switch the API to the real arm (${backend.real})` : 'No real arm: start server.mjs with --controller <url>';
  const pill = $('link-pill');
  pill.className = 'pill ' + (real ? 'live' : serverMode() ? 'view' : '');
  pill.textContent = !serverMode() ? (link?.error ? 'server: connection error' : 'local simulation only')
    : real ? `REAL ARM · ${backend.real}${backend.allowMotion ? '' : ' · read only'}` : 'shared simulation';
  $('live-banner').hidden = !real;
  $('live-banner').className = 'live-banner live';
  $('live-banner-text').textContent = `REAL ARM (${backend.real}): every motion call is simulated first and blocked if it collides.${backend.allowMotion ? '' : ' Read only: start the server with --allow-motion to move it.'}${off ? ' Its outputs are off.' : ''}`;
  $('live-power').hidden = !(real && off);
  document.body.classList.toggle('armed', real);
  $('joints-target').textContent = real ? '→ REAL ARM' : serverMode() ? '→ shared simulation' : '→ simulation';
  $('joints-target').classList.toggle('live', real);
  $('run-sim').textContent = real ? '▶ Run on REAL arm' : '▶ Run';
  $('run-sim').classList.toggle('danger', real);
  $('dlg-backend').innerHTML = !serverMode() ? 'The page is not connected to <code>server.mjs</code>; it runs its own simulation.'
    : `One robot API for everybody at <code>${location.origin}/api</code> (the controller's low-level API). Active backend: <b>${real ? `real arm ${backend.real}` : 'shared simulation'}</b>.`
      + (backend.realAvailable ? ` Real arm: ${backend.real}${backend.allowMotion ? ', motion allowed after the simulation check.' : ', read only (<code>--allow-motion</code> not given).'}` : ' No real arm configured (<code>--controller &lt;url&gt;</code>).');
  updateRealReason();
  scenarios?.render();
}

/** The line under the task buttons. */
function updateRealReason() {
  $('real-reason').textContent = onReal() ? 'Runs on the real arm: the planner computed it in the simulation, and the server simulates every call again before sending it.' : '';
}

/** Short confirmation before a multi-step run goes to the real arm. rows: [[label, ...counts]]. */
function confirmReal(title, html, rows = []) {
  const dlg = $('send-dlg');
  $('send-title').textContent = title;
  const table = rows.length ? `<div class="moves"><table><tr><th>Step</th>${SERVOS.map(s => `<th>${s.name}</th>`).join('')}</tr>${rows.map(r => `<tr>${r.map(c => `<td>${c}</td>`).join('')}</tr>`).join('')}</table></div>` : '';
  $('send-body').innerHTML = `<p>${html}</p>${table}`;
  dlg.returnValue = '';
  dlg.showModal();
  return new Promise(resolve => dlg.addEventListener('close', () => resolve(dlg.returnValue === 'go' && onReal()), { once: true }));
}

const powerOn = async () => { try { await send([{ op: 'power_on', label: 'power on (PARK → HOME)' }]); } catch (e) { toast(e.message); log(e.message, 'err'); } };
$('target-sim').onclick = () => setTarget('sim');
$('target-real').onclick = () => setTarget('real');
$('live-off').onclick = () => setTarget('sim');
$('live-power').onclick = powerOn;
$('link-pill').onclick = () => { updateLiveUi(); $('real-dlg').showModal(); };
$('open-real').onclick = () => { updateLiveUi(); $('real-dlg').showModal(); };
$('show-ghost').onchange = () => { needsRender = true; };

(async () => {
  // a page served by the controller itself has no /local/info (404); a network error is retried
  for (let i = 0; i < 8; i++) {
    try {
      const r = await fetch('/local/info', { cache: 'no-store' });
      backend = r.ok ? await r.json() : { backend: 'controller' };
      break;
    } catch (e) { console.warn('/local/info:', e.message); await new Promise(res => setTimeout(res, 500 * (i + 1))); }
  }
  if (backend.backend === 'arm-server') connect();
  updateLiveUi();
})();

// ---------------- calibration panel ----------------

function renderCal() {
  $('calib').innerHTML = '<tr><th></th><th>angle at HOME (°)</th><th>dir</th><th>counts/°</th></tr>' +
    SERVOS.slice(0, 5).map((s, i) => `<tr><td>${s.name}</td><td><input data-i="${i}" data-k="qHome" type="number" step="0.5" value="${s.qHome}"></td>
      <td><select data-i="${i}" data-k="dir"><option value="1"${s.dir > 0 ? ' selected' : ''}>+1</option><option value="-1"${s.dir < 0 ? ' selected' : ''}>−1</option></select></td>
      <td><input data-i="${i}" data-k="cpd" type="number" step="0.01" value="${s.cpd.toFixed(3)}"></td></tr>`).join('');
  $('calib').querySelectorAll('[data-k]').forEach(el => el.onchange = () => { SERVOS[el.dataset.i][el.dataset.k] = Number(el.value); world.dirty = true; needsRender = true; current = null; });
  $('mat-d').value = MAT.sToEdge;
}
$('mat-d').onchange = () => { MAT.sToEdge = Number($('mat-d').value); placeMatAndCamera(); needsRender = true; };
$('calib-save').onclick = () => { try { localStorage.setItem(CAL_KEY, JSON.stringify(currentCal())); toast('Calibration saved in this browser'); } catch { toast('Browser storage unavailable'); } };
$('calib-reset').onclick = () => { applyCal(structuredClone(DEFAULT_CAL)); renderCal(); placeMatAndCamera(); needsRender = true; };
$('calib-export').onclick = async () => { const j = JSON.stringify(currentCal(), null, 1); try { await navigator.clipboard.writeText(j); toast('Calibration copied'); } catch { console.log(j); } };
renderCal();

// ---------------- tabs and camera tab ----------------

document.querySelectorAll('[data-tab]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-tab]').forEach(x => x.classList.toggle('on', x === b));
  $('tab-control').hidden = b.dataset.tab !== 'control';
  $('tab-camera').hidden = b.dataset.tab !== 'camera';
  if (b.dataset.tab === 'camera') { camTab.activate(); camTab.update(); }
});
/** Replace the sim cubes with the camera's detections. */
function placeDetections(list) {
  if (serverMode()) {
    setScene(list.map(d => { const [x, y] = tableToBoard([d.X, d.Y, 0]); return [d.colour, x, y, 0, d.yaw]; }))
      .then(() => log(`${list.length} cubes from the camera put into the shared simulation`, 'ok')).catch(e => toast(e.message));
    return;
  }
  world.cubes = []; world.holding = null;
  for (const d of list) {
    const [x, y] = tableToBoard([d.X, d.Y, 0]);
    let id = d.colour, n = 2; while (world.cube(id)) id = `${d.colour}${n++}`;
    world.addCube(id, d.colour, x, y, 0, d.yaw);
  }
  world.dirty = true; renderCubes(); needsRender = true;
  log(`${list.length} cubes from the camera put on the mat`, 'ok');
}
const camTab = initCameraTab({
  scene, cals: camCals, hideForCamera, log, toast,
  armAngles: () => anglesFromCounts(ctl.commands),
  // every tracking round: the fit (or null when this round found too few tags) and the tags each camera saw
  onArmTrack: (r, seen, tagErr = []) => {
    if (r) tracked = { angles: r.angles, rms: r.rms, views: r.residuals.length, free: r.free, seen, tagErr, t: Date.now(), miss: false };
    else if (tracked) Object.assign(tracked, { seen, tagErr: [], miss: true });
    needsRender = true;
  },
  clearArmTrack: () => { tracked = null; needsRender = true; },
  onLiveStats: s => { liveStats = s; const m = s.match(/: (\d+) ms/); liveRound = m ? `round ${(m[1] / 1000).toFixed(1).replace('.', ',')} s` : ''; },
  onLiveCubes: list => { liveCubes = list; needsRender = true; },
  onGap: g => { liveGapInfo = g; needsRender = true; },
  onRecalibrated: () => { placeMatAndCamera(); needsRender = true; },
  onLive: on => { if (!on) { liveCubes = null; liveGapInfo = null; } $('live-track').classList.toggle('on', on); $('live-track').textContent = on ? '● Live: stop' : '● Live tracking'; needsRender = true; },
  // the next fit starts where the last one found the arm (the joint panel can be far from the real arm)
  trackStart: () => tracked?.angles,
  linkBase: () => (link ? link.base : ''),
  onApply: (cal, id = 'oak') => {
    camCals[id] = cal;
    if (id === 'oak') CAMERA.calib = cal;
    try { localStorage.setItem(CAM_KEY(id), JSON.stringify({ R: cal.R, t: cal.t, f: cal.f, cx: cal.cx, cy: cal.cy, centre: cal.centre, rms: cal.rms, H: cal.H })); } catch { /* not saved */ }
    placeMatAndCamera(); if (oakView) applyOakView(); needsRender = true;
  },
  useDetections: list => placeDetections(list),
  // cubes located in 3D by two cameras (locate3d.js): placed at their stack level
  useCubes3d: list => {
    setScene(list.map(c => [c.colour, c.board[0], c.board[1], c.level, c.yaw || 0]))
      .then(() => log(`${list.length} cubes from both cameras put ${serverMode() ? 'into the shared simulation' : 'on the mat'}, with their stack levels`, 'ok'))
      .catch(e => toast(e.message));
  },
  trackCubes3d: (() => {
    let last = 0;
    return list => { if (Date.now() - last < 1500) return; last = Date.now(); setScene(list.filter(c => Math.abs(c.X[2] - (c.level * CUBE * 10 + CUBE * 5)) < 6).map(c => [c.colour, c.board[0], c.board[1], c.level, c.yaw || 0])).catch(() => {}); };
  })(),
  // Live tracking: move the nearest sim cube of the same colour to each detection (within 6 cm,
  // and only if it moved more than 3 mm, so camera noise does not keep the physics awake);
  // add cubes the sim does not have yet. Cubes in the gripper are left alone.
  trackDetections: list => {
    let changed = false;
    for (const d of list) {
      const [x, y] = tableToBoard([d.X, d.Y, 0]);
      const same = world.cubes.filter(c => c.colour === d.colour && c.state !== 'held');
      const near = same.sort((a, b) => Math.hypot(a.p[0] - x, a.p[1] - y) - Math.hypot(b.p[0] - x, b.p[1] - y))[0];
      if (near && Math.hypot(near.p[0] - x, near.p[1] - y) < 6) {
        if (Math.hypot(near.p[0] - x, near.p[1] - y) < 0.3) continue;
        near.p = [x, y, CUBE / 2]; near.v = [0, 0, 0]; near.w = [0, 0, 0];
        near.R = [Math.cos(d.yaw * Math.PI / 180), -Math.sin(d.yaw * Math.PI / 180), 0, Math.sin(d.yaw * Math.PI / 180), Math.cos(d.yaw * Math.PI / 180), 0, 0, 0, 1];
        near.state = 'awake'; changed = true;
      } else {
        let id = d.colour, n = 2; while (world.cube(id)) id = `${d.colour}${n++}`;
        world.addCube(id, d.colour, x, y, 0, d.yaw); changed = true;
      }
    }
    if (changed) { renderCubes(); needsRender = true; if (serverMode()) setScene(sceneOf()).catch(() => {}); }
  },
});
// one click: both cameras live, the arm tracked and drawn where it really is (Camera tab: startLive)
$('live-track').onclick = () => (camTab.live ? camTab.stopLive() : camTab.startLive());

// ---------------- scenarios (scenarios.js) ----------------

scenarios = initScenarios({
  ctl, world, $, log, toast, renderCubes, showPath, confirmReal, demo, send, setScene, serverMode, onReal,
  counts: () => (serverMode() ? LiveLink.counts(link.state) : ctl.commands.slice()),
  idle: () => (serverMode() ? !link.state.busy && world.settled() : !ctl.busy && world.settled()),
  detections: () => camTab.detections, useDetections: placeDetections,
  stopCalls: () => { if (link) link.abort = true; simArm.abort = true; ctl.stop('scenario stopped'); },
});
$('demo-reset').onclick = () => scenarios.reset();
$('sync-cams').onclick = async () => {
  $('sync-cams').disabled = true;
  try { const msg = await camTab.syncCubes(); toast(msg); log(`sync from the cameras: ${msg}`, 'ok'); }
  catch (e) { toast(e.message); log(e.message, 'err'); }
  $('sync-cams').disabled = false;
};

// ---------------- main loop ----------------

// the real arm as the cameras see it (Camera tab → Arm tracking); while it is followed, the arm model is drawn there
let tracked = null, liveStats = '', liveRound = '';
// the cubes the cameras see right now (live tracking, js/cubetrack.js), at their real height: on the mat, stacked,
// in the gripper or in the air. While they are shown, the simulation's own cubes are hidden (one set of cubes).
const liveCubeGroup = new THREE.Group(); liveCubeGroup.visible = false; scene.add(liveCubeGroup);
const liveCubeItems = [];   // { colour, mesh, txt, key }
let liveCubes = null, liveGapInfo = null;   // the jaw opening seen by the cameras: { gap, source, holding }
function showLiveCubes() {
  const on = !!liveCubes && camTab.live;
  liveCubeGroup.visible = on;
  for (const m of cubeMeshes.values()) m.visible = !on;
  if (!on) return;
  liveCubes.forEach((c, i) => {
    let it = liveCubeItems[i];
    if (!it || it.colour !== c.colour) {
      if (it) { liveCubeGroup.remove(it.mesh); if (it.txt) liveCubeGroup.remove(it.txt); }
      it = liveCubeItems[i] = { colour: c.colour, mesh: buildCube(c.colour), txt: null, key: '' };
      liveCubeGroup.add(it.mesh);
    }
    it.mesh.visible = true;
    it.mesh.position.set(...(c.shown || c.board));
    it.mesh.rotation.set(0, 0, (c.yaw || 0) * Math.PI / 180);
    const key = cubeText(c);
    if (key !== it.key) {                                    // labels are textures: rebuilt only when the text changes
      if (it.txt) { liveCubeGroup.remove(it.txt); it.txt.material.map.dispose(); it.txt.material.dispose(); }
      it.txt = label(key, c.colour); it.txt.scale.multiplyScalar(0.8); it.txt.frustumCulled = false; liveCubeGroup.add(it.txt);
      it.key = key;
    }
    it.txt.visible = true;
    const p = c.shown || c.board; it.txt.position.set(p[0] + 1.6, p[1], p[2] + 1.6);
  });
  for (let i = liveCubes.length; i < liveCubeItems.length; i++) { liveCubeItems[i].mesh.visible = false; if (liveCubeItems[i].txt) liveCubeItems[i].txt.visible = false; }
}

const following = () => !!tracked && $('cam-arm-ghost').checked;
$('cam-arm-ghost').addEventListener('change', () => { needsRender = true; });
setInterval(() => { if (following()) needsRender = true; }, 1000);   // keeps the overlay's age current
// ---- large readout while the arm is tracked: grip point in mat coordinates, jaw axis, tilt, opening, joints, cubes;
// "Remember" adds the current values to a list (kept in this browser)
const MEM_KEY = 'pxsim.remembered.v1';
let remembered = [];
try { remembered = JSON.parse(localStorage.getItem(MEM_KEY)) || []; } catch { remembered = []; }
const saveMem = () => { try { localStorage.setItem(MEM_KEY, JSON.stringify(remembered)); } catch { /* kept for this page only */ } };
const toMat = p => [(p[1] - MAT.sToEdge) * 10, -p[0] * 10, p[2] * 10];          // board cm -> mat (table) mm
const n0 = v => Math.round(v).toString().replace('-', '−'), n1 = v => v.toFixed(1).replace('.', ',').replace('-', '−');
function readoutNow() {
  if (!following()) return null;
  const f = fk(tracked.angles), x = [f.hand.R[0], f.hand.R[3], f.hand.R[6]];
  let jaw = Math.atan2(-x[0], x[1]) * 180 / Math.PI; jaw = ((jaw + 90) % 180 + 180) % 180 - 90;   // jaw axis in the mat frame, folded
  return {
    time: new Date().toLocaleTimeString(),
    grip: toMat(f.grasp).map(v => Math.round(v)), jaw: Math.round(jaw), tilt: Math.round(toolYawAndTilt(f.hand).tilt),
    open: liveGapInfo?.gap != null ? +liveGapInfo.gap.toFixed(1) : null,
    joints: Object.fromEntries(['s1', 's2', 's3', 's4', 's5'].map(j => [j, +tracked.angles[j].toFixed(1)])),
    cubes: (liveCubes || []).map(c => { const m = toMat(c.shown || c.board); return { colour: c.colour, at: m.map(v => Math.round(v)), yaw: Math.round(c.yaw || 0), where: c.where }; }),
  };
}
let readoutKey = '', showDetails = false;   // the detailed tracking box (bottom right) only on request
function liveReadout() {
  const box = $('live-readout'), r = readoutNow();
  box.hidden = !r;
  if (!r) return;
  const key = JSON.stringify({ ...r, time: 0 }) + remembered.length + showDetails + liveRound + tracked.rms.toFixed(1) + JSON.stringify(tracked.seen) + tracked.miss;
  if (key === readoutKey) return;                                       // redraw only when something changed
  readoutKey = key;
  const cube = c => `<div class="ro-cube"><span class="chip" style="background:${c.colour}"></span>${c.colour} <b>X ${n0(c.at[0])} · Y ${n0(c.at[1])} · Z ${n0(c.at[2])}</b> mm · ${n0(c.yaw)}° · ${{ mat: 'on the mat', desk: 'on the desk', stack: 'stacked', gripper: 'in the gripper', air: 'in the air' }[c.where]}</div>`;
  box.innerHTML = `<div class="ro-head"><span>${camTab.live ? 'LIVE · ' : ''}grip point, mat mm</span><span><button data-ro="details" title="The detailed tracking box: errors per tag, round timing, tracked minus joint panel">${showDetails ? 'Hide details' : 'Details'}</button> <button data-ro="remember" title="Add the current values to the list below">★ Remember</button></span></div>
    <div class="ro-big">X <b>${n0(r.grip[0])}</b> Y <b>${n0(r.grip[1])}</b> Z <b>${n0(r.grip[2])}</b></div>
    <div class="ro-mid">jaw axis <b>${n0(r.jaw)}°</b> · tilt <b>${n0(r.tilt)}°</b>${r.open != null ? ` · open <b>${n1(r.open)} cm</b>` : ''}</div>
    <div class="ro-small">${tracked.seen.map(c => `${c.name} ${c.ids.join(' ') || '–'}`).join(' · ')} · fit ${n1(tracked.rms)} px${tracked.miss ? ' · <span class="miss">last round: too few tags</span>' : ''}${liveRound ? ` · ${liveRound}` : ''}</div>
    <div class="ro-small">${Object.entries(r.joints).map(([j, v]) => `${j.toUpperCase()} ${n1(v)}°`).join(' · ')}</div>
    ${r.cubes.map(cube).join('')}
    ${remembered.length ? `<div class="ro-list-head"><span>Remembered (${remembered.length})</span><span><button data-ro="copy" title="Copy the list as CSV">Copy</button> <button data-ro="clear">Clear</button></span></div>
    <div class="ro-list">${remembered.slice().reverse().map((m, i) => `<div><b>${remembered.length - i}</b> ${m.time} · X ${n0(m.grip[0])} Y ${n0(m.grip[1])} Z ${n0(m.grip[2])} · ${n0(m.jaw)}°${m.open != null ? ` · ${n1(m.open)} cm` : ''}${m.cubes.map(c => ` · ${c.colour} ${c.at.map(n0).join('/')} ${n0(c.yaw)}°`).join('')}</div>`).join('')}</div>` : ''}`;
}
$('live-readout').addEventListener('click', e => {
  const what = e.target.closest('[data-ro]')?.dataset.ro;
  if (what === 'remember') { const r = readoutNow(); if (r) { remembered.push(r); saveMem(); log(`remembered: grip X ${r.grip[0]} Y ${r.grip[1]} Z ${r.grip[2]} mm, jaw axis ${r.jaw}°`, 'ok'); } }
  if (what === 'clear') { remembered = []; saveMem(); }
  if (what === 'details') showDetails = !showDetails;
  if (what === 'copy') {
    const rows = [['#', 'time', 'grip X mm', 'grip Y mm', 'grip Z mm', 'jaw axis deg', 'tilt deg', 'open cm', 'S1', 'S2', 'S3', 'S4', 'S5', 'cubes']]
      .concat(remembered.map((m, i) => [i + 1, m.time, ...m.grip, m.jaw, m.tilt, m.open ?? '', ...['s1', 's2', 's3', 's4', 's5'].map(j => m.joints[j]), m.cubes.map(c => `${c.colour} ${c.at.join('/')} ${c.yaw}deg ${c.where}`).join('; ')]));
    const text = rows.map(r => r.join('\t')).join('\n');
    try { navigator.clipboard.writeText(text); toast('List copied (tab separated, pastes into a spreadsheet)'); } catch { toast('Copy failed'); }
  }
  readoutKey = ''; needsRender = true;
});

function trackHud() {
  const box = $('track-hud');
  box.hidden = !following() || !showDetails;
  if (box.hidden) return;
  const T = tracked, a0 = anglesFromCounts(ctl.position), s = v => (v >= 0 ? '+' : '') + v.toFixed(1);
  const ago = (Date.now() - T.t) / 1000;
  const perTag = {}; for (const x of T.tagErr || []) (perTag[x.id] ??= []).push(`${x.cam} ${x.err.toFixed(1)} px`);
  box.innerHTML = `<b>${camTab.live ? 'LIVE · ' : ''}Arm drawn where the cameras see it</b><br>` +
    Object.entries(perTag).map(([id, l]) => `tag ${id}: ${l.join(' · ')}`).join('<br>') + (Object.keys(perTag).length ? '<br>' : '') +
    T.seen.map(c => `${c.name}: ${c.ids.length ? c.ids.length + ' tag' + (c.ids.length > 1 ? 's' : '') : 'no tags'}`).join(' · ') + '<br>' +
    `fit ${T.rms.toFixed(1)} px over ${T.views} tag view${T.views === 1 ? '' : 's'} · ${ago < 2 ? 'now' : `${ago.toFixed(0)} s ago`}` +
    (T.miss ? ' · <span class="miss">last round: too few tags, pose kept</span>' : '') + '<br>' +
    (camTab.live && liveGapInfo?.gap != null ? `gripper: ${liveGapInfo.gap.toFixed(1).replace('.', ',')} cm open (${liveGapInfo.source}${liveGapInfo.holding ? ', holds a cube' : ''})<br>` : '') +
    (camTab.live && liveCubes ? `cubes: ${liveCubes.length ? liveCubes.map(cubeText).join(' · ') : 'none seen by two cameras'}${liveCubes.onArm?.length ? ` (${liveCubes.onArm.length} coloured blob${liveCubes.onArm.length > 1 ? 's' : ''} on the arm left out)` : ''}<br>` : '') +
    (camTab.live && liveStats ? `<span style="color:var(--muted)">${liveStats}</span><br>` : '') + (onReal() ? 'tracked − real arm counts: ' : 'tracked − joint panel (simulation, not the real arm): ') + ['s1', 's2', 's3', 's4', 's5'].filter(j => T.free.includes(j)).map(j => `${j.toUpperCase()} ${s(T.angles[j] - a0[j])}°`).join(' ');
}
let needsRender = true, last = performance.now(), lastContacts = new Set(), eventIdx = 0, lastCubeKey = '';
controls.addEventListener('change', () => { needsRender = true; });
$('show-shapes').onchange = () => { needsRender = true; };
$('show-tags').onchange = () => { tagOverlay.group.visible = $('show-tags').checked; needsRender = true; };
$('show-path').onchange = () => { pathLine.visible = pathDots.visible = $('show-path').checked; needsRender = true; };

world.step(0, ctl.position);
renderCubes();

function frame(now) {
  const real = Math.min(0.05, (now - last) / 1000); last = now;
  const dt = real * Number($('time-scale').value);
  if (mirroring()) {
    // smooth the 10 Hz pose of the server's arm; cubes come from /sim/world (applyWorld)
    const tgt = mirror.position;
    if (tgt) {
      let moved = false;
      ctl.position = ctl.position.map((p, i) => { const q = p + (tgt[i] - p) * Math.min(1, real * 14); if (Math.abs(q - p) > 1e-3) moved = true; return q; });
      if (moved || needsRender) { world.frames = fk(anglesFromCounts(ctl.position)); needsRender = true; }
    }
  } else if (ctl.busy || !world.settled() || !!world.holding || world.dirty) {
    const n = Math.min(40, Math.max(1, Math.ceil(dt / (1 / 120))));
    for (let i = 0; i < n; i++) {
      ctl.update(dt / n);
      world.step(dt / n, ctl.position);
      // Stop on a new contact, like the real arm should be stopped.
      const keys = new Set(world.contacts.map(k => `${k.a} × ${k.b}`));
      // Cubes are physical now: the arm may push them (logged). Stop only for the arm hitting itself or the board.
      const fresh = [...keys].filter(k => !lastContacts.has(k) && !k.endsWith(' cube'));
      lastContacts = keys;
      if (fresh.length && ctl.busy && $('stop-on-contact').checked) { ctl.stop(`collision: ${fresh.join(', ')}`); toast(`Collision: ${fresh.join(', ')} — stopped`); break; }
    }
    world.dirty = false;
    needsRender = true;
  }
  // world events -> log
  for (; eventIdx < world.events.length; eventIdx++) { const e = world.events[eventIdx]; log(e.text, e.kind === 'warn' ? 'warn' : e.kind === 'grasp' ? 'ok' : ''); }
  if (eventIdx > world.events.length) eventIdx = world.events.length;

  if (!oakView && controls.update()) needsRender = true;
  if (needsRender) {
    // only one arm: where the cameras see it while tracking is followed, else the commanded (simulated) pose
    const a = following() ? tracked.angles : anglesFromCounts(ctl.position);
    arm.setPose(a, following() && liveGapInfo?.gap != null ? liveGapInfo.gap : world.gap);
    if (tagOverlay.group.visible) tagOverlay.update(a);
    ghost.root.visible = false;
    seenOverlay.group.visible = following();
    if (following()) { const seen = {}; for (const x of tracked.tagErr || []) (seen[x.id] ??= []).push(x); seenOverlay.update(a, seen); }
    showLiveCubes();
    trackHud();
    liveReadout();
    syncCubeMeshes();
    const hitNames = new Set(world.contacts.flatMap(k => [k.a, k.b]));
    updateShapes(armShapes(world.frames, world.gap), hitNames, $('show-shapes').checked);
    $('contacts').hidden = !world.contacts.length;
    $('contacts').textContent = world.contacts.map(k => `${k.a} × ${k.b} ${fmt(-k.sep * 10, 1)} mm`).join(' · ');
    updateJointPanel(ctl.position);
    const f = world.frames, t = toolYawAndTilt(f.hand);
    $('pose-readout').textContent = `Cube-centre point x ${fmt(f.grasp[0])} · y ${fmt(f.grasp[1])} · z ${fmt(f.grasp[2])} cm · tool ${fmt(t.tilt, 0)}° off vertical`;
    $('hud').textContent = ctl.outputsOff ? `stopped: ${ctl.error}` : (mirroring() ? mirror.phase : ctl.motion?.label) || 'idle';
    if (current) {
      const lbl = mirroring() ? mirror.phase : ctl.motion?.label;
      $('plan-out').querySelectorAll('tr[data-i]').forEach(tr => tr.classList.toggle('cur', !!lbl && current.plan.steps[tr.dataset.i]?.label === lbl.split(':')[0]));
    }
    // The backdrop wall hides itself when the camera swings behind it.
    if (backdrop) backdrop.visible = camera.position.y > backdrop.position.y + 2;
    if (oakView) hideForCamera(view.render); else view.render();
    camTab.markDirty(); camTab.render();
    needsRender = false;
    const cubeKey = world.cubes.map(c => c.id + c.state + c.p.map(v => v.toFixed(1))).join('|');
    if (cubeKey !== lastCubeKey && !placing) { lastCubeKey = cubeKey; renderCubes(); }
  }
  requestAnimationFrame(frame);
}
resize();
requestAnimationFrame(frame);

// for the console and for tests
window.sim = { ctl, world, scene, camera, controls, renderer, view, arm, ghost, camTab, get link() { return link; }, makePlan, THREE, hideForCamera, anglesFromCounts, camCals };
