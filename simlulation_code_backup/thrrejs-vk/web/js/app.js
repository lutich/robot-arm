// Page glue: rendering, simulated joint control, tasks, scene editing and calibration.

import * as THREE from 'three';
import { createView } from './render.js';
import { SERVOS, MAT, CAMERA, CUBE, COLOURS } from './config.js';
import { fk, anglesFromCounts, countsFromAngles, ikTopDown, guardViolations, camToBoard, toolYawAndTilt, Unreachable, gripperTheta, gapForTheta } from './kinematics.js';
import { SimController } from './controller.js';
import { World } from './world.js';
import { cubeTilt } from './physics.js';
import { verify, toMoves } from './planner.js';
import { makeMaterials, buildArm, buildTable, buildCamera, buildCube } from './model/index.js';
import { initCameraTab, defaultCalibration, applyPinhole, tableToBoard } from './camview.js';
import { calibrate, MARKER_CORNERS } from './calib.js';
import { detectMarkers } from './aruco.js';
import { initDemo } from './demo.js';
import { loadCanonicalSpec, applyCanonicalServoMetadata, countsArray, countsObject, runtimeSpec } from './advanced.js';
import { optimizeJointPath, pathCost, planReferenceTransition, planTextTask } from '../src/core/planning.js';
import { RobotArmClient } from '../src/operational/client.js';
import { calibrationRows } from './calibration-map.js';
import { initPlanRunner } from './plan-runner.js';

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => v.toFixed(d).replace('-0.0', '0.0');
const canonicalSpec = await loadCanonicalSpec();
let planRunner = null; // task plan panel (plan-runner.js), set up after the scene exists

// ---------------- calibration (editable, saved per browser) ----------------

const CAL_KEY = 'pxsim.calibration.v1';
function applyCal(cal) {
  cal.servos.forEach((c, i) => Object.assign(SERVOS[i], c));
  MAT.sToEdge = cal.d;
}
try { const saved = JSON.parse(localStorage.getItem(CAL_KEY)); if (saved?.servos?.length === 5) applyCal(saved); } catch { /* defaults */ }
const REFERENCE_S5_VISUAL = applyCanonicalServoMetadata(SERVOS);
const DEFAULT_CAL = { servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge };
const currentCal = () => ({ servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge });

// ---------------- sim state ----------------

const ctl = new SimController();
const world = new World();
const DEFAULT_SCENE = [['orange', 10.8, 14.0, 0, 3], ['blue', -8, 12, 0, 0], ['red', 3, 10, 0, 0], ['green', -4, 17, 0, 10]];
function resetScene() {
  world.cubes = []; world.holding = null;
  for (const [c, x, y, l, yaw] of DEFAULT_SCENE) world.addCube(c, c, x, y, l, yaw);
}
resetScene();

// ---------------- renderer ----------------

const canvas = $('canvas');
const view = createView(canvas);
const { renderer, scene, camera, controls } = view;
let needsRender = true;
let s5VisualOffset = REFERENCE_S5_VISUAL.HOME;
let s5OffsetMotion = null;
let pendingVisualS5Offset = null;

function moveVisualS5Offset(target, durationSeconds = 0.8) {
  s5OffsetMotion = { start: s5VisualOffset, target, elapsed: 0, duration: Math.max(0.01, durationSeconds) };
}

function updateVisualS5Offset(dt) {
  if (!s5OffsetMotion) return false;
  s5OffsetMotion.elapsed += dt;
  const t = Math.min(1, s5OffsetMotion.elapsed / s5OffsetMotion.duration);
  const eased = t * t * (3 - 2 * t);
  s5VisualOffset = THREE.MathUtils.lerp(s5OffsetMotion.start, s5OffsetMotion.target, eased);
  if (t >= 1) s5OffsetMotion = null;
  return true;
}

function visualModelCounts(counts, wristOffset = s5VisualOffset) {
  const visual = counts.slice();
  visual[4] += wristOffset * SERVOS[4].cpd / SERVOS[4].dir;
  return visual;
}
camera.position.set(-30, 50, 30);
controls.target.set(0, 11, 7);
controls.update();

const M = makeMaterials();
const table = buildTable(M); scene.add(table);
const backdrop = table.getObjectByName('backdrop');
const arm = buildArm(M); scene.add(arm.root);

// Live arm ghost: the robot's commanded pose while Live Mode is connected, so a plan (solid
// arm) and the real robot can be compared in one view.
const ghostMaterial = new THREE.MeshStandardMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.3, depthWrite: false, roughness: 0.6 });
const ghostArm = buildArm(Object.fromEntries(Object.keys(M).map(key => [key, ghostMaterial])));
ghostArm.root.traverse(object => { object.castShadow = false; object.receiveShadow = false; });
ghostArm.root.visible = false;
scene.add(ghostArm.root);
let liveGhostCounts = null;
function updateGhost() {
  const show = robotClient.connected && !!liveGhostCounts && $('ghost-show').checked;
  ghostArm.root.visible = show;
  if (show) ghostArm.setPose(anglesFromCounts(liveGhostCounts), gapForTheta(gripperTheta(liveGhostCounts[5])));
}

// The OAK camera: fitted to the mat markers (camview.js); the fit is kept per browser.
const CAM_KEY = 'pxsim.camera.v1';
CAMERA.calib = defaultCalibration();
try { const c = JSON.parse(localStorage.getItem(CAM_KEY)); if (c && c.R && c.f) CAMERA.calib = c; } catch { /* default fit */ }
let oak = null;
function camPose() {
  const cal = CAMERA.calib, C = cal.centre, z = cal.R[2];
  const s = -C[2] / z[2];                                   // optical axis meets the mat
  return { pos: tableToBoard(C), target: tableToBoard([C[0] + s * z[0], C[1] + s * z[1], 0]) };
}
/** Render with the OAK camera's own body hidden (it would block its view). */
const hideForCamera = fn => { const v = oak.visible; oak.visible = false; fn(); oak.visible = v; };
function placeMatAndCamera() {
  table.userData.placeMat(MAT.sToEdge);
  if (oak) scene.remove(oak);
  const p = camPose(); oak = buildCamera(M, p.pos, p.target);
  // viewing pyramid: rays through the four image corners down to the desk (at most 1,5 m)
  const cal = CAMERA.calib, C = tableToBoard(cal.centre), pts = [];
  for (const [u, v] of [[0, 0], [CAMERA.w, 0], [CAMERA.w, CAMERA.h], [0, CAMERA.h]]) {
    const d = [(u - cal.cx) / cal.f, (v - cal.cy) / cal.f, 1];
    const wd = [0, 1, 2].map(i => cal.R[0][i] * d[0] + cal.R[1][i] * d[1] + cal.R[2][i] * d[2]);   // ray in the mat frame
    const s = wd[2] < -1e-6 ? Math.min((-10 - cal.centre[2]) / wd[2], 1500 / Math.hypot(...wd)) : 1500 / Math.hypot(...wd);   // desk is 10 mm below the mat
    pts.push(tableToBoard(cal.centre.map((c, i) => c + s * wd[i])));
  }
  const seg = [];
  for (let i = 0; i < 4; i++) seg.push(...C, ...pts[i], ...pts[i], ...pts[(i + 1) % 4]);
  const frustum = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(seg, 3)),
    new THREE.LineBasicMaterial({ color: 0x0e8a77, transparent: true, opacity: 0.55 }));
  frustum.name = 'camera-frustum';
  oak.add(frustum);
  scene.add(oak);
}
placeMatAndCamera();

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
  oak: () => { oakView = true; controls.enabled = false; applyOakView(); },
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
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x.dataset.view === 'orbit'));
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x === b));
  if (b.dataset.view !== 'oak') { leaveOakView(); VIEWS[b.dataset.view](); camera.updateProjectionMatrix(); controls.update(); }
  else VIEWS.oak();
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

// ---------------- shared live task status ----------------

const TASK_STATUS_ENDPOINT = '/simulator/task-status';
let taskStatus = {
  revision: 0, task: 'No active task', current: 'Waiting for a task', destination: '—',
  status: 'idle', message: 'No task is currently running.', updatedAt: null,
};
let sharedTaskRevision = -1;
let pendingTaskWrites = 0;
let taskWriteQueue = Promise.resolve();

function currentPositionText() {
  const p = world.frames?.grasp;
  const tcp = p ? `TCP x ${fmt(p[0])} · y ${fmt(p[1])} · z ${fmt(p[2])} cm` : 'TCP awaiting model update';
  return `${tcp} · counts [${ctl.commands.join(', ')}]`;
}

function renderTaskStatus() {
  $('task-status-name').textContent = taskStatus.task;
  $('task-current').textContent = taskStatus.current;
  $('task-destination').textContent = taskStatus.destination;
  $('task-status-message').textContent = taskStatus.message;
  $('task-status-badge').textContent = taskStatus.status;
  $('task-status-badge').className = `task-state ${taskStatus.status}`;
}

function acceptSharedTaskStatus(next) {
  if (!next || !Number.isInteger(next.revision) || next.revision < sharedTaskRevision) return;
  sharedTaskRevision = next.revision;
  taskStatus = next;
  renderTaskStatus();
}

function setTaskStatus(update, { share = true } = {}) {
  taskStatus = { ...taskStatus, ...update, current: update.current ?? currentPositionText() };
  renderTaskStatus();
  if (!share) return Promise.resolve(taskStatus);
  const snapshot = {
    task: taskStatus.task, current: taskStatus.current, destination: taskStatus.destination,
    status: taskStatus.status, message: taskStatus.message,
  };
  pendingTaskWrites++;
  taskWriteQueue = taskWriteQueue.then(async () => {
    const response = await fetch(TASK_STATUS_ENDPOINT, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(snapshot),
    });
    if (!response.ok) throw new Error(`task status HTTP ${response.status}`);
    acceptSharedTaskStatus(await response.json());
  }).catch(() => {
    // Vite's development server has no shared status endpoint. The panel still
    // works locally; the production gateway shares it between browsers.
  }).finally(() => { pendingTaskWrites--; });
  return taskWriteQueue;
}

async function pollTaskStatus() {
  if (pendingTaskWrites) return;
  try {
    const response = await fetch(TASK_STATUS_ENDPOINT, { cache: 'no-store' });
    if (!response.ok || !String(response.headers.get('content-type')).includes('application/json')) return;
    acceptSharedTaskStatus(await response.json());
  } catch { /* gateway absent or restarting */ }
}

renderTaskStatus();
void pollTaskStatus();
setInterval(pollTaskStatus, 750);

// The label of the latest motion, kept between queued motions (stepRapier's grasp freeze needs it).
let lastMotionLabel = '';
ctl.on((kind, detail) => {
  if (kind === 'motion') lastMotionLabel = detail;
  if (kind === 'stop') {
    pendingVisualS5Offset = null;
    log(`STOP: ${detail}`, 'err');
    void setTaskStatus({ status: 'stopped', message: `Stopped: ${detail}` });
  }
  else if (kind === 'motion' && !planRunner?.ownsArm) void setTaskStatus({ status: 'moving', message: detail });
  else if (kind === 'power') log(`power ${detail}`);
  else if (kind === 'idle' && pendingVisualS5Offset) {
    const pending = pendingVisualS5Offset;
    pendingVisualS5Offset = null;
    moveVisualS5Offset(pending.target, pending.duration);
  }
  if (kind === 'idle' && taskStatus.status === 'moving' && !planRunner?.ownsArm) {
    void setTaskStatus({ status: 'complete', message: 'Destination reached; controller is idle.' });
  }
  needsRender = true;
});

// ---------------- canonical planning, Rapier and operating mode ----------------

// Poll often enough that the simulation view visibly follows the robot.
const robotClient = new RobotArmClient({ apiBase: canonicalSpec.operational.sameOriginApiBase, pollIntervalMs: 500 });
const currentRuntimeSpec = () => runtimeSpec(SERVOS, world.cubes);
let rapierWorld = null;
let rapierGeneration = 0;
let rapierHeld = null;

async function rebuildRapier() {
  const generation = ++rapierGeneration;
  // Do not let the previous physics world write stale cube transforms back into
  // a freshly reset/edited scene while the replacement world initializes.
  rapierWorld = null;
  rapierHeld = null;
  try {
    const { PhysicsWorld } = await import('../src/core/physics.js');
    const physics = await new PhysicsWorld(currentRuntimeSpec()).initialize(countsObject(ctl.position));
    if (generation !== rapierGeneration) return;
    rapierWorld = physics;
    rapierHeld = null;
    $('mode-badge').textContent = robotClient.mode === 'simulation' ? 'simulation · Rapier' : robotClient.mode;
    log('Rapier3D world ready (millimetre canonical model)', 'ok');
    needsRender = true;
  } catch (error) {
    if (generation !== rapierGeneration) return;
    rapierWorld = null;
    $('mode-badge').textContent = 'simulation · physics unavailable';
    log(`Rapier3D unavailable: ${error.message}`, 'err');
  }
}

const quaternionMatrix = new THREE.Matrix4();
const rapierQuaternion = new THREE.Quaternion();
function syncRapierCubes(frozenCubeIds = new Set()) {
  if (!rapierWorld?.ready) return;
  for (const [id, transform] of Object.entries(rapierWorld.cubeTransforms())) {
    const cube = world.cube(id);
    if (!cube || cube.state === 'held' || frozenCubeIds.has(id)) continue;
    cube.p = transform.position.map((value) => value * 100);
    rapierQuaternion.set(...transform.rotation);
    quaternionMatrix.makeRotationFromQuaternion(rapierQuaternion);
    const e = quaternionMatrix.elements;
    cube.R = [e[0], e[4], e[8], e[1], e[5], e[9], e[2], e[6], e[10]];
    cube.v = transform.velocity.map((value) => value * 100);
    cube.w = [...transform.angularVelocity];
    cube.state = transform.sleeping ? 'asleep' : 'awake';
  }
}

function stepRapier(counts, dt) {
  if (!rapierWorld?.ready) return;
  const held = world.holding?.id ?? null;
  if (held !== rapierHeld) {
    if (rapierHeld) rapierWorld.releaseCube();
    if (held) rapierWorld.graspCube(held, countsObject(counts));
    rapierHeld = held;
  }
  rapierWorld.setRobotPose(countsObject(counts));
  rapierWorld.step(dt);
  const plannedPick = planRunner?.pickCube ?? current?.plan?.info?.pick;
  // Between queued motions ctl.motion is null for a tick; keep the freeze through those gaps,
  // or single-servo grip steps let Rapier's pushed copy of the cube slip out of the jaws.
  const plannedPhase = ctl.motion?.label ?? lastMotionLabel;
  // The detailed World model provides the deliberate jaw-centring/grasp rule;
  // do not let Rapier's free-body copy overwrite that selected cube while the
  // claw is approaching and closing. Other cubes remain fully dynamic.
  const freezePick = plannedPick && !world.holding && /^(Open gripper|Above |Descend|Grip |approach pick|pick|grasp)/i.test(plannedPhase);
  syncRapierCubes(freezePick ? new Set([plannedPick]) : new Set());
}

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
  deg[4] += s5VisualOffset;
  counts.forEach((c, i) => {
    const r = jointRows[i];
    if (!r.dragging && document.activeElement !== r.num) { r.range.value = Math.round(c); r.num.value = Math.round(c); }
    r.deg.textContent = i < 5 ? `model angle ${fmt(deg[i])}°` : `gap ${fmt(world.gap, 2)} cm${world.holding ? ` · holding ${world.holding.id}` : ''}`;
  });
}

async function command(kind, ch, value) {
  const target = kind === 'jog' ? ctl.commands[ch] + value * ctl.settings.step_size : value;
  const task = `${kind === 'jog' ? 'Jog' : 'Move'} ${SERVOS[ch].name}`;
  void setTaskStatus({ task, destination: `${SERVOS[ch].name} count ${target}`, status: 'planning', message: 'Checking servo bounds. No collision model is active.' });
  try {
    ctl.checkTarget(ch, target);
    const next = ctl.commands.slice(); next[ch] = target;
    const planning = currentRuntimeSpec();
    if (robotClient.mode === 'operational-armed') {
      await robotClient.executeSingleJointPath(planning, [countsObject(ctl.commands), countsObject(next)]);
      ctl.setCommands(next);
      void setTaskStatus({ status: 'complete', message: 'Robot API confirmed the requested servo count. This calibration move was not collision checked.' });
    } else if (robotClient.mode === 'simulation') {
      void setTaskStatus({ status: 'moving', message: 'Executing the unchecked virtual calibration movement.' });
      ctl.move(ch, target);
    } else {
      throw new Error('Manual motion is disabled until operational mode is explicitly armed or returned to Simulation');
    }
    moveVisualS5Offset(0, 0.35);
  } catch (e) { toast(e.message); log(e.message, 'err'); void setTaskStatus({ status: 'blocked', message: e.message }); }
}

function straightUpPose(planning) {
  const raw = countsFromAngles({ s1: 0, s2: 0, s3: 0, s4: 0, s5: 0 }, planning.gripper.openTick);
  return Object.fromEntries(planning.joints.map((joint, index) => [
    joint.id,
    Math.round(THREE.MathUtils.clamp(raw[index], joint.min, joint.max)),
  ]));
}

const poseCmd = name => async () => {
  const destinationCounts = name === 'STRAIGHT UP' ? 'provisional straight-up pose' : `${name} [${ctl.poses[name].join(', ')}]`;
  void setTaskStatus({ task: `Move arm to ${name}`, destination: destinationCounts, status: 'planning', message: `Preparing an unchecked simulation transition to ${name}.` });
  try {
    const planning = currentRuntimeSpec();
    if (name === 'STRAIGHT UP' && robotClient.mode !== 'simulation') {
      throw new Error('STRAIGHT UP is a provisional modelling pose and is available only in simulation');
    }
    if (robotClient.mode === 'operational-armed') {
      // The controller runs its own HOME/PARK definition and trajectory; the sim mirrors the confirmed counts.
      void setTaskStatus({ status: 'moving', message: `Robot API is moving all six servos to its ${name} definition. Not collision checked.` });
      const state = await robotClient.goPose(name);
      mirrorBackendState(state);
      void setTaskStatus({ destination: `${name} [${backendCounts(state).join(', ')}]`, status: 'complete', message: `Robot API confirmed ${name}. This move was not collision checked.` });
      log(`${name}: robot API confirmed [${backendCounts(state).join(', ')}], collision checks disabled`, 'warn');
      return;
    }
    const planned = name === 'STRAIGHT UP'
      ? optimizeJointPath(planning, countsObject(ctl.commands), straightUpPose(planning), { ignoreCubes: false })
      : planReferenceTransition(planning, countsObject(ctl.commands), name.toLowerCase());
    if (!planned.success) throw new Error(`${name} transition rejected: ${planned.reason}`);
    const targetOffset = REFERENCE_S5_VISUAL[name] ?? 0;
    const visualDuration = Math.max(0.5, planned.metrics.duration);
    if (robotClient.mode === 'simulation') {
      const poses = planned.path.slice(1).map((pose) => countsArray(pose));
      ctl.runPoses(poses, ctl.speed, poses.map((_pose, index) => `${name} unchecked ${index + 1}/${poses.length}`));
      // Keep the photo-reference wrist orientation out of the spatial move: it
      // is not represented by a separate hardware command, so rotating it while
      // the planned path runs can sweep visibly through a cube. Apply it only
      // after the interpolated controller path has settled.
      pendingVisualS5Offset = { target: targetOffset, duration: Math.min(0.8, visualDuration) };
      if (!ctl.busy) {
        const pending = pendingVisualS5Offset;
        pendingVisualS5Offset = null;
        moveVisualS5Offset(pending.target, pending.duration);
      }
      void setTaskStatus({ status: 'moving', message: `Executing ${planned.path.length - 1} unchecked calibration waypoint${planned.path.length === 2 ? '' : 's'} toward ${name}.` });
    } else {
      throw new Error('Pose motion is disabled in read-only or disarmed mode');
    }
    log(`${name}: ${planned.algorithm}, cost ${pathCost(planning, planned.path).toFixed(0)}, collision checks disabled`, 'warn');
  } catch (e) { toast(e.message); log(e.message, 'err'); void setTaskStatus({ status: 'blocked', message: e.message }); }
};
$('go-home').onclick = poseCmd('HOME');
$('go-park').onclick = poseCmd('PARK');
$('go-straight').onclick = poseCmd('STRAIGHT UP');
$('power-on').onclick = () => {
  if (robotClient.mode !== 'simulation') { toast('Use the guarded operational controls in Tools for the physical robot'); return; }
  ctl.rearm();
};
$('estop').onclick = async () => {
  demo.stop();
  ctl.stop('emergency stop button');
  void setTaskStatus({ status: 'stopped', message: 'Emergency stop requested; motion cancelled.' });
  if (robotClient.connected || robotClient.mode === 'fault-stopped') {
    try { await robotClient.emergencyStop(); } catch (e) { log(`robot stop: ${e.message}`, 'err'); }
  }
};

// ---------------- task panel ----------------

let current = null; // { plan, report, moves, start, mode, simulationPassed, worldStart }
let workflowBusy = false;

function updateExecutionGate(message = null) {
  const simulated = !!current?.simulationPassed;
  $('simulation-gate').textContent = simulated ? 'simulation passed' : current?.report?.ok ? 'ready to simulate' : 'not simulated';
  $('simulation-gate').className = simulated ? 'ok' : '';
  // Real task playback panel is commented out in index.html.
  // $('live-execution-confirm').checked = false;
  // $('live-execution-confirm').disabled = true;
  // $('run-live').disabled = true;
  // $('execution-gate-status').textContent = message ?? 'Automated physical task playback is disabled in calibration-only mode. Use simulation or explicitly armed single-joint sliders.';
}

const startCounts = () => ctl.commands.slice();

function describe(report) {
  const parts = [];
  parts.push(report.ok ? '<span class="ok">predicted physics outcome: success</span>' : '<span class="warn">predicted physics outcome: needs calibration</span>');
  parts.push(`${fmt(report.duration, 0)} s`);
  const warns = report.events.filter(e => e.kind === 'warn').map(e => e.text);
  if (warns.length) parts.push(`<span class="warn">${warns.join('; ')}</span>`);
  if (!report.ok) parts.push(`<span class="warn">predicted cube offset ${fmt(report.offset, 2)} cm</span>`);
  parts.push('<span class="warn">collision checks disabled</span>');
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
  updateExecutionGate();
}

function makePlan() {
  const commandText = $('cmd').value.trim() || 'Unnamed task';
  void setTaskStatus({ task: commandText, destination: commandText, status: 'planning', message: 'Solving inverse kinematics and checking servo bounds. Collision checks are disabled.' });
  try {
    const mode = $('exec-mode').value;
    const speed = Math.max(1, Math.min(100, Number($('demo-speed').value) || 20));
    const start = startCounts();
    const planning = currentRuntimeSpec();
    const advanced = planTextTask(planning, countsObject(start), $('cmd').value);
    if (!advanced.success) throw new Error(advanced.reason);
    const labels = new Map(advanced.waypoints.map((waypoint) => [JSON.stringify(waypoint.counts), waypoint.label]));
    const plan = {
      steps: advanced.path.slice(1).map((pose, index) => ({
        label: labels.get(JSON.stringify(pose)) ?? `optimized ${index + 1}`,
        counts: countsArray(pose),
      })),
      info: {
        hover: canonicalSpec.planning.approachHeight / 10,
        vias: Math.max(0, advanced.path.length - advanced.waypoints.length - 1),
        pick: advanced.task.cube,
        retainGrasp: advanced.task.type === 'pick',
        target: (advanced.task.destination ?? planning.world.cubes.find((cube) => cube.id === advanced.task.cube).position).map((value) => value / 10),
      },
    };
    let moves = null; let notes = [];
    if (mode === 'moves') ({ moves, notes } = toMoves(world, plan, start));
    const report = verify(world, plan, start, { mode, speed, moves });
    const executionPath = advanced.path;
    notes = [...(notes ?? []), `${advanced.algorithm}; cost ${advanced.cost.toFixed(0)}. IK and servo bounds checked; collision checks disabled.`];
    current = { plan, report, moves, notes, start, mode, speed, advanced, planning, executionPath, simulationPassed: false, worldStart: world.snapshot() };
    // $('live-execution-confirm').checked = false;
    renderPlan();
    showPath(plan.steps, start);
    log(`plan "${$('cmd').value}": ${plan.steps.length} poses, ${advanced.algorithm}, simulated outcome ${report.ok ? 'OK' : 'FAILED'}; collision checks disabled`, report.ok ? 'ok' : 'err');
    void setTaskStatus({
      status: 'ready',
      message: `Calibration plan ready: ${plan.steps.length} poses. Choose Run in sim to observe the outcome; automated real playback is disabled.`,
    });
    return current;
  } catch (e) {
    current = null;
    $('plan-out').innerHTML = `<span class="bad">${e.message}</span>`;
    $('run-sim').disabled = $('export-plan').disabled = true;
    void setTaskStatus({ status: 'blocked', message: e.message });
    updateExecutionGate();
    return null;
  }
}
// Task panel is commented out in index.html.
// $('plan-btn').onclick = makePlan;
// $('task-preset').onchange = () => {
//   const command = $('task-preset').value;
//   if (!command) return;
//   $('cmd').value = command;
//   current = null;
//   // $('live-execution-confirm').checked = false;
//   updateExecutionGate();
//   $('run-sim').disabled = $('export-plan').disabled = true;
//   $('plan-out').innerHTML = '<span class="note">Task selected. Choose Plan to calculate an unchecked calibration trajectory.</span>';
//   void setTaskStatus({ task: command, destination: command, status: 'waiting', message: 'Task selected. Choose Plan to calculate its calibration trajectory.' });
// };
// $('cmd').addEventListener('input', () => {
//   if ($('cmd').value !== $('task-preset').value) $('task-preset').value = '';
// });
// $('cmd').addEventListener('keydown', e => { if (e.key === 'Enter') makePlan(); });
// $('exec-mode').onchange = () => { if (current) makePlan(); };

function waitForSimulationIdle(timeoutMs = 120000) {
  return new Promise((resolve, reject) => {
    if (!ctl.busy) return resolve();
    const timer = setTimeout(() => { off(); reject(new Error('Visible simulation did not finish before the timeout')); }, timeoutMs);
    const off = ctl.on((kind, detail) => {
      if (kind === 'idle') { clearTimeout(timer); off(); resolve(); }
      else if (kind === 'stop') { clearTimeout(timer); off(); reject(new Error(`Visible simulation stopped: ${detail}`)); }
    });
  });
}

async function runCurrentSimulation() {
  if (!current) throw new Error('Create a calibration plan first');
  if (robotClient.mode !== 'simulation') throw new Error('Visible simulation must run in Simulation mode before Live Mode is armed');
  if (ctl.busy) throw new Error('Wait for the current simulated motion to finish');
  world.restore(current.worldStart);
  ctl.setCommands(current.start);
  ctl.rearm();
  await rebuildRapier();
  renderCubes();
  current.simulationPassed = false;
  // $('live-execution-confirm').checked = false;
  updateExecutionGate('Running the complete trajectory in the virtual robot…');
  moveVisualS5Offset(0, 0.5);
  void setTaskStatus({ task: $('cmd').value.trim() || taskStatus.task, destination: $('cmd').value.trim() || taskStatus.destination, status: 'moving', message: 'Running the unchecked calibration trajectory in simulation. Automated real execution remains disabled.' });
  if (current.mode === 'moves') ctl.runMoves(current.moves, current.moves.map(move => move.label));
  else ctl.runPoses(current.plan.steps.map(step => step.counts), current.speed, current.plan.steps.map(step => step.label));
  if (current.plan.steps.at(-1)?.label === 'HOME') pendingVisualS5Offset = { target: REFERENCE_S5_VISUAL.HOME, duration: 0.8 };
  world.events = [];
  await waitForSimulationIdle();
  const heldExpectedCube = current.advanced.task.type !== 'pick' || world.holding?.id === current.advanced.task.cube;
  current.simulationPassed = heldExpectedCube;
  current.simulatedAt = new Date().toISOString();
  updateExecutionGate();
  log(`visible simulation completed for "${$('cmd').value}"; ${heldExpectedCube ? 'expected grasp observed' : 'grasp needs calibration'}; automated real playback remains disabled`, heldExpectedCube ? 'ok' : 'warn');
  void setTaskStatus({ status: 'complete', message: heldExpectedCube
    ? 'Visible simulation completed with the expected outcome. Collision checks and automated real playback remain disabled.'
    : `Visible simulation completed, but the ${current.advanced.task.cube} cube was not held. Use this result for calibration; automated real playback remains disabled.` });
  return current;
}

// Task panel is commented out in index.html.
// $('run-sim').onclick = async () => {
//   try { await runCurrentSimulation(); }
//   catch (error) {
//     if (current) current.simulationPassed = false;
//     updateExecutionGate(error.message);
//     toast(error.message);
//     void setTaskStatus({ status: 'error', message: error.message });
//   }
// };
//
// $('export-plan').onclick = async () => {
//   if (!current) return;
//   const json = JSON.stringify({ name: $('cmd').value, units: 'PCA9685_counts', nominal_frequency_hz: 60,
//     positions: current.plan.steps.map(s => ({ name: s.label, counts: Object.fromEntries(s.counts.map((c, i) => [String(i), c])) })) }, null, 1);
//   try { await navigator.clipboard.writeText(json); toast('Plan copied as demo JSON'); } catch { console.log(json); toast('Clipboard blocked; JSON printed to the console'); }
// };

$('goto-xyz').onclick = () => {
  const destination = `TCP x ${$('gx').value} · y ${$('gy').value} · z ${$('gz').value} cm`;
  void setTaskStatus({ task: 'Move gripper to XYZ', destination, status: 'planning', message: 'Solving inverse kinematics and checking servo bounds.' });
  try {
    const s5 = anglesFromCounts(ctl.commands).s5;
    const a = ikTopDown(Number($('gx').value), Number($('gy').value), Number($('gz').value), { s5Near: s5 });
    const c = countsFromAngles(a, ctl.commands[5]);
    const bad = guardViolations(c); if (bad.length) throw new Error(bad.join(', '));
    void setTaskStatus({ status: 'moving', message: 'Executing the inverse-kinematics solution in simulation.' });
    // Demo speed input lives in the commented-out Task panel; 40 was its default.
    ctl.runPoses([c], Math.max(1, Number($('demo-speed')?.value ?? 40) || 20), ['go to x y z']);
  } catch (e) { const message = e instanceof Unreachable ? `Unreachable: ${e.message}` : e.message; toast(message); void setTaskStatus({ status: 'blocked', message }); }
};

// ---------------- demonstration (demo.js) ----------------

const demo = initDemo({
  ctl, world, $, log, toast, renderCubes, showPath,
  setTaskStatus: update => setTaskStatus(update),
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
    rm.onclick = () => { if (world.holding?.id === c.id) world.holding = null; world.cubes = world.cubes.filter(k => k !== c); world.dirty = true; void rebuildRapier(); renderCubes(); needsRender = true; };
    $('cubes').append(row);
  }
}
$('add-cube').onclick = () => {
  const colour = $('new-colour').value;
  let id = colour, n = 2;
  while (world.cube(id)) id = `${colour}${n++}`;
  world.addCube(id, colour, Number($('nx').value), Number($('ny').value), 0, 0);
  world.cube(id).p[2] = 30;
  void rebuildRapier();
  renderCubes();
};
$('reset-scene').onclick = () => { resetScene(); void rebuildRapier(); renderCubes(); needsRender = true; };

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
  if (c && c.state !== 'held') { c.p = [hit.x, hit.y, 12]; c.v = [0, 0, 0]; c.w = [0.4, 0.3, 0]; c.state = 'awake'; }
  placing = null; renderCubes();
});

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

// ---------------- guarded virtual / operational modes ----------------

const operatingModeStatus = $('operating-mode-status');
let liveModeBusy = false;
const backendCounts = (state) => SERVOS.map((_servo, index) => {
  const value = state?.commands?.[index] ?? state?.commands?.[String(index)];
  return Number.isInteger(value) ? value : null;
});
function mirrorBackendState(state) {
  const counts = backendCounts(state);
  if (counts.every(Number.isInteger)) {
    liveGhostCounts = counts;
    planRunner?.onBackendState(counts);
    // While a plan is shown, the solid arm belongs to the plan; the ghost shows the robot.
    if (!planRunner?.ownsArm) {
      ctl.setCommands(counts);
      moveVisualS5Offset(0, 0.4);
    }
    needsRender = true;
  }
  $('live-servo-counts').textContent = counts.every(Number.isInteger)
    ? `Commanded S1–S6: ${counts.join(', ')}${state?.busy ? ' · moving' : ' · idle'}`
    : 'Servos: controller did not provide a complete commanded pose';
}

let liveCameraTimer = null;
let liveCameraStarted = false;
function scheduleLiveCameraRefresh(delay = 1000) {
  clearTimeout(liveCameraTimer);
  liveCameraTimer = setTimeout(refreshLiveCamera, delay);
}
function refreshLiveCamera() {
  if (document.hidden) return scheduleLiveCameraRefresh(1000);
  const image = $('live-camera-photo');
  image.onload = () => {
    $('live-camera-state').textContent = `updated ${new Date().toLocaleTimeString()}`;
    scheduleLiveCameraRefresh(1000);
  };
  image.onerror = () => {
    $('live-camera-state').textContent = 'camera unavailable';
    scheduleLiveCameraRefresh(2000);
  };
  image.src = `/api/camera/snapshot.jpg?live=${Date.now()}`;
}
function startLiveCamera() {
  if (liveCameraStarted) return;
  liveCameraStarted = true;
  refreshLiveCamera();
}
startLiveCamera();

function renderOperatingMode(message = null) {
  const mode = robotClient.mode;
  const liveButton = $('live-mode');
  $('operating-mode-label').textContent = mode;
  $('mode-badge').textContent = mode === 'simulation' ? (rapierWorld?.ready ? 'simulation · Rapier' : 'simulation') : mode;
  liveButton.textContent = ({
    simulation: 'Live Mode',
    'connected-read-only': 'Arm Live',
    'operational-disarmed': 'Arm Live',
    'operational-armed': 'Live Armed',
    'fault-stopped': 'Reconnect Live',
  }[mode] ?? 'Live Mode');
  liveButton.className = `live-mode${liveModeBusy ? ' connecting' : ''}${mode === 'operational-armed' ? ' armed' : mode !== 'simulation' ? ' connected' : ''}`;
  liveButton.disabled = liveModeBusy;
  liveButton.setAttribute('aria-pressed', String(mode === 'operational-armed'));
  $('park-confirm').disabled = mode !== 'operational-disarmed';
  $('arm-operational').disabled = mode !== 'operational-disarmed' || !$('park-confirm').checked;
  $('power-off-operational').disabled = !['operational-armed', 'operational-disarmed'].includes(mode);
  // $('run-sim').textContent = 'Run in sim'; // Task panel is commented out in index.html.
  operatingModeStatus.textContent = message ?? ({
    simulation: 'Virtual simulation. No robot API requests are sent.',
    'connected-read-only': 'Connected read-only. Displayed positions are commanded values because measured feedback is unavailable.',
    'operational-disarmed': 'Operational mode selected but disarmed. Confirm supported PARK before arming.',
    'operational-armed': 'OPERATIONAL ARMED for explicit single-joint calibration only. No collision model is active.',
    'fault-stopped': 'Fault stopped. Return to Simulation or reconnect read-only.',
  }[mode] ?? mode);
  updateExecutionGate();
}

async function toggleLiveMode() {
  if (liveModeBusy) return;
  liveModeBusy = true;
  let finalMessage = null;
  renderOperatingMode('Connecting to the physical robot read-only…');
  void setTaskStatus({ task: 'Connect Live Mode', destination: 'Physical robot controller', status: 'planning', message: 'Reading controller state; no motion has been requested.' });
  try {
    if (robotClient.mode === 'operational-armed') {
      const confirmed = window.confirm('Exit Live Mode by returning the physical arm to PARK and switching servo outputs off? Keep the workspace clear and remain at the power switch.');
      if (!confirmed) return;
      void setTaskStatus({ status: 'moving', destination: 'PARK, then outputs off', message: 'Returning the physical arm to PARK before leaving Live Mode.' });
      await robotClient.powerOff();
      $('park-confirm').checked = false;
      $('operating-mode').value = 'connected-read-only';
      void setTaskStatus({ status: 'complete', message: 'Controller confirmed PARK and outputs off. Live motion is disabled.' });
      return;
    }

    if (!['connected-read-only', 'operational-disarmed'].includes(robotClient.mode)) {
      await robotClient.connectReadOnly();
      mirrorBackendState(robotClient.state);
      robotClient.startPolling();
    }
    if (robotClient.mode === 'connected-read-only') robotClient.enterOperationalDisarmed();
    $('operating-mode').value = 'operational-disarmed';

    const state = await robotClient.refresh();
    mirrorBackendState(state);
    const counts = backendCounts(state);
    const alreadyArmed = state.armed && !state.outputs_off;
    const confirmation = alreadyArmed
      ? `The controller is already ARMED and idle at commanded counts [${counts.join(', ')}]. Confirm that the physical arm matches the displayed simulator pose, the workspace is clear, and you are at the power switch. Enable live joint sliders?`
      : 'The controller outputs are off. Confirm that the physical arm is supported in PARK, the workspace is clear, and you are at the power switch. Power on and move PARK → HOME?';
    if (!window.confirm(confirmation)) {
      void setTaskStatus({ status: 'waiting', message: 'Connected read-only. Live joint motion is still disarmed.' });
      return;
    }

    $('park-confirm').checked = !alreadyArmed;
    if (alreadyArmed) await robotClient.adoptArmedSession({ operatorConfirmed: true });
    else await robotClient.arm({ parkConfirmed: true });
    mirrorBackendState(robotClient.state);
    void setTaskStatus({
      status: 'ready',
      current: currentPositionText(),
      destination: 'Operator-selected joint target',
      message: 'LIVE ARMED. Release a joint slider to send one checked servo command.',
    });
  } catch (error) {
    finalMessage = error.message;
    if (!robotClient.connected) liveLinkError = error.message;
    log(`Live Mode: ${error.message}`, 'err');
    void setTaskStatus({ status: 'error', message: error.message });
  } finally {
    liveModeBusy = false;
    renderOperatingMode(finalMessage);
  }
}

$('live-mode').onclick = toggleLiveMode;

// ---------------- live link banner in the simulation view ----------------

let lastLiveStateAt = 0;
let liveLinkError = null;
/** Say in the 3D view whether it mirrors the live robot, or that there is no connection. */
function renderLiveLink() {
  const banner = $('live-link');
  const age = lastLiveStateAt ? (Date.now() - lastLiveStateAt) / 1000 : Infinity;
  const lastPose = lastLiveStateAt ? ` from ${new Date(lastLiveStateAt).toLocaleTimeString()}` : '';
  let text = null; let lost = false;
  if (robotClient.connected && age < 5) {
    text = planRunner?.ownsArm
      ? `LIVE · connected (updated ${age.toFixed(0)} s ago) · the solid arm shows the plan, the blue ghost shows the robot`
      : `LIVE · mirroring the robot's commanded servo counts (updated ${age.toFixed(0)} s ago)`;
  } else if (robotClient.connected) {
    lost = true;
    text = `No connection to the live robot arm for ${Number.isFinite(age) ? age.toFixed(0) : '?'} s · showing the last known pose${lastPose}`;
  } else if (robotClient.mode === 'fault-stopped' || liveLinkError) {
    lost = true;
    text = `No connection to the live robot arm${liveLinkError ? `: ${liveLinkError}` : ''} · this view shows the simulation${lastLiveStateAt ? `; last robot pose${lastPose}` : ''} · press ${robotClient.mode === 'fault-stopped' ? 'Reconnect Live' : 'Live Mode'} to retry`;
  }
  banner.hidden = !text;
  if (text) { banner.textContent = text; banner.className = `live-link ${lost ? 'lost' : 'ok'}`; }
}
setInterval(renderLiveLink, 1000);

robotClient.addEventListener('modechange', () => { renderOperatingMode(); renderLiveLink(); });
robotClient.addEventListener('statechange', (event) => {
  lastLiveStateAt = Date.now();
  liveLinkError = null;
  mirrorBackendState(event.detail.state);
  void setTaskStatus({ current: currentPositionText() }, { share: false });
  renderOperatingMode(`Connected to ${event.detail.state.mode}; commanded pose updated${event.detail.state.busy ? ' · motion in progress' : ''}.`);
  renderLiveLink();
});
robotClient.addEventListener('connectionerror', (event) => {
  liveLinkError = event.detail.error.message;
  renderOperatingMode(`Connection lost: ${event.detail.error.message}`);
  log(`Robot API connection lost: ${event.detail.error.message}`, 'err');
  renderLiveLink();
});
$('park-confirm').onchange = () => renderOperatingMode();
$('connect-mode').onclick = async () => {
  try {
    const requested = $('operating-mode').value;
    if (requested === 'simulation') {
      robotClient.returnToSimulation();
      liveLinkError = null; // an explicit return to Simulation is not a lost connection
      renderOperatingMode();
      renderLiveLink();
      return;
    }
    operatingModeStatus.textContent = 'Connecting read-only to the same-origin robot API…';
    if (robotClient.mode !== 'connected-read-only') await robotClient.connectReadOnly();
    mirrorBackendState(robotClient.state);
    if (requested === 'operational-disarmed') robotClient.enterOperationalDisarmed();
    robotClient.startPolling();
    renderOperatingMode();
  } catch (error) {
    if (!robotClient.connected) liveLinkError = error.message;
    renderOperatingMode(error.message);
    renderLiveLink();
    log(error.message, 'err');
  }
};
$('arm-operational').onclick = async () => {
  try {
    operatingModeStatus.textContent = 'Arming from confirmed supported PARK…';
    await robotClient.arm({ parkConfirmed: $('park-confirm').checked });
    mirrorBackendState(robotClient.state);
    renderOperatingMode();
  } catch (error) {
    renderOperatingMode(error.message);
    log(error.message, 'err');
  }
};
$('power-off-operational').onclick = async () => {
  try {
    operatingModeStatus.textContent = 'Returning to PARK and disabling outputs…';
    await robotClient.powerOff();
    mirrorBackendState(robotClient.state);
    $('park-confirm').checked = false;
    renderOperatingMode();
  } catch (error) {
    renderOperatingMode(error.message);
    log(error.message, 'err');
  }
};

// Real task playback panel is commented out in index.html.
// $('live-execution-confirm').onchange = () => updateExecutionGate();

async function executeCurrentOnRobot() {
  throw new Error('Automated physical task playback is disabled while calibration mode has no collision model');
}

// $('run-live').onclick = async () => {
//   try { await executeCurrentOnRobot(); }
//   catch (error) { toast(error.message, 7000); log(error.message, 'err'); }
// };
renderOperatingMode();

// ---------------- tabs and camera tab ----------------

document.querySelectorAll('[data-tab]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-tab]').forEach(x => x.classList.toggle('on', x === b));
  $('tab-control').hidden = b.dataset.tab !== 'control';
  $('tab-camera').hidden = b.dataset.tab !== 'camera';
  if (b.dataset.tab === 'camera') { camTab.activate(); camTab.update(); }
});
function tcpForCounts(counts, wristOffset = s5VisualOffset) {
  return fk(anglesFromCounts(visualModelCounts(counts, wristOffset))).grasp.map(value => value * 10);
}
function tcpComparison() {
  const simulatedBoardMm = world.frames?.grasp?.map(value => value * 10) ?? tcpForCounts(ctl.position);
  const motion = ctl.motion ?? ctl.queue[0] ?? null;
  const plannedCounts = ctl.position.slice();
  if (motion) for (const channel of motion.channels) plannedCounts[channel] = motion.target[channel];
  return {
    simulatedBoardMm,
    plannedBoardMm: tcpForCounts(plannedCounts, s5OffsetMotion?.target ?? s5VisualOffset),
    plannedLabel: motion?.label ?? 'current pose',
  };
}
let latestGripperObservation = null;
const SVG_NS = 'http://www.w3.org/2000/svg';
function renderMatReferenceMap(rows) {
  const svg = $('mat-reference-svg');
  const title = svg.querySelector('title');
  const description = svg.querySelector('desc');
  svg.replaceChildren(title, description);
  const element = (name, attributes = {}, text = '') => {
    const node = document.createElementNS(SVG_NS, name);
    for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
    if (text) node.textContent = text;
    svg.append(node);
    return node;
  };
  const screen = ([x, y]) => [150 - y * 0.70, 238 - x * 0.55];

  element('rect', { x: 45, y: 29, width: 210, height: 209, rx: 5, class: 'mat-outline' });
  for (const x of [0, 100, 200, 300]) {
    const [, sy] = screen([x, 0]);
    element('line', { x1: 45, y1: sy, x2: 255, y2: sy, class: 'mat-grid-line' });
    element('text', { x: 259, y: sy + 3, class: 'mat-axis-label' }, `${x}`);
  }
  element('line', { x1: 150, y1: 238, x2: 150, y2: 20, class: 'mat-axis' });
  element('line', { x1: 150, y1: 238, x2: 35, y2: 238, class: 'mat-axis' });
  element('text', { x: 155, y: 20, class: 'mat-axis-title' }, '+X away from arm');
  element('text', { x: 48, y: 252, class: 'mat-axis-title' }, '+Y left');
  element('text', { x: 175, y: 252, class: 'mat-arm-label' }, 'ARM · X 0');

  const markers = new Map(rows.map(row => [row.marker.id, row.marker]));
  for (const marker of [...markers.values()].sort((a, b) => a.id - b.id)) {
    const [x, y] = screen(marker.matMm);
    if (marker.cameraPixel) element('circle', { cx: x, cy: y, r: 9, class: 'mat-marker-seen' });
    element('circle', { cx: x, cy: y, r: 5, class: 'mat-marker' });
    element('text', { x: x + 8, y: y - 6, class: 'mat-marker-label' }, `M${marker.id}`);
  }

  const grouped = new Map();
  for (const row of rows) {
    const key = row.matMm.join(',');
    const group = grouped.get(key) ?? { matMm: row.matMm, names: [] };
    group.names.push(row.name.replace('Pos', 'P').replace('_left', ' L').replace('_right', ' R'));
    grouped.set(key, group);
  }
  for (const group of grouped.values()) {
    const [x, y] = screen(group.matMm);
    element('path', { d: `M ${x} ${y - 6} L ${x + 6} ${y} L ${x} ${y + 6} L ${x - 6} ${y} Z`, class: 'mat-pose' });
    const label = group.names.join('/').replace('P1 L/P1 R', 'P1 L/R');
    const text = element('text', { x: x + 8, y: y + 12, class: 'mat-pose-label' }, label);
    element('title', {}, `${group.names.join(' / ')}: X ${group.matMm[0]}, Y ${group.matMm[1]} mm`);
    text.append(svg.lastChild);
  }
}

function renderCalibrationMap(observation = {}) {
  const rows = calibrationRows(MARKER_CORNERS, observation);
  renderMatReferenceMap(rows);
  const body = $('calibration-map-body');
  body.replaceChildren();

  for (const row of rows) {
    const tr = document.createElement('tr');
    const addCell = (text, className = '') => {
      const td = document.createElement('td');
      td.textContent = text;
      if (className) td.className = className;
      tr.append(td);
    };
    addCell(`${row.name} · ${row.provenance}`, row.provenance.includes('inferred') ? 'provisional' : '');
    addCell(`X ${row.matMm[0]} · Y ${row.matMm[1]} mm · ±${row.toleranceMm} mm`);
    addCell(`ID ${row.marker.id} · X ${row.marker.matMm[0]} · Y ${row.marker.matMm[1]} mm · Δ ${row.marker.deltaMm.toFixed(1)} mm`);

    const markerText = row.marker.cameraPixel
      ? `ID ${row.marker.id}: u ${row.marker.cameraPixel[0].toFixed(1)} · v ${row.marker.cameraPixel[1].toFixed(1)} px`
      : `ID ${row.marker.id}: not detected`;
    const cubeText = row.cameraCube
      ? ` · nearest ${row.cameraCube.colour} X ${row.cameraCube.tableMm[0].toFixed(0)} · Y ${row.cameraCube.tableMm[1].toFixed(0)} (Δ ${row.cameraCube.deltaMm.toFixed(1)} mm)`
      : '';
    addCell(markerText + cubeText, row.marker.cameraPixel ? 'camera-seen' : 'camera-missing');
    addCell(row.counts.join(' · '), 'servo-counts');
    body.append(tr);
  }

  const seen = new Set((observation.markers ?? []).map(marker => marker.id)).size;
  const when = observation.timestamp ? ` · ${new Date(observation.timestamp).toLocaleTimeString()}` : '';
  $('calibration-map-status').textContent = seen ? `${seen}/6 mat markers observed${when}` : 'Waiting for a camera scan';
}

function renderVisionObservation(observation) {
  const camera = observation.camera;
  if (camera) {
    const p = camera.boardMm;
    $('vision-camera').textContent = `Camera position (board frame): x ${p[0].toFixed(1)} · y ${p[1].toFixed(1)} · z ${p[2].toFixed(1)} mm · pitch ${camera.pitchDeg.toFixed(1)}° · heading ${camera.yawDeg.toFixed(1)}° · roll ${camera.rollDeg.toFixed(1)}° · ${camera.source} · fit ${camera.rmsPx.toFixed(2)} px RMS`;
  }
  const blocks = $('vision-blocks');
  blocks.replaceChildren();
  if (!observation.cubes?.length) {
    blocks.textContent = 'Blocks: none detected in the visible calibrated mat area.';
  } else {
    for (const cube of observation.cubes) {
      const chip = document.createElement('span');
      chip.className = `vision-block${cube.ok ? '' : ' uncertain'}`;
      chip.style.setProperty('--cube-colour', `#${new THREE.Color(COLOURS[cube.colour] ?? 0x999999).getHexString()}`);
      const p = cube.boardMm;
      chip.textContent = `${cube.colour}: x ${p[0].toFixed(1)}, y ${p[1].toFixed(1)}, z ${p[2].toFixed(1)} mm · yaw ${cube.yawDeg.toFixed(0)}°${cube.ok ? '' : ' · uncertain/partly hidden'}`;
      blocks.append(chip);
    }
  }
  const when = observation.timestamp ? ` · ${new Date(observation.timestamp).toLocaleTimeString()}` : '';
  $('vision-status').textContent = `${observation.message}${when} · Z is the assumed 13.25 mm cube-centre height; it is not a depth measurement.`;
  $('vision-status').className = `vision-status${observation.state === 'error' ? ' error' : ''}`;
  renderCalibrationMap(observation);
}

function putDetectionsIntoSimulation(list) {
  world.cubes = []; world.holding = null;
  for (const detection of list) {
    const [x, y] = tableToBoard([detection.X, detection.Y, 0]);
    let id = detection.colour; let suffix = 2;
    while (world.cube(id)) id = `${detection.colour}${suffix++}`;
    world.addCube(id, detection.colour, x, y, 0, detection.yaw);
  }
  void rebuildRapier();
  renderCubes();
  needsRender = true;
}

// Live Mode only: after a camera scan or snapshot refresh, make the simulation match the real scene.
// The arm takes the controller's commanded counts (a fresh read-only GET /api/state; the statechange
// listener mirrors them) and the sim cubes are replaced by the reliable camera detections.
async function syncSimulationWithLive() {
  if (!robotClient.connected) return null;
  planRunner?.markStale('a camera scan replaced the scene');
  try { await robotClient.refresh(); }
  catch (error) { log(`live sync: arm state unavailable (${error.message}); blocks synced only`, 'warn'); }
  const reliable = camTab.detections.filter(detection => detection.ok);
  putDetectionsIntoSimulation(reliable);
  sceneSource = 'camera sync';
  sceneSyncedAt = new Date().toISOString();
  log(`live sync: arm at commanded [${backendCounts(robotClient.state).join(', ')}], ${reliable.length} camera block${reliable.length === 1 ? '' : 's'} placed`, 'ok');
  return reliable.length;
}

const camTab = initCameraTab({
  scene, initial: CAMERA.calib, hideForCamera, log, toast,
  gripperMarker: canonicalSpec.tracking.gripperMarker,
  tcpComparison,
  onGripperObservation: observation => { latestGripperObservation = observation; },
  onVisionUpdate: renderVisionObservation,
  onConnectedRefresh: () => scanConnectedCamera(),
  onApply: cal => {
    CAMERA.calib = cal;
    try { localStorage.setItem(CAM_KEY, JSON.stringify({ R: cal.R, t: cal.t, f: cal.f, cx: cal.cx, cy: cal.cy, centre: cal.centre, rms: cal.rms, H: cal.H })); } catch { /* not saved */ }
    placeMatAndCamera(); if (oakView) applyOakView(); needsRender = true;
  },
  useDetections: list => {
    putDetectionsIntoSimulation(list);
    log(`${list.length} cubes from the camera put on the mat`, 'ok');
    toast(`${list.length} cubes placed. Try a task like "${list[0]?.colour || 'orange'} to 0 14".`);
  },
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
    if (changed) { void rebuildRapier(); renderCubes(); needsRender = true; }
  },
});
renderCalibrationMap();

async function scanConnectedCamera({ rethrow = false } = {}) {
  const button = $('vision-scan');
  button.disabled = true;
  button.textContent = 'Scanning…';
  void setTaskStatus({
    task: 'Identify blocks with OAK camera',
    destination: 'Camera observations below the robot viewport',
    status: 'planning',
    message: 'Loading a read-only camera snapshot, fitting the mat markers and detecting coloured blocks.',
  });
  try {
    const observation = await camTab.scanConnected();
    const reliable = observation.cubes.filter(cube => cube.ok).length;
    const synced = await syncSimulationWithLive();
    void setTaskStatus({
      status: 'complete',
      message: `Camera scan complete: ${reliable} reliable block${reliable === 1 ? '' : 's'} and ${observation.cubes.length - reliable} uncertain candidate${observation.cubes.length - reliable === 1 ? '' : 's'} displayed.${synced === null ? '' : ` Simulation synced to the live arm and ${synced} block${synced === 1 ? '' : 's'}.`}`,
    });
    return observation;
  } catch (error) {
    toast(error.message);
    void setTaskStatus({ status: 'error', message: `Camera scan failed: ${error.message}` });
    if (rethrow) throw error;
    return null;
  } finally {
    button.disabled = false;
    button.textContent = 'Scan camera';
  }
}

$('vision-scan').onclick = scanConnectedCamera;
void scanConnectedCamera();

// ---------------- task plan panel and the scene Claude plans against ----------------

let sceneSource = 'simulation';
let sceneSyncedAt = null;
let lastScenePayload = '';
/** Publish what the page shows (cubes, commanded counts, calibration) unless a plan owns the view. */
async function publishScene() {
  if (planRunner?.ownsArm) return;
  const payload = JSON.stringify({ commands: ctl.commands.map(Math.round), snapshot: world.snapshot(), calibration: currentCal(),
    mode: robotClient.mode, source: sceneSource, syncedAt: sceneSyncedAt });
  if (payload === lastScenePayload) return;
  try {
    const response = await fetch('/simulator/scene', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: payload });
    if (response.ok) lastScenePayload = payload;
  } catch { /* no plan channel on this server */ }
}
setInterval(publishScene, 2000);

planRunner = initPlanRunner({
  $, ctl, world, robotClient, log, toast, canonicalSpec, countsObject, backendCounts,
  setTaskStatus, rebuildRapier, renderCubes, showPath,
  runtimeSpec: currentRuntimeSpec,
  sceneSettled: () => world.settled() && !(rapierWorld?.ready && !rapierWorld.isSettled()),
  resumeLiveMirror: () => { if (robotClient.connected && robotClient.state) mirrorBackendState(robotClient.state); },
});

async function detectPlanAndSimulate() {
  if (workflowBusy) return;
  if (robotClient.mode !== 'simulation') throw new Error('Return to Simulation mode before preparing a new physical task');
  const commandText = $('cmd').value.trim();
  if (!commandText) throw new Error('Enter a task such as "grab green cube"');
  workflowBusy = true;
  const button = $('workflow-btn');
  button.disabled = true;
  button.textContent = 'Detecting…';
  try {
    void setTaskStatus({ task: commandText, destination: commandText, status: 'planning', message: 'Detecting cubes with the live camera before trajectory planning.' });
    await scanConnectedCamera({ rethrow: true });
    const reliable = camTab.detections.filter(detection => detection.ok);
    const requestedColour = ['orange', 'green', 'blue', 'yellow', 'red'].find(colour => new RegExp(`\\b${colour}\\b`, 'i').test(commandText));
    if (!requestedColour) throw new Error('Name a cube colour in the task');
    if (!reliable.some(detection => detection.colour === requestedColour)) throw new Error(`The camera did not obtain a reliable ${requestedColour} cube detection`);
    putDetectionsIntoSimulation(reliable);

    button.textContent = 'Reading arm…';
    const state = await robotClient.refresh();
    if (state.busy) throw new Error('The physical controller is busy; wait before planning');
    const counts = backendCounts(state);
    if (!counts.every(Number.isInteger)) throw new Error('The physical controller did not report all six commanded servo values');
    ctl.setCommands(counts);
    moveVisualS5Offset(0, 0.3);
    needsRender = true;
    log(`workflow starts from controller-commanded counts [${counts.join(', ')}] (read-only)`, 'ok');

    button.textContent = 'Planning…';
    const planned = makePlan();
    if (!planned) throw new Error('The detected task did not produce a calibration plan');
    button.textContent = 'Simulating…';
    await runCurrentSimulation();
  } finally {
    workflowBusy = false;
    button.disabled = false;
    button.textContent = 'Detect → plan → simulate';
  }
}

// Task panel is commented out in index.html.
// $('workflow-btn').onclick = async () => {
//   try { await detectPlanAndSimulate(); }
//   catch (error) {
//     if (current) current.simulationPassed = false;
//     updateExecutionGate(error.message);
//     toast(error.message, 7000);
//     log(`workflow: ${error.message}`, 'err');
//     void setTaskStatus({ status: 'error', message: error.message });
//   }
// };

// ---------------- main loop ----------------

let last = performance.now(), eventIdx = 0, lastCubeKey = '', lastTaskPositionUpdate = 0;
controls.addEventListener('change', () => { needsRender = true; });
$('show-path').onchange = () => { pathLine.visible = pathDots.visible = $('show-path').checked; needsRender = true; };
$('ghost-show').onchange = () => { needsRender = true; };
robotClient.addEventListener('modechange', () => { needsRender = true; });

world.step(0, visualModelCounts(ctl.position));
renderCubes();
void rebuildRapier();

function frame(now) {
  const real = Math.min(0.05, (now - last) / 1000); last = now;
  const dt = real * Number($('time-scale').value);
  const physicsMoving = rapierWorld?.ready && !rapierWorld.isSettled();
  const moving = ctl.busy || !world.settled() || !!world.holding || physicsMoving;
  if (updateVisualS5Offset(real)) needsRender = true;
  if (moving || world.dirty) {
    const n = Math.min(40, Math.max(1, Math.ceil(dt / (1 / 120))));
    for (let i = 0; i < n; i++) {
      ctl.update(dt / n);
      world.step(rapierWorld?.ready ? 0 : dt / n, visualModelCounts(ctl.position));
      stepRapier(ctl.position, dt / n);
    }
    world.dirty = false;
    needsRender = true;
  }
  // world events -> log
  for (; eventIdx < world.events.length; eventIdx++) { const e = world.events[eventIdx]; log(e.text, e.kind === 'warn' ? 'warn' : e.kind === 'grasp' ? 'ok' : ''); }
  if (eventIdx > world.events.length) eventIdx = world.events.length;

  if (!oakView && controls.update()) needsRender = true;
  if (needsRender) {
    const a = anglesFromCounts(visualModelCounts(ctl.position));
    arm.setPose(a, world.gap);
    updateGhost();
    syncCubeMeshes();
    updateJointPanel(ctl.position);
    const f = world.frames, t = toolYawAndTilt(f.hand);
    $('pose-readout').textContent = `Cube-centre point x ${fmt(f.grasp[0])} · y ${fmt(f.grasp[1])} · z ${fmt(f.grasp[2])} cm · tool ${fmt(t.tilt, 0)}° off vertical`;
    $('hud').textContent = ctl.outputsOff ? `stopped: ${ctl.error}` : ctl.motion ? ctl.motion.label : 'idle';
    if (taskStatus.status === 'moving' && now - lastTaskPositionUpdate > 200) {
      lastTaskPositionUpdate = now;
      taskStatus.current = currentPositionText();
      renderTaskStatus();
    }
    if (current) {
      const lbl = ctl.motion?.label;
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
window.sim = {
  ctl, world, scene, camera, controls, renderer, view, arm, camTab, makePlan, THREE,
  canonicalSpec, currentRuntimeSpec, robotClient,
  photoReference: { calibrate, detectMarkers, markerCorners: MARKER_CORNERS, tableToBoard },
  tcpComparison,
  get latestGripperObservation() { return latestGripperObservation ? structuredClone(latestGripperObservation) : null; },
  get rapierWorld() { return rapierWorld; },
};
