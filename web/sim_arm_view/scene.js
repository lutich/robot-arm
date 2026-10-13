// The arm scene of the page's 3D sections (3D arm viewer, Trajectory planning): renderer, desk and
// mat, the OAK camera with its viewing pyramid, the arm, cubes, path lines and the four views.
// Each section poses the arm, adds its own overlays and calls draw() from its animation frame.

import * as THREE from 'three';
import { createView } from './render.js';
import { MAT, CAMERA } from './config.js';
import { makeMaterials, buildArm, buildTable, buildCamera, buildCube } from './model/index.js';
import { defaultCalibration, applyPinhole, tableToBoard } from './camview.js';

const CAM_KEY = 'roboter-arm.sim.camera.v1';

/** The OAK camera fit both sections draw, from the mat markers; a refit is kept per browser. */
export function cameraCalibration() {
  if (!CAMERA.calib) {
    CAMERA.calib = defaultCalibration();
    try { const c = JSON.parse(localStorage.getItem(CAM_KEY)); if (c && c.R && c.f) CAMERA.calib = c; } catch { /* default fit */ }
  }
  return CAMERA.calib;
}

export function saveCameraCalibration(cal) {
  CAMERA.calib = cal;
  try { localStorage.setItem(CAM_KEY, JSON.stringify({ R: cal.R, t: cal.t, f: cal.f, cx: cal.cx, cy: cal.cy, centre: cal.centre, rms: cal.rms, H: cal.H })); } catch { /* not saved */ }
}

/** three.js rotation from a row-major 3x3 tuple, as the API reports frames. */
export const rotation = R => new THREE.Matrix4().set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);

const VIEWS = {
  orbit: (camera, controls) => { camera.fov = 36; camera.position.set(-30, 50, 30); controls.target.set(0, 11, 7); },
  top: (camera, controls) => { camera.fov = 36; camera.position.set(0, 15.5, 95); controls.target.set(0, 15.5, 0); },
  side: (camera, controls) => { camera.fov = 36; camera.position.set(75, 12, 14); controls.target.set(0, 12, 12); },
};

/**
 * canvas: the section's canvas; buttons: its view buttons, whose dataset[key] is orbit, top, side or oak.
 * Returns { scene, arm, placeMatAndCamera(), hideForCamera(fn), syncCubes(cubes), line(color),
 * dots(color, size), setPoints(object, points), invalidate(), draw(update), resize() }.
 */
export function createArmScene(canvas, buttons, key) {
  const view = createView(canvas);
  const { scene, camera, controls } = view;
  let dirty = true, oakView = false;
  const invalidate = () => { dirty = true; };
  VIEWS.orbit(camera, controls);
  controls.update();

  const M = makeMaterials();
  const table = buildTable(M); scene.add(table);
  const backdrop = table.getObjectByName('backdrop');
  const arm = buildArm(M); scene.add(arm.root);

  // ---------------- OAK camera ----------------
  cameraCalibration();
  let oak = null;
  const camPose = () => {
    const cal = CAMERA.calib, C = cal.centre, z = cal.R[2];
    const s = -C[2] / z[2];                                   // optical axis meets the mat
    return { pos: tableToBoard(C), target: tableToBoard([C[0] + s * z[0], C[1] + s * z[1], 0]) };
  };
  const hideForCamera = fn => { const v = oak.visible; oak.visible = false; fn(); oak.visible = v; };
  function applyOakView() {
    const r = canvas.getBoundingClientRect();
    applyPinhole(camera, CAMERA.calib, r.width, r.height);
    controls.target.set(...camPose().target);
  }
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
    if (oakView) applyOakView();
    invalidate();
  }
  placeMatAndCamera();

  // ---------------- cubes ----------------
  const cubeMeshes = new Map();
  function syncCubes(cubes) {
    for (const [id, m] of cubeMeshes) if (!cubes.some(c => c.id === id)) { scene.remove(m); cubeMeshes.delete(id); }
    for (const c of cubes) {
      let m = cubeMeshes.get(c.id);
      if (!m) { m = buildCube(c.colour); cubeMeshes.set(c.id, m); scene.add(m); }
      m.position.set(...c.p);
      if (c.R) m.setRotationFromMatrix(rotation(c.R));
      else m.rotation.set(0, 0, c.yaw * Math.PI / 180);
    }
  }

  // ---------------- path lines ----------------
  const line = color => { const l = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color })); l.frustumCulled = false; scene.add(l); return l; };
  const dots = (color, size) => { const d = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color, size })); d.frustumCulled = false; scene.add(d); return d; };
  const setPoints = (obj, pts) => { obj.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3)); obj.geometry.computeBoundingSphere(); invalidate(); };

  // ---------------- views ----------------
  function leaveOakView() {
    if (!oakView) return;
    oakView = false; controls.enabled = true;
    camera.fov = 36; camera.updateProjectionMatrix();
    buttons.forEach(x => x.classList.toggle('on', x.dataset[key] === 'orbit'));
  }
  buttons.forEach(b => b.onclick = () => {
    buttons.forEach(x => x.classList.toggle('on', x === b));
    if (b.dataset[key] === 'oak') { oakView = true; controls.enabled = false; applyOakView(); }
    else { leaveOakView(); VIEWS[b.dataset[key]](camera, controls); camera.updateProjectionMatrix(); controls.update(); }
    invalidate();
  });
  // dragging in the OAK view leaves it and orbits from where the camera is
  canvas.addEventListener('pointerdown', () => { if (oakView) { leaveOakView(); controls.update(); } }, true);
  const resize = () => { view.resize(); if (oakView) applyOakView(); invalidate(); };
  new ResizeObserver(resize).observe(canvas.parentElement);
  controls.addEventListener('change', invalidate);

  /** Render if anything changed; `update` poses the arm and overlays first. Returns whether it rendered. */
  function draw(update) {
    if (!oakView && controls.update()) dirty = true;
    if (!dirty) return false;
    update?.();
    if (backdrop) backdrop.visible = camera.position.y > backdrop.position.y + 2;
    if (oakView) hideForCamera(view.render); else view.render();
    dirty = false;
    return true;
  }

  return { scene, arm, placeMatAndCamera, hideForCamera, syncCubes, line, dots, setPoints, invalidate, draw, resize };
}
