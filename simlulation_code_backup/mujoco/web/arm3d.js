// 3D view of the arm, built from primitives to look like the real kit: black aluminium
// brackets with hole patterns, MG996R servos, servo cables, the aluminium forearm tube and
// the claw gripper. Follows the robot-arm API when one is reachable, otherwise it runs on
// its own with sliders. Only dependency: three.js (vendored).
import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { RoomEnvironment } from './vendor/RoomEnvironment.js';
import { CONFIG } from './arm3d-config.js';
import { angleOf, anglesOf, countOf, forward, gapForLinkAngle, gripperState, linkAngleForGap, World } from './arm3d-core.js';

const mm = (v) => v / 1000;
const DEG = Math.PI / 180;
const L = CONFIG.links, G = CONFIG.gripper;

// ---- materials ------------------------------------------------------------------------
const mat = (color, metalness, roughness, extra = {}) => new THREE.MeshStandardMaterial({ color, metalness, roughness, ...extra });
const coated = (color, metalness, roughness, clearcoat = 0.5, envMapIntensity = 0.35) => new THREE.MeshPhysicalMaterial({ color, metalness, roughness, clearcoat, clearcoatRoughness: 0.4, envMapIntensity });
const M = {
  black: coated(0x16181b, 0.15, 0.78, 0.0, 0.25),   // matte black anodised sheet
  servo: coated(0x1a1b1f, 0.05, 0.85, 0.0, 0.2),   // matte ABS servo case
  alu: mat(0xcfd3d8, 0.9, 0.55),
  screw: mat(0xd9dde2, 0.9, 0.5),
  hole: mat(0x0a0b0d, 0.0, 1.0),
  pad: mat(0xf0f1f1, 0.0, 0.85),
  board: mat(0x2e3137, 0.25, 0.6),
  plate: coated(0x111214, 0.15, 0.8, 0.0, 0.25),
  pcb: mat(0x2b5fb5, 0.1, 0.55),
  pink: mat(0xe95fa3, 0.1, 0.5),
  chip: mat(0x111216, 0.3, 0.45),
  label: mat(0xe0cb4f, 0.0, 0.7),
  cable: [mat(0xf08a2c, 0.0, 0.7), mat(0xc63a2c, 0.0, 0.7), mat(0x6b4529, 0.0, 0.7)],
  bearing: mat(0xc4c9d0, 0.9, 0.5),
  paper: mat(0xffffff, 0.0, 0.92),
  washer: mat(0xc8322b, 0.0, 0.6),
};

// ---- primitives -----------------------------------------------------------------------
function mesh(geometry, material, pos = [0, 0, 0]) {
  const m = new THREE.Mesh(geometry, material);
  m.position.set(...pos);
  m.castShadow = m.receiveShadow = true;
  return m;
}
const box = (w, d, h, material, pos) => mesh(new THREE.BoxGeometry(mm(w), mm(d), mm(h)), material, pos.map(mm));
// Cylinder with its axis along x, y or z.
function cyl(r, h, material, axis, pos, segments = 24) {
  const m = mesh(new THREE.CylinderGeometry(mm(r), mm(r), mm(h), segments), material, pos.map(mm));
  if (axis === 'z') m.rotation.x = Math.PI / 2;
  if (axis === 'x') m.rotation.z = Math.PI / 2;
  return m;
}
// ---- sheet-metal parts -----------------------------------------------------------------
// The kit is 2 mm black anodised aluminium sheet: plates with rounded ends and hole patterns.
// Shapes are drawn in (u, v) mm; `axis` is the plate normal, the sheet is centred on `pos`.
const HOLE = 1.7;              // M3 clearance
function pathCircle(u, v, r) {
  const path = new THREE.Path();
  path.absarc(mm(u), mm(v), mm(r), 0, Math.PI * 2, true);
  return path;
}
// Rectangle w x h (centred on u=0, v from v0 to v0+h) with optional semicircular ends.
function plateShape(w, v0, v1, { roundTop = false, roundBottom = false, r = null } = {}) {
  const R = r ?? w / 2;
  const sh = new THREE.Shape();
  const x = mm(w / 2);
  if (roundBottom) {
    sh.moveTo(-x, mm(v0 + R));
    sh.absarc(0, mm(v0 + R), mm(R), Math.PI, 0, false);
  } else {
    sh.moveTo(-x, mm(v0)); sh.lineTo(x, mm(v0));
  }
  if (roundTop) {
    sh.lineTo(x, mm(v1 - R));
    sh.absarc(0, mm(v1 - R), mm(R), 0, Math.PI, false);
  } else {
    sh.lineTo(x, mm(v1)); sh.lineTo(-x, mm(v1));
  }
  sh.closePath();
  return sh;
}
function sheet(parent, shape, t, axis, pos, holes = [], material = M.black) {
  for (const [u, v, r] of holes) shape.holes.push(pathCircle(u, v, r ?? HOLE));
  const g = new THREE.ExtrudeGeometry(shape, { depth: mm(t), bevelEnabled: false, curveSegments: 10 });
  g.translate(0, 0, -mm(t) / 2);
  if (axis === 'y') g.applyMatrix4(new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1));          // u->x, v->z, t->y
  else if (axis === 'x') g.applyMatrix4(new THREE.Matrix4().set(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));     // u->y, v->z, t->x
  const m = new THREE.Mesh(g, material);                                                                           // z: u->x, v->y
  m.position.set(...pos.map(mm));
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
function screw(parent, pos, axis = 'z', r = 1.9, washer = false) {
  const g = new THREE.Group();
  g.add(cyl(r, 1.8, M.screw, 'z', [0, 0, 0], 14));
  g.add(cyl(r * 0.45, 0.6, M.hole, 'z', [0, 0, 0.9], 6));      // hex socket
  if (washer) g.add(cyl(r + 1.4, 0.8, M.washer, 'z', [0, 0, -1.2], 16));   // red nylon washer, as in the kit
  if (axis === 'x') g.rotation.y = Math.PI / 2;
  if (axis === 'y') g.rotation.x = -Math.PI / 2;
  g.position.set(...pos.map(mm));
  parent.add(g);
}
function grid(n, m, pitch) {
  const out = [];
  for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) out.push([(i - (n - 1) / 2) * pitch, (j - (m - 1) / 2) * pitch]);
  return out;
}

// MG996R: 40.7 x 19.7 x 42.9 mm body. Local +z is the shaft; the horn sits at z = 0 and the body
// hangs below. The shaft is 10 mm from one end, so the body is offset along -x.
function servo({ horn = true, offset = true, wires = true } = {}) {
  const g = new THREE.Group();
  const [bw, bd, bh] = CONFIG.servo.body.map((v) => v * 1000);
  const cx = offset ? -(bw / 2 - 10) : 0;
  const top = -4;                                                     // boss between horn and body
  g.add(box(bw, bd, bh, M.servo, [cx, 0, top - bh / 2]));
  g.add(box(bw + 14, bd - 2, 2.5, M.servo, [cx, 0, top - 9]));       // mounting ears
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) screw(g, [cx + sx * (bw / 2 + 4.5), sy * 5, top - 7.6], 'z', 1.7, true);
  g.add(cyl(6.6, 4.2, M.servo, 'z', [0, 0, top - 2], 22));            // output boss
  g.add(cyl(4.6, 2.5, M.servo, 'z', [cx - 9, 0, top - 1.3], 18));     // second boss
  g.add(cyl(2.9, 5, M.alu, 'z', [0, 0, -1.5], 16));                    // spline
  if (horn) {
    g.add(cyl(10, 2, M.alu, 'z', [0, 0, 0], 28));
    for (const [u, v] of grid(2, 2, 10)) g.add(cyl(1.1, 2.4, M.hole, 'z', [u, v, 0], 8));
    g.add(cyl(2.4, 1.8, M.screw, 'z', [0, 0, 1.4], 12));
  }
  g.add(box(0.5, 13, 16, M.label, [cx + bw / 2 + 0.25, 0, top - bh / 2 - 4]));   // yellow label
  g.add(box(0.5, 13, 16, M.label, [cx - bw / 2 - 0.25, 0, top - bh / 2 - 4]));
  g.add(box(3, 6, 3.5, M.chip, [cx - bw / 2 - 1, 0, top - bh + 5]));    // cable exit
  g.userData.exit = anchor(g, [cx - bw / 2 - 3, 0, top - bh + 5]);
  return g;
}

// Cable anchors are empty objects on the moving groups; a loom is three wires through the
// anchors' world positions, rebuilt whenever the pose changes, so cables follow the joints.
function anchor(parent, pos) {
  const o = new THREE.Object3D();
  o.position.set(...pos.map(mm));
  parent.add(o);
  return o;
}
const looms = [];
const WIRES = [[0, 0], [1.3, 0.7], [-1.1, 1.3]];
function loom(anchors) {
  const meshes = WIRES.map((_, i) => { const m = new THREE.Mesh(new THREE.BufferGeometry(), M.cable[i]); m.castShadow = true; m.frustumCulled = false; scene.add(m); return m; });
  looms.push({ anchors, meshes });
}
function updateLooms() {
  const v = new THREE.Vector3();
  for (const { anchors, meshes } of looms) {
    const pts = anchors.map((a) => a.getWorldPosition(new THREE.Vector3()));
    WIRES.forEach(([ox, oy], i) => {
      const curve = new THREE.CatmullRomCurve3(pts.map((p) => p.clone().add(v.set(mm(ox), mm(oy), mm(ox * 0.5)))), false, 'centripetal', 0.6);
      const g = new THREE.TubeGeometry(curve, 10 * pts.length, mm(1.1), 6, false);
      meshes[i].geometry.dispose();
      meshes[i].geometry = g;
    });
  }
}

// Three servo wires along a fixed curve within one part.
function cable(parent, points, offsets = [[0, 0], [1.4, 0.6], [-1.2, 1.2]], segments = 48) {
  const base = points.map((p) => new THREE.Vector3(...p.map(mm)));
  offsets.forEach(([ox, oy], i) => {
    const pts = base.map((p) => p.clone().add(new THREE.Vector3(mm(ox), mm(oy), 0)));
    const curve = new THREE.CatmullRomCurve3(pts);
    const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, segments, mm(1.1), 8, false), M.cable[i]);
    tube.castShadow = true;
    parent.add(tube);
  });
}
function zipTie(parent, pos, axis, r = 4) {
  parent.add(cyl(r, 2.4, M.chip, axis, pos, 16));
}

// Orient a group so that its local x and z axes point along the given world directions.
function orient(group, xAxis, zAxis) {
  const x = new THREE.Vector3(...xAxis).normalize(), z = new THREE.Vector3(...zAxis).normalize();
  const y = new THREE.Vector3().crossVectors(z, x);
  group.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}

function gear(parent, pos, axis, r = 8.5, teeth = 16) {
  const g = new THREE.Group();
  g.position.set(...pos.map(mm));
  g.add(cyl(r, 2.4, M.plate, axis, [0, 0, 0], 32));
  for (let i = 0; i < teeth; i++) {
    const a = (i / teeth) * Math.PI * 2;
    const tooth = box(2.4, 1.5, 2.4, M.plate, [0, 0, 0]);
    if (axis === 'x') { tooth.position.set(0, mm(r * Math.cos(a)), mm(r * Math.sin(a))); tooth.rotation.x = a; }
    else if (axis === 'y') { tooth.position.set(mm(r * Math.cos(a)), 0, mm(r * Math.sin(a))); tooth.rotation.y = -a; }
    else { tooth.position.set(mm(r * Math.cos(a)), mm(r * Math.sin(a)), 0); tooth.rotation.z = a; }
    g.add(tooth);
  }
  parent.add(g);
  return g;
}

// Finger plate profile in the (sideways, down) plane of the finger body, mm: straight bar from
// the link end, then a knee bending inward, then the claw tip. Pads sit on the inner face.
const FINGER = [[-3, -3], [10, -1], [9, 20], [3, 42], [-7, 62], [-11, 67], [-13.5, 66], [-13, 57], [-5, 40], [-2, 20]];
const INLINE = (G.pads_axis || 'x') === 'x';     // jaws open in the arm plane (x) or across it (y)
function finger(side, layer = 0) {
  const s = side === 'left' ? -1 : 1;
  const shape = new THREE.Shape();
  FINGER.forEach(([u, v], i) => (i ? shape.lineTo(mm(s * u), mm(v)) : shape.moveTo(mm(s * u), mm(v))));
  shape.closePath();
  for (const [u, v] of [[1, 0], [3, 30], [-2, 52]]) shape.holes.push(pathCircle(s * u, v, 1.3));
  const geometry = new THREE.ExtrudeGeometry(shape, { depth: mm(2), bevelEnabled: false, curveSegments: 8 });
  if (INLINE) geometry.rotateX(Math.PI / 2);
  else geometry.applyMatrix4(new THREE.Matrix4().set(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));
  const m = new THREE.Mesh(geometry, M.black);
  if (INLINE) m.position.y = mm(layer); else m.position.x = mm(-layer);
  m.castShadow = m.receiveShadow = true;
  return m;
}
const sw = (along, across = 0) => (INLINE ? [along, across] : [across, along]);

// ---- the arm --------------------------------------------------------------------------
function buildArm() {
  const base = new THREE.Group();            // turns with S1
  base.position.set(CONFIG.base.x, CONFIG.base.y, 0);
  const foot = new THREE.Group();            // static
  foot.position.copy(base.position);
  const [pw, pd, pt] = CONFIG.base.plate.map((v) => v * 1000);
  const bigHoles = [[-28, 28, 4], [0, 28, 4], [28, 28, 4], [-28, -28, 4], [0, -28, 4], [28, -28, 4]];
  const smallHoles = [...grid(5, 2, 14).map(([u, v]) => [u, v * 2.5, HOLE]), ...grid(2, 7, 11).map(([u, v]) => [u * 3.6, v, HOLE])];

  // Two base plates with lips, one under the arm and one behind it, as in the photos.
  for (const dx of [0, -(pw + 2)]) {
    sheet(foot, plateShape(pw, -pd / 2, pd / 2, { r: 4 }), pt, 'z', [dx, 0, pt / 2], [...bigHoles, ...smallHoles]);
    foot.add(box(pw, 2, 10, M.black, [dx, -pd / 2 + 1, pt + 5]));
    foot.add(box(pw, 2, 10, M.black, [dx, pd / 2 - 1, pt + 5]));
  }

  // S1: servo standing in a U bracket, shaft up. Its horn carries the turntable disc.
  const s1 = servo({ offset: false });
  s1.rotation.z = Math.PI / 2;
  s1.position.z = mm(pt + 51);
  foot.add(s1);
  for (const sx of [-1, 1]) {
    sheet(foot, plateShape(30, 0, 50, { roundTop: true, r: 8 }), 2, 'x', [sx * 11.5, 0, pt + 2], [[0, 42, 2.2], [-8, 20], [8, 20], [0, 12]]);
    sheet(foot, plateShape(10, -15, 15), 2, 'z', [sx * 17.5, 0, pt + 1.5], [[0, -9], [0, 9]]);
  }
  base.add(cyl(24, 3, M.plate, 'z', [0, 0, pt + 54.5], 48));          // turntable disc
  for (const [u, v] of grid(2, 2, 14)) screw(base, [u, v, pt + 56.3], 'z');
  base.add(cyl(13, 2, M.bearing, 'z', [0, 0, pt + 57], 36));

  // Column: a U bracket standing on the disc, carrying the S2 servo with its shaft along +y.
  const h0 = L.shoulder_height * 1000;
  // Plate spacing: the servos sit inside the U brackets along their 43 mm height, so the
  // brackets are 46 mm wide inside; the upper arm plates ride outside the column plates.
  const COL = 23, ARM = 25;
  sheet(base, plateShape(46, -COL, COL), 2, 'z', [0, 0, pt + 59], [[-16, 0], [16, 0], [0, 0, 6]]);
  for (const sy of [-1, 1]) {
    sheet(base, plateShape(32, pt + 58, h0 + 16, { roundTop: true, r: 16 }), 2, 'y', [0, sy * COL, 0],
      [[0, h0, 2.5], [-7, h0 - 7], [7, h0 - 7], [-7, h0 + 7], [7, h0 + 7], ...grid(2, 2, 12).map(([u, v]) => [u, pt + 72 + v])]);
  }
  const s2 = servo();
  orient(s2, [0, 0, 1], [0, 1, 0]);          // shaft +y, the long end of the body hangs below the axis
  s2.position.set(0, mm(COL + 1), mm(h0));
  base.add(s2);
  base.add(cyl(4, 3, M.bearing, 'y', [0, -COL - 1, h0], 20));
  for (const z of [pt + 70, pt + 84]) screw(base, [-14, -COL - 1.1, z], 'y');
  zipTie(base, [-27, 4, pt + 48], 'z', 5);
  const baseRun = [anchor(base, [-30, 2, pt + 48]), anchor(base, [-28, 12, h0 - 20]), anchor(base, [-20, 24, h0 - 4])];

  // Shoulder: two long plates (the upper arm) bolted to the S2 horn, with the PCA9685 board.
  const shoulder = new THREE.Group();
  shoulder.position.z = L.shoulder_height;
  base.add(shoulder);
  const ua = L.upper_arm * 1000;
  for (const sy of [-1, 1]) {
    sheet(shoulder, plateShape(32, -16, ua + 16, { roundTop: true, roundBottom: true, r: 16 }), 2, 'y', [0, sy * ARM, 0],
      [[0, 0, 2.5], [0, ua, 2.5], [-7, -7], [7, -7], [-7, 7], [7, 7], [-7, ua - 7], [7, ua - 7], [-7, ua + 7], [7, ua + 7],
       ...grid(2, 5, 12).map(([u, v]) => [u * 0.9, ua / 2 + v])]);
  }
  for (const z of [30, ua - 30]) shoulder.add(box(26, 2 * ARM - 2, 2, M.black, [0, 0, z]));     // spacers
  for (const [u, v] of grid(2, 2, 14)) { screw(shoulder, [u, ARM + 1.2, v], 'y'); screw(shoulder, [u, -ARM - 1.2, v], 'y'); }
  // PCA9685 board in pink foam on the front plate.
  const pcb = new THREE.Group();
  pcb.position.set(0, mm(ARM + 2.2), mm(ua / 2 + 2));
  shoulder.add(pcb);
  pcb.add(box(30, 2.6, 62, M.pink, [0, 0, 0]));
  pcb.add(box(26, 1.6, 56, M.pcb, [0, 1.6, 0]));
  for (const [u, v] of grid(2, 6, 7)) pcb.add(box(2.2, 2.6, 2.2, [M.label, M.chip, M.cable[1]][(u > 0) + (v % 3 === 0)], [u * 1.6 + 6, 3.6, v]));
  for (const [u, v] of grid(3, 2, 8)) pcb.add(box(5, 1.2, 4, M.chip, [u - 7, 3, v + 14]));
  pcb.add(box(5, 4, 6, mat(0x2e9d4a, 0.1, 0.6), [-8, 4, -22]));                        // green terminal
  pcb.add(box(10, 1.6, 7, M.chip, [-7, 3, -4]));                                        // PCA9685 chip
  zipTie(shoulder, [-10, ARM + 5, ua / 2 - 36], 'z', 4);
  zipTie(shoulder, [-10, ARM + 5, ua / 2 + 38], 'z', 4);
  const pcbBottom = anchor(shoulder, [-12, ARM + 7, ua / 2 - 24]), pcbTop = anchor(shoulder, [-12, ARM + 7, ua / 2 + 26]);
  const armRun = [anchor(shoulder, [-14, ARM + 8, ua - 8]), anchor(shoulder, [-16, ARM + 9, ua / 2 + 42])];

  // Elbow: S3 servo between the plates, horn bolted to the front plate; the aluminium tube
  // starts on an L bracket behind the servo and ends in the U bracket that holds S4.
  const elbow = new THREE.Group();
  elbow.position.z = L.upper_arm;
  shoulder.add(elbow);
  const s3 = servo();
  orient(s3, [0, 0, -1], [0, 1, 0]);          // shaft +y on the front plate, body along the forearm
  s3.position.set(0, mm(ARM - 1), 0);
  elbow.add(s3);
  elbow.add(cyl(4, 3, M.bearing, 'y', [0, -ARM - 1, 0], 20));
  const fa = L.forearm * 1000;
  elbow.add(box(24, 24, 6, M.black, [0, 0, 36]));                                       // L bracket on the servo
  elbow.add(cyl(11, 3, M.alu, 'z', [0, 0, 40.5], 32));                                  // flange
  elbow.add(cyl(6.5, fa - 62, M.alu, 'z', [0, 0, 42 + (fa - 62) / 2], 28));            // tube
  elbow.add(cyl(11, 3, M.alu, 'z', [0, 0, fa - 20.5], 32));
  for (const z of [40.5, fa - 20.5]) for (const [u, v] of grid(2, 2, 12)) screw(elbow, [u, v, z + (z < fa / 2 ? -1.8 : 1.8)], 'z');
  sheet(elbow, plateShape(24, -12, 12), 2, 'z', [0, 0, fa - 19], [[-7, -7], [7, -7], [-7, 7], [7, 7], [0, 0, 3]]);
  sheet(elbow, plateShape(24, -ARM, ARM), 2, 'z', [0, 0, fa - 17], [[-7, -16], [7, -16], [-7, 16], [7, 16]]);
  for (const sy of [-1, 1]) {
    sheet(elbow, plateShape(24, fa - 18, fa + 12, { roundTop: true, r: 12 }), 2, 'y', [0, sy * ARM, 0], [[0, fa, 2.5], [-7, fa - 7], [7, fa - 7], [-7, fa + 7], [7, fa + 7]]);
  }
  zipTie(elbow, [-10, 9, fa / 2], 'z', 5);
  const tubeRun = [anchor(elbow, [-12, 10, fa - 10]), anchor(elbow, [-13, 11, fa / 2]), anchor(elbow, [-14, 12, 44]), anchor(elbow, [-18, 20, 6])];

  // Wrist: S4 servo with its body above the axis, horn on the elbow's U bracket; a hanging
  // bracket carries the S5 servo pointing down, then the bearing and the gripper flange.
  const wrist = new THREE.Group();
  wrist.position.z = L.forearm;
  elbow.add(wrist);
  const wl = L.wrist * 1000;
  const s4 = servo();
  orient(s4, [0, 0, -1], [0, 1, 0]);          // shaft +y on the elbow's plate, body hanging along the wrist link
  s4.position.set(0, mm(ARM - 1), 0);
  wrist.add(s4);
  wrist.add(cyl(4, 3, M.bearing, 'y', [0, -ARM - 1, 0], 20));
  // Hanging bracket bolted to the lower end of the S4 body; the S5 servo sits in it, shaft down.
  for (const sx of [-1, 1]) {
    sheet(wrist, plateShape(44, 12, wl - 8, { roundBottom: false, r: 8 }), 2, 'x', [sx * 11.5, -12, 0],
      [[-14, 20], [14, 20], [-14, wl - 16], [14, wl - 16], [0, wl / 2 + 4, 4]]);
  }
  const s5 = servo({ horn: false });
  orient(s5, [0, 1, 0], [0, 0, 1]);           // shaft down toward the flange, body behind the chain axis
  s5.position.set(0, 0, mm(wl - 3));
  wrist.add(s5);
  wrist.add(cyl(11, 3.5, M.bearing, 'z', [0, 0, wl - 2], 32));
  const wristRun = [anchor(wrist, [-16, -30, wl - 8]), anchor(wrist, [-18, -32, 14]), anchor(wrist, [-16, -24, -8])];

  // Flange: S5 roll. Gripper base plate with two gears, the S6 servo behind it, parallel
  // links and the claw fingers with rubber pads.
  const flange = new THREE.Group();
  flange.position.z = L.wrist;
  wrist.add(flange);
  flange.add(cyl(10, 2, M.alu, 'z', [0, 0, 1], 28));
  const gw = G.base_plate_width * 1000, gh = G.base_plate_height * 1000;
  const normal = INLINE ? 'y' : 'x';
  const P = (along, across, z) => [...sw(along, across), z];
  const gearU = G.gear_u * 1000, gearV = G.gear_v * 1000, outerU = G.outer_u * 1000, outerV = G.outer_v * 1000;
  const gripperPlate = plateShape(gw - 6, 2, gearV + 13, { r: 8 });
  sheet(flange, gripperPlate, 2.5, normal, P(0, -1.5, 0), [[-gearU, gearV, 2], [gearU, gearV, 2], [-outerU, outerV, 1.6], [outerU, outerV, 1.6], [-26, 8], [26, 8]]);
  sheet(flange, plateShape(gw - 22, 2, gearV + 13, { r: 8 }), 2, normal, P(0, 4.5, 0), [[-gearU, gearV, 2], [gearU, gearV, 2], [-outerU, outerV, 1.6], [outerU, outerV, 1.6]]);
  const s6 = servo({ horn: false, offset: false });
  if (INLINE) orient(s6, [1, 0, 0], [0, 1, 0]); else orient(s6, [0, 1, 0], [-1, 0, 0]);   // shaft through the plate, body behind it
  s6.position.set(...P(-gearU, -3, gearV).map(mm));
  flange.add(s6);
  gear(flange, P(-gearU, 1.5, gearV), normal, 8, 16);
  gear(flange, P(gearU, 1.5, gearV), normal, 8, 16);
  for (const sg of [-1, 1]) screw(flange, P(sg * outerU, 6, outerV), normal);
  screw(flange, P(gearU, 3, gearV), normal, 2.4);

  const jaws = {};
  for (const side of ['left', 'right']) {
    const s = side === 'left' ? -1 : 1;
    const jaw = new THREE.Group();
    jaw.position.set(...sw(s * G.gear_u, 0), G.gear_v);
    flange.add(jaw);
    const link = G.link_length * 1000;
    const bar = (along, across, z, len) => box(...(INLINE ? [6, 2, len] : [2, 6, len]), M.black, P(along, across, z));
    jaw.add(bar(0, 7, link / 2, link + 6));                                      // gear link, front layer
    const du = s * (outerU - gearU), dv = outerV - gearV;
    jaw.add(bar(du, 3.5, link / 2 + dv, link + 6));                              // outer link
    screw(jaw, P(0, 9.2, link), normal); screw(jaw, P(du, 5.2, dv + link), normal);
    const fingerGroup = new THREE.Group();
    fingerGroup.position.z = G.link_length;
    jaw.add(fingerGroup);
    fingerGroup.add(finger(side, 9.5));
    fingerGroup.add(finger(side, 0.5));
    const padT = G.pad_thickness * 1000, padH = G.pad_height * 1000;
    const padAlong = (-s * G.tip_inward + s * G.pad_thickness / 2) * 1000;
    fingerGroup.add(box(...(INLINE ? [padT, 10, padH] : [10, padT, padH]), M.pad, P(padAlong, 5, G.tip_down * 1000)));
    jaws[side] = { jaw, fingerGroup, s };
  }
  // Looms: gripper and wrist servos up the forearm to the board; base servos up the column.
  loom([s6.userData.exit, anchor(flange, P(-gearU - 30, -24, gearV + 6)), ...wristRun, s4.userData.exit, ...tubeRun, s3.userData.exit, ...armRun, pcbTop]);
  loom([s5.userData.exit, wristRun[1], wristRun[2], tubeRun[2], tubeRun[3], armRun[0], armRun[1], pcbTop]);
  loom([s1.userData.exit, ...baseRun, s2.userData.exit, anchor(shoulder, [-14, ARM + 6, 10]), pcbBottom]);
  return { base, foot, shoulder, elbow, wrist, flange, jaws };
}

// ---- blocks ---------------------------------------------------------------------------
function halfRoundGeometry(width, height, depth) {
  const r = (width * width / 4 + height * height) / (2 * height);
  const phi0 = Math.asin(Math.min(1, width / 2 / r));
  const shape = new THREE.Shape();
  shape.moveTo(-width / 2, 0);
  for (let i = 0; i <= 24; i++) {
    const phi = -phi0 + (2 * phi0 * i) / 24;
    shape.lineTo(r * Math.sin(phi), Math.max(0, height - r + r * Math.cos(phi)));
  }
  shape.lineTo(width / 2, 0);
  shape.closePath();
  const geometry = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false });
  geometry.translate(0, 0, -depth / 2);
  geometry.rotateX(Math.PI / 2);           // extrude along y, arc up along z
  return geometry;
}

function buildBlocks(scene) {
  const meshes = {};
  for (const spec of CONFIG.blocks) {
    const color = new THREE.Color().setRGB(...spec.color.slice(0, 3), THREE.SRGBColorSpace);   // config colours are sRGB
    const material = mat(color, 0.0, 0.7, { envMapIntensity: 0.3 });   // painted wood: matte, saturated
    let geometry;
    if (spec.kind === 'cube') geometry = new THREE.BoxGeometry(spec.size, spec.size, spec.size);
    else geometry = halfRoundGeometry(spec.width, spec.height, spec.depth);
    const m = new THREE.Mesh(geometry, material);
    m.castShadow = m.receiveShadow = true;
    scene.add(m);
    meshes[spec.name] = { mesh: m, kind: spec.kind, height: spec.kind === 'cube' ? spec.size : spec.height };
  }
  return meshes;
}

// ---- scene ----------------------------------------------------------------------------
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.VSMShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xe9ebee);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 10);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;

// A neutral room as environment: metals and the anodised plates reflect it, which is what
// makes black parts readable instead of flat. Lights: soft key with soft shadows, fill, rim.
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.6;
scene.add(new THREE.HemisphereLight(0xf4f6ff, 0xa8adb5, 0.55));
const sun = new THREE.DirectionalLight(0xfff3e6, 1.25);
sun.position.set(0.55, -0.7, 1.2);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
sun.shadow.camera.near = 0.1; sun.shadow.camera.far = 4;
sun.shadow.camera.left = sun.shadow.camera.bottom = -0.5;
sun.shadow.camera.right = sun.shadow.camera.top = 0.5;
sun.shadow.bias = -0.0003;
sun.shadow.normalBias = 0.002;
sun.shadow.radius = 10;
sun.shadow.blurSamples = 16;
scene.add(sun);
const fill = new THREE.DirectionalLight(0xe8eeff, 0.8);
fill.position.set(-0.9, 0.6, 0.5);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xffffff, 0.45);
rim.position.set(-0.3, 0.9, 0.9);
scene.add(rim);

// Ground, table board with a hole grid.
const ground = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: 0xe6e8eb, roughness: 0.95, metalness: 0 }));
ground.position.z = -CONFIG.table.thickness - 0.0005;
ground.receiveShadow = true;
scene.add(ground);
const T = CONFIG.table;
const board = mesh(new THREE.BoxGeometry(T.depth, T.width, T.thickness), M.board, [T.depth / 2, 0, -T.thickness / 2]);
scene.add(board);
const desk = mesh(new THREE.BoxGeometry(0.25, T.width + 0.3, T.thickness), new THREE.MeshStandardMaterial({ color: 0xe9ebee, roughness: 0.9 }), [-0.125, 0, -T.thickness / 2]);
scene.add(desk);
{
  const holes = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.002, 0.002, 0.0006, 10), M.hole, 11 * 9);
  const o = new THREE.Object3D();
  let i = 0;
  for (let a = 0; a < 11; a++) for (let b = 0; b < 9; b++) {
    const x = 0.03 + a * 0.032, y = -0.128 + b * 0.032;
    if (x > 0.005 && x < 0.355 && Math.abs(y) < 0.138) continue;      // under the printed sheet
    o.position.set(x, y, 0.0002);
    o.rotation.x = Math.PI / 2;
    o.updateMatrix();
    holes.setMatrixAt(i++, o.matrix);
  }
  scene.add(holes);
}

// The printed reference sheet of the camera calibration: 360 x 280 mm, 10 mm grid, six markers.
function paperTexture() {
  const px = 4, w = 360 * px, h = 280 * px;
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const g = c.getContext('2d');
  g.fillStyle = '#fbfbfa'; g.fillRect(0, 0, w, h);
  for (let x = 0; x <= 360; x += 5) {
    g.strokeStyle = x % 50 === 0 ? '#9a9da3' : x % 10 === 0 ? '#c9ccd1' : '#e4e6ea';
    g.lineWidth = x % 50 === 0 ? 2 : 1;
    g.beginPath(); g.moveTo(x * px, 0); g.lineTo(x * px, h); g.stroke();
  }
  for (let y = 0; y <= 280; y += 5) {
    g.strokeStyle = y % 50 === 0 ? '#9a9da3' : y % 10 === 0 ? '#c9ccd1' : '#e4e6ea';
    g.lineWidth = y % 50 === 0 ? 2 : 1;
    g.beginPath(); g.moveTo(0, y * px, 0); g.lineTo(w, y * px); g.stroke();
  }
  g.fillStyle = '#6f737a'; g.font = `${7 * px}px Helvetica, Arial, sans-serif`; g.textAlign = 'center';
  for (let x = 50; x <= 350; x += 50) g.fillText(String(x), x * px, (140 + 4) * px);
  // Markers: table (X away from arm, Y left) -> sheet pixels (u along X, v down = -Y).
  const bits = [[1, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 1, 0]];
  [[25, 110], [25, -110], [160, 110], [160, -110], [335, 110], [335, -110]].forEach(([X, Y], i) => {
    const u = X * px, v = (140 - Y) * px, s = 30 * px;
    g.fillStyle = '#111'; g.fillRect(u - s / 2, v - s / 2, s, s);
    g.fillStyle = '#fff';
    bits[i].forEach((b, k) => { if (b) g.fillRect(u - s / 2 + (k % 2 + 1) * s / 4 - s / 8, v - s / 2 + (Math.floor(k / 2) + 1) * s / 4 - s / 8, s / 4, s / 4); });
  });
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
{
  const sheet = new THREE.Mesh(new THREE.PlaneGeometry(0.36, 0.28), new THREE.MeshStandardMaterial({ map: paperTexture(), roughness: 0.92, metalness: 0 }));
  sheet.position.set(0.18, 0, 0.0004);
  sheet.receiveShadow = true;
  scene.add(sheet);
}

const arm = buildArm();
scene.add(arm.foot, arm.base);
const blockMeshes = buildBlocks(scene);
const world = new World(CONFIG);

// ---- cameras --------------------------------------------------------------------------
// Presets give a viewing direction; the distance is chosen so the whole arm fits the window,
// whatever its aspect ratio, and is recomputed on resize.
// Sphere around the arm itself in its current pose (base column to gripper), so the arm sits in
// the middle of the window and the camera orbits around it. Blocks are not part of the fit.
function focus() {
  const pts = [[CONFIG.base.x, CONFIG.base.y, 0], [CONFIG.base.x, CONFIG.base.y, L.shoulder_height]];
  const fk = forward(CONFIG, anglesOf(CONFIG, state ? state.visual : CONFIG.park));
  pts.push(fk.s3, fk.s4, fk.flange, fk.grasp);
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of pts) for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], p[i]); hi[i] = Math.max(hi[i], p[i]); }
  const centre = [0, 1, 2].map((i) => (lo[i] + hi[i]) / 2);
  const radius = Math.max(0.12, ...pts.map((p) => Math.hypot(p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]))) + 0.08;
  return { centre, radius };
}
const CAMS = {
  photo: { dir: [0.08, 1, 0.1], fov: 30 },        // side view like the product photo, arm reaching left
  iso: { dir: [0.75, 0.6, 0.45], fov: 36 },
  front: { dir: [1, 0, 0.25], fov: 34 },
  top: { dir: [0.001, 0, 1], fov: 36 },
  gripper: { dir: [0.9, 0.9, 0.25], fov: 28, centre: [0.17, 0, 0.08], radius: 0.09 },
};
let current = 'photo';
function fitDistance(fovDeg, radius) {
  const vfov = fovDeg * DEG;
  const hfov = 2 * Math.atan(Math.tan(vfov / 2) * camera.aspect);
  return (radius / Math.sin(Math.min(vfov, hfov) / 2)) * 1.08;
}
function setCamera(name) {
  current = name;
  if (name === 'oak' && CONFIG.oak) {
    const [x, y, z] = CONFIG.oak.pos;
    const xa = new THREE.Vector3(...CONFIG.oak.xyaxes.slice(0, 3)), ya = new THREE.Vector3(...CONFIG.oak.xyaxes.slice(3));
    const za = new THREE.Vector3().crossVectors(xa, ya);
    camera.position.set(x, y, z);
    camera.fov = CONFIG.oak.fovy;
    camera.up.copy(ya);
    controls.target.copy(new THREE.Vector3(x, y, z).sub(za.multiplyScalar(0.55)));
  } else {
    const c = CAMS[name] || CAMS.photo;
    const f = focus();
    const centre = new THREE.Vector3(...(c.centre || f.centre));
    const dir = new THREE.Vector3(...c.dir).normalize();
    camera.fov = c.fov;
    camera.up.set(0, 0, 1);
    camera.position.copy(centre.clone().add(dir.multiplyScalar(fitDistance(c.fov, c.radius || f.radius))));
    controls.target.copy(centre);
  }
  camera.updateProjectionMatrix();
  controls.update();
}
document.querySelectorAll('[data-cam]').forEach((b) => b.addEventListener('click', () => setCamera(b.dataset.cam)));
document.getElementById('fit').addEventListener('click', () => setCamera(current));
window.addEventListener('keydown', (e) => { if (e.key === 'f') setCamera(current); });

function resize() {
  renderer.setSize(window.innerWidth, window.innerHeight);     // CSS size follows the window, the buffer the pixel ratio
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  if (current !== 'oak' && typeof state !== 'undefined') setCamera(current);
}
window.addEventListener('resize', resize);

const eventLog = [];
let lastAngles = [];
// ---- state: API or standalone ---------------------------------------------------------
const params = new URLSearchParams(location.search);
const api = (params.get('api') || '').replace(/\/$/, '') || (location.pathname.startsWith('/arm3d') ? '' : null);
const startPose = params.get('pose') === 'home' ? CONFIG.home : CONFIG.park;   // ?pose=home for a standalone start at HOME
const state = { commanded: [...startPose], visual: [...startPose], known: false, online: false, text: '', simBlocks: null };
resize();
setCamera(new URLSearchParams(location.search).get('cam') || 'photo');
const hud = { joints: document.getElementById('joints'), status: document.getElementById('status'), events: document.getElementById('events') };
const NAMES = ['S1 base', 'S2 shoulder', 'S3 elbow', 'S4 wrist flex', 'S5 wrist rot', 'S6 gripper'];

async function poll() {
  if (api === null) return;
  try {
    const r = await fetch(`${api}/api/state`, { cache: 'no-store' });
    const s = await r.json();
    const commands = Object.values(s.commands);
    state.known = commands.every((c) => c !== null);
    if (state.known && !virtualOnly) state.commanded = commands.map(Number);
    state.online = true;
    state.last = s;
    state.text = `${s.mode}${s.armed ? ', armed' : ''}${s.busy ? ' · ' + (s.phase || 'moving') : ''}${s.error ? ' · ' + s.error : ''}`;
    try {
      const b = await fetch(`${api}/sim/blocks`, { cache: 'no-store' });
      state.simBlocks = b.ok && !virtualOnly ? await b.json() : null;
    } catch { state.simBlocks = null; }
  } catch {
    state.online = false;
    state.text = api ? `no API at ${api || location.origin}; standalone` : 'standalone';
  }
  if (!state.online) controlsNote.textContent = 'standalone: sliders move the model only';
}
setInterval(poll, 100);
poll();

// Sliders and pose buttons. Standalone they move the model; connected to an arm they send the
// API's own actions (move, go_pose), so the same controls drive sandbox and real arm.
async function action(path, body) {
  const r = await fetch(`${api}/api/${path}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) throw new Error(j.error || `${path}: HTTP ${r.status}`);
  return j.result;
}
const controlsBox = document.getElementById('controls');
const controlsNote = document.getElementById('controls-note');
function report(text) { eventLog.unshift(text); eventLog.length = Math.min(eventLog.length, 6); }
// Virtual-only mode: every control moves the 3D model and nothing reaches the arm.
// Default ON: the page starts safe and has to be switched to live on purpose, every time.
let virtualOnly = true;
const virtualBtn = document.getElementById('virtual');
function setVirtual(on) {
  virtualOnly = on;
  virtualBtn.classList.toggle('armed', on);
  controlsBox.classList.toggle('virtual', on);
  virtualBtn.textContent = on ? 'Virtual only' : 'LIVE';
  if (on) { state.commanded = state.visual.slice(); report('VIRTUAL: nothing is sent to the arm'); }
  else { const s = state.last || {}; if (s.mode === 'hardware' && !window.confirm('Switch to LIVE: the controls will move the REAL ARM. Keep the servo power switch within reach. Continue?')) return setVirtual(true); report('LIVE: controls drive the arm'); }
}
let virtualRun = null;                                   // a sequence being played on the model only
function playVirtual(positions, speed) {
  if (virtualRun) clearTimeout(virtualRun.timer);
  const run = { timer: null, i: 0 };
  virtualRun = run;
  const next = () => {
    if (run !== virtualRun || run.i >= positions.length) { if (run === virtualRun) virtualRun = null; return; }
    const target = positions[run.i++];
    const span = Math.max(...target.map((v, c) => Math.abs(v - state.commanded[c])));
    state.commanded = target.slice();
    run.timer = setTimeout(next, (1.5 * span / speed) * 1000 + 300);
  };
  next();
}
async function send(path, body, what) {
  if (virtualOnly) return virtualSend(path, body, what);
  try { await action(path, body); report(`sent ${what}`); } catch (e) { report(`refused ${what}: ${e.message}`); }
}
function virtualSend(path, body, what) {
  const s = state.last || {};
  if (path === 'move') { state.commanded[body.channel] = body.target; }
  else if (path === 'go_pose') { const j = s.joints || []; state.commanded = [0, 1, 2, 3, 4, 5].map((c) => (j[c] ? j[c][body.name.toLowerCase()] : (body.name === 'HOME' ? CONFIG.home : CONFIG.park)[c])); }
  else if (path === 'demo/pause') { if (virtualRun) { clearTimeout(virtualRun.timer); virtualRun = null; } }
  else return report(`virtual: ${what} ignored`);
  report(`virtual: ${what}`);
}
{
  const sliders = document.getElementById('sliders');
  const inputs = [];
  let dragging = -1;
  for (let c = 0; c < 6; c++) {
    const [lo, hi] = CONFIG.bounds[String(c)];
    const label = document.createElement('label');
    label.innerHTML = `<span>S${c + 1}</span><input type="range" min="${lo}" max="${hi}" value="${startPose[c]}"><span></span>`;
    const input = label.querySelector('input');
    const show = () => { label.lastElementChild.textContent = input.value; };
    input.addEventListener('pointerdown', () => { dragging = c; });
    input.addEventListener('input', () => { show(); if (!state.online || virtualOnly) state.commanded[c] = Number(input.value); });
    input.addEventListener('change', () => {                 // released: command the arm
      dragging = -1;
      if (state.online && !virtualOnly) send('move', { channel: c, target: Number(input.value), first_clear: false }, `S${c + 1} → ${input.value}`);
    });
    show();
    sliders.appendChild(label);
    inputs.push(input);
  }
  const set = (counts) => counts.forEach((v, c) => { inputs[c].value = v; inputs[c].dispatchEvent(new Event('input')); });
  const pose = (name) => (state.online ? send('go_pose', { name }, `go ${name}`) : set(name === 'HOME' ? CONFIG.home : CONFIG.park));
  virtualBtn.addEventListener('click', () => setVirtual(!virtualOnly));
  setVirtual(virtualOnly);
  const grip = (count) => (state.online ? send('move', { channel: 5, target: count, first_clear: false }, `gripper → ${count}`) : set([...state.commanded.slice(0, 5), count]));
  document.querySelector('[data-pose=home]').addEventListener('click', () => pose('HOME'));
  document.querySelector('[data-pose=park]').addEventListener('click', () => pose('PARK'));
  document.querySelector('[data-grip=open]').addEventListener('click', () => grip(G.open_count));
  document.querySelector('[data-grip=close]').addEventListener('click', () => grip(G.closed_count));
  document.getElementById('power').addEventListener('click', async () => {
    if (!state.online) return report('no arm connected');
    if (virtualOnly) return report('virtual: power ignored');
    const s = state.last || {};
    if (s.armed) return send('power_off', {}, 'power off');
    if (s.mode === 'hardware' && !window.confirm('REAL ARM: is it physically in PARK, supported, and the servo power switch within reach?')) return report('power on aborted');
    send('power_on', { park_confirmed: true }, 'power on');
  });
  document.getElementById('stop').addEventListener('click', () => (state.online ? send('stop', {}, 'STOP') : report('no arm connected')));
  document.getElementById('reset').addEventListener('click', async () => {
    if (state.online && state.simBlocks) {
      try { await fetch(`${api}/sim/reset`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }); } catch {}
    }
    world.reset();
  });
  // Sequences: the arm's own saved demos, played through the API's demo actions.
  const demoSelect = document.getElementById('demo-select'), speedSelect = document.getElementById('speed-select');
  const demoStatus = document.getElementById('demo-status');
  let listed = '';
  async function setSpeed() {
    const s = state.last || {};
    const speed = Number(speedSelect.value);
    if (s.settings && s.settings.demo_speed !== speed) await action('settings', { step_size: s.settings.step_size, demo_speed: speed });
  }
  async function runDemo(mode) {
    if (!state.online) return report('no arm connected');
    const filename = demoSelect.value;
    if (!filename) return report('choose a sequence first');
    const s = state.last || {};
    if (virtualOnly) {
      const loaded = (s.demo && s.demo.saved.find((d) => d.filename === filename)) || {};
      const positions = s.demo && s.demo.name === loaded.name && s.demo.positions.length ? s.demo.positions : null;
      if (!positions) return report('virtual: load the sequence once while live (Run), then play it here');
      playVirtual(positions.map((p) => [0, 1, 2, 3, 4, 5].map((c) => p.counts[c])), Number(speedSelect.value));
      return report(`virtual: playing ${loaded.name}`);
    }
    if (!s.armed) return report('power on first');
    try {
      await setSpeed();
      const loaded = (s.demo && s.demo.name) === demoSelect.selectedOptions[0].dataset.name && s.demo.positions.length;
      if (!loaded || mode === 'run') await action('demo/load', { filename });
      await action(mode === 'run' ? 'demo/run' : 'demo/next', {});
      report(`${mode === 'run' ? 'running' : 'next step of'} ${demoSelect.selectedOptions[0].dataset.name}`);
    } catch (e) { report(`refused: ${e.message}`); }
  }
  document.getElementById('demo-run').addEventListener('click', () => runDemo('run'));
  document.getElementById('demo-next').addEventListener('click', () => runDemo('next'));
  document.getElementById('demo-pause').addEventListener('click', () => (state.online ? send('demo/pause', {}, 'pause') : report('no arm connected')));
  speedSelect.addEventListener('change', () => { if (state.online) setSpeed().catch((e) => report(`refused: ${e.message}`)); });
  setInterval(() => {
    const s = state.last;
    if (!state.online || !s || !s.demo) { demoStatus.textContent = ''; return; }
    const key = JSON.stringify(s.demo.saved.map((d) => d.filename));
    if (key !== listed) {                                    // refill the list only when it changes
      listed = key;
      const current = demoSelect.value;
      demoSelect.innerHTML = '<option value="">sequence…</option>' + s.demo.saved.map((d) => `<option value="${d.filename}" data-name="${d.name}">${d.name} (${d.positions})</option>`).join('');
      demoSelect.value = current;
    }
    const d = s.demo;
    demoStatus.textContent = d.positions.length ? `${d.name}: ${d.status}, ${d.cursor}/${d.positions.length}${s.phase ? ' · ' + s.phase : ''}` : `${s.demo.saved.length} saved sequences`;
  }, 300);

  // Go to x, y: type the coordinates (table frame, mm) or pick them on the table, then the arm
  // travels above the point and lowers the open gripper to z. Served by /move_to on the sandbox
  // and by the proxy in front of the real arm.
  const toX = document.getElementById('to-x'), toY = document.getElementById('to-y'), toZ = document.getElementById('to-z');
  const toStatus = document.getElementById('to-status'), pickBtn = document.getElementById('to-pick');
  let picking = false;
  const targetMark = new THREE.Mesh(new THREE.RingGeometry(0.014, 0.019, 32), new THREE.MeshBasicMaterial({ color: 0x0a7d6b, side: THREE.DoubleSide, transparent: true, opacity: 0.85 }));
  targetMark.position.z = 0.0012; scene.add(targetMark);
  const showTarget = () => targetMark.position.set(Number(toX.value) / 1000, Number(toY.value) / 1000, 0.0012);
  for (const el of [toX, toY]) el.addEventListener('input', showTarget);
  showTarget();
  pickBtn.addEventListener('click', () => { picking = !picking; pickBtn.classList.toggle('armed', picking); canvas.classList.toggle('picking', picking); });
  const ray = new THREE.Raycaster(), tablePlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit = new THREE.Vector3();
  let down = null;
  canvas.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY]; });
  canvas.addEventListener('pointerup', (e) => {
    if (!picking || !down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
    ray.setFromCamera(new THREE.Vector2((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1), camera);
    if (!ray.ray.intersectPlane(tablePlane, hit)) return;
    toX.value = Math.round(hit.x * 1000); toY.value = Math.round(hit.y * 1000);
    showTarget();
    picking = false; pickBtn.classList.remove('armed'); canvas.classList.remove('picking');
    report(`target ${toX.value}, ${toY.value} mm`);
  });
  async function moveTo(lower) {
    if (!state.online) return report('no arm connected');
    if (!virtualOnly && !(state.last || {}).armed) return report('power on first');
    const body = { x: Number(toX.value) / 1000, y: Number(toY.value) / 1000, z: Number(toZ.value) / 1000, lower, dry: virtualOnly, path: document.getElementById('to-path').value };
    try {
      const r = await fetch(`${api}/move_to`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
      const j = await r.json().catch(() => ({}));
      if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
      if (virtualOnly) { playVirtual(j.result.steps.map((st) => st.counts), 40); return report(`virtual: ${lower ? 'go & lower' : 'go above'} ${toX.value}, ${toY.value} mm`); }
      report(`${lower ? 'go & lower' : 'go above'} ${toX.value}, ${toY.value} mm`);
    } catch (e) { report(`refused: ${e.message}`); }
  }
  document.getElementById('to-go').addEventListener('click', () => moveTo(true));
  document.getElementById('to-above').addEventListener('click', () => moveTo(false));
  document.getElementById('to-stop').addEventListener('click', async () => {
    if (!state.online) return report('no arm connected');
    try { await fetch(`${api}/move_to/stop`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }); report('stopped'); } catch (e) { report(`stop: ${e.message}`); }
  });
  setInterval(async () => {
    if (!state.online) { toStatus.textContent = ''; return; }
    try {
      const r = await fetch(`${api}/move_to/status`, { cache: 'no-store' });
      if (!r.ok) { toStatus.textContent = 'move_to not available on this server'; return; }
      const m = await r.json();
      const t = m.target ? ` (${(m.target[0] * 1000).toFixed(0)}, ${(m.target[1] * 1000).toFixed(0)}, ${(m.target[2] * 1000).toFixed(0)} mm)` : '';
      toStatus.textContent = m.running ? `moving: ${m.step}${t}` : m.error ? `failed: ${m.error}` : m.done ? `arrived${t}` : 'enter X, Y or pick them on the table';
    } catch {}
  }, 400);

  // Sliders follow the animated counts while a move plays, unless the user is dragging one.
  setInterval(() => {
    for (let c = 0; c < 6; c++) if (c !== dragging && (state.known || virtualOnly || !state.online)) { const v = Math.round(state.visual[c]); inputs[c].value = v; inputs[c].nextElementSibling.textContent = v; }
    if (!state.online) return;
    const s = state.last || {};
    document.getElementById('power').textContent = s.armed ? 'Power off' : 'Power on';
    controlsNote.textContent = virtualOnly ? 'VIRTUAL: the model only, nothing is sent' : s.armed ? 'sliders move the arm (20 counts/s)' : s.mode === 'hardware' ? 'REAL ARM, off. Power on enables the servos at PARK' : 'arm off: Power on first';
  }, 200);
}

// ---- animation ------------------------------------------------------------------------
const SPEED = 60;                 // counts per second when the view moves on its own
let last = performance.now();


function animate(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  // Follow the commanded counts: the API already interpolates, standalone sliders do not.
  for (let c = 0; c < 6; c++) {
    const d = state.commanded[c] - state.visual[c];
    const step = (state.online && !virtualOnly ? 400 : SPEED) * dt;
    state.visual[c] += Math.abs(d) <= step ? d : Math.sign(d) * step;
  }
  const angles = anglesOf(CONFIG, state.visual);
  // Fingers stop on a held block.
  const commandedLink = angles[5];
  let fk = forward(CONFIG, angles);
  if (world.held) { angles[5] = Math.max(angles[5], linkAngleForGap(CONFIG, world.widthAlong(world.held, fk.padAxis) || 0.0265)); fk = forward(CONFIG, angles); }
  arm.base.rotation.z = angles[0] * DEG;
  arm.shoulder.rotation.y = angles[1] * DEG;
  arm.elbow.rotation.y = angles[2] * DEG;
  arm.wrist.rotation.y = angles[3] * DEG;
  arm.flange.rotation.z = angles[4] * DEG;
  for (const side of ['left', 'right']) {
    const j = arm.jaws[side];
    if (INLINE) { j.jaw.rotation.y = j.s * angles[5] * DEG; j.fingerGroup.rotation.y = -j.s * angles[5] * DEG; }
    else { j.jaw.rotation.x = -j.s * angles[5] * DEG; j.fingerGroup.rotation.x = j.s * angles[5] * DEG; }
  }
  if (angles.some((a, i) => a !== lastAngles[i])) { scene.updateMatrixWorld(true); updateLooms(); lastAngles = angles.slice(); }
  // Blocks: MuJoCo positions when the sandbox provides them, otherwise the minimal physics.
  if (state.simBlocks) {
    for (const [name, b] of Object.entries(state.simBlocks)) {
      const wb = world.block(name);
      if (wb) { wb.x = b.x; wb.y = b.y; wb.z = b.z; wb.yaw = b.yaw; }
    }
  } else {
    for (const e of world.step(dt, gripperState(CONFIG, fk, commandedLink))) eventLog.unshift(e);
  }
  for (const b of world.blocks) {
    const m = blockMeshes[b.name];
    m.mesh.position.set(b.x, b.y, b.kind === 'cube' ? b.z : b.z - b.hz);
    m.mesh.rotation.z = b.yaw * DEG;
  }
  // HUD
  hud.joints.innerHTML = angles.map((a, c) => `<tr><td>${NAMES[c]}</td><td>${state.visual[c].toFixed(0)}</td><td>${a.toFixed(1)}°</td></tr>`).join('')
    + `<tr><td>claw tip x / y / z</td><td colspan="2">${fk.grasp.map((v) => (v * 1000).toFixed(0)).join(' / ')} mm</td></tr>`
    + `<tr><td>flange x / y / z</td><td colspan="2">${fk.flange.map((v) => (v * 1000).toFixed(0)).join(' / ')} mm</td></tr>`
    + `<tr><td>gap</td><td colspan="2">${(fk.gap * 1000).toFixed(1)} mm${world.held ? ' · holding ' + world.held.name : ''}</td></tr>`;
  hud.status.textContent = state.online ? `API: ${state.text}${state.simBlocks ? ' · blocks from MuJoCo' : ' · blocks: browser physics'}` : state.text;
  hud.events.innerHTML = eventLog.slice(0, 4).join('<br>');
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
