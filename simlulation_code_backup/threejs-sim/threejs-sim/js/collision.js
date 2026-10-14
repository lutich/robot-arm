// Collision primitives and the arm's collision model. Pure math, no three.js.
//
// Shapes:
//   capsule  { kind: 'capsule', a: [x,y,z], b: [x,y,z], r }
//   box      { kind: 'box', c: [x,y,z], R: row-major 3x3 (columns = box axes in world), h: [hx,hy,hz] }
// Every query returns a signed separation in cm: > 0 apart, < 0 penetrating.
// Box-box uses the separating axis test, so its positive value is a lower bound of the
// distance; that is enough for "collides or not" with a small tolerance.

import { ARM, BASE, CUBE, GRIPPER } from './config.js';
import { apply3, transpose3, point, compose, I3, rotZ, padDepthForTheta, thetaForGap } from './kinematics.js';

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// ---------------- distances ----------------

/** Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9). */
export function segSegDist(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  let s, t;
  if (a <= 1e-12 && e <= 1e-12) return Math.sqrt(dot(r, r));
  if (a <= 1e-12) { s = 0; t = Math.min(1, Math.max(0, f / e)); }
  else {
    const c = dot(d1, r);
    if (e <= 1e-12) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
    else {
      const b = dot(d1, d2), den = a * e - b * b;
      s = den > 1e-12 ? Math.min(1, Math.max(0, (b * f - c * e) / den)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); }
      else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
    }
  }
  const c1 = [p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s];
  const c2 = [p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t];
  const d = sub(c1, c2);
  return Math.sqrt(dot(d, d));
}

/** Distance from a point given in box-local coordinates to the box (0 inside, negative depth inside). */
function localPointBoxDist(q, h) {
  const dx = Math.abs(q[0]) - h[0], dy = Math.abs(q[1]) - h[1], dz = Math.abs(q[2]) - h[2];
  const ox = Math.max(dx, 0), oy = Math.max(dy, 0), oz = Math.max(dz, 0);
  const outside = Math.sqrt(ox * ox + oy * oy + oz * oz);
  return outside > 0 ? outside : Math.max(dx, dy, dz); // negative inside: depth to the nearest face
}

/** Signed distance between segment a-b and a box. Convex in the segment parameter, so a golden-section search is exact enough. */
export function segBoxDist(a, b, box) {
  const RT = transpose3(box.R);
  const pa = apply3(RT, sub(a, box.c)), pb = apply3(RT, sub(b, box.c));
  const d = sub(pb, pa);
  const f = t => localPointBoxDist([pa[0] + d[0] * t, pa[1] + d[1] * t, pa[2] + d[2] * t], box.h);
  let lo = 0, hi = 1;
  const g = 0.6180339887;
  let x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo), f1 = f(x1), f2 = f(x2);
  for (let i = 0; i < 28; i++) {
    if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - g * (hi - lo); f1 = f(x1); }
    else { lo = x1; x1 = x2; f1 = f2; x2 = lo + g * (hi - lo); f2 = f(x2); }
  }
  return Math.min(f1, f2, f(0), f(1));
}

const boxAxis = (box, i) => [box.R[i], box.R[3 + i], box.R[6 + i]];

/** Separating axis test for two boxes. Returns the smallest overlap (negative = penetration depth) as -depth, or a positive separation. */
export function boxBoxSep(A, B) {
  const ax = [boxAxis(A, 0), boxAxis(A, 1), boxAxis(A, 2)];
  const bx = [boxAxis(B, 0), boxAxis(B, 1), boxAxis(B, 2)];
  const t = sub(B.c, A.c);
  let best = -Infinity;
  const test = L => {
    const n = Math.sqrt(dot(L, L));
    if (n < 1e-6) return false;
    const ra = A.h[0] * Math.abs(dot(ax[0], L)) + A.h[1] * Math.abs(dot(ax[1], L)) + A.h[2] * Math.abs(dot(ax[2], L));
    const rb = B.h[0] * Math.abs(dot(bx[0], L)) + B.h[1] * Math.abs(dot(bx[1], L)) + B.h[2] * Math.abs(dot(bx[2], L));
    const sep = (Math.abs(dot(t, L)) - ra - rb) / n;
    if (sep > best) best = sep;
    return sep > 0;
  };
  for (let i = 0; i < 3; i++) if (test(ax[i])) return best;
  for (let i = 0; i < 3; i++) if (test(bx[i])) return best;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) if (test(cross(ax[i], bx[j]))) return best;
  return best; // all axes overlap: best is minus the smallest overlap
}

/** Signed separation between any two shapes. */
export function separation(s1, s2) {
  if (s1.kind === 'capsule' && s2.kind === 'capsule') return segSegDist(s1.a, s1.b, s2.a, s2.b) - s1.r - s2.r;
  if (s1.kind === 'capsule') return segBoxDist(s1.a, s1.b, s2) - s1.r;
  if (s2.kind === 'capsule') return segBoxDist(s2.a, s2.b, s1) - s2.r;
  return boxBoxSep(s1, s2);
}

/** Height of the lowest point of a shape. */
export function lowestZ(s) {
  if (s.kind === 'capsule') return Math.min(s.a[2], s.b[2]) - s.r;
  const R = s.R;
  return s.c[2] - (Math.abs(R[6]) * s.h[0] + Math.abs(R[7]) * s.h[1] + Math.abs(R[8]) * s.h[2]);
}

/** Bounding sphere for the broad phase. */
export function bound(s) {
  if (s.kind === 'capsule') {
    const c = [(s.a[0] + s.b[0]) / 2, (s.a[1] + s.b[1]) / 2, (s.a[2] + s.b[2]) / 2];
    const d = sub(s.b, s.a);
    return { c, r: Math.sqrt(dot(d, d)) / 2 + s.r };
  }
  return { c: s.c, r: Math.sqrt(dot(s.h, s.h)) };
}

// ---------------- shape builders ----------------

/** Box from a frame, a centre offset in that frame, and half sizes. */
export function boxInFrame(f, offset, h, extraR = null) {
  const R = extraR ? multiplyR(f.R, extraR) : f.R;
  return { kind: 'box', c: point(f, offset), R, h };
}
function multiplyR(a, b) { return compose({ R: a, p: [0, 0, 0] }, b).R; }

export function capsuleInFrame(f, a, b, r) {
  return { kind: 'capsule', a: point(f, a), b: point(f, b), r };
}

/** A free or held cube as a box. Pose { p: [x,y,z] centre, R } or { p, yaw } for an upright cube. */
export function cubeShape(cube) {
  const R = cube.R || rotZ(cube.yaw * Math.PI / 180);
  return { kind: 'box', c: cube.p, R, h: [CUBE / 2, CUBE / 2, CUBE / 2] };
}

// Link indices: parts of neighbouring links always touch at their joint, so pairs whose
// link indices differ by less than 2 are not tested against each other.
export const LINK = { base: 0, turret: 1, upper: 2, fore: 3, wrist: 4, hand: 5, fingerL: 6, fingerR: 6 };

/**
 * Collision shapes of the arm for joint frames `f` (from fk) and the actual jaw gap.
 * Sizes are from the photos and the MG996R body (4,07 x 1,97 x 4,29 cm); they are not measured.
 */
export function armShapes(f, gap) {
  const s = [];
  const add = (name, link, shape) => { shape.name = name; shape.link = link; s.push(shape); };
  const world = { R: I3(), p: [0, 0, 0] };

  // Base: two U beams 9 x 8,4 x 2,5, the S1 servo and its bracket; does not turn with S1.
  add('base', LINK.base, boxInFrame(world, [0, BASE.offsetY, BASE.h / 2], [4.5, 4.2, BASE.h / 2]));   // plate centre 2,2 in front of S1
  add('S1 servo', LINK.base, boxInFrame(world, [-1.0, 0, 4.5], [3.1, 1.4, 2.0]));
  // Turret: bracket on the S1 horn with the S2 servo lying toward the back; turns with S1.
  add('turret', LINK.turret, boxInFrame(f.base, [0, -0.8, 8.95], [2.7, 2.6, 1.95]));
  // Upper arm: side plates at x = +-2,75 around the S3 servo, shortened at the joints.
  add('upper arm', LINK.upper, boxInFrame(f.shoulder, [0, 0, ARM.L2 / 2], [2.9, 1.3, ARM.L2 / 2 - 1.3]));
  // PCA9685 on its foam pad on the -X side plate, plugged connectors included (model/arm.js PCA)
  add('PCA9685', LINK.upper, boxInFrame(f.shoulder, [-4.35, -0.2, 4.5], [1.65, 1.7, 3.6]));
  // Forearm: aluminium tube, the elbow bracket, and the S4 servo in its bracket at the far end.
  add('S3 servo', LINK.fore, boxInFrame(f.elbow, [0, 0, -0.95], [2.1, 1.25, 2.55]));   // rides on the forearm, behind the elbow
  add('forearm', LINK.fore, capsuleInFrame(f.elbow, [0, 0, 1.9], [0, 0, ARM.L3 - 2.8], 0.95));
  add('S4 bracket', LINK.fore, boxInFrame(f.elbow, [0, -1.0, ARM.L3 - 0.9], [2.6, 2.1, 1.6]));
  // Wrist: U bracket from the S4 horn and the S5 servo body.
  add('wrist bracket', LINK.wrist, boxInFrame(f.wrist, [0, 0, 1.4], [2.9, 1.3, 1.3]));
  add('S5 servo', LINK.wrist, boxInFrame(f.wrist, [1.035, 0, ARM.L4 - 2.335], [2.04, 0.99, 1.84]));
  // Gripper base plate with the gears, and the S6 servo behind it.
  const pz0 = 0.3, pz1 = GRIPPER.gear[1] + 1.35;   // plate: from the flange down past the gears
  add('gripper plate', LINK.hand, boxInFrame(f.hand, [0, 0, (pz0 + pz1) / 2], [GRIPPER.plate.w / 2, 0.65, (pz1 - pz0) / 2]));
  // S6 body: shaft on the -X gear, 4,07 x 3,72 x 1,97 reaching toward -X and -Y (model/gripper.js)
  add('S6 servo', LINK.hand, boxInFrame(f.hand, [-GRIPPER.gear[0] - 1.035, GRIPPER.s6ShaftY - 1.86, GRIPPER.gear[1]], [2.04, 1.86, 0.99]));

  // Fingers: links from the gear pivot, finger body down to the pad, and the pad itself.
  const th = thetaForGap(gap) * Math.PI / 180;
  const vPad = padDepthForTheta(thetaForGap(gap));
  const { gear, link, pad } = GRIPPER;
  for (const side of [-1, 1]) {
    const name = side < 0 ? 'left' : 'right';
    const lk = side < 0 ? LINK.fingerL : LINK.fingerR;
    const gu = side * gear[0], ge = [gu + side * link * Math.sin(th), 0, gear[1] + link * Math.cos(th)];
    add(`${name} link`, lk, capsuleInFrame(f.hand, [gu, 0, gear[1]], ge, 0.35));
    const uIn = side * (gap / 2 + pad.t);
    add(`${name} finger`, lk, boxInFrame(f.hand, [uIn + side * 0.3, 0, (ge[2] + vPad + 0.7) / 2],
      [0.3, 0.4, (vPad + 0.7 - ge[2]) / 2]));
    add(`${name} pad`, lk, boxInFrame(f.hand, [side * (gap / 2 + pad.t / 2), 0, vPad], [pad.t / 2, pad.w / 2, pad.h / 2]));
  }
  for (const sh of s) sh.bound = bound(sh);
  return s;
}

// ---------------- scene queries ----------------

/**
 * All contacts between the arm, the board and the cubes.
 * arm: shapes from armShapes; cubes: [{ id, p, yaw|R, held }]; board: { w, d, axisToBackEdge }.
 * Returns [{ a, b, sep }] for every pair closer than -tol (penetrating by more than tol).
 */
/**
 * `margin` (cm, planning only): arm parts and a held cube must also keep this much clearance
 * from free cubes; a near miss closer than that counts as a contact.
 */
export function findContacts(arm, cubes, board, tol = 0.05, margin = 0) {
  const limitCube = Math.max(-tol, margin);
  const out = [];
  const near = (x, y, extra) => {
    const dx = x.bound.c[0] - y.bound.c[0], dy = x.bound.c[1] - y.bound.c[1], dz = x.bound.c[2] - y.bound.c[2];
    const rr = x.bound.r + y.bound.r + extra;
    return dx * dx + dy * dy + dz * dz <= rr * rr;
  };
  // Self collision
  for (let i = 0; i < arm.length; i++) for (let j = i + 1; j < arm.length; j++) {
    const A = arm[i], B = arm[j];
    if (Math.abs(A.link - B.link) < 2) continue;
    if (!near(A, B, 0)) continue;
    const sep = separation(A, B);
    if (sep < -tol) out.push({ a: A.name, b: B.name, sep });
  }
  // Board: every part except the base, and every cube, must stay above the board top.
  const onBoard = (x, y) => Math.abs(x) <= board.w / 2 && y >= -board.axisToBackEdge && y <= board.d - board.axisToBackEdge;
  for (const A of arm) {
    if (A.link === LINK.base) continue;
    const z = lowestZ(A);
    if (z < -tol && onBoard(A.bound.c[0], A.bound.c[1])) out.push({ a: A.name, b: 'board', sep: z });
  }
  const cubeShapes = cubes.map(c => { const s = cubeShape(c); s.name = `${c.id} cube`; s.held = !!c.held; s.bound = bound(s); return s; });
  // Arm against cubes. A held cube moves with the hand, so it is not tested against the gripper.
  for (const C of cubeShapes) for (const A of arm) {
    if (C.held && A.link >= LINK.hand) continue;
    if (!near(A, C, 0)) continue;
    const sep = separation(A, C);
    if (sep < limitCube) out.push({ a: A.name, b: C.name, sep });
  }
  // Cubes against each other and the board.
  for (let i = 0; i < cubeShapes.length; i++) {
    const C = cubeShapes[i];
    const z = lowestZ(C);
    if (z < -tol && onBoard(C.c[0], C.c[1])) out.push({ a: C.name, b: 'board', sep: z });
    for (let j = i + 1; j < cubeShapes.length; j++) {
      const D = cubeShapes[j];
      if (!near(C, D, 0)) continue;
      const sep = separation(C, D);
      if (sep < (C.held || D.held ? limitCube : -tol)) out.push({ a: C.name, b: D.name, sep });
    }
  }
  return out;
}
