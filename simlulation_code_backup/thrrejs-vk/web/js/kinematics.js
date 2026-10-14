// Forward and inverse kinematics, counts <-> angles, gripper linkage.
// Pure math, no three.js, so node can test it (web/test/).
// Model angles: 0 = all links straight up; positive S1 turns +Y toward +X;
// positive S2, S3, S4 tilt the arm toward +Y; S5 turns around the link axis.

import { ARM, CUBE, GRIPPER, SERVOS } from './config.js';

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;

// ---------------- small rigid-transform helpers (row-major 3x3 + translation) ----------------

export const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export function mul3(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
}
export const apply3 = (R, v) => [
  R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
  R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
  R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
];
export const transpose3 = R => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
export function rotX(a) { const c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; }
export function rotZ(a) { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; }

/** Frame = { R, p }. Child frame expressed in the parent: compose(parent, R, offset). */
export function compose(f, R, offset = [0, 0, 0]) {
  const o = apply3(f.R, offset);
  return { R: mul3(f.R, R), p: [f.p[0] + o[0], f.p[1] + o[1], f.p[2] + o[2]] };
}
export const point = (f, v) => { const o = apply3(f.R, v); return [f.p[0] + o[0], f.p[1] + o[1], f.p[2] + o[2]]; };
export const axis = (f, i) => [f.R[i], f.R[3 + i], f.R[6 + i]];

// ---------------- counts <-> model angles ----------------

export function angleFromCounts(servo, counts) {
  return servo.qHome + servo.dir * (counts - servo.home) / servo.cpd;
}
export function countsFromAngle(servo, deg) {
  return servo.home + servo.dir * (deg - servo.qHome) * servo.cpd;
}
/** Model angles {s1..s5} in degrees from a counts array [c0..c5]. */
export function anglesFromCounts(counts, servos = SERVOS) {
  return { s1: angleFromCounts(servos[0], counts[0]), s2: angleFromCounts(servos[1], counts[1]),
           s3: angleFromCounts(servos[2], counts[2]), s4: angleFromCounts(servos[3], counts[3]),
           s5: angleFromCounts(servos[4], counts[4]) };
}
/** Rounded counts for S1..S5 from model angles; S6 is passed through. */
export function countsFromAngles(a, s6, servos = SERVOS) {
  return [a.s1, a.s2, a.s3, a.s4, a.s5].map((q, i) => Math.round(countsFromAngle(servos[i], q))).concat([s6]);
}
/** Servos whose counts fall outside the controller's guards. */
export function guardViolations(counts, servos = SERVOS) {
  const bad = [];
  counts.forEach((c, i) => { if (c < servos[i].low || c > servos[i].high) bad.push(`${servos[i].name} ${c} outside ${servos[i].low}..${servos[i].high}`); });
  return bad;
}

// ---------------- gripper linkage ----------------

/** Link angle from vertical, degrees. Linear in counts between the measured end points. */
export function gripperTheta(counts, s6 = SERVOS[5]) {
  const t = (counts - s6.open) / (s6.closed - s6.open);
  return GRIPPER.thetaOpen + t * (GRIPPER.thetaClosed - GRIPPER.thetaOpen);
}
export const gapForTheta = th => Math.max(0, 2 * (GRIPPER.gear[0] + GRIPPER.link * Math.sin(th * RAD) + GRIPPER.tip[0]));
export const padDepthForTheta = th => GRIPPER.gear[1] + GRIPPER.link * Math.cos(th * RAD) + GRIPPER.tip[1];
export function thetaForGap(gap) {
  const s = (gap / 2 - GRIPPER.gear[0] - GRIPPER.tip[0]) / GRIPPER.link;
  return Math.asin(Math.max(-1, Math.min(1, s))) * DEG;
}
/** S6 counts at which the pads first touch a face of width w. */
export function countsForGap(w, s6 = SERVOS[5]) {
  const th = thetaForGap(w);
  return s6.open + (th - GRIPPER.thetaOpen) / (GRIPPER.thetaClosed - GRIPPER.thetaOpen) * (s6.closed - s6.open);
}

// ---------------- forward kinematics ----------------

/**
 * Frames of every joint for model angles a = {s1..s5} (degrees).
 * The hand frame sits at the flange; its +Z is the tool axis (pointing away from the
 * wrist), +X is the jaw closing axis, +Y the jaw depth axis.
 */
export function fk(a) {
  const world = { R: I3(), p: [0, 0, 0] };
  const base = compose(world, rotZ(-a.s1 * RAD));
  const shoulder = compose(base, rotX(-a.s2 * RAD), [0, 0, ARM.H_S2]);
  const elbow = compose(shoulder, rotX(-a.s3 * RAD), [0, 0, ARM.L2]);
  const wrist = compose(elbow, rotX(-a.s4 * RAD), [0, 0, ARM.L3]);
  const hand = compose(wrist, rotZ(a.s5 * RAD), [0, 0, ARM.L4]);
  const grasp = point(hand, [0, 0, ARM.G_BLOCK]);
  return { base, shoulder, elbow, wrist, hand, grasp };
}

/** World yaw (deg, CCW from +X) of the jaw closing axis, and how far the tool is from vertical. */
export function toolYawAndTilt(hand) {
  const x = axis(hand, 0), z = axis(hand, 2);
  return { yaw: Math.atan2(x[1], x[0]) * DEG, tilt: Math.acos(Math.max(-1, Math.min(1, -z[2]))) * DEG };
}

// ---------------- inverse kinematics (docs/pick-and-place-plan.md, section 3) ----------------

export class Unreachable extends Error {}

const wrap90 = d => ((d + 45) % 90 + 90) % 90 - 45;

/**
 * Top-down grasp: the held cube centre at (x, y, zc), gripper pointing straight down,
 * jaws square to a cube with world yaw `cubeYaw` (deg).
 * `s5Near` picks which of the equivalent S5 solutions (90 deg apart) to use; `servos`
 * limits that choice to the S5 guard range.
 * Returns model angles {s1..s5} plus the plan's intermediate values.
 */
export function ikTopDown(x, y, zc, { cubeYaw = 0, s5Near = 0, servos = SERVOS } = {}) {
  const { H_S2, L2, L3, L4, G_BLOCK } = ARM;
  const tool = L4 + G_BLOCK;
  const theta1 = Math.atan2(x, y) * DEG;
  const r = Math.hypot(x, y);
  const dz = zc + tool - H_S2;
  const d = Math.hypot(r, dz);
  if (!(Math.abs(L2 - L3) <= d && d <= L2 + L3))
    throw new Unreachable(`wrist point ${d.toFixed(2)} cm from S2, reach is ${Math.abs(L2 - L3).toFixed(1)} to ${(L2 + L3).toFixed(1)}`);
  const c3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  const q3 = -Math.acos(Math.max(-1, Math.min(1, c3)));                    // elbow-up
  const phi2 = Math.atan2(dz, r) - Math.atan2(L3 * Math.sin(q3), L2 + L3 * Math.cos(q3));
  const phi3 = phi2 + q3;
  const q4 = -Math.PI / 2 - phi3;

  // Jaw yaw with the tool down is -s1 - s5 (see fk); square to the cube every 90 deg.
  const base = wrap90(-theta1 - cubeYaw);
  const s5Servo = servos[4];
  const lo = Math.min(angleFromCounts(s5Servo, s5Servo.low), angleFromCounts(s5Servo, s5Servo.high));
  const hi = Math.max(angleFromCounts(s5Servo, s5Servo.low), angleFromCounts(s5Servo, s5Servo.high));
  let s5 = null;
  for (let k = -3; k <= 3; k++) {
    const c = base + 90 * k;
    if (c >= lo && c <= hi && (s5 === null || Math.abs(c - s5Near) < Math.abs(s5 - s5Near))) s5 = c;
  }
  if (s5 === null) throw new Unreachable('no wrist rotation inside the S5 guards');

  return {
    s1: theta1, s2: 90 - phi2 * DEG, s3: -q3 * DEG, s4: -q4 * DEG, s5,
    plan: { theta1, r, dz, phi2: phi2 * DEG, q3: q3 * DEG, phi3: phi3 * DEG, q4: q4 * DEG },
  };
}

/** Largest horizontal distance from S1 for a top-down grasp with the cube centre at zc. */
export function maxReach(zc = CUBE / 2) {
  const dz = zc + ARM.L4 + ARM.G_BLOCK - ARM.H_S2;
  return Math.sqrt((ARM.L2 + ARM.L3) ** 2 - dz * dz);
}

// ---------------- frames ----------------

/** Camera table frame (mm) -> board frame (cm), docs/reference-robot-arm-repo.md section 3. */
export const camToBoard = (X, Y, d) => [-Y / 10, X / 10 + d];
export const boardToCam = (x, y, d) => [(y - d) * 10, -x * 10];
