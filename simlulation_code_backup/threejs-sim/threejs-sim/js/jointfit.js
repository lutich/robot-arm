// Parts joined by a joint: the child part can only turn about one axis fixed in the parent part.
//
//   fitJointPair(joint, observations)   one frame: parent pose + joint angle (6 + 1 numbers) from the tags
//                                       of BOTH parts in all cameras, so each part's tags help the other
//   estimateJointAxis(joint, records)   several frames: the real axis in the parent part (the child's poses
//                                       relative to the parent are turns about one line), corrected tag
//                                       layouts for both parts, and the servo's angle offset and direction
//
// Model axes (kinematics.js fk): s5 = the gripper (hand) on the wrist, about the wrist's Z through (0, 0, L4).
// S4 (wrist on the forearm) works the same way once the forearm carries tags.

import { ARM, ARM_TAGS } from './config.js';
import { tagPoints, projectBoard, camWeight } from './armtrack.js';
import { rodrigues, rotvec, solve } from './calib.js';
import { fitPartPose } from './rigidtrack.js';

const sub = (a, b) => a.map((v, i) => v - b[i]);
const norm = a => { const l = Math.hypot(...a); return a.map(v => v / l); };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const mulRR = (A, B) => { const r = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j]; return r; };
const T3 = R => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
const apply = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const toRows = R => [[R[0], R[1], R[2]], [R[3], R[4], R[5]], [R[6], R[7], R[8]]];
const fromRows = M => [...M[0], ...M[1], ...M[2]];
const shifted = (q, k) => [0, 1, 2, 3].map(i => q[(i + k) % 4]);
const r3 = v => v.map(x => +x.toFixed(3));

/** R' = Q R, with Q the smallest rotation taking unit vector a to unit vector b. */
function rotationTaking(a, b, R = [1, 0, 0, 0, 1, 0, 0, 0, 1]) {
  const v = cross(a, b), c = dot(a, b);
  if (c < -0.999999) return mulRR([1, 0, 0, 0, -1, 0, 0, 0, -1], R);
  const k = 1 / (1 + c), V = [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];
  const V2 = mulRR(V, V);
  return mulRR([1, 0, 0, 0, 1, 0, 0, 0, 1].map((x, i) => x + V[i] + k * V2[i]), R);
}

export const JOINTS = {
  s5: { parent: 'wrist', child: 'hand', axis: [0, 0, 1], origin: [0, 0, ARM.L4], rot: a => { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; } },
};

/** The child part's pose for parent pose P and joint angle a (rad), per the model. */
export function childPose(joint, P, a) {
  const J = JOINTS[joint], o = apply(P.R, J.origin);
  return { R: mulRR(P.R, J.rot(a)), p: P.p.map((v, i) => v + o[i]) };
}

/**
 * Parent part and joint angle together (6 + 1 numbers) from the tags of both parts in all cameras.
 * Returns { joint, parent, child (poses), angle (deg), rms (px), views, cameras } or null without a child tag.
 */
export function fitJointPair(joint, observations, { init = null, iterations = 60 } = {}) {
  const J = JOINTS[joint];
  const tags = ARM_TAGS.filter(t => (t.link === J.parent || t.link === J.child) && t.id != null);
  const obs = [];
  observations.forEach((o, ci) => { for (const t of tags) if (o.tags?.[t.id]) obs.push({ ci, cal: o.cal, tag: t, q: o.tags[t.id].corners }); });
  if (!obs.some(o => o.tag.link === J.child)) return null;
  // start: the parent from its own fit (or from the child's fit and the model joint), the angle between them
  const pf = fitPartPose(J.parent, observations, { init: init?.parent });
  const cf = fitPartPose(J.child, observations, { init: init?.child });
  const P0 = pf || { R: cf.R.slice(), p: sub(cf.p, apply(cf.R, J.origin)) };
  let a0 = 0;
  if (cf && pf) { const Rr = mulRR(T3(P0.R), cf.R); a0 = Math.atan2(Rr[3], Rr[0]); }
  const shifts = { ...(pf?.shifts || {}), ...(cf?.shifts || {}) };
  const pack = (P, a) => [...rotvec(toRows(P.R)), ...P.p, a];
  const unpack = v => ({ P: { R: fromRows(rodrigues(v.slice(0, 3))), p: v.slice(3, 6) }, a: v[6] });
  const err = (v, weighted) => {
    const { P, a } = unpack(v), C = childPose(joint, P, a), r = [];
    for (const o of obs) {
      const pose = o.tag.link === J.parent ? P : C, m = tagPoints(o.tag, null, { [o.tag.link]: pose }).corners, q = shifted(o.q, shifts[o.tag.id] || 0);
      const w = weighted ? camWeight(o.cal) : 1;
      for (let i = 0; i < 4; i++) { const p = projectBoard(o.cal, m[i]); r.push(w * (p[0] - q[i][0]), w * (p[1] - q[i][1])); }
    }
    return r;
  };
  const sq = r => r.reduce((s, x) => s + x * x, 0);
  let v = pack(P0, a0), r = err(v, true), c = sq(r), mu = 1e-3;
  for (let it = 0; it < iterations; it++) {
    const Jm = r.map(() => new Array(7).fill(0));
    for (let k = 0; k < 7; k++) { const h = k < 3 || k === 6 ? 1e-5 : 1e-4, vv = v.slice(); vv[k] += h; const rr = err(vv, true); for (let i = 0; i < r.length; i++) Jm[i][k] = (rr[i] - r[i]) / h; }
    const JtJ = [...Array(7)].map((_, i) => [...Array(7)].map((__, j) => Jm.reduce((s, row) => s + row[i] * row[j], 0)));
    const Jtr = [...Array(7)].map((_, i) => Jm.reduce((s, row, k) => s + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((x, j) => x + (i === j ? mu * (JtJ[i][i] || 1) + 1e-12 : 0)));
      let dv; try { dv = solve(A, Jtr.map(x => -x)); } catch { mu *= 10; continue; }
      const vn = v.map((x, k) => x + dv[k]), rn = err(vn, true), cn = sq(rn);
      if (cn < c) { improved = c - cn > 1e-10 * c; v = vn; r = rn; c = cn; mu = Math.max(1e-12, mu / 5); if (!improved) break; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  const { P, a } = unpack(v), e = err(v, false);
  return { joint, parent: P, child: childPose(joint, P, a), angle: a * 180 / Math.PI, rms: Math.sqrt(sq(e) / (e.length / 2)), views: obs.length, cameras: new Set(obs.map(o => o.ci)).size };
}

/**
 * The joint's real axis from several frames in which both parts were fitted on their own (fitPartPose).
 *   records: [{ parent: pose, child: pose, commanded: model angle from the counts (deg, optional) }]
 * Needs 3 or more frames whose joint angles differ (30 degrees or more of spread is good).
 * Returns { axis, point (parent frame), angles (deg), spread, fit (cm), model: { axisDeg, offset (cm) },
 *   layout (corrected ARM_TAGS entries of both parts), servo: { slope, offset } or null }.
 */
export function estimateJointAxis(joint, records) {
  const J = JOINTS[joint];
  if (records.length < 2) throw new Error('need 2 or more recorded poses with different joint angles (3 or more are better)');
  const rel = records.map(r => ({ R: mulRR(T3(r.parent.R), r.child.R), p: apply(T3(r.parent.R), sub(r.child.p, r.parent.p)) }));
  // axis direction: the rotation axis of each relative rotation against the first, weighted by its angle
  let ax = [0, 0, 0];
  for (let i = 1; i < rel.length; i++) {
    const v = rotvec(toRows(mulRR(rel[i].R, T3(rel[0].R))));
    const s = dot(v, J.axis) < 0 ? -1 : 1;                  // same sense as the model axis
    ax = ax.map((x, k) => x + s * v[k]);
  }
  if (Math.hypot(...ax) < 0.1) throw new Error('the recorded poses barely differ in this joint; move it further between them');
  const a = norm(ax);
  // p_i = t + R_i k with t on the axis: least squares in (t, k); k's part along the axis is free (tiny pull)
  const A = [...Array(6)].map(() => new Array(6).fill(0)), b = new Array(6).fill(0);
  for (const { R, p } of rel) for (let row = 0; row < 3; row++) {
    const g = [row === 0 ? 1 : 0, row === 1 ? 1 : 0, row === 2 ? 1 : 0, R[3 * row], R[3 * row + 1], R[3 * row + 2]];
    for (let i = 0; i < 6; i++) { b[i] += g[i] * p[row]; for (let j = 0; j < 6; j++) A[i][j] += g[i] * g[j]; }
  }
  for (let i = 0; i < 6; i++) A[i][i] += 1e-6;
  const x = solve(A, b), t0 = x.slice(0, 3), k = x.slice(3, 6);
  const point = t0.map((v, i) => v + dot(sub(J.origin, t0), a) * a[i]);              // nearest to the model origin
  const fit = Math.sqrt(rel.reduce((s, { R, p }) => s + sub(p, t0.map((v, i) => v + apply(R, k)[i])).reduce((q, d) => q + d * d, 0), 0) / rel.length);
  // the parent correction: turn the measured axis onto the model axis and shift it onto the model line
  const Q = rotationTaking(a, J.axis);
  const Qp = apply(Q, point), d0 = sub(J.origin, Qp), shift = d0.map((v, i) => v - dot(d0, J.axis) * J.axis[i]);
  const toParentPt = v => apply(Q, v).map((y, i) => y + shift[i]);
  // per frame: the joint angle about the corrected axis, and the child tags in the corrected child frame
  const angles = [], childTags = ARM_TAGS.filter(t => t.link === J.child);
  const acc = Object.fromEntries(childTags.map(t => [t.key, { c: [0, 0, 0], n: [0, 0, 0], u: [0, 0, 0] }]));
  for (const { R, p } of rel) {
    const Rc = mulRR(Q, R), pc = toParentPt(p);
    const ang = Math.atan2(Rc[3], Rc[0]);
    angles.push(ang * 180 / Math.PI);
    const Hinv = T3(J.rot(ang));
    for (const t of childTags) {
      const cP = apply(Rc, t.centre).map((v, i) => v + pc[i]);
      const cC = apply(Hinv, sub(cP, J.origin)), nC = apply(Hinv, apply(Rc, t.normal)), uC = apply(Hinv, apply(Rc, t.up));
      for (let i = 0; i < 3; i++) { acc[t.key].c[i] += cC[i] / rel.length; acc[t.key].n[i] += nC[i] / rel.length; acc[t.key].u[i] += uC[i] / rel.length; }
    }
  }
  const layout = [
    ...ARM_TAGS.filter(t => t.link === J.parent).map(t => ({ ...t, centre: r3(toParentPt(t.centre)), normal: r3(norm(apply(Q, t.normal))), up: r3(norm(apply(Q, t.up))) })),
    ...childTags.map(t => ({ ...t, centre: r3(acc[t.key].c), normal: r3(norm(acc[t.key].n)), up: r3(norm(acc[t.key].u)) })),
  ];
  // the servo: measured angle against the commanded model angle
  let servo = null;
  const cmd = records.map(r => r.commanded);
  if (cmd.every(Number.isFinite)) {
    const n = cmd.length, mx = cmd.reduce((s, v) => s + v, 0) / n, my = angles.reduce((s, v) => s + v, 0) / n;
    const sxy = cmd.reduce((s, v, i) => s + (v - mx) * (angles[i] - my), 0), sxx = cmd.reduce((s, v) => s + (v - mx) ** 2, 0);
    if (sxx > 1e-6) { const slope = sxy / sxx; servo = { slope: +slope.toFixed(3), offset: +(my - slope * mx).toFixed(1) }; }
  }
  const off = sub(point, J.origin), offPerp = off.map((v, i) => v - dot(off, a) * a[i]);
  return {
    axis: r3(a), point: r3(point), angles: angles.map(v => +v.toFixed(1)), spread: +(Math.max(...angles) - Math.min(...angles)).toFixed(1), fit: +fit.toFixed(2),
    model: { axisDeg: +(Math.acos(Math.min(1, Math.abs(dot(a, J.axis)))) * 180 / Math.PI).toFixed(1), offset: +Math.hypot(...offPerp).toFixed(2) },
    layout, servo,
  };
}
