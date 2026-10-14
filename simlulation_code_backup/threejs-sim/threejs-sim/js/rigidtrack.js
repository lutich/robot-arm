// Rigid parts of the arm from their AprilTags, without the arm model's joint chain.
//
// Tags on one part (config.js ARM_TAGS, ARM_PARTS) are fixed to each other, so together they are one
// object with a known layout. Its 6-D pose (rotation + position in the board frame) is fitted to the
// tag corners seen by every camera: one camera seeing one tag is enough (4 corners), more tags and
// cameras make it better, and two cameras seeing different tags of the part still meet in one pose.
//
//   squarePose(cal, corners, size)        one tag's pose from its 4 corners in one camera
//   fitPartPose(link, observations, init) the part's pose; also finds how each tag is turned on its
//                                         face (which corner is "up"), so that guess need not be right
//   learnTagLayout(link, observations, pose)  each tag's place on the part, measured (to fix ARM_TAGS)
//
// Frames: a pose is { R (row-major 3x3, board <- part), p (cm) }, like kinematics.js fk() frames.

import { ARM_TAGS } from './config.js';
import { tagPoints, projectBoard, tableToBoardPt, camWeight, tagAxes } from './armtrack.js';
import { rodrigues, rotvec, solve, homography } from './calib.js';

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sub = (a, b) => a.map((v, i) => v - b[i]);
const norm = a => { const l = Math.hypot(...a); return a.map(v => v / l); };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
/** Row-major 3x3 from three column vectors. */
const fromCols = (x, y, z) => [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
const col = (R, i) => [R[i], R[3 + i], R[6 + i]];
const mulRR = (A, B) => { const r = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j]; return r; };
const T3 = R => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
const apply = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const toRows = R => [[R[0], R[1], R[2]], [R[3], R[4], R[5]], [R[6], R[7], R[8]]];
const fromRows = M => [...M[0], ...M[1], ...M[2]];

/** A tag's axes in its link frame: right, up, normal as columns. */
function tagBasis(tag) {
  const { n, up, right } = tagAxes(tag);
  return fromCols(right, up, n);
}

/**
 * Pose of a square tag (side `size` cm) from its 4 corners (TL, TR, BR, BL as detected) in one camera:
 * { R (board <- tag; columns = the tag's right, up and normal), p (tag centre, board cm) } or null.
 */
export function squarePose(cal, corners, size) {
  const h = size / 2, local = [[-h, h], [h, h], [h, -h], [-h, -h]];
  const H = homography(local, corners);
  // K^-1 H = lambda [r1 r2 t] (camera frame, cm)
  const Ki = v => [(v[0] - cal.cx * v[2]) / cal.f, (v[1] - cal.cy * v[2]) / cal.f, v[2]];
  const c0 = Ki([H[0][0], H[1][0], H[2][0]]), c1 = Ki([H[0][1], H[1][1], H[2][1]]), c2 = Ki([H[0][2], H[1][2], H[2][2]]);
  let l = 2 / (Math.hypot(...c0) + Math.hypot(...c1));
  if (c2[2] * l < 0) l = -l;                                  // the tag lies in front of the camera
  let r1 = c0.map(v => v * l), r2 = c1.map(v => v * l);
  const t = c2.map(v => v * l);
  r1 = norm(r1); r2 = norm(sub(r2, r1.map(v => v * dot(r1, r2)))); const r3 = cross(r1, r2);
  // tag point (x, y, 0) cm -> camera cm -> table mm -> board cm
  const Rct = fromCols(r1, r2, r3);
  const toBoard = x => {
    const pc = apply(Rct, x).map((v, i) => 10 * (v + t[i]));            // camera frame, mm
    const d = pc.map((v, i) => v - cal.t[i]);
    const X = [0, 1, 2].map(k => cal.R[0][k] * d[0] + cal.R[1][k] * d[1] + cal.R[2][k] * d[2]);
    return tableToBoardPt(X);
  };
  const o = toBoard([0, 0, 0]);
  const ex = norm(sub(toBoard([1, 0, 0]), o)), ey = norm(sub(toBoard([0, 1, 0]), o));
  const ez = cross(ex, ey), ey2 = cross(ez, ex);
  const first = { R: fromCols(ex, ey2, ez), p: o };
  // Refine on the reprojection error, from the decomposition and from its mirror (a small tag seen at an
  // angle has two poses that look almost alike: the normal reflected about the viewing ray), keep the better.
  const cam = cameraCentreBoard(cal), ray = norm(sub(cam, o));
  const n = ez, m = sub(ray.map(v => 2 * dot(n, ray) * v), n);         // n reflected about the ray
  const flip = { R: rotationTaking(n, norm(m), first.R), p: o };
  let best = null;
  for (const start of [first, flip]) {
    const r = refinePose(start, P => {
      const out = [];
      local.forEach(([x, y], i) => { const q = projectBoard(cal, sub(P.p, [0, 0, 0]).map((v, k) => v + x * P.R[3 * k] + y * P.R[3 * k + 1])); out.push(q[0] - corners[i][0], q[1] - corners[i][1]); });
      return out;
    });
    if (!best || r.cost < best.cost) best = r;
  }
  return { R: best.R, p: best.p, rms: Math.sqrt(best.cost / 4) };
}

/** Camera centre in the board frame (cm). */
function cameraCentreBoard(cal) {
  const C = cal.centre || [0, 1, 2].map(i => -(cal.R[0][i] * cal.t[0] + cal.R[1][i] * cal.t[1] + cal.R[2][i] * cal.t[2]));
  return tableToBoardPt(C);
}

/** R' = Q R, with Q the smallest rotation taking unit vector a to unit vector b. */
function rotationTaking(a, b, R) {
  const v = cross(a, b), c = dot(a, b);
  if (c < -0.999999) return R.map((x, i) => (i % 3 === 2 ? x : -x));   // half turn: keep normal axis flipped
  const k = 1 / (1 + c), V = [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];
  const V2 = mulRR(V, V), Q = [1, 0, 0, 0, 1, 0, 0, 0, 1].map((x, i) => x + V[i] + k * V2[i]);
  return mulRR(Q, R);
}

/** Levenberg-Marquardt over a pose { R, p }; resid(pose) -> residuals. Returns { R, p, cost }. */
function refinePose(start, resid, iterations = 300) {
  const pack = P => [...rotvec(toRows(P.R)), ...P.p];
  const unpack = v => ({ R: fromRows(rodrigues(v.slice(0, 3))), p: v.slice(3, 6) });
  const sq = r => r.reduce((a, x) => a + x * x, 0);
  let v = pack(start), r = resid(unpack(v)), c = sq(r), mu = 1e-3;
  for (let it = 0; it < iterations; it++) {
    const J = r.map(() => new Array(6).fill(0));
    for (let k = 0; k < 6; k++) { const h = k < 3 ? 1e-5 : 1e-4, vv = v.slice(); vv[k] += h; const rr = resid(unpack(vv)); for (let i = 0; i < r.length; i++) J[i][k] = (rr[i] - r[i]) / h; }
    const JtJ = [...Array(6)].map((_, i) => [...Array(6)].map((__, j) => J.reduce((a, row) => a + row[i] * row[j], 0)));
    const Jtr = [...Array(6)].map((_, i) => J.reduce((a, row, k) => a + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((x, j) => x + (i === j ? mu * (JtJ[i][i] || 1) + 1e-12 : 0)));
      let dv; try { dv = solve(A, Jtr.map(x => -x)); } catch { mu *= 10; continue; }
      const vn = v.map((x, k) => x + dv[k]), rn = resid(unpack(vn)), cn = sq(rn);
      if (cn < c) { improved = c - cn > 1e-10 * c; v = vn; r = rn; c = cn; mu = Math.max(1e-12, mu / 5); if (!improved) break; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  return { ...unpack(v), cost: c };
}

/** The part pose that puts `tag` (in its part) at the tag pose `tp` (board): R = B T^T, p = o - R c. */
function partFromTag(tag, tp) {
  const R = mulRR(tp.R, T3(tagBasis(tag)));
  return { R, p: sub(tp.p, apply(R, tag.centre)) };
}

/** Detected corners turned by k quarter turns, to match the model's corner order. */
const shifted = (q, k) => [0, 1, 2, 3].map(i => q[(i + k) % 4]);

/**
 * The 6-D pose of one rigid part from its tags in all cameras.
 *   observations: [{ cal, tags: { [id]: { corners } } }]   (armtrack.js detectArmTags, per camera)
 *   init: a starting pose (e.g. from the commanded joint angles), or null to start from one tag's own pose
 * Returns { link, R, p, rms (px), views (tag views used), shifts: { [id]: k } } or null without a view.
 */
export function fitPartPose(link, observations, { init = null, iterations = 60 } = {}) {
  const tags = ARM_TAGS.filter(t => t.link === link && t.id != null);
  const obs = [];
  observations.forEach((o, ci) => { for (const t of tags) if (o.tags?.[t.id]) obs.push({ ci, cal: o.cal, tag: t, q: o.tags[t.id].corners }); });
  if (!obs.length) return null;
  const corners = (pose, tag) => tagPoints(tag, null, { [link]: pose }).corners;
  const err = (pose, o, k) => {
    const m = corners(pose, o.tag), q = shifted(o.q, k);
    let s = 0; for (let i = 0; i < 4; i++) { const p = projectBoard(o.cal, m[i]); s += (p[0] - q[i][0]) ** 2 + (p[1] - q[i][1]) ** 2; }
    return s;
  };
  // starting poses: the given one, and each tag's own pose in each camera with each of the 4 turns
  const starts = init ? [init] : [];
  for (const o of obs) for (let k = 0; k < 4; k++) {
    const sp = squarePose(o.cal, shifted(o.q, k), o.tag.size);
    if (sp) starts.push(partFromTag(o.tag, sp));
  }
  const bestShifts = pose => Object.fromEntries(tags.map(t => {
    const mine = obs.filter(o => o.tag === t);
    if (!mine.length) return [t.id, 0];
    let best = 0, be = Infinity;
    for (let k = 0; k < 4; k++) { const e = mine.reduce((s, o) => s + err(pose, o, k), 0); if (e < be) { be = e; best = k; } }
    return [t.id, best];
  }));
  const total = (pose, shifts) => obs.reduce((s, o) => s + err(pose, o, shifts[o.tag.id]), 0);
  // Pick the best start, then refine rotation and position (Levenberg-Marquardt, numeric Jacobian).
  // A single square tag cannot tell a quarter turn of the whole part about its normal: among starts that
  // fit about equally well, prefer the one nearest the given pose (the commanded joint angles), then the
  // one that keeps the tags as configured.
  const rotDeg = (A, B) => { let tr = 0; for (let i = 0; i < 9; i++) tr += A[i] * B[i]; return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI; };
  let pose = null, shifts = null, score = Infinity;
  for (const st of starts) {
    const sh = bestShifts(st), c = total(st, sh) / obs.length;
    // the configured layout is measured: a tag turned on its face only when the other tags clearly show it;
    // the commanded pose only breaks ties (it can be tens of degrees off before the HOME calibration)
    const sc = c + (init ? Math.min(3, 0.3 * (rotDeg(st.R, init.R) / 15) ** 2) : 0) + 25 * Object.values(sh).filter(k => k).length;
    if (sc < score) { score = sc; pose = st; shifts = sh; }
  }
  const pack = P => [...rotvec(toRows(P.R)), ...P.p];
  const unpack = v => ({ R: fromRows(rodrigues(v.slice(0, 3))), p: v.slice(3, 6) });
  const resid = v => {
    const P = unpack(v), r = [];
    // all tags of the part in all cameras together; each camera weighted by its calibration quality
    for (const o of obs) { const w = camWeight(o.cal), m = corners(P, o.tag), q = shifted(o.q, shifts[o.tag.id]); for (let i = 0; i < 4; i++) { const p = projectBoard(o.cal, m[i]); r.push(w * (p[0] - q[i][0]), w * (p[1] - q[i][1])); } }
    return r;
  };
  const sq = r => r.reduce((s, x) => s + x * x, 0);
  let v = pack(pose), r = resid(v), c = sq(r), mu = 1e-3;
  for (let it = 0; it < iterations; it++) {
    const J = r.map(() => new Array(6).fill(0));
    for (let k = 0; k < 6; k++) { const vv = v.slice(); vv[k] += k < 3 ? 1e-5 : 1e-4; const rr = resid(vv); for (let i = 0; i < r.length; i++) J[i][k] = (rr[i] - r[i]) / (k < 3 ? 1e-5 : 1e-4); }
    const JtJ = [...Array(6)].map((_, i) => [...Array(6)].map((__, j) => J.reduce((s, row) => s + row[i] * row[j], 0)));
    const Jtr = [...Array(6)].map((_, i) => J.reduce((s, row, k) => s + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((x, j) => x + (i === j ? mu * (JtJ[i][i] || 1) + 1e-12 : 0)));
      let dv; try { dv = solve(A, Jtr.map(x => -x)); } catch { mu *= 10; continue; }
      const vn = v.map((x, k) => x + dv[k]), rn = resid(vn), cn = sq(rn);
      if (cn < c) { improved = c - cn > 1e-10 * c; v = vn; r = rn; c = cn; mu = Math.max(1e-12, mu / 5); if (!improved) break; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  const fin = unpack(v); let e2 = 0;
  for (const o of obs) { const m = corners(fin, o.tag), q = shifted(o.q, shifts[o.tag.id]); for (let i = 0; i < 4; i++) { const p = projectBoard(o.cal, m[i]); e2 += (p[0] - q[i][0]) ** 2 + (p[1] - q[i][1]) ** 2; } }
  return { link, ...fin, rms: Math.sqrt(e2 / (4 * obs.length)), views: obs.length, cameras: new Set(obs.map(o => o.ci)).size, shifts };
}

/**
 * Where each seen tag actually sits on its part, given the part's fitted pose: { [id]: { centre, normal, up } }
 * in the part's frame, from the tag's own pose in the camera that sees it largest. With one well-placed
 * reference tag, this measures the others: copy the result into config.js ARM_TAGS.
 */
export function learnTagLayout(link, observations, pose, shifts = {}) {
  const out = {};
  for (const t of ARM_TAGS.filter(x => x.link === link && x.id != null)) {
    let best = null;
    for (const o of observations) {
      const q = o.tags?.[t.id]?.corners; if (!q) continue;
      const area = Math.abs((q[0][0] - q[2][0]) * (q[1][1] - q[3][1]) - (q[1][0] - q[3][0]) * (q[0][1] - q[2][1])) / 2;
      if (!best || area > best.area) best = { area, sp: squarePose(o.cal, shifted(q, shifts[t.id] || 0), t.size) };
    }
    if (!best?.sp) continue;
    const Rt = T3(pose.R), local = apply(Rt, sub(best.sp.p, pose.p));
    const r = c => col(best.sp.R, c), inPart = v => apply(Rt, v).map(x => +x.toFixed(3));
    out[t.id] = { centre: local.map(x => +x.toFixed(2)), up: inPart(r(1)), normal: inPart(r(2)) };
  }
  return out;
}
