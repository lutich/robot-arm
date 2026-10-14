// Recalibration from captures (docs/recalibration.md): frames of both cameras, taken while the gripper holds a
// cube at a known spot on the mat (tools/capture.mjs --at ... --level ...). Each capture gives the arm's tags in
// the cameras and where the grip point really is; together they fit, besides each capture's joint angles,
// the model values that put the tags and the grip point on the arm:
//   wrist tags   one shift for all tags on the wrist part (wrist frame, cm): where they sit relative to S4 and S5
//   hand tags    one sideways shift for all tags on the gripper part (hand frame x, y, cm); their height along the
//                tool axis cannot be told apart from the grip length, so it stays as measured
//   grip         flange (S5 hub) to the centre of a held cube along the tool axis (ARM.G_BLOCK, cm)
//   mat offset   S1 axis to the mat's near edge (MAT.sToEdge, cm)
// Pure math (no page, no three.js): the Camera tab runs it, and test/recalib.test.mjs checks it on synthetic data.

import { ARM, ARM_TAGS, MAT } from './config.js';
import { fk, point, ikTopDown } from './kinematics.js';
import { fitArmPose, tagAxes, projectBoard, tableToBoardPt, boardToTable, camWeight } from './armtrack.js';
import { solve } from './calib.js';

const JOINTS = ['s1', 's2', 's3', 's4', 's5'];
/** The global values, in the order they are fitted, with their units and what they change. */
export const RECAL_PARAMS = [
  { key: 'wristX', unit: 'cm', what: 'wrist tags, shift along the wrist frame X (S4 axis direction)' },
  { key: 'wristY', unit: 'cm', what: 'wrist tags, shift along the wrist frame Y' },
  { key: 'wristZ', unit: 'cm', what: 'wrist tags, shift along the wrist frame Z (S4 axis toward the S5 hub)' },
  { key: 'handX', unit: 'cm', what: 'gripper tags, shift along the hand frame X (jaw axis)' },
  { key: 'handY', unit: 'cm', what: 'gripper tags, shift along the hand frame Y' },
  { key: 'grip', unit: 'cm', what: 'S5 hub to the centre of a held cube (ARM.G_BLOCK)' },
  { key: 'd', unit: 'cm', what: 'S1 axis to the mat\'s near edge (MAT.sToEdge)' },
];

/** The cube centre (table mm) for a target { at: [x, y] mm or [x, y, z], level }; z from the level if not given. */
export function targetPoint(target) {
  const [x, y, z] = target.at;
  return [x, y, z ?? 13.25 + 26.5 * (target.level || 0)];
}

/** Board-frame tag corners for angles, with the layout shifted by the global values g. */
function tagCorners(tag, frames, g) {
  const f = frames[tag.link];
  const { up, right } = tagAxes(tag);
  const c = tag.centre.slice();
  if (tag.link === 'wrist') { c[0] += g.wristX; c[1] += g.wristY; c[2] += g.wristZ; }
  if (tag.link === 'hand') { c[0] += g.handX; c[1] += g.handY; }
  const h = tag.size / 2, P = (u, v) => point(f, c.map((x, i) => x + u * right[i] + v * up[i]));
  return [P(-h, h), P(h, h), P(h, -h), P(-h, -h)];
}

/** Residuals of one capture: tag corners in px (weighted by camera quality), then the grip point (1 px per mm). */
function captureResiduals(cap, a, g, tags) {
  const frames = fk(a), r = [];
  for (const v of cap.views) {
    const w = camWeight(v.cal);
    for (const [id, t] of Object.entries(v.tags)) {
      const tag = tags[id]; if (!tag || !t.corners) continue;
      tagCorners(tag, frames, g).forEach((P, k) => { const q = projectBoard(v.cal, P, g.d); r.push(w * (q[0] - t.corners[k][0]), w * (q[1] - t.corners[k][1])); });
    }
  }
  if (cap.target) {
    const grip = boardToTable(point(frames.hand, [0, 0, g.grip]), g.d), T = targetPoint(cap.target);
    for (let i = 0; i < 3; i++) r.push(grip[i] - T[i]);
  }
  return r;
}

/** Joint angles of one capture from its tags alone (several starts: the arm may have been moved by hand). */
export function startAngles(cap, { d = MAT.sToEdge } = {}) {
  const tgt = cap.target ? tableToBoardPt(targetPoint(cap.target), d) : [0, 18, 3];
  const s1 = Math.atan2(tgt[0], tgt[1]) * 180 / Math.PI;
  const bases = [{ s2: 10, s3: 80, s4: 90 }, { s2: 40, s3: 60, s4: 70 }, { s2: -10, s3: 100, s4: 90 }];
  try { const ik = ikTopDown(...tgt); bases.unshift({ s2: ik.s2, s3: ik.s3, s4: ik.s4 }); } catch { /* beyond the vertical-gripper reach */ }
  let best = null;
  for (const base of bases)
    for (let s5 = -180; s5 < 180; s5 += 45) {
      try { const r = fitArmPose(cap.views, { init: { s1, ...base, s5 }, useCorners: true, prior: 90, d }); if (!best || r.rms < best.rms) best = r; } catch { /* too few tags */ }
    }
  return best ? best.angles : { s1, s2: 10, s3: 80, s4: 90, s5: 0 };
}

/**
 * How well tracking would place the grip point with the values g: each capture's angles fitted to its tags only
 * (as live tracking does), then the grip error against the target. Leaves the model as it was.
 */
export function evaluate(captures, g, startFrom = null) {
  const keep = { tags: ARM_TAGS.map(t => t.centre.slice()), grip: ARM.G_BLOCK, d: MAT.sToEdge };
  applyRecalibration(g);
  try {
    return captures.map((c, i) => {
      let r = null;
      for (const init of [startFrom?.[i], startAngles(c, { d: g.d })].filter(Boolean)) {
        try { const x = fitArmPose(c.views, { init, useCorners: true, prior: 90, d: g.d }); if (!r || x.rms < r.rms) r = x; } catch { /* too few tags */ }
      }
      if (!r) return { name: c.name, tagPx: null, gripMm: null };
      const grip = boardToTable(point(fk(r.angles).hand, [0, 0, g.grip]), g.d);
      const e = c.target ? grip.map((v, k) => v - targetPoint(c.target)[k]) : null;
      return { name: c.name, tagPx: +r.rms.toFixed(1), gripMm: e ? +Math.hypot(...e).toFixed(1) : null, gripErr: e && e.map(v => +v.toFixed(1)) };
    });
  } finally {
    ARM_TAGS.forEach((t, i) => { t.centre = keep.tags[i]; });
    ARM.G_BLOCK = keep.grip; MAT.sToEdge = keep.d;
  }
}

/** Root mean square of a list. */
const rms = a => Math.sqrt(a.reduce((s, v) => s + v * v, 0) / Math.max(1, a.length));

/**
 * Fits the global values (RECAL_PARAMS) and each capture's joint angles to all captures.
 *   captures: [{ name, views: [{ cal, tags: { id: { corners } } }], target: { at, level } | null }]
 *   fit: which global values to fit (default all); start: their start values (default: the model's)
 * Returns { values, change, before, after: [{ name, tagPx, gripMm, gripErr }] (angles from the tags only, as live
 * tracking finds them), joint (angles fitted to tags and target together), iterations, angles }.
 */
export function recalibrate(captures, { fit = RECAL_PARAMS.map(p => p.key), start = {}, iterations = 40 } = {}) {
  const tags = Object.fromEntries(ARM_TAGS.filter(t => t.id != null).map(t => [t.id, t]));
  const g0 = { wristX: 0, wristY: 0, wristZ: 0, handX: 0, handY: 0, grip: ARM.G_BLOCK, d: MAT.sToEdge, ...start };
  // each tag's centre too (the corners' mean), as the pose fit wants it
  const withCentres = v => ({ ...v, tags: Object.fromEntries(Object.entries(v.tags).map(([id, t]) => [id, { ...t, centre: t.centre || [0, 1].map(k => t.corners.reduce((s, c) => s + c[k], 0) / 4) }])) });
  const caps = captures.map(c => ({ ...c, views: c.views.map(withCentres) })).filter(c => c.views.some(v => Object.keys(v.tags).length));
  const A0 = caps.map(c => startAngles(c, { d: g0.d }));
  const n = fit.length, N = caps.length, nAll = n + 5 * N;
  const unpack = p => {
    const g = { ...g0 }; fit.forEach((k, i) => { g[k] = p[i]; });
    return { g, A: caps.map((_, c) => Object.fromEntries(JOINTS.map((j, k) => [j, p[n + 5 * c + k]]))) };
  };
  const report = p => {
    const { g, A } = unpack(p);
    return caps.map((c, i) => {
      const r = captureResiduals(c, A[i], g, tags), m = c.target ? 3 : 0;
      return { name: c.name, tagPx: +rms(r.slice(0, r.length - m)).toFixed(1), gripMm: c.target ? +Math.hypot(...r.slice(-3)).toFixed(1) : null };
    });
  };
  let p = [...fit.map(k => g0[k]), ...A0.flatMap(a => JOINTS.map(j => a[j]))];
  const before = evaluate(caps, g0, A0);
  // residual blocks per capture (only a capture's own angles change its block)
  const blocks = q => { const { g, A } = unpack(q); return caps.map((c, i) => captureResiduals(c, A[i], g, tags)); };
  const sq = bs => bs.reduce((s, b) => s + b.reduce((t, v) => t + v * v, 0), 0);
  let B = blocks(p), cost = sq(B), mu = 1e-3, it = 0;
  for (; it < iterations; it++) {
    const R = B.flat(), m = R.length, J = Array.from({ length: m }, () => new Float64Array(nAll));
    const offs = []; let o = 0; for (const b of B) { offs.push(o); o += b.length; }
    for (let k = 0; k < nAll; k++) {
      const h = k < n ? 1e-4 : 1e-3, q = p.slice(); q[k] += h;
      if (k < n) { const Bq = blocks(q).flat(); for (let i = 0; i < m; i++) J[i][k] = (Bq[i] - R[i]) / h; }
      else {                                                       // a capture's angle: only its block
        const c = Math.floor((k - n) / 5), { g, A } = unpack(q), bq = captureResiduals(caps[c], A[c], g, tags);
        for (let i = 0; i < bq.length; i++) J[offs[c] + i][k] = (bq[i] - R[offs[c] + i]) / h;
      }
    }
    const JtJ = Array.from({ length: nAll }, (_, a) => Array.from({ length: nAll }, (_, b) => { let s = 0; for (let i = 0; i < m; i++) s += J[i][a] * J[i][b]; return s; }));
    const Jtr = Array.from({ length: nAll }, (_, a) => { let s = 0; for (let i = 0; i < m; i++) s += J[i][a] * R[i]; return s; });
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const M = JtJ.map((row, a) => row.map((v, b) => v + (a === b ? mu * (JtJ[a][a] || 1) + 1e-9 : 0)));
      let dp; try { dp = solve(M, Jtr.map(v => -v)); } catch { mu *= 10; continue; }
      const pn = p.map((v, k) => v + dp[k]), Bn = blocks(pn), cn = sq(Bn);
      if (cn < cost) { improved = cost - cn > 1e-9 * cost; p = pn; B = Bn; cost = cn; mu = Math.max(1e-12, mu / 5); }
      else mu *= 10;
    }
    if (!improved) break;
  }
  const { g, A } = unpack(p);
  const values = Object.fromEntries(RECAL_PARAMS.map(({ key }) => [key, +g[key].toFixed(3)]));
  const change = Object.fromEntries(RECAL_PARAMS.map(({ key }) => [key, +(g[key] - g0[key]).toFixed(3)]));
  return { values, change, fitted: fit, before, after: evaluate(caps, g, A), joint: report(p), iterations: it, angles: A.map(a => Object.fromEntries(JOINTS.map(j => [j, +a[j].toFixed(1)]))) };
}

/**
 * The fitted values as config.js would hold them: the shifted tag centres (ARM_TAGS), the grip length and the
 * mat offset. apply(values) uses them in this session (ARM_TAGS, ARM.G_BLOCK, MAT.sToEdge).
 */
export function layoutFor(values) {
  return ARM_TAGS.filter(t => t.id != null).map(t => {
    const c = t.centre.slice();
    if (t.link === 'wrist') { c[0] += values.wristX; c[1] += values.wristY; c[2] += values.wristZ; }
    if (t.link === 'hand') { c[0] += values.handX; c[1] += values.handY; }
    return { key: t.key, id: t.id, link: t.link, centre: c.map(v => +v.toFixed(3)) };
  });
}
export function applyRecalibration(values) {
  for (const l of layoutFor(values)) ARM_TAGS.find(t => t.key === l.key).centre = l.centre;
  ARM.G_BLOCK = values.grip;
  MAT.sToEdge = values.d;
}
