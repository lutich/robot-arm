// Rigid-body physics for the cubes. Pure math, no three.js, no library.
//
// - Cubes are dynamic boxes with full 3D rotation; their inertia is isotropic (a cube), so
//   the inverse inertia is a scalar.
// - Arm parts are kinematic colliders (boxes and capsules) with velocities taken from their
//   motion between substeps, so the arm can push, roll and topple cubes.
// - The board top (z = 0) and the desk around it (z = -board thickness) are planes.
// - Contacts: separating axis test, then face clipping (up to 8 points) for box-box,
//   vertex tests for the planes, closest points for capsule-box.
// - Solver: sequential impulses with accumulated clamping, Coulomb friction (two tangent
//   directions), speculative contacts and Baumgarte position correction. Restitution 0.
// - Bodies that stay still for a while go to sleep and wake on contact.
//
// Units: cm, s. Gravity 981 cm/s^2. Masses only matter relative to each other.

import { BOARD, CUBE } from './config.js';

export const PHYS = {
  h: 1 / 240,          // substep
  iterations: 14,
  gravity: 981,
  slop: 0.004,         // allowed penetration, cm
  beta: 0.25,          // position correction per substep
  maxBias: 30,         // cm/s
  speculative: 0.03,   // contacts are created this far before touching, cm
  sleepV: 0.6, sleepW: 0.25, sleepTime: 0.35,
  mu: { cube: 0.45, board: 0.4, desk: 0.4, arm: 0.5, pad: 0.9 },
};

// ---------------- small vector helpers (arrays) ----------------

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.sqrt(dot(a, a));
const norm = a => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const col = (R, i) => [R[i], R[3 + i], R[6 + i]];
const mv = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const mtv = (R, v) => [R[0] * v[0] + R[3] * v[1] + R[6] * v[2], R[1] * v[0] + R[4] * v[1] + R[7] * v[2], R[2] * v[0] + R[5] * v[1] + R[8] * v[2]];

/** Re-orthonormalise a rotation matrix (columns) after integration. */
export function orthonormalize(R) {
  let x = col(R, 0), y = col(R, 1);
  x = norm(x);
  y = norm(sub(y, mul(x, dot(x, y))));
  const z = cross(x, y);
  return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
}

/** Rotate R by angular velocity w (world) over h. */
function integrateR(R, w, h) {
  const a = len(w) * h;
  if (a < 1e-9) return R;
  const k = norm(w), c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  const Q = [t * k[0] * k[0] + c, t * k[0] * k[1] - s * k[2], t * k[0] * k[2] + s * k[1],
             t * k[0] * k[1] + s * k[2], t * k[1] * k[1] + c, t * k[1] * k[2] - s * k[0],
             t * k[0] * k[2] - s * k[1], t * k[1] * k[2] + s * k[0], t * k[2] * k[2] + c];
  const out = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) out[3 * i + j] = Q[3 * i] * R[j] + Q[3 * i + 1] * R[3 + j] + Q[3 * i + 2] * R[6 + j];
  return orthonormalize(out);
}

// ---------------- bodies ----------------

/** Make a cube a physics body (adds v, w, sleep fields). */
export function initBody(c, mass = 1) {
  c.v = c.v || [0, 0, 0];
  c.w = c.w || [0, 0, 0];
  c.invM = 1 / mass;
  c.invI = 6 / (mass * CUBE * CUBE);   // solid cube: I = m s^2 / 6 about any axis
  c.sleepT = c.sleepT || 0;
  return c;
}

const HALF = CUBE / 2;
const corners = (c, R, h = HALF) => {
  const out = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) out.push(add(c, mv(R, [sx * h, sy * h, sz * h])));
  return out;
};

// ---------------- contact generation ----------------
// A contact: { A, B, n, p, depth, mu, kin } — n points from A to B, B is always a dynamic cube.
// A is a dynamic cube or null (plane or kinematic arm part; then `kin` gives that surface's velocity at p).

function planeContacts(c, out) {
  for (const p of corners(c.p, c.R)) {
    const onBoard = Math.abs(p[0]) <= BOARD.w / 2 && p[1] >= -BOARD.axisToBackEdge && p[1] <= BOARD.d - BOARD.axisToBackEdge;
    const z0 = onBoard ? 0 : -BOARD.t;
    const depth = z0 - p[2];
    if (depth > -PHYS.speculative) out.push({ A: null, B: c, n: [0, 0, 1], p: [p[0], p[1], z0], depth, mu: onBoard ? PHYS.mu.board : PHYS.mu.desk, kin: [0, 0, 0], what: onBoard ? 'board' : 'desk' });
  }
}

/** Clip polygon (array of points) to the half space dot(p, n) <= d. */
function clip(poly, n, d) {
  const out = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const da = dot(a, n) - d, db = dot(b, n) - d;
    if (da <= 0) out.push(a);
    if ((da <= 0) !== (db <= 0)) out.push(add(a, mul(sub(b, a), da / (da - db))));
  }
  return out;
}

/**
 * Box-box contact manifold. Boxes: { c, R, h: [hx,hy,hz] }. Returns { n (A->B), pts: [{p, depth}] } or null.
 */
export function boxBoxManifold(A, B) {
  const a = [col(A.R, 0), col(A.R, 1), col(A.R, 2)], b = [col(B.R, 0), col(B.R, 1), col(B.R, 2)];
  const t = sub(B.c, A.c);
  let best = { overlap: Infinity };
  const test = (L, kind, i, j) => {
    const l = len(L);
    if (l < 1e-6) return true;
    L = mul(L, 1 / l);
    const ra = A.h[0] * Math.abs(dot(a[0], L)) + A.h[1] * Math.abs(dot(a[1], L)) + A.h[2] * Math.abs(dot(a[2], L));
    const rb = B.h[0] * Math.abs(dot(b[0], L)) + B.h[1] * Math.abs(dot(b[1], L)) + B.h[2] * Math.abs(dot(b[2], L));
    const dist = dot(t, L);
    const overlap = ra + rb - Math.abs(dist);
    if (overlap < -PHYS.speculative) return false;
    // prefer face axes over edge axes unless clearly better (stable manifolds)
    const score = kind === 'edge' ? overlap * 1.05 + 0.002 : overlap;
    if (score < best.score || best.overlap === Infinity) best = { overlap, score, n: dist < 0 ? mul(L, -1) : L, kind, i, j };
    return true;
  };
  for (let i = 0; i < 3; i++) if (!test(a[i], 'A', i)) return null;
  for (let i = 0; i < 3; i++) if (!test(b[i], 'B', i)) return null;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) if (!test(cross(a[i], b[j]), 'edge', i, j)) return null;

  const n = best.n;
  if (best.kind === 'edge') {
    // closest points between the two edges that touch
    const ea = add(A.c, [0, 1, 2].filter(k => k !== best.i).reduce((s, k) => add(s, mul(a[k], A.h[k] * Math.sign(dot(a[k], n)) || A.h[k])), [0, 0, 0]));
    const eb = add(B.c, [0, 1, 2].filter(k => k !== best.j).reduce((s, k) => add(s, mul(b[k], -B.h[k] * Math.sign(dot(b[k], n)) || B.h[k])), [0, 0, 0]));
    const d1 = a[best.i], d2 = b[best.j], r = sub(ea, eb);
    const bb = dot(d1, d2), den = 1 - bb * bb;
    let s = 0, u = 0;
    if (den > 1e-6) { s = (bb * dot(d2, r) - dot(d1, r)) / den; u = (dot(d2, r) + bb * s); }
    s = Math.max(-A.h[best.i], Math.min(A.h[best.i], s)); u = Math.max(-B.h[best.j], Math.min(B.h[best.j], u));
    const pa = add(ea, mul(d1, s)), pb = add(eb, mul(d2, u));
    return { n, pts: [{ p: mul(add(pa, pb), 0.5), depth: best.overlap }] };
  }
  // face contact: reference box R (owns the face), incident box I
  const refIsA = best.kind === 'A';
  const Rb = refIsA ? A : B, Ib = refIsA ? B : A, rAx = refIsA ? a : b, iAx = refIsA ? b : a;
  const nRef = refIsA ? n : mul(n, -1);               // reference face normal, pointing toward the incident box
  const k = best.i;
  const refC = add(Rb.c, mul(rAx[k], Rb.h[k] * Math.sign(dot(rAx[k], nRef))));
  // incident face: the face of I most anti-parallel to nRef
  let m = 0, md = 0;
  for (let q = 0; q < 3; q++) { const d = dot(iAx[q], nRef); if (Math.abs(d) > Math.abs(md)) { md = d; m = q; } }
  const fc = add(Ib.c, mul(iAx[m], -Ib.h[m] * Math.sign(md)));
  const [u1, u2] = [0, 1, 2].filter(q => q !== m);
  let poly = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([s1, s2]) => add(fc, add(mul(iAx[u1], s1 * Ib.h[u1]), mul(iAx[u2], s2 * Ib.h[u2]))));
  for (const q of [0, 1, 2].filter(q => q !== k)) {
    poly = clip(poly, rAx[q], dot(Rb.c, rAx[q]) + Rb.h[q]);
    poly = clip(poly, mul(rAx[q], -1), -dot(Rb.c, rAx[q]) + Rb.h[q]);
    if (!poly.length) return null;
  }
  const pts = [];
  for (const p of poly) {
    const sep = dot(sub(p, refC), nRef);
    if (sep < PHYS.speculative) pts.push({ p: sub(p, mul(nRef, sep / 2)), depth: -sep });
  }
  return pts.length ? { n, pts: reduce(pts) } : null;
}

/** Keep at most 4 contact points: the deepest and the ones spreading the area most. */
function reduce(pts) {
  if (pts.length <= 4) return pts;
  const out = [pts.reduce((m, p) => (p.depth > m.depth ? p : m))];
  while (out.length < 4) {
    let best = null, bd = -1;
    for (const p of pts) { if (out.includes(p)) continue; const d = Math.min(...out.map(o => len(sub(o.p, p.p)))); if (d > bd) { bd = d; best = p; } }
    out.push(best);
  }
  return out;
}

/** Capsule (segment a-b, radius r) against a box. Returns { n (capsule->box), p, depth } or null. */
function capsuleBox(cap, box) {
  const RT = (v) => mtv(box.R, sub(v, box.c));
  const pa = RT(cap.a), pb = RT(cap.b), d = sub(pb, pa);
  const clampP = q => [Math.max(-box.h[0], Math.min(box.h[0], q[0])), Math.max(-box.h[1], Math.min(box.h[1], q[1])), Math.max(-box.h[2], Math.min(box.h[2], q[2]))];
  const f = t => { const q = add(pa, mul(d, t)); return len(sub(q, clampP(q))); };
  let lo = 0, hi = 1;
  for (let i = 0; i < 30; i++) { const m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3; if (f(m1) < f(m2)) hi = m2; else lo = m1; }
  const t = (lo + hi) / 2, s = add(pa, mul(d, t)), q = clampP(s);
  let dist = len(sub(s, q)), nLocal;
  if (dist > 1e-6) nLocal = mul(sub(q, s), 1 / dist);          // from capsule axis toward box
  else {
    // axis inside the box: push out through the nearest face
    let bi = 0, bv = Infinity;
    for (let i = 0; i < 3; i++) { const v = box.h[i] - Math.abs(s[i]); if (v < bv) { bv = v; bi = i; } }
    nLocal = [0, 0, 0]; nLocal[bi] = -Math.sign(s[bi]) || 1; dist = -bv;
  }
  const depth = cap.r - dist;
  if (depth < -PHYS.speculative) return null;
  const n = mv(box.R, nLocal);
  const p = add(box.c, mv(box.R, q));
  return { n, p, depth };
}

// ---------------- solver ----------------

function velAt(body, r) { return body ? add(body.v, cross(body.w, r)) : null; }

function prepare(c, h) {
  const { A, B, n, p } = c;
  c.ra = A ? sub(p, A.p) : null;
  c.rb = sub(p, B.p);
  const kn = B.invM + B.invI * dot(cross(c.rb, n), cross(c.rb, n)) + (A ? A.invM + A.invI * dot(cross(c.ra, n), cross(c.ra, n)) : 0);
  c.mn = 1 / kn;
  const t1 = Math.abs(n[2]) < 0.9 ? norm(cross(n, [0, 0, 1])) : norm(cross(n, [1, 0, 0]));
  c.t = [t1, cross(n, t1)];
  c.mt = c.t.map(t => 1 / (B.invM + B.invI * dot(cross(c.rb, t), cross(c.rb, t)) + (A ? A.invM + A.invI * dot(cross(c.ra, t), cross(c.ra, t)) : 0)));
  c.bias = c.depth > 0 ? Math.min(PHYS.maxBias, PHYS.beta / h * Math.max(0, c.depth - PHYS.slop)) : c.depth / h; // speculative: allow closing up to contact
  c.jn = 0; c.jt = [0, 0];
}

function applyImpulse(c, J) {
  const { A, B } = c;
  B.v = add(B.v, mul(J, B.invM)); B.w = add(B.w, mul(cross(c.rb, J), B.invI));
  if (A) { A.v = sub(A.v, mul(J, A.invM)); A.w = sub(A.w, mul(cross(c.ra, J), A.invI)); }
}

function solve(c) {
  const vb = velAt(c.B, c.rb), va = c.A ? velAt(c.A, c.ra) : c.kin;
  const vr = sub(vb, va);
  // normal
  const vn = dot(vr, c.n);
  let dj = -(vn - c.bias) * c.mn;
  const jn0 = c.jn; c.jn = Math.max(0, jn0 + dj); dj = c.jn - jn0;
  if (dj) applyImpulse(c, mul(c.n, dj));
  // friction
  const vb2 = velAt(c.B, c.rb), va2 = c.A ? velAt(c.A, c.ra) : c.kin, vr2 = sub(vb2, va2);
  const lim = c.mu * c.jn;
  for (let k = 0; k < 2; k++) {
    const vt = dot(vr2, c.t[k]);
    let d = -vt * c.mt[k];
    const j0 = c.jt[k]; c.jt[k] = Math.max(-lim, Math.min(lim, j0 + d)); d = c.jt[k] - j0;
    if (d) applyImpulse(c, mul(c.t[k], d));
  }
}

// ---------------- one physics step ----------------

/**
 * Advance the dynamic cubes by h.
 * cubes: bodies with { p, R, v, w, state: 'awake' | 'asleep' | 'held' }.
 * kin: kinematic arm shapes for this substep, each { kind, ..., name, prev } where prev is the
 *      same shape h earlier (for surface velocities); pads have `pad: true`.
 * Returns the arm-cube contacts that carried force: [{ part, cube, depth, impulse }].
 */
export function stepPhysics(cubes, kin, h) {
  const dyn = cubes.filter(c => c.state === 'awake' || c.state === 'asleep');
  const contacts = [];
  // arm parts against cubes (wake a sleeping cube the arm reaches)
  for (const c of dyn) {
    const box = { c: c.p, R: c.R, h: [HALF, HALF, HALF] };
    for (const s of kin) {
      const dx = s.bound.c[0] - c.p[0], dy = s.bound.c[1] - c.p[1], dz = s.bound.c[2] - c.p[2], rr = s.bound.r + HALF * 1.74 + PHYS.speculative;
      if (dx * dx + dy * dy + dz * dz > rr * rr) continue;
      let hits = [];
      if (s.kind === 'box') {
        const m = boxBoxManifold(s, box);
        if (m) hits = m.pts.map(q => ({ n: m.n, p: q.p, depth: q.depth }));
      } else {
        const m = capsuleBox(s, box);
        if (m) hits = [m];
      }
      for (const hit of hits) {
        const kv = kinVelocity(s, hit.p, h);
        if (c.state === 'asleep' && (hit.depth > 0 || len(kv) > 0.5)) wake(c);
        contacts.push({ A: null, B: c, n: hit.n, p: hit.p, depth: hit.depth, mu: s.pad ? PHYS.mu.pad : PHYS.mu.arm, kin: kv, part: s.name });
      }
    }
  }
  // cube against cube, and cubes against the planes
  for (let i = 0; i < dyn.length; i++) {
    const a = dyn[i];
    if (a.state === 'awake') planeContacts(a, contacts);
    for (let j = i + 1; j < dyn.length; j++) {
      const b = dyn[j];
      if (a.state === 'asleep' && b.state === 'asleep') continue;
      const d = sub(b.p, a.p);
      if (dot(d, d) > (CUBE * 1.75) ** 2) continue;
      const m = boxBoxManifold({ c: a.p, R: a.R, h: [HALF, HALF, HALF] }, { c: b.p, R: b.R, h: [HALF, HALF, HALF] });
      if (!m) continue;
      // A resting cube is only woken by a neighbour that really moves or pushes into it; otherwise
      // it acts as a fixed support. (Waking it on any touch makes two settling neighbours wake each
      // other in turn, so neither ever sleeps.)
      const moving = c => len(c.v) > PHYS.sleepV * 3 || len(c.w) > PHYS.sleepW * 3;
      const pushing = m.pts.some(q => q.depth > 0.03);
      if (a.state === 'asleep' && (moving(b) || pushing)) wake(a);
      if (b.state === 'asleep' && (moving(a) || pushing)) wake(b);
      for (const q of m.pts) {
        if (a.state === 'awake' && b.state === 'awake') contacts.push({ A: a, B: b, n: m.n, p: q.p, depth: q.depth, mu: PHYS.mu.cube, kin: null });
        else if (b.state === 'awake') contacts.push({ A: null, B: b, n: m.n, p: q.p, depth: q.depth, mu: PHYS.mu.cube, kin: [0, 0, 0] });              // a rests
        else if (a.state === 'awake') contacts.push({ A: null, B: a, n: m.n.map(v => -v), p: q.p, depth: q.depth, mu: PHYS.mu.cube, kin: [0, 0, 0] }); // b rests
      }
    }
  }
  // held cubes act like arm parts on free cubes
  for (const hc of cubes.filter(c => c.state === 'held')) {
    for (const c of dyn) {
      const d = sub(c.p, hc.p);
      if (dot(d, d) > (CUBE * 1.75) ** 2) continue;
      const m = boxBoxManifold({ c: hc.p, R: hc.R, h: [HALF, HALF, HALF] }, { c: c.p, R: c.R, h: [HALF, HALF, HALF] });
      if (!m) continue;
      if (c.state === 'asleep') wake(c);
      for (const q of m.pts) contacts.push({ A: null, B: c, n: m.n, p: q.p, depth: q.depth, mu: PHYS.mu.cube, kin: hc.v || [0, 0, 0], part: `${hc.id} cube (held)` });
    }
  }

  const awake = dyn.filter(c => c.state === 'awake');
  for (const c of awake) c.v = add(c.v, [0, 0, -PHYS.gravity * h]);
  const active = contacts.filter(c => c.B.state === 'awake');
  for (const c of active) prepare(c, h);
  for (let it = 0; it < PHYS.iterations; it++) for (const c of active) solve(c);
  for (const c of awake) {
    c.p = add(c.p, mul(c.v, h));
    c.R = integrateR(c.R, c.w, h);
    // sleep when still
    if (len(c.v) < PHYS.sleepV && len(c.w) < PHYS.sleepW) { c.sleepT += h; if (c.sleepT > PHYS.sleepTime) { c.state = 'asleep'; c.v = [0, 0, 0]; c.w = [0, 0, 0]; } }
    else c.sleepT = 0;
  }
  return active.filter(c => c.part && c.jn > 1e-4).map(c => ({ part: c.part, cube: c.B.id, depth: c.depth, impulse: c.jn }));
}

function wake(c) { c.state = 'awake'; c.sleepT = 0; }

/** Velocity of a kinematic shape's surface at world point p, from its pose h earlier. */
function kinVelocity(s, p, h) {
  if (!s.prev || h <= 0) return [0, 0, 0];
  if (s.kind === 'box') {
    const local = mtv(s.R, sub(p, s.c));
    const before = add(s.prev.c, mv(s.prev.R, local));
    return mul(sub(p, before), 1 / h);
  }
  const ab = sub(s.b, s.a), t = Math.max(0, Math.min(1, dot(sub(p, s.a), ab) / (dot(ab, ab) || 1)));
  const va = mul(sub(s.a, s.prev.a), 1 / h), vb = mul(sub(s.b, s.prev.b), 1 / h);
  return add(mul(va, 1 - t), mul(vb, t));
}

/** Yaw (deg) of a cube: rotation about Z of whichever body axis is horizontal-most, folded into -45..45. */
export function cubeYaw(c) {
  const R = c.R;
  let up = 0, best = -1;
  for (let i = 0; i < 3; i++) { const z = Math.abs(R[6 + i]); if (z > best) { best = z; up = i; } }
  const k = (up + 1) % 3;
  const yaw = Math.atan2(R[3 + k], R[k]) * 180 / Math.PI;
  return ((yaw + 45) % 90 + 90) % 90 - 45;
}

/** How far (deg) the cube's nearest face normal is from vertical. */
export function cubeTilt(c) {
  const z = Math.max(Math.abs(c.R[6]), Math.abs(c.R[7]), Math.abs(c.R[8]));
  return Math.acos(Math.min(1, z)) * 180 / Math.PI;
}
