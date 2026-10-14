// Cubes in 3D from two or more calibrated cameras: no assumption that they lie on the mat, so stacked
// cubes and a cube in the gripper are found at their real height.
//
// Each camera's cube detector (detect.js) gives a colour blob and its image centroid. For each colour the
// blobs are paired across cameras, their viewing rays intersected (least squares; the gap between the rays
// says how well they meet), and the result moved from the centroid of the visible faces to the cube's centre.
// Frames: calibrations in the camera table frame (mm), as calib.js; results also in the board frame (cm).

import { CUBE } from './config.js';
import { triangulate, tableToBoardPt } from './armtrack.js';
import { solve, project } from './calib.js';

const H_MM = CUBE * 10 / 2;   // half the cube side, mm

/** Camera centre and the unit ray through pixel uv, table frame. */
export function ray(cal, uv) {
  const d = [(uv[0] - cal.cx) / cal.f, (uv[1] - cal.cy) / cal.f, 1];
  const w = [0, 1, 2].map(i => cal.R[0][i] * d[0] + cal.R[1][i] * d[1] + cal.R[2][i] * d[2]);
  const n = Math.hypot(...w);
  const C = cal.centre || [0, 1, 2].map(i => -(cal.R[0][i] * cal.t[0] + cal.R[1][i] * cal.t[1] + cal.R[2][i] * cal.t[2]));
  return { C, d: w.map(v => v / n) };
}

/** Shortest distance between two rays (mm) and the midpoint of the closest points. */
export function rayGap(a, b) {
  const w0 = a.C.map((v, i) => v - b.C[i]);
  const A = a.d.reduce((s, v, i) => s + v * a.d[i], 0), B = a.d.reduce((s, v, i) => s + v * b.d[i], 0), Cc = b.d.reduce((s, v, i) => s + v * b.d[i], 0);
  const D = a.d.reduce((s, v, i) => s + v * w0[i], 0), E = b.d.reduce((s, v, i) => s + v * w0[i], 0);
  const den = A * Cc - B * B;
  const sa = den > 1e-12 ? (B * E - Cc * D) / den : 0, sb = den > 1e-12 ? (A * E - B * D) / den : E / Cc;
  const pa = a.C.map((v, i) => v + sa * a.d[i]), pb = b.C.map((v, i) => v + sb * b.d[i]);
  return { gap: Math.hypot(...pa.map((v, i) => v - pb[i])), mid: pa.map((v, i) => (v + pb[i]) / 2) };
}

/** The point closest to all rays { C, d (unit) } in the least-squares sense. */
export function intersectRays(rays) {
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
  for (const { C, d } of rays) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    const m = (i === j ? 1 : 0) - d[i] * d[j];
    A[i][j] += m; b[i] += m * C[j];
  }
  return solve(A, b);
}

/**
 * Offset (mm) from a cube's centre to the centroid of its faces as seen from direction v (unit vector from
 * the cube to the camera): faces weighted by their projected area. Axis-aligned cube turned by yaw (deg).
 */
export function visibleCentroidOffset(v, yaw = 0, { top = true } = {}) {
  const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180);
  const normals = [[c, s, 0], [-c, -s, 0], [-s, c, 0], [s, -c, 0]];   // the bottom face is never seen,
  if (top) normals.push([0, 0, 1]);                                     // the top one not under another cube
  let w = 0; const o = [0, 0, 0];
  for (const n of normals) { const k = n[0] * v[0] + n[1] * v[1] + n[2] * v[2]; if (k > 0) { w += k; for (let i = 0; i < 3; i++) o[i] += k * n[i] * H_MM; } }
  return w > 0 ? o.map(x => x / w) : [0, 0, 0];
}

/** Area of the convex hull of 2D points (the outline of a projected cube). */
function hullArea(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]), cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length > 1 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.slice().reverse()) { while (up.length > 1 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  const h = lo.slice(0, -1).concat(up.slice(0, -1));
  let a = 0; for (let i = 0; i < h.length; i++) { const [x1, y1] = h[i], [x2, y2] = h[(i + 1) % h.length]; a += x1 * y2 - x2 * y1; }
  return Math.abs(a) / 2;
}

/**
 * The cube's known size against what each camera saw: the outline of a 26,5 mm cube at the located centre,
 * projected into the camera, against the colour blob there. Returns the worst ratio of blob area to outline
 * area and of blob size to outline size over the cameras ({ area, size }); a cube that is partly hidden (by the
 * fingers) shows less, a cable or the foam board a different size or shape.
 */
/** Convex hull of 2D points, counter-clockwise (monotone chain). */
function hull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]), cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length > 1 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.slice().reverse()) { while (up.length > 1 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

/** The 8 corners (table mm) of an upright cube with centre X and yaw (deg). */
export function cubeCorners(X, yaw) {
  const h = H_MM, c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), out = [];
  for (const a of [-h, h]) for (const b of [-h, h]) for (const z of [-h, h]) out.push([X[0] + a * c - b * s, X[1] + a * s + b * c, X[2] + z]);
  return out;
}

/** Overlap (intersection over union) of a projected cube outline with a blob's sampled pixels in one camera. */
function overlap(cal, X, yaw, blob) {
  const H = hull(cubeCorners(X, yaw).map(P => project(cal, P)));
  let area = 0; for (let i = 0; i < H.length; i++) { const [x1, y1] = H[i], [x2, y2] = H[(i + 1) % H.length]; area += x1 * y2 - x2 * y1; }
  area = Math.abs(area) / 2;
  let inside = 0;
  for (const [u, v] of blob.samples) {
    let ok = true;
    for (let i = 0; i < H.length && ok; i++) { const [x1, y1] = H[i], [x2, y2] = H[(i + 1) % H.length]; if ((x2 - x1) * (v - y1) - (y2 - y1) * (u - x1) < 0) ok = false; }
    if (ok) inside++;
  }
  const inter = inside * blob.cell, union = blob.samples.length * blob.cell + area - inter;
  return union > 0 ? inter / union : 0;
}

/**
 * A cube's pose from its blobs in two or more cameras: the cube is upright (it can only be lifted and turned), so
 * four numbers, centre (table mm) and yaw, are fitted so that the projected 26,5 mm cube covers each camera's blob
 * as well as possible (summed overlap). The outline's shape turns with the cube, which gives the yaw.
 *   obs: [{ cal, samples, cell }]; start: { X, yaw }
 * Returns { X, yaw (deg, -45..45), overlap (mean IoU over the cameras) }.
 */
export function fitCubePose(obs, start) {
  const use = obs.filter(o => o.samples?.length);
  if (!use.length) return { X: start.X.slice(), yaw: start.yaw || 0, overlap: null };
  const score = (X, yaw) => use.reduce((s, o) => s + overlap(o.cal, X, yaw, o), 0) / use.length;
  let X = start.X.slice(), yaw = 0, best = -1;
  for (let y = -45; y < 45; y += 5) { const s = score(X, y); if (s > best) { best = s; yaw = y; } }   // the turn first
  // then all four together, coordinate steps halving down to 0,5 mm and 0,5 degrees
  for (let step = 4; step >= 0.5; step /= 2) {
    let moved = true;
    while (moved) {
      moved = false;
      for (const [k, d] of [[0, 1], [0, -1], [1, 1], [1, -1], [2, 1], [2, -1], [3, 1], [3, -1]]) {
        const Xn = X.slice(); let yn = yaw;
        if (k < 3) Xn[k] += d * step; else yn += d * step;
        const s = score(Xn, yn);
        if (s > best + 1e-6) { best = s; X = Xn; yaw = yn; moved = true; }
      }
    }
  }
  return { X, yaw: ((yaw + 45) % 90 + 90) % 90 - 45, overlap: best };
}

/**
 * Is it a cube, by size? At least one camera must see about a whole cube there (the other may see only part:
 * pale faces, the fingers in front), and none much more than one; cables and the foam board are far smaller.
 */
export const plausibleSize = f => f.big >= 0.4 && f.big <= 1.8 && f.area >= 0.08 && f.size >= 0.25;

export function sizeCheck(c) {
  const h = CUBE * 10 / 2, yaw = (c.yaw || 0) * Math.PI / 180, cs = Math.cos(yaw), sn = Math.sin(yaw);
  const corners = [];
  for (const a of [-h, h]) for (const b of [-h, h]) for (const z of [-h, h]) corners.push([c.X[0] + a * cs - b * sn, c.X[1] + a * sn + b * cs, c.X[2] + z]);
  let area = Infinity, size = Infinity, big = 0;
  for (const s of c.seen || []) {
    const uv = corners.map(P => project(s.cal, P)), A = hullArea(uv);
    const bw = s.bbox[2] - s.bbox[0], bh = s.bbox[3] - s.bbox[1];
    const ow = Math.max(...uv.map(p => p[0])) - Math.min(...uv.map(p => p[0])), oh = Math.max(...uv.map(p => p[1])) - Math.min(...uv.map(p => p[1]));
    area = Math.min(area, s.pixels / A); big = Math.max(big, s.pixels / A);
    size = Math.min(size, Math.min(bw / ow, bh / oh));
  }
  return { area, size, big };
}

// ---- several cubes of one colour: touching ones form one blob, so the cubes are fitted to all of a colour's
// blobs in all cameras together (they are all 26,5 mm cubes, upright, and cannot overlap)

const polyArea = H => { let a = 0; for (let i = 0; i < H.length; i++) { const [x1, y1] = H[i], [x2, y2] = H[(i + 1) % H.length]; a += x1 * y2 - x2 * y1; } return Math.abs(a) / 2; };
const inPoly = (H, u, v) => { for (let i = 0; i < H.length; i++) { const [x1, y1] = H[i], [x2, y2] = H[(i + 1) % H.length]; if ((x2 - x1) * (v - y1) - (y2 - y1) * (u - x1) < 0) return false; } return true; };

/**
 * How several cubes explain one camera's samples of a colour: the union overlap (each cube's outline area
 * counted, so overlapping outlines cost), and per cube how much of its own outline is coloured.
 */
function coverage(cal, cubes, o, { unexplained = false } = {}) {
  const hulls = cubes.map(c => hull(cubeCorners(c.X, c.yaw).map(P => project(cal, P)))), areas = hulls.map(polyArea);
  const own = cubes.map(() => 0); let inside = 0;
  for (const [u, v] of o.samples) { let hit = false; hulls.forEach((H, i) => { if (inPoly(H, u, v)) { own[i]++; hit = true; } }); if (hit) inside++; }
  const inter = inside * o.cell, union = o.samples.length * o.cell + areas.reduce((a, b) => a + b, 0) - inter;
  return { iou: union > 0 ? inter / union : 0, own: own.map((n, i) => (areas[i] ? Math.min(1, n * o.cell / areas[i]) : 0)), area: areas,
    unexplained: unexplained ? o.samples.filter(([u, v]) => !hulls.some(H => inPoly(H, u, v))) : null };
}
/** At most  samples (every k-th), each then standing for k times the area: the fits stay fast. */
const thin = (o, max = 250) => { const k = Math.ceil(o.samples.length / max); return k <= 1 ? o : { ...o, samples: o.samples.filter((_, i) => i % k === 0), cell: o.cell * k }; };
/** Two cubes may touch but not overlap (one on top of the other is fine). */
const clash = cubes => cubes.some((a, i) => cubes.some((b, j) => j > i && Math.abs(a.X[0] - b.X[0]) < 0.9 * 2 * H_MM && Math.abs(a.X[1] - b.X[1]) < 0.9 * 2 * H_MM && Math.abs(a.X[2] - b.X[2]) < 0.9 * 2 * H_MM));

/** All cubes of one colour refined together: coordinate steps on every cube's centre and yaw, union overlap summed over the cameras. */
function fitCubesTogether(obs, cubes) {
  const score = cs => clash(cs) ? -1 : obs.reduce((s, o) => s + coverage(o.cal, cs, o).iou, 0) / obs.length;
  let cs = cubes.map(c => ({ ...c, X: c.X.slice() })), best = score(cs);
  for (let step = 4; step >= 0.5; step /= 2) {
    let moved = true, rounds = 0;
    while (moved && rounds++ < 25) {
      moved = false;
      for (let i = 0; i < cs.length; i++) for (const [k, d] of [[0, 1], [0, -1], [1, 1], [1, -1], [2, 1], [2, -1], [3, 1], [3, -1]]) {
        const t = cs.map(c => ({ ...c, X: c.X.slice() }));
        if (k < 3) t[i].X[k] += d * step; else t[i].yaw += d * step;
        const s = score(t);
        if (s > best + 1e-6) { best = s; cs = t; moved = true; }
      }
    }
  }
  return { cubes: cs, score: best };
}

/**
 * Adds cubes of a colour while a camera still sees a cube's worth of that colour unexplained: a new cube starts at
 * the unexplained part (rays from two cameras, or one camera at mat height), all of them are refitted together,
 * and it is kept if it explains clearly more. obs: [{ cal, samples, cell }] per camera with this colour.
 */
function addMissingCubes(obs, cubes, colour, { maxCubes = 6 } = {}) {
  let cur = cubes.length ? fitCubesTogether(obs, cubes) : { cubes: [], score: 0 };
  // per cube: how much of the colour lies nearest to it, in cube outlines (in the camera that sees most of it). About
  // two or more: touching cubes of the same colour seen as one; try them stacked, or side by side along its edges
  for (let i = 0; i < cur.cubes.length; i++) {
    const ref = cur.cubes[i];
    const share = obs.map(o => {
      const centres = cur.cubes.map(c => project(o.cal, c.X));
      const mine = o.samples.filter(([u, v]) => centres.reduce((b, c, j) => Math.hypot(u - c[0], v - c[1]) < Math.hypot(u - centres[b][0], v - centres[b][1]) ? j : b, 0) === i).length;
      return mine * o.cell / polyArea(hull(cubeCorners(ref.X, ref.yaw).map(P => project(o.cal, P))));
    });
    const n = Math.min(3, Math.round(Math.max(...share)));
    if (n < 2) continue;
    const others = cur.cubes.filter((_, j) => j !== i), base = { ...ref, added: true }, h = 2 * H_MM;
    const c = Math.cos(ref.yaw * Math.PI / 180), s = Math.sin(ref.yaw * Math.PI / 180);
    const stack = Array.from({ length: n }, (_, k) => ({ ...base, X: [ref.X[0], ref.X[1], H_MM + k * h] }));
    const row = dir => Array.from({ length: n }, (_, k) => { const o = (k - (n - 1) / 2) * h; return { ...base, X: [ref.X[0] + o * dir[0], ref.X[1] + o * dir[1], ref.X[2]] }; });
    let best = null;
    for (const hyp of [stack, row([c, s]), row([-s, c])]) { const trial = fitCubesTogether(obs, [...others, ...hyp]); if (!best || trial.score > best.score) best = trial; }
    // kept when it explains clearly more (the orange stack in the edge-case frames: +0,025; a single cube tried as
    // two: -0,15)
    if (best.score > cur.score + 0.02) { cur = best; i = -1; }        // start over: the cubes changed
  }
  for (let n = cur.cubes.length; n < maxCubes; n++) {
    const cov = obs.map(o => ({ o, c: coverage(o.cal, cur.cubes, o, { unexplained: true }) }));
    // a camera with at least half a cube's worth of colour left over (a cube there is about f^2 * 26,5^2 / depth^2 px)
    const left = cov.map(({ o, c }) => {
      if (c.unexplained.length < 8) return null;
      const uv = [0, 1].map(k => c.unexplained.reduce((s, p) => s + p[k], 0) / c.unexplained.length);
      const P = rayPlane(o.cal, uv, H_MM), H1 = P && polyArea(hull(cubeCorners(P, 0).map(Q => project(o.cal, Q))));
      return P && c.unexplained.length * o.cell >= 0.5 * H1 ? { o, uv, P } : null;
    }).filter(Boolean);
    if (!left.length) break;
    const start = left.length >= 2 ? intersectRays(left.map(l => ray(l.o.cal, l.uv))) : left[0].P;
    // the new cube's turn first (5 degree steps, the others held), then all together
    const add = { colour, X: start, yaw: 0, cameras: obs.length, gap: 0, added: true };
    let by = 0, bs = -2;
    for (let y = -45; y < 45; y += 5) { const cs = [...cur.cubes, { ...add, yaw: y }], s = clash(cs) ? -1 : obs.reduce((a, o) => a + coverage(o.cal, cs, o).iou, 0); if (s > bs) { bs = s; by = y; } }
    const trial = fitCubesTogether(obs, [...cur.cubes, { ...add, yaw: by }]);
    trial.cubes.forEach(c => { c.yaw = ((c.yaw + 45) % 90 + 90) % 90 - 45; });
    const mine = obs.map(o => coverage(o.cal, trial.cubes, o).own[trial.cubes.length - 1]);
    if (!(trial.score > cur.score + 0.03 && Math.max(...mine) >= 0.5)) break;
    cur = trial;
  }
  return cur.cubes;
}

/** Where the ray through pixel uv meets the plane Z = z (table mm), or null. */
function rayPlane(cal, uv, z) {
  const { C, d } = ray(cal, uv);
  if (Math.abs(d[2]) < 1e-6) return null;
  const s = (z - C[2]) / d[2];
  return s > 0 ? [C[0] + s * d[0], C[1] + s * d[1], z] : null;
}

/**
 * Locate cubes from several cameras. views: [{ cal, detections: detect.js results }].
 * Returns [{ colour, X (table mm), board (cm), level, gap (mm), cameras, yaw }] for every colour blob
 * seen by at least two cameras; blobs of one colour are paired by the smallest ray gap.
 */
export function locateCubes(views, { maxGap = 25, checkSize = true, fitPose = true, minOverlap = 0.45, multi = true, minOwn = 0.5 } = {}) {
  const usable = views.filter(v => v.cal && v.detections?.length);
  if (usable.length < 2) return [];
  const colours = [...new Set(usable.flatMap(v => v.detections.map(d => d.colour)))];
  const out = [];
  for (const colour of colours) {
    // blobs of this colour per camera, each with its ray
    const per = usable.map(v => v.detections.filter(d => d.colour === colour && d.uv).map(d => ({ d, cal: v.cal, r: ray(v.cal, d.uv), used: false })));
    // greedy pairing between the first two cameras that have this colour, others join by smallest gap
    const cams = per.filter(p => p.length);
    if (cams.length < 2) continue;
    const pairs = [];
    for (const a of cams[0]) for (const b of cams[1]) pairs.push({ a, b, ...rayGap(a.r, b.r) });
    pairs.sort((p, q) => p.gap - q.gap);
    for (const p of pairs) {
      if (p.a.used || p.b.used || p.gap > maxGap) continue;
      // the known cube size: both blobs must be about a cube's size at the place their rays meet (a cable, the foam
      // board or another blue thing paired by chance is not)
      if (checkSize && [p.a, p.b].every(m => m.d.pixels && m.d.bbox)) { const f = sizeCheck({ X: p.mid, yaw: p.a.d.yaw, seen: [p.a, p.b].map(m => ({ cal: m.cal, pixels: m.d.pixels, bbox: m.d.bbox })) }); if (!plausibleSize(f)) continue; }
      p.a.used = p.b.used = true;
      const members = [p.a, p.b];
      for (const more of cams.slice(2)) {
        const best = more.filter(m => !m.used).map(m => ({ m, ...rayGap(p.a.r, m.r) })).sort((x, y) => x.gap - y.gap)[0];
        if (best && best.gap < maxGap) { best.m.used = true; members.push(best.m); }
      }
      // intersect the rays through the blob centroids (the centre of the visible faces, not of the cube)
      const tri = triangulate(members.map(m => ({ cal: m.cal, uv: m.d.uv })));
      out.push({ colour, raw: tri.X, members, gap: p.gap, rms: tri.rms, cameras: members.length, yaw: members[0].d.yaw ?? 0 });
    }
  }
  // Shift each to its centre: the visible faces' centroid as seen from each camera. A cube with another
  // one right on top shows no top face; two passes, since "on top" needs the centres.
  // Each camera's ray passes through ITS visible-face centroid, centre + offset(camera): shift every ray
  // back by its own offset and intersect the shifted rays (least squares).
  const centre = (c, top) => intersectRays(c.members.map(m => {
    const v = m.r.C.map((x, i) => x - c.raw[i]), n = Math.hypot(...v);
    const off = visibleCentroidOffset(v.map(x => x / n), c.yaw, { top });
    return { C: m.r.C.map((x, i) => x - off[i]), d: m.r.d };
  }));
  for (const c of out) c.X = centre(c, true);
  for (const c of out) {
    const covered = out.some(o => o !== c && Math.hypot(o.X[0] - c.X[0], o.X[1] - c.X[1]) < H_MM && Math.abs(o.X[2] - c.X[2] - 2 * H_MM) < H_MM);
    if (covered) c.X = centre(c, false);
  }
  // the pose that makes a real-size cube cover what the cameras saw: centre and yaw (needs the blobs' pixel samples)
  if (fitPose) for (const c of out) {
    const obs = c.members.map(m => ({ cal: m.cal, samples: m.d.samples, cell: m.d.cell }));
    if (obs.every(o => o.samples?.length)) { const p = fitCubePose(obs, { X: c.X, yaw: c.yaw }); c.X = p.X; c.yaw = p.yaw; c.overlap = p.overlap; }
  }
  if (fitPose && multi) {
    for (const colour of colours) {
      const obs = usable.map(v => { const ds = v.detections.filter(d => d.colour === colour && d.samples?.length); return ds.length ? thin({ cal: v.cal, samples: ds.flatMap(d => d.samples), cell: ds[0].cell }) : null; }).filter(Boolean);
      if (obs.length < 2) continue;
      const mine = out.filter(c => c.colour === colour);
      const fitted = addMissingCubes(obs, mine, colour);
      // each cube: how much of its own outline the cameras see coloured (mean); a real cube most of it, a cable or
      // the foam board little
      const own = fitted.map((_, i) => obs.reduce((s, o) => s + coverage(o.cal, fitted, o).own[i], 0) / obs.length);
      for (const c of mine) out.splice(out.indexOf(c), 1);
      fitted.forEach((c, i) => { if (own[i] >= minOwn) out.push({ ...c, members: c.members || [], coloured: own[i] }); });
    }
  } else if (fitPose) for (let i = out.length - 1; i >= 0; i--) if (out[i].overlap != null && out[i].overlap < minOverlap) out.splice(i, 1);
  for (const c of out) {
    // what each camera saw of it (for size checks: js/cubetrack.js)
    c.seen = c.members.map(m => ({ cal: m.cal, pixels: m.d.pixels, bbox: m.d.bbox }));
    c.board = tableToBoardPt(c.X);
    c.level = Math.max(0, Math.round((c.X[2] - H_MM) / (2 * H_MM)));
    delete c.members; delete c.raw;
  }
  return out.sort((a, b) => a.level - b.level || a.colour.localeCompare(b.colour));
}
