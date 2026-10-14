// Tracking the real arm with the AprilTags on it (config.js ARM_TAGS) and two or more calibrated cameras.
//
//   1. detectArmTags(image)            tag corners in one camera frame (aruco.js, family 36h11)
//   2. triangulate(views)              a tag seen by >= 2 cameras -> its 3D position (linear least squares)
//   3. fitArmPose(observations, init)  joint angles s1..s5 that put the model's tags where the cameras
//                                      see them (Levenberg-Marquardt on the reprojection error, all cameras)
//
// Cameras are calibrations from calib.js (pinhole, OpenCV convention, camera table frame in mm, as the
// Camera tab fits them to the mat markers). Model points are in the board frame (cm) and converted
// with the mat offset d. Pure math, no three.js, tested in node.

import { ARM_TAGS, MAT } from './config.js';
import { fk, point } from './kinematics.js';
import { project, eigSym, solve } from './calib.js';
import { detectMarkers } from './aruco.js';


/**
 * How much a camera's pixels count in a fit: 1 / its calibration error (px rms from the mat markers),
 * at most 2. The OAK (about 0,4 px) then counts several times as much as the webcam (about 2 px), and
 * together they still give the parallax that one camera alone lacks for depth.
 */
export const camWeight = cal => 1 / Math.max(0.5, Number.isFinite(cal?.rms) ? cal.rms : 1);

/** Board frame (cm) -> camera table frame (mm). The inverse of camview.js tableToBoard. */
export const boardToTable = ([x, y, z], d = MAT.sToEdge) => [(y - d) * 10, -x * 10, z * 10];
export const tableToBoardPt = ([X, Y, Z], d = MAT.sToEdge) => [-Y / 10, X / 10 + d, Z / 10];

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(...a); return a.map(v => v / l); };

/**
 * A calibration (calib.js shape) for a camera at posMm looking at lookAtMm (table frame), level
 * horizon, focal length f px, image w x h. Used as the first guess before the marker fit.
 */
export function calFromLookAt(posMm, lookAtMm, f, w = 1280, h = 720) {
  const fwd = norm(lookAtMm.map((v, i) => v - posMm[i]));
  const right = norm(cross(fwd, [0, 0, 1]));
  const down = cross(fwd, right);
  const R = [right, down, fwd];
  const t = R.map(r => -(r[0] * posMm[0] + r[1] * posMm[1] + r[2] * posMm[2]));
  return { R, t, f, cx: w / 2, cy: h / 2, centre: posMm.slice(), rms: NaN };
}

/** A tag's normal, up and right as an orthonormal set (up made square to the normal; right = up x normal). */
export function tagAxes(tag) {
  const n = norm(tag.normal), d = tag.up[0] * n[0] + tag.up[1] * n[1] + tag.up[2] * n[2];
  const up = norm(tag.up.map((v, i) => v - d * n[i]));
  return { n, up, right: cross(up, n) };
}

/** Board-frame centre and corners (TL, TR, BR, BL as printed) of an arm tag for model angles a. */
export function tagPoints(tag, a, frames = fk(a)) {
  const f = frames[tag.link];
  const { n, up, right } = tagAxes(tag);
  const h = tag.size / 2, P = (u, v) => point(f, tag.centre.map((c, i) => c + u * right[i] + v * up[i]));
  return { centre: point(f, tag.centre), corners: [P(-h, h), P(h, h), P(h, -h), P(-h, -h)] };
}

/** Pixel position of a board-frame point in a calibrated camera. */
export const projectBoard = (cal, P, d = MAT.sToEdge) => project(cal, boardToTable(P, d));

/**
 * Tag corners in an image (ImageData), trying a few threshold settings like OpenCV does.
 * Returns { [id]: { corners: [[u,v] x 4], centre: [u,v] } } for the arm tags found.
 */
export function detectArmTags(img, ids = ARM_TAGS.filter(t => t.id != null).map(t => t.id), { roi = null, zoom = 2, full = true } = {}) {
  const out = {};
  // small tags (under about 25 px) read better enlarged: first search a 2x crop around the arm
  if (roi) {
    const [x0, y0, x1, y1] = [Math.max(0, Math.floor(roi[0])), Math.max(0, Math.floor(roi[1])), Math.min(img.width, Math.ceil(roi[2])), Math.min(img.height, Math.ceil(roi[3]))];
    if (x1 - x0 > 16 && y1 - y0 > 16) {
      const crop = upscale(img, x0, y0, x1, y1, zoom);
      for (const [id, t] of Object.entries(detectArmTags(crop, ids)))
        out[id] = { corners: t.corners.map(([x, y]) => [x0 + (x + 0.5) / zoom - 0.5, y0 + (y + 0.5) / zoom - 0.5]), centre: [x0 + (t.centre[0] + 0.5) / zoom - 0.5, y0 + (t.centre[1] + 0.5) / zoom - 0.5] };
    }
  }
  // the whole image at a few threshold settings (left out with full: false, when the crop is enough)
  if (full) for (const [window, offset] of [[15, 3], [31, 7], [31, 3], [61, 7]]) {
    const { markers } = detectMarkers(img, { family: '36h11', ids, window, offset, maxArea: img.width * img.height / 4 });
    for (const [id, q] of Object.entries(markers)) if (!out[id]) {
      out[id] = { corners: q, centre: lineIntersection(q[0], q[2], q[1], q[3]) };   // diagonals meet at the projected centre
    }
    if (ids.every(id => out[id])) break;
  }
  return out;
}
/** Bilinear zoom of a region of an RGBA image. */
function upscale(img, x0, y0, x1, y1, z) {
  const w = Math.round((x1 - x0) * z), h = Math.round((y1 - y0) * z), data = new Uint8ClampedArray(w * h * 4), src = img.data, W = img.width;
  for (let y = 0; y < h; y++) {
    const sy = Math.min(img.height - 1.001, Math.max(0, y0 + (y + 0.5) / z - 0.5)), iy = Math.floor(sy), fy = sy - iy;
    for (let x = 0; x < w; x++) {
      const sx = Math.min(W - 1.001, Math.max(0, x0 + (x + 0.5) / z - 0.5)), ix = Math.floor(sx), fx = sx - ix;
      const i00 = 4 * (iy * W + ix), i01 = i00 + 4, i10 = i00 + 4 * W, i11 = i10 + 4, o = 4 * (y * w + x);
      for (let c = 0; c < 3; c++) data[o + c] = (src[i00 + c] * (1 - fx) + src[i01 + c] * fx) * (1 - fy) + (src[i10 + c] * (1 - fx) + src[i11 + c] * fx) * fy;
      data[o + 3] = 255;
    }
  }
  return { width: w, height: h, data };
}
function lineIntersection(a, b, c, d) {
  const den = (a[0] - b[0]) * (c[1] - d[1]) - (a[1] - b[1]) * (c[0] - d[0]);
  if (Math.abs(den) < 1e-9) return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const t = ((a[0] - c[0]) * (c[1] - d[1]) - (a[1] - c[1]) * (c[0] - d[0])) / den;
  return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
}

/**
 * 3D point (camera table frame, mm) from its pixels in >= 2 calibrated cameras: views = [{ cal, uv }].
 * Linear triangulation (DLT), then the per-view reprojection error. Returns { X, rms, errors }.
 */
export function triangulate(views) {
  if (views.length < 2) throw new Error('need at least two cameras');
  const A = [];
  for (const { cal, uv } of views) {
    const P = [0, 1, 2].map(i => [...cal.R[i], cal.t[i]]);
    const K = [[cal.f, 0, cal.cx], [0, cal.f, cal.cy], [0, 0, 1]];
    const KP = K.map(r => [0, 1, 2, 3].map(j => r[0] * P[0][j] + r[1] * P[1][j] + r[2] * P[2][j]));
    // each row scaled by the camera's weight, and by 1/depth so it is an error in pixels
    const w = camWeight(cal) / Math.max(1e-6, KP[2].slice(0, 3).reduce((s, v) => s + v * v, 0) ** 0.5);
    A.push(KP[0].map((v, j) => w * (uv[0] * KP[2][j] - v)), KP[1].map((v, j) => w * (uv[1] * KP[2][j] - v)));
  }
  // smallest eigenvector of A^T A, after scaling the unknowns to metres-ish for conditioning
  const s = 1000, Asc = A.map(r => [r[0] * s, r[1] * s, r[2] * s, r[3]]);
  const AtA = [0, 1, 2, 3].map(i => [0, 1, 2, 3].map(j => Asc.reduce((acc, r) => acc + r[i] * r[j], 0)));
  const { values, vectors } = eigSym(AtA);
  const k = values.indexOf(Math.min(...values));
  const v = [0, 1, 2, 3].map(i => vectors[i][k]);
  const X = [v[0] * s / v[3], v[1] * s / v[3], v[2] * s / v[3]];
  const errors = views.map(({ cal, uv }) => { const p = project(cal, X); return Math.hypot(p[0] - uv[0], p[1] - uv[1]); });
  return { X, rms: Math.sqrt(errors.reduce((a, e) => a + e * e, 0) / errors.length), errors };
}

/**
 * Joint angles from tag observations in any number of calibrated cameras.
 *   observations: [{ cal, tags: { [id]: { centre: [u,v], corners?: [[u,v] x 4] } } }]
 *   init: model angles to start from (e.g. the commanded pose); free: joints to fit
 *   useCorners: also fit the four corners (needs the tags' "up" right; the centres alone do not)
 * Returns { angles, rms (px), residuals: [{ cam, id, err }], points: { [id]: board-frame centre } }.
 */
export function fitArmPose(observations, { init, free = null, useCorners = false, shifts = {}, prior = 90, d = MAT.sToEdge, iterations = 100 } = {}) {
  const a0 = { s1: 0, s2: 0, s3: 0, s4: 0, s5: 0, ...init };
  const tags = Object.fromEntries(ARM_TAGS.filter(t => t.id != null).map(t => [t.id, t]));
  const obs = [];
  observations.forEach((o, ci) => {
    for (const [id, t] of Object.entries(o.tags || {})) {
      if (!tags[id]) continue;
      obs.push({ ci, cal: o.cal, tag: tags[id], k: -1, uv: t.centre });
      // corners turned by the quarter turns the tag is stuck on with (shifts[id], see rigidtrack.js)
      if (useCorners && t.corners) t.corners.forEach((_, k) => obs.push({ ci, cal: o.cal, tag: tags[id], k, uv: t.corners[(k + (shifts[id] || 0)) % 4] }));
    }
  });
  if (obs.length < 2) throw new Error(`too few tag observations (${obs.length}); need a tag in at least two views`);
  // S5 turns only what is beyond the wrist: fit it only when a tag on the hand is seen
  free ??= obs.some(o => o.tag.link === 'hand') ? ['s1', 's2', 's3', 's4', 's5'] : ['s1', 's2', 's3', 's4'];
  const unpack = p => { const a = { ...a0 }; free.forEach((j, i) => { a[j] = p[i]; }); return a; };
  const resid = p => {
    const a = unpack(p), frames = fk(a), cache = new Map(), r = [];
    for (const o of obs) {
      if (!cache.has(o.tag.id)) cache.set(o.tag.id, tagPoints(o.tag, a, frames));
      const tp = cache.get(o.tag.id), P = o.k < 0 ? tp.centre : tp.corners[o.k];
      const q = projectBoard(o.cal, P, d);
      const w = camWeight(o.cal);
      r.push(w * (q[0] - o.uv[0]), w * (q[1] - o.uv[1]));
    }
    for (const j of free) r.push((a[j] - a0[j]) / prior);   // weak pull to the start (1 px per `prior` degrees)
    return r;
  };
  const sq = r => r.reduce((s, v) => s + v * v, 0);
  let p = free.map(j => a0[j]), r = resid(p), cost = sq(r), mu = 1e-3;
  const n = p.length;
  for (let it = 0; it < iterations; it++) {
    const J = r.map(() => new Array(n).fill(0));
    for (let k = 0; k < n; k++) { const pp = p.slice(); pp[k] += 1e-3; const rr = resid(pp); for (let i = 0; i < r.length; i++) J[i][k] = (rr[i] - r[i]) / 1e-3; }
    const JtJ = [...Array(n)].map((_, i) => [...Array(n)].map((__, j) => J.reduce((s, row) => s + row[i] * row[j], 0)));
    const Jtr = [...Array(n)].map((_, i) => J.reduce((s, row, k) => s + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((v, j) => v + (i === j ? mu * (JtJ[i][i] || 1) + 1e-12 : 0)));
      let dp; try { dp = solve(A, Jtr.map(v => -v)); } catch { mu *= 10; continue; }
      const pn = p.map((v, k) => v + dp[k]), rn = resid(pn), cn = sq(rn);
      if (cn < cost) { improved = cost - cn > 1e-9 * cost; p = pn; r = rn; cost = cn; mu = Math.max(1e-12, mu / 5); if (!improved) break; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  const angles = unpack(p), frames = fk(angles);
  const residuals = obs.filter(o => o.k < 0).map(o => {
    const q = projectBoard(o.cal, tagPoints(o.tag, angles, frames).centre, d);
    return { cam: o.ci, id: o.tag.id, err: Math.hypot(q[0] - o.uv[0], q[1] - o.uv[1]) };
  });
  const pts = Object.fromEntries(ARM_TAGS.filter(t => t.id != null).map(t => [t.id, tagPoints(t, angles, frames).centre]));
  const e2 = residuals.reduce((s, x) => s + x.err * x.err, 0);
  return { angles, free, rms: Math.sqrt(e2 / Math.max(1, residuals.length)), residuals, points: pts };
}

