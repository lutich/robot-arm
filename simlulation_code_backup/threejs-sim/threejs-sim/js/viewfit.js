// Camera pose and arm pose from a photo: points marked in the photo (pixels) are matched to
// named points on the model (KEYPOINTS), then the camera (centre, rotation, focal length) and
// the joint angles are fitted together by Levenberg-Marquardt on the reprojection error.
// Pure math, no three.js, so node can run it (tools/fit-views.mjs) and test it.
//
// Frames: board frame (cm) of docs/arm-dimensions.md. Camera: OpenCV convention (x right,
// y down, z forward), p_cam = R (P - C), u = cx + f x / z, v = cy + f y / z, principal point
// at the image centre.

import { ARM, ARM_TAGS, BASE, MAT } from './config.js';
import { tagAxes } from './armtrack.js';
import { fk, point, gapForTheta, gripperTheta, thetaForGap, padDepthForTheta } from './kinematics.js';
import { rodrigues, rotvec, solve, MARKER_CORNERS, markerCentre } from './calib.js';

const RAD = Math.PI / 180, DEG = 180 / Math.PI;
const BASE_Y = BASE.offsetY, BASE_H = BASE.h;   // config.js BASE: base plate centre and height
const baseFront = BASE_Y + 4.12, baseBack = BASE_Y - 4.12;

/**
 * Named model points: [frame, [x, y, z]] in that frame (cm). Frames: board (fixed), turret
 * (turns with S1), shoulder, elbow, wrist, hand (fk frames), mat (marker corners, table mm).
 * "+x"/"-x" sides are the arm's right and left (+X to the right seen from the robot). Horns sit
 * on the +X side of S2, S3, S4; the -X side has the bearing pins.
 * The *_axis points are the joint axes' midpoints (between horn and pin): schematic values, so they
 * stay right where the drawn brackets, tube and servo bodies differ from the real arm. In a photo,
 * take the middle between the two visible ends of an axis, or the middle of the link across it.
 */
export const KEYPOINTS = {
  s2_axis: ['turret', [0, 0, ARM.H_S2]], s3_axis: ['shoulder', [0, 0, ARM.L2]], s4_axis: ['elbow', [0, 0, ARM.L3]],
  base_top_fl: ['board', [-4.5, baseFront, BASE_H]], base_top_fr: ['board', [4.5, baseFront, BASE_H]],
  base_top_bl: ['board', [-4.5, baseBack, BASE_H]], base_top_br: ['board', [4.5, baseBack, BASE_H]],
  base_bot_fl: ['board', [-4.5, baseFront, 0]], base_bot_fr: ['board', [4.5, baseFront, 0]],
  base_bot_bl: ['board', [-4.5, baseBack, 0]], base_bot_br: ['board', [4.5, baseBack, 0]],
  s2_horn: ['turret', [2.6, 0, ARM.H_S2]], s2_pin: ['turret', [-2.8, 0, ARM.H_S2]],
  ua_mid_px: ['shoulder', [2.7, 0, ARM.L2 / 2]], ua_mid_nx: ['shoulder', [-2.7, 0, ARM.L2 / 2]],
  pca: ['shoulder', [-3.96, -0.1, 4.3]],                     // PCA9685 board centre on the -X side plate (model/arm.js PCA)
  // its board corners: top = toward the elbow, front = +Y edge (power terminal), back = header edge
  pca_top_front: ['shoulder', [-3.96, 1.17, 7.43]], pca_top_back: ['shoulder', [-3.96, -1.37, 7.43]],
  pca_bot_front: ['shoulder', [-3.96, 1.17, 1.18]], pca_bot_back: ['shoulder', [-3.96, -1.37, 1.18]],
  s3_horn: ['shoulder', [2.6, 0, ARM.L2]], s3_pin: ['shoulder', [-2.63, 0, ARM.L2]],
  s3_back: ['elbow', [0, 0, -3.45]],                         // back plate of the S3 servo bracket, behind the elbow
  tube_lo: ['elbow', [0, 0, 2.05]], tube_hi: ['elbow', [0, 0, ARM.L3 - 2.8]],
  s4_horn: ['elbow', [2.6, 0, ARM.L3]], s4_pin: ['elbow', [-2.45, 0, ARM.L3]],
  wrist_plate: ['wrist', [0, 0, 2.9]],                       // middle of the plate the S5 servo hangs from
  flange: ['hand', [0, 0, 0]],                               // S5 horn, centre
  pad_px: ['hand', ['pad', 1]], pad_nx: ['hand', ['pad', -1]],
};
// mat marker corners m<id>.<k> (OpenCV order) and centres m<id>
for (const id of Object.keys(MARKER_CORNERS)) {
  KEYPOINTS[`m${id}`] = ['mat', [...markerCentre(id), 0]];
  MARKER_CORNERS[id].forEach((p, k) => { KEYPOINTS[`m${id}.${k}`] = ['mat', [p[0], p[1], 0]]; });
}
// arm tags (config.js ARM_TAGS): centre t<id> and black-square corners t<id>.<k> (TL, TR, BR, BL as printed,
// the order OpenCV / detectArmTags return them in)
for (const tag of ARM_TAGS) {
  const { up, right } = tagAxes(tag), h = tag.size / 2;
  const P = (u, v) => tag.centre.map((c, i) => c + u * right[i] + v * up[i]);
  KEYPOINTS[`t${tag.id}`] = [tag.link, tag.centre.slice()];
  [P(-h, h), P(h, h), P(h, -h), P(-h, -h)].forEach((p, k) => { KEYPOINTS[`t${tag.id}.${k}`] = [tag.link, p]; });
}

/** Board-frame position of a keypoint for model angles a, jaw gap and mat offset d. */
export function keypointWorld(name, a, gap, d = MAT.sToEdge, frames = fk(a)) {
  const kp = KEYPOINTS[name];
  if (!kp) throw new Error(`unknown keypoint ${name}`);
  const [frame, p] = kp;
  if (frame === 'board') return p;
  if (frame === 'mat') return [-p[1] / 10, p[0] / 10 + d, 0];
  if (frame === 'turret') return point(frames.base, [p[0], p[1], p[2]]);
  if (p[0] === 'pad') {
    const th = thetaForGap(gap);
    return point(frames.hand, [p[1] * gap / 2, 0, padDepthForTheta(th)]);
  }
  return point(frames[frame], p);
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mulv = (R, v) => R.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);

/** Pixel position of a board-frame point. cam = { R, C, f, cx, cy }. */
export function projectPoint(cam, P) {
  const q = mulv(cam.R, sub(P, cam.C));
  return [cam.cx + cam.f * q[0] / q[2], cam.cy + cam.f * q[1] / q[2], q[2]];
}

/** Camera looking from `pos` at `target` (board frame), Z up, rolled by `roll` degrees, vertical fov. */
export function cameraFromLookAt({ pos, target, fov, roll = 0 }, width, height) {
  const fwd = norm(sub(target, pos));
  let right = norm(cross(fwd, [0, 0, 1]));
  let down = cross(fwd, right);
  const r = roll * RAD, c = Math.cos(r), s = Math.sin(r);
  // three.js rotateZ(roll) turns the camera's x toward its y (up): right' = c right - s down
  [right, down] = [right.map((v, i) => c * v - s * down[i]), down.map((v, i) => c * v + s * right[i])];
  const f = height / 2 / Math.tan(fov * RAD / 2);
  return { R: [right, down, fwd], C: pos.slice(), f, cx: width / 2, cy: height / 2 };
}

/** pos / target / fov / roll for compare.mjs (three.js camera, up = +Z then rotateZ(roll)). */
export function lookAtFromCamera(cam, height, dist = 30) {
  const [right, down, fwd] = cam.R;
  const right0 = norm(cross(fwd, [0, 0, 1])), down0 = cross(fwd, right0);
  const roll = Math.atan2(-dot(right, down0), dot(right, right0)) * DEG;
  return {
    pos: cam.C.map(round1), target: cam.C.map((v, i) => round1(v + dist * fwd[i])),
    fov: +(2 * Math.atan(height / 2 / cam.f) * DEG).toFixed(2), roll: +roll.toFixed(2),
  };
}

const JOINTS = ['s1', 's2', 's3', 's4', 's5'];

/**
 * Fit camera and arm to marked points.
 *   points: { name: [u, v] or [u, v, weight] } in photo pixels
 *   init: { camera: {pos, target, fov, roll}, angles: {s1..s5}, gap, d }
 *   free: joints to fit (default all but s5), fitD: also fit the mat offset d, fitF: focal length
 *   prior: angle prior sigma (deg) toward init.angles; keeps unobserved joints in place
 *   fovPrior: [vertical fov, sigma] (deg); phone photos without EXIF need it when few points are deep in the scene
 * Returns { camera (lookAt form), cam, angles, d, rms, residuals: { name: [du, dv] } }.
 */
export function fitView(points, { width, height, init, fovPrior = null, ...opts }) {
  const { views, angles, d } = fitViews([{ points, width, height, camera: init.camera, fovPrior }], { init, ...opts });
  return { ...views[0], angles, d };
}

/**
 * Several photos of the same arm pose: one camera per photo, one set of joint angles (and mat
 * offset d) for all. views: [{ points, width, height, camera, fovPrior }]; options as fitView.
 * Returns { views: [{ camera, cam, rms, residuals }], angles, d }.
 */
export function fitViews(views, { init, free = ['s1', 's2', 's3', 's4'], fitD = false, fitF = true, prior = 60, iterations = 200 }) {
  const list = views.map(v => ({ ...v, names: Object.keys(v.points).filter(n => KEYPOINTS[n]), cam0: cameraFromLookAt(v.camera, v.width, v.height) }));
  for (const v of list) if (v.names.length < 3) throw new Error(`need at least 3 known points per photo, got ${v.names.length}`);
  const a0 = { s1: 0, s2: 0, s3: 0, s4: 0, s5: 0, ...init.angles };
  const gap = init.gap ?? 0, d0 = init.d ?? MAT.sToEdge, nc = 7 * list.length;
  // parameters: per photo rotvec(3), C(3), log f; then the free angles and d
  const pack = () => [...list.flatMap(v => [...rotvec(v.cam0.R), ...v.cam0.C, Math.log(v.cam0.f)]), ...free.map(j => a0[j]), ...(fitD ? [d0] : [])];
  const unpack = p => {
    const angles = { ...a0 }; free.forEach((j, i) => { angles[j] = p[nc + i]; });
    const cams = list.map((v, k) => ({ R: rodrigues(p.slice(7 * k, 7 * k + 3)), C: p.slice(7 * k + 3, 7 * k + 6),
      f: fitF ? Math.exp(p[7 * k + 6]) : v.cam0.f, cx: v.width / 2, cy: v.height / 2 }));
    return { cams, angles, d: fitD ? p[nc + free.length] : d0 };
  };
  // priors are scaled to about one pixel per percent of the image diagonal
  const pxPerSigma = list.reduce((s, v) => s + Math.hypot(v.width, v.height), 0) / list.length / 100;
  const resid = p => {
    const { cams, angles, d } = unpack(p), frames = fk(angles), r = [];
    list.forEach((v, k) => {
      for (const n of v.names) {
        const [u, w0, w = 1] = v.points[n];
        const q = projectPoint(cams[k], keypointWorld(n, angles, gap, d, frames));
        if (q[2] <= 0.1) { r.push(1e4, 1e4); continue; }
        r.push(w * (q[0] - u), w * (q[1] - w0));
      }
      if (v.fovPrior) r.push(pxPerSigma * (2 * Math.atan(v.height / 2 / cams[k].f) * DEG - v.fovPrior[0]) / v.fovPrior[1]);
    });
    for (const j of free) r.push(pxPerSigma * (angles[j] - a0[j]) / prior);
    if (fitD) r.push(pxPerSigma * (d - d0) / 3);
    return r;
  };
  let p = pack(), r = resid(p), cost = dotSelf(r), mu = 1e-3;
  const n = p.length;
  for (let it = 0; it < iterations; it++) {
    const J = r.map(() => new Array(n).fill(0));
    for (let k = 0; k < n; k++) {
      if (k < nc && k % 7 === 6 && !fitF) continue;
      const h = k < nc ? (k % 7 < 3 || k % 7 === 6 ? 1e-5 : 1e-4) : 1e-4, pp = p.slice(); pp[k] += h;
      const rr = resid(pp);
      for (let i = 0; i < r.length; i++) J[i][k] = (rr[i] - r[i]) / h;
    }
    const JtJ = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => J.reduce((s, row) => s + row[i] * row[j], 0)));
    const Jtr = Array.from({ length: n }, (_, i) => J.reduce((s, row, k) => s + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 12 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((v, j) => v + (i === j ? mu * (JtJ[i][i] || 1) + 1e-12 : 0)));
      let dp; try { dp = solve(A, Jtr.map(v => -v)); } catch { mu *= 10; continue; }
      const pn = p.map((v, k) => v + dp[k]), rn = resid(pn), cn = dotSelf(rn);
      if (cn < cost) { improved = cost - cn > 1e-10 * cost; p = pn; r = rn; cost = cn; mu = Math.max(1e-12, mu / 5); if (!improved) break; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  const { cams, angles, d } = unpack(p), frames = fk(angles);
  const out = list.map((v, k) => {
    const residuals = {}; let s = 0;
    for (const nm of v.names) {
      const q = projectPoint(cams[k], keypointWorld(nm, angles, gap, d, frames));
      const e = [q[0] - v.points[nm][0], q[1] - v.points[nm][1]];
      residuals[nm] = e.map(round1); s += e[0] ** 2 + e[1] ** 2;
    }
    return { camera: lookAtFromCamera(cams[k], v.height), cam: cams[k], rms: +Math.sqrt(s / v.names.length).toFixed(1), residuals };
  });
  for (const j of JOINTS) angles[j] = +angles[j].toFixed(1);
  return { views: out, angles, d: +d.toFixed(2) };
}

/** Jaw gap (cm) for an S6 count. */
export const gapForCounts = c => gapForTheta(gripperTheta(c));

/** Pixel positions of all keypoints (for drawing them on the model panel). */
export function projectKeypoints(cam, angles, gap, d = MAT.sToEdge, filter = () => true) {
  const frames = fk(angles), out = {};
  for (const n of Object.keys(KEYPOINTS)) if (filter(n)) {
    const q = projectPoint(cam, keypointWorld(n, angles, gap, d, frames));
    if (q[2] > 0) out[n] = [round1(q[0]), round1(q[1])];
  }
  return out;
}

function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function norm(a) { const l = Math.hypot(...a); return a.map(v => v / l); }
function dotSelf(r) { return r.reduce((s, v) => s + v * v, 0); }
function round1(v) { return Math.round(v * 10) / 10; }
