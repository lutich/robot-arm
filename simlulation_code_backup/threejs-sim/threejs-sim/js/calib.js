// Camera calibration from the printed mat: pinhole model (focal length, principal point,
// pose) fitted to points on the mat plane (Z = 0). Pure math, no three.js, no OpenCV.
//
// 1. Homography from >= 4 point pairs (normalised DLT; null vector by Jacobi eigen-solve).
// 2. Focal length from the homography's orthogonality constraints, principal point at the
//    image centre to start with.
// 3. Pose from the homography (columns r1, r2, t), re-orthonormalised.
// 4. Levenberg-Marquardt refinement of the reprojection error over rotation vector,
//    translation, focal length and (optionally) the principal point.
//
// Frames: world = camera table frame of the mat (mm, X away from the arm, Y left, Z up);
// camera = OpenCV convention (x right, y down, z forward). p_cam = R p_world + t.

// ---------------- small linear algebra ----------------

/** Eigen-decomposition of a symmetric n x n matrix (array of rows), Jacobi rotations. */
export function eigSym(A) {
  const n = A.length, a = A.map(r => r.slice()), V = a.map((_, i) => a.map((__, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < 1e-22) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
      for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
    }
  }
  return { values: a.map((r, i) => r[i]), vectors: V };   // column i of V is the eigenvector of values[i]
}

/** Solve A x = b (Gaussian elimination with partial pivoting). */
export function solve(A, b) {
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-15) throw new Error('singular system');
    for (let r = c + 1; r < n; r++) { const f = M[r][c] / M[c][c]; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = M[r][n]; for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k]; x[r] = s / M[r][r]; }
  return x;
}

const mat3mul = (A, B) => A.map((r, i) => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);

export function rodrigues(r) {
  const th = norm(r);
  if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const k = r.map(v => v / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  return [
    [c + k[0] * k[0] * C, k[0] * k[1] * C - k[2] * s, k[0] * k[2] * C + k[1] * s],
    [k[1] * k[0] * C + k[2] * s, c + k[1] * k[1] * C, k[1] * k[2] * C - k[0] * s],
    [k[2] * k[0] * C - k[1] * s, k[2] * k[1] * C + k[0] * s, c + k[2] * k[2] * C]];
}
export function rotvec(R) {
  const c = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)), th = Math.acos(c);
  if (th < 1e-9) return [0, 0, 0];
  if (Math.PI - th < 1e-6) {   // 180 deg: axis from the diagonal
    // R = 2 k k^T - I: signs relative to the largest component (a camera looking straight along -Y has k[0] = 0)
    const k = [0, 1, 2].map(i => Math.sqrt(Math.max(0, (R[i][i] + 1) / 2)));
    const m = k.indexOf(Math.max(...k));
    for (let i = 0; i < 3; i++) if (i !== m && R[m][i] + R[i][m] < 0) k[i] = -k[i];
    return k.map(v => v * th);
  }
  const s = 2 * Math.sin(th);
  return [(R[2][1] - R[1][2]) / s * th, (R[0][2] - R[2][0]) / s * th, (R[1][0] - R[0][1]) / s * th];
}

// ---------------- homography ----------------

/** Homography H (3x3 rows) with p_img ~ H [X, Y, 1]. world: [[X, Y]], img: [[u, v]]. */
export function homography(world, img) {
  const norm2 = pts => {
    const n = pts.length, mx = pts.reduce((s, p) => s + p[0], 0) / n, my = pts.reduce((s, p) => s + p[1], 0) / n;
    const d = pts.reduce((s, p) => s + Math.hypot(p[0] - mx, p[1] - my), 0) / n || 1, k = Math.SQRT2 / d;
    return { T: [[k, 0, -k * mx], [0, k, -k * my], [0, 0, 1]], p: pts.map(p => [k * (p[0] - mx), k * (p[1] - my)]) };
  };
  const W = norm2(world), I = norm2(img);
  const AtA = Array.from({ length: 9 }, () => new Array(9).fill(0));
  const add = row => { for (let i = 0; i < 9; i++) for (let j = 0; j < 9; j++) AtA[i][j] += row[i] * row[j]; };
  W.p.forEach(([X, Y], i) => {
    const [u, v] = I.p[i];
    add([-X, -Y, -1, 0, 0, 0, u * X, u * Y, u]);
    add([0, 0, 0, -X, -Y, -1, v * X, v * Y, v]);
  });
  const { values, vectors } = eigSym(AtA);
  const k = values.indexOf(Math.min(...values));
  const h = vectors.map(r => r[k]);
  const Hn = [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], h[8]]];
  // denormalise: H = Ti^-1 Hn Tw
  const Ti = I.T, s = Ti[0][0];
  const TiInv = [[1 / s, 0, -Ti[0][2] / s], [0, 1 / s, -Ti[1][2] / s], [0, 0, 1]];
  const H = mat3mul(mat3mul(TiInv, Hn), W.T);
  const sc = H[2][2] || 1;
  return H.map(r => r.map(v => v / sc));
}

// ---------------- camera ----------------

export function project(cam, P) {
  const R = cam.R, t = cam.t;
  const x = R[0][0] * P[0] + R[0][1] * P[1] + R[0][2] * P[2] + t[0];
  const y = R[1][0] * P[0] + R[1][1] * P[1] + R[1][2] * P[2] + t[1];
  const z = R[2][0] * P[0] + R[2][1] * P[1] + R[2][2] * P[2] + t[2];
  return [cam.f * x / z + cam.cx, cam.f * y / z + cam.cy];
}

export function rms(cam, world, img) {
  let s = 0;
  world.forEach((P, i) => { const p = project(cam, [P[0], P[1], P[2] || 0]); s += (p[0] - img[i][0]) ** 2 + (p[1] - img[i][1]) ** 2; });
  return Math.sqrt(s / world.length);
}

/** Camera centre in world coordinates: -R^T t. */
export const centre = cam => [0, 1, 2].map(i => -(cam.R[0][i] * cam.t[0] + cam.R[1][i] * cam.t[1] + cam.R[2][i] * cam.t[2]));

/** Initial camera from a homography, square pixels, principal point (cx, cy). */
export function fromHomography(H, cx, cy) {
  const G = [[H[0][0] - cx * H[2][0], H[0][1] - cx * H[2][1], H[0][2] - cx * H[2][2]],
             [H[1][0] - cy * H[2][0], H[1][1] - cy * H[2][1], H[1][2] - cy * H[2][2]],
             H[2].slice()];
  const g = j => [G[0][j], G[1][j], G[2][j]];
  const g1 = g(0), g2 = g(1), g3 = g(2);
  // w = 1/f^2 from  a*w + b = 0  for both constraints, least squares
  const rows = [[g1[0] * g2[0] + g1[1] * g2[1], g1[2] * g2[2]], [g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2, g1[2] ** 2 - g2[2] ** 2]];
  const w = -rows.reduce((s, [a, b]) => s + a * b, 0) / rows.reduce((s, [a]) => s + a * a, 0);
  if (!(w > 0)) throw new Error('could not estimate the focal length from these points (spread them over the mat)');
  const f = 1 / Math.sqrt(w);
  const Dg = v => [v[0] / f, v[1] / f, v[2]];
  let r1 = Dg(g1), r2 = Dg(g2), t = Dg(g3);
  const lam = 2 / (norm(r1) + norm(r2));
  r1 = r1.map(v => v * lam); r2 = r2.map(v => v * lam); t = t.map(v => v * lam);
  if (t[2] < 0) { r1 = r1.map(v => -v); r2 = r2.map(v => -v); t = t.map(v => -v); }
  // orthonormalise
  const n1 = norm(r1); r1 = r1.map(v => v / n1);
  const d = r1[0] * r2[0] + r1[1] * r2[1] + r1[2] * r2[2];
  r2 = r2.map((v, i) => v - d * r1[i]); const n2 = norm(r2); r2 = r2.map(v => v / n2);
  const r3 = cross(r1, r2);
  const R = [[r1[0], r2[0], r3[0]], [r1[1], r2[1], r3[1]], [r1[2], r2[2], r3[2]]];
  return { R, t, f, cx, cy };
}

/**
 * Full calibration. world: [[X, Y, Z]] (mm), img: [[u, v]] (px).
 * Options: width/height of the image, fixPrincipal (keep the principal point at the centre).
 */
export function calibrate(world, img, { width = 1280, height = 720, fixPrincipal = false, iterations = 60 } = {}) {
  if (world.length < 4) throw new Error('need at least 4 points');
  const H = homography(world.map(p => [p[0], p[1]]), img);
  let cam = fromHomography(H, width / 2, height / 2);
  const pack = c => [...rotvec(c.R), ...c.t, c.f, c.cx, c.cy];
  const unpack = p => ({ R: rodrigues(p.slice(0, 3)), t: p.slice(3, 6), f: p[6], cx: p[7], cy: p[8] });
  const nfree = fixPrincipal ? 7 : 9;
  const resid = p => { const c = unpack(p); const r = []; world.forEach((P, i) => { const q = project(c, P); r.push(q[0] - img[i][0], q[1] - img[i][1]); }); return r; };
  let p = pack(cam), r = resid(p), cost = r.reduce((s, v) => s + v * v, 0), mu = 1e-3;
  for (let it = 0; it < iterations; it++) {
    const J = r.map(() => new Array(nfree).fill(0));
    for (let k = 0; k < nfree; k++) {
      const h = k < 3 ? 1e-6 : k < 6 ? 1e-3 : 1e-3;
      const pp = p.slice(); pp[k] += h;
      const rr = resid(pp);
      for (let i = 0; i < r.length; i++) J[i][k] = (rr[i] - r[i]) / h;
    }
    const JtJ = Array.from({ length: nfree }, (_, i) => Array.from({ length: nfree }, (_, j) => J.reduce((s, row) => s + row[i] * row[j], 0)));
    const Jtr = Array.from({ length: nfree }, (_, i) => J.reduce((s, row, k) => s + row[i] * r[k], 0));
    let improved = false;
    for (let tries = 0; tries < 10 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((v, j) => v + (i === j ? mu * (JtJ[i][i] || 1) : 0)));
      let dp;
      try { dp = solve(A, Jtr.map(v => -v)); } catch { mu *= 10; continue; }
      const pn = p.slice(); for (let k = 0; k < nfree; k++) pn[k] += dp[k];
      const rn = resid(pn), cn = rn.reduce((s, v) => s + v * v, 0);
      if (cn < cost) { p = pn; r = rn; improved = cost - cn > 1e-12 * cost; cost = cn; mu = Math.max(1e-9, mu / 5); if (!improved) break; improved = true; }
      else mu *= 10;
    }
    if (!improved) break;
  }
  cam = unpack(p);
  return { ...cam, rms: Math.sqrt(cost / world.length), centre: centre(cam), H };
}

// ---------------- the mat ----------------

/** Corners (mm, table frame) of the six mat markers; order as OpenCV reports them. */
export const MARKER_CORNERS = {
  0: [[40, 125], [40, 95], [10, 95], [10, 125]], 1: [[40, -95], [40, -125], [10, -125], [10, -95]],
  2: [[175, 125], [175, 95], [145, 95], [145, 125]], 3: [[175, -95], [175, -125], [145, -125], [145, -95]],
  4: [[350, 125], [350, 95], [320, 95], [320, 125]], 5: [[350, -95], [350, -125], [320, -125], [320, -95]],
};
export const markerCentre = id => [MARKER_CORNERS[id].reduce((s, p) => s + p[0], 0) / 4, MARKER_CORNERS[id].reduce((s, p) => s + p[1], 0) / 4];

/** Marker corners found by vision/blocks.py in docs/images/photos/controller-camera-2026-10-12.jpg. */
export const SAVED_FRAME_CORNERS = {
  0: [[902, 107], [851, 108], [846, 76], [895, 75]], 1: [[527, 112], [476, 112], [482, 81], [530, 80]],
  2: [[941, 285], [881, 285], [874, 243], [932, 242]], 3: [[504, 290], [446, 290], [453, 247], [509, 246]],
  4: [[1014, 628], [939, 628], [927, 558], [999, 557]], 5: [[461, 629], [388, 628], [400, 559], [470, 559]],
};
