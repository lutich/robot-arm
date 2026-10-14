// Marker detection in plain JS: the mat's ArUco markers (DICT_4X4_50, ids 0..5) and the AprilTags
// on the arm (36h11, e.g. 71 and 72; family '36h11').
// Works on any camera position: the camera is then calibrated from the detected corners.
//
// 1. Grey image, adaptive threshold (local mean over a window, integral image).
// 2. Connected dark regions; each region's boundary pixels -> convex hull -> best quadrilateral.
// 3. Corners refined to sub-pixel by fitting a line to each edge and intersecting neighbours.
// 4. The quad is sampled as a 6 x 6 grid (black border + 4 x 4 bits) through its homography,
//    thresholded with Otsu, checked for a black border, and matched against the dictionary in
//    all four rotations (Hamming distance <= 1).
// 5. Corners are returned in OpenCV's order (clockwise from the marker's top-left), the same
//    order as calib.js MARKER_CORNERS, so they can go straight into the calibration.

import { APRILTAG_36H11, tagBits } from './apriltag36h11.js';

// DICT_4X4_50 inner bits for ids 0..5, rows top to bottom, 1 = white (from OpenCV).
export const DICT = {
  0: ['1011', '0101', '0011', '0010'], 1: ['0000', '1111', '1001', '1010'], 2: ['0011', '0011', '0010', '1101'],
  3: ['1001', '1001', '0100', '0110'], 4: ['0101', '0100', '1001', '1110'], 5: ['0111', '1001', '1100', '1101'],
};
const CODES = Object.entries(DICT).map(([id, rows]) => ({ id: Number(id), bits: rows.join('').split('').map(Number) }));
let CODES_36H11 = null;
/** Code lists per family: { n (data cells per side), maxErr (bits), codes: [{ id, bits }] }. */
function family(name, ids) {
  if (name === '4x4') return { n: 4, maxErr: 1, codes: CODES };
  if (name !== '36h11') throw new Error(`unknown marker family ${name}`);
  CODES_36H11 ??= APRILTAG_36H11.map((_, id) => ({ id, bits: tagBits(id) }));
  return { n: 6, maxErr: 2, codes: ids ? CODES_36H11.filter(c => ids.includes(c.id)) : CODES_36H11 };
}

/** Grey values (0..255) from RGBA ImageData. */
export function toGrey({ width, height, data }) {
  const g = new Float32Array(width * height);
  for (let i = 0, j = 0; i < g.length; i++, j += 4) g[i] = 0.299 * data[j] + 0.587 * data[j + 1] + 0.114 * data[j + 2];
  return g;
}

function bilinear(g, w, h, x, y) {
  const x0 = Math.max(0, Math.min(w - 2, Math.floor(x))), y0 = Math.max(0, Math.min(h - 2, Math.floor(y)));
  const fx = Math.min(1, Math.max(0, x - x0)), fy = Math.min(1, Math.max(0, y - y0)), i = y0 * w + x0;
  return (g[i] * (1 - fx) + g[i + 1] * fx) * (1 - fy) + (g[i + w] * (1 - fx) + g[i + w + 1] * fx) * fy;
}

/** Homography mapping the unit square (0,0),(1,0),(1,1),(0,1) to quad q0..q3. */
function squareToQuad(q) {
  const [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] = q;
  const dx1 = x1 - x2, dx2 = x3 - x2, dx3 = x0 - x1 + x2 - x3, dy1 = y1 - y2, dy2 = y3 - y2, dy3 = y0 - y1 + y2 - y3;
  const den = dx1 * dy2 - dx2 * dy1;
  const g = (dx3 * dy2 - dx2 * dy3) / den, h = (dx1 * dy3 - dx3 * dy1) / den;
  return (u, v) => {
    const W = g * u + h * v + 1;
    return [((x1 - x0 + g * x1) * u + (x3 - x0 + h * x3) * v + x0) / W, ((y1 - y0 + g * y1) * u + (y3 - y0 + h * y3) * v + y0) / W];
  };
}

function convexHull(pts) {
  pts = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
const polyArea = p => { let a = 0; for (let i = 0; i < p.length; i++) { const q = p[i], r = p[(i + 1) % p.length]; a += q[0] * r[1] - r[0] * q[1]; } return a / 2; };

/** The four hull vertices spanning the largest quadrilateral (rotating search, hull sizes are small). */
function bestQuad(h) {
  const n = h.length;
  if (n < 4) return null;
  // farthest pair as the first diagonal
  let bi = 0, bj = 1, bd = -1;
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) { const d = (h[i][0] - h[j][0]) ** 2 + (h[i][1] - h[j][1]) ** 2; if (d > bd) { bd = d; bi = i; bj = j; } }
  const side = (a, b, p) => (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
  let k = -1, kv = 0, l = -1, lv = 0;
  for (let i = 0; i < n; i++) {
    const s = side(h[bi], h[bj], h[i]);
    if (s > kv) { kv = s; k = i; }
    if (s < lv) { lv = s; l = i; }
  }
  if (k < 0 || l < 0) return null;
  const q = [h[bi], h[k], h[bj], h[l]];
  return polyArea(q) < 0 ? [q[0], q[3], q[2], q[1]] : q;    // positive area = clockwise on screen (y down)
}

/** Refine quad corners: fit a line to the boundary pixels of each edge, intersect neighbours. */
function refine(quad, boundary) {
  const lines = [];
  for (let e = 0; e < 4; e++) {
    const a = quad[e], b = quad[(e + 1) % 4], dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy);
    const nx = -dy / L, ny = dx / L;
    let sx = 0, sy = 0, n = 0; const pts = [];
    for (const p of boundary) {
      const t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (L * L), d = (p[0] - a[0]) * nx + (p[1] - a[1]) * ny;
      if (t > 0.15 && t < 0.85 && Math.abs(d) < 2.5) { pts.push(p); sx += p[0]; sy += p[1]; n++; }
    }
    if (n < 4) return quad;
    const mx = sx / n, my = sy / n;
    let sxx = 0, syy = 0, sxy = 0;
    for (const p of pts) { sxx += (p[0] - mx) ** 2; syy += (p[1] - my) ** 2; sxy += (p[0] - mx) * (p[1] - my); }
    const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    // boundary pixels are the dark pixels on the edge; the true edge lies half a pixel further out
    const dir = [Math.cos(ang), Math.sin(ang)];
    let out = [-dir[1], dir[0]];
    const cx = (quad[0][0] + quad[1][0] + quad[2][0] + quad[3][0]) / 4, cy = (quad[0][1] + quad[1][1] + quad[2][1] + quad[3][1]) / 4;
    if ((mx - cx) * out[0] + (my - cy) * out[1] < 0) out = [-out[0], -out[1]];
    lines.push({ p: [mx + 0.5 * out[0], my + 0.5 * out[1]], d: dir });
  }
  const meet = (l1, l2) => {
    const den = l1.d[0] * l2.d[1] - l1.d[1] * l2.d[0];
    if (Math.abs(den) < 1e-6) return null;
    const t = ((l2.p[0] - l1.p[0]) * l2.d[1] - (l2.p[1] - l1.p[1]) * l2.d[0]) / den;
    return [l1.p[0] + t * l1.d[0], l1.p[1] + t * l1.d[1]];
  };
  const out = [];
  for (let c = 0; c < 4; c++) {
    const p = meet(lines[(c + 3) % 4], lines[c]);                 // corner c lies between edge c-1 and edge c
    if (!p || Math.hypot(p[0] - quad[c][0], p[1] - quad[c][1]) > 4) return quad;
    out.push(p);
  }
  return out;
}

/** Read the (n + 2) x (n + 2) cells of a quad (black border + n x n data); returns the n * n data bits or null. */
function readBits(g, w, h, quad, n = 4) {
  const map = squareToQuad(quad), samples = [], G = n + 2, N = G * G;
  for (let r = 0; r < G; r++) for (let c = 0; c < G; c++) {
    let s = 0;
    for (const [du, dv] of [[0.35, 0.35], [0.65, 0.35], [0.35, 0.65], [0.65, 0.65], [0.5, 0.5]]) { const [x, y] = map((c + du) / G, (r + dv) / G); s += bilinear(g, w, h, x, y); }
    samples.push(s / 5);
  }
  // Otsu threshold over the cell means
  const sorted = samples.slice().sort((a, b) => a - b);
  let best = 0, thr = sorted[N >> 1];
  for (let i = 1; i < N; i++) {
    const a = sorted.slice(0, i), b = sorted.slice(i), ma = a.reduce((s, v) => s + v, 0) / a.length, mb = b.reduce((s, v) => s + v, 0) / b.length;
    const v = a.length * b.length * (ma - mb) ** 2;
    if (v > best) { best = v; thr = (sorted[i - 1] + sorted[i]) / 2; }
  }
  if (sorted[N - 1] - sorted[0] < 25) return null;                // no contrast
  const cell = (r, c) => (samples[r * G + c] > thr ? 1 : 0);
  let borderWhite = 0;
  for (let i = 0; i < G; i++) borderWhite += cell(0, i) + cell(G - 1, i) + (i > 0 && i < G - 1 ? cell(i, 0) + cell(i, G - 1) : 0);
  if (borderWhite > 1) return null;
  const bits = [];
  for (let r = 1; r <= n; r++) for (let c = 1; c <= n; c++) bits.push(cell(r, c));
  return bits;
}

const rotCW = (b, n = 4) => { const o = new Array(n * n); for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) o[r * n + c] = b[(n - 1 - c) * n + r]; return o; };

/**
 * Detect markers. img: ImageData. Returns { [id]: [[x,y] x 4] } in OpenCV corner order,
 * plus `candidates` (quads looked at) for display.
 * family: '4x4' (the mat, default) or '36h11' (AprilTags; `ids` limits the search, e.g. [71, 72]).
 */
export function detectMarkers(img, { window = 31, offset = 7, minArea = 60, maxArea = 60000, family: fam = '4x4', ids = null } = {}) {
  const F = family(fam, ids);
  const { width: w, height: h } = img;
  const g = toGrey(img);
  // integral image for the local mean
  const I = new Float64Array((w + 1) * (h + 1));
  for (let y = 0; y < h; y++) { let row = 0; for (let x = 0; x < w; x++) { row += g[y * w + x]; I[(y + 1) * (w + 1) + x + 1] = I[y * (w + 1) + x + 1] + row; } }
  const r = window >> 1, dark = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    const y0 = Math.max(0, y - r), y1 = Math.min(h, y + r + 1);
    for (let x = 0; x < w; x++) {
      const x0 = Math.max(0, x - r), x1 = Math.min(w, x + r + 1);
      const sum = I[y1 * (w + 1) + x1] - I[y0 * (w + 1) + x1] - I[y1 * (w + 1) + x0] + I[y0 * (w + 1) + x0];
      if (g[y * w + x] < sum / ((x1 - x0) * (y1 - y0)) - offset) dark[y * w + x] = 1;
    }
  }
  // connected dark regions (4-neighbour) and their boundary pixels
  const label = new Int32Array(w * h).fill(-1), found = {}, candidates = [];
  let next = 0;
  const stack = [];
  for (let start = 0; start < w * h; start++) {
    if (!dark[start] || label[start] >= 0) continue;
    const id = next++;
    label[start] = id; stack.push(start);
    const boundary = [];
    let area = 0;
    while (stack.length) {
      const p = stack.pop(); area++;
      const x = p % w, y = (p / w) | 0;
      let edge = false;
      for (const [q, ok] of [[p - 1, x > 0], [p + 1, x < w - 1], [p - w, y > 0], [p + w, y < h - 1]]) {
        if (!ok || !dark[q]) { edge = true; continue; }
        if (label[q] < 0) { label[q] = id; stack.push(q); }
      }
      if (edge) boundary.push([x, y]);
    }
    if (area < minArea || area > maxArea || boundary.length < 16) continue;
    const hull = convexHull(boundary);
    const quad = bestQuad(hull);
    if (!quad) continue;
    const qa = polyArea(quad), ha = Math.abs(polyArea(hull));
    if (qa < 0.85 * ha || qa < 80) continue;                         // not quadrilateral enough
    const sides = quad.map((p, i) => Math.hypot(p[0] - quad[(i + 1) % 4][0], p[1] - quad[(i + 1) % 4][1]));
    if (Math.min(...sides) < 7 || Math.max(...sides) / Math.min(...sides) > 4) continue;
    const q = refine(quad, boundary);
    candidates.push(q);
    const bits = readBits(g, w, h, q, F.n);
    if (!bits) continue;
    // Rotate the observation clockwise k times until it equals a code. Then the observation is the
    // code rotated clockwise m = (4 - k) % 4 times, and the code's top-left cell sits at quad corner m
    // (one clockwise turn moves the top-left cell to the top-right corner).
    let b = bits;
    for (let k = 0; k < 4; k++) {
      for (const code of F.codes) {
        let d = 0; for (let i = 0; i < b.length && d <= F.maxErr; i++) d += b[i] !== code.bits[i];
        if (d <= F.maxErr && !found[code.id]) {
          const m = (4 - k) % 4;
          found[code.id] = [0, 1, 2, 3].map(i => q[(m + i) % 4]);
        }
      }
      b = rotCW(b, F.n);
    }
  }
  return { markers: found, candidates };
}
