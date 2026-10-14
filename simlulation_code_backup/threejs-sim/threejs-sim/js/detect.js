// Cube recognition in a camera frame, in the browser (no OpenCV).
// Assumes the cubes lie on the mat (level 0), as at the start of a task.
//
// 1. Colour classes in HSV per pixel (orange, yellow, green, blue, red; the mat is white or
//    grey and the markers black, so they drop out on saturation and brightness). Pale pixels of the
//    same hue (a brightly lit face) are added to a blob where they touch it (hysteresis).
// 2. Only pixels whose viewing ray hits the mat inside its printed area count.
// 3. Connected components (4-neighbour) on a half-resolution mask.
// 4. Each blob's pixels are back-projected onto the plane at half cube height; their mean is
//    the cube centre, their principal axis the yaw (mod 90).
// 5. Size check: the blob's pixel area against a 26,5 mm cube projected at that spot. A cube
//    in the gripper or partly hidden fails it and is marked as uncertain.

import { CUBE } from './config.js';
import { project } from './calib.js';

const CUBE_MM = CUBE * 10;
// test: a pixel that surely belongs to a cube (a blob starts there); hue: the same colour, looser, for
// pale pixels (a brightly lit top face) that join a blob only where they touch it.
export const COLOUR_RULES = [
  { colour: 'red', test: (h, s, v) => (h < 10 || h > 345) && s > 0.5 && v > 0.3, hue: h => h < 12 || h > 340 },
  { colour: 'orange', test: (h, s, v) => h >= 10 && h < 38 && s > 0.5 && v > 0.45, hue: h => h >= 8 && h < 40 },
  { colour: 'yellow', test: (h, s, v) => h >= 38 && h < 70 && s > 0.45 && v > 0.45, hue: h => h >= 36 && h < 72 },
  { colour: 'green', test: (h, s, v) => h >= 80 && h < 170 && s > 0.35 && v > 0.2, hue: h => h >= 78 && h < 172 },
  { colour: 'blue', test: (h, s, v) => h >= 190 && h < 255 && s > 0.4 && v > 0.2, hue: h => h >= 186 && h < 258 },
];
const WEAK_S = 0.14, WEAK_V = 0.3;
const onMatArea = m => !!m && m[0] > -25 && m[0] < 385 && Math.abs(m[1]) < 165;   // table mm, the mat with 2 cm around it

function hsv(r, g, b) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
  let h = 0;
  if (d > 0) {
    if (max === r) h = 60 * (((g - b) / d) % 6);
    else if (max === g) h = 60 * ((b - r) / d + 2);
    else h = 60 * ((r - g) / d + 4);
  }
  if (h < 0) h += 360;
  return [h, max ? d / max : 0, max / 255];
}

/** Ray from pixel (u, v) through the camera, intersected with the plane Z = z (table frame, mm). */
export function backProject(cal, u, v, z) {
  const d = [(u - cal.cx) / cal.f, (v - cal.cy) / cal.f, 1];
  const R = cal.R, C = cal.centre;
  const w = [R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2], R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2], R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2]];
  if (Math.abs(w[2]) < 1e-9) return null;
  const s = (z - C[2]) / w[2];
  return s > 0 ? [C[0] + s * w[0], C[1] + s * w[1]] : null;
}

function hullArea(pts) {
  pts = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (const p of pts.reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  const h = lo.slice(0, -1).concat(up.slice(0, -1));
  let a = 0; for (let i = 0; i < h.length; i++) { const p = h[i], q = h[(i + 1) % h.length]; a += p[0] * q[1] - q[0] * p[1]; }
  return Math.abs(a) / 2;
}

/** Pixel area of a cube standing on the mat at (X, Y), seen by the camera. */
export function expectedArea(cal, X, Y, yaw = 0) {
  const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), h = CUBE_MM / 2;
  const pts = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const z of [0, CUBE_MM]) pts.push(project(cal, [X + sx * h * c - sy * h * s, Y + sx * h * s + sy * h * c, z]));
  return hullArea(pts);
}

/**
 * Find cubes in an image. img: ImageData of the full 1280 x 720 frame (or any size; it is
 * scaled to the calibration's 1280 x 720 pixel grid). cal: camera from calib.js.
 * Returns [{ colour, X, Y (mm, table frame, assuming level 0), yaw, pixels, areaRatio, ok, bbox, uv (blob centroid, px),
 * samples (the blob's sampled pixels, px), cell (px area per sample) }].
 */
export function detectCubes(img, cal, { frameW = 1280, frameH = 720, step = 2, minPixels = 120, refine = false } = {}) {
  const { width, height, data } = img;
  const sx = frameW / width, sy = frameH / height;
  const gw = Math.floor(width / step), gh = Math.floor(height / step);
  const label = new Int16Array(gw * gh).fill(-1), weak = new Int16Array(gw * gh).fill(-1);
  // mat membership is cheap to test per sampled pixel via the plane at Z = 0
  for (let y = 0; y < gh; y++) for (let x = 0; x < gw; x++) {
    const i = 4 * ((y * step) * width + x * step);
    const [h, s, v] = hsv(data[i], data[i + 1], data[i + 2]);
    const k = COLOUR_RULES.findIndex(r => r.test(h, s, v));
    const kw = k >= 0 ? k : s > WEAK_S && v > WEAK_V ? COLOUR_RULES.findIndex(r => r.hue(h)) : -1;
    if (kw < 0) continue;
    // on the mat (2 cm margin) seen at mat height or at two cubes' height: a cube's top face lands far behind it on
    // the mat plane, so cubes at the edge, stacked or held would be cut off by the mat-plane test alone
    const u0 = x * step * sx, v0 = y * step * sy;
    if (!onMatArea(backProject(cal, u0, v0, 0)) && !onMatArea(backProject(cal, u0, v0, 2 * CUBE_MM))) continue;
    if (k >= 0) label[y * gw + x] = k; else weak[y * gw + x] = kw;
  }
  const seen = new Uint8Array(gw * gh), blobs = [];
  for (let start = 0; start < gw * gh; start++) {
    if (label[start] < 0 || seen[start]) continue;
    const k = label[start], stack = [start], px = [];
    seen[start] = 1;
    while (stack.length) {
      const p = stack.pop(); px.push(p);
      const x = p % gw, y = (p / gw) | 0;
      for (const q of [x > 0 ? p - 1 : -1, x < gw - 1 ? p + 1 : -1, y > 0 ? p - gw : -1, y < gh - 1 ? p + gw : -1])
        if (q >= 0 && !seen[q] && (label[q] === k || weak[q] === k)) { seen[q] = 1; stack.push(q); }   // pale pixels join, never start
    }
    if (px.length * step * step < minPixels) continue;
    blobs.push({ k, px });
  }
  // each blob grown again at full resolution with its own learned colour (refineBlob), back on the sampling grid
  if (refine) for (const b of blobs) {
    const full = refineBlob(img, b.px.map(p => [(p % gw) * step, ((p / gw) | 0) * step]));
    const idx = new Set();
    for (const [x, y] of full) if (x % step === 0 && y % step === 0 && x / step < gw && y / step < gh) idx.add((y / step) * gw + x / step);
    if (idx.size >= b.px.length) b.px = [...idx];
  }
  const out = [];
  for (const { k, px } of blobs) {
    let sumX = 0, sumY = 0, n = 0, minU = Infinity, minV = Infinity, maxU = 0, maxV = 0, sumU = 0, sumV = 0;
    const pts = [];
    for (const p of px) {
      const u = (p % gw) * step * sx, v = ((p / gw) | 0) * step * sy;
      minU = Math.min(minU, u); maxU = Math.max(maxU, u); minV = Math.min(minV, v); maxV = Math.max(maxV, v);
      sumU += u; sumV += v;
      const m = backProject(cal, u, v, CUBE_MM / 2);
      if (!m) continue;
      pts.push(m); sumX += m[0]; sumY += m[1]; n++;
    }
    if (!n) continue;
    const X = sumX / n, Y = sumY / n;
    // principal axis of the footprint -> yaw, folded to -45..45
    let sxx = 0, syy = 0, sxy = 0;
    for (const [a, b] of pts) { sxx += (a - X) ** 2; syy += (b - Y) ** 2; sxy += (a - X) * (b - Y); }
    let yaw = 0.5 * Math.atan2(2 * sxy, sxx - syy) * 180 / Math.PI;
    yaw = ((yaw + 45) % 90 + 90) % 90 - 45;
    const pixels = px.length * step * step * sx * sy;
    const areaRatio = pixels / expectedArea(cal, X, Y, yaw);
    // uv: the blob's centroid in the image (1280 x 720 grid), for locating the cube in 3D from two cameras (locate3d.js)
    const uv = [sumU / px.length + step * sx / 2, sumV / px.length + step * sy / 2];
    // the blob's sampled pixels (frame grid, cell centres) and the area each stands for: for fitting a cube's pose to it
    const samples = px.map(p => [((p % gw) + 0.5) * step * sx, (((p / gw) | 0) + 0.5) * step * sy]);
    out.push({ colour: COLOUR_RULES[k].colour, X, Y, yaw, pixels: Math.round(pixels), areaRatio, ok: areaRatio > 0.55 && areaRatio < 1.6, bbox: [minU, minV, maxU, maxV], uv, samples, cell: step * step * sx * sy });
  }
  // largest blob per colour first; drop tiny fragments next to a bigger blob of the same colour
  out.sort((a, b) => b.pixels - a.pixels);
  return out.filter((c, i) => !out.slice(0, i).some(o => o.colour === c.colour && Math.hypot(o.X - c.X, o.Y - c.Y) < CUBE_MM));
}

// ---- refinement: each blob grown again at full resolution with its own colour, in CIELAB, holes closed.
// Off by default (detectCubes refine): on the edge-case frames it let golden yellow cubes grow into orange (their
// shaded faces fall in the orange range) and cost several times the time, for little gain on single cubes.

const LIN = Float32Array.from({ length: 256 }, (_, i) => { const c = i / 255; return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4; });
const fLab = t => (t > 0.008856 ? Math.cbrt(t) : 7.787 * t + 16 / 116);
/** sRGB -> CIELAB (D65): L lightness, a/b the colour. Light and shadow mostly change L, not the hue angle atan2(b, a). */
export function rgbToLab(r, g, b) {
  const R = LIN[r], G = LIN[g], B = LIN[b];
  const fx = fLab((0.4124 * R + 0.3576 * G + 0.1805 * B) / 0.95047), fy = fLab(0.2126 * R + 0.7152 * G + 0.0722 * B), fz = fLab((0.0193 * R + 0.1192 * G + 0.9505 * B) / 1.08883);
  return [116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)];
}
const HUE_TOL = 25, CHROMA_MIN = 7;   // degrees around the blob's own hue; chroma the white mat and grey markers stay below

/**
 * One blob grown again at full resolution in a window around it (half its size on each side): its colour is
 * learned from its seed pixels (the strict rules): mean hue angle and chroma in CIELAB. Neighbours join while
 * their hue is within HUE_TOL of it and their chroma reaches a quarter of the blob's (pale lit faces and
 * shadowed ones of the same cube do, the mat does not). Then small holes (glare, a screw in front) are closed.
 * seeds: [[x, y]] image px. Returns the mask's pixels as [[x, y]] (image px).
 */
export function refineBlob(img, seeds) {
  const W = img.width, Hh = img.height, d = img.data;
  let minX = Infinity, minY = Infinity, maxX = 0, maxY = 0;
  for (const [x, y] of seeds) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y); }
  const mx = Math.max(4, (maxX - minX) / 2), my = Math.max(4, (maxY - minY) / 2);
  const x0 = Math.max(0, Math.floor(minX - mx)), y0 = Math.max(0, Math.floor(minY - my)), x1 = Math.min(W - 1, Math.ceil(maxX + mx)), y1 = Math.min(Hh - 1, Math.ceil(maxY + my));
  const w = x1 - x0 + 1, h = y1 - y0 + 1, n = w * h;
  const A = new Float32Array(n), B = new Float32Array(n);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const i = 4 * ((y + y0) * W + x + x0), [, a, b] = rgbToLab(d[i], d[i + 1], d[i + 2]);
    A[y * w + x] = a; B[y * w + x] = b;
  }
  // the blob's own colour from its seeds
  let sa = 0, sb = 0, k = 0;
  for (const [x, y] of seeds) { const j = (y - y0) * w + (x - x0); if (j >= 0 && j < n) { sa += A[j]; sb += B[j]; k++; } }
  if (!k) return seeds;
  const hue0 = Math.atan2(sb / k, sa / k), c0 = Math.hypot(sa / k, sb / k), cMin = Math.max(CHROMA_MIN, 0.25 * c0), cosTol = Math.cos(HUE_TOL * Math.PI / 180);
  const ca = Math.cos(hue0), cb = Math.sin(hue0);
  const fits = j => { const c = Math.hypot(A[j], B[j]); return c >= cMin && (A[j] * ca + B[j] * cb) / c >= cosTol; };
  // grow from the seeds (4-neighbour)
  const mask = new Uint8Array(n), stack = [];
  for (const [x, y] of seeds) { const j = (y - y0) * w + (x - x0); if (j >= 0 && j < n && !mask[j]) { mask[j] = 1; stack.push(j); } }
  while (stack.length) {
    const j = stack.pop(), x = j % w, y = (j / w) | 0;
    for (const q of [x > 0 ? j - 1 : -1, x < w - 1 ? j + 1 : -1, y > 0 ? j - w : -1, y < h - 1 ? j + w : -1])
      if (q >= 0 && !mask[q] && fits(q)) { mask[q] = 1; stack.push(q); }
  }
  // close small holes: a pixel with at least 5 of its 8 neighbours in the blob joins (twice)
  for (let pass = 0; pass < 2; pass++) {
    const add = [];
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
      const j = y * w + x; if (mask[j]) continue;
      const s = mask[j - 1] + mask[j + 1] + mask[j - w] + mask[j + w] + mask[j - w - 1] + mask[j - w + 1] + mask[j + w - 1] + mask[j + w + 1];
      if (s >= 5) add.push(j);
    }
    for (const j of add) mask[j] = 1;
  }
  const out = [];
  for (let j = 0; j < n; j++) if (mask[j]) out.push([(j % w) + x0, ((j / w) | 0) + y0]);
  return out;
}
