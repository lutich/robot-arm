// Cube recognition in a camera frame, in the browser (no OpenCV).
// Assumes the cubes lie on the mat (level 0), as at the start of a task.
//
// 1. Colour classes in HSV per pixel (orange, yellow, green, blue, red; the mat is white or
//    grey and the markers black, so they drop out on saturation and brightness).
// 2. Only pixels whose viewing ray hits the mat inside its printed area count.
// 3. Connected components (4-neighbour) on a half-resolution mask.
// 4. Each blob's pixels are back-projected onto the plane at half cube height; their mean is
//    the cube centre, their principal axis the yaw (mod 90).
// 5. Size check: the blob's pixel area against a 26,5 mm cube projected at that spot. A cube
//    in the gripper or partly hidden fails it and is marked as uncertain.

import { CUBE } from './config.js';
import { project } from './calib.js';

const CUBE_MM = CUBE * 10;
export const COLOUR_RULES = [
  { colour: 'red', test: (h, s, v) => (h < 10 || h > 345) && s > 0.5 && v > 0.3 },
  { colour: 'orange', test: (h, s, v) => h >= 10 && h < 38 && s > 0.5 && v > 0.45 },
  { colour: 'yellow', test: (h, s, v) => h >= 38 && h < 70 && s > 0.45 && v > 0.45 },
  { colour: 'green', test: (h, s, v) => h >= 80 && h < 170 && s > 0.35 && v > 0.2 },
  { colour: 'blue', test: (h, s, v) => h >= 190 && h < 255 && s > 0.4 && v > 0.2 },
];

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
 * Returns [{ colour, X, Y (mm, table frame), yaw, pixels, areaRatio, ok, bbox }].
 */
export function detectCubes(img, cal, { frameW = 1280, frameH = 720, step = 2, minPixels = 120 } = {}) {
  const { width, height, data } = img;
  const sx = frameW / width, sy = frameH / height;
  const gw = Math.floor(width / step), gh = Math.floor(height / step);
  const label = new Int16Array(gw * gh).fill(-1);
  // mat membership is cheap to test per sampled pixel via the plane at Z = 0
  for (let y = 0; y < gh; y++) for (let x = 0; x < gw; x++) {
    const i = 4 * ((y * step) * width + x * step);
    const [h, s, v] = hsv(data[i], data[i + 1], data[i + 2]);
    const k = COLOUR_RULES.findIndex(r => r.test(h, s, v));
    if (k < 0) continue;
    const m = backProject(cal, x * step * sx, y * step * sy, 0);
    if (!m || m[0] < -5 || m[0] > 365 || Math.abs(m[1]) > 145) continue;
    label[y * gw + x] = k;
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
        if (q >= 0 && !seen[q] && label[q] === k) { seen[q] = 1; stack.push(q); }
    }
    if (px.length * step * step < minPixels) continue;
    blobs.push({ k, px });
  }
  const out = [];
  for (const { k, px } of blobs) {
    let sumX = 0, sumY = 0, n = 0, minU = Infinity, minV = Infinity, maxU = 0, maxV = 0;
    const pts = [];
    for (const p of px) {
      const u = (p % gw) * step * sx, v = ((p / gw) | 0) * step * sy;
      minU = Math.min(minU, u); maxU = Math.max(maxU, u); minV = Math.min(minV, v); maxV = Math.max(maxV, v);
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
    out.push({ colour: COLOUR_RULES[k].colour, X, Y, yaw, pixels: Math.round(pixels), areaRatio, ok: areaRatio > 0.55 && areaRatio < 1.6, bbox: [minU, minV, maxU, maxV] });
  }
  // largest blob per colour first; drop tiny fragments next to a bigger blob of the same colour
  out.sort((a, b) => b.pixels - a.pixels);
  return out.filter((c, i) => !out.slice(0, i).some(o => o.colour === c.colour && Math.hypot(o.X - c.X, o.Y - c.Y) < CUBE_MM));
}
