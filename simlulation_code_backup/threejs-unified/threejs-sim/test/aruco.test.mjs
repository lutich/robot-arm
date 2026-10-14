// node --test test/*.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { detectMarkers, DICT } from '../js/aruco.js';
import { calibrate, project, rodrigues, MARKER_CORNERS, SAVED_FRAME_CORNERS } from '../js/calib.js';

/** Render the six mat markers (white mat, black markers with their bits) through camera `cam` into RGBA. */
function renderMat(cam, W = 1280, H = 720) {
  const data = new Uint8ClampedArray(W * H * 4).fill(255);
  // pixel -> mat plane: invert the plane homography of the camera
  const K = [[cam.f, 0, cam.cx], [0, cam.f, cam.cy], [0, 0, 1]];
  const Hm = [0, 1, 2].map(i => [0, 1, 2].map(j => K[i][0] * (j < 2 ? cam.R[0][j] : cam.t[0]) + K[i][1] * (j < 2 ? cam.R[1][j] : cam.t[1]) + K[i][2] * (j < 2 ? cam.R[2][j] : cam.t[2])));
  const det = Hm[0][0] * (Hm[1][1] * Hm[2][2] - Hm[1][2] * Hm[2][1]) - Hm[0][1] * (Hm[1][0] * Hm[2][2] - Hm[1][2] * Hm[2][0]) + Hm[0][2] * (Hm[1][0] * Hm[2][1] - Hm[1][1] * Hm[2][0]);
  const inv = [
    [(Hm[1][1] * Hm[2][2] - Hm[1][2] * Hm[2][1]) / det, (Hm[0][2] * Hm[2][1] - Hm[0][1] * Hm[2][2]) / det, (Hm[0][1] * Hm[1][2] - Hm[0][2] * Hm[1][1]) / det],
    [(Hm[1][2] * Hm[2][0] - Hm[1][0] * Hm[2][2]) / det, (Hm[0][0] * Hm[2][2] - Hm[0][2] * Hm[2][0]) / det, (Hm[0][2] * Hm[1][0] - Hm[0][0] * Hm[1][2]) / det],
    [(Hm[1][0] * Hm[2][1] - Hm[1][1] * Hm[2][0]) / det, (Hm[0][1] * Hm[2][0] - Hm[0][0] * Hm[2][1]) / det, (Hm[0][0] * Hm[1][1] - Hm[0][1] * Hm[1][0]) / det]];
  const SS = 3;                                                   // 3 x 3 supersampling for clean edges
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    let dark = 0;
    for (let sy = 0; sy < SS; sy++) for (let sx = 0; sx < SS; sx++) {
      const u = x + (sx + 0.5) / SS, v = y + (sy + 0.5) / SS;      // pixel centres at +0.5
      const w = inv[2][0] * u + inv[2][1] * v + inv[2][2];
      const X = (inv[0][0] * u + inv[0][1] * v + inv[0][2]) / w, Y = (inv[1][0] * u + inv[1][1] * v + inv[1][2]) / w;
      for (const [id, c] of Object.entries(MARKER_CORNERS)) {
        // corner order: TL=(Xmax,Ymax) TR=(Xmax,Ymin) BR=(Xmin,Ymin) BL=(Xmin,Ymax); rows run -X, columns run -Y
        const Xmax = c[0][0], Ymax = c[0][1];
        const col = Math.floor((Ymax - Y) / 5), row = Math.floor((Xmax - X) / 5);
        if (row < 0 || row > 5 || col < 0 || col > 5) continue;
        const white = row > 0 && row < 5 && col > 0 && col < 5 && DICT[id][row - 1][col - 1] === '1';
        if (!white) dark++;
      }
    }
    const v = 255 - Math.round(235 * dark / (SS * SS));
    const i = 4 * (y * W + x); data[i] = data[i + 1] = data[i + 2] = v;
  }
  return { width: W, height: H, data };
}

const savedCamera = () => {
  const world = [], img = [];
  for (const id of Object.keys(SAVED_FRAME_CORNERS)) MARKER_CORNERS[id].forEach((p, k) => { world.push([p[0], p[1], 0]); img.push(SAVED_FRAME_CORNERS[id][k]); });
  return calibrate(world, img);
};

function checkView(cam, label) {
  const img = renderMat(cam);
  const { markers } = detectMarkers(img);
  const ids = Object.keys(markers).map(Number).sort();
  // every marker that lies fully inside the image must be found
  const inFrame = Object.keys(MARKER_CORNERS).map(Number).filter(id => MARKER_CORNERS[id].every(p => {
    const [u, v] = project(cam, [p[0], p[1], 0]); return u > 2 && v > 2 && u < 1278 && v < 718;
  }));
  assert.deepEqual(ids, inFrame, `${label}: found ${ids}, in frame ${inFrame}`);
  assert.ok(ids.length >= 4, `${label}: too few markers in frame for the test`);
  let worst = 0;
  for (const id of ids) MARKER_CORNERS[id].forEach((p, k) => {
    // image coordinates here put pixel centres at +0.5, as OpenCV does not; shift the truth to match
    const t = project(cam, [p[0], p[1], 0]);
    worst = Math.max(worst, Math.hypot(markers[id][k][0] - t[0], markers[id][k][1] - t[1]));
  });
  assert.ok(worst < 1.5, `${label}: worst corner error ${worst.toFixed(2)} px`);
  // and the camera comes back from the detected corners
  const world = [], uv = [];
  for (const id of ids) MARKER_CORNERS[id].forEach((p, k) => { world.push([p[0], p[1], 0]); uv.push(markers[id][k]); });
  const back = calibrate(world, uv);
  assert.ok(Math.hypot(...back.centre.map((v, i) => v - cam.centre[i])) < 15, `${label}: camera centre off by ${Math.hypot(...back.centre.map((v, i) => v - cam.centre[i])).toFixed(1)} mm`);
  return worst;
}

test('finds all six mat markers with their corners from the original camera position', () => {
  checkView(savedCamera(), 'frontal');
});

test('finds them from an oblique side view too, and recovers the moved camera', () => {
  // the camera moved round to the side of the mat, like the 2026-10-12 17:21 frame
  const c0 = savedCamera();
  const turn = rodrigues([0, 0, 0.6]);                            // 34 deg about the mat's vertical axis
  const pivot = [180, 0, 0];
  const R = c0.R.map(row => [0, 1, 2].map(j => row[0] * turn[0][j] + row[1] * turn[1][j] + row[2] * turn[2][j]));   // R * turn
  const Rt = [0, 1, 2].map(i => R[i][0] * pivot[0] + R[i][1] * pivot[1] + R[i][2] * pivot[2]);
  const Rp0 = [0, 1, 2].map(i => c0.R[i][0] * pivot[0] + c0.R[i][1] * pivot[1] + c0.R[i][2] * pivot[2]);
  const t = c0.t.map((v, i) => v + Rp0[i] - Rt[i]);               // same view of the pivot, from a turned position
  const cam = { ...c0, R, t };
  cam.centre = [0, 1, 2].map(i => -(R[0][i] * t[0] + R[1][i] * t[1] + R[2][i] * t[2]));
  checkView(cam, 'side');
});
