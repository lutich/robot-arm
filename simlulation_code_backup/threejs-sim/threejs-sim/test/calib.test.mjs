// node --test test/*.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { calibrate, homography, project, rodrigues, rotvec, MARKER_CORNERS, SAVED_FRAME_CORNERS, markerCentre } from '../js/calib.js';

const pairs = () => {
  const world = [], img = [];
  for (const id of Object.keys(SAVED_FRAME_CORNERS)) MARKER_CORNERS[id].forEach((p, k) => { world.push([p[0], p[1], 0]); img.push(SAVED_FRAME_CORNERS[id][k]); });
  return { world, img };
};

test('reproduces the OpenCV fit of vision/blocks.py on the saved frame', () => {
  const { world, img } = pairs();
  const cam = calibrate(world, img);
  // blocks.py / OpenCV on the same 24 corners: f 950.4, principal point (654.6, 359.6),
  // centre (480.8, -8.3, 371.7) mm, 0.88 px rms
  assert.ok(Math.abs(cam.f - 950.4) < 3, `f ${cam.f}`);
  assert.ok(Math.abs(cam.cx - 654.6) < 4 && Math.abs(cam.cy - 359.6) < 4, `pp ${cam.cx} ${cam.cy}`);
  assert.ok(Math.hypot(cam.centre[0] - 480.8, cam.centre[1] + 8.3, cam.centre[2] - 371.7) < 3, `centre ${cam.centre}`);
  assert.ok(cam.rms < 1.0, `rms ${cam.rms}`);
});

test('a homography maps the mat to the image', () => {
  const { world, img } = pairs();
  const H = homography(world.map(p => [p[0], p[1]]), img);
  const [X, Y] = markerCentre(2);
  const w = H[2][0] * X + H[2][1] * Y + H[2][2];
  const u = (H[0][0] * X + H[0][1] * Y + H[0][2]) / w, v = (H[1][0] * X + H[1][1] * Y + H[1][2]) / w;
  const c = SAVED_FRAME_CORNERS[2].reduce((s, p) => [s[0] + p[0] / 4, s[1] + p[1] / 4], [0, 0]);
  assert.ok(Math.hypot(u - c[0], v - c[1]) < 2, `${u},${v} vs ${c}`);
});

test('six clicked marker centres are enough for a usable camera', () => {
  const world = [], img = [];
  for (const id of Object.keys(SAVED_FRAME_CORNERS)) {
    const [X, Y] = markerCentre(id);
    world.push([X, Y, 0]);
    img.push(SAVED_FRAME_CORNERS[id].reduce((s, p) => [s[0] + p[0] / 4 + 0.8 * (Math.random() - 0.5), s[1] + p[1] / 4 + 0.8 * (Math.random() - 0.5)], [0, 0]));
  }
  const cam = calibrate(world, img, { fixPrincipal: true });
  assert.ok(Math.abs(cam.f - 950) < 40, `f ${cam.f}`);
  assert.ok(Math.hypot(cam.centre[0] - 480.8, cam.centre[1] + 8.3, cam.centre[2] - 371.7) < 25, `centre ${cam.centre}`);
  const p = project(cam, [175, 0, 0]);
  assert.ok(p[0] > 0 && p[0] < 1280 && p[1] > 0 && p[1] < 720);
});

test('rotvec round trip, also at 180 degrees about any axis', () => {
  const Rs = [[[-1, 0, 0], [0, 0, -1], [0, -1, 0]], [[-1, 0, 0], [0, -1, 0], [0, 0, 1]], [[0, 1, 0], [1, 0, 0], [0, 0, -1]], [[0, 0, 1], [0, -1, 0], [1, 0, 0]]];
  for (const R of Rs) {
    const back = rodrigues(rotvec(R));
    R.forEach((row, i) => row.forEach((v, j) => assert.ok(Math.abs(back[i][j] - v) < 1e-9, `R[${i}][${j}] ${back[i][j]} vs ${v}`)));
  }
});
