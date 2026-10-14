import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fitView, cameraFromLookAt, lookAtFromCamera, projectKeypoints } from '../js/viewfit.js';

test('lookAt camera round trip, including roll', () => {
  const v = { pos: [-30, 20, 26], target: [0, 0, 8], fov: 55, roll: 7 };
  const back = lookAtFromCamera(cameraFromLookAt(v, 1299, 1732), 1732, Math.hypot(30, 20, 18));
  assert.deepEqual(back.pos, v.pos);
  back.target.forEach((x, i) => assert.ok(Math.abs(x - v.target[i]) < 0.15));
  assert.ok(Math.abs(back.fov - v.fov) < 0.01 && Math.abs(back.roll - v.roll) < 0.01);
});

test('fitView recovers camera and joint angles from projected keypoints', () => {
  const W = 1299, H = 1732, truth = { s1: 3, s2: -25, s3: 115, s4: 85, s5: 0 }, gap = 0;
  const cam = cameraFromLookAt({ pos: [-32, 18, 24], target: [0, 2, 9], fov: 52, roll: -4 }, W, H);
  const names = ['base_top_fl', 'base_top_bl', 'base_bot_fl', 's2_pin', 's3_pin', 'ua_mid_nx', 'tube_lo', 'tube_hi', 's4_pin', 'flange', 'pad_nx', 'm0', 'm2'];
  const pts = projectKeypoints(cam, truth, gap, -2.5, n => names.includes(n));
  const fit = fitView(pts, { width: W, height: H, init: { camera: { pos: [-25, 25, 30], target: [0, 0, 8], fov: 60 }, angles: { s1: 0, s2: -40, s3: 128, s4: 93, s5: 0 }, gap },
                             free: ['s1', 's2', 's3', 's4'], prior: 1e6 });
  assert.ok(fit.rms < 0.5, `rms ${fit.rms}`);
  for (const j of ['s1', 's2', 's3', 's4']) assert.ok(Math.abs(fit.angles[j] - truth[j]) < 0.3, `${j} ${fit.angles[j]}`);
  assert.ok(Math.abs(fit.camera.fov - 52) < 0.2);
});
