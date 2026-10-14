// node --test test/*.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { SERVOS, CUBE } from '../js/config.js';
import { World } from '../js/world.js';
import { cubeTilt } from '../js/physics.js';
import { ikTopDown, countsFromAngles, fk, anglesFromCounts, toolYawAndTilt } from '../js/kinematics.js';

const HOME = SERVOS.map(s => s.home);
const run = (w, seconds, counts = HOME, dt = 1 / 60) => { for (let t = 0; t < seconds; t += dt) w.step(dt, counts); };

test('a stack of three cubes stands still and goes to sleep', () => {
  const w = new World();
  w.addCube('a', 'orange', 5, 14, 0); w.addCube('b', 'blue', 5, 14, 1, 3); w.addCube('c', 'yellow', 5, 14, 2, -4);
  const start = w.cubes.map(c => c.p.slice());
  run(w, 3);
  w.cubes.forEach((c, i) => {
    assert.ok(Math.hypot(c.p[0] - start[i][0], c.p[1] - start[i][1]) < 0.1, `${c.id} drifted`);
    assert.ok(Math.abs(c.p[2] - start[i][2]) < 0.05, `${c.id} sank or rose: ${c.p[2]} vs ${start[i][2]}`);
    assert.equal(c.state, 'asleep', `${c.id} still moving`);
  });
});

test('a tumbling cube dropped from 8 cm lands flat on the board', () => {
  const w = new World();
  const c = w.addCube('a', 'green', 0, 15, 3);
  c.w = [6, 4, 1];
  run(w, 4);
  assert.ok(cubeTilt(c) < 3, `tilt ${cubeTilt(c)}`);
  assert.ok(Math.abs(c.p[2] - CUBE / 2) < 0.05, `height ${c.p[2]}`);
  assert.ok(Math.abs(c.p[0]) < 10 && Math.abs(c.p[1] - 15) < 10, 'stayed near where it fell');
});

test('a cube set mostly over the edge of another topples off', () => {
  const w = new World();
  w.addCube('base', 'orange', 0, 15, 0);
  w.addCube('top', 'blue', 0.75 * CUBE, 15, 1);
  run(w, 3);
  const top = w.cube('top');
  assert.ok(top.p[2] < CUBE, `top cube should have fallen, z = ${top.p[2]}`);
});

test('the arm sweeping through a cube pushes it away and reports the contact', () => {
  const w = new World();
  const c = w.addCube('a', 'orange', 0, 16, 0);
  run(w, 1);
  // gripper hanging low beside the cube, swept along X so that one finger passes through it
  let y = 16;
  const pose = x => countsFromAngles(ikTopDown(x, y, 1.0, { s5Near: 90 }), 490);   // open pads sit 1 cm higher than a held cube
  const jaw = toolYawAndTilt(fk(anglesFromCounts(pose(0))).hand).yaw;
  if (Math.abs(Math.cos(jaw * Math.PI / 180)) < 0.7) y = 16 - 2.4;   // jaws along Y: put one finger on the cube's line
  const a = pose(-5), b = pose(5);
  w.step(1 / 60, a);
  const before = c.p.slice();
  let pushed = false;
  for (let i = 1; i <= 240; i++) {
    w.step(1 / 60, a.map((v, k) => v + (b[k] - v) * i / 240));
    if (w.contacts.some(k => k.b === 'a cube')) pushed = true;
  }
  run(w, 2, b);
  assert.ok(pushed, 'contact reported');
  assert.ok(Math.hypot(c.p[0] - before[0], c.p[1] - before[1]) > 0.5, `cube moved ${Math.hypot(c.p[0] - before[0], c.p[1] - before[1])}`);
});

test('cubes landing next to each other all fall asleep (no wake ping-pong)', () => {
  const w = new World();
  let seed = 7; const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let i = 0; i < 8; i++) {
    const c = w.addCube(`c${i}`, 'red', -4 + 8 * rnd(), 14 + 6 * rnd(), 2 + i, 360 * rnd());
    c.w = [(rnd() - 0.5) * 10, (rnd() - 0.5) * 10, (rnd() - 0.5) * 10];
  }
  run(w, 10);
  const awake = w.cubes.filter(c => c.state === 'awake').map(c => c.id);
  assert.deepEqual(awake, [], `still awake after 10 s: ${awake}`);
  // resting on the board (centre 1,325) or, if one tumbled over the edge, on the desk 1 cm lower
  assert.ok(w.cubes.every(c => c.p.every(Number.isFinite) && (c.p[2] > 1.2 || (c.p[2] > 0.2 && !w.onBoard(c.p[0], c.p[1])))), 'every cube rests on the board or the desk');
});
