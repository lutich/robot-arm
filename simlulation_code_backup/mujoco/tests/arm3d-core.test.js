// node --test tests/arm3d-core.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { CONFIG } from '../web/arm3d-config.js';
import { angleOf, countOf, forward, gripperLength, gripperState, linkAngleForGap, World } from '../web/arm3d-core.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);

test('counts round-trip through angles for every channel inside the bounds', () => {
  for (let channel = 0; channel < 6; channel++) {
    const [lo, hi] = CONFIG.bounds[String(channel)];
    for (let count = lo; count <= hi; count++) assert.equal(countOf(CONFIG, channel, angleOf(CONFIG, channel, count)), count);
  }
});

test('home counts give the configured reference angles', () => {
  for (let channel = 0; channel < 5; channel++) near(angleOf(CONFIG, channel, CONFIG.home[channel]), CONFIG.channels[String(channel)].home_angle, 1e-9, `S${channel + 1}`);
  near(angleOf(CONFIG, 5, CONFIG.gripper.open_count), CONFIG.gripper.open_angle, 1e-9, 'gripper');
});

test('gripper geometry matches the measured gaps and lengths', () => {
  near(gripperLength(CONFIG, CONFIG.gripper.open_angle), 0.100, 0.001, 'open length');
  near(gripperLength(CONFIG, CONFIG.gripper.closed_angle), 0.115, 0.001, 'closed length');
  near(linkAngleForGap(CONFIG, 0.0265), 34.7, 0.3, 'cube link angle');
  const fk = forward(CONFIG, [0, 0, 0, 0, 0, CONFIG.gripper.open_angle]);
  near(fk.gap, 0.045, 0.001, 'open gap');
});

test('straight-up pose stacks the joints at the measured heights', () => {
  const fk = forward(CONFIG, [0, 0, 0, 0, 0, CONFIG.gripper.open_angle]);
  const L = CONFIG.links;
  near(fk.s2[2], L.shoulder_height, 1e-9, 's2');
  near(fk.s4[2], L.shoulder_height + L.upper_arm + L.forearm, 1e-9, 's4');
  near(fk.flange[2], 0.394, 0.001, 'flange');
  near(fk.flange[0], CONFIG.base.x, 1e-9, 'flange x');
});

test('home pose points the gripper down over the table and pads separate sideways', () => {
  const fk = forward(CONFIG, CONFIG.home.map((c, ch) => angleOf(CONFIG, ch, c)));
  assert.ok(fk.direction[2] < -0.95, 'pointing down (within the assumed 15° tilt)');
  assert.ok(fk.grasp[0] > 0.15 && fk.grasp[2] > 0.02 && fk.grasp[2] < 0.04, `grasp ${fk.grasp}`);
  const sideways = (CONFIG.gripper.pads_axis || 'x') === 'y';
  near(Math.abs(fk.padAxis[sideways ? 1 : 0]), 1, 0.01, 'pad axis at zero roll');
});

test('blocks start on their slots and rest on the table', () => {
  const world = new World(CONFIG);
  const far = { left: [0.3, 0.1, 0.2], right: [0.3, 0.145, 0.2], gap: 0.045, commandedGap: 0.045, padHalf: [0.00135, 0.008, 0.006] };
  for (let i = 0; i < 100; i++) world.step(0.02, far);
  for (const b of world.blocks) {
    const slot = CONFIG.slots[CONFIG.layout[b.name]];
    near(b.x, slot.x, 1e-9, b.name); near(b.y, slot.y, 1e-9, b.name); near(b.z, b.hz, 1e-9, `${b.name} z`);
  }
});

test('a dropped block lands on a block below it, and a pushed block shoves its neighbour', () => {
  const world = new World(CONFIG);
  const red = world.block('red'), green = world.block('green');
  green.x = red.x; green.y = red.y; green.z = 0.1;
  const far = { left: [0.3, 0.1, 0.2], right: [0.3, 0.145, 0.2], gap: 0.045, commandedGap: 0.045, padHalf: [0.00135, 0.008, 0.006] };
  for (let i = 0; i < 200; i++) world.step(0.01, far);
  near(green.z, red.hz * 2 + green.hz, 1e-6, 'stacked');
  const blue = world.block('blue'), yellow = world.block('yellow');
  blue.x = yellow.x; blue.y = yellow.y + 0.023;         // overlapping yellow by 3,5 mm
  world.shove(blue);
  near(Math.abs(yellow.y - blue.y), 0.0265, 1e-9, 'shoved apart');
});

test('closing the pads on a cube holds it, lifting carries it, opening releases it', () => {
  const world = new World(CONFIG);
  const red = world.block('red');
  const pads = (z, gap, commanded) => ({ left: [red.x - gap / 2, red.y, z], right: [red.x + gap / 2, red.y, z], gap, commandedGap: commanded, padHalf: [0.00135, 0.008, 0.006] });
  world.step(0.02, pads(red.z, 0.045, 0.045));
  assert.equal(world.held, null);
  const events = world.step(0.02, pads(red.z, 0.0265, 0.0225));
  assert.deepEqual(events, ['holding red']);
  world.step(0.02, { ...pads(red.z + 0.05, 0.0265, 0.0225) });
  near(red.z, red.hz + 0.05, 1e-9, 'carried up');
  const moved = { left: [0.11 - 0.01325, -0.11, 0.05], right: [0.11 + 0.01325, -0.11, 0.05], gap: 0.0265, commandedGap: 0.0225, padHalf: [0.00135, 0.008, 0.006] };
  world.step(0.02, moved);
  near(red.x, 0.11, 1e-9, 'carried x'); near(red.y, -0.11, 1e-9, 'carried y');
  const opened = { ...moved, left: [0.11 - 0.0225, -0.11, 0.05], right: [0.11 + 0.0225, -0.11, 0.05], gap: 0.045, commandedGap: 0.045 };
  const release = world.step(0.02, opened);
  assert.deepEqual(release, ['released red']);
  for (let i = 0; i < 100; i++) world.step(0.02, { ...opened, left: [0.3, 0, 0.2], right: [0.3, 0.045, 0.2] });
  near(red.z, red.hz, 1e-9, 'landed'); near(red.x, 0.11, 1e-9, 'stays'); near(red.y, -0.11, 1e-9, 'stays');
});

test('gripperState places the pads around the grasp point', () => {
  const fk = forward(CONFIG, CONFIG.home.map((c, ch) => angleOf(CONFIG, ch, c)));
  const g = gripperState(CONFIG, fk, CONFIG.gripper.open_angle);
  near(Math.hypot(g.right[0] - g.left[0], g.right[1] - g.left[1], g.right[2] - g.left[2]), 0.045, 1e-4, 'pad distance');
  near(g.commandedGap, 0.045, 1e-4, 'commanded');
});
