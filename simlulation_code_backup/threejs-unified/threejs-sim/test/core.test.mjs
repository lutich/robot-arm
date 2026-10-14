// node --test web/test/
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { SERVOS, BASE, CUBE, GRIPPER } from '../js/config.js';
import { ikTopDown, fk, toolYawAndTilt, anglesFromCounts, countsFromAngles, gapForTheta, gripperTheta,
         padDepthForTheta, countsForGap, maxReach } from '../js/kinematics.js';
import { segSegDist, boxBoxSep, segBoxDist, armShapes, findContacts } from '../js/collision.js';
import { rotZ, point } from '../js/kinematics.js';
import { SimController, ControllerError } from '../js/controller.js';
import { World } from '../js/world.js';
import { planTransfer, verify, parseCommand, toMoves } from '../js/planner.js';
import { BOARD } from '../js/config.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const HOME = SERVOS.map(s => s.home);
const PARK = SERVOS.map(s => s.park);

test('IK matches the table in docs/pick-and-place-plan.md section 8', () => {
  const rows = [[5, 15, 0, 18.4, 17.9, 74.7, 87.4], [5, 15, 5, 18.4, 23.3, 44.4, 112.3], [-8, 12, 0, -33.7, 10.2, 84.0, 85.9],
                [0, 10, 0, 0, -14.2, 107.6, 86.6], [0, 20, 0, 0, 46.3, 33.8, 99.9]];
  for (const [x, y, h, s1, s2, s3, s4] of rows) {
    const a = ikTopDown(x, y, CUBE / 2 + h);
    near(a.s1, s1, 0.06, 's1'); near(a.s2, s2, 0.06, 's2'); near(a.s3, s3, 0.06, 's3'); near(a.s4, s4, 0.06, 's4');
  }
  assert.throws(() => ikTopDown(0, 25, CUBE / 2));
  near(maxReach(), 21.1, 0.05);
});

test('FK puts the held cube centre on the IK target, tool down, jaws square', () => {
  for (const [x, y, yaw] of [[5, 15, 0], [-8, 12, 10], [0, 12, 30], [10, 10, -20], [-12, 6, 0]]) {
    const a = ikTopDown(x, y, 4, { cubeYaw: yaw, s5Near: 90 });
    const f = fk(a);
    near(f.grasp[0], x, 1e-9); near(f.grasp[1], y, 1e-9); near(f.grasp[2], 4, 1e-9);
    const { yaw: jaw, tilt } = toolYawAndTilt(f.hand);
    near(tilt, 0, 1e-6, 'tilt');
    const e = ((jaw - yaw) % 90 + 135) % 90 - 45;
    near(e, 0, 1e-6, 'jaw yaw');
  }
});

test('gripper linkage reproduces the measured values', () => {
  near(gapForTheta(56.4), 4.5, 0.01); near(padDepthForTheta(56.4), 10.0, 0.01);
  near(gapForTheta(11.0), 0, 0.01); near(padDepthForTheta(11.0), 11.5, 0.01);
  near(padDepthForTheta(34.7), 11.0, 0.07);
  near(gapForTheta(gripperTheta(490)), 4.5, 0.01);
  const contact = countsForGap(CUBE);
  assert.ok(contact > 490 && contact < 590, `contact at ${contact}`);
});

test('default calibration preserves the HOME anchor and round-trips counts', () => {
  // counts -> angles -> counts round trip
  const a = anglesFromCounts(HOME);
  assert.deepEqual(a, { s1: 0, s2: 6, s3: 62, s4: 84, s5: 90 });
  assert.deepEqual(countsFromAngles(a, HOME[5]), HOME);
});

test('collision primitives', () => {
  near(segSegDist([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]), 1, 1e-12);
  near(segSegDist([0, 0, 0], [2, 0, 0], [1, -1, 1], [1, 1, 1]), 1, 1e-12);
  const A = { kind: 'box', c: [0, 0, 0], R: rotZ(0), h: [1, 1, 1] };
  near(boxBoxSep(A, { kind: 'box', c: [3, 0, 0], R: rotZ(0), h: [1, 1, 1] }), 1, 1e-9);
  near(boxBoxSep(A, { kind: 'box', c: [1.5, 0, 0], R: rotZ(0.3), h: [1, 1, 1] }), -(2 + 0 - 1.5) + (1 - Math.cos(0.3) - Math.sin(0.3)) * -1, 0.6);
  assert.ok(boxBoxSep(A, { kind: 'box', c: [1.5, 0, 0], R: rotZ(0.3), h: [1, 1, 1] }) < 0);
  near(segBoxDist([2, -5, 0], [2, 5, 0], A), 1, 1e-6);
  assert.ok(segBoxDist([-5, 0, 0], [5, 0, 0], A) < 0);
});

test('HOME is free of contacts; driving the wrist into the board is caught', () => {
  const w = new World();
  w.step(0, HOME);
  assert.deepEqual(w.contacts, []);
  const f = fk(ikTopDown(0, 15, -1.5));   // cube centre 1,5 cm below the board
  const c = findContacts(armShapes(f, 4.5), [], BOARD);
  assert.ok(c.some(k => k.b === 'board'));
});

test('PARK rests the closed claw on the raised upper base plate', () => {
  const w = new World();
  w.step(0, PARK);
  assert.deepEqual(w.contacts, []);
  const a = anglesFromCounts(PARK), f = fk(a);
  const gap = gapForTheta(gripperTheta(PARK[5]));
  const padDepth = padDepthForTheta(gripperTheta(PARK[5]));
  // The metal finger extends 0,1 cm below the rubber pad in the detailed claw model.
  const tip = point(f.hand, [-gap / 2, 0, padDepth + 0.7]);
  near(tip[2], BASE.h, 0.01, 'claw tip must meet the raised plate top');
  assert.ok(Math.abs(tip[0]) <= BASE.w / 2, 'claw tip must lie across the upper plate');
  const frontY = BASE.offsetY + BASE.beamY;
  assert.ok(Math.abs(tip[1] - frontY) <= BASE.beamD / 2, 'claw tip must lie on the front upper plate');
});

test('HOME to PARK and PARK to HOME transitions have no blocking contacts', () => {
  const check = (from, name, target) => {
    const ctl = new SimController(), w = new World();
    ctl.setCommands(from);
    w.step(0, ctl.position);
    ctl.go_pose(name);
    for (let i = 0; i < 5000 && ctl.busy; i++) {
      ctl.update(1 / 120);
      w.step(1 / 120, ctl.position);
      assert.deepEqual(w.contacts, [], `${name} blocked at ${ctl.position.map(v => v.toFixed(1)).join(', ')}`);
    }
    assert.equal(ctl.busy, false, `${name} transition did not finish`);
    assert.deepEqual(ctl.commands, target);
  };
  check(HOME, 'PARK', PARK);
  check(PARK, 'HOME', HOME);
});

test('controller emulation: rejection, smoothstep timing, demo', () => {
  const ctl = new SimController();
  assert.throws(() => ctl.move(1, 100), ControllerError);
  ctl.move(0, 420);                                     // 100 counts at 20 c/s -> 7,5 s
  assert.throws(() => ctl.move(1, 400), /already running/);
  ctl.update(3.75); near(ctl.position[0], 370, 1e-9);  // halfway in time = halfway in counts
  ctl.update(3.75); assert.equal(ctl.commands[0], 420); assert.equal(ctl.busy, false);
  ctl.demo_add('a'); ctl.move(0, 320); ctl.update(10); ctl.demo_add('b');
  ctl.demo_run(); for (let i = 0; i < 300; i++) ctl.update(0.1);
  assert.equal(ctl.demo.status, 'complete'); assert.equal(ctl.commands[0], 320);
  ctl.stop('test'); assert.throws(() => ctl.move(0, 330), /Outputs are off/);
});

function scene() {
  const w = new World();
  w.addCube('orange', 'orange', 5, 15);
  w.addCube('blue', 'blue', -8, 12);
  w.addCube('green', 'green', 10, 8, 0, 15);
  w.step(0, HOME);
  return w;
}

test('pick and place on the board, demo playback', () => {
  const w = scene();
  const plan = planTransfer(w, { pick: 'orange', place: { x: 0, y: 14 } }, HOME, { home: HOME });
  const r = verify(w, plan, HOME, { speed: 100 });
  assert.equal(r.contacts.length, 0, JSON.stringify(r.contacts));
  assert.ok(r.ok, `offset ${r.offset}`);
  assert.ok(r.events.some(e => e.kind === 'grasp'));
});

test('stack blue on orange, then green on blue', () => {
  const w = scene();
  for (const cmd of ['blue on orange', 'grün auf blau']) {
    const plan = planTransfer(w, parseCommand(cmd, w), HOME, { home: HOME });
    const r = verify(w, plan, HOME, { speed: 100 });
    assert.ok(r.ok, `${cmd}: ${JSON.stringify(r.contacts)} offset ${r.offset}`);
    w.restore(r.final);
  }
  assert.equal(w.levelOf(w.cube('green')), 2);
  assert.equal(w.levelOf(w.cube('blue')), 1);
});

test('the same plan as single-joint moves (what the HTTP API can send)', () => {
  const w = scene();
  const plan = planTransfer(w, { pick: 'blue', place: { on: 'orange' } }, HOME, { home: HOME, step: 0.25 });
  const { moves, notes } = toMoves(w, plan, HOME);
  assert.ok(moves.every(m => Number.isInteger(m.target)));
  const r = verify(w, plan, HOME, { mode: 'moves', moves });
  assert.ok(r.ok, `${notes.join('; ')} ${JSON.stringify(r.contacts)}`);
});

test('a cube under another cannot be picked; out of reach is reported', () => {
  const w = scene();
  w.addCube('yellow', 'yellow', 5, 15, 1);
  w.step(0.5, HOME);
  assert.throws(() => planTransfer(w, { pick: 'orange', place: { x: 0, y: 12 } }, HOME), /yellow on top/);
  assert.throws(() => planTransfer(w, { pick: 'blue', place: { x: 0, y: 30 } }, HOME), /unreachable/);
});
