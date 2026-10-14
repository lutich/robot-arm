import { test } from 'node:test';
import assert from 'node:assert/strict';
import { ArmBackend, Blocked } from '../js/armbackend.js';

const counts = s => [0, 1, 2, 3, 4, 5].map(i => s.commands[i]);
const run = (b, seconds) => { for (let t = 0; t < seconds; t += 0.02) b.tick(0.02); };

// A second backend in sim mode plays the real controller: same API, nothing physical involved.
function withFakeReal(opts = {}) {
  const realArm = new ArmBackend();
  const real = { get: p => realArm.get(p), post: (p, b) => realArm.post(p, b) };
  return { realArm, b: new ArmBackend({ real, realName: 'fake', allowMotion: true, ...opts }) };
}

test('sim backend answers the low-level API and stops on a collision', async () => {
  const b = new ArmBackend();
  assert.equal((await b.get('state')).backend, 'sim');
  await b.post('move', { channel: 0, target: 400 });
  run(b, 10);
  assert.equal(counts(await b.get('state'))[0], 400);
  await b.post('move', { channel: 1, target: 185 });    // S2 all the way down: the fingers reach the board
  run(b, 20);
  const s = await b.get('state');
  assert.equal(s.outputs_off, true);
  assert.match(s.error, /collision/);
});

test('real backend: the same calls, colliding ones blocked, stop always passes', async () => {
  const { realArm, b } = withFakeReal();
  await b.setTarget('real');
  assert.equal((await b.get('state')).backend, 'real');
  await assert.rejects(b.post('move', { channel: 1, target: 185 }), Blocked);
  assert.equal(realArm.ctl.busy, false);                 // nothing reached the "real" arm
  await b.post('move', { channel: 0, target: 400 });
  assert.equal(realArm.ctl.busy, true);                  // a clear move goes through
  await b.post('stop', {});
  assert.equal(realArm.ctl.outputsOff, true);
  await b.post('power_on', {});                          // PARK -> HOME: the gripper may rest on the base at PARK
  run(realArm, 40);
  await b.post('go_pose', { name: 'PARK' });
  run(realArm, 40);
  assert.deepEqual(counts(await realArm.get('state')), realArm.ctl.poses.PARK);
});

test('real backend without --allow-motion only lets stop through', async () => {
  const { b } = withFakeReal({ allowMotion: false });
  await b.setTarget('real');
  await assert.rejects(b.post('move', { channel: 0, target: 400 }), /allow-motion/);
  await b.post('stop', {});
});

test('switching back to sim continues from the real pose', async () => {
  const { realArm, b } = withFakeReal();
  await realArm.post('move', { channel: 0, target: 380 }); run(realArm, 10);
  await b.setTarget('real'); await b.get('state');
  await b.setTarget('sim');
  assert.equal(counts(await b.get('state'))[0], 380);
});
