import { test } from 'node:test';
import assert from 'node:assert/strict';
import { SimController } from '../js/controller.js';
import { SimArm, runCalls } from '../js/armapi.js';
import { controllerApi } from '../js/controller-api.js';

// the controller's clock runs 50x real time here
const clock = ctl => { let last = performance.now(); return setInterval(() => { const now = performance.now(); ctl.update((now - last) / 1000 * 50); last = now; }, 5); };

test('the simulated arm answers the controller API like the mock does', async () => {
  const ctl = new SimController(), api = controllerApi(ctl, 'simulation');
  assert.equal(api.get('state').mode, 'simulation');
  assert.throws(() => api.post('nope', {}), /Unknown path/);
  assert.deepEqual(api.post('move', { channel: 0, target: 330 }), { ok: true, result: true });
  assert.throws(() => api.post('move', { channel: 1, target: 999 }));   // outside the guards: rejected, not clamped
});

test('runCalls executes a call list on the simulated arm', async () => {
  const ctl = new SimController(), arm = new SimArm(ctl), t = clock(ctl);
  try {
    const home = ctl.poses.HOME;
    await runCalls(arm, [
      { op: 'move', ch: 0, target: home[0] + 40, label: 'S1 +40' },
      { op: 'move', ch: 2, target: home[2] - 30, label: 'S3 -30' },
      { op: 'go_pose', name: 'HOME', label: 'go HOME' },
      { op: 'move', ch: 5, target: 600, label: 'grip' },
    ]);
    const s = await arm.read();
    assert.equal(s.busy, false);
    assert.deepEqual(SimArm.counts(s), [...home.slice(0, 5), 600]);
    ctl.stop('test');
    await assert.rejects(runCalls(arm, [{ op: 'move', ch: 0, target: home[0] + 10, label: 'x' }]));   // outputs off: nothing runs
    await runCalls(arm, [{ op: 'power_on', label: 'power on' }]);
    assert.deepEqual(SimArm.counts(await arm.read()), home);
  } finally { clearInterval(t); }
});
