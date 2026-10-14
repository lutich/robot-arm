import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import { createPlanChannel, validatePlan, validateProgress } from '../src/operational/plan-channel.js';

const start = [320, 427, 403, 220, 480, 650];
const plan = (overrides = {}) => ({
  id: 'plan-test', task: 'grab green', createdAt: new Date().toISOString(), start,
  steps: [{ label: 'open', counts: [320, 427, 403, 220, 480, 490] }, { label: 'turn', counts: [330, 427, 403, 220, 480, 490] }],
  scene: { cubes: [{ id: 'green', colour: 'green', p: [-4, 17, 1.325] }], holding: null, gap: 0 },
  expect: { type: 'pick', cube: 'green', target: [-4, 17, 1.325] },
  ...overrides,
});

test('plans must be single-servo steps from the stated start', () => {
  assert.equal(validatePlan(plan()).steps.length, 2);
  assert.throws(() => validatePlan(plan({ steps: [{ label: 'two', counts: [330, 437, 403, 220, 480, 650] }] })), /changes 2 servos/);
  assert.throws(() => validatePlan(plan({ steps: [{ label: 'none', counts: start }] })), /changes 0 servos/);
  assert.throws(() => validatePlan(plan({ start: [320, 427, 403, 220, 480] })), /six integer/);
  assert.throws(() => validatePlan(plan({ expect: { type: 'throw', cube: 'green', target: [0, 0, 0] } })), /expect.type/);
});

test('progress is accepted only for the current plan revision', () => {
  assert.equal(validateProgress({ revision: 3, phase: 'simulating', step: 4, total: 10 }, 3).step, 4);
  assert.throws(() => validateProgress({ revision: 2, phase: 'simulating' }, 3), /revision 2/);
  assert.throws(() => validateProgress({ revision: 3, phase: 'moving-robot' }, 3), /Unknown plan phase/);
});

async function withServer(fn) {
  const channel = createPlanChannel();
  const server = http.createServer((request, response) => {
    const pathname = new URL(request.url, 'http://test').pathname;
    if (channel.handles(pathname)) return void channel.handle(request, response, pathname);
    response.writeHead(404).end();
  });
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  const base = `http://127.0.0.1:${server.address().port}`;
  try { await fn(base); } finally { server.close(); }
}

const post = (url, body, headers = {}) => fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json', ...headers }, body: JSON.stringify(body) });

test('the channel publishes plans with revisions and shares progress', async () => {
  await withServer(async (base) => {
    assert.deepEqual(await (await fetch(`${base}/simulator/plan`)).json(), { revision: 0, plan: null });
    const published = await (await post(`${base}/simulator/plan`, plan())).json();
    assert.deepEqual(published, { revision: 1, steps: 2 });
    const current = await (await fetch(`${base}/simulator/plan`)).json();
    assert.equal(current.plan.task, 'grab green');
    assert.equal((await (await fetch(`${base}/simulator/plan/progress`)).json()).phase, 'sent');
    assert.equal((await post(`${base}/simulator/plan/progress`, { revision: 1, phase: 'simulated', step: 2, total: 2, ok: true })).status, 200);
    assert.equal((await (await fetch(`${base}/simulator/plan/progress`)).json()).ok, true);
    assert.equal((await post(`${base}/simulator/plan/progress`, { revision: 0, phase: 'simulated' })).status, 400);
  });
});

test('the channel rejects cross-origin writes and invalid plans', async () => {
  await withServer(async (base) => {
    assert.equal((await post(`${base}/simulator/plan`, plan(), { Origin: 'http://evil.example' })).status, 403);
    assert.equal((await post(`${base}/simulator/plan`, plan({ steps: [] }))).status, 400);
    assert.equal((await fetch(`${base}/simulator/scene`)).status, 404);
    const scene = { commands: start, snapshot: plan().scene, calibration: null, mode: 'simulation', source: 'simulation' };
    assert.equal((await (await post(`${base}/simulator/scene`, scene)).json()).revision, 1);
    assert.deepEqual((await (await fetch(`${base}/simulator/scene`)).json()).commands, start);
  });
});
