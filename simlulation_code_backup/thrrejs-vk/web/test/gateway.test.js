import test from 'node:test';
import assert from 'node:assert/strict';
import { connectionInfo, proxyDecision } from '../src/operational/gateway.js';
import { createGateway, referencePhotoFile, updateTaskStatus } from '../server.mjs';

test('gateway is read-only plus emergency stop by default', () => {
  assert.deepEqual(proxyDecision('GET', '/api/state'), { allowed: true, kind: 'read' });
  assert.deepEqual(proxyDecision('GET', '/api/camera/status'), { allowed: true, kind: 'read' });
  assert.deepEqual(proxyDecision('GET', '/api/camera/snapshot.jpg'), { allowed: true, kind: 'read' });
  assert.equal(proxyDecision('GET', '/api/camera/stream.mjpg').allowed, false);
  assert.deepEqual(proxyDecision('POST', '/api/stop'), { allowed: true, kind: 'stop' });
  assert.equal(proxyDecision('POST', '/api/move').allowed, false);
  assert.equal(proxyDecision('POST', '/api/power_on').status, 403);
});

test('explicit motion mode exposes only required operational endpoints', () => {
  for (const path of ['/api/power_on', '/api/power_off', '/api/move', '/api/go_pose']) {
    assert.deepEqual(proxyDecision('POST', path, { allowMotion: true }), { allowed: true, kind: 'motion' });
    assert.equal(proxyDecision('POST', path, { allowMotion: false }).status, 403);
  }
  assert.equal(proxyDecision('POST', '/api/limits', { allowMotion: true }).allowed, false);
  assert.equal(proxyDecision('GET', '/api/openapi.json', { allowMotion: true }).allowed, false);
});

test('connection metadata never implies motion without both controller and flag', () => {
  assert.deepEqual(connectionInfo(null, true), {
    backend: 'none', controllerConfigured: false, motionAllowed: false, policy: 'simulation-only',
  });
  assert.equal(connectionInfo(new URL('http://robot.local:8765'), false).policy, 'read-only-plus-stop');
  assert.equal(connectionInfo(new URL('http://robot.local:8765'), true).motionAllowed, true);
});

test('gateway construction has no listening or controller side effect', () => {
  const gateway = createGateway({ controller: new URL('http://robot.local:8765'), allowMotion: false });
  assert.equal(gateway.listening, false);
  gateway.close();
});

test('reference photo route accepts only a flat image filename', () => {
  assert.match(referencePhotoFile('/reference-photos/park-front.jpg'), /docs\/images\/photos\/park-front\.jpg$/);
  assert.equal(referencePhotoFile('/reference-photos/../arm-spec.json'), null);
  assert.equal(referencePhotoFile('/reference-photos/note.txt'), null);
});

test('task status validates fields and advances its revision', () => {
  const next = updateTaskStatus({ revision: 4 }, {
    task: 'Move to HOME', current: 'counts [1,2,3,4,5,6]', destination: 'HOME', status: 'moving', message: 'Executing step 2 of 5.',
  });
  assert.equal(next.revision, 5);
  assert.equal(next.status, 'moving');
  assert.match(next.updatedAt, /^\d{4}-\d{2}-\d{2}T/);
  assert.throws(() => updateTaskStatus(next, { ...next, status: 'teleporting' }), /Unknown task status/);
});

test('gateway shares task status between browser clients', async (t) => {
  const gateway = createGateway();
  await new Promise((resolve) => gateway.listen(0, '127.0.0.1', resolve));
  t.after(() => gateway.close());
  const { port } = gateway.address();
  const base = `http://127.0.0.1:${port}`;
  const initial = await (await fetch(`${base}/simulator/task-status`)).json();
  assert.equal(initial.status, 'idle');
  const response = await fetch(`${base}/simulator/task-status`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', Origin: base },
    body: JSON.stringify({ task: 'Pick orange', current: 'HOME', destination: 'orange cube', status: 'planning', message: 'Checking servo bounds.' }),
  });
  assert.equal(response.status, 200);
  const shared = await (await fetch(`${base}/simulator/task-status`)).json();
  assert.equal(shared.task, 'Pick orange');
  assert.equal(shared.revision, 1);
});

test('gateway rejects cross-origin task status updates', async (t) => {
  const gateway = createGateway();
  await new Promise((resolve) => gateway.listen(0, '127.0.0.1', resolve));
  t.after(() => gateway.close());
  const { port } = gateway.address();
  const response = await fetch(`http://127.0.0.1:${port}/simulator/task-status`, {
    method: 'POST', headers: { 'Content-Type': 'application/json', Origin: 'http://example.invalid' },
    body: JSON.stringify({ task: 'Bad', current: 'Here', destination: 'There', status: 'moving', message: 'No.' }),
  });
  assert.equal(response.status, 403);
});
