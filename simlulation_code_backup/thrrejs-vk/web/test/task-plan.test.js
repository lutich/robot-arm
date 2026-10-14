import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { SERVOS } from '../js/config.js';
import { World } from '../js/world.js';
import { configureCanonicalSpec, applyCanonicalServoMetadata } from '../js/advanced.js';
import { buildTaskPlan, parseTask } from '../src/agent/task-plan.js';
import { validatePlan } from '../src/operational/plan-channel.js';
import { validateArmSpec } from '../src/config/spec.js';
import { loadSpec } from './helpers.js';

const webFile = (name) => fs.readFileSync(new URL(`../${name}`, import.meta.url), 'utf8');
const spec = validateArmSpec(loadSpec());
configureCanonicalSpec(spec);
applyCanonicalServoMetadata(SERVOS, spec);
const start = spec.joints.map((joint) => joint.home);

function pageScene() {
  const world = new World();
  // The page's default scene (app.js DEFAULT_SCENE).
  for (const [colour, x, y, level, yaw] of [['orange', 10.8, 14.0, 0, 3], ['blue', -8, 12, 0, 0], ['red', 3, 10, 0, 0], ['green', -4, 17, 0, 10]]) {
    world.addCube(colour, colour, x, y, level, yaw);
  }
  return world;
}

test('task text: grab holds a cube; other commands use the page grammar', () => {
  const world = pageScene();
  assert.deepEqual(parseTask('grab green', world), { pick: 'green', place: null });
  assert.deepEqual(parseTask('blue on orange', world), { pick: 'blue', place: { on: 'orange' } });
  assert.deepEqual(parseTask('red to 5 15', world), { pick: 'red', place: { x: 5, y: 15 } });
  assert.throws(() => parseTask('grab yellow', world), /no yellow cube/);
});

test('a planned task is single-servo steps that pass the headless grasp pre-check', () => {
  const result = buildTaskPlan({ spec, world: pageScene(), start, text: 'blue on orange' });
  assert.equal(result.success, true);
  const { plan } = result;
  assert.equal(plan.precheck.ok, true);
  assert.ok(plan.precheck.offsetCm < 0.9);
  assert.deepEqual(plan.start, start);
  assert.equal(plan.expect.type, 'pick-place');
  assert.ok(plan.estimatedSeconds > 0);
  assert.doesNotThrow(() => validatePlan(JSON.parse(JSON.stringify(plan))));
  let previous = start;
  for (const step of plan.steps) {
    assert.equal(step.counts.filter((count, channel) => count !== previous[channel]).length, 1);
    previous = step.counts;
  }
});

test('unknown cubes and unreachable targets are reported, not planned', () => {
  assert.match(buildTaskPlan({ spec, world: pageScene(), start, text: 'yellow on red' }).reason, /no yellow cube/);
  assert.match(buildTaskPlan({ spec, world: pageScene(), start, text: 'red to 60 60' }).reason, /off the board|unreachable/);
});

test('every element the plan panel uses exists in the page', () => {
  const index = webFile('index.html').replace(/<!--[\s\S]*?-->/g, '');
  const ids = [...webFile('js/plan-runner.js').matchAll(/\$\('([a-z0-9-]+)'\)/g)].map((match) => match[1]);
  assert.ok(ids.length > 8);
  for (const id of new Set([...ids, 'ghost-show'])) assert.match(index, new RegExp(`id="${id}"`), `#${id} is missing from index.html`);
});
