import test from 'node:test';
import assert from 'node:assert/strict';
import { controllerCompatiblePath, optimizeJointPath } from '../src/core/planning.js';
import { referencePose } from '../src/core/kinematics.js';
import { loadSpec } from './helpers.js';

test('calibration planner uses a direct unchecked waypoint and no collision specification', () => {
  const spec = loadSpec();
  const home = referencePose(spec, 'home');
  const park = referencePose(spec, 'park');
  assert.equal(spec.collision, undefined);
  const plan = optimizeJointPath(spec, home, park);
  assert.equal(plan.success, true);
  assert.deepEqual(plan.path, [home, park]);
  assert.deepEqual(plan.verification, { valid: true, mode: 'unchecked-calibration' });
});

test('controller preview changes exactly one servo per deterministic step', () => {
  const spec = loadSpec();
  const home = referencePose(spec, 'home');
  const park = referencePose(spec, 'park');
  const converted = controllerCompatiblePath(spec, [home, park]);
  assert.equal(converted.success, true);
  for (let index = 1; index < converted.path.length; index += 1) {
    const changed = spec.joints.filter((joint) => converted.path[index - 1][joint.id] !== converted.path[index][joint.id]);
    assert.equal(changed.length, 1);
  }
  assert.deepEqual(converted.path.at(-1), park);
});
