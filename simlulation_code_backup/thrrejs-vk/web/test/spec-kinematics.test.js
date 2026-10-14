import test from 'node:test';
import assert from 'node:assert/strict';
import { validateArmSpec, createRenderSpec } from '../src/config/spec.js';
import { degreesToTick, forwardKinematics, referencePose, solveInverseKinematics, tickToDegrees } from '../src/core/kinematics.js';
import { loadSpec } from './helpers.js';

test('canonical browser spec uses millimetres and preserves provenance categories', () => {
  const spec = validateArmSpec(loadSpec());
  assert.equal(spec.metadata.units, 'millimetres');
  assert.equal(spec.fixture.upperPlate.top, 25);
  assert.equal(referencePose(spec, 'home').S6, 650);
  assert.equal(spec.visualReferences.park.contactSurface, 'Base_upper_plate');
  assert.deepEqual(spec.provenance.validatedSafeLimits, []);
  assert.ok(spec.joints.every((joint) => joint.boundsStatus === 'historical-unvalidated'));
  assert.equal(createRenderSpec(spec).dimensions.upperArm, 0.105);
});

test('tick/angle conversion and forward kinematics are deterministic', () => {
  const spec = loadSpec(); const s1 = spec.joints[0]; const s2 = spec.joints[1];
  assert.equal(tickToDegrees(s1, s1.zero), 0);
  assert.ok(Math.abs(degreesToTick(s1, tickToDegrees(s1, 427)) - 427) <= 1);
  assert.equal(tickToDegrees(s2, 427), 20);
  const first = forwardKinematics(spec, referencePose(spec, 'home'));
  const second = forwardKinematics(spec, referencePose(spec, 'home'));
  assert.deepEqual(first.tcp, second.tcp);
  assert.ok(first.tcp.every(Number.isFinite));
});

test('inverse kinematics recovers a reachable forward-kinematic target', () => {
  const spec = loadSpec(); const home = referencePose(spec, 'home');
  const target = forwardKinematics(spec, home).tcp;
  const solution = solveInverseKinematics(spec, target, { start: home, gripperTick: home.S6 });
  assert.equal(solution.reachable, true);
  assert.ok(solution.error <= spec.planning.goalTolerance);
});
