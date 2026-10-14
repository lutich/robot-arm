import test from 'node:test';
import assert from 'node:assert/strict';
import { validateArmSpec } from '../src/config/spec.js';
import { pathCost, planReferenceTransition } from '../src/core/planning.js';
import { referencePose } from '../src/core/kinematics.js';
import { loadSpec } from './helpers.js';

test('schema rejects unit drift and loss of provenance', () => {
  const wrongUnits = structuredClone(loadSpec()); wrongUnits.metadata.units = 'centimetres';
  assert.throws(() => validateArmSpec(wrongUnits), /metadata.units/);
  const promotedBounds = structuredClone(loadSpec()); promotedBounds.joints[0].boundsStatus = 'safe';
  assert.throws(() => validateArmSpec(promotedBounds), /historical-unvalidated provenance/);
});

test('schema rejects reference poses outside historical bounds', () => {
  const spec = structuredClone(loadSpec()); spec.joints[3].park = spec.joints[3].max + 1;
  assert.throws(() => validateArmSpec(spec), /reference counts must be inside/);
});

test('schema reserves a measured millimetre configuration for gripper marker tracking', () => {
  const spec = structuredClone(loadSpec());
  assert.deepEqual(spec.tracking.gripperMarker.markerToTcpMm, [0, 0, 0]);
  spec.tracking.gripperMarker.id = 5;
  assert.throws(() => validateArmSpec(spec), /reserved DICT_4X4_50 marker id 6/);
});

test('reference calibration planning is deterministic and explicitly unchecked', () => {
  const spec = loadSpec(); const home = referencePose(spec, 'home');
  const first = planReferenceTransition(spec, home, 'park', { ignoreCubes: true });
  const second = planReferenceTransition(spec, home, 'park', { ignoreCubes: true });
  assert.equal(first.success, true);
  assert.deepEqual(first.path, second.path);
  assert.equal(pathCost(spec, first.path), pathCost(spec, second.path));
  assert.equal(first.verification.mode, 'unchecked-calibration');
});

test('motion cost prefers no movement over a needless servo adjustment', () => {
  const spec = loadSpec(); const home = referencePose(spec, 'home');
  assert.ok(pathCost(spec, [home]) < pathCost(spec, [home, { ...home, S1: home.S1 + 20 }]));
});
