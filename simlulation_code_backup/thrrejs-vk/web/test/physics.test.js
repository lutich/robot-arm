import test from 'node:test';
import assert from 'node:assert/strict';
import { PhysicsWorld } from '../src/core/physics.js';
import { referencePose } from '../src/core/kinematics.js';
import { loadSpec } from './helpers.js';

test('Rapier world initializes fixtures, arm interaction bodies and dynamic cubes', async () => {
  const spec = loadSpec(); const physics = await new PhysicsWorld(spec).initialize(referencePose(spec, 'home'));
  assert.equal(physics.ready, true);
  assert.equal(physics.cubeBodies.size, 4);
  assert.ok(physics.robotBodies.size >= 5);
  physics.setRobotPose(referencePose(spec, 'park'));
  physics.step(1 / 120);
  assert.ok(Math.abs(physics.world.timestep - 1 / 120) < 1e-8);
  assert.equal(Object.keys(physics.cubeTransforms()).length, 4);
  physics.graspCube('red', referencePose(spec, 'home'));
  assert.equal(physics.carriedCubeId, 'red');
  physics.releaseCube();
  assert.equal(physics.carriedCubeId, null);
});
