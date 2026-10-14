import { loadArmSpec } from '../src/config/spec.js';
import { angleFromCounts } from './kinematics.js';

let defaultSpec = null;

export async function loadCanonicalSpec() {
  defaultSpec = await loadArmSpec();
  return defaultSpec;
}

export function configureCanonicalSpec(spec) {
  defaultSpec = spec;
  return spec;
}

function resolvedSpec(spec) {
  if (!spec && !defaultSpec) throw new Error('Canonical arm specification has not been configured');
  return spec ?? defaultSpec;
}

export function applyCanonicalServoMetadata(servos, spec = null) {
  spec = resolvedSpec(spec);
  const visualReferences = {
    HOME: angleFromCounts(servos[4], servos[4].home),
    PARK: angleFromCounts(servos[4], servos[4].park),
  };
  spec.joints.forEach((joint, index) => {
    const servo = servos[index];
    Object.assign(servo, {
      low: joint.min,
      high: joint.max,
      home: joint.home,
      park: joint.park,
    });
    if (index === 4) {
      // HOME and PARK share the documented S5 count. Their photo-derived wrist
      // orientation is a rendering reference, not a second servo calibration.
      servo.qHome = 0;
    }
    if (index === 5) {
      servo.open = spec.gripper.openTick;
      servo.closed = spec.gripper.closedTick;
    }
  });
  return visualReferences;
}

export function countsObject(counts, spec = null) {
  spec = resolvedSpec(spec);
  return Object.fromEntries(spec.joints.map((joint, index) => [joint.id, Math.round(counts[index])]));
}

export function countsArray(counts, spec = null) {
  spec = resolvedSpec(spec);
  return spec.joints.map((joint) => Math.round(counts[joint.id]));
}

/**
 * Build the millimetre planning/physics model from the canonical specification,
 * but use the preserved photo-fitted count-to-angle mapping. This is the bridge
 * that keeps planning and physics data aligned with the detailed visible model
 * rather than silently switching to a second arm calibration.
 */
export function runtimeSpec(servos, cubes = null, spec = null) {
  spec = resolvedSpec(spec);
  const runtime = structuredClone(spec);
  runtime.joints.slice(0, 5).forEach((joint, index) => {
    const servo = servos[index];
    const sign = index === 0 ? -1 : 1;
    joint.calibrationAnchors = [servo.low, servo.high].map((tick) => ({
      tick,
      degrees: sign * angleFromCounts(servo, tick),
      source: 'photo-fitted-render-model-adapter',
    }));
    joint.calibrationStatus = 'photo-fitted-provisional';
  });
  if (cubes) {
    runtime.world.cubes = cubes.map((cube) => ({
      id: cube.id,
      color: cube.colour,
      position: cube.p.map((value) => value * 10),
    }));
    for (const cube of runtime.world.cubes) {
      if (!runtime.world.dropZones[cube.id]) runtime.world.dropZones[cube.id] = [...cube.position];
    }
  }
  return runtime;
}
