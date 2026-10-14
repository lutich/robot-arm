import { forwardKinematics, interpolateCounts, referencePose, solveInverseKinematics } from './kinematics.js';
import { distance } from './math.js';

function changedJointCount(spec, a, b) {
  return spec.joints.filter((joint) => a[joint.id] !== b[joint.id]).length;
}

export function pathMetrics(spec, path) {
  let countTravel = 0; let changedServos = 0; let tcpLength = 0; let acceleration = 0;
  let previousDelta = null;
  for (let index = 1; index < path.length; index += 1) {
    const previous = path[index - 1];
    const delta = spec.joints.map((joint) => path[index][joint.id] - previous[joint.id]);
    countTravel += delta.reduce((sum, value) => sum + Math.abs(value), 0);
    changedServos += changedJointCount(spec, previous, path[index]);
    tcpLength += distance(forwardKinematics(spec, previous).tcp, forwardKinematics(spec, path[index]).tcp);
    if (previousDelta) acceleration += delta.reduce((sum, value, jointIndex) => sum + Math.abs(value - previousDelta[jointIndex]), 0);
    previousDelta = delta;
  }
  return { countTravel, changedServos, tcpLength, acceleration, duration: countTravel / 220 };
}

export function costFromMetrics(spec, metrics) {
  const weights = spec.planning.weights;
  return weights.changedServos * metrics.changedServos
    + weights.countTravel * metrics.countTravel
    + weights.duration * metrics.duration
    + weights.acceleration * metrics.acceleration
    + weights.tcpLength * metrics.tcpLength;
}

export function pathCost(spec, path) {
  return costFromMetrics(spec, pathMetrics(spec, path));
}

/**
 * Calibration-only joint planning. This deliberately performs no geometric
 * collision or clearance checks. Bounds are enforced by the kinematics and
 * controller layers; real multi-joint task playback stays disabled in the UI.
 */
export function optimizeJointPath(spec, start, goal) {
  const path = [{ ...start }, { ...goal }];
  return {
    success: true,
    path,
    verification: { valid: true, mode: 'unchecked-calibration' },
    metrics: pathMetrics(spec, path),
    algorithm: 'direct calibration waypoint (collision checks disabled)',
  };
}

/** Convert coordinated waypoints to deterministic one-servo-at-a-time steps. */
export function controllerCompatiblePath(spec, path) {
  if (!path.length) return { success: false, path: [], reason: 'Path is empty' };
  const coordinated = [{ ...path[0] }];
  const stepCounts = Math.max(1, spec.planning.controllerStepCounts ?? 8);
  for (let index = 1; index < path.length; index += 1) {
    const largestDelta = Math.max(...spec.joints.map((joint) => Math.abs(path[index][joint.id] - path[index - 1][joint.id])));
    const steps = Math.max(1, Math.ceil(largestDelta / stepCounts));
    for (let sample = 1; sample <= steps; sample += 1) coordinated.push(interpolateCounts(spec, path[index - 1], path[index], sample / steps));
  }
  const result = [{ ...coordinated[0] }];
  for (let index = 1; index < coordinated.length; index += 1) {
    let cursor = { ...result.at(-1) };
    for (const joint of spec.joints) {
      if (cursor[joint.id] === coordinated[index][joint.id]) continue;
      cursor = { ...cursor, [joint.id]: coordinated[index][joint.id] };
      result.push(cursor);
    }
  }
  return { success: true, path: result, mode: 'unchecked-calibration' };
}

export function parseTextTask(text, spec) {
  const normalized = text.trim().toLowerCase();
  const colors = spec.world.cubes.map((cube) => cube.id);
  const cube = colors.find((color) => new RegExp(`\\b${color}\\b`).test(normalized));
  if (!cube) throw new Error(`Name a cube: ${colors.join(', ')}`);
  const coordinateMatch = normalized.match(/(?:to|at)\s*\(?\s*(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)\s*(?:mm)?\s*\)?/);
  const destinationColor = colors.find((color) => color !== cube && new RegExp(`(?:to|onto|at)\\s+(?:the\\s+)?${color}`).test(normalized));
  const pickOnly = /\b(?:grab|grip|pick)(?:\s+up)?\b/.test(normalized) && !coordinateMatch && !destinationColor;
  if (pickOnly) return { type: 'pick', cube, destination: null, description: `Grab and lift ${cube} cube` };
  const destination = coordinateMatch
    ? [Number(coordinateMatch[1]), Number(coordinateMatch[2]), spec.dimensions.testCube / 2]
    : spec.world.dropZones[destinationColor ?? cube];
  return { type: 'pick-place', cube, destination, description: `Move ${cube} cube to (${destination[0]}, ${destination[1]}) mm` };
}

export function planTextTask(spec, start, text) {
  const task = parseTextTask(text, spec);
  const cube = spec.world.cubes.find((item) => item.id === task.cube);
  const approach = spec.planning.approachHeight; const lift = spec.planning.liftHeight;
  const contactZ = spec.dimensions.testCube + 10;
  const pickTargets = [
    { label: 'approach pick', point: [cube.position[0], cube.position[1], contactZ + approach], grip: spec.gripper.openTick },
    { label: 'pick', point: [cube.position[0], cube.position[1], contactZ], grip: spec.gripper.openTick },
    { label: 'grasp', point: [cube.position[0], cube.position[1], contactZ], grip: spec.gripper.closedTick },
    { label: 'lift', point: [cube.position[0], cube.position[1], contactZ + lift], grip: spec.gripper.closedTick },
  ];
  const placeTargets = task.type === 'pick' ? [] : [
    { label: 'approach place', point: [task.destination[0], task.destination[1], contactZ + lift], grip: spec.gripper.closedTick },
    { label: 'place', point: [task.destination[0], task.destination[1], contactZ], grip: spec.gripper.closedTick },
    { label: 'release', point: [task.destination[0], task.destination[1], contactZ], grip: spec.gripper.openTick },
    { label: 'retreat', point: [task.destination[0], task.destination[1], contactZ + approach], grip: spec.gripper.openTick },
  ];
  const fullPath = [{ ...start }]; const waypoints = []; const events = []; let cursor = { ...start };
  for (const target of [...pickTargets, ...placeTargets]) {
    const ik = solveInverseKinematics(spec, target.point, { start: cursor, gripperTick: target.grip });
    if (!ik.reachable) return { success: false, task, reason: `${target.label} is outside the provisional IK tolerance (${ik.error.toFixed(1)} mm error)`, waypoints };
    const planned = optimizeJointPath(spec, cursor, ik.counts);
    fullPath.push(...planned.path.slice(1));
    if (target.label === 'grasp') events.push({ type: 'grasp', cube: task.cube, counts: { ...ik.counts } });
    if (target.label === 'release') events.push({ type: 'release', cube: task.cube, counts: { ...ik.counts } });
    waypoints.push({ ...target, counts: ik.counts, ikError: ik.error, uncheckedCalibration: true });
    cursor = ik.counts;
  }
  const metrics = pathMetrics(spec, fullPath);
  return {
    success: true,
    task,
    path: fullPath,
    waypoints,
    events,
    verification: { valid: true, mode: 'unchecked-calibration' },
    metrics,
    cost: costFromMetrics(spec, metrics),
    algorithm: 'Cartesian approach/lift waypoints + IK + unchecked calibration interpolation',
  };
}

export function planReferenceTransition(spec, from, name) {
  return optimizeJointPath(spec, from, referencePose(spec, name));
}
