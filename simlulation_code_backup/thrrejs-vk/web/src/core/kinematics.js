import { clamp, compose, distance, identityTransform, lerp, rotationX, rotationZ, seededRandom, transformPoint } from './math.js';

export function jointMap(spec) {
  return Object.fromEntries(spec.joints.map((joint) => [joint.id, joint]));
}

export function referencePose(spec, name) {
  if (!['home', 'park'].includes(name)) throw new Error(`Unknown reference pose: ${name}`);
  return Object.fromEntries(spec.joints.map((joint) => [joint.id, joint[name]]));
}

export function tickToDegrees(joint, tick) {
  if (joint.calibrationAnchors?.length >= 2) {
    const anchors = [...joint.calibrationAnchors].sort((a, b) => a.tick - b.tick);
    let lower = anchors[0]; let upper = anchors[1];
    if (tick >= anchors.at(-1).tick) [lower, upper] = anchors.slice(-2);
    else if (tick > anchors[0].tick) {
      for (let index = 0; index < anchors.length - 1; index += 1) {
        if (tick <= anchors[index + 1].tick) { lower = anchors[index]; upper = anchors[index + 1]; break; }
      }
    }
    return lerp(lower.degrees, upper.degrees, (tick - lower.tick) / (upper.tick - lower.tick));
  }
  if (!joint.ticksPerDegree) return null;
  return joint.direction * (tick - joint.zero) / joint.ticksPerDegree;
}

export function degreesToTick(joint, degrees) {
  if (joint.calibrationAnchors?.length >= 2) {
    const anchors = [...joint.calibrationAnchors].sort((a, b) => a.degrees - b.degrees);
    let lower = anchors[0]; let upper = anchors[1];
    if (degrees >= anchors.at(-1).degrees) [lower, upper] = anchors.slice(-2);
    else if (degrees > anchors[0].degrees) {
      for (let index = 0; index < anchors.length - 1; index += 1) {
        if (degrees <= anchors[index + 1].degrees) { lower = anchors[index]; upper = anchors[index + 1]; break; }
      }
    }
    return clamp(Math.round(lerp(lower.tick, upper.tick, (degrees - lower.degrees) / (upper.degrees - lower.degrees))), joint.min, joint.max);
  }
  return clamp(Math.round(joint.zero + degrees * joint.ticksPerDegree / joint.direction), joint.min, joint.max);
}

export function gripperState(spec, tick) {
  const closure = clamp((tick - spec.gripper.openTick) / (spec.gripper.closedTick - spec.gripper.openTick), 0, 1);
  return {
    closure,
    gap: lerp(spec.dimensions.gripperOpenGap, spec.dimensions.gripperClosedGap, closure),
    length: lerp(spec.dimensions.gripperOpenLength, spec.dimensions.gripperClosedLength, closure),
  };
}

export function forwardKinematics(spec, counts) {
  const joints = jointMap(spec);
  const radians = (id) => tickToDegrees(joints[id], counts[id]) * Math.PI / 180;
  let transform = compose(identityTransform(), rotationZ(radians('S1')));
  const base = [0, 0, 0];
  const shoulder = transformPoint(transform, [0, 0, spec.dimensions.shoulderHeight]);
  transform = compose(transform, rotationX(-radians('S2')), [0, 0, spec.dimensions.shoulderHeight]);
  const elbow = transformPoint(transform, [0, 0, spec.dimensions.upperArm]);
  transform = compose(transform, rotationX(-radians('S3')), [0, 0, spec.dimensions.upperArm]);
  const wrist = transformPoint(transform, [0, 0, spec.dimensions.forearm]);
  transform = compose(transform, rotationX(-radians('S4')), [0, 0, spec.dimensions.forearm]);
  const flange = transformPoint(transform, [0, 0, spec.dimensions.wrist]);
  transform = compose(transform, rotationZ(radians('S5')), [0, 0, spec.dimensions.wrist]);
  const grip = gripperState(spec, counts.S6);
  const tcp = transformPoint(transform, [0, 0, grip.length]);
  return { base, shoulder, elbow, wrist, flange, tcp, gripper: grip, toolTransform: transform };
}

export function interpolateCounts(spec, start, end, amount) {
  return Object.fromEntries(spec.joints.map((joint) => [joint.id, Math.round(lerp(start[joint.id], end[joint.id], amount))]));
}

export function countDistance(spec, a, b, includeGripper = true) {
  const joints = includeGripper ? spec.joints : spec.joints.slice(0, 5);
  return Math.hypot(...joints.map((joint) => a[joint.id] - b[joint.id]));
}

export function solveInverseKinematics(spec, target, options = {}) {
  const tolerance = options.tolerance ?? spec.planning.goalTolerance;
  const random = seededRandom(options.seed ?? spec.planning.seed);
  const controlled = spec.joints.slice(0, 4);
  const retained = options.start ?? referencePose(spec, 'home');
  const starts = [retained, referencePose(spec, 'home'), referencePose(spec, 'park')];
  for (let index = 0; index < 14; index += 1) {
    starts.push(Object.fromEntries(spec.joints.map((joint) => [joint.id, joint.id === 'S6'
      ? (options.gripperTick ?? retained.S6)
      : Math.round(lerp(joint.min, joint.max, random()))])));
  }

  let best = null;
  for (const seedPose of starts) {
    const pose = { ...retained, ...seedPose, S5: options.s5 ?? retained.S5, S6: options.gripperTick ?? retained.S6 };
    let step = 64;
    let error = distance(forwardKinematics(spec, pose).tcp, target);
    for (let pass = 0; pass < 90 && error > tolerance; pass += 1) {
      let improved = false;
      for (const joint of controlled) {
        let localBest = pose[joint.id]; let localError = error;
        for (const direction of [-1, 1]) {
          const candidate = clamp(Math.round(pose[joint.id] + direction * step), joint.min, joint.max);
          pose[joint.id] = candidate;
          const candidateError = distance(forwardKinematics(spec, pose).tcp, target);
          if (candidateError < localError) { localBest = candidate; localError = candidateError; }
        }
        pose[joint.id] = localBest;
        if (localError + 0.01 < error) { error = localError; improved = true; }
      }
      if (!improved) step /= 2;
      if (step < 0.5) break;
    }
    const candidate = { counts: Object.fromEntries(spec.joints.map((joint) => [joint.id, Math.round(pose[joint.id])])), error, tcp: forwardKinematics(spec, pose).tcp };
    if (!best || candidate.error < best.error) best = candidate;
    if (candidate.error <= tolerance) break;
  }
  return { ...best, reachable: best.error <= tolerance };
}
