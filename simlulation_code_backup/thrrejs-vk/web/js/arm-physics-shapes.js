// Approximate arm geometry used only for simulated arm/cube dynamics.
// These shapes are not a clearance model and must not gate motion.

import { ARM, BASE, GRIPPER } from './config.js';
import { point, compose, I3, padDepthForTheta, thetaForGap } from './kinematics.js';

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];

function bound(shape) {
  if (shape.kind === 'capsule') {
    const c = [(shape.a[0] + shape.b[0]) / 2, (shape.a[1] + shape.b[1]) / 2, (shape.a[2] + shape.b[2]) / 2];
    const d = sub(shape.b, shape.a);
    return { c, r: Math.sqrt(dot(d, d)) / 2 + shape.r };
  }
  return { c: shape.c, r: Math.sqrt(dot(shape.h, shape.h)) };
}

function multiplyR(a, b) {
  return compose({ R: a, p: [0, 0, 0] }, b).R;
}

function boxInFrame(frame, offset, halfSize, extraR = null) {
  return { kind: 'box', c: point(frame, offset), R: extraR ? multiplyR(frame.R, extraR) : frame.R, h: halfSize };
}

function capsuleInFrame(frame, a, b, radius) {
  return { kind: 'capsule', a: point(frame, a), b: point(frame, b), r: radius };
}

const LINK = { base: 0, turret: 1, upper: 2, fore: 3, wrist: 4, hand: 5, fingerL: 6, fingerR: 6 };

export function armPhysicsShapes(frames, gap) {
  const shapes = [];
  const add = (name, link, shape) => { shape.name = name; shape.link = link; shapes.push(shape); };
  const world = { R: I3(), p: [0, 0, 0] };
  for (const side of [-1, 1]) {
    const end = side < 0 ? 'rear' : 'front';
    const y = BASE.offsetY + side * BASE.beamY;
    for (const [level, z] of [['lower', BASE.plateT / 2], ['upper', BASE.h - BASE.plateT / 2]])
      add(`base ${end} ${level} plate`, LINK.base, boxInFrame(world, [0, y, z], [BASE.w / 2, BASE.beamD / 2, BASE.plateT / 2]));
    add(`base ${end} web`, LINK.base, boxInFrame(world, [0, BASE.offsetY + side * BASE.webY, BASE.h / 2], [BASE.w / 2, BASE.webT / 2, (BASE.h - 0.3) / 2]));
  }
  add('S1 servo', LINK.base, boxInFrame(world, [-1, 0, 4.5], [3.1, 1.4, 2]));
  add('turret', LINK.turret, boxInFrame(frames.base, [0, -0.8, 8.95], [2.7, 2.6, 1.95]));
  add('upper arm', LINK.upper, boxInFrame(frames.shoulder, [0, 0, ARM.L2 / 2], [2.9, 1.3, ARM.L2 / 2 - 1.3]));
  add('S3 servo', LINK.fore, boxInFrame(frames.elbow, [0, 0, -0.95], [2.1, 1.25, 2.55]));
  add('forearm', LINK.fore, capsuleInFrame(frames.elbow, [0, 0, 1.9], [0, 0, ARM.L3 - 2.8], 0.95));
  add('S4 bracket', LINK.fore, boxInFrame(frames.elbow, [0, -1, ARM.L3 - 0.9], [2.6, 2.1, 1.6]));
  add('wrist bracket', LINK.wrist, boxInFrame(frames.wrist, [0, 0, 1.4], [2.9, 1.3, 1.3]));
  add('S5 servo', LINK.wrist, boxInFrame(frames.wrist, [1.035, 0, ARM.L4 - 2.335], [2.04, 0.99, 1.84]));
  add('gripper plate', LINK.hand, boxInFrame(frames.hand, [0, 0, 1.25], [GRIPPER.plate.w / 2, 0.65, 0.95]));
  add('S6 servo', LINK.hand, boxInFrame(frames.hand, [GRIPPER.gear[0] + 1.035, -2.635, GRIPPER.gear[1]], [2.04, 1.84, 0.99]));

  const theta = thetaForGap(gap) * Math.PI / 180;
  const padDepth = padDepthForTheta(thetaForGap(gap));
  const { gear, link, pad } = GRIPPER;
  for (const side of [-1, 1]) {
    const name = side < 0 ? 'left' : 'right';
    const linkIndex = side < 0 ? LINK.fingerL : LINK.fingerR;
    const gearX = side * gear[0];
    const end = [gearX + side * link * Math.sin(theta), 0, gear[1] + link * Math.cos(theta)];
    add(`${name} link`, linkIndex, capsuleInFrame(frames.hand, [gearX, 0, gear[1]], end, 0.35));
    const innerX = side * (gap / 2 + pad.t);
    add(`${name} finger`, linkIndex, boxInFrame(frames.hand, [innerX + side * 0.3, 0, (end[2] + padDepth + 0.7) / 2], [0.3, 0.4, (padDepth + 0.7 - end[2]) / 2]));
    add(`${name} pad`, linkIndex, boxInFrame(frames.hand, [side * (gap / 2 + pad.t / 2), 0, padDepth], [pad.t / 2, pad.w / 2, pad.h / 2]));
  }
  for (const shape of shapes) shape.bound = bound(shape);
  return shapes;
}
