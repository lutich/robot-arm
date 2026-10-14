// The claw: base plates, meshing gears, parallel links and fingers driven by the jaw gap.

import * as THREE from 'three';
import { GRIPPER } from '../config.js';
import { thetaForGap } from '../kinematics.js';
import { V, mesh, at, orient, box, cyl, addHoles, hullShape, plateFromShape, joint } from './geometry.js';
import { screwAt, servo, horn } from './parts.js';

const RAD = Math.PI / 180;

/** Finger outline in finger-local coordinates (u outward, v down from the gear-link end): a straight bar. */
export function fingerShape() {
  const tu = GRIPPER.tip[0], tv = GRIPPER.tip[1];
  // a straight bar (photos/arm-tags-106-107-109-*.jpg): a lug at the top around both pivots, then straight
  // down and slightly inward to the pad block; the pad position itself is the measured GRIPPER.tip
  const pts = [[-0.45, -0.35], [0.3, -0.8], [1.65, -0.95], [1.85, -0.4], [0.75, 0.75], [tu + 1.0, tv - 0.9], [tu + 0.95, tv + 0.72],
               [tu + 0.3, tv + 0.72], [tu + 0.28, tv - 0.62], [-0.5, 0.45]];
  const s = new THREE.Shape();
  s.moveTo(pts[0][0], -pts[0][1]);
  for (const [u, v] of pts.slice(1)) s.lineTo(u, -v);
  s.closePath();
  addHoles(s, [[0, 0, 0.13], [1.35, -0.5, 0.13]]);
  return s;
}

export function linkPlate(M, len, w = 0.65, t = 0.2) {
  const r = w / 2, s = new THREE.Shape();
  s.moveTo(-r, 0); s.lineTo(-r, -len); s.absarc(0, -len, r, Math.PI, 2 * Math.PI, false);
  s.lineTo(r, 0); s.absarc(0, 0, r, 0, Math.PI, false);
  return plateFromShape(s, t, M.acrylic, 0.03);
}

export function gearShape(r, teeth = 14) {
  const s = new THREE.Shape();
  for (let i = 0; i <= teeth * 4; i++) {
    const a = i / (teeth * 4) * Math.PI * 2;
    const rr = (i % 4 === 1 || i % 4 === 2) ? r + 0.1 : r - 0.1;
    if (i === 0) s.moveTo(rr * Math.cos(a), rr * Math.sin(a)); else s.lineTo(rr * Math.cos(a), rr * Math.sin(a));
  }
  return addHoles(s, [[0, 0, 0.2]]);
}

/**
 * Gripper in the hand frame: u = X (jaw axis), v = +Z (down the tool), depth = Y.
 * Back plate at y = -0.45, gears and front plate in front, links and fingers in two layers.
 */
export function buildGripper(M) {
  const group = new THREE.Group();
  const joints = [];
  const { gear, outer, link } = GRIPPER;
  const fl = horn(M, 1.0, M.hornBlack); fl.rotation.x = Math.PI; at(fl, 0, 0, 0.3); group.add(fl);
  // base plates: rounded outline around the gears, the outer pivots and the flange
  const outline = () => addHoles(hullShape([[-gear[0], gear[1], 1.35], [gear[0], gear[1], 1.35], [-outer[0], outer[1], 0.5], [outer[0], outer[1], 0.5], [0, 0.45, 0.9]]),
    [[-gear[0], gear[1], 0.2], [gear[0], gear[1], 0.2], [-outer[0], outer[1], 0.13], [outer[0], outer[1], 0.13]]);
  for (const y of [-0.5, 0.62]) {
    const p = plateFromShape(outline(), 0.2, M.acrylic, 0.03);
    orient(p, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(p, 0, y, 0); group.add(p);
  }
  for (const sx of [-1, 1]) {
    // standoffs between the plates and screws on both faces
    const so = cyl(0.17, 1.0, M.screw, 'y', 6); at(so, sx * outer[0], 0.06, outer[1]); group.add(so);
    for (const y of [-0.62, 0.74]) screwAt(group, M, V(sx * outer[0], y, outer[1]), V(0, Math.sign(y), 0), 0.14);
    screwAt(group, M, V(sx * gear[0], 0.74, gear[1]), V(0, 1, 0), 0.16);
  }
  // S6 servo behind the back plate (photos/arm-tags-106-107-109-*.jpg, gripper-gears-closeup.jpg, docs/gripper-layout.md):
  // shaft through the plate into the -X gear, label side up toward the wrist (tag 107), body reaching 4 cm toward
  // -X, the cable leaving at the shaft end (+X, next to the S5 axis; tag 106 on a clear spacer there).
  // Seen from the front with the gripper hanging down, the servo is on the right of the gear plate.
  const s6 = orient(servo(M, { cableAtShaft: true }), V(-1, 0, 0), V(0, 0, 1), V(0, 1, 0));
  at(s6, -gear[0], GRIPPER.s6ShaftY, gear[1]); group.add(s6);
  // gears between the plates
  const gears = [];
  for (const sx of [-1, 1]) {
    const holder = joint(new THREE.Group()); holder.position.set(sx * gear[0], 0.3, gear[1]);
    const gm = plateFromShape(gearShape(0.8), 0.25, M.acrylic, 0.02); holder.add(gm);
    group.add(holder); gears.push({ holder, sx }); joints.push(holder);
  }
  // links and fingers per side
  const sides = [];
  for (const sx of [-1, 1]) {
    const gl = joint(new THREE.Group()), ol = joint(new THREE.Group()), finger = joint(new THREE.Group());
    gl.position.set(sx * gear[0], 0, gear[1]); ol.position.set(sx * outer[0], 0, outer[1]);
    for (const y of [-0.3, 0.35]) {
      const a = linkPlate(M, link); orient(a, V(1, 0, 0), V(0, 0, -1), V(0, 1, 0)); a.position.y = y; gl.add(a);
      const b = linkPlate(M, link); orient(b, V(1, 0, 0), V(0, 0, -1), V(0, 1, 0)); b.position.y = y; ol.add(b);
    }
    for (const lk of [gl, ol]) for (const y of [-0.45, 0.5]) screwAt(lk, M, V(0, y, link), V(0, Math.sign(y), 0), 0.13);
    for (const y of [-0.42, 0.42]) {
      const fm = plateFromShape(fingerShape(), 0.2, M.acrylic, 0.03);
      orient(fm, V(sx, 0, 0), V(0, 0, -1), V(0, sx, 0)); at(fm, 0, y, 0); finger.add(fm);
    }
    // spacers between the two finger plates, and the screws at the joints
    for (const [u, v] of [[-0.3, 2.8], [GRIPPER.tip[0] + 0.7, GRIPPER.tip[1] - 0.9]]) {   // on the bar's centre line
      const sp = cyl(0.16, 0.65, M.screw, 'y', 6); at(sp, sx * u, 0, v); finger.add(sp);
      for (const y of [-0.55, 0.55]) screwAt(finger, M, V(sx * u, y, v), V(0, Math.sign(y), 0), 0.13);
    }
    const pad = box(GRIPPER.pad.t, GRIPPER.pad.w, GRIPPER.pad.h, M.pad, 0.06);
    at(pad, sx * (GRIPPER.tip[0] + GRIPPER.pad.t / 2), 0, GRIPPER.tip[1]); finger.add(pad);
    group.add(gl, ol, finger);
    sides.push({ sx, gl, ol, finger });
    joints.push(gl, ol, finger);
  }

  const setGap = gap => {
    const t = thetaForGap(Math.max(0, gap)) * RAD;
    for (const { sx, gl, ol, finger } of sides) {
      gl.rotation.y = sx * t; ol.rotation.y = sx * t;
      finger.position.set(sx * (gear[0] + link * Math.sin(t)), 0, gear[1] + link * Math.cos(t));
    }
    for (const { holder, sx } of gears) holder.rotation.set(Math.PI / 2, 0, 0), holder.rotateZ(-sx * t * 1.0);
  };
  setGap(4.5);
  return { group, setGap, joints };
}
