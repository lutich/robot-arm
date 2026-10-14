// The arm: base, turret, upper arm, forearm, wrist and hand, each a joint group.
// Joint distances come from the schematic (docs/images/arm-dimensions.png, canon); bracket,
// horn and claw shapes follow the photos and the kit drawings.

import * as THREE from 'three';
import { ARM, BASE } from '../config.js';
import { V, at, orient, box, cyl, plate, hornHoles, bake, joint } from './geometry.js';
import { screwAt, nut, servo, horn, sidePlate, bend, zipTie } from './parts.js';
import { buildGripper } from './gripper.js';
import { buildCables } from './cables.js';

const RAD = Math.PI / 180;

/** Centre of the 9 x 8,4 base plate relative to the S1 axis (cm, +Y forward). */
export const BASE_OFFSET_Y = BASE.offsetY;

/**
 * Builds the arm. Returns { root, setPose(angles, gap), groups }.
 * root is in the board frame; angles follow kinematics.js (degrees).
 */
export function buildArm(M) {
  const root = new THREE.Group();
  root.name = 'arm';

  // ---- static base: two "U beam" brackets, 90 x 42 x 25 mm (docs/images/web/bracket-u-beam-base-dimensions.jpg),
  // lying on their sides with the webs back to back in the middle: legs form the 9 x 8,4 top and
  // bottom plates, the meeting webs form the centre groove, the open sides face front and back.
  const H = BASE.h;
  const base = new THREE.Group(); root.add(base);
  // The plate is not centred on S1: the S1 servo stands at its back, the plate reaches forward
  // over the mat (photos/base-shoulder-closeup.jpg, photos/setup-top-view-base.jpg).
  const plates = new THREE.Group(); plates.position.y = BASE_OFFSET_Y; base.add(plates);
  const legHoles = sy => [-3.475, 0, 3.475].flatMap(x => hornHoles(x, sy * 0.908))
    .concat([[-1.375, sy * 1.608, 0.15], [1.375, sy * 1.608, 0.15], [-4.0, sy * 1.608, 0.15], [4.0, sy * 1.608, 0.15]]);
  for (const sy of [-1, 1]) {
    const yc = sy * BASE.beamY;
    for (const z of [BASE.plateT / 2, H - BASE.plateT / 2]) { const p = plate(BASE.w, BASE.beamD, BASE.plateT, M.anodised, legHoles(sy), 0.5); at(p, 0, yc, z); plates.add(p); }
    const web = plate(BASE.w, H - 0.3, BASE.webT, M.anodised, [-3.475, 0, 3.475].map(x => [x, 0, 0.4]), 0.05);
    orient(web, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(web, 0, sy * BASE.webY, H / 2); plates.add(web);
    for (const z of [0.15, H - 0.15]) { const bd = bend(M, BASE.w, 'x'); at(bd, 0, sy * 0.2, z); plates.add(bd); }
    for (const x of [-4.0, 4.0]) {
      const so = cyl(0.2, H - 0.4, M.screw, 'z', 6); at(so, x, yc + sy * 1.4, H / 2); plates.add(so);
      for (const z of [-0.02, H + 0.02]) screwAt(plates, M, V(x, yc + sy * 1.4, z), V(0, 0, z > 0 ? 1 : -1), 0.17);
    }
  }

  // S1 servo upright in a multi-function bracket on the base, shaft on the S1 axis.
  const zS1 = H + 3.92;   // S1 case on the bracket plate, shaft on top
  const s1 = servo(M); s1.rotation.z = Math.PI; at(s1, 0, 0, zS1); base.add(s1);
  const mb = plate(6.1, 2.6, 0.2, M.anodised, [[-2.4, 0.7, 0.16], [-2.4, -0.7, 0.16], [2.4, 0.7, 0.16], [2.4, -0.7, 0.16], [0, 0, 0.6]], 0.25);
  at(mb, -1.03, 0, H + 0.1); base.add(mb);
  for (const sx of [-1, 1]) {
    const wall = plate(2.6, zS1 - 1.1 - H, 0.2, M.anodised, [[0, 0.3, 0.3]], 0.15);
    orient(wall, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(wall, -1.03 + sx * 3.0, 0, (H + zS1 - 1.1) / 2); base.add(wall);
    const lip = plate(0.8, 2.6, 0.2, M.anodised, [[0, 0.5, 0.15], [0, -0.5, 0.15]], 0.1); at(lip, -1.03 + sx * 2.7, 0, zS1 - 1.05); base.add(lip);
  }
  for (const [x, y] of [[-3.43, 0.7], [-3.43, -0.7], [1.37, 0.7], [1.37, -0.7]]) screwAt(base, M, V(x, y, H + 0.2), V(0, 0, 1), 0.16);

  // ---- turret: S1 horn and the U bracket holding S2 (shaft +X at z = 9,6), body toward the back
  const turret = joint(new THREE.Group()); root.add(turret);
  const h1 = horn(M); at(h1, 0, 0, zS1 + 0.35); turret.add(h1);
  const zT = zS1 + 0.75;
  const tp = plate(5.4, 4.6, 0.2, M.anodised, hornHoles(0, 0).concat([[-1.9, 1.7, 0.15], [1.9, 1.7, 0.15], [-1.9, -1.7, 0.15], [1.9, -1.7, 0.15]]), 0.3);
  at(tp, 0, -0.6, zT); turret.add(tp);
  for (const sx of [-1, 1]) {
    turret.add(at(sidePlate(M, sx * 2.5, zT, ARM.H_S2 + 1.3, 4.6, [[0, ARM.H_S2 - zT, 0.5], [-1.4, 0.8, 0.15], [1.4, 0.8, 0.15]]), sx * 2.5, -0.6, (zT + ARM.H_S2 + 1.3) / 2));
    const b = bend(M, 4.6, 'y'); at(b, sx * 2.4, -0.6, zT + 0.1); turret.add(b);
  }
  const s2 = orient(servo(M), V(0, -1, 0), V(0, 0, -1), V(1, 0, 0)); at(s2, 1.85, 0, ARM.H_S2); turret.add(s2);
  const pin = cyl(0.42, 0.5, M.bearing, 'x', 24); at(pin, -2.55, 0, ARM.H_S2); turret.add(pin);

  // ---- upper arm (shoulder frame: Z along the link, pivots around X)
  // Two long U brackets (52 x 25 x 62,5 mm, horn 50 mm up the leg) bolted base to base: horn to
  // horn 2 x 5,0 + 2 x 0,2 = 10,4, which matches the measured 10,5 (photos/elbow-s3-forearm-closeup.jpg).
  const shoulder = joint(new THREE.Group()); shoulder.position.z = ARM.H_S2; turret.add(shoulder);
  const h2 = horn(M); orient(h2, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h2, 2.05, 0, 0); shoulder.add(h2);
  const zMid = ARM.L2 / 2;
  const legHolesU = (zh, dir) => [...hornHoles(0, 0), [0, dir * 1.86, 0.15], [0, dir * 3.547, 0.15]].map(([u, v, r]) => [u, v + zh, r]);
  for (const sx of [-1, 1]) {
    // lower U: round end at S2, legs up to the middle; upper U: from the middle to the round end at S3
    shoulder.add(sidePlate(M, sx * 2.6, -1.25, zMid - 0.2, 2.5, legHolesU(1.25, 1)));
    shoulder.add(sidePlate(M, sx * 2.6, zMid + 0.2, ARM.L2 + 1.25, 2.5, legHolesU(ARM.L2 + 1.25 - zMid - 0.2, -1)));
    for (const z of [zMid - 0.3, zMid + 0.3]) { const b = bend(M, 2.5, 'y'); at(b, sx * 2.5, 0, z); shoulder.add(b); }
  }
  for (const z of [zMid - 0.1, zMid + 0.1]) { const p = plate(5.0, 2.5, 0.2, M.anodised, [...hornHoles(0, 0), [-1.86, 0, 0.15], [1.86, 0, 0.15]], 0.05); at(p, 0, 0, z); shoulder.add(p); }
  for (const x of [-1.86, 1.86]) { screwAt(shoulder, M, V(x, 0, zMid + 0.2), V(0, 0, 1), 0.16); const n = nut(M, 0.2); n.rotation.set(0, 0, 0); at(n, x, 0, zMid - 0.3); shoulder.add(n); }
  const pin3 = cyl(0.42, 0.45, M.bearing, 'x', 24); at(pin3, -2.4, 0, ARM.L2); shoulder.add(pin3);
  // PCA9685 servo driver on pink foam on the back of the upper arm (photos/base-s2-shoulder-side.jpg)
  const pca = new THREE.Group();
  pca.add(at(box(2.7, 0.6, 6.2, M.pinkFoam, 0.18), 0, 0, 0));
  pca.add(at(box(2.5, 0.16, 6.2, M.pcbBlue, 0.03), 0, -0.38, 0));
  for (let i = 0; i < 4; i++) pca.add(at(box(0.85, 0.12, 0.25, M.chip), 0.55, -0.5, -1.0 + i * 0.5));          // resistor arrays
  pca.add(at(box(0.9, 0.18, 0.9, M.chip), -0.35, -0.52, 1.3));                                               // PCA9685 chip
  pca.add(at(box(0.9, 0.7, 0.8, M.terminal, 0.05), 0.5, -0.75, 2.65));                                        // power terminal
  for (const x of [0.25, 0.75]) screwAt(pca, M, V(x, -1.1, 2.65), V(0, -1, 0), 0.12);
  pca.add(at(box(0.15, 0.1, 0.1, M.led), 0.95, -0.5, 1.9));
  for (let k = 0; k < 16; k++) {                                                                              // 16 x 3 servo header
    const z = -2.75 + k * 0.254;
    pca.add(at(box(0.75, 0.25, 0.22, M.chip), -0.75, -0.6, z));
    if (k < 6) pca.add(at(box(0.78, 1.45, 0.26, M.chip, 0.03), -0.75, -1.3, z));                              // plugged servo connectors S1..S6
    else for (let r = 0; r < 3; r++) pca.add(at(box(0.064, 0.6, 0.064, M.gold), -1.0 + r * 0.254, -0.95, z));
  }
  at(pca, 0.2, -1.65, zMid - 0.4); shoulder.add(pca);

  // ---- forearm (elbow frame). The S3 servo rides on the forearm: its body sits in a bracket
  // behind the elbow, the upper arm's U bracket is screwed to its horn (photos/elbow-s3-forearm-closeup.jpg).
  const elbow = joint(new THREE.Group()); elbow.position.z = ARM.L2; shoulder.add(elbow);
  const h3 = horn(M); orient(h3, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h3, 2.05, 0, 0); elbow.add(h3);
  const s3 = orient(servo(M), V(0, 0, -1), V(0, 1, 0), V(1, 0, 0)); at(s3, 1.85, 0, 0); elbow.add(s3);
  for (const sy of [-1, 1]) {
    const side = plate(4.6, 4.9, 0.2, M.anodised, [[-1.2, 1.5, 0.15], [1.2, 1.5, 0.15], [0, -0.9, 0.4]], 0.15);
    orient(side, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(side, 0, sy * 1.12, -0.8); elbow.add(side);
  }
  const back = plate(4.6, 2.44, 0.2, M.anodised, [[-1.5, 0, 0.15], [1.5, 0, 0.15]], 0.1); at(back, 0, 0, -3.35); elbow.add(back);
  const cap = plate(4.6, 2.44, 0.2, M.anodised, [...hornHoles(0, 0), [-1.6, 0, 0.15], [1.6, 0, 0.15]], 0.1); at(cap, 0, 0, 1.55); elbow.add(cap);
  for (const x of [-1.6, 1.6]) screwAt(elbow, M, V(x, 0, 1.65), V(0, 0, 1), 0.15);
  const tubeLen = ARM.L3 - 4.45;
  const tube = cyl(0.6, tubeLen, M.tube, 'z', 40); at(tube, 0, 0, 1.85 + tubeLen / 2); elbow.add(tube);
  for (const z of [2.05, ARM.L3 - 2.8]) {
    const fl = cyl(1.0, 0.18, M.bearing, 'z', 32); at(fl, 0, 0, z); elbow.add(fl);
    const col = cyl(0.78, 0.45, M.bearing, 'z', 32); at(col, 0, 0, z + (z < 5 ? 0.3 : -0.3)); elbow.add(col);
    for (let k = 0; k < 3; k++) screwAt(elbow, M, V(0.72 * Math.cos(k * 2.09), 0.72 * Math.sin(k * 2.09), z + (z < 5 ? 0.1 : -0.1)), V(0, 0, z < 5 ? 1 : -1), 0.11);
    const grub = cyl(0.09, 0.3, M.screw, 'x', 8); at(grub, 0.85, 0, z + (z < 5 ? 0.3 : -0.3)); elbow.add(grub);
  }
  const zt = zipTie(M, 0.6); at(zt, 0, 0, ARM.L3 * 0.45); elbow.add(zt);
  const cap2 = plate(5.1, 2.4, 0.2, M.anodised, [[0, 0, 0.62], [-1.6, 0, 0.15], [1.6, 0, 0.15]], 0.2); at(cap2, 0, 0, ARM.L3 - 2.65); elbow.add(cap2);
  for (const sx of [-1, 1]) elbow.add(sidePlate(M, sx * 2.45, ARM.L3 - 2.75, ARM.L3 + 1.2, 2.4, [...hornHoles(0, 2.75)]));
  const s4 = orient(servo(M), V(0, -1, 0), V(0, 0, -1), V(1, 0, 0)); at(s4, 1.85, 0, ARM.L3); elbow.add(s4);
  const pin4 = cyl(0.42, 0.4, M.bearing, 'x', 24); at(pin4, -2.25, 0, ARM.L3); elbow.add(pin4);

  // ---- wrist (wrist frame): U bracket from the S4 horn down to the S5 servo
  const wrist = joint(new THREE.Group()); wrist.position.z = ARM.L3; elbow.add(wrist);
  const h4 = horn(M); orient(h4, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h4, 2.05, 0, 0); wrist.add(h4);
  for (const sx of [-1, 1]) wrist.add(sidePlate(M, sx * 2.75, -1.25, 2.8, 2.5, [...hornHoles(0, 1.25)]));
  const wp = plate(5.7, 2.5, 0.2, M.anodised, [[-1.6, 0, 0.18], [1.6, 0, 0.18], [0, 0, 0.5]], 0.2); at(wp, 0, 0, 2.9); wrist.add(wp);
  for (const sx of [-1, 1]) { const b = bend(M, 2.5, 'y'); at(b, sx * 2.65, 0, 2.8); wrist.add(b); }
  const s5 = servo(M); at(s5, 0, 0, ARM.L4 - 0.48); wrist.add(s5);
  for (const sx of [-1, 1]) wrist.add(sidePlate(M, 1.03 + sx * 2.95, 3.0, ARM.L4 - 1.0, 2.2, [[0, 1.5, 0.15]]));

  // ---- hand (flange frame, Z = tool axis)
  const hand = joint(new THREE.Group()); hand.position.z = ARM.L4; wrist.add(hand);
  const gripper = buildGripper(M); hand.add(gripper.group);

  for (const g of [root, turret, shoulder, elbow, wrist, hand, ...gripper.joints]) bake(g);

  const setPose = (a, gap) => {
    turret.rotation.z = -a.s1 * RAD;
    shoulder.rotation.x = -a.s2 * RAD;
    elbow.rotation.x = -a.s3 * RAD;
    wrist.rotation.x = -a.s4 * RAD;
    hand.rotation.z = a.s5 * RAD;
    gripper.setGap(gap);
  };
  const groups = { root, base, turret, shoulder, elbow, wrist, hand };
  const cables = buildCables(M, groups, zS1);
  root.add(cables.group);
  const setPoseAndCables = (a, gap) => { setPose(a, gap); cables.update(); };
  return { root, setPose: setPoseAndCables, groups };
}
