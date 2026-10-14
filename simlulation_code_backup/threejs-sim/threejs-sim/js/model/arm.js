// The arm: base, turret, upper arm, forearm, wrist and hand, each a joint group.
// Joint distances come from the schematic (docs/images/arm-dimensions.png, canon); bracket,
// horn and claw shapes follow the photos and the kit drawings.

import * as THREE from 'three';
import { ARM, ARM_TAGS, BASE } from '../config.js';
import { V, at, orient, box, cyl, plate, hornHoles, bake, joint } from './geometry.js';
import { screwAt, nut, servo, horn, sidePlate, bend, zipTie } from './parts.js';
import { buildGripper } from './gripper.js';
import { buildCables } from './cables.js';
import { buildTag } from './tags.js';

const RAD = Math.PI / 180;

/**
 * PCA9685 on the upper arm, shoulder frame (cm): board centre at (face, y, z), `face` = the board's
 * component side, foam pad between it and the side plate's outer face at x = -2,7.
 */
export const PCA = { foam: 1.1, y: -0.1, z: 4.3, face: -2.7 - 1.1 - 0.16 };
/** Height of servo header channel k on the board (groups of four with a gap), relative to its centre. */
export const pcaHeaderZ = k => -2.5 + k * 0.254 + Math.floor(k / 4) * 0.33;

/** Centre of the 9 x 8,4 base plate relative to the S1 axis (cm, +Y forward). */
export const BASE_OFFSET_Y = 2.2;

function buildPca(M) {
  const g = new THREE.Group(); at(g, 0, PCA.y, PCA.z);
  const xs = PCA.face, top = 3.125;
  g.add(at(box(PCA.foam, 3.1, 7.2, M.pinkFoam, 0.3), -2.7 - PCA.foam / 2, 0.3, 0.35));          // foam pad, overhangs top and front
  g.add(at(box(0.16, 2.54, 6.25, M.pcbBlue, 0.03), xs + 0.08, 0, 0));
  g.add(at(box(0.12, 0.95, 0.45, M.chip), xs - 0.06, 0.15, -0.05));                                  // PCA9685 chip
  for (let i = 0; i < 4; i++) for (const y of [-0.25, -0.55]) g.add(at(box(0.08, 0.2, 0.32, M.chip), xs - 0.04, y, 0.75 + i * 0.5));   // 220 R arrays
  for (let i = 0; i < 6; i++) g.add(at(box(0.03, 0.2, 0.16, M.gold), xs - 0.015, 0.85, 1.0 + i * 0.27)); // address pads
  g.add(at(box(0.85, 0.7, 0.8, M.terminal, 0.05), xs - 0.425, 0.8, -0.55));                          // power terminal, front edge
  for (const z of [-0.75, -0.35]) screwAt(g, M, V(xs - 0.86, 0.8, z), V(-1, 0, 0), 0.12);
  g.add(at(box(0.1, 0.15, 0.1, M.led), xs - 0.05, 0.75, -1.15));
  const cap = cyl(0.32, 0.75, M.chip, 'x', 20); at(cap, xs - 0.38, 0.35, -2.1); g.add(cap);         // electrolytic capacitor
  const capTop = cyl(0.3, 0.02, M.screw, 'x', 20); at(capTop, xs - 0.76, 0.35, -2.1); g.add(capTop);
  for (let k = 0; k < 16; k++) {                                                                     // 16 x 3 servo headers, back edge
    const z = pcaHeaderZ(k);
    g.add(at(box(0.25, 0.78, 0.24, M.chip), xs - 0.125, -0.85, z));
    if (k < 6) g.add(at(box(1.45, 0.78, 0.26, M.chip, 0.03), xs - 0.97, -0.85, z));                  // plugged servo connectors S1..S6
    else for (const y of [-0.6, -0.85, -1.1]) g.add(at(box(0.6, 0.064, 0.064, M.gold), xs - 0.55, y, z));
  }
  g.add(at(box(0.3, 1.6, 0.25, M.chip), xs - 0.15, 0.1, -top + 0.2));                               // I2C header and its connector
  g.add(at(box(0.5, 1.6, 0.8, M.chip, 0.04), xs - 0.3, 0.1, -top - 0.35));
  for (const z of [2.75, -2.85]) {                                                                   // zip ties around plate, foam and board
    const xb = -2.4, w = xs - 0.03 - xb;
    g.add(at(box(0.05, 3.1, 0.25, M.zip), xs - 0.03, 0.15, z));
    g.add(at(box(0.05, 3.1, 0.25, M.zip), xb, 0.15, z));
    for (const y of [-1.4, 1.7]) g.add(at(box(Math.abs(w), 0.05, 0.25, M.zip), (xs - 0.03 + xb) / 2, y, z));
    const tail = box(2.4, 0.05, 0.22, M.zip); tail.rotation.z = Math.PI / 2 - 0.25; at(tail, xs - 0.35, -2.6, z); g.add(tail);
  }
  return g;
}

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
    const yc = sy * 2.12;
    for (const z of [0.1, H - 0.1]) { const p = plate(9.0, 4.0, 0.2, M.anodised, legHoles(sy), 0.5); at(p, 0, yc, z); plates.add(p); }
    const web = plate(9.0, H - 0.3, 0.2, M.anodised, [-3.475, 0, 3.475].map(x => [x, 0, 0.4]), 0.05);
    orient(web, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(web, 0, sy * 0.13, H / 2); plates.add(web);
    for (const z of [0.15, H - 0.15]) { const bd = bend(M, 9.0, 'x'); at(bd, 0, sy * 0.2, z); plates.add(bd); }
    for (const x of [-4.0, 4.0]) {
      const so = cyl(0.2, H - 0.4, M.screw, 'z', 6); at(so, x, yc + sy * 1.4, H / 2); plates.add(so);
      for (const z of [-0.02, H + 0.02]) screwAt(plates, M, V(x, yc + sy * 1.4, z), V(0, 0, z > 0 ? 1 : -1), 0.17);
    }
  }

  // S1 servo upright in a multi-function bracket on the base, shaft on the S1 axis.
  const zS1 = BASE.s1Shaft;   // S1 shaft height (the case reaches down into the base box)
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
  // PCA9685 servo driver on a pink foam pad on the -X side plate, board facing out (-X), long side
  // along the link (photos/pca9685-closeup.jpg (photo removed 2026-10-13), photos/pca9685-on-upper-arm.jpg, photos/park-front.jpg):
  // the 16 servo headers on the back (-Y) edge with channel 0 at the bottom, the power terminal on the
  // front edge, the I2C header on the bottom edge, two zip ties around plate, foam and board.
  shoulder.add(buildPca(M));

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
  // S5 servo between the bracket's side plates, body along X, its wide (label) faces toward +-Y
  // (photos/arm-tags-108-109-wrist-front.jpg: tag 109 on that face, tag 108 on the side plate)
  // short wire stub: the +X side plate (tag 108 on it) is only 0,8 cm beyond the cable exit
  const s5 = servo(M, { wire: 0.5 }); at(s5, 0, 0, ARM.L4 - 0.48); wrist.add(s5);
  // the +X plate reaches up to just below the S4 horn: tag 108 sits on it (config.js ARM_TAGS)
  for (const sx of [-1, 1]) { const z0 = sx > 0 ? 1.0 : 3.0; wrist.add(sidePlate(M, 1.03 + sx * 2.95, z0, ARM.L4 - 1.0, 2.2, [[0, 4.5 - z0, 0.15]])); }

  // ---- hand (flange frame, Z = tool axis)
  const hand = joint(new THREE.Group()); hand.position.z = ARM.L4; wrist.add(hand);
  const gripper = buildGripper(M); hand.add(gripper.group);

  // AprilTags for tracking the real arm with the cameras (config.js ARM_TAGS)
  const linkGroups = { turret, shoulder, elbow, wrist, hand };
  for (const t of ARM_TAGS) linkGroups[t.link].add(buildTag(t));

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
  const cables = buildCables(M, groups, zS1, { ...PCA, headerZ: pcaHeaderZ });
  root.add(cables.group);
  const setPoseAndCables = (a, gap) => { setPose(a, gap); cables.update(); };
  return { root, setPose: setPoseAndCables, groups };
}
