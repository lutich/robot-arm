// Servo cables, the PCA9685 ribbon and the power leads, routed through the link frames.

import * as THREE from 'three';
import { ARM, GRIPPER } from '../config.js';

/**
 * Servo cables routed from each servo to the PCA9685 on the upper arm's -X side plate, and the board's ribbon
 * and power leads down to the desk. Every route is a list of anchors in link frames, so the
 * cables follow the arm; between anchors they sag a little like real slack wire.
 * Rebuilt only when the pose changes.
 */
export function buildCables(M, G, zS1, pca) {
  const group = new THREE.Group();
  const L3 = ARM.L3, L4 = ARM.L4, gz = GRIPPER.gear[1];
  // channel k of the PCA9685 on the -X side plate (arm.js PCA): the cable comes round the back edge
  // and ends in the plugged connector
  const ch = k => { const z = pca.z + pca.headerZ(k); return [['shoulder', [pca.face - 1.2, -2.6, z - 0.4]], ['shoulder', [pca.face - 1.7, pca.y - 0.85, z]]]; };
  const pz = pca.z, px = pca.face;
  const servoWires = [M.cableBrown, M.cableRed, M.cableOrange];
  const routes = [
    { mats: servoWires, pts: [['base', [-3.77, 0, zS1 - 2.9]], ['base', [-4.6, -1.8, zS1 - 1.5]], ['turret', [-2.9, -3.8, 8.8]], ['shoulder', [-1.8, -2.9, 0.8]], ...ch(0)], sag: 0.6 },
    { mats: servoWires, pts: [['turret', [-1.05, -3.77, ARM.H_S2]], ['turret', [-1.6, -4.4, ARM.H_S2 + 0.9]], ['shoulder', [-1.4, -3.0, 1.2]], ...ch(1)], sag: 0.5 },
    { mats: servoWires, pts: [['elbow', [-1.05, 0, -3.77]], ['elbow', [-1.6, -1.6, -3.6]], ['shoulder', [-1.3, -2.9, ARM.L2 - 2.0]], ...ch(2)], sag: 0.7 },
    { mats: servoWires, pts: [['elbow', [-1.05, -3.77, L3]], ['elbow', [-0.9, -1.4, L3 - 3.2]], ['elbow', [-0.7, -0.85, L3 * 0.45]], ['elbow', [-1.2, -1.6, 0.4]], ['shoulder', [-1.1, -3.0, ARM.L2 - 1.6]], ...ch(3)], sag: 0.5 },
    { mats: servoWires, pts: [['wrist', [3.77, 0, L4 - 3.4]], ['wrist', [3.3, -1.7, 0.6]], ['elbow', [0.4, -0.85, L3 * 0.45]], ['elbow', [-0.9, -1.7, 0.6]], ['shoulder', [-0.9, -3.1, ARM.L2 - 1.2]], ...ch(4)], sag: 0.5 },
    { mats: servoWires, pts: [['hand', [4.62, -3.52, gz]], ['hand', [3.6, -3.8, -1.0]], ['wrist', [2.2, -2.1, 1.2]], ['elbow', [0.65, -0.75, L3 * 0.45 + 0.3]], ['elbow', [-0.7, -1.8, 0.8]], ['shoulder', [-0.7, -3.2, ARM.L2 - 0.8]], ...ch(5)], sag: 0.6 },
    // rainbow ribbon from the board's I2C pins to the Raspberry Pi GPIO header on the desk
    { mats: [M.wireYellow, M.wireGreen, M.wireBlue, M.wirePurple], pts: [['shoulder', [px - 0.3, pca.y + 0.1, pz - 3.9]], ['shoulder', [px - 0.6, -1.6, pz - 5.0]], ['root', [-1.5, -7.5, 4.5]], ['root', [-6.4, -9.2, 1.9]]], sag: 1.6, r: 0.045 },
    // red and black power leads from the board's terminal to the bench supply
    { mats: [M.cableRed, M.wireBlack], pts: [['shoulder', [px - 0.86, pca.y + 1.0, pz - 0.55]], ['shoulder', [px - 0.9, pca.y + 0.6, pz - 3.2]], ['shoulder', [px - 0.7, -1.4, pz - 5.2]], ['root', [-8, -6, 3]], ['root', [-22, -6, -0.6]], ['root', [-28, -0.5, 2.5]]], sag: 2.0, r: 0.08 },
  ];
  const v = new THREE.Vector3();
  const objs = routes.map(r => r.mats.map(m => { const me = new THREE.Mesh(new THREE.BufferGeometry(), m); me.castShadow = true; group.add(me); return me; }));
  let lastKey = '';
  const update = () => {
    G.root.updateMatrixWorld(true);
    const inv = G.root.matrixWorld.clone().invert();
    const key = ['turret', 'shoulder', 'elbow', 'wrist', 'hand'].map(k => G[k].matrixWorld.elements.map(x => x.toFixed(3)).join()).join('|');
    if (key === lastKey) return;
    lastKey = key;
    routes.forEach((r, ri) => {
      const pts = r.pts.map(([g, p]) => v.set(...p).applyMatrix4(G[g].matrixWorld).applyMatrix4(inv).clone());
      const pathPts = [pts[0]];
      for (let i = 1; i < pts.length; i++) {
        const a = pts[i - 1], b = pts[i], d = a.distanceTo(b);
        pathPts.push(a.clone().lerp(b, 0.5).add(new THREE.Vector3(0, 0, -Math.min(r.sag, 0.12 * d))));
        pathPts.push(b);
      }
      const curve = new THREE.CatmullRomCurve3(pathPts, false, 'centripetal');
      const n = Math.min(160, Math.max(24, Math.round(curve.getLength() * 3)));
      const frames = curve.computeFrenetFrames(n, false);
      const k = r.mats.length, spacing = (r.r || 0.058) * 2;
      objs[ri].forEach((me, si) => {
        const off = [];
        for (let j = 0; j <= n; j++) off.push(curve.getPointAt(j / n).add(frames.binormals[j].clone().multiplyScalar((si - (k - 1) / 2) * spacing)));
        me.geometry.dispose();
        me.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(off), n, r.r || 0.058, 6, false);
      });
    });
  };
  return { group, update };
}
