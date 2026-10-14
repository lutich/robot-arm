// Everything around the arm: board, printed mat, desk, partition, Raspberry Pi, bench supply,
// ultrasonic sensor, the OAK camera on its stand, and the wooden cubes.

import * as THREE from 'three';
import { RoundedBoxGeometry } from '../../vendor/RoundedBoxGeometry.js';
import { BOARD, CUBE, MAT, COLOURS } from '../config.js';
import { V, mesh, at, orient, box, cyl, roundedRect } from './geometry.js';
import { screwAt } from './parts.js';
import { canvasTexture, rnd, woodTexture } from './textures.js';

export function buildTable(M) {
  const g = new THREE.Group();
  // Grey aluminium-composite board 30 x 38, top at z = 0, S1 axis 1,5 from its back edge.
  const yc = BOARD.d / 2 - BOARD.axisToBackEdge;
  const board = mesh(new RoundedBoxGeometry(BOARD.w, BOARD.d, BOARD.t, 3, 0.12), M.board); at(board, 0, yc, -BOARD.t / 2); g.add(board);
  // Printed mat: canvas top = far edge (+y), canvas right = +x, so the plane needs no rotation.
  const matGroup = new THREE.Group(); g.add(matGroup);
  g.userData.placeMat = d => placeMat(M, matGroup, d);
  placeMat(M, matGroup, MAT.sToEdge);
  const desk = mesh(new THREE.PlaneGeometry(240, 130), M.desk, false); at(desk, 0, 15, -BOARD.t - 0.01); g.add(desk);
  // Felt partition behind the desk; faces the arm (+Y) and is hidden by the app when the camera goes behind it.
  const wall = mesh(new THREE.PlaneGeometry(240, 90), M.wall, false); wall.rotation.set(Math.PI / 2, Math.PI, 0);
  wall.name = 'backdrop'; at(wall, 0, -24, 44); g.add(wall);

  const pi = buildPi(M); pi.rotation.z = Math.PI; at(pi, -4, -11, -BOARD.t); g.add(pi);
  // network and USB cables leaving toward the partition
  for (const [x, mat, r] of [[-0.8, M.piWhite, 0.32], [-4.85, M.wireBlack, 0.28]]) {
    const c = new THREE.CatmullRomCurve3([V(x, -8.0, 0.9), V(x + 0.3, -12, 1.2), V(x + 1.5, -18, 0.0), V(x + 4, -22, -0.7)]);
    g.add(mesh(new THREE.TubeGeometry(c, 40, r, 10, false), mat));
  }
  // bench supply beside the board (photos/setup-desk-overview.jpg)
  const psu = buildPSU(M); psu.rotation.z = 0.25; at(psu, -34, -10, -BOARD.t); g.add(psu);

  // HC-SR04 ultrasonic sensor on the desk behind the arm.
  const us = new THREE.Group();
  us.add(at(box(4.5, 2.0, 0.16, M.pcbBlue), 0, 0, 0.1));
  for (const x of [-1.3, 1.3]) { const c = cyl(0.8, 1.2, M.sensorSilver, 'z', 28); at(c, x, 0, 0.8); us.add(c); }
  us.rotation.z = 0.4; at(us, 15, -12, -BOARD.t + 0.05); g.add(us);
  return g;
}

/**
 * The printed mat (290 x 375 mm paper, field near edge at y = d). Where the paper reaches past
 * the board's back edge it folds down over the edge instead of floating in the air.
 */
export function placeMat(M, group, d) {
  group.clear();
  const PW = 29, PH = 37.5, y0 = d - 0.5, edge = -BOARD.axisToBackEdge;
  const strip = (ya, yb, va, vb) => {
    const geo = new THREE.PlaneGeometry(PW, yb - ya);
    const uv = geo.attributes.uv;
    for (let i = 0; i < uv.count; i++) uv.setY(i, uv.getY(i) > 0.5 ? vb : va);
    return mesh(geo, M.mat, false);
  };
  const flat0 = Math.max(y0, edge), vCut = (flat0 - y0) / PH;
  const top = strip(flat0, y0 + PH, vCut, 1); at(top, 0, (flat0 + y0 + PH) / 2, 0.012); top.name = 'mat'; group.add(top);
  if (y0 < edge) {
    const over = edge - y0;
    const flap = strip(0, over, 0, vCut);
    flap.rotation.x = Math.PI / 2; at(flap, 0, edge - 0.012, -over / 2 + 0.01); group.add(flap);   // hanging down the back edge
  }
  for (const x of [-PW / 2 + 0.5, PW / 2 - 0.5]) {
    const t = mesh(new THREE.PlaneGeometry(1.8, y0 + PH - flat0), M.tape, false); at(t, x, (flat0 + y0 + PH) / 2, 0.02); group.add(t);
  }
}

/** Raspberry Pi 4 Model B in the red base of the official case, no lid (photos/raspberry-pi-case.jpg). Origin: case bottom centre. */
export function buildPi(M) {
  const g = new THREE.Group();
  const pcbTex = canvasTexture(1024, 680, (c, w, h) => {
    c.fillStyle = '#1d7a3c'; c.fillRect(0, 0, w, h);
    c.strokeStyle = 'rgba(210,190,90,0.35)'; c.lineWidth = 2;               // traces
    for (let i = 0; i < 60; i++) { c.beginPath(); let x = rnd() * w, y = rnd() * h; c.moveTo(x, y); for (let k = 0; k < 4; k++) { x += (rnd() - 0.5) * 160; y += rnd() < 0.5 ? 0 : (rnd() - 0.5) * 160; c.lineTo(x, y); } c.stroke(); }
    c.fillStyle = '#f0f0f0'; c.font = 'bold 30px Arial'; c.fillText('Raspberry Pi 4 Model B', 300, 600); c.font = '22px Arial'; c.fillText('© Raspberry Pi 2018', 330, 635);
    c.font = '18px Arial'; for (const [t, x, y] of [['CAMERA', 600, 300], ['DISPLAY', 120, 520], ['GPIO', 120, 70], ['RUN', 900, 120], ['POE', 800, 90]]) c.fillText(t, x, y);
    for (const [x, y] of [[40, 40], [w - 40, 40], [40, h - 40], [w - 40, h - 40]]) { c.fillStyle = '#d8c060'; c.beginPath(); c.arc(x, y, 22, 0, 7); c.fill(); c.fillStyle = '#111'; c.beginPath(); c.arc(x, y, 12, 0, 7); c.fill(); }
  });
  const pcbMat = new THREE.MeshPhysicalMaterial({ map: pcbTex, roughness: 0.45, clearcoat: 0.5 });
  // case: red tray with rounded walls and the vent slot
  const tray = roundedRect(9.4, 6.6, 0.6); tray.holes.push(roundedRect(8.8, 6.0, 0.4));
  g.add(mesh(new THREE.ExtrudeGeometry(tray, { depth: 2.0, bevelEnabled: true, bevelSize: 0.05, bevelThickness: 0.05, bevelSegments: 2 }), M.piCase));
  g.add(at(box(9.4, 6.6, 0.2, M.piCase, 0.1), 0, 0, 0.1));
  const pcb = mesh(new THREE.BoxGeometry(8.5, 5.6, 0.16), [M.pcbGreen, M.pcbGreen, M.pcbGreen, M.pcbGreen, pcbMat, M.pcbGreen]); at(pcb, 0, 0, 1.0); g.add(pcb);
  // SoC, RAM, USB controller under heat sinks; Wi-Fi can
  for (const [x, y, w] of [[-1.2, 0.4, 1.4], [0.6, 0.6, 1.0], [-1.6, -1.4, 0.8]]) {
    g.add(at(box(w * 0.95, w * 0.95, 0.12, M.chip), x, y, 1.14));
    const hs = new THREE.Group();
    hs.add(at(box(w, w, 0.12, M.metalBox), 0, 0, 0));
    for (let i = 0; i < 5; i++) hs.add(at(box(w, 0.05, 0.38, M.metalBox), 0, -w / 2 + 0.08 + i * (w - 0.16) / 4, 0.24));
    at(hs, x, y, 1.26); g.add(hs);
  }
  g.add(at(box(1.1, 0.9, 0.18, M.sensorSilver, 0.03), 2.2, 1.3, 1.17));
  // ports along the short edge: Ethernet, 2 x USB 3 (blue), 2 x USB 2
  g.add(at(box(1.6, 2.1, 1.35, M.metalBox, 0.05), -3.2, -3.0, 1.75));
  g.add(at(box(1.2, 0.05, 0.9, M.chip), -3.2, -4.06, 1.75));
  for (const [x, blue] of [[-1.15, true], [0.85, false]]) {
    g.add(at(box(1.45, 2.1, 1.6, M.metalBox, 0.05), x, -3.0, 1.88));
    for (const z of [1.5, 2.25]) g.add(at(box(1.2, 0.05, 0.28, blue ? M.wireBlue : M.chip), x, -4.06, z));
  }
  // USB-C power, 2 x micro HDMI, audio on the long edge
  for (const [y, w] of [[2.4, 0.9], [0.9, 0.65], [-0.4, 0.65]]) g.add(at(box(0.7, w, 0.33, M.metalBox, 0.05), 4.05, y, 1.25));
  const jack = cyl(0.3, 0.7, M.chip, 'x', 16); at(jack, 4.05, -1.9, 1.4); g.add(jack);
  // 2 x 20 GPIO header with gold pins, ribbon connector on four of them
  g.add(at(box(5.1, 0.5, 0.25, M.chip), 0.4, 2.45, 1.21));
  for (let i = 0; i < 20; i++) for (const r of [0, 1]) g.add(at(box(0.064, 0.064, 0.6, M.gold), 0.4 - 2.41 + i * 0.254, 2.33 + r * 0.254, 1.5));
  g.add(at(box(1.05, 0.55, 0.85, M.chip, 0.04), -1.2, 2.45, 1.9));
  g.add(at(box(0.25, 0.4, 0.1, M.zip), -1.2, 2.45, 2.35));
  // camera and display FFC connectors, LEDs
  for (const [x, y, rz] of [[0.9, -0.8, 0], [-3.6, 0.6, Math.PI / 2]]) { const c = at(box(2.2, 0.35, 0.3, M.piWhite), x, y, 1.25); c.rotation.z = rz; g.add(c); }
  g.add(at(box(0.18, 0.12, 0.08, M.led), 4.0, 2.7, 1.12));
  g.add(at(box(0.18, 0.12, 0.08, new THREE.MeshStandardMaterial({ color: 0x24ff5a, emissive: 0x24ff5a, emissiveIntensity: 1.2 })), 3.75, 2.7, 1.12));
  // yellow tape over the jumper wires (photo)
  const tape = mesh(new THREE.CylinderGeometry(0.22, 0.22, 2.2, 12), new THREE.MeshStandardMaterial({ color: 0xe8d23a, roughness: 0.4 }));
  tape.rotation.z = Math.PI / 2 + 0.2; at(tape, -1.6, 2.0, 2.15); g.add(tape);
  return g;
}

/**
 * Bench power supply (the team's Eventek unit, photos/setup-desk-overview.jpg): vented grey
 * steel case, cream front panel with voltage and current read-outs, two knobs, banana sockets,
 * rocker switch and rubber feet. Origin: centre of the bottom, front facing +Y.
 */
export function buildPSU(M) {
  const g = new THREE.Group();
  const Wd = 13, D = 24, Hh = 15;
  const body = new THREE.MeshPhysicalMaterial({ color: 0x6a6e74, metalness: 0.55, roughness: 0.5, clearcoat: 0.2 });
  g.add(at(box(Wd, D, Hh, body, 0.25), 0, 0, Hh / 2 + 0.4));
  // vent slots on both sides and on top
  const slot = new THREE.MeshStandardMaterial({ color: 0x1b1c1f, roughness: 0.8 });
  for (const sx of [-1, 1]) for (let i = 0; i < 9; i++) for (let j = 0; j < 2; j++) g.add(at(box(0.05, 2.8, 0.35, slot), sx * (Wd / 2 + 0.01), -7 + j * 7, 5 + i * 0.9));
  for (let i = 0; i < 8; i++) g.add(at(box(6, 0.35, 0.05, slot), 0, -8 + i * 1.6, Hh + 0.41));
  for (const [x, z] of [[-5.8, 1.1], [5.8, 1.1], [-5.8, 14.4], [5.8, 14.4]]) screwAt(g, M, V(x, D / 2 - 0.4, z), V(0, 1, 0), 0.18);
  // front panel texture: displays and labels
  const panelTex = canvasTexture(640, 740, (c, w, h) => {
    c.fillStyle = '#e9e3d2'; c.fillRect(0, 0, w, h);
    c.fillStyle = '#4b4f55'; c.font = 'bold 34px Arial'; c.fillText('EVENTEK', 40, 70);
    c.font = '20px Arial'; c.fillText('DC POWER SUPPLY  KPS305D', 40, 100);
    const disp = (y, text, label) => {
      c.fillStyle = '#141414'; c.fillRect(40, y, 300, 110);
      c.fillStyle = '#ff3b2a'; c.shadowColor = '#ff3b2a'; c.shadowBlur = 18; c.font = 'bold 86px "Courier New", monospace'; c.fillText(text, 60, y + 88); c.shadowBlur = 0;
      c.fillStyle = '#4b4f55'; c.font = 'bold 22px Arial'; c.fillText(label, 350, y + 65);
    };
    disp(150, '05.2', 'V'); disp(300, '0.54', 'A');
    c.font = '18px Arial'; c.fillText('VOLTAGE', 450, 200); c.fillText('CURRENT', 450, 360);
    c.fillText('COARSE   FINE', 440, 520); c.fillText('+          -', 450, 690); c.fillText('POWER', 60, 690);
  });
  const panel = box(Wd - 0.4, 0.3, Hh - 0.4, M.psuFront, 0.1); at(panel, 0, D / 2 + 0.15, Hh / 2 + 0.4); g.add(panel);
  // printed face: texture right = -X (the viewer's right when facing the front), up = +Z
  const face = mesh(new THREE.PlaneGeometry(Wd - 0.6, Hh - 0.6), new THREE.MeshStandardMaterial({ map: panelTex, roughness: 0.55 }), false);
  orient(face, V(-1, 0, 0), V(0, 0, 1), V(0, 1, 0)); at(face, 0, D / 2 + 0.31, Hh / 2 + 0.4); g.add(face);
  // knobs with knurled skirts
  const knobMat = new THREE.MeshStandardMaterial({ color: 0xd9d5cc, roughness: 0.45 });
  for (const [x, z, r] of [[-3.7, 11.2, 0.95], [-3.7, 7.6, 0.95], [-2.3, 4.4, 0.6], [-4.9, 4.4, 0.6]]) {   // viewer's right = -X
    const k = cyl(r, 1.1, knobMat, 'y', 32); at(k, x, D / 2 + 0.85, z); g.add(k);
    const sk = cyl(r * 1.12, 0.3, knobMat, 'y', 24); at(sk, x, D / 2 + 0.45, z); g.add(sk);
    const ln = box(0.08, 0.05, r * 0.8, M.chip); at(ln, x, D / 2 + 1.42, z + r * 0.45); g.add(ln);
  }
  // banana sockets (red +, black -) and the rocker switch
  for (const [x, mat] of [[-2.4, M.cableRed], [-4.9, M.wireBlack]]) {
    const b = cyl(0.42, 0.9, mat, 'y', 20); at(b, x, D / 2 + 0.6, 1.9); g.add(b);
    const h = cyl(0.16, 0.95, M.chip, 'y', 12); at(h, x, D / 2 + 0.62, 1.9); g.add(h);
  }
  g.add(at(box(1.2, 0.5, 1.6, M.wireBlack, 0.1), 4.6, D / 2 + 0.4, 1.9));
  g.add(at(box(0.9, 0.2, 0.6, M.cableRed, 0.05), 4.6, D / 2 + 0.7, 2.2));
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const ft = cyl(0.8, 0.4, M.wireBlack, 'z', 16); at(ft, sx * 5, sy * 10, 0.2); g.add(ft); }
  return g;
}

/** The OAK camera on its clamp arm, at the fitted position, looking at `target`. */
export function buildCamera(M, pos, target) {
  const g = new THREE.Group();
  const head = new THREE.Group();
  head.add(box(9.1, 2.6, 2.2, M.camBody, 0.45));
  for (const x of [-3.75, 0, 3.75]) { const l = cyl(x === 0 ? 0.55 : 0.38, 0.25, M.glass, 'y', 24); at(l, x, 1.3, 0); head.add(l); }
  head.position.set(...pos);
  head.quaternion.setFromUnitVectors(V(0, 1, 0), V(...target).sub(V(...pos)).normalize());
  g.add(head);
  // articulated stand down to the desk
  const base = V(pos[0] + 4, pos[1] + 9, -BOARD.t), mid = V(pos[0] + 5, pos[1] + 6, pos[2] * 0.55);
  for (const [a, b] of [[base, mid], [mid, V(...pos).add(V(0, 1.8, -1.2))]]) {
    const len = a.distanceTo(b);
    const rod = mesh(new THREE.CylinderGeometry(0.5, 0.5, len, 16), M.camBody);
    rod.position.copy(a).add(b).multiplyScalar(0.5);
    rod.quaternion.setFromUnitVectors(V(0, 1, 0), b.clone().sub(a).normalize());
    g.add(rod);
    const j = mesh(new THREE.SphereGeometry(0.85, 20, 14), M.camBody); j.position.copy(b); g.add(j);
  }
  g.add(at(box(5, 6, 1.2, M.camBody, 0.2), base.x, base.y, base.z + 0.6));
  return g;
}

/** A wooden test cube with slightly rounded edges. */
export const woodCache = new Map();

export function buildCube(colour) {
  const hex = COLOURS[colour] ?? 0x999999;
  if (!woodCache.has(hex)) woodCache.set(hex, new THREE.MeshPhysicalMaterial({ map: woodTexture(hex), roughness: 0.62, clearcoat: 0.15, clearcoatRoughness: 0.6 }));
  const m = mesh(new RoundedBoxGeometry(CUBE, CUBE, CUBE, 3, 0.1), woodCache.get(hex));
  m.name = colour;
  return m;
}
