// Kit parts used all over the arm: screws, the MG996R servo, round horns, bracket plates, zip ties.

import * as THREE from 'three';
import { V, mesh, at, orient, aimZ, box, cyl, roundedRect, addHoles, plateFromShape, plate } from './geometry.js';

/** Phillips pan-head screw, head facing +Z, at the origin. */
export function screw(M, r = 0.17) {
  const g = new THREE.Group();
  const head = mesh(new THREE.CylinderGeometry(r * 0.92, r, 0.07, 16), M.screw); head.rotation.x = Math.PI / 2; at(head, 0, 0, 0.035); g.add(head);
  const dome = mesh(new THREE.SphereGeometry(r * 0.92, 16, 6, 0, Math.PI * 2, 0, Math.PI / 2), M.screw);
  dome.rotation.x = Math.PI / 2; dome.scale.set(1, 0.45, 1); at(dome, 0, 0, 0.07); g.add(dome);
  for (const rz of [0, Math.PI / 2]) { const s = box(r * 1.2, 0.035, 0.05, M.slot); s.rotation.z = rz; at(s, 0, 0, 0.13); g.add(s); }
  return g;
}

/** Screw on a parent at position p with the head facing direction d. */
export function screwAt(parent, M, p, d, r) { const s = screw(M, r); aimZ(s, d); s.position.copy(p); parent.add(s); return s; }

export function nut(M, s = 0.28) { const n = mesh(new THREE.CylinderGeometry(s, s, 0.22, 6), M.screw); n.rotation.x = Math.PI / 2; return n; }

/**
 * MG996R. Local frame: output shaft on +Z at the origin, body along +X (shaft 1,0 from the
 * near end), width along Y. Case top at z = -0.05, bottom at z = -3.72.
 */
export function servo(M, { label = true } = {}) {
  const g = new THREE.Group();
  const L = 4.07, W = 1.97, xc = L / 2 - 1.0;
  const top = box(L, W, 0.75, M.servoCap, 0.08); at(top, xc, 0, -0.43); g.add(top);
  const mid = box(L - 0.04, W - 0.04, 2.1, M.servo, 0.04); at(mid, xc, 0, -1.87); g.add(mid);
  const bot = box(L, W, 0.8, M.servoCap, 0.08); at(bot, xc, 0, -3.32); g.add(bot);
  // mounting ears with two holes each (drawn as dark discs) and screws holding them
  for (const sx of [-1, 1]) {
    const ear = plate(0.75, W, 0.25, M.servo, [[0.05, 0.5, 0.2], [0.05, -0.5, 0.2]], 0.12);
    at(ear, xc + sx * (L / 2 + 0.37), 0, -0.92); ear.scale.x = sx; g.add(ear);
    for (const sy of [-0.5, 0.5]) screwAt(g, M, V(xc + sx * (L / 2 + 0.42), sy, -0.8), V(0, 0, 1), 0.15);
  }
  // shaft boss, gear dome and splined output
  const boss = cyl(0.72, 0.22, M.servoCap, 'z', 32); at(boss, 0, 0, 0.06); g.add(boss);
  const dome = cyl(0.5, 0.12, M.servoCap, 'z', 24); at(dome, 1.0, 0, 0.0); g.add(dome);
  const spline = cyl(0.28, 0.42, M.spline, 'z', 18); at(spline, 0, 0, 0.32); g.add(spline);
  // four case screws on the bottom
  for (const [x, y] of [[-0.75, -0.7], [-0.75, 0.7], [2.8, -0.7], [2.8, 0.7]]) screwAt(g, M, V(x, y, -3.73), V(0, 0, -1), 0.11);
  // cable exit on the far end
  const exit = box(0.5, 0.9, 0.5, M.servo, 0.08); at(exit, L - 1.0 + 0.15, 0, -2.9); g.add(exit);
  const mats = [M.cableBrown, M.cableRed, M.cableOrange];
  [-0.12, 0, 0.12].forEach((y, i) => { const w = cyl(0.06, 0.9, mats[i], 'x', 8); at(w, L - 1.0 + 0.7, y, -2.9); g.add(w); });
  if (label) {
    const lab = mesh(new THREE.PlaneGeometry(L - 0.4, 2.0), M.servoLabel, false);
    orient(lab, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(lab, xc, -W / 2 - 0.004, -1.9); g.add(lab);
  }
  return g;
}

/** Round aluminium servo horn on +Z with its screw circle and centre screw. */
export function horn(M, r = 1.0, mat = M.horn) {
  // Round 25T aluminium horn (Probots drawing): 20 mm disc, 2 mm thick, 9 mm boss, 4,5 mm overall,
  // 4 x M3 on a 14 mm circle, so it matches the brackets' horn pattern.
  const g = new THREE.Group();
  const holes = [0, 1, 2, 3].map(k => [0.7 * Math.cos(k * Math.PI / 2), 0.7 * Math.sin(k * Math.PI / 2), 0.15]);
  const d = plateFromShape(addHoles(roundedRect(2 * r, 2 * r, r - 0.001), holes), 0.2, mat, 0.03); at(d, 0, 0, 0.35); g.add(d);
  const hub = cyl(0.45, 0.25, mat, 'z', 24); at(hub, 0, 0, 0.125); g.add(hub);
  for (let k = 0; k < 4; k++) screwAt(g, M, V(0.7 * Math.cos(k * Math.PI / 2), 0.7 * Math.sin(k * Math.PI / 2), 0.45), V(0, 0, 1), 0.14);
  screwAt(g, M, V(0, 0, 0.45), V(0, 0, 1), 0.2);
  return g;
}

/** Bracket side plate in the YZ plane (normal X): width along Y, from z0 to z1, round ends. */
export function sidePlate(M, x, z0, z1, width = 2.5, holes = [], mat = M.anodised) {
  const len = z1 - z0;
  const p = plateFromShape(addHoles(roundedRect(width, len, Math.min(width / 2, 1.25)), holes.map(([u, v, r]) => [u, v - len / 2, r])), 0.2, mat);
  orient(p, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0));
  return at(p, x, 0, (z0 + z1) / 2);
}

/** Bend between two bracket plates: a quarter-round along the given axis. */
export function bend(M, len, ax = 'y') { return cyl(0.2, len, M.anodised, ax, 10); }

/** White nylon zip tie around a tube of radius r (axis Z), with its tail. */
export function zipTie(M, r) {
  const g = new THREE.Group();
  const ring = mesh(new THREE.TorusGeometry(r + 0.05, 0.05, 6, 28), M.zip); ring.scale.z = 2.2; g.add(ring);
  const head = box(0.3, 0.25, 0.3, M.zip, 0.04); at(head, r + 0.15, 0, 0); g.add(head);
  const tail = box(2.6, 0.05, 0.22, M.zip); at(tail, r + 1.5, 0.12, 0); tail.rotation.z = 0.35; g.add(tail);
  return g;
}
