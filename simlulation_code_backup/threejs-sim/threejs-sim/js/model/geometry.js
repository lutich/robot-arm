// Geometry helpers: meshes, placement, plates with holes, and merging static meshes per link.

import * as THREE from 'three';
import { RoundedBoxGeometry } from '../../vendor/RoundedBoxGeometry.js';
import { mergeGeometries } from '../../vendor/jsm/utils/BufferGeometryUtils.js';

export const V = (x, y, z) => new THREE.Vector3(x, y, z);

export const mesh = (geo, mat, cast = true) => { const m = new THREE.Mesh(geo, mat); m.castShadow = cast; m.receiveShadow = true; return m; };

export const at = (o, x, y, z) => { o.position.set(x, y, z); return o; };

/** Rotate o so its local X, Y, Z axes point along the given parent axes (must be right-handed). */
export function orient(o, x, y, z) {
  if (x.clone().cross(y).dot(z) < 0.99) throw new Error('orient: axes are not a right-handed basis');
  o.setRotationFromMatrix(new THREE.Matrix4().makeBasis(x, y, z)); return o;
}

/** Rotate o so its local Z axis points along d. */
export function aimZ(o, d) { o.quaternion.setFromUnitVectors(V(0, 0, 1), d.clone().normalize()); return o; }

export const box = (sx, sy, sz, mat, r = 0) => mesh(r > 0 ? new RoundedBoxGeometry(sx, sy, sz, 2, r) : new THREE.BoxGeometry(sx, sy, sz), mat);

export function cyl(r, h, mat, ax = 'z', seg = 28, r2 = r) {
  const m = mesh(new THREE.CylinderGeometry(r2, r, h, seg), mat);
  if (ax === 'z') m.rotation.x = Math.PI / 2;
  if (ax === 'x') m.rotation.z = -Math.PI / 2;
  return m;
}

export function roundedRect(w, h, r) {
  const s = new THREE.Shape(), x0 = -w / 2, y0 = -h / 2;
  r = Math.min(r, w / 2 - 1e-3, h / 2 - 1e-3);
  s.moveTo(x0 + r, y0); s.lineTo(x0 + w - r, y0); s.absarc(x0 + w - r, y0 + r, r, -Math.PI / 2, 0, false);
  s.lineTo(x0 + w, y0 + h - r); s.absarc(x0 + w - r, y0 + h - r, r, 0, Math.PI / 2, false);
  s.lineTo(x0 + r, y0 + h); s.absarc(x0 + r, y0 + h - r, r, Math.PI / 2, Math.PI, false);
  s.lineTo(x0, y0 + r); s.absarc(x0 + r, y0 + r, r, Math.PI, 1.5 * Math.PI, false);
  return s;
}

/** Smooth outline around a set of circles [[x, y, r]] (convex hull of points on them). */
export function hullShape(circles, n = 24) {
  const pts = [];
  for (const [x, y, r] of circles) for (let i = 0; i < n; i++) { const a = i / n * 2 * Math.PI; pts.push([x + r * Math.cos(a), y + r * Math.sin(a)]); }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of pts) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
  for (const p of pts.slice().reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  const s = new THREE.Shape(); s.moveTo(...hull[0]); for (const p of hull.slice(1)) s.lineTo(...p); s.closePath();
  return s;
}

export function addHoles(shape, holes) {
  for (const [hx, hy, hr] of holes) { const p = new THREE.Path(); p.absarc(hx, hy, hr, 0, Math.PI * 2, true); shape.holes.push(p); }
  return shape;
}

/** Extruded plate (outline in XY, thickness along Z, centred on z = 0) with a small edge bevel. */
export function plateFromShape(shape, t, mat, bevel = 0.035) {
  const g = new THREE.ExtrudeGeometry(shape, { depth: t - 2 * bevel, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 20 });
  g.translate(0, 0, -(t - 2 * bevel) / 2);
  return mesh(g, mat);
}

export const plate = (w, h, t, mat, holes = [], r = 0.3) => plateFromShape(addHoles(roundedRect(w, h, r), holes), t, mat);

/** Horn pattern of every kit bracket (Probots drawings): 8 mm bore, 4 x 3 mm holes on a 14 mm circle. */
export const hornHoles = (x, y, R = 0.4) => [[x, y, R], ...[0, 1, 2, 3].map(k => [x + 0.7 * Math.cos(k * Math.PI / 2), y + 0.7 * Math.sin(k * Math.PI / 2), 0.15])];

/** Merge all static meshes under `group` (not inside child groups marked as joints) per material. */
export function bake(group) {
  group.updateMatrixWorld(true);
  const inv = group.matrixWorld.clone().invert();
  const buckets = new Map();
  const victims = [];
  const visit = o => {
    for (const c of o.children) {
      if (c.userData.joint) continue;
      if (c.isMesh) {
        let geo = c.geometry.index ? c.geometry.toNonIndexed() : c.geometry.clone();
        for (const k of Object.keys(geo.attributes)) if (!['position', 'normal', 'uv'].includes(k)) geo.deleteAttribute(k);
        if (!geo.attributes.uv) geo.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(geo.attributes.position.count * 2), 2));
        geo.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, c.matrixWorld));
        const key = `${c.material.uuid}|${c.castShadow}`;
        if (!buckets.has(key)) buckets.set(key, { mat: c.material, cast: c.castShadow, geos: [] });
        buckets.get(key).geos.push(geo);
        victims.push(c);
      }
      visit(c);
    }
  };
  visit(group);
  for (const v of victims) v.parent.remove(v);
  for (const { mat, cast, geos } of buckets.values()) group.add(mesh(mergeGeometries(geos), mat, cast));
}

export const joint = g => { g.userData.joint = true; return g; };
