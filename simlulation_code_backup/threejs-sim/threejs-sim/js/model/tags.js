// AprilTags (36h11) as printed and stuck to the arm (config.js ARM_TAGS), and the second camera's body.

import * as THREE from 'three';
import { tagBits } from '../apriltag36h11.js';
import { canvasTexture } from './textures.js';
import { tagAxes } from '../armtrack.js';
import { V, mesh, at, orient, box, cyl } from './geometry.js';

const materials = new Map();

/** The printed tag: white margin, black border, 6 x 6 data cells (10 x 10 cells in all). */
function tagMaterial(id) {
  if (!materials.has(id)) {
    const bits = id == null ? new Array(36).fill(0) : tagBits(id);   // a tag being reprinted: black square only
    const map = canvasTexture(200, 200, (g, w) => {
      const c = w / 10;
      g.fillStyle = '#f4f4f0'; g.fillRect(0, 0, w, w);
      g.fillStyle = '#111'; g.fillRect(c, c, 8 * c, 8 * c);
      g.fillStyle = '#f4f4f0';
      bits.forEach((b, i) => { if (b) g.fillRect((2 + i % 6) * c, (2 + Math.floor(i / 6)) * c, c, c); });
    });
    map.magFilter = THREE.NearestFilter; map.wrapS = map.wrapT = THREE.ClampToEdgeWrapping;
    materials.set(id, new THREE.MeshStandardMaterial({ map, roughness: 0.8 }));
  }
  return materials.get(id);
}

/** A tag on its link: plane of the tag plus its white margin, 0,12 mm in front of the face it sits on. */
export function buildTag(tag) {
  const side = tag.size * 10 / 8;
  const m = mesh(new THREE.PlaneGeometry(side, side), tagMaterial(tag.id), false);
  const ax = tagAxes(tag), n = V(...ax.n), up = V(...ax.up), right = V(...ax.right);
  orient(m, right, up, n);
  m.position.set(...tag.centre).addScaledVector(n, 0.012);
  m.name = `tag ${tag.id}`;
  return m;
}

/** A clip-on webcam looking from pos at target (board frame, cm). The monitor it is clipped to is left out: it blocked the view. */
export function buildWebcam(M, pos, target) {
  const g = new THREE.Group();
  const head = new THREE.Group();
  head.add(box(9.5, 2.9, 2.4, M.camBody, 0.6));                      // body: X across, lens on +Y
  const lens = cyl(0.75, 0.3, M.glass, 'y', 24); at(lens, 0, 1.3, 0); head.add(lens);
  head.position.set(...pos);
  head.quaternion.setFromUnitVectors(V(0, 1, 0), V(...target).sub(V(...pos)).normalize());
  g.add(head);
  return g;
}
