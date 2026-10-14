// Overlay that shows which tags sit on which rigid part of the arm and what moves that part
// (config.js ARM_TAGS, ARM_PARTS): tag outlines in the part's colour, a line joining the tags of one part
// (they act as one object), a line to the joint axis that moves the part, the axis itself, and labels.

import * as THREE from 'three';
import { ARM_TAGS, ARM_PARTS } from './config.js';
import { fk, point } from './kinematics.js';
import { tagPoints } from './armtrack.js';

export function label(text, colour) {
  const c = document.createElement('canvas'); c.width = 1024; c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 30px system-ui';
  const w = Math.min(1020, g.measureText(text).width + 24);
  g.fillStyle = 'rgba(20,24,30,0.78)'; g.beginPath(); g.roundRect(0, 6, w, 52, 10); g.fill();
  g.fillStyle = '#' + new THREE.Color(colour).getHexString(); g.fillRect(0, 6, 8, 52);
  g.fillStyle = '#fff'; g.fillText(text, 16, 42);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false, transparent: true }));
  s.scale.set(0.9 * 1024 / 64, 0.9, 1); s.center.set(0, 0.5); s.renderOrder = 10;   // 0,9 cm tall text band
  return s;
}

/** Builds the overlay group; update(angles) moves it with the arm. */
export function buildTagOverlay() {
  const group = new THREE.Group();
  group.name = 'tag-overlay';
  const lineMat = c => new THREE.LineBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.95 });
  const items = ARM_TAGS.map(t => {
    const colour = ARM_PARTS[t.link].colour;
    const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([0, 1, 2, 3].map(() => new THREE.Vector3())), lineMat(colour));
    const toAxis = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lineMat(colour));
    toAxis.material.opacity = 0.5;
    const txt = label(`tag ${t.id ?? '(reprint)'}: ${t.on}`, colour);
    for (const o of [outline, toAxis, txt]) { o.renderOrder = 10; o.frustumCulled = false; group.add(o); }
    return { t, outline, toAxis, txt };
  });
  const parts = Object.entries(ARM_PARTS).map(([link, p]) => {
    const join = new THREE.Line(new THREE.BufferGeometry().setFromPoints(ARM_TAGS.filter(t => t.link === link).map(() => new THREE.Vector3())), lineMat(p.colour));
    join.material.opacity = 0.7;
    const axis = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lineMat(p.colour));
    const txt = label(`${p.name} — moved by ${p.joints}, turns about ${p.last}`, p.colour);
    for (const o of [join, axis, txt]) { o.renderOrder = 10; o.frustumCulled = false; group.add(o); }
    return { link, p, join, axis, txt };
  });

  function update(angles) {
    const f = fk(angles), V = a => new THREE.Vector3(...a);
    const axisPoint = link => point(f[ARM_PARTS[link].axis[0]], ARM_PARTS[link].axis[1]);
    for (const it of items) {
      const tp = tagPoints(it.t, angles, f);
      it.outline.geometry.setFromPoints(tp.corners.map(V));
      it.toAxis.geometry.setFromPoints([V(tp.centre), V(axisPoint(it.t.link))]);
      const n = V(point(f[it.t.link], it.t.centre.map((c, i) => c + 2.5 * it.t.normal[i])));
      it.txt.position.copy(n);
    }
    for (const pt of parts) {
      const tags = ARM_TAGS.filter(t => t.link === pt.link).map(t => V(tagPoints(t, angles, f).centre));
      pt.join.geometry.setFromPoints(tags);
      // the joint axis that moves the part: S4 (along the elbow frame's X), S5 (along the wrist frame's Z)
      const fr = f[pt.p.axis[0]], c = pt.p.axis[1];
      const dir = pt.link === 'wrist' ? [1, 0, 0] : [0, 0, 1], h = pt.link === 'wrist' ? 4 : 2.5;
      const a = point(fr, c.map((v, i) => v - h * dir[i])), b = point(fr, c.map((v, i) => v + h * dir[i]));
      pt.axis.geometry.setFromPoints([V(a), V(b)]);
      pt.txt.position.copy(V(b)).add(new THREE.Vector3(0, 0, 2));
    }
  }
  return { group, update };
}

/**
 * The tags the cameras see right now (live tracking), on the arm model: a filled square over each seen tag,
 * green when two cameras see it, amber for one, with a label "106 · OAK + Webcam"; tags not seen
 * get a thin grey outline. update(angles, seen) with seen = { [id]: [{ cam, err }] }.
 */
export function buildSeenOverlay() {
  const group = new THREE.Group();
  group.name = 'seen-tags';
  const GREEN = 0x16a34a, AMBER = 0xd97706, GREY = 0x8a94a3;
  const items = ARM_TAGS.filter(t => t.id != null).map(t => {
    const fill = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: GREEN, transparent: true, opacity: 0.35, depthTest: false, side: THREE.DoubleSide }));
    fill.geometry.setIndex([0, 1, 2, 0, 2, 3]);
    fill.geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Array(12).fill(0), 3));
    const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([0, 1, 2, 3].map(() => new THREE.Vector3())),
      new THREE.LineBasicMaterial({ color: GREY, depthTest: false, transparent: true, opacity: 0.9 }));
    for (const o of [fill, outline]) { o.renderOrder = 11; o.frustumCulled = false; group.add(o); }
    return { t, fill, outline, txt: null, key: '' };
  });
  function update(angles, seen = {}) {
    const f = fk(angles), V = a => new THREE.Vector3(...a);
    for (const it of items) {
      const tp = tagPoints(it.t, angles, f), list = seen[it.t.id] || [];
      // a little larger than the black square, so it frames the printed tag
      const c = V(tp.centre), corners = tp.corners.map(p => V(p).sub(c).multiplyScalar(1.3).add(c));
      it.outline.geometry.setFromPoints(corners);
      it.fill.geometry.attributes.position.copyArray(corners.flatMap(p => [p.x, p.y, p.z]));
      it.fill.geometry.attributes.position.needsUpdate = true;
      const colour = list.length >= 2 ? GREEN : list.length ? AMBER : GREY;
      it.fill.visible = list.length > 0;
      it.fill.material.color.setHex(colour); it.outline.material.color.setHex(colour);
      // the cameras only (the errors are in the tracking box): a label is a texture, rebuilt only when this text changes
      const key = list.length ? `${it.t.id} · ${list.map(s => s.cam).join(' + ')}` : '';
      if (key !== it.key) {                                   // labels are textures: rebuilt only when the text changes
        if (it.txt) { group.remove(it.txt); it.txt.material.map.dispose(); it.txt.material.dispose(); it.txt = null; }
        if (key) { it.txt = label(key, colour); it.txt.scale.multiplyScalar(0.8); it.txt.renderOrder = 12; it.txt.frustumCulled = false; group.add(it.txt); }
        it.key = key;
      }
      if (it.txt) it.txt.position.copy(V(point(f[it.t.link], it.t.centre.map((v, i) => v + 2.2 * it.t.normal[i]))));
    }
  }
  return { group, update };
}
