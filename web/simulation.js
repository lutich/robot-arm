'use strict';
(function () {
// 3D simulation viewer for the control API. Pure Canvas 2D software rendering; no libraries.
// Geometry, count-to-angle mapping and block physics are an assumed visual model, not calibration.

const MODEL = {
  degPerCount: 180 / 570,            // Nominal servo travel over the historical 90–660 count span.
  reference: {0: 320, 1: 427, 2: 403, 3: 220, 4: 480, 5: 650},   // Reference HOME counts the angles below describe.
  joints: {
    0: {angle: 0, sign: -1, label: 'base yaw'},         // + counts turn clockwise seen from above, as the page's arrows say.
    1: {angle: 30, sign: -1, label: 'shoulder pitch'},  // From vertical; + leans forward (+Y).
    2: {angle: 60, sign: 1, label: 'elbow bend'},       // 0 continues the upper arm; + bends down.
    3: {angle: 30, sign: -1, label: 'wrist flexion'},   // 0 continues the forearm; + counts (page arrow ↑) raise the gripper.
    4: {angle: 0, sign: -1, label: 'wrist roll'},       // + counts turn clockwise seen from the gripper towards the arm.
  },
  gripper: {open: 490, closed: 660, maxOpening: 60},   // Opening in mm between the fingers at `open`.
  links: {shoulderHeight: 60, upperArm: 105, foreArm: 100, wrist: 50, finger: 35, palm: 12},
  fingerDepth: 12, fingerThickness: 6, palmWidth: 46,  // Collision boxes of the gripper, mm.
  table: {size: 340, centreY: 120},
  blocks: [
    {name: 'red', size: 30, x: -70, y: 170, color: '#d44343'},
    {name: 'green', size: 30, x: 0, y: 200, color: '#16836e'},
    {name: 'blue', size: 30, x: 70, y: 170, color: '#3c6fd1'},
    {name: 'yellow', size: 24, x: 0, y: 130, color: '#e0a62a'},
  ],
};

const $ = id => document.getElementById(id);
let state = null;
let visual = null;          // Smoothed counts drawn by the renderer.
let lastFrame = null;
let world = {blocks: [], held: null, heldOffset: null, between: null, lastOpening: null, collision: null, tip: [0, 0, 0]};
const camera = {yaw: 35, pitch: 28, dist: 620, target: [0, 120, 70]};

// ---- Small matrix helpers (row-major 4x4; mul(a, b) applies b first, then a) ----
const identity = () => [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
function mul(a, b) {
  const m = new Array(16);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++)
    m[r*4+c] = a[r*4]*b[c] + a[r*4+1]*b[4+c] + a[r*4+2]*b[8+c] + a[r*4+3]*b[12+c];
  return m;
}
const chain = (...ms) => ms.reduce(mul);
const translate = (x, y, z) => [1,0,0,x, 0,1,0,y, 0,0,1,z, 0,0,0,1];
function rotX(deg) { const c = Math.cos(deg*Math.PI/180), s = Math.sin(deg*Math.PI/180); return [1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1]; }
function rotY(deg) { const c = Math.cos(deg*Math.PI/180), s = Math.sin(deg*Math.PI/180); return [c,0,s,0, 0,1,0,0, -s,0,c,0, 0,0,0,1]; }
function rotZ(deg) { const c = Math.cos(deg*Math.PI/180), s = Math.sin(deg*Math.PI/180); return [c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1]; }
const pitch = deg => rotX(-deg);   // Positive pitch leans a +Z link towards +Y (forward).
const apply = (m, p) => [m[0]*p[0]+m[1]*p[1]+m[2]*p[2]+m[3], m[4]*p[0]+m[5]*p[1]+m[6]*p[2]+m[7], m[8]*p[0]+m[9]*p[1]+m[10]*p[2]+m[11]];
const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const dot = (a, b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const norm = a => { const l = Math.hypot(...a) || 1; return [a[0]/l, a[1]/l, a[2]/l]; };
const dist = (a, b) => Math.hypot(...sub(a, b));
function invert(m) {   // Inverse of a rotation+translation matrix: transpose the rotation, negate the moved origin.
  const r = [m[0],m[4],m[8], m[1],m[5],m[9], m[2],m[6],m[10]], t = [m[3], m[7], m[11]];
  return [r[0],r[1],r[2],-(r[0]*t[0]+r[1]*t[1]+r[2]*t[2]), r[3],r[4],r[5],-(r[3]*t[0]+r[4]*t[1]+r[5]*t[2]),
          r[6],r[7],r[8],-(r[6]*t[0]+r[7]*t[1]+r[8]*t[2]), 0,0,0,1];
}
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// ---- Meshes: faces wound counter-clockwise when seen from outside ----
function box(cx, cy, cz, sx, sy, sz, color) {
  const x = sx/2, y = sy/2, z = sz/2;
  const v = [[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]].map(p => [p[0]+cx, p[1]+cy, p[2]+cz]);
  const faces = [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]];
  return {verts: v, faces, color};
}
function cylinder(cx, cy, z0, z1, r, color, n = 18) {
  const verts = [], faces = [];
  for (let i = 0; i < n; i++) {
    const a = i/n*Math.PI*2;
    verts.push([cx + r*Math.cos(a), cy + r*Math.sin(a), z0], [cx + r*Math.cos(a), cy + r*Math.sin(a), z1]);
  }
  for (let i = 0; i < n; i++) {
    const j = (i+1) % n;
    faces.push([2*i, 2*j, 2*j+1, 2*i+1]);
  }
  faces.push(Array.from({length: n}, (_, i) => 2*(n-1-i)));   // bottom
  faces.push(Array.from({length: n}, (_, i) => 2*i+1));       // top
  return {verts, faces, color};
}
// Cylinder along the local X axis (joint pins).
function pin(cx, cy, cz, length, r, color) {
  const c = cylinder(0, 0, -length/2, length/2, r, color, 14);
  const m = mul(translate(cx, cy, cz), rotY(90));
  return {verts: c.verts.map(p => apply(m, p)), faces: c.faces, color};
}

// ---- Kinematics ----
function angle(channel, count) {
  const j = MODEL.joints[channel];
  return j.angle + j.sign * (count - MODEL.reference[channel]) * MODEL.degPerCount;
}
function openingMm(count) {
  const g = MODEL.gripper, t = (g.closed - count) / (g.closed - g.open);
  return Math.max(0, Math.min(1, t)) * g.maxOpening;
}
function frames(counts) {
  const L = MODEL.links;
  const m0 = rotZ(angle(0, counts[0]));
  const m1 = chain(m0, translate(0, 0, L.shoulderHeight), pitch(angle(1, counts[1])));
  const m2 = chain(m1, translate(0, 0, L.upperArm), pitch(angle(2, counts[2])));
  const m3 = chain(m2, translate(0, 0, L.foreArm), pitch(angle(3, counts[3])));
  const m4 = chain(m3, translate(0, 0, L.wrist), rotZ(angle(4, counts[4])));
  return {m0, m1, m2, m3, m4, opening: openingMm(counts[5])};
}
function graspPoint(f) {
  const L = MODEL.links;
  return apply(f.m4, [0, 0, L.palm + L.finger*0.6]);
}

function armMeshes(f, dim, heldSize = 0) {
  const L = MODEL.links;
  const tone = dim ? ['#b8c0c8', '#c9d0d6', '#a9b2bb'] : ['#3d4f60', '#6d7f90', '#2c3b4a'];
  const parts = [];
  const place = (mesh, m) => parts.push({verts: mesh.verts.map(p => apply(m, p)), faces: mesh.faces, color: mesh.color});
  place(cylinder(0, 0, 0, 12, 48, tone[1]), identity());
  place(cylinder(0, 0, 12, L.shoulderHeight - 14, 30, tone[0]), f.m0);
  place(box(0, 0, L.shoulderHeight - 7, 44, 30, 14, tone[0]), f.m0);
  place(pin(0, 0, 0, 52, 11, tone[2]), f.m1);
  place(box(0, 0, L.upperArm/2, 24, 22, L.upperArm, tone[1]), f.m1);
  place(pin(0, 0, 0, 44, 9, tone[2]), f.m2);
  place(box(0, 0, L.foreArm/2, 20, 18, L.foreArm, tone[1]), f.m2);
  place(pin(0, 0, 0, 36, 8, tone[2]), f.m3);
  place(box(0, 0, L.wrist/2, 18, 16, L.wrist, tone[1]), f.m3);
  place(box(0, 0, L.palm/2, MODEL.palmWidth, 16, L.palm, tone[0]), f.m4);
  const gap = Math.max(f.opening, heldSize)/2 + MODEL.fingerThickness/2;
  for (const side of [-1, 1])
    place(box(side*gap, 0, L.palm + L.finger/2, MODEL.fingerThickness, MODEL.fingerDepth, L.finger, dim ? '#aeb6be' : '#1f2b37'), f.m4);
  return {parts, frames: f};
}

// ---- Blocks: contact-based grasping, pushing and table/stack support ----
function resetBlocks() {
  world.blocks = MODEL.blocks.map(b => ({...b, z: b.size/2}));
  world.held = null;
  world.heldOffset = null;
  world.between = null;
  world.lastOpening = null;
  world.collision = null;
  renderBlocks();
}
const footprintsOverlap = (a, b) => Math.abs(a.x - b.x) < (a.size + b.size)/2 - 0.5 && Math.abs(a.y - b.y) < (a.size + b.size)/2 - 0.5;
// Height of the surface under a block's centre: the table, or the top of a block it overlaps that is not above it.
function supportHeight(block) {
  let top = 0;
  for (const other of world.blocks) {
    if (other !== block && other !== world.held && footprintsOverlap(other, block) && other.z <= block.z + 0.5)
      top = Math.max(top, other.z + other.size/2);
  }
  return top;
}
// Gripper collision boxes in the gripper frame: [xmin, xmax, ymin, ymax, zmin, zmax].
function gripperBoxes(opening) {
  const L = MODEL.links, half = MODEL.fingerThickness, g = opening/2, d = MODEL.fingerDepth/2;
  return [[-MODEL.palmWidth/2, MODEL.palmWidth/2, -8, 8, 0, L.palm],
          [-g - half, -g, -d, d, L.palm, L.palm + L.finger],
          [g, g + half, -d, d, L.palm, L.palm + L.finger]];
}
// Treat a block as a sphere for contact with the gripper boxes; returns the push-out vector in the gripper frame or null.
function penetration(centre, radius, boxBounds) {
  const q = [clamp(centre[0], boxBounds[0], boxBounds[1]), clamp(centre[1], boxBounds[2], boxBounds[3]), clamp(centre[2], boxBounds[4], boxBounds[5])];
  const d = sub(centre, q), len = Math.hypot(...d);
  if (len >= radius || len < 1e-6) return null;
  return d.map(v => v/len*(radius - len));
}
function updateWorld(f) {
  const L = MODEL.links, opening = f.opening, toLocal = invert(f.m4);
  const origin = apply(f.m4, [0, 0, 0]);
  const toWorldDir = v => sub(apply(f.m4, v), origin);
  world.tip = graspPoint(f);
  world.collision = null;
  // The gripper itself cannot pass through the table.
  const gap = Math.max(opening, world.held?.size ?? 0)/2 + MODEL.fingerThickness;
  const lowest = Math.min(...[[-gap, 0, L.palm + L.finger], [gap, 0, L.palm + L.finger], [-MODEL.palmWidth/2, 0, 0], [MODEL.palmWidth/2, 0, 0]]
    .map(p => apply(f.m4, p)[2]));
  if (lowest < 0) world.collision = 'The gripper is below the table surface';

  if (world.held) {
    const b = world.held, target = apply(f.m4, world.heldOffset);
    b.x = target[0]; b.y = target[1];
    b.z = Math.max(target[2], supportHeight(b) + b.size/2);   // The table or a block underneath carries it.
    if (opening > b.size + 2) {
      world.held = null;
      world.heldOffset = null;
      b.z = supportHeight(b) + b.size/2;
      status('Released the ' + b.name + ' block.');
      renderBlocks();
    }
  } else {
    // Grasp: a block that was between the open fingers is held once they close onto its faces.
    const closing = world.lastOpening !== null && opening < world.lastOpening;
    const candidate = world.between;
    if (closing && candidate && opening <= candidate.size + 1) {
      const local = apply(toLocal, [candidate.x, candidate.y, candidate.z]);
      world.held = candidate;
      world.heldOffset = [0, local[1], local[2]];
      status('Holding the ' + candidate.name + ' block.');
      renderBlocks();
    }
  }
  // Which block sits between the fingers now, and which blocks the gripper pushes.
  world.between = null;
  let moved = false;
  for (const b of world.blocks) {
    if (b === world.held) continue;
    const c = apply(toLocal, [b.x, b.y, b.z]), r = b.size/2;
    const inReach = Math.abs(c[1]) <= MODEL.fingerDepth/2 + r && c[2] + r > L.palm + 2 && c[2] - r < L.palm + L.finger;
    if (inReach && Math.abs(c[0]) + r <= opening/2 + 3 && (!world.between || Math.abs(c[0]) < Math.abs(apply(toLocal, [world.between.x, world.between.y, world.between.z])[0])))
      world.between = b;
    for (const bounds of gripperBoxes(opening)) {
      const push = penetration(c, r, bounds);
      if (!push) continue;
      const w = toWorldDir(push);
      if (Math.hypot(w[0], w[1]) < 0.3*Math.abs(w[2])) { world.collision = 'The gripper is pressing on the ' + b.name + ' block'; continue; }
      b.x += w[0]; b.y += w[1];
      moved = true;
    }
  }
  // Pushed blocks shove their neighbours aside instead of overlapping them.
  for (const a of world.blocks) for (const b of world.blocks) {
    if (a === b || a === world.held || b === world.held || !footprintsOverlap(a, b) || Math.abs(a.z - b.z) >= (a.size + b.size)/2 - 0.5) continue;
    const dx = b.x - a.x, dy = b.y - a.y, ox = (a.size + b.size)/2 - Math.abs(dx), oy = (a.size + b.size)/2 - Math.abs(dy);
    if (ox < oy) b.x += Math.sign(dx || 1)*ox; else b.y += Math.sign(dy || 1)*oy;
    moved = true;
  }
  for (const b of world.blocks) if (b !== world.held) b.z = supportHeight(b) + b.size/2;
  if (moved) renderBlocks();
  world.lastOpening = opening;
}
function renderBlocks() {
  const list = $('blocks');
  list.innerHTML = '';
  for (const b of world.blocks) {
    const li = document.createElement('li');
    const swatch = document.createElement('span');
    swatch.className = 'swatch';
    swatch.style.background = b.color;
    const text = document.createElement('span');
    text.textContent = b.name + ' · ' + b.x.toFixed(0) + ', ' + b.y.toFixed(0) + ', ' + b.z.toFixed(0) + ' mm' + (world.held === b ? ' · held' : '');
    li.append(swatch, text);
    list.append(li);
  }
}

// ---- Rendering ----
function project(p, view, f, cx, cy) {
  const c = apply(view, p);
  const depth = c[1] + camera.dist;
  if (depth < 20) return null;
  const s = f/depth;
  return [cx + c[0]*s, cy - c[2]*s, depth];
}
function shade(hex, intensity) {
  const n = parseInt(hex.slice(1), 16);
  const ch = v => Math.round(Math.max(0, Math.min(255, v*intensity)));
  return 'rgb(' + ch(n >> 16) + ',' + ch((n >> 8) & 255) + ',' + ch(n & 255) + ')';
}
function draw() {
  const canvas = $('scene'), ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width*dpr) || canvas.height !== Math.round(rect.height*dpr)) {
    canvas.width = Math.round(rect.width*dpr);
    canvas.height = Math.round(rect.height*dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = rect.width, h = rect.height, cx = w/2, cy = h/2, f = Math.min(w, h)*1.35;
  ctx.fillStyle = '#eef2f5';
  ctx.fillRect(0, 0, w, h);
  const t = camera.target;
  const view = chain(rotX(-camera.pitch), rotZ(-camera.yaw), translate(-t[0], -t[1], -t[2]));
  const eye = apply(chain(translate(t[0], t[1], t[2]), rotZ(camera.yaw), rotX(camera.pitch)), [0, -camera.dist, 0]);
  const light = norm([-0.45, -0.55, 0.75]);

  const counts = visual || parkCounts(), kin = frames(counts);
  if (visual) updateWorld(kin); else world.tip = graspPoint(kin);
  const arm = armMeshes(kin, !visual, world.held?.size ?? 0);
  const T = MODEL.table;
  const ground = [box(0, T.centreY, -6, T.size, T.size, 12, '#d9dfe5')];
  const scene = [...arm.parts, ...world.blocks.map(b => box(b.x, b.y, b.z, b.size, b.size, b.size, b.color))];
  const facesOf = meshes => {
    const faces = [];
    for (const mesh of meshes) {
      const pts = mesh.verts.map(p => project(p, view, f, cx, cy));
      for (const face of mesh.faces) {
        if (face.some(i => !pts[i])) continue;
        const a = mesh.verts[face[0]], b = mesh.verts[face[1]], c = mesh.verts[face[2]];
        const n = norm(cross(sub(b, a), sub(c, a)));
        if (dot(n, sub(eye, a)) <= 0) continue;
        const depth = face.reduce((s, i) => s + pts[i][2], 0)/face.length;
        faces.push({pts: face.map(i => pts[i]), depth, color: shade(mesh.color, 0.55 + 0.45*Math.max(0, dot(n, light)))});
      }
    }
    return faces.sort((a, b) => b.depth - a.depth);
  };
  const paint = faces => {
    for (const face of faces) {
      ctx.beginPath();
      ctx.moveTo(face.pts[0][0], face.pts[0][1]);
      for (const p of face.pts.slice(1)) ctx.lineTo(p[0], p[1]);
      ctx.closePath();
      ctx.fillStyle = face.color;
      ctx.strokeStyle = face.color;
      ctx.lineWidth = 0.8;
      ctx.fill();
      ctx.stroke();
    }
  };
  paint(facesOf(ground));
  // Table grid on the slab, under everything above it.
  ctx.strokeStyle = '#c5ccd3';
  ctx.lineWidth = 1;
  const half = T.size/2, y0 = T.centreY - half, y1 = T.centreY + half;
  for (let g = -half; g <= half; g += 50) {
    const a = project([g, y0, 0.3], view, f, cx, cy), b = project([g, y1, 0.3], view, f, cx, cy);
    const c = project([-half, T.centreY + g, 0.3], view, f, cx, cy), d = project([half, T.centreY + g, 0.3], view, f, cx, cy);
    if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
    if (c && d) { ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke(); }
  }
  paint(facesOf(scene));
  // Axes gizmo.
  const origin = project([0, 0, 0], view, f, cx, cy);
  for (const [axis, color] of [[[60, 0, 0], '#d44343'], [[0, 60, 0], '#16836e'], [[0, 0, 60], '#3c6fd1']]) {
    const end = project(axis, view, f, cx, cy);
    if (!origin || !end) continue;
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(origin[0], origin[1]); ctx.lineTo(end[0], end[1]); ctx.stroke();
  }
  const hud = $('hud');
  hud.textContent = 'Grasp point ' + world.tip.map(v => v.toFixed(0)).join(', ') + ' mm · opening ' + arm.frames.opening.toFixed(0) + ' mm' +
    (world.held ? ' · holding ' + world.held.name : world.between ? ' · ' + world.between.name + ' between the fingers: close S6 (+) to grasp' : '') +
    (world.collision ? ' · ' + world.collision : '') + (visual ? '' : ' · no commands: drawn at PARK');
  hud.classList.toggle('warn', Boolean(world.collision));
}

function parkCounts() {
  const counts = {};
  for (const j of state?.joints || []) counts[j.channel] = j.park;
  if (!state) for (const c of Object.keys(MODEL.reference)) counts[c] = MODEL.reference[c];
  return counts;
}
function animate(now) {
  const dt = lastFrame ? Math.min(0.1, (now - lastFrame)/1000) : 0;
  lastFrame = now;
  if (state) {
    const known = Object.values(state.commands).every(v => v !== null);
    if (!known) visual = null;
    else {
      if (!visual) visual = {...state.commands};
      const rate = Math.max(80, 4*(state.speed || 20));
      for (const c of Object.keys(state.commands)) {
        const diff = state.commands[c] - visual[c];
        visual[c] += Math.sign(diff)*Math.min(Math.abs(diff), rate*dt);
      }
    }
  }
  draw();
  requestAnimationFrame(animate);
}

// ---- Camera interaction ----
function cameraControls() {
  const canvas = $('scene');
  let drag = null;
  canvas.addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener('pointermove', e => {
    if (!drag) return;
    camera.yaw += (e.clientX - drag[0])*0.4;
    camera.pitch = Math.max(5, Math.min(85, camera.pitch + (e.clientY - drag[1])*0.3));
    drag = [e.clientX, e.clientY];
  });
  canvas.addEventListener('pointerup', () => { drag = null; });
  canvas.addEventListener('pointercancel', () => { drag = null; });
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    camera.dist = Math.max(250, Math.min(1400, camera.dist*(1 + Math.sign(e.deltaY)*0.08)));
  }, {passive: false});
  const views = {front: [0, 20], side: [90, 15], top: [0, 85], iso: [35, 28]};
  for (const button of document.querySelectorAll('[data-view]')) button.addEventListener('click', () => {
    [camera.yaw, camera.pitch] = views[button.dataset.view];
  });
}

// ---- Host integration ----
function status(message) {
  const line = $('sim-status');
  if (line) line.textContent = message;
}
function connection(message, error) {
  const line = $('sim-connection');
  if (!line) return;
  line.textContent = message;
  line.classList.toggle('error', error);
}
// The manual-control page feeds its polled state here; null means the connection was lost.
function update(next) {
  state = next;
  if (!next) { connection('Connection lost · the app is not answering.', true); return; }
  connection((next.mode === 'preview' ? 'Preview · the simulation is the only arm' : 'HARDWARE session · the real arm receives the same commands') +
    ' · geometry is assumed, not calibrated', false);
}
async function pollSelf() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    update(response.ok ? await response.json() : null);
  } catch (error) {
    update(null);
  }
}

function init() {
  resetBlocks();
  cameraControls();
  $('reset-blocks').addEventListener('click', () => { resetBlocks(); status('Blocks reset.'); });
  if (document.body.dataset.simulation === 'standalone') {
    pollSelf();
    setInterval(pollSelf, 200);
  }
  requestAnimationFrame(animate);
}

// Host hook for the control page, plus a read-only snapshot for the console and tests.
window.simulation = {
  model: MODEL,
  update, angle, openingMm, frames, graspPoint, resetBlocks,
  step: counts => { const f = frames(counts); updateWorld(f); return world.tip; },   // Apply a pose to the block world without the API.
  world: () => ({tip: [...world.tip], held: world.held?.name ?? null,
    blocks: world.blocks.map(b => ({name: b.name, size: b.size, x: b.x, y: b.y, z: b.z}))}),
};

if (document.getElementById('scene')) init();
})();
