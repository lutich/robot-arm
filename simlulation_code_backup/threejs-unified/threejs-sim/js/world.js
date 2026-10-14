// The simulated world: arm pose from servo counts, the gripper's grasp logic, the cubes as
// rigid bodies (physics.js) and the collision report. Pure math, no three.js.
//
// - Free cubes are dynamic: gravity, friction, cube-cube and cube-board contacts, and the
//   arm (a kinematic collider) can push or topple them.
// - Grasping is rule based, because a two-finger grip through friction alone is not stable
//   in a small solver: when the closing jaws reach a cube that sits between the pads, the
//   jaws square it up, centre it and it moves rigidly with the hand. Opening the jaws
//   releases it as a dynamic body with the hand's velocity.
// - step(0, counts) is a pure kinematic check (used by the planner): it reports any arm part
//   overlapping a cube. step(dt > 0, counts) runs the physics; arm-cube contacts that carry
//   force are reported as collisions, except the pads centring the cube they are gripping.

import { BOARD, CUBE, GRIPPER, SERVOS, PLAN } from './config.js';
import { fk, anglesFromCounts, gripperTheta, gapForTheta, padDepthForTheta, thetaForGap, countsForGap,
         toolYawAndTilt, apply3, transpose3, point, rotZ } from './kinematics.js';
import { armShapes, findContacts } from './collision.js';
import { PHYS, initBody, stepPhysics, cubeYaw, cubeTilt } from './physics.js';

const wrap90 = d => ((d + 45) % 90 + 90) % 90 - 45;
const copyCube = c => ({ ...c, p: c.p.slice(), R: c.R.slice(), v: c.v.slice(), w: c.w.slice() });

export class World {
  constructor(servos = SERVOS) {
    this.servos = servos;
    this.cubes = [];
    this.holding = null;        // { id, width, w, v }
    this.gap = 4.5;
    this.contacts = [];
    this.events = [];
    this.time = 0;
    this.frames = null;
    this.tol = PLAN.clearance;
    this.margin = 0;            // extra arm-to-cube clearance for kinematic checks (planner sets it)
    this.prevCounts = null;
    this.prevShapes = null;
  }

  addCube(id, colour, x, y, level = 0, yaw = 0) {
    const c = initBody({ id, colour, p: [x, y, CUBE / 2 + level * CUBE], R: rotZ(yaw * Math.PI / 180), state: 'awake' });
    this.cubes.push(c);
    return c;
  }
  cube(id) { return this.cubes.find(c => c.id === id); }

  snapshot() { return { cubes: this.cubes.map(copyCube), holding: this.holding && { ...this.holding }, gap: this.gap }; }
  restore(s) { this.cubes = s.cubes.map(copyCube); this.holding = s.holding && { ...s.holding }; this.gap = s.gap; this.prevCounts = null; this.prevShapes = null; }

  event(kind, text) { this.events.push({ t: this.time, kind, text }); if (this.events.length > 300) this.events.shift(); }

  // ---------------- static queries (planner) ----------------

  /** Is (x, y) inside the footprint of cube c, with margin m (cm)? */
  static inFootprint(c, x, y, m = 0) {
    const a = -cubeYaw(c) * Math.PI / 180, dx = x - c.p[0], dy = y - c.p[1];
    const lx = dx * Math.cos(a) - dy * Math.sin(a), ly = dx * Math.sin(a) + dy * Math.cos(a);
    return Math.abs(lx) <= CUBE / 2 + m && Math.abs(ly) <= CUBE / 2 + m;
  }
  onBoard(x, y) { return Math.abs(x) <= BOARD.w / 2 && y >= -BOARD.axisToBackEdge && y <= BOARD.d - BOARD.axisToBackEdge; }

  /** What a cube dropped at (x, y) would land on: { z, on, edge }. `edge` is a cube it would topple off. */
  supportFor(probe) {
    const x = probe.p[0], y = probe.p[1];
    let z = this.onBoard(x, y) ? 0 : -BOARD.t, on = this.onBoard(x, y) ? 'board' : 'floor', edge = null;
    for (const o of this.cubes) {
      if (o === probe || o.state === 'held') continue;
      const top = o.p[2] + CUBE / 2;
      if (top <= z) continue;
      if (World.inFootprint(o, x, y)) { z = top; on = o.id; edge = null; }
      else if (World.inFootprint(o, x, y, CUBE * 0.5)) edge = o;
    }
    return { z, on, edge: edge && edge.p[2] + CUBE / 2 > z ? edge : null };
  }

  /** The cube resting directly on top of cube c, if any. */
  cubeOn(c) {
    return this.cubes.find(o => o !== c && o.state !== 'held' && Math.abs(o.p[2] - c.p[2] - CUBE) < 0.3 && World.inFootprint(c, o.p[0], o.p[1], 0.3));
  }
  levelOf(c) { return Math.round((c.p[2] - CUBE / 2) / CUBE); }
  settled() { return !this.cubes.some(c => c.state === 'awake'); }

  // ---------------- one step ----------------

  /** Advance by dt with the arm at `counts` (6 values, may be fractional). Returns { frames, gap, contacts }. */
  step(dt, counts) {
    this.time += dt;
    const f = this.frames = fk(anglesFromCounts(counts, this.servos));
    const cmdGap = gapForTheta(gripperTheta(counts[5], this.servos[5]));
    // the jaws count as "closing" while the commanded gap shrinks, until they hold or reopen
    if (cmdGap < (this.lastCmdGap ?? cmdGap) - 1e-6) this.closing = true;
    else if (cmdGap > (this.lastCmdGap ?? cmdGap) + 1e-6) this.closing = false;
    this.lastCmdGap = cmdGap;
    const { yaw: jawYaw, tilt } = toolYawAndTilt(f.hand);
    const RT = transpose3(f.hand.R);
    const local = p => apply3(RT, [p[0] - f.hand.p[0], p[1] - f.hand.p[1], p[2] - f.hand.p[2]]);

    // ---- grasp and release
    if (this.holding) {
      const c = this.cube(this.holding.id);
      if (cmdGap > this.holding.width + 0.02) {
        this.released = { id: c.id, width: this.holding.width };      // the pads still touch it while they open
        this.holding = null; this.gap = cmdGap;
        c.state = 'awake'; c.sleepT = 0; c.w = [0, 0, 0];
        if (tilt > 10) this.event('warn', `${c.id} released with the gripper ${tilt.toFixed(0)}° off vertical`);
        this.event('release', `released ${c.id}`);
      } else this.gap = this.holding.width;
    } else {
      let grasped = null;
      const vPad = padDepthForTheta(thetaForGap(Math.max(cmdGap, 0)));
      if (cmdGap < this.gap + 1e-9) for (const c of this.cubes) {
        if (c.state === 'held' || cubeTilt(c) > 25) continue;
        const q = local(c.p);                    // u = jaw axis, w = depth, v = down the tool
        const e = wrap90(cubeYaw(c) - jawYaw);
        const width = Math.abs(e) < 20 ? CUBE : CUBE * (Math.abs(Math.cos(e * Math.PI / 180)) + Math.abs(Math.sin(e * Math.PI / 180)));
        if (cmdGap >= width) continue;
        const between = Math.abs(q[0]) + width / 2 <= this.gap / 2 + 0.05;
        const vOverlap = Math.abs(q[2] - vPad) < CUBE / 2 + GRIPPER.pad.h / 2 - 0.2;
        const wOverlap = Math.abs(q[1]) < CUBE / 2 + GRIPPER.pad.w / 2 - 0.2;
        if (!(between && vOverlap && wOverlap && tilt < 30)) continue;
        if (Math.abs(e) >= 20) { this.event('warn', `jaws hit the corners of ${c.id} (${e.toFixed(0)}° off), no grasp`); continue; }
        grasped = { c, width, q, e };
        break;
      }
      if (grasped) {
        const { c, width, q, e } = grasped;
        this.holding = { id: c.id, width, w: q[1], v: q[2] };
        c.state = 'held'; c.v = [0, 0, 0]; c.w = [0, 0, 0];
        this.gap = width;
        const above = this.cubeOn(c);
        this.event('grasp', `grasped ${c.id}: centred by ${Math.abs(q[0]).toFixed(2)} cm, squared by ${Math.abs(e).toFixed(1)}°, ` +
          `jaws touch at S6 ≈ ${countsForGap(width, this.servos[5]).toFixed(0)}; counts beyond that squeeze the cube` +
          (above ? ` (${above.id} sits on it and will fall)` : ''));
      } else this.gap = cmdGap;
    }

    const gap = this.gap;
    const heldPose = fr => { const h = this.holding; return h ? { p: point(fr.hand, [0, h.w, h.v]), R: fr.hand.R.slice() } : null; };

    // ---- kinematic check only (planner)
    if (dt <= 0) {
      if (this.holding) { const c = this.cube(this.holding.id), hp = heldPose(f); c.p = hp.p; c.R = hp.R; }
      const shapes = armShapes(f, gap);
      this.contacts = findContacts(shapes, this.cubes, BOARD, this.tol, this.margin)
        // pads closing on the cube between them are gripping, not colliding
        .filter(k => !(this.margin > 0 && /pad|finger/.test(k.a) && k.b.endsWith(' cube') && this.cube(k.b.slice(0, -5)) && this.centring({ part: k.a, cube: k.b.slice(0, -5) }, f, gap)));
      this.prevCounts = counts.slice(); this.prevShapes = null;
      return { frames: f, gap, contacts: this.contacts };
    }

    // ---- physics, in substeps with the arm interpolated between the last and this pose
    const from = this.prevCounts || counts;
    const n = Math.max(1, Math.ceil(dt / PHYS.h)), h = dt / n;
    const pushes = new Map();
    const finalShapes = armShapes(f, gap);
    const needed = this.cubes.some(c => c.state === 'awake' || c.state === 'held') || this.armNearCubes(finalShapes) || (this.prevShapes && this.armNearCubes(this.prevShapes));
    let prev = this.prevShapes;
    if (needed) {
      for (let i = 1; i <= n; i++) {
        const ci = from.map((v, k) => v + (counts[k] - v) * i / n);
        const fi = i === n ? f : fk(anglesFromCounts(ci, this.servos));
        const shapes = i === n ? finalShapes : armShapes(fi, gap);
        if (prev) shapes.forEach((s, k) => { s.prev = prev[k]; });
        for (const s of shapes) if (s.name.endsWith('pad') || s.name.endsWith('finger')) s.pad = true;
        if (this.holding) {
          const c = this.cube(this.holding.id), hp = heldPose(fi);
          c.v = [(hp.p[0] - c.p[0]) / h, (hp.p[1] - c.p[1]) / h, (hp.p[2] - c.p[2]) / h];
          c.p = hp.p; c.R = hp.R;
        }
        for (const hit of stepPhysics(this.cubes, shapes, h)) {
          if (this.centring(hit, fi, gap)) continue;
          const key = `${hit.part} × ${hit.cube} cube`;
          const old = pushes.get(key);
          if (!old || hit.depth > old.depth) pushes.set(key, { a: hit.part, b: `${hit.cube} cube`, sep: -Math.max(hit.depth, this.tol * 1.2) });
        }
        prev = shapes;
      }
    } else if (this.holding) {
      const c = this.cube(this.holding.id), hp = heldPose(f); c.p = hp.p; c.R = hp.R;
    }
    this.prevShapes = needed ? prev : finalShapes;
    this.prevCounts = counts.slice();
    this.afterPhysics();

    // self collision and the board are kinematic checks; cubes report through the physics
    this.contacts = findContacts(finalShapes, [], BOARD, this.tol).concat([...pushes.values()]);
    for (const k of pushes.keys()) if (!this.lastPushes?.has(k)) this.event('warn', `arm pushed: ${k}`);
    this.lastPushes = new Set(pushes.keys());
    return { frames: f, gap, contacts: this.contacts };
  }

  armNearCubes(shapes) {
    for (const c of this.cubes) {
      if (c.state === 'held') continue;
      for (const s of shapes) {
        const dx = s.bound.c[0] - c.p[0], dy = s.bound.c[1] - c.p[1], dz = s.bound.c[2] - c.p[2], r = s.bound.r + CUBE;
        if (dx * dx + dy * dy + dz * dz < r * r) return true;
      }
    }
    return false;
  }

  /** A pad or finger touching a cube that sits between the jaws while they close: centring, not a collision. */
  centring(hit, f, gap) {
    // Only while the jaws close and are near the cube's width: open jaws passing a cube are a collision.
    if (!/pad|finger/.test(hit.part)) return false;
    if (this.released && hit.cube === this.released.id && gap < this.released.width + 1.0) return true;   // letting go
    if (!this.closing || gap > CUBE + 1.2) return false;
    const c = this.cube(hit.cube);
    const RT = transpose3(f.hand.R);
    const q = apply3(RT, [c.p[0] - f.hand.p[0], c.p[1] - f.hand.p[1], c.p[2] - f.hand.p[2]]);
    return Math.abs(q[0]) + CUBE / 2 <= gap / 2 + 0.3 && Math.abs(q[1]) < CUBE && Math.abs(q[2] - padDepthForTheta(thetaForGap(gap))) < CUBE;
  }

  /** Notes when cubes come to rest: toppled, fell off the board. */
  afterPhysics() {
    for (const c of this.cubes) {
      if (c.state === 'awake') { c._was = 'awake'; continue; }
      if (c._was !== 'awake' || c.state !== 'asleep') continue;
      c._was = 'asleep';
      if (c.p[2] < -0.2) this.event('warn', `${c.id} fell off the board`);
      else if (cubeTilt(c) > 20) this.event('warn', `${c.id} toppled and lies tilted ${cubeTilt(c).toFixed(0)}°`);
      else this.event('land', `${c.id} at rest on level ${this.levelOf(c)}`);
    }
  }
}
