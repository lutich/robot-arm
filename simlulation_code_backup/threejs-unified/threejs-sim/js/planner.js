// Pick-and-place and stacking plans in servo counts (docs/pick-and-place-plan.md section 4,
// docs/tracking-plan.md section 5), checked by running them headless in the simulation.

import { CUBE, PLAN, SERVOS } from './config.js';
import { ikTopDown, countsFromAngles, anglesFromCounts, guardViolations, Unreachable } from './kinematics.js';
import { World } from './world.js';
import { cubeYaw, cubeTilt } from './physics.js';
import { SimController } from './controller.js';

export class PlanError extends Error {}

const RELEASE_GAP = 0.2; // let go 2 mm above the support, the cube drops onto it

const COLOUR_WORDS = {
  orange: 'orange', blue: 'blue', blau: 'blue', yellow: 'yellow', gelb: 'yellow',
  green: 'green', 'grün': 'green', gruen: 'green', red: 'red', rot: 'red',
};

/**
 * "put blue on orange", "blau auf orange", "move green to 5 15", "gelb nach -8 12".
 * Returns { pick, place: { on } | { x, y } }.
 */
export function parseCommand(text, world) {
  const words = text.toLowerCase().replace(/[,;]/g, ' ').split(/\s+/).filter(Boolean);
  const colours = words.map(w => COLOUR_WORDS[w]).filter(Boolean);
  const nums = words.map(Number).filter(n => Number.isFinite(n));
  const known = id => { if (!world.cube(id)) throw new PlanError(`no ${id} cube in the scene`); return id; };
  if (colours.length >= 2) return { pick: known(colours[0]), place: { on: known(colours[1]) } };
  if (colours.length === 1 && nums.length >= 2) return { pick: known(colours[0]), place: { x: nums[0], y: nums[1] } };
  throw new PlanError('say "<colour> on <colour>" or "<colour> to <x> <y>" (cm, board frame)');
}

/** The cube at the top of the stack that contains cube c. */
function topOf(world, c) {
  let t = c, n;
  while ((n = world.cubeOn(t))) t = n;
  return t;
}

/**
 * Plan moving cube `pick` to `place`. `start` is the current six counts.
 * Returns { steps: [{ label, phase, counts }], info }.
 */
export function planTransfer(world, { pick, place }, start, { servos = SERVOS, home = null, step = PLAN.descentStep } = {}) {
  const c = world.cube(pick);
  if (!c) throw new PlanError(`no cube "${pick}"`);
  if (c.state === 'held') throw new PlanError(`${pick} is already in the gripper`);
  if (cubeTilt(c) > 20) throw new PlanError(`${pick} lies tilted ${cubeTilt(c).toFixed(0)}°; a top-down grasp needs it flat`);
  if (c.p[2] < 0) throw new PlanError(`${pick} is off the board`);
  const onTop = world.cubeOn(c);
  if (onTop) throw new PlanError(`${pick} has ${onTop.id} on top; move ${onTop.id} first`);

  // Where the cube centre goes.
  let tx, ty, tz, tyaw, onWhat;
  if (place.on) {
    const base = world.cube(place.on);
    if (!base) throw new PlanError(`no cube "${place.on}"`);
    if (base === c) throw new PlanError('cannot put a cube on itself');
    const top = topOf(world, base);
    if (top === c) throw new PlanError(`${pick} already sits on ${place.on}`);
    [tx, ty] = top.p; tz = top.p[2] + CUBE; tyaw = cubeYaw(top); onWhat = top.id;
  } else {
    tx = place.x; ty = place.y; tyaw = place.yaw || 0;
    const probe = { id: '_probe', p: [tx, ty, 100] };
    const s = world.supportFor(probe);
    if (s.edge) throw new PlanError(`(${tx}, ${ty}) is on the edge of ${s.edge.id}; it would topple`);
    if (s.on === 'floor') throw new PlanError(`(${tx}, ${ty}) is off the board`);
    tz = s.z + CUBE / 2; onWhat = s.on;
  }

  const S6 = servos[5];
  let s5Near = anglesFromCounts(start, servos).s5;
  const solve = (x, y, zc, yaw, label) => {
    let a;
    try { a = ikTopDown(x, y, zc, { cubeYaw: yaw, s5Near, servos }); }
    catch (e) { if (e instanceof Unreachable) throw new PlanError(`${label}: unreachable (${e.message})`); throw e; }
    s5Near = a.s5;
    const counts = countsFromAngles(a, 0, servos).slice(0, 5);
    const bad = guardViolations(counts.concat([S6.open]), servos);
    if (bad.length) throw new PlanError(`${label}: ${bad.join(', ')}`);
    return counts;
  };

  // Highest hover that both ends can reach (docs/tracking-plan.md: shorter hover above stacks).
  const want = Math.min(c.p[2] < CUBE ? PLAN.hover : PLAN.hoverStacked, tz < CUBE ? PLAN.hover : PLAN.hoverStacked);
  let hover = null;
  for (const h of [want, 3.5, 2, 1.5, 1].filter(h => h <= want)) {
    try { solve(c.p[0], c.p[1], c.p[2] + h, cubeYaw(c), 'probe'); solve(tx, ty, tz + RELEASE_GAP + h, tyaw, 'probe'); hover = h; break; }
    catch (e) { if (!(e instanceof PlanError)) throw e; }
  }
  s5Near = anglesFromCounts(start, servos).s5;
  if (hover === null) {
    // Report the real reason with the lowest hover.
    solve(c.p[0], c.p[1], c.p[2] + 1, cubeYaw(c), `above ${pick}`);
    solve(tx, ty, tz + RELEASE_GAP + 1, tyaw, `above ${onWhat}`);
  }

  const steps = [];
  const push = (label, phase, five, s6) => {
    const counts = five.concat([s6]);
    const last = steps[steps.length - 1];
    if (last && last.counts.every((v, i) => v === counts[i])) return;
    steps.push({ label, phase, counts });
  };
  const vertical = (x, y, z0, z1, yaw, s6, label, phase) => {
    const n = Math.max(1, Math.ceil(Math.abs(z1 - z0) / step));
    for (let i = 1; i <= n; i++) push(`${label} ${i}/${n}`, phase, solve(x, y, z0 + (z1 - z0) * i / n, yaw, label), s6);
  };

  push('Open gripper', 'pick', start.slice(0, 5), S6.open);
  push(`Above ${pick}`, 'pick', solve(c.p[0], c.p[1], c.p[2] + hover, cubeYaw(c), `above ${pick}`), S6.open);
  vertical(c.p[0], c.p[1], c.p[2] + hover, c.p[2], cubeYaw(c), S6.open, 'Descend', 'pick');
  const grip = steps[steps.length - 1].counts.slice(0, 5);
  push(`Grip ${pick}`, 'pick', grip, S6.grip);
  vertical(c.p[0], c.p[1], c.p[2], c.p[2] + hover, cubeYaw(c), S6.grip, 'Lift', 'carry');
  const placeZ = tz + RELEASE_GAP;
  push(`Above ${onWhat === 'board' ? `(${tx.toFixed(1)}, ${ty.toFixed(1)})` : onWhat}`, 'carry', solve(tx, ty, placeZ + hover, tyaw, 'above target'), S6.grip);
  vertical(tx, ty, placeZ + hover, placeZ, tyaw, S6.grip, 'Lower', 'place');
  push('Release', 'place', steps[steps.length - 1].counts.slice(0, 5), S6.open);
  vertical(tx, ty, placeZ, placeZ + hover, tyaw, S6.open, 'Withdraw', 'place');
  if (home) push('HOME', 'return', home.slice(0, 5), home[5]);

  const vias = insertVias(world, steps, start, servos);
  return { steps, info: { pick, onWhat, hover, target: [tx, ty, tz], targetYaw: tyaw, vias } };
}

// ---------------- collision-aware transits ----------------

/** Does the straight joint-space path a -> b (what demo playback does) stay free of contacts? */
function segmentClear(w, a, b) {
  const n = Math.max(2, Math.ceil(Math.max(...a.map((v, i) => Math.abs(b[i] - v))) / 4));
  const snap = w.snapshot();
  let clear = true;
  for (let i = 1; i <= n && clear; i++) {
    w.step(0, a.map((v, k) => v + (b[k] - v) * i / n));
    if (w.contacts.length) clear = false;
  }
  w.restore(snap);
  return clear;
}

/**
 * Walk through the plan; wherever a joint-space transit would hit something, insert one or
 * two raised via poses (gripper pointing down, above the tallest stack, closer to S1).
 * Changes `steps` in place and returns how many vias were added.
 */
function insertVias(world, steps, start, servos) {
  const w = new World(servos);
  w.restore(world.snapshot());
  w.margin = PLAN.transitMargin;
  let cur = start.slice(), added = 0;
  w.step(0, cur);
  for (let i = 0; i < steps.length; i++) {
    const next = steps[i].counts;
    if (!segmentClear(w, cur, next)) {
      const via = findVia(w, cur, next, servos);
      if (via) {
        const s6 = cur[5];
        const extra = via.map((five, k) => ({ label: `Transit ${k + 1}`, phase: steps[i].phase, counts: five.concat([s6]) }));
        steps.splice(i, 0, ...extra);
        added += extra.length;
        i += extra.length - 1;
        cur = extra[extra.length - 1].counts.slice();
        w.step(0, cur);
        continue;
      }
    }
    w.step(0, next);
    for (let k = 0; k < 60 && !w.settled(); k++) w.step(0.02, next);
    cur = next.slice();
  }
  return added;
}

function findVia(w, a, b, servos) {
  const free = w.cubes.filter(c => c.state !== 'held');
  const topZ = free.reduce((m, c) => Math.max(m, c.p[2] + CUBE / 2), 0);
  const holding = !!w.holding;
  const zBase = topZ + (holding ? CUBE / 2 : 0.7) + 1.0;   // grasp point height that clears every stack
  const s1a = anglesFromCounts(a, servos).s1, s1b = anglesFromCounts(b, servos).s1;
  const s5 = anglesFromCounts(a, servos).s5;
  const pose = (s1deg, r, zc) => {
    const t = s1deg * Math.PI / 180;
    try {
      const q = ikTopDown(r * Math.sin(t), r * Math.cos(t), zc, { cubeYaw: -s1deg - s5, s5Near: s5, servos });
      const c = countsFromAngles(q, a[5], servos);
      return guardViolations(c, servos).length ? null : c.slice(0, 5);
    } catch (e) { if (e instanceof Unreachable) return null; throw e; }
  };
  const radii = [12, 14, 10, 16, 18], lifts = [0, 2, 4, 6];
  const ok = (p, q) => segmentClear(w, p, q);
  const full = p => p.concat([a[5]]);
  // One via, between the two S1 angles.
  for (const dz of lifts) for (const r of radii) for (const s1 of [(s1a + s1b) / 2, s1a, s1b]) {
    const v = pose(s1, r, zBase + dz);
    if (v && ok(a, full(v)) && ok(full(v), b)) return [v];
  }
  // Two vias: rise above the start, swing, come down above the end.
  for (const dz of lifts) for (const r of radii) {
    const v1 = pose(s1a, r, zBase + dz), v2 = pose(s1b, r, zBase + dz);
    if (v1 && v2 && ok(a, full(v1)) && ok(full(v1), full(v2)) && ok(full(v2), b)) return [v1, v2];
  }
  return null;
}

// ---------------- headless verification ----------------

/**
 * Run a plan in a copy of the world. mode 'demo' plays full poses at `speed` counts/s
 * (the controller's demo playback); mode 'moves' plays single-joint moves at 20 counts/s
 * (what the HTTP API can do). Returns a report with every contact and the end state.
 */
export function verify(world, plan, start, { mode = 'demo', speed = 20, servos = SERVOS, moves = null, dt = 1 / 30 } = {}) {
  const w = new World(servos);
  w.restore(world.snapshot());
  const ctl = new SimController(servos);
  ctl.setCommands(start);
  let labels = plan.steps.map(s => s.label);
  if (mode === 'moves') {
    const mv = moves || toMoves(world, plan, start, servos).moves;
    ctl.runMoves(mv, mv.map(m => m.label));
  } else {
    ctl.runPoses(plan.steps.map(s => s.counts), speed, labels);
  }
  const contacts = new Map();
  let t = 0, label = '';
  w.events = [];
  w.step(0, ctl.position);
  while ((ctl.busy || !w.settled()) && t < 1200) {
    ctl.update(dt);
    if (ctl.motion) label = ctl.motion.label;
    w.step(dt, ctl.position);
    t += dt;
    for (const k of w.contacts) {
      const key = `${k.a} × ${k.b}`;
      const prev = contacts.get(key);
      if (!prev || k.sep < prev.sep) contacts.set(key, { a: k.a, b: k.b, sep: k.sep, step: prev ? prev.step : label, t: prev ? prev.t : t });
    }
  }
  const c = w.cube(plan.info.pick);
  const [tx, ty, tz] = plan.info.target;
  const off = Math.hypot(c.p[0] - tx, c.p[1] - ty);
  const ok = c.state !== 'held' && off < 0.9 && Math.abs(c.p[2] - tz) < 0.3 && cubeTilt(c) < 10 && contacts.size === 0;
  return { ok, duration: t, contacts: [...contacts.values()], events: w.events, offset: off, final: w.snapshot() };
}

// ---------------- full poses -> single-joint moves ----------------

const PERMS = arr => arr.length <= 1 ? [arr] : arr.flatMap((x, i) => PERMS([...arr.slice(0, i), ...arr.slice(i + 1)]).map(p => [x, ...p]));

/**
 * The real API moves one joint at a time. For each step, pick the joint order whose
 * intermediate poses stay free of contacts (checked kinematically at a few points).
 */
export function toMoves(world, plan, start, servos = SERVOS) {
  const w = new World(servos);
  w.restore(world.snapshot());
  w.margin = PLAN.transitMargin;
  let cur = start.slice();
  const moves = [], notes = [];
  w.step(0, cur);
  const clearOrder = order => {
    const snap = w.snapshot();
    let c = cur.slice(), clean = true;
    for (const ch of order) {
      const from = c[ch], to = target[ch];
      for (const f of [0.25, 0.5, 0.75, 1]) {
        c[ch] = from + (to - from) * f;
        w.step(0, c);
        if (w.contacts.length) { clean = false; break; }
      }
      if (!clean) break;
    }
    w.restore(snap);
    return clean;
  };
  let target;
  for (const s of plan.steps) {
    target = s.counts;
    const changed = [0, 1, 2, 3, 4, 5].filter(ch => target[ch] !== cur[ch]);
    // Lifting: raise S2 (shoulder) last when going down, first when going up; S6 alone.
    const candidates = changed.length <= 4 ? PERMS(changed)
      : [changed, changed.slice().reverse(), [1, 2, 3, 0, 4, 5].filter(c => changed.includes(c)), [3, 2, 1, 0, 4, 5].filter(c => changed.includes(c))];
    let order = candidates.find(clearOrder);
    if (!order) { order = candidates[0]; notes.push(`${s.label}: no contact-free joint order found`); }
    for (const ch of order) moves.push({ ch, target: target[ch], label: `${s.label}: ${servos[ch].name} → ${target[ch]}` });
    // Advance the world through this step so grasp and release happen.
    const c = cur.slice();
    for (const ch of order) { c[ch] = target[ch]; w.step(0, c); }
    for (let i = 0; i < 60 && !w.settled(); i++) w.step(0.02, c);
    cur = target.slice();
  }
  return { moves, notes };
}
