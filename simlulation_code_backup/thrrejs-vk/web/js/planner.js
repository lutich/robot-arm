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
  if (!place) {
    onWhat = 'gripper';
  } else if (place.on) {
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
  const pickHover = c.p[2] < CUBE ? PLAN.hover : PLAN.hoverStacked;
  const want = place ? Math.min(pickHover, tz < CUBE ? PLAN.hover : PLAN.hoverStacked) : pickHover;
  let hover = null;
  for (const h of [want, 3.5, 2, 1.5, 1].filter(h => h <= want)) {
    try {
      solve(c.p[0], c.p[1], c.p[2] + h, cubeYaw(c), 'probe');
      if (place) solve(tx, ty, tz + RELEASE_GAP + h, tyaw, 'probe');
      hover = h; break;
    }
    catch (e) { if (!(e instanceof PlanError)) throw e; }
  }
  s5Near = anglesFromCounts(start, servos).s5;
  if (hover === null) {
    // Report the real reason with the lowest hover.
    solve(c.p[0], c.p[1], c.p[2] + 1, cubeYaw(c), `above ${pick}`);
    if (place) solve(tx, ty, tz + RELEASE_GAP + 1, tyaw, `above ${onWhat}`);
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
  if (!place) {
    return { steps, info: { pick, onWhat, hover, target: [...c.p], targetYaw: cubeYaw(c), vias: 0, retainGrasp: true } };
  }
  const placeZ = tz + RELEASE_GAP;
  push(`Above ${onWhat === 'board' ? `(${tx.toFixed(1)}, ${ty.toFixed(1)})` : onWhat}`, 'carry', solve(tx, ty, placeZ + hover, tyaw, 'above target'), S6.grip);
  vertical(tx, ty, placeZ + hover, placeZ, tyaw, S6.grip, 'Lower', 'place');
  push('Release', 'place', steps[steps.length - 1].counts.slice(0, 5), S6.open);
  vertical(tx, ty, placeZ, placeZ + hover, tyaw, S6.open, 'Withdraw', 'place');
  if (home) push('HOME', 'return', home.slice(0, 5), home[5]);

  return { steps, info: { pick, onWhat, hover, target: [tx, ty, tz], targetYaw: tyaw, vias: 0 } };
}

// ---------------- headless verification ----------------

/**
 * Run a plan in a copy of the world. mode 'demo' plays full poses at `speed` counts/s
 * (the controller's demo playback); mode 'moves' plays single-joint moves at 20 counts/s
 * (what the HTTP API can do). This checks the simulated grasp/place outcome;
 * geometric collision checking is intentionally disabled during calibration.
 */
export function verify(world, plan, start, { mode = 'demo', speed = 20, servos = SERVOS, moves = null, dt = 1 / 30 } = {}) {
  const w = new World(servos);
  w.restore(world.snapshot());
  const ctl = new SimController(servos);
  ctl.setCommands(start);
  const labels = plan.steps.map(s => s.label);
  if (mode === 'moves') {
    const mv = moves || toMoves(world, plan, start, servos).moves;
    ctl.runMoves(mv, mv.map(m => m.label));
  } else {
    ctl.runPoses(plan.steps.map(s => s.counts), speed, labels);
  }
  let t = 0;
  w.events = [];
  w.step(0, ctl.position);
  while ((ctl.busy || !w.settled()) && t < 1200) {
    ctl.update(dt);
    w.step(dt, ctl.position);
    t += dt;
  }
  const c = w.cube(plan.info.pick);
  const [tx, ty, tz] = plan.info.target;
  const off = Math.hypot(c.p[0] - tx, c.p[1] - ty);
  const ok = plan.info.retainGrasp
    ? c.state === 'held'
    : c.state !== 'held' && off < 0.9 && Math.abs(c.p[2] - tz) < 0.3 && cubeTilt(c) < 10;
  return { ok, duration: t, contacts: [], events: w.events, offset: off, final: w.snapshot(), verification: 'physics-outcome-only' };
}

// ---------------- full poses -> single-joint moves ----------------

/**
 * Convert each coordinated pose to deterministic S1→S6 single-joint steps.
 * The result is for simulation/controller preview only while collision checks are disabled.
 */
export function toMoves(world, plan, start, servos = SERVOS) {
  let cur = start.slice();
  const moves = [];
  for (const s of plan.steps) {
    const target = s.counts;
    const changed = [0, 1, 2, 3, 4, 5].filter(ch => target[ch] !== cur[ch]);
    for (const ch of changed) moves.push({ ch, target: target[ch], label: `${s.label}: ${servos[ch].name} → ${target[ch]}` });
    cur = target.slice();
  }
  return { moves, notes: ['Calibration ordering only; no collision model is active.'] };
}
