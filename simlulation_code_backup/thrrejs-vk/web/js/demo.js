// Demonstration: build a tower with the planner, then a physics finale.
// Every move uses the same calibration planner as the Task panel. Simulation only.

import { COLOURS } from './config.js';
import { planTransfer, verify } from './planner.js';

const SCENE = [['orange', 0, 11, 0, 0], ['blue', -9, 12, 0, 0], ['yellow', 9, 11, 0, 20], ['green', -6, 17.5, 0, -15]];
const TASKS = [['blue', 'orange'], ['yellow', 'blue']];

/** Rotation matrix (row-major) about an axis. */
function rot(axis, deg) {
  const n = Math.hypot(...axis), [x, y, z] = axis.map(v => v / n), a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}

/**
 * ctx: { ctl, world, $, log, toast, renderCubes, showPath, setPathVisible(on),
 *        resetView() }
 * Returns { run, stop, running }.
 */
export function initDemo(ctx) {
  const { ctl, world, $, log } = ctx;
  let running = false, abort = false;
  const until = (cond, maxS = 300) => new Promise(res => {
    const t0 = performance.now();
    const id = setInterval(() => { if (cond() || abort || performance.now() - t0 > maxS * 1000) { clearInterval(id); res(); } }, 60);
  });
  const pause = s => until(() => false, s);

  const spawn = (colour, p, R, v, w) => {
    let id = colour, n = 2; while (world.cube(id)) id = `${colour}${n++}`;
    const c = world.addCube(id, colour, p[0], p[1], 0, 0);
    Object.assign(c, { p, R, v, w, state: 'awake', sleepT: 0 });
    ctx.renderCubes();
    return c;
  };

  async function buildTower() {
    for (const [pick, on] of TASKS) {
      if (abort) return false;
      ctx.setTaskStatus?.({ task: `Demo: ${pick} on ${on}`, destination: `${pick} cube on ${on} cube`, status: 'planning', message: 'Calculating an unchecked simulation transfer.' });
      $('cmd').value = `${pick} on ${on}`;
      const start = ctl.commands.slice();
      const plan = planTransfer(world, { pick, place: { on } }, start, { home: ctl.poses.HOME });
      const report = verify(world, plan, start, { speed: 60 });
      ctx.showPath(plan.steps, start);
      if (!report.ok) { log(`demo: "${pick} on ${on}" does not pass the simulation check, stopping`, 'err'); return false; }
      log(`demo: ${pick} on ${on} (${plan.steps.length} poses, checked)`);
      ctx.setTaskStatus?.({ task: `Demo: ${pick} on ${on}`, destination: `${pick} cube on ${on} cube`, status: 'moving', message: `Running ${plan.steps.length} unchecked simulation poses.` });
      ctl.runPoses(plan.steps.map(s => s.counts), 60, plan.steps.map(s => s.label));
      await until(() => !ctl.busy && world.settled());
      await pause(0.6);
    }
    return true;
  }

  async function finale() {
    ctx.setTaskStatus?.({ task: 'Demo: physics finale', destination: 'Completed cube tower', status: 'moving', message: 'Running the dynamic-cube physics sequence.' });
    ctx.setPathVisible(false);
    // Swing the arm aside in simulation; collision checks are disabled.
    const aside = ctl.commands.slice(); aside[0] = 520;
    ctl.runPoses([aside], 60, ['swing aside']); await until(() => !ctl.busy);
    log('demo: physics finale, dropping cubes onto the tower', 'ok');
    const top = world.cube('yellow');
    // green lands flat on top (a 4th level by gravity), red clips the top edge
    for (const [colour, x, y, z, w, wait] of [['green', top.p[0], top.p[1], top.p[2] + 9, [0, 0, 1.5], 2.0], ['red', top.p[0] + 1.6, top.p[1] + 0.4, top.p[2] + 16, [6, -5, 2], 3.0]]) {
      if (abort) return;
      const c = world.cube(colour) || world.addCube(colour, colour, x, y, 0, 0);
      Object.assign(c, { p: [x, y, z], v: [0, 0, -20], w, R: [1, 0, 0, 0, 1, 0, 0, 0, 1], state: 'awake', sleepT: 0 });
      ctx.renderCubes();
      await pause(wait);
    }
    if (abort) return;
    log('demo: a cube dropped on its corner'); spawn('blue', [7, 17, 14], rot([1, 1, 0], 54.7), [0, 0, -10], [0, 0, 3]); await pause(2.5);
    if (abort) return;
    log('demo: a cube thrown at the tower'); spawn('orange', [-15, world.cube('yellow').p[1], 8.5], rot([0, 0, 1], 30), [160, 0, 30], [0, -8, 4]); await pause(3);
    if (abort) return;
    log('demo: cube rain');
    const colours = Object.keys(COLOURS);
    let seed = 7; const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
    for (let i = 0; i < 6 && !abort; i++) {
      spawn(colours[i % colours.length], [-7 + 14 * rnd(), 15 + 8 * rnd(), 16 + 14 * rnd()], rot([rnd() - 0.5, rnd() - 0.5, rnd() - 0.5], 360 * rnd()),
        [(rnd() - 0.5) * 20, (rnd() - 0.5) * 20, -10], [(rnd() - 0.5) * 12, (rnd() - 0.5) * 12, (rnd() - 0.5) * 12]);
      await pause(0.35);
    }
    await until(() => world.settled(), 20);
    log('demo: done. "Reset" in the Scene panel restores the cubes', 'ok');
    ctx.setTaskStatus?.({ task: 'Demo: build a tower', destination: 'Completed cube tower', status: 'complete', message: 'Demo and physics finale completed.' });
  }

  async function run() {
    if (running) { abort = true; return; }
    running = true; abort = false;
    $('demo-btn').textContent = '■ Stop demo';
    try {
      ctl.rearm(); ctl.setCommands(ctl.poses.HOME.slice());
      world.cubes = []; world.holding = null;
      for (const [c, x, y, l, yaw] of SCENE) world.addCube(c, c, x, y, l, yaw);
      world.dirty = true; ctx.renderCubes();
      ctx.resetView();
      log('demo: building a tower', 'ok');
      await until(() => world.settled(), 5);
      if (await buildTower() && !abort) await finale();
    } catch (e) { log(`demo: ${e.message}`, 'err'); ctx.toast(e.message); ctx.setTaskStatus?.({ status: 'error', message: e.message }); }
    ctx.setPathVisible($('show-path').checked);
    running = false; abort = false;
    $('demo-btn').textContent = '▶ Demo: build a tower';
  }

  // $('demo-btn').onclick = run; // Task panel is commented out in index.html.
  return { run, stop: () => { if (running) { abort = true; ctx.setTaskStatus?.({ status: 'stopped', message: 'Demo stop requested.' }); } }, get running() { return running; } };
}
