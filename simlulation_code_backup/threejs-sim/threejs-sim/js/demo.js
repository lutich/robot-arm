// Demonstration: build a tower with the planner, then a physics finale.
// Every move is planned with the same planner as the Task panel and checked in a copy of the
// world before it runs. Simulation only.

import { planTransfer, verify } from './planner.js';

const SCENE = [['orange', 0, 11, 0, 0], ['blue', -9, 12, 0, 0], ['yellow', 9, 11, 0, 20], ['green', -6, 17.5, 0, -15]];
const TASKS = [['blue', 'orange'], ['yellow', 'blue']];

/** Rotation matrix (row-major) about an axis. */
function rot(axis, deg) {
  const n = Math.hypot(...axis), [x, y, z] = axis.map(v => v / n), a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}

/**
 * ctx: { ctl, world, $, log, toast, renderCubes, showPath, setPathVisible(on), moveIsClear(from, to),
 *        liveControl(), resetView() }
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
      $('cmd').value = `${pick} on ${on}`;
      const start = ctl.commands.slice();
      const plan = planTransfer(world, { pick, place: { on } }, start, { home: ctl.poses.HOME });
      const report = verify(world, plan, start, { speed: 60 });
      ctx.showPath(plan.steps, start);
      if (!report.ok) { log(`demo: "${pick} on ${on}" does not pass the simulation check, stopping`, 'err'); return false; }
      log(`demo: ${pick} on ${on} (${plan.steps.length} poses, checked)`);
      ctl.runPoses(plan.steps.map(s => s.counts), 60, plan.steps.map(s => s.label));
      await until(() => !ctl.busy && world.settled());
      await pause(0.6);
    }
    return true;
  }

  async function finale() {
    ctx.setPathVisible(false);
    // swing the arm aside, checked like every other move
    const aside = ctl.commands.slice(); aside[0] = 520;
    if (!ctx.moveIsClear(ctl.commands.slice(), aside)) { ctl.runPoses([aside], 60, ['swing aside']); await until(() => !ctl.busy); }
    log('demo: physics finale, a few cubes dropped onto the tower in real time', 'ok');
    // the drops play at 1x so they can be followed; the arm moves before ran at the chosen speed (3x by default)
    const speed = $('time-scale'), speedWas = speed.value;
    speed.value = '1';
    try {
      const top = world.cube('yellow');
      // green lands flat on top (a 4th level by gravity), red clips the top edge and falls off
      for (const [colour, x, y, z, w, wait] of [['green', top.p[0], top.p[1], top.p[2] + 9, [0, 0, 1.5], 2.0], ['red', top.p[0] + 1.6, top.p[1] + 0.4, top.p[2] + 16, [6, -5, 2], 3.0]]) {
        if (abort) return;
        const c = world.cube(colour) || world.addCube(colour, colour, x, y, 0, 0);
        Object.assign(c, { p: [x, y, z], v: [0, 0, -20], w, R: [1, 0, 0, 0, 1, 0, 0, 0, 1], state: 'awake', sleepT: 0 });
        ctx.renderCubes();
        await pause(wait);
      }
      if (abort) return;
      log('demo: a cube thrown at the tower'); spawn('orange', [-15, world.cube('yellow').p[1], 8.5], rot([0, 0, 1], 30), [160, 0, 30], [0, -8, 4]);
      await until(() => world.settled(), 8);
    } finally { speed.value = speedWas; }
    log('demo: done. "Reset" in the Scene panel restores the cubes', 'ok');
  }

  async function run() {
    if (running) { abort = true; return; }
    if (ctx.liveControl()) { ctx.toast('The demo runs in the simulation only; switch off live control first'); return; }
    running = true; abort = false;
    $('demo-btn').textContent = '■ Stop demo';
    const ghostWas = $('show-ghost').checked;
    $('show-ghost').checked = false;                     // the idle controller arm's ghost would distract
    try {
      ctl.rearm(); ctl.setCommands(ctl.poses.HOME.slice());
      world.cubes = []; world.holding = null;
      for (const [c, x, y, l, yaw] of SCENE) world.addCube(c, c, x, y, l, yaw);
      world.dirty = true; ctx.renderCubes();
      ctx.resetView();
      log('demo: building a tower', 'ok');
      await until(() => world.settled(), 5);
      if (await buildTower() && !abort) await finale();
    } catch (e) { log(`demo: ${e.message}`, 'err'); ctx.toast(e.message); }
    ctx.setPathVisible($('show-path').checked);
    $('show-ghost').checked = ghostWas;
    running = false; abort = false;
    $('demo-btn').textContent = '▶ Demo: build a tower';
  }

  $('demo-btn').onclick = run;
  return { run, stop: () => { if (running) abort = true; }, get running() { return running; } };
}
