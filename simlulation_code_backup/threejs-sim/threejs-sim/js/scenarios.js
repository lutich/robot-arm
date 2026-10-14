// Scenarios: a block layout plus the steps to execute, predefined or with the cubes the camera sees.
//
// A step is a task in the Task panel's language ("blue on orange", "yellow to 0 14"), which the planner
// turns into a trajectory, or a stored trajectory (named poses in counts, like a demo on the controller).
// Reset puts the cubes into the shared simulation (and the simulated arm HOME); Run sends every step as
// robot API calls to the active backend, so the same scenario runs on the simulation or on the real arm.

import { planTransfer, verify, toMoves, parseCommand } from './planner.js';
import { movesToCalls } from './armapi.js';

// The team's hand-taught pick and place (docs/controller-live-2026-10-12.md).
export const TAUGHT = { name: 'step1-hacks-10-direct-approach', positions: [
  ['HOME start', [320, 427, 403, 220, 250, 550]], ['Above orange, open', [440, 427, 360, 185, 340, 490]],
  ['Single approach, open', [440, 347, 360, 185, 340, 490]], ['Close gripper', [440, 347, 360, 185, 340, 590]],
  ['Lift cube', [440, 427, 360, 185, 340, 590]], ['Transfer', [230, 427, 360, 185, 340, 590]],
  ['Lower to destination', [235, 347, 360, 185, 380, 590]], ['Release cube', [235, 347, 360, 185, 380, 490]],
  ['Withdraw', [235, 407, 360, 185, 380, 490]], ['HOME finish', [320, 427, 403, 220, 250, 550]]].map(([name, counts]) => ({ name, counts })) };

/** Cubes: [colour, x, y, level, yaw] in the board frame (cm, deg). */
export const SCENARIOS = [
  { id: 'tower', name: 'Tower of three', about: 'Blue onto orange, then yellow onto blue.',
    cubes: [['orange', 0, 14, 0, 0], ['blue', -8, 12, 0, 0], ['yellow', 8, 12, 0, 0], ['green', -5, 18, 0, 0]],
    steps: [{ task: 'blue on orange' }, { task: 'yellow on blue' }] },
  { id: 'two-stacks', name: 'Two stacks', about: 'Yellow onto orange, green onto blue.',
    cubes: [['orange', -6, 13, 0, 0], ['blue', 6, 13, 0, 0], ['yellow', -3, 18, 0, 0], ['green', 4, 18, 0, 0]],
    steps: [{ task: 'yellow on orange' }, { task: 'green on blue' }] },
  { id: 'line-up', name: 'Line up', about: 'Three cubes into a row at y = 13.',
    cubes: [['orange', -9, 16, 0, 15], ['blue', 2, 18, 0, -10], ['yellow', 9, 10, 0, 0]],
    steps: [{ task: 'orange to -5 13' }, { task: 'blue to 0 13' }, { task: 'yellow to 5 13' }] },
  { id: 'taught', name: "Team's taught pick and place", about: 'The poses taught on the controller (step1-hacks-10-direct-approach), played as they are.',
    cubes: [['orange', -11.9, 14.5, 0, -16]],
    steps: [{ trajectory: TAUGHT }] },
  { id: 'show', name: 'Tower and physics show (sim only)', about: 'Builds a tower, then drops and throws cubes at it. Simulation only.',
    cubes: [['orange', 0, 11, 0, 0], ['blue', -9, 12, 0, 0], ['yellow', 9, 11, 0, 20], ['green', -6, 17.5, 0, -15]], show: true },
];

/**
 * ctx: { ctl, world, $, log, toast, renderCubes, showPath, confirmReal, demo, send(calls), setScene(cubes),
 *        serverMode(), onReal(), counts(), idle(), detections(), useDetections(list) }
 * Steps are planned on the current (mirrored) world from the arm's current counts and sent as robot API
 * calls to the active backend: the shared simulation, or the real arm (each call simulated first there).
 */
export function initScenarios(ctx) {
  const { ctl, world, $, log, toast } = ctx;
  let scn = SCENARIOS[0], source = 'predefined';
  let run = null, busy = false, abort = false;   // run: { step, status: 'running' | 'done' | 'failed' }
  let resetting = null;                           // the reset in progress (a promise)
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const settle = async () => { await sleep(400); for (let i = 0; i < 300 && !abort && !ctx.idle(); i++) await sleep(100); };

  $('scn-select').innerHTML = SCENARIOS.map(s => `<option value="${s.id}">${s.name}</option>`).join('');
  const stepText = s => s.task || `trajectory "${s.trajectory.name}" (${s.trajectory.positions.length} poses)`;
  const stepsOf = () => (source === 'live' ? $('scn-tasks').value.split('\n').map(t => t.trim()).filter(Boolean).map(task => ({ task })) : (scn.steps || []));

  function render() {
    const live = source === 'live';
    $('scn-source-pre').classList.toggle('on', !live);
    $('scn-source-live').classList.toggle('on', live);
    $('scn-pre').hidden = live;
    $('scn-live').hidden = !live;
    $('scn-about').textContent = live ? '' : scn.about;
    const steps = stepsOf();
    $('scn-steps').innerHTML = scn.show && !live ? '<li>Tower with the planner, then the physics show (local view only)</li>'
      : steps.map((s, i) => `<li class="${run && run.step > i ? 'done' : run && run.step === i ? (run.status === 'failed' ? 'bad' : run.status === 'running' ? 'cur' : '') : ''}">${stepText(s)}</li>`).join('');
    $('scn-sim').textContent = busy ? '■ Stop' : ctx.onReal() ? '▶ Run on REAL arm' : '▶ Run';
    $('scn-sim').classList.toggle('danger', ctx.onReal() && !busy);
    $('scn-state').textContent = resetting ? 'resetting…' : busy && run ? `step ${run.step + 1} running on the ${ctx.onReal() ? 'real arm' : 'simulation'}…`
      : !run ? `↺ Reset puts the cubes${ctx.onReal() ? ' of the simulation' : ' and the arm'} in place.`
      : run.status === 'failed' ? `step ${run.step + 1} failed.` : run.status === 'done' ? 'Done.' : '';
  }

  function reset() {
    if (resetting) return resetting;
    abort = true; ctx.demo.stop();
    resetting = doReset().finally(() => { resetting = null; render(); });
    render();
    return resetting;
  }
  async function doReset() {
    while (busy) await sleep(50);   // a running scenario stops first (abort)
    try {
      if (source === 'live') {
        const list = ctx.detections().filter(d => d.ok);
        if (!list.length) toast('No cubes from the camera yet: Camera tab → load or grab a frame → Detect cubes.');
        else ctx.useDetections(list);
      } else await ctx.setScene(scn.cubes);
      // the simulated arm goes HOME (power on if a collision stopped it); a reset never moves the real arm
      if (!ctx.onReal()) {
        if (ctl.outputsOff) await ctx.send([{ op: 'power_on', label: 'power on (PARK → HOME)' }]);
        else await ctx.send([{ op: 'go_pose', name: 'HOME', label: 'go HOME' }]);
      }
    } catch (e) { toast(e.message); }
    run = null;
    log(`scenario: ${source === 'live' ? 'cubes from the camera' : scn.name}, reset`);
  }

  async function runAll() {
    if (busy) { abort = true; ctx.stopCalls(); return; }   // stops after the current call; STOP in the header stops at once
    if (resetting) await resetting;
    if (scn.show && source !== 'live') { if (ctx.onReal()) { toast('The physics show runs in the local view only.'); return; } ctx.demo.run(); return; }
    const steps = stepsOf();
    if (!steps.length) { toast('No steps to run.'); return; }
    if (ctx.onReal() && !await ctx.confirmReal(`Run "${source === 'live' ? 'camera scenario' : scn.name}" on the real arm?`,
      `${steps.length} step${steps.length > 1 ? 's' : ''}: ${steps.map(stepText).join(', ')}. Each step is computed in the simulation from the arm's current pose; the server simulates every call again before it moves the arm. ${source === 'live' ? 'The cubes are where the camera saw them.' : 'Place the real cubes as in the simulation.'}`)) return;
    busy = true; abort = false;
    run = { step: 0, status: 'running' };
    render();
    try {
      for (let i = 0; i < steps.length && !abort; i++) {
        run.step = i; render();
        await settle();
        const s = steps[i], cur = ctx.counts();
        const plan = s.task
          ? planTransfer(world, parseCommand(s.task, world), cur, { home: ctl.poses.HOME, step: 0.25 })
          : { steps: s.trajectory.positions.map(p => ({ label: p.name, counts: p.counts.slice() })) };
        const moves = toMoves(world, plan, cur).moves;
        const report = verify(world, plan, cur, { mode: 'moves', moves });
        ctx.showPath(plan.steps, cur);
        if (!report.ok) throw new Error(`"${stepText(s)}" does not pass the simulation check${report.contacts.length ? `: ${report.contacts.map(k => `${k.a} × ${k.b}`).join(', ')}` : ''}`);
        log(`scenario step ${i + 1}: ${stepText(s)} (${moves.length} moves)`);
        await ctx.send(movesToCalls(moves));
        run.step = i + 1;
      }
      await settle();
      run.status = abort ? 'failed' : 'done';
      if (!abort) log(`scenario done on the ${ctx.onReal() ? 'real arm' : 'shared simulation'}`, 'ok');
    } catch (e) { run.status = 'failed'; log(`scenario: ${e.message}`, 'err'); toast(e.message); }
    busy = false; abort = false;
    render();
  }

  $('scn-select').onchange = () => { scn = SCENARIOS.find(s => s.id === $('scn-select').value); reset(); };
  $('scn-source-pre').onclick = () => { source = 'predefined'; reset(); };
  $('scn-source-live').onclick = () => { source = 'live'; run = null; render(); };
  $('scn-tasks').oninput = () => { run = null; render(); };
  $('scn-reset').onclick = reset;
  $('scn-sim').onclick = runAll;
  render();
  return { render, reset, get scenario() { return scn; } };
}
