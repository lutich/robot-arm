// Task plan panel (inside Live task status). A plan comes from the task box here or from
// Claude through the plan channel; it is simulated live in this page right away, and only
// after a passed simulation can the operator run it on the armed robot, one confirmed
// servo step at a time. The solid arm shows the plan; the ghost arm shows the live robot.
import { cubeTilt } from './physics.js';
import { World } from './world.js';
import { buildTaskPlan } from '../src/agent/task-plan.js';

const PLAN_URL = '/simulator/plan';
const PROGRESS_URL = '/simulator/plan/progress';
const ROBOT_SPEED = 20; // counts/s, the controller's move speed
const OWNS_ARM = new Set(['simulating', 'simulated', 'sim-failed', 'running-live']);
const LABEL = {
  idle: 'no plan', simulating: 'simulating', simulated: 'sim passed', 'sim-failed': 'sim failed', stale: 'stale',
  'running-live': 'running live', 'live-complete': 'live done', 'live-halted': 'halted', 'live-failed': 'live failed',
};
const BADGE = {
  idle: 'idle', simulating: 'moving', simulated: 'complete', 'sim-failed': 'error', stale: 'blocked',
  'running-live': 'moving', 'live-complete': 'complete', 'live-halted': 'stopped', 'live-failed': 'error',
};

const sameCounts = (a, b) => a.length === b.length && a.every((value, index) => value === b[index]);
/** "S5 250 (plan 480), S6 550 (plan 650)": where the robot differs from the plan start. */
const startDifference = (live, start) => live.map((count, index) => (count === start[index] ? null : `S${index + 1} ${count} (plan ${start[index]})`))
  .filter(Boolean).join(', ');
const duration = (seconds) => seconds >= 90 ? `${(seconds / 60).toFixed(1)} min` : `${Math.round(seconds)} s`;

export function initPlanRunner(ctx) {
  const { $, ctl, world, robotClient, log, toast } = ctx;
  let plan = null; let revision = 0; let channel = null; // channel: null unknown, true/false available
  let phase = 'idle'; let simPassed = false; let staleReason = null; let haltRequested = false;
  let done = 0; let simToken = 0; let lastReport = 0; let outcome = ''; let publishing = false;
  let holdResultUntil = 0;
  const total = () => plan?.steps.length ?? 0;
  const finished = () => phase === 'simulated' || phase === 'sim-failed';
  /**
   * Whether the plan drives the solid arm. With Live Mode connected the view mirrors the robot,
   * so a finished simulation holds its result briefly, then hands the arm back (Replay shows it again).
   */
  const ownsArm = () => phase === 'simulating' || phase === 'running-live'
    || (finished() && (!robotClient.connected || performance.now() < holdResultUntil));

  /** Back to reality: the pre-plan scene and the robot's commanded pose. */
  function releaseToLive() {
    world.restore(plan.scene);
    void ctx.rebuildRapier().then(() => ctx.renderCubes());
    ctx.resumeLiveMirror();
    render();
  }

  // ---------------- channel ----------------

  async function report(extra = {}, force = false) {
    if (!channel || !plan?.revision) return;
    const now = performance.now();
    if (!force && now - lastReport < 1000) return;
    lastReport = now;
    try {
      await fetch(PROGRESS_URL, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({
        revision: plan.revision, phase: phase === 'idle' ? 'stale' : phase, step: done, total: total(),
        message: `${LABEL[phase]} ${done}/${total()}${outcome ? `: ${outcome}` : ''}${staleReason ? ` (${staleReason})` : ''}`, ...extra,
      }) });
    } catch { /* gateway restarting; the next report catches up */ }
  }

  async function poll() {
    if (channel === false || publishing) return;
    try {
      const response = await fetch(PLAN_URL, { cache: 'no-store' });
      if (!response.ok || !String(response.headers.get('content-type')).includes('application/json')) { channel = false; render(); return; }
      channel = true;
      const data = await response.json();
      if (data.plan && data.revision > revision) {
        revision = data.revision;
        void load({ ...data.plan, revision: data.revision }, 'Claude');
      }
    } catch { /* server restarting */ }
  }

  /** Share a task-box plan through the channel, so Claude sees it and its progress; polling must not reload it. */
  async function publish(next) {
    if (!channel) return next;
    publishing = true;
    try {
      const response = await fetch(PLAN_URL, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(next) });
      if (!response.ok) return next;
      const result = await response.json();
      revision = Math.max(revision, result.revision);
      return { ...next, revision: result.revision };
    } catch { return next; }
    finally { publishing = false; }
  }

  // ---------------- planning and simulation ----------------

  /** The scene a new plan starts from: the pre-simulation scene while a plan owns the view. */
  const baseScene = () => (plan && OWNS_ARM.has(phase) ? plan.scene : world.snapshot());

  async function planFromText(text) {
    if (!text.trim()) return;
    if (phase === 'running-live') { toast('Wait for the live run to finish'); return; }
    const start = robotClient.connected && robotClient.state ? ctx.backendCounts(robotClient.state) : ctl.commands.slice();
    if (!start.every(Number.isInteger)) { toast('The robot did not report a complete commanded pose'); return; }
    const scene = new World();
    scene.restore(baseScene());
    const result = buildTaskPlan({ spec: ctx.canonicalSpec, world: scene, start, text });
    if (!result.success) {
      toast(result.reason);
      log(`plan "${text}": ${result.reason}`, 'err');
      void ctx.setTaskStatus({ task: text, destination: text, status: 'blocked', message: result.reason });
      return;
    }
    await load(await publish(result.plan), 'task box');
  }

  async function load(next, source) {
    if (phase === 'running-live') { log(`plan "${next.task}" from ${source} ignored: a live run is in progress`, 'warn'); return; }
    plan = next; staleReason = null; outcome = '';
    log(`plan from ${source}: "${plan.task}", ${total()} single-servo steps${plan.precheck ? `, pre-check ${plan.precheck.ok ? 'passed' : 'FAILED'}` : ''}`, plan.precheck?.ok === false ? 'warn' : 'ok');
    void ctx.setTaskStatus({ task: plan.task, destination: plan.description ?? plan.task, status: 'planning',
      message: `Plan from ${source}: ${total()} single-servo steps (~${duration(plan.estimatedSeconds ?? 0)} on the robot). Simulating now.` });
    await simulate();
  }

  async function simulate() {
    if (!plan || phase === 'running-live') return;
    const token = ++simToken;
    phase = 'simulating'; simPassed = false; done = 0; outcome = '';
    ctl.setCommands(plan.start); ctl.rearm();
    world.restore(plan.scene); world.events = [];
    await ctx.rebuildRapier();
    if (token !== simToken) return;
    ctx.renderCubes(); ctx.showPath(plan.steps, plan.start);
    render(); void report({}, true);
    const factor = Number($('plan-speed').value) || 1;
    try { ctl.runPoses(plan.steps.map((step) => step.counts), ROBOT_SPEED * factor, plan.steps.map((step) => step.label)); }
    catch (error) { finishSimulation(token, { ok: false, text: error.message }); return; }
    const off = ctl.on((kind, detail) => {
      if (token !== simToken) { off(); return; }
      if (kind === 'settled') { done += 1; render(); void report(); }
      else if (kind === 'stop') { off(); finishSimulation(token, { ok: false, text: `stopped: ${detail}` }); }
      else if (kind === 'idle') { off(); void judgeWhenSettled(token); }
    });
  }

  async function judgeWhenSettled(token) {
    const until = performance.now() + 4000;
    while (!ctx.sceneSettled() && performance.now() < until) await new Promise((resolve) => setTimeout(resolve, 100));
    if (token === simToken) finishSimulation(token, judge());
  }

  function judge() {
    const cube = world.cube(plan.expect.cube);
    if (!cube) return { ok: false, text: `the ${plan.expect.cube} cube is missing` };
    if (plan.expect.type === 'pick') {
      return world.holding?.id === cube.id ? { ok: true, text: `${cube.id} is held` } : { ok: false, text: `${cube.id} was not held` };
    }
    const [x, y, z] = plan.expect.target;
    const offset = Math.hypot(cube.p[0] - x, cube.p[1] - y);
    const ok = cube.state !== 'held' && offset < 0.9 && Math.abs(cube.p[2] - z) < 0.3 && cubeTilt(cube) < 10;
    const where = `${(offset * 10).toFixed(1)} mm from target`;
    return { ok, offsetCm: offset, text: ok ? `${cube.id} placed ${where}` : `${cube.id} ended ${where}${cube.state === 'held' ? ', still held' : ''}` };
  }

  function finishSimulation(token, result) {
    if (token !== simToken) return;
    phase = result.ok ? 'simulated' : 'sim-failed';
    simPassed = result.ok; outcome = result.text;
    holdResultUntil = performance.now() + 2500;
    setTimeout(() => { if (token === simToken && finished() && robotClient.connected) releaseToLive(); }, 2600);
    render(); void report({ ok: result.ok, offsetCm: result.offsetCm }, true);
    log(`simulation of "${plan.task}": ${result.text}`, result.ok ? 'ok' : 'err');
    void ctx.setTaskStatus({ status: result.ok ? 'ready' : 'blocked', message: result.ok
      ? `Simulation passed: ${result.text}. ${robotClient.mode === 'operational-armed' ? 'Run live is available.' : 'Arm Live Mode to enable Run live.'}`
      : `Simulation failed: ${result.text}. Not runnable on the robot.` });
  }

  // ---------------- live run ----------------

  function liveBlockers() {
    const reasons = [];
    if (!plan) return ['no plan'];
    if (phase === 'running-live') reasons.push('live run in progress');
    if (staleReason) reasons.push(`plan is stale: ${staleReason}`);
    else if (!simPassed) reasons.push(phase === 'simulating' ? 'simulation still running' : 'simulation has not passed');
    if (robotClient.mode !== 'operational-armed') reasons.push('Live Mode is not armed');
    const live = robotClient.connected && robotClient.state ? ctx.backendCounts(robotClient.state) : null;
    if (!staleReason && live?.every(Number.isInteger) && !sameCounts(live, plan.start)) {
      reasons.push(`the robot is not at the plan start: ${startDifference(live, plan.start)}`);
    }
    return reasons;
  }

  async function runLive() {
    const blockers = liveBlockers();
    if (blockers.length) { toast(`Run live blocked: ${blockers[0]}`); return; }
    try { await robotClient.refresh(); }
    catch (error) { toast(`Robot state unavailable: ${error.message}`); return; }
    const live = ctx.backendCounts(robotClient.state);
    if (!sameCounts(live, plan.start)) { markStale(`the robot is not at the plan start: ${startDifference(live, plan.start)}`); return; }
    phase = 'running-live'; haltRequested = false; done = 0; outcome = ''; simToken += 1;
    ctl.setCommands(plan.start); ctl.rearm();
    world.restore(plan.scene); world.events = [];
    await ctx.rebuildRapier();
    ctx.renderCubes(); ctx.showPath(plan.steps, plan.start);
    render(); void report({}, true);
    void ctx.setTaskStatus({ status: 'moving', message: `Running ${total()} single-servo steps on the robot; each is confirmed. Halt holds the pose, STOP cuts the outputs.` });
    log(`live run of "${plan.task}" started: ${total()} steps`, 'warn');
    const path = [plan.start, ...plan.steps.map((step) => step.counts)].map((counts) => ctx.countsObject(counts));
    try {
      const result = await robotClient.executeSingleJointPath(ctx.runtimeSpec(), path, {
        shouldAbort: () => haltRequested,
        onStep: (event) => {
          if (event.phase === 'sending') {
            // The solid arm animates the step at robot speed; the ghost shows what the robot reports.
            if (ctl.busy) ctl.setCommands(plan.steps[event.index - 2]?.counts ?? plan.start);
            try { ctl.runMoves([{ ch: event.channel, target: event.target }], [plan.steps[event.index - 1].label]); } catch { /* display only */ }
          } else { done = event.index; render(); void report(); }
        },
      });
      phase = result?.aborted ? 'live-halted' : 'live-complete';
      outcome = result?.aborted ? `halted after step ${done}/${total()}; the robot holds its pose` : `all ${total()} steps confirmed by the robot`;
    } catch (error) {
      phase = 'live-failed'; outcome = error.message;
    }
    simPassed = false; // the robot has left the plan start; a new plan is needed
    render(); void report({ ok: phase === 'live-complete' }, true);
    log(`live run of "${plan.task}": ${outcome}`, phase === 'live-complete' ? 'ok' : 'err');
    void ctx.setTaskStatus({ status: phase === 'live-complete' ? 'complete' : phase === 'live-halted' ? 'stopped' : 'error', message: `Live run: ${outcome}` });
  }

  // ---------------- invalidation ----------------

  function cancelSimulation() {
    simToken += 1;
    if (ctl.busy) ctl.setCommands(ctl.commands.slice());
  }

  /** Something changed in reality after planning: Run live needs a new plan. */
  function markStale(reason) {
    if (!plan || phase === 'running-live' || staleReason) return;
    staleReason = reason; simPassed = false;
    if (OWNS_ARM.has(phase)) {
      cancelSimulation();
      world.restore(plan.scene); // back to the pre-simulation scene
      void ctx.rebuildRapier().then(() => ctx.renderCubes());
    }
    phase = 'stale';
    render(); void report({}, true);
    log(`plan "${plan.task}" is stale: ${reason}`, 'warn');
  }

  /** Stop any simulation, restore the pre-plan scene and reset the status, ready for another task. */
  function clear() {
    if (!plan || phase === 'running-live') return;
    if (OWNS_ARM.has(phase)) {
      cancelSimulation();
      world.restore(plan.scene);
      void ctx.rebuildRapier().then(() => ctx.renderCubes());
    }
    outcome = 'cleared by the operator';
    void report({ phase: 'cleared' }, true); // body is built before plan is reset below
    log(`plan "${plan.task}" cleared`);
    plan = null; phase = 'idle'; simPassed = false; staleReason = null; outcome = ''; done = 0;
    ctx.showPath([], ctl.commands);
    void ctx.setTaskStatus({ task: 'No active task', destination: '—', status: 'idle', message: 'Plan cleared. Type another task.' });
    render();
  }

  // ---------------- view ----------------

  function render() {
    $('plan-phase').textContent = LABEL[phase];
    $('plan-phase').className = `task-state ${BADGE[phase]}`;
    $('plan-summary').textContent = plan
      ? `${plan.description ?? plan.task} · ${total()} single-servo steps · ~${duration(plan.estimatedSeconds ?? 0)} on the robot${plan.precheck ? ` · pre-check ${plan.precheck.ok ? 'passed' : 'FAILED'}` : ''}`
      : channel === false
        ? 'Type a task above. (Plans from Claude need the gateway or dev server plan channel.)'
        : `Type a task above, or ask Claude in chat. The plan simulates here immediately.${robotClient.connected ? ''
          : ' To run it on the robot later, connect Live Mode first so the plan starts from the robot’s pose.'}`;
    $('plan-progress-bar').style.width = `${total() ? (100 * done) / total() : 0}%`;
    const stepLabel = OWNS_ARM.has(phase) && ctl.motion?.label ? ` · ${ctl.motion.label}` : '';
    $('plan-step').textContent = plan ? `Step ${done}/${total()}${stepLabel}${outcome ? ` · ${outcome}` : ''}${staleReason ? ` · stale: ${staleReason}` : ''}` : '';
    $('plan-replay').disabled = !plan || phase === 'running-live';
    $('plan-halt').disabled = !['simulating', 'running-live'].includes(phase);
    $('plan-clear').disabled = !plan || phase === 'running-live';
    const blockers = liveBlockers();
    $('plan-run-live').disabled = blockers.length > 0;
    $('plan-gate').textContent = !plan ? '' : blockers.length
      ? `Run live needs: ${blockers.join(' · ')}${staleReason || blockers.some((reason) => reason.startsWith('the robot is not'))
        ? '. Press Plan & simulate to re-plan from the robot’s current pose.' : ''}`
      : 'Ready: simulation passed, Live Armed, robot at the plan start. Stay at the power switch.';
  }

  $('plan-form').addEventListener('submit', (event) => { event.preventDefault(); void planFromText($('plan-text').value); });
  $('plan-replay').onclick = () => { void simulate(); }; // a stale plan stays stale; replay is for watching
  $('plan-run-live').onclick = () => { void runLive(); };
  $('plan-halt').onclick = () => {
    if (phase === 'running-live') { haltRequested = true; outcome = 'halt requested; finishing the current step'; render(); }
    else if (phase === 'simulating') { cancelSimulation(); finishSimulation(simToken, { ok: false, text: 'halted by the operator' }); }
  };
  $('plan-clear').onclick = clear;
  robotClient.addEventListener('modechange', () => {
    // Connecting after a simulation: the view switches from the result to the live robot.
    if (plan && finished() && robotClient.connected && performance.now() >= holdResultUntil) releaseToLive();
    else render();
  });
  render();
  void poll();
  setInterval(poll, 700);

  return {
    /** While true, the solid arm shows the plan and must not mirror the live robot. */
    get ownsArm() { return ownsArm(); },
    /** The cube this plan picks, while the plan drives the solid arm (Rapier holds it still for the grasp). */
    get pickCube() { return ownsArm() ? plan?.expect.cube ?? null : null; },
    markStale,
    onBackendState(counts) {
      if (plan && ['simulating', 'simulated', 'sim-failed'].includes(phase) && !sameCounts(counts, plan.start)) {
        markStale(`the robot is not at the plan start: ${startDifference(counts, plan.start)}`);
      } else render();
    },
  };
}
