// "Trajectory planning" section of the manual control page. The mock controller plans a task and
// rehearses it in a copy of a cube scene (POST /api/sim/plan; src/roboter_arm/sim/domain/planning);
// this section plays the result back in its own 3D view. It sends no servo command.
// Parameters, their rules and the default scene come from GET /api/sim/planning (planning.json).
// Edits here go with each plan request and stay in this browser; planning.json is the shared copy.

import { fk, anglesFromCounts, gapForTheta, gripperTheta } from '/sim_arm_view/kinematics.js';
import { CUBE } from '/sim_arm_view/config.js';
import { createArmScene } from '/sim_arm_view/scene.js';

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => v.toFixed(d).replace('-0.0', '0.0');
const DRAFT_KEY = 'roboter-arm.planning.draft.v1';
const COLOURS = ['orange', 'blue', 'yellow', 'green', 'red'];
const copy = value => JSON.parse(JSON.stringify(value));
const seconds = s => s >= 90 ? `${fmt(s / 60)} min` : `${fmt(s)} s`;

/** ctx: { onStatus(label, ok), notice(text, error), defaults }. Returns { setOpen(open), onState(state) }. */
export function start(ctx) {
  const defaults = ctx.defaults;       // { parameters, scene, rules }
  let robot = null, result = null, open = false, frameId = null, last = performance.now();
  let t = 0, playing = false, frameIndex = 0, cubesAt = [], rows = [], shownStep = -1, flight = false;

  const view = createArmScene($('plan-canvas'), document.querySelectorAll('[data-plan-view]'), 'planView');
  const pathLine = view.line(0xff7a1a);
  const waypointDots = view.dots(0xff7a1a, 0.6);

  // ---------------- draft: task, start pose, scene and parameters ----------------
  const fresh = () => ({ task: '', start: 'park', scene: copy(defaults.scene), parameters: copy(defaults.parameters) });
  let draft = fresh();
  try {
    const saved = JSON.parse(localStorage.getItem(DRAFT_KEY));
    const same = (a, b) => JSON.stringify(Object.keys(a).sort()) === JSON.stringify(Object.keys(b).sort());
    if (saved && Array.isArray(saved.scene) && saved.parameters && same(saved.parameters, defaults.parameters)
        && Object.keys(defaults.parameters).every(g => same(saved.parameters[g], defaults.parameters[g]))) draft = { ...draft, ...saved };
  } catch { /* defaults */ }
  const save = () => { try { localStorage.setItem(DRAFT_KEY, JSON.stringify(draft)); } catch { /* not kept */ } };
  $('plan-text').value = draft.task;
  $('plan-start').value = draft.start;

  // ---------------- start pose from the controller's state ----------------
  function startCounts() {
    if (!robot) return null;
    const joints = [...robot.joints].sort((a, b) => a.channel - b.channel);
    const counts = draft.start === 'current' ? [0, 1, 2, 3, 4, 5].map(c => robot.commands[c] ?? robot.commands[String(c)])
      : joints.map(j => j[draft.start]);
    return counts.length === 6 && counts.every(Number.isInteger) ? counts : null;
  }
  const homeCounts = () => {
    const home = robot ? [...robot.joints].sort((a, b) => a.channel - b.channel).map(j => j.home) : [];
    return home.length === 6 && home.every(Number.isInteger) ? home : null;
  };

  // ---------------- scene editor ----------------
  function sceneCubes() {
    const ids = new Map();
    return draft.scene.map(c => {
      const n = (ids.get(c.colour) ?? 0) + 1; ids.set(c.colour, n);
      return { id: n > 1 ? `${c.colour}${n}` : c.colour, colour: c.colour, p: [c.x, c.y, CUBE / 2 + (c.level ?? 0) * CUBE], yaw: c.yaw ?? 0 };
    });
  }
  function renderScene() {
    const table = $('plan-scene');
    table.replaceChildren();
    table.insertAdjacentHTML('beforeend', '<tr><th>Colour</th><th>x</th><th>y</th><th>Level</th><th>Yaw °</th><th></th></tr>');
    draft.scene.forEach((cube, index) => {
      const row = table.insertRow();
      const select = document.createElement('select');
      for (const colour of COLOURS) select.add(new Option(colour, colour, false, colour === cube.colour));
      select.onchange = () => edit(index, 'colour', select.value);
      row.insertCell().append(select);
      for (const [key, step, min, max] of [['x', 0.1, -15, 15], ['y', 0.1, -1.5, 36.5], ['level', 1, 0, 6], ['yaw', 1, -90, 90]]) {
        const input = Object.assign(document.createElement('input'), { type: 'number', step, min, max, value: cube[key] ?? 0 });
        input.onchange = () => { const v = Number(input.value); if (Number.isFinite(v)) edit(index, key, key === 'level' ? Math.round(v) : v); };
        row.insertCell().append(input);
      }
      const remove = Object.assign(document.createElement('button'), { type: 'button', className: 'secondary', textContent: 'Remove' });
      remove.onclick = () => { draft.scene.splice(index, 1); changed(); renderScene(); };
      row.insertCell().append(remove);
    });
  }
  function edit(index, key, value) { draft.scene[index][key] = value; changed(); }
  $('plan-scene-add').onclick = () => {
    const free = COLOURS.find(c => !draft.scene.some(s => s.colour === c)) ?? 'red';
    draft.scene.push({ colour: free, x: 0, y: 15, level: 0, yaw: 0 }); changed(); renderScene();
  };
  $('plan-scene-reset').onclick = () => { draft.scene = copy(defaults.scene); changed(); renderScene(); };

  // ---------------- parameter form, built from the rules ----------------
  function renderParameters() {
    const box = $('plan-params');
    box.replaceChildren();
    for (const [group, values] of Object.entries(defaults.parameters)) {
      const set = document.createElement('fieldset');
      set.append(Object.assign(document.createElement('legend'), { textContent: group }));
      for (const key of Object.keys(values)) {
        const rule = defaults.rules[`${group}.${key}`], value = draft.parameters[group][key];
        const label = Object.assign(document.createElement('label'), { textContent: key.replaceAll('_', ' ') });
        let input;
        if (rule.kind === 'choice') {
          input = document.createElement('select');
          for (const choice of rule.choices) input.add(new Option(choice.replace('_', ' '), choice, false, choice === value));
        } else if (rule.kind === 'flag') {
          input = Object.assign(document.createElement('input'), { type: 'checkbox', checked: value });
        } else if (rule.kind === 'numbers') {
          input = Object.assign(document.createElement('input'), { type: 'text', value: value.join(', ') });
        } else {
          input = Object.assign(document.createElement('input'), { type: 'number', min: rule.min, max: rule.max, step: rule.kind === 'integer' ? 1 : 'any', value });
        }
        input.title = rule.kind === 'choice' ? rule.choices.join(' or ') : rule.kind === 'flag' ? 'on or off'
          : `${rule.kind === 'numbers' ? 'comma-separated numbers' : rule.kind}, ${rule.min} to ${rule.max}`;
        input.onchange = () => {
          draft.parameters[group][key] = rule.kind === 'flag' ? input.checked : rule.kind === 'choice' ? input.value
            : rule.kind === 'numbers' ? input.value.split(/[\s,;]+/).filter(Boolean).map(Number) : Number(input.value);
          mark(input, group, key); changed();
        };
        mark(input, group, key);
        label.append(input); set.append(label);
      }
      box.append(set);
    }
  }
  function mark(input, group, key) {
    input.classList.toggle('changed', JSON.stringify(draft.parameters[group][key]) !== JSON.stringify(defaults.parameters[group][key]));
  }
  $('plan-params-reset').onclick = () => { draft.parameters = copy(defaults.parameters); changed(); renderParameters(); };
  $('plan-params-copy').onclick = async () => {
    const text = JSON.stringify({ ...draft.parameters, scene: draft.scene }, null, 2);
    const box = $('plan-params-json');
    box.hidden = false; box.value = text; box.select();
    try { await navigator.clipboard.writeText(text); ctx.notice('Copied. Paste it into src/roboter_arm/sim/domain/planning/planning.json to make it the default.'); }
    catch { ctx.notice('Select the text below and copy it into planning.json to make it the default.'); }
  };

  /** The scene, parameters or start changed: a shown result no longer matches them. */
  function changed() {
    save();
    if (result) clearResult('Scene, start or parameters changed; plan again to rehearse them.');
    else showStart();
  }

  // ---------------- plan ----------------
  async function plan() {
    if (flight) return;
    const counts = startCounts();
    if (!counts) { say(robot ? `The controller reports no ${draft.start === 'current' ? 'complete current commands' : draft.start.toUpperCase()}` : 'Waiting for the controller state', true); return; }
    draft.task = $('plan-text').value.trim(); save();
    if (!draft.task) { say('Type a task first.', true); return; }
    flight = true; $('plan-run').disabled = true;
    ctx.onStatus('Planning', false);
    say(`Planning "${draft.task}"…`);
    try {
      const response = await fetch('/api/sim/plan', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ task: draft.task, start: counts, home: homeCounts(), scene: draft.scene, parameters: draft.parameters }) });
      const data = await response.json().catch(() => ({}));
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
      show(data);
    } catch (error) {
      clearResult(`Not planned: ${error.message}`, true);
      ctx.onStatus('Not planned', false);
    } finally {
      flight = false; $('plan-run').disabled = false;
    }
  }

  function show(data) {
    result = data;
    cubesAt = [];
    result.frames.forEach((f, i) => cubesAt.push(f.cubes ? i : cubesAt[i - 1]));
    const grasp = result.frames.filter((_, i) => i % 2 === 0 || i === result.frames.length - 1).flatMap(f => fk(anglesFromCounts(f.counts)).grasp);
    view.setPoints(pathLine, grasp);
    view.setPoints(waypointDots, result.waypoints.flatMap(w => fk(anglesFromCounts(w.counts)).grasp));
    renderSteps();
    const { verdict } = result, hard = result.contacts.filter(k => k.kind === 'contact');
    $('plan-summary').textContent = `${result.description} · ${result.parameters.motion.mode.replace('_', ' ')} · `
      + `${result.steps.length} steps · ${seconds(result.seconds)} at ${result.parameters.motion.speed} counts/s`;
    $('plan-verdict').hidden = false;
    $('plan-verdict').textContent = `${verdict.ok ? 'Rehearsal passed' : 'Rehearsal failed'}: ${verdict.text}`;
    $('plan-verdict').classList.toggle('error', !verdict.ok);
    log(result.events.map(e => [e.t, `${fmt(e.t)} s · ${e.text}`, e.kind === 'warn' ? 'warn' : ''])
      .concat(hard.map(k => [k.t, `${fmt(k.t)} s · contact ${k.a} × ${k.b}, up to ${fmt(k.depth_mm)} mm`, 'warn']))
      .sort((a, b) => a[0] - b[0]).map(([, text, kind]) => [text, kind]));
    ctx.onStatus(verdict.ok ? 'Passed' : 'Failed', verdict.ok);
    for (const id of ['plan-play', 'plan-replay', 'plan-scrub']) $(id).disabled = false;
    seek(0); playing = true; updatePlay();
  }

  function clearResult(message, error = false) {
    result = null; playing = false; cubesAt = []; rows = []; shownStep = -1;
    view.setPoints(pathLine, []); view.setPoints(waypointDots, []);
    $('plan-verdict').hidden = !error;
    $('plan-verdict').textContent = message;
    $('plan-verdict').classList.toggle('error', error);
    $('plan-summary').textContent = '';
    $('plan-steps').replaceChildren();
    log([]);
    for (const id of ['plan-play', 'plan-replay', 'plan-scrub']) $(id).disabled = true;
    $('plan-scrub').value = '0';
    updatePlay();
    showStart();
    say(error ? 'Fix the task or the scene, then plan again.' : message);
  }

  // ---------------- views of the result ----------------
  function say(text, error = false) {
    $('plan-step').textContent = text;
    $('plan-step').classList.toggle('error', error);
  }
  function log(lines) {
    $('plan-log').replaceChildren(...lines.map(([text, kind]) => Object.assign(document.createElement('li'), { textContent: text, className: kind })));
  }
  function renderSteps() {
    const table = $('plan-steps');
    table.replaceChildren();
    table.insertAdjacentHTML('beforeend', '<tr><th>#</th><th>Step</th><th>S1</th><th>S2</th><th>S3</th><th>S4</th><th>S5</th><th>S6</th><th>s</th></tr>');
    rows = result.steps.map((step, i) => {
      const row = table.insertRow();
      for (const value of [i + 1, step.label, ...step.counts, fmt(step.seconds, 2)]) row.insertCell().textContent = value;
      row.onclick = () => seek(result.frames.find(f => f.step === i + 1)?.t ?? 0);
      return row;
    });
  }
  function showStart() {
    const counts = startCounts();
    if (!result) {
      say(counts ? 'Type a task and press Plan & rehearse.' : robot ? `The controller reports no ${draft.start.toUpperCase()} pose.` : 'Waiting for the controller state…', !counts && !!robot);
      if (open) ctx.onStatus(counts ? 'Ready' : 'Waiting', !!counts);
    }
    view.invalidate();
  }

  // ---------------- playback ----------------
  const duration = () => result?.frames.at(-1).t ?? 0;
  function seek(time) {
    t = Math.max(0, Math.min(duration(), time));
    frameIndex = 0;
    view.invalidate();
  }
  function updatePlay() { $('plan-play').textContent = playing ? 'Pause' : 'Play'; }
  $('plan-play').onclick = () => { if (!playing && t >= duration()) seek(0); playing = !playing; updatePlay(); };
  $('plan-replay').onclick = () => { seek(0); playing = true; updatePlay(); };
  $('plan-scrub').oninput = () => { seek(duration() * $('plan-scrub').value / 1000); playing = false; updatePlay(); };
  $('plan-show-path').onchange = () => { pathLine.visible = waypointDots.visible = $('plan-show-path').checked; view.invalidate(); };
  $('plan-start').onchange = () => { draft.start = $('plan-start').value; changed(); };
  $('plan-form').addEventListener('submit', event => { event.preventDefault(); void plan(); });

  function currentFrame() {
    const frames = result.frames;
    if (frames[frameIndex].t > t) frameIndex = 0;
    while (frameIndex + 1 < frames.length && frames[frameIndex + 1].t <= t) frameIndex++;
    return frameIndex;
  }

  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (result && playing) {
      t = Math.min(duration(), t + dt * Number($('plan-speed').value));
      if (t >= duration()) { playing = false; updatePlay(); }
      view.invalidate();
    }
    view.draw(() => {
      if (result) {
        const i = currentFrame(), f = result.frames[i];
        view.arm.setPose(anglesFromCounts(f.counts), f.gap);
        view.syncCubes(result.frames[cubesAt[i]].cubes);
        $('plan-scrub').value = String(Math.round(1000 * t / (duration() || 1)));
        if (f.step !== shownStep) {
          rows[shownStep - 1]?.classList.remove('on');
          rows[f.step - 1]?.classList.add('on');
          shownStep = f.step;
        }
        const step = result.steps[f.step - 1];
        say(`${fmt(t)} / ${fmt(duration())} s · step ${f.step}/${result.steps.length}${step ? ` · ${step.label}` : ' · start'}`);
      } else {
        const counts = startCounts();
        if (counts) view.arm.setPose(anglesFromCounts(counts), gapForTheta(gripperTheta(counts[5])));
        view.syncCubes(sceneCubes());
      }
    });
    frameId = open ? requestAnimationFrame(frame) : null;
  }

  function setOpen(value) {
    open = value;
    if (open) {
      view.resize();
      showStart();
      if (result) ctx.onStatus(result.verdict.ok ? 'Passed' : 'Failed', result.verdict.ok);
      if (frameId === null) { last = performance.now(); frameId = requestAnimationFrame(frame); }
    }
  }

  renderScene();
  renderParameters();
  return {
    setOpen,
    // the controller's /api/state, which the page already reads: PARK, HOME and current commands
    onState(state) {
      const before = JSON.stringify(startCounts());
      robot = state;
      const current = $('plan-start').querySelector('[value="current"]');
      current.disabled = !(state && [0, 1, 2, 3, 4, 5].every(c => Number.isInteger(state.commands?.[c] ?? state.commands?.[String(c)])));
      if (!result && JSON.stringify(startCounts()) !== before) showStart();
    },
  };
}
