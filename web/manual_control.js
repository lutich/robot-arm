'use strict';

const $ = id => document.getElementById(id);
const names = ['Base rotation', 'Shoulder', 'Elbow', 'Wrist flexion', 'Wrist rotation', 'Gripper'];
let state = null, connected = false, selected = 0, picked = -1, pending = 0;
let pollFlight = null, renderedList = '', renderedBounds = '', renderedSettings = '', lastError = null;

function notice(message, error = false) {
  $('notice').textContent = message;
  $('notice').classList.toggle('error', error);
}

function stage(value) {
  $('target').value = value ?? '';
  $('slider').value = value ?? $('slider').min;
}

function joint() {
  return state?.joints.find(item => item.channel === selected);
}

function choose(channel) {
  const item = state?.joints.find(candidate => candidate.channel === channel);
  if (!item?.allowed) return;
  selected = channel;
  syncLimits(true);
  stage(state.commands[channel] ?? item.home);
  render();
}

function syncLimits(force = false) {
  const item = joint();
  if (!item) return;
  const boundsKey = JSON.stringify([selected, item.low, item.high]);
  for (const input of [$('slider'), $('target')]) {
    input.min = item.low;
    input.max = item.high;
  }
  for (const [id, value] of [['min-limit', item.low], ['max-limit', item.high]]) {
    const input = $(id);
    input.min = item.historical_low;
    input.max = item.historical_high;
    if (force || boundsKey !== renderedBounds) input.value = value;
  }
  renderedBounds = boundsKey;
}

function syncSettings() {
  const settings = state.settings, key = JSON.stringify([settings.step_size, settings.demo_speed]);
  $('demo-speed').max = settings.demo_speed_max;
  if (key !== renderedSettings) {
    $('step-size').value = settings.step_size;
    $('demo-speed').value = settings.demo_speed;
  }
  renderedSettings = key;
}

async function request(path, args) {
  const response = await fetch('/api/' + path, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(args)
  });
  const data = await response.json();
  if (!response.ok) throw Error(data.error || 'Request failed');
  return data;
}

async function action(path, args = {}, success = '', after = null) {
  pending++;
  render();
  try {
    const data = await request(path, args);
    if (after) after(data);
    if (success) notice(typeof success === 'function' ? success(data) : success);
    if (pollFlight) await pollFlight;
    await poll();
    return data;
  } catch (error) {
    if (path === 'power_on') $('ready').checked = false;
    notice(error.message, true);
    return null;
  } finally {
    pending--;
    render();
  }
}

function renderDemo(canMove, editing) {
  const demo = state.demo, positions = demo.positions;
  if (picked >= positions.length) picked = positions.length - 1;
  const hasPicked = picked >= 0 && picked < positions.length;
  const running = ['running', 'pausing'].includes(demo.status);
  const allValid = positions.length > 0 && positions.every(position => position.valid);
  const listKey = JSON.stringify([positions, demo.cursor, demo.status, picked]);
  $('position-count').textContent = '(' + positions.length + ' position' + (positions.length === 1 ? '' : 's') + ')';
  if (listKey !== renderedList) {
    renderedList = listKey;
    $('demo-list').replaceChildren();
    if (!positions.length) {
      const empty = document.createElement('li');
      empty.className = 'empty-demo';
      empty.textContent = 'Move the arm, then add a position.';
      $('demo-list').append(empty);
    }
    positions.forEach((position, index) => {
      const li = document.createElement('li'), button = document.createElement('button');
      const number = document.createElement('span'), name = document.createElement('span'), status = document.createElement('span');
      const moving = running && index === demo.cursor, done = index < demo.cursor;
      button.className = 'step-row' + (index === picked ? ' selected' : '') + (moving ? ' moving' : done ? ' done' : '') + (!position.valid ? ' invalid' : '');
      button.setAttribute('aria-pressed', index === picked);
      number.className = 'step-number';
      number.textContent = done ? '✓' : index + 1;
      name.className = 'step-name';
      name.textContent = position.name;
      status.className = 'step-status';
      status.textContent = !position.valid ? 'Check limits' : moving ? 'Moving' : done ? 'Done' : index === demo.cursor ? 'Next' : 'Pending';
      if (position.error) button.title = position.error;
      button.append(number, name, status);
      button.onclick = () => { picked = index; render(); };
      li.append(button);
      $('demo-list').append(li);
    });
  }
  const position = hasPicked ? positions[picked] : null;
  if (document.activeElement !== $('step-name')) $('step-name').value = position?.name ?? '';
  $('step-name').disabled = !editing || !hasPicked;
  $('step-up').disabled = !editing || !hasPicked || picked === 0;
  $('step-down').disabled = !editing || !hasPicked || picked === positions.length - 1;
  $('step-remove').disabled = !editing || !hasPicked;
  $('replace-position').disabled = !canMove || !hasPicked;
  $('step-counts').textContent = position ? names.map((name, channel) => name + ': ' + position.counts[channel]).join(' · ') : 'Select a position.';
  $('next-step').disabled = !canMove || !allValid || demo.cursor >= positions.length || demo.status === 'paused';
  $('run-all').disabled = !canMove || !allValid;
  $('run-all').textContent = demo.status === 'paused' ? 'Resume' : demo.status === 'complete' ? 'Run again' : 'Run all once';
  $('run-all').title = demo.status === 'paused' ? 'Continue from the next position' : 'Start at position 1 and run the entire sequence once';
  $('pause').hidden = !running || demo.mode !== 'all';
  $('pause').disabled = !connected || demo.pause_requested || pending > 0;
  $('pause').textContent = demo.pause_requested ? 'Pausing after this step…' : 'Pause after step';
  $('restart').disabled = !editing || !positions.length;
  $('save-demo').disabled = !editing || !positions.length;
  $('load-demo').disabled = !editing;
  $('demo-name').disabled = !editing;
  if (document.activeElement !== $('demo-name')) $('demo-name').value = demo.name;
  $('demo-status').textContent = running ? 'Position ' + (demo.cursor + 1) + ' of ' + positions.length + ' · Moving' :
    demo.status === 'paused' ? 'Paused · ' + demo.cursor + ' of ' + positions.length + ' completed' :
    demo.status === 'complete' ? 'Complete · ' + demo.cursor + ' of ' + positions.length :
    positions.length ? demo.cursor + ' of ' + positions.length + ' completed' : 'No positions recorded';
}

function render() {
  if (!state) return;
  const item = joint(), on = state.armed || state.busy || state.outputs_off === false;
  const idle = connected && !state.busy && pending === 0;
  const canMove = idle && state.armed && item.allowed;
  const complete = state.joints.every(candidate => candidate.allowed && Number.isInteger(state.commands[candidate.channel]));
  $('state-label').textContent = !connected ? 'Offline' : state.phase || (on ? 'On' : 'Off');
  $('init-state').classList.toggle('on', connected && on);
  $('power').textContent = on ? 'Power off' : 'Power on';
  $('power').classList.toggle('on', on);
  $('power').disabled = !idle || (on ? !state.armed : !$('ready').checked);
  $('ready').disabled = !connected || on || pending > 0;
  $('selected-name').textContent = names[selected];
  $('move-count').textContent = '(' + state.moves + ' move' + (state.moves === 1 ? '' : 's') + ')';
  $('commanded').textContent = state.commands[selected] ?? '—';
  $('park-value').textContent = item.park;
  $('home-value').textContent = item.home;
  syncLimits();
  syncSettings();
  for (const id of ['step-size', 'demo-speed']) $(id).disabled = !idle;
  for (const id of ['slider', 'target', 'move']) $(id).disabled = !canMove || state.remaining_moves === 0;
  for (const id of ['min-limit', 'max-limit', 'apply-limits']) $(id).disabled = !idle || !item.allowed;
  for (const id of ['save-home', 'save-park', 'go-home', 'go-park', 'add-position']) $(id).disabled = !canMove || !complete;
  for (const button of document.querySelectorAll('[data-joint]')) {
    const active = Number(button.dataset.joint) === selected;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', active);
    button.disabled = !state.joints.find(candidate => candidate.channel === Number(button.dataset.joint)).allowed;
  }
  for (const button of document.querySelectorAll('[data-jog-channel]')) {
    const channel = Number(button.dataset.jogChannel), target = state.commands[channel];
    const limits = state.joints.find(candidate => candidate.channel === channel);
    const next = target + Number(button.dataset.direction) * state.settings.step_size;
    button.disabled = !idle || !state.armed || !limits.allowed || !Number.isInteger(target) ||
      state.remaining_moves === 0 || next < limits.low || next > limits.high;
  }
  renderDemo(canMove && complete, idle);
  if (state.inactivity_warning) notice('No adjustment or pose save for 10 minutes. Servos are still holding.', true);
}

async function poll() {
  if (pollFlight) return pollFlight;
  pollFlight = (async () => {
    try {
      const response = await fetch('/api/state', {cache: 'no-store'});
      if (!response.ok) throw Error('State unavailable');
      const previous = state;
      state = await response.json();
      connected = true;
      $('connection').classList.remove('error');
      $('connection').textContent = state.mode === 'preview' ? 'Preview · no hardware access' : 'Local connection';
      if (!previous) selected = state.joints.find(candidate => candidate.allowed).channel;
      if (!previous || previous.commands[selected] !== state.commands[selected]) stage(state.commands[selected] ?? joint().home);
      if (previous?.armed && !state.armed || state.error && !state.armed) $('ready').checked = false;
      if (state.error && state.error !== lastError) notice(state.error, true);
      lastError = state.error;
      render();
    } catch (error) {
      connected = false;
      $('ready').checked = false;
      $('connection').classList.add('error');
      $('connection').textContent = 'Connection lost · support the arm and switch Eventek off.';
      render();
    } finally {
      pollFlight = null;
    }
  })();
  return pollFlight;
}

function move() {
  if (!state?.armed || state.busy || pending) return;
  const target = Number($('target').value), item = joint();
  if (!Number.isInteger(target) || target < item.low || target > item.high) {
    notice('Choose an integer target within the applied limits.', true);
    return;
  }
  action('move', {channel: selected, target, first_clear: false}, 'Moving ' + names[selected] + '…');
}

function settingsChanged() {
  const step_size = Number($('step-size').value), demo_speed = Number($('demo-speed').value);
  if (!Number.isInteger(step_size) || step_size < 1 || !Number.isInteger(demo_speed) ||
      demo_speed < 1 || demo_speed > state.settings.demo_speed_max) {
    notice('Use a positive integer step and a demo speed from 1 to ' + state.settings.demo_speed_max + ' counts/s.', true);
    return;
  }
  action('settings', {step_size, demo_speed}, 'Step size and demo speed updated.');
}

function jog(channel, direction) {
  if (!state?.armed || state.busy || pending) return;
  choose(channel);
  action('jog', {channel, direction}, 'Jogging ' + names[channel] + ' ' + (direction > 0 ? '+' : '−') + state.settings.step_size + ' counts…');
}

function savePose(name) {
  if (!window.confirm('Have you observed the complete ' + name + ' pose and checked support and clearance? Saving makes these counts the active ' + name + ' for pose moves and power transitions, including after restart.')) return;
  action('save', {name, observed: true, provenance: 'User confirmed the complete ' + name + ' pose and support/clearance in the manual app. ' + new Date().toISOString()},
    data => name + ' saved and active: ' + data.result + '. Retained after restart.');
}

function loadDialog() {
  $('saved-demos').replaceChildren();
  for (const record of state.demo.saved) {
    const option = document.createElement('option');
    option.value = record.filename;
    option.textContent = record.name + ' · ' + record.positions + ' positions';
    $('saved-demos').append(option);
  }
  $('load-feedback').textContent = state.demo.saved.length ? '' : 'No saved demos are available.';
  $('load-confirm').disabled = !state.demo.saved.length;
  $('load-dialog').showModal();
}

$('ready').onchange = render;
$('power').onclick = () => state.armed ? action('power_off', {}, 'Returning to PARK, then disabling PWM. Keep support.') :
  action('power_on', {park_confirmed: $('ready').checked}, 'Enabling PARK, then moving to HOME. Watch the arm.');
$('emergency').onclick = () => {
  $('ready').checked = false;
  action('stop', {}, data => data.result === true ? 'PWM disabled immediately. Keep support and switch Eventek off.' :
    'Stop requested. Check outputs and switch Eventek off.');
};
for (const name of ['HOME', 'PARK']) {
  $('go-' + name.toLowerCase()).onclick = () => action('go_pose', {name}, 'Going to ' + name + '…');
  $('save-' + name.toLowerCase()).onclick = () => savePose(name);
}
document.querySelectorAll('[data-joint]').forEach(button => button.onclick = () => choose(Number(button.dataset.joint)));
document.querySelectorAll('[data-jog-channel]').forEach(button => button.onclick = () => jog(Number(button.dataset.jogChannel), Number(button.dataset.direction)));
for (const id of ['step-size', 'demo-speed']) $(id).onchange = settingsChanged;
$('slider').oninput = () => { $('target').value = $('slider').value; };
$('target').oninput = () => { $('slider').value = $('target').value; };
$('slider').onchange = move;
$('move').onclick = move;
$('apply-limits').onclick = () => action('limits', {channel: selected, low: Number($('min-limit').value), high: Number($('max-limit').value)}, 'Limits updated. Recorded positions rechecked.');
$('add-position').onclick = () => action('demo/add', {}, 'Position added.', () => { picked = state.demo.positions.length; });
$('replace-position').onclick = () => action('demo/replace', {index: picked}, 'Position updated.');
$('step-name').onchange = () => action('demo/rename', {index: picked, name: $('step-name').value.trim()});
$('step-remove').onclick = () => action('demo/remove', {index: picked}, 'Position removed.');
for (const [id, direction] of [['step-up', -1], ['step-down', 1]]) {
  $(id).onclick = () => action('demo/reorder', {index: picked, direction}, '', () => { picked += direction; });
}
$('next-step').onclick = () => action('demo/next');
$('run-all').onclick = () => action(state.demo.status === 'paused' ? 'demo/resume' : 'demo/run');
$('pause').onclick = () => action('demo/pause');
$('restart').onclick = () => action('demo/restart');
$('demo-name').onchange = () => action('demo/name', {name: $('demo-name').value.trim()});
$('save-demo').onclick = async () => {
  const name = $('demo-name').value.trim();
  if (name !== state.demo.name && !await action('demo/name', {name})) return;
  action('demo/save', {}, data => 'Demo saved: ' + data.result);
};
$('load-demo').onclick = loadDialog;
$('load-cancel').onclick = () => $('load-dialog').close();
$('load-confirm').onclick = async () => {
  const data = await action('demo/load', {filename: $('saved-demos').value}, 'Demo loaded. Playback starts at position 1.', () => { picked = 0; });
  if (data) $('load-dialog').close();
};

poll();
setInterval(poll, 500);
