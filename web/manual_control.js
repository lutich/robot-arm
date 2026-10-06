'use strict';

const $ = id => document.getElementById(id);
const names = ['Base rotation', 'Shoulder', 'Elbow', 'Wrist flexion', 'Wrist rotation', 'Gripper'];
let state = null, connected = false, selected = 0, picked = -1, pending = 0;
let pollFlight = null, renderedList = '', renderedBounds = '', renderedSettings = '', lastError = null;
let cameraStatus = null, cameraConnected = false, cameraPollFlight = null;
let cameraSettingsFlight = false, cameraCaptureFlight = false, cameraViewing = false, cameraCaptured = null;
let cameraDisplay = 'rgb';
let cameraDraft = null, cameraFormKey = '', cameraLastError = null;
const cameraDirty = new Set();

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
  renderCamera();
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

function cameraNotice(message, error = false) {
  $('camera-feedback').textContent = message;
  $('camera-feedback').classList.toggle('error', error);
}

async function cameraRequest(path, options = {}) {
  const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 4000);
  try {
    const response = await fetch('/api/camera/' + path, {cache: 'no-store', ...options, signal: controller.signal});
    if (!response.ok) {
      const data = await response.json().catch(() => ({})), error = Error(data.error || 'Camera request failed');
      error.status = response.status;
      throw error;
    }
    return await response.json();
  } catch (error) {
    if (error.name === 'AbortError') throw Error('Camera request timed out.');
    throw error;
  } finally {
    clearTimeout(timeout);
  }
}

function cameraOptions(id, modes, labels = {}) {
  const select = $(id);
  select.replaceChildren(...modes.map(mode => {
    const option = document.createElement('option');
    option.value = mode;
    option.textContent = labels[mode] || mode[0].toUpperCase() + mode.slice(1);
    return option;
  }));
}

function cameraBounds(id, range) {
  for (const bound of ['min', 'max']) {
    if (Number.isFinite(range?.[bound])) $(id)[bound] = range[bound];
    else $(id).removeAttribute(bound);
  }
}

function syncCameraForm() {
  const status = cameraStatus, controls = status.capabilities?.controls || {};
  const key = JSON.stringify([status.run_id, status.settings_revision, status.requested, controls]);
  if (cameraDraft && cameraDraft.run_id !== status.run_id) cameraDirty.clear();
  if (key === cameraFormKey || cameraDirty.size || cameraSettingsFlight) return;
  cameraFormKey = key;
  cameraDraft = {run_id: status.run_id, revision: status.settings_revision};
  const requested = status.requested || {}, frame = status.latest_frame || {};
  for (const [group, id] of [['exposure', 'camera-exposure-mode'], ['focus', 'camera-focus-mode'],
    ['white_balance', 'camera-white-balance-mode'], ['anti_banding', 'camera-anti-banding']]) {
    cameraOptions(id, controls[group]?.modes || [], {once: 'Once', off: 'Off', '50hz': '50 Hz', '60hz': '60 Hz'});
    $(id).value = group === 'anti_banding' ? requested[group] || '' : requested[group]?.mode || 'auto';
  }
  for (const [id, group, field, value] of [
    ['camera-time', 'exposure', 'time_us', requested.exposure?.time_us ?? frame.time_us],
    ['camera-iso', 'exposure', 'iso', requested.exposure?.iso ?? frame.iso],
    ['camera-lens', 'focus', 'lens_position', requested.focus?.lens_position ?? frame.lens_position],
    ['camera-temperature', 'white_balance', 'temperature_k', requested.white_balance?.temperature_k ?? frame.temperature_k]
  ]) {
    cameraBounds(id, controls[group]?.[field]);
    $(id).value = Number.isFinite(value) ? value : '';
  }
}

function cameraFrameText(frame) {
  if (!frame) return 'No frame yet.';
  return [Number.isFinite(frame.time_us) ? frame.time_us + ' µs' : null,
    Number.isFinite(frame.iso) ? 'ISO ' + frame.iso : null,
    Number.isFinite(frame.temperature_k) ? frame.temperature_k + ' K' : null,
    Number.isInteger(frame.lens_position) && frame.lens_position >= 0 ? 'Lens ' + frame.lens_position : null,
    'Frame ' + frame.sequence].filter(Boolean).join(' · ');
}

function cameraDepthText(depth) {
  if (!depth) return 'No depth frame yet.';
  return [depth.width + ' × ' + depth.height + ' · ' + depth.unit + ' · aligned to ' + depth.aligned_to.toUpperCase(),
    Number.isFinite(depth.valid_fraction) ? (depth.valid_fraction * 100).toFixed(1) + '% valid' : null,
    Number.isFinite(depth.sync_delta_ms) ? 'Skew ' + depth.sync_delta_ms.toFixed(1) + ' ms' : null,
    Number.isFinite(depth.age_ms) ? Math.round(depth.age_ms) + ' ms old' : null].filter(Boolean).join(' · ');
}

function pauseCameraView() {
  cameraViewing = false;
  $('camera-image').removeAttribute('src');
  const url = cameraDisplay === 'depth' ? cameraCaptured?.depthPreviewUrl : cameraCaptured?.rgbUrl;
  if (url) {
    $('camera-image').src = url;
    $('camera-image').hidden = false;
    $('camera-placeholder').hidden = true;
  } else {
    $('camera-image').hidden = true;
    $('camera-placeholder').hidden = false;
  }
}

function startCameraView() {
  cameraViewing = true;
  $('camera-image').removeAttribute('src');
  $('camera-image').src = '/api/camera/' + (cameraDisplay === 'depth' ? 'depth/stream' : 'stream.mjpg');
  $('camera-image').hidden = false;
  $('camera-placeholder').hidden = true;
}

function renderCamera() {
  const status = cameraStatus, available = cameraConnected && status?.available;
  const depthAvailable = available && status.depth_available;
  const retainedDepth = Boolean(cameraCaptured?.depthPreviewUrl);
  const controls = status?.capabilities?.controls || {};
  const editing = available && connected && !state?.busy && !pending && !status.locked &&
    !status.pending && !cameraSettingsFlight;
  if (cameraViewing && (!available || cameraDisplay === 'depth' && !depthAvailable)) pauseCameraView();
  if (cameraDisplay === 'depth' && !depthAvailable && !retainedDepth) {
    cameraDisplay = 'rgb';
    $('camera-display').value = 'rgb';
    pauseCameraView();
  }
  $('camera-display').disabled = cameraCaptureFlight || !available && !cameraCaptured;
  $('camera-depth-option').disabled = !depthAvailable && !retainedDepth;
  $('camera-depth-legend').hidden = cameraDisplay !== 'depth';
  $('camera-image').alt = cameraDisplay === 'depth' ? 'Colorized depth preview; display only' : 'Camera view of the arm and table';
  $('camera-label').textContent = !cameraConnected ? 'Offline' : !status.enabled ? 'Disabled' :
    status.application_status === 'starting' ? 'Starting' : available ? 'Available' : 'Unavailable';
  $('camera-state').classList.toggle('on', Boolean(available));
  $('camera-view').textContent = cameraViewing ? 'Pause' : 'View';
  $('camera-view').disabled = !available || cameraDisplay === 'depth' && !depthAvailable || cameraCaptureFlight;
  $('camera-capture').disabled = !available || !status.latest_frame || status.pending || cameraSettingsFlight || cameraCaptureFlight;
  $('camera-download').disabled = !cameraCaptured || cameraCaptureFlight;
  $('camera-depth-download').disabled = !cameraCaptured?.depthUrl || cameraCaptureFlight;
  if (!status) return;
  syncCameraForm();
  $('camera-focus-group').hidden = !status.capabilities?.has_autofocus || !controls.focus;
  for (const [group, id] of [['exposure', 'camera-exposure-group'], ['focus', 'camera-focus-group'],
    ['white_balance', 'camera-white-balance-group'], ['anti_banding', 'camera-anti-banding-group']]) {
    $(id).disabled = !editing || !controls[group];
  }
  for (const [mode, ids] of [['camera-exposure-mode', ['camera-time', 'camera-iso']],
    ['camera-focus-mode', ['camera-lens']], ['camera-white-balance-mode', ['camera-temperature']]]) {
    for (const id of ids) {
      $(id).disabled = !editing || $(mode).value !== 'manual';
      $(id).required = $(mode).value === 'manual';
    }
  }
  $('camera-anti-banding').disabled = !editing || $('camera-exposure-mode').value !== 'auto';
  $('camera-focus-once').disabled = !editing || cameraDirty.size > 0;
  $('camera-apply').disabled = !editing || !cameraDirty.size;
  $('camera-use-readings').disabled = !editing || !status.latest_frame;
  const profile = status.profile;
  $('camera-profile').textContent = !status.enabled ? 'Camera is disabled for this session.' : profile ?
    profile.width + ' × ' + profile.height + ' · ' + profile.fps + ' fps · JPEG quality ' + profile.jpeg_quality +
    (profile.undistortion ? ' · Undistorted' : '') +
    (status.capabilities?.has_autofocus ? ' · Autofocus lens' : available ? ' · Fixed focus' : '') +
    (status.capabilities?.has_depth ? ' · Camera depth only; robot calibration pending.' :
      ' · Image access only; metric calibration pending.') : 'Waiting for camera.';
  const depthProfile = status.depth_profile;
  if (depthProfile) $('camera-depth-legend').textContent = 'Depth preview: ' + depthProfile.preview_near_mm +
    ' mm red · ' + depthProfile.preview_far_mm + ' mm blue · black unknown. Depth PNG stores millimetres; 0 is unknown.';
  const requested = status.requested || {};
  $('camera-requested').textContent = ['Exposure ' + (requested.exposure?.mode || '—'),
    requested.exposure?.mode === 'manual' ? requested.exposure.time_us + ' µs · ISO ' + requested.exposure.iso : null,
    'WB ' + (requested.white_balance?.mode === 'manual' ? requested.white_balance.temperature_k + ' K' : 'Auto'),
    requested.focus ? 'Focus ' + (requested.focus.mode === 'manual' ? requested.focus.lens_position : 'once') : null,
    'Revision ' + status.settings_revision + ' · ' + status.application_status].filter(Boolean).join(' · ');
  $('camera-observed').textContent = cameraFrameText(status.latest_frame) +
    (Number.isFinite(status.frame_age_ms) ? ' · ' + Math.round(status.frame_age_ms) + ' ms old' : '');
  $('camera-depth-observed').textContent = cameraDepthText(status.latest_frame?.depth);
}

async function pollCamera() {
  if (cameraPollFlight) return cameraPollFlight;
  cameraPollFlight = (async () => {
    try {
      cameraStatus = await cameraRequest('status');
      cameraConnected = true;
      if (cameraStatus.last_error && cameraStatus.last_error !== cameraLastError) cameraNotice(cameraStatus.last_error, true);
      cameraLastError = cameraStatus.last_error;
    } catch (error) {
      cameraConnected = false;
      cameraNotice(error.message, true);
    } finally {
      cameraPollFlight = null;
      renderCamera();
    }
  })();
  return cameraPollFlight;
}

async function applyCameraSettings(groups) {
  if (cameraSettingsFlight || !cameraDraft) return;
  cameraSettingsFlight = true;
  renderCamera();
  try {
    await cameraRequest('settings', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({expected_run_id: cameraDraft.run_id, expected_revision: cameraDraft.revision, ...groups})});
    cameraDirty.clear();
    cameraFormKey = '';
    cameraNotice('Settings requested. Frame readings show what the camera reports.');
  } catch (error) {
    if (error.status === 409) {
      cameraDirty.clear();
      cameraFormKey = '';
    }
    cameraNotice(error.message, true);
  } finally {
    cameraSettingsFlight = false;
    if (cameraPollFlight) await cameraPollFlight;
    await pollCamera();
  }
}

function cameraSettingsSubmit(event) {
  event.preventDefault();
  const groups = {};
  for (const group of cameraDirty) {
    if (group === 'anti_banding') groups[group] = $('camera-anti-banding').value;
    if (group === 'exposure') groups[group] = $('camera-exposure-mode').value === 'auto' ? {mode: 'auto'} :
      {mode: 'manual', time_us: Number($('camera-time').value), iso: Number($('camera-iso').value)};
    if (group === 'focus') groups[group] = $('camera-focus-mode').value === 'once' ? {mode: 'once'} :
      {mode: 'manual', lens_position: Number($('camera-lens').value)};
    if (group === 'white_balance') groups[group] = $('camera-white-balance-mode').value === 'auto' ? {mode: 'auto'} :
      {mode: 'manual', temperature_k: Number($('camera-temperature').value)};
  }
  if (cameraDirty.size) applyCameraSettings(groups);
}

function useCameraReadings() {
  const frame = cameraStatus.latest_frame, controls = cameraStatus.capabilities.controls;
  if (!frame) return;
  for (const [group, modeId, fields] of [
    ['exposure', 'camera-exposure-mode', [['time_us', 'camera-time'], ['iso', 'camera-iso']]],
    ['focus', 'camera-focus-mode', [['lens_position', 'camera-lens']]],
    ['white_balance', 'camera-white-balance-mode', [['temperature_k', 'camera-temperature']]]
  ]) {
    if (!controls[group] || !fields.every(([field]) => Number.isInteger(frame[field]) &&
      frame[field] >= controls[group][field].min && frame[field] <= controls[group][field].max)) continue;
    $(modeId).value = 'manual';
    for (const [field, id] of fields) $(id).value = frame[field];
    cameraDirty.add(group);
  }
  cameraDraft = {run_id: cameraStatus.run_id, revision: cameraStatus.settings_revision};
  cameraNotice('Current readings staged. Review them, then Apply.');
  renderCamera();
}

async function captureCamera() {
  if (cameraCaptureFlight) return;
  cameraCaptureFlight = true;
  pauseCameraView();
  cameraNotice('Capturing a fresh camera frame…');
  renderCamera();
  try {
    const current = await cameraRequest('status'), frame = current.latest_frame;
    if (!current.available || !frame) throw Error('Camera has no fresh frame.');
    const query = new URLSearchParams({wait_ms: '2000', after_run_id: frame.run_id, after_sequence: frame.sequence});
    const capture = await cameraRequest('capture?' + query), metadata = capture.metadata;
    if (!metadata?.run_id || !Number.isInteger(metadata.sequence)) throw Error('Captured frame metadata is missing.');
    const imageBlob = (encoded, type) => new Blob([Uint8Array.from(atob(encoded), value => value.charCodeAt(0))], {type});
    const rgb = imageBlob(capture.rgb_jpeg, 'image/jpeg');
    const depth = capture.depth_png ? imageBlob(capture.depth_png, 'image/png') : null;
    const preview = capture.depth_preview_png ? imageBlob(capture.depth_preview_png, 'image/png') : null;
    const previous = cameraCaptured;
    cameraCaptured = {metadata, rgbUrl: URL.createObjectURL(rgb),
      depthUrl: depth ? URL.createObjectURL(depth) : null, depthPreviewUrl: preview ? URL.createObjectURL(preview) : null};
    pauseCameraView();
    releaseCameraCapture(previous);
    $('camera-capture-info').textContent = 'Captured ' + metadata.captured_at + ' · ' +
      cameraFrameText(metadata) + ' · Revision ' + metadata.settings_revision +
      (metadata.depth ? ' · Depth: ' + cameraDepthText(metadata.depth) : ' · RGB only');
    cameraNotice(metadata.depth ? 'Pair captured. Both downloads save this pair.' : 'Image captured. Download RGB saves this image.');
  } catch (error) {
    cameraNotice(error.message, true);
  } finally {
    cameraCaptureFlight = false;
    renderCamera();
  }
}

$('camera-settings').onsubmit = cameraSettingsSubmit;
document.querySelectorAll('[data-camera-group]').forEach(input => input.oninput = () => {
  cameraDirty.add(input.dataset.cameraGroup);
  renderCamera();
});
$('camera-focus-once').onclick = () => applyCameraSettings({focus: {mode: 'once'}});
$('camera-use-readings').onclick = useCameraReadings;
$('camera-view').onclick = () => {
  if (cameraViewing) pauseCameraView();
  else startCameraView();
  renderCamera();
};
$('camera-display').onchange = () => {
  const viewing = cameraViewing;
  cameraDisplay = $('camera-display').value;
  pauseCameraView();
  if (viewing && cameraConnected && cameraStatus?.available &&
    (cameraDisplay === 'rgb' || cameraStatus.depth_available)) startCameraView();
  renderCamera();
};
$('camera-image').onerror = () => {
  if (!cameraViewing) return;
  pauseCameraView();
  cameraNotice('Live view unavailable. Camera status and robot controls remain separate.', true);
  renderCamera();
};
$('camera-capture').onclick = captureCamera;
function downloadCamera(url, suffix) {
  if (!cameraCaptured || !url) return;
  const link = document.createElement('a');
  link.href = url;
  link.download = 'camera-' + cameraCaptured.metadata.run_id + '-' + cameraCaptured.metadata.sequence + suffix;
  document.body.append(link);
  link.click();
  link.remove();
}
$('camera-download').onclick = () => downloadCamera(cameraCaptured?.rgbUrl, '.jpg');
$('camera-depth-download').onclick = () => downloadCamera(cameraCaptured?.depthUrl, '-depth.png');
function releaseCameraCapture(capture) {
  if (!capture) return;
  for (const url of [capture.rgbUrl, capture.depthUrl, capture.depthPreviewUrl]) if (url) URL.revokeObjectURL(url);
}
window.addEventListener('pagehide', () => {
  cameraViewing = false;
  $('camera-image').removeAttribute('src');
  releaseCameraCapture(cameraCaptured);
});

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
pollCamera();
setInterval(pollCamera, 1000);
