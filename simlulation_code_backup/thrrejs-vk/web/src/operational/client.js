const MODES = new Set(['simulation', 'connected-read-only', 'operational-disarmed', 'operational-armed', 'fault-stopped']);
const CONNECTED_MODES = new Set(['connected-read-only', 'operational-disarmed', 'operational-armed']);
const CHANNELS = [0, 1, 2, 3, 4, 5];

const isRecord = (value) => value !== null && typeof value === 'object' && !Array.isArray(value);
const commandFor = (commands, channel) => commands?.[channel] ?? commands?.[String(channel)];

export function validateRobotState(state) {
  if (!isRecord(state) || !['preview', 'hardware', 'mock'].includes(state.mode)) {
    throw new Error('Robot state must identify preview, mock or hardware mode');
  }
  if (!Array.isArray(state.joints) || state.joints.length !== CHANNELS.length) {
    throw new Error('Robot state must contain all six joints');
  }
  const channels = state.joints.map((joint) => joint?.channel).sort((a, b) => a - b);
  if (!CHANNELS.every((channel, index) => channels[index] === channel)) {
    throw new Error('Robot state joint channels must be exactly 0 through 5');
  }
  for (const joint of state.joints) {
    if (!Number.isInteger(joint.low) || !Number.isInteger(joint.high) || joint.low > joint.high) {
      throw new Error(`Robot state channel ${joint.channel} bounds are invalid`);
    }
    if (typeof joint.allowed !== 'boolean') throw new Error(`Robot state channel ${joint.channel} allowed flag is missing`);
  }
  if (!isRecord(state.commands)) throw new Error('Robot state commands are missing');
  for (const channel of CHANNELS) {
    const command = commandFor(state.commands, channel);
    if (command !== null && !Number.isInteger(command)) {
      throw new Error(`Robot state command ${channel} must be an integer or null`);
    }
  }
  for (const field of ['armed', 'busy', 'outputs_off']) {
    if (typeof state[field] !== 'boolean') throw new Error(`Robot state ${field} flag is missing`);
  }
  return state;
}

export class RobotArmClient extends EventTarget {
  constructor({
    apiBase = '/api',
    fetchImpl = globalThis.fetch,
    requestTimeoutMs = 5000,
    pollIntervalMs = 1500,
    sleepImpl = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds)),
  } = {}) {
    super();
    this.apiBase = apiBase.replace(/\/$/, '');
    this.fetchImpl = (...args) => fetchImpl(...args);
    this.requestTimeoutMs = requestTimeoutMs;
    this.pollIntervalMs = pollIntervalMs;
    this.sleep = sleepImpl;
    this.mode = 'simulation';
    this.state = null;
    this.pollTimer = null;
  }

  get connected() { return CONNECTED_MODES.has(this.mode); }

  setMode(mode) {
    if (!MODES.has(mode)) throw new Error(`Unknown operating mode: ${mode}`);
    if (mode === this.mode) return;
    this.mode = mode;
    if (!CONNECTED_MODES.has(mode)) this.stopPolling();
    this.dispatchEvent(new CustomEvent('modechange', { detail: { mode, state: this.state } }));
  }

  setState(state) {
    this.state = validateRobotState(state);
    this.dispatchEvent(new CustomEvent('statechange', { detail: { state: this.state } }));
    return this.state;
  }

  async request(path, options = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), this.requestTimeoutMs);
    try {
      const response = await this.fetchImpl(`${this.apiBase}/${path}`, {
        cache: 'no-store',
        ...options,
        signal: controller.signal,
      });
      const text = await response.text();
      let data = {};
      if (text) {
        try { data = JSON.parse(text); }
        catch { throw new Error(`${path} returned a non-JSON response`); }
      }
      if (!response.ok) throw Object.assign(new Error(data.error || `${path} failed (${response.status})`), { status: response.status });
      return data;
    } catch (error) {
      if (error?.name === 'AbortError') throw new Error(`${path} timed out after ${this.requestTimeoutMs} ms`);
      throw error;
    } finally {
      clearTimeout(timeout);
    }
  }

  async connectReadOnly() {
    this.stopPolling();
    try {
      this.setState(await this.request('state'));
      this.setMode('connected-read-only');
      return this.state;
    } catch (error) {
      this.setMode('fault-stopped');
      throw new Error(`Read-only connection failed. Serve the simulator through the robot API or the guarded gateway. ${error.message}`);
    }
  }

  startPolling() {
    if (!this.connected) throw new Error('Connect read-only before starting state polling');
    this.stopPolling();
    const tick = async () => {
      if (!this.connected) return;
      try {
        await this.refresh();
        this.pollTimer = setTimeout(tick, this.pollIntervalMs);
      } catch (error) {
        this.dispatchEvent(new CustomEvent('connectionerror', { detail: { error } }));
        this.setMode('fault-stopped');
      }
    };
    this.pollTimer = setTimeout(tick, this.pollIntervalMs);
  }

  stopPolling() {
    clearTimeout(this.pollTimer);
    this.pollTimer = null;
  }

  enterOperationalDisarmed() {
    if (this.mode !== 'connected-read-only') throw new Error('Connect read-only before selecting operational mode');
    this.setMode('operational-disarmed');
  }

  async post(path, body = {}) {
    return this.request(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  }

  async arm({ parkConfirmed = false } = {}) {
    if (this.mode !== 'operational-disarmed') throw new Error('Operational mode must be selected before arming');
    if (!parkConfirmed) throw new Error('Fresh physical supported PARK confirmation is required');
    await this.refresh();
    if (this.state.busy) throw new Error('Robot API is busy');
    if (!this.state.outputs_off || this.state.armed) throw new Error('Robot outputs must be off before power_on');
    if (this.state.joints.some((joint) => joint.allowed === false)) throw new Error('All six joints must be enabled for operational planning');
    await this.post('power_on', { park_confirmed: true });
    await this.waitForIdle();
    if (!this.state.armed || this.state.outputs_off) throw new Error(this.state.error || 'Backend did not report an armed state');
    this.setMode('operational-armed');
  }

  async adoptArmedSession({ operatorConfirmed = false } = {}) {
    if (this.mode !== 'operational-disarmed') throw new Error('Operational mode must be selected before adopting an armed session');
    if (!operatorConfirmed) throw new Error('Fresh operator confirmation is required');
    await this.refresh();
    if (this.state.busy) throw new Error('Robot API is busy');
    if (!this.state.armed || this.state.outputs_off) throw new Error('Backend does not report an already armed session');
    if (this.state.joints.some((joint) => joint.allowed === false)) throw new Error('All six joints must be enabled for operational control');
    this.setMode('operational-armed');
  }

  async refresh() { return this.setState(await this.request('state')); }

  async waitForIdle(timeoutMs = 60000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      await this.refresh();
      if (this.state.error) throw new Error(this.state.error);
      if (!this.state.busy) return this.state;
      await this.sleep(200);
    }
    throw new Error('Robot API did not become idle before the timeout');
  }

  validatePath(spec, path) {
    if (!Array.isArray(path) || path.length < 1) throw new Error('Operational path is empty');
    for (const [index, pose] of path.entries()) {
      for (const joint of spec.joints) {
        const target = pose?.[joint.id];
        if (!Number.isInteger(target)) throw new Error(`${joint.id} target at path step ${index} is not an integer`);
        const channel = Number(joint.id.slice(1)) - 1;
        const backendJoint = this.state.joints.find((candidate) => candidate.channel === channel);
        if (backendJoint.allowed === false) throw new Error(`${joint.id} is disabled by the backend session`);
        if (target < backendJoint.low || target > backendJoint.high) {
          throw new Error(`${joint.id} target ${target} is outside backend session bounds ${backendJoint.low}–${backendJoint.high}`);
        }
      }
      if (index > 0) {
        const changed = spec.joints.filter((joint) => pose[joint.id] !== path[index - 1][joint.id]);
        if (changed.length !== 1) throw new Error('Operational playback accepts only single-servo calibration steps');
      }
    }
  }

  /** shouldAbort is checked before each step; a halt keeps the arm holding its last confirmed pose. */
  async executeSingleJointPath(spec, path, { onStep = null, shouldAbort = null } = {}) {
    if (this.mode !== 'operational-armed') throw new Error('Explicit operational arming is required');
    await this.refresh();
    if (!this.state.armed || this.state.outputs_off || this.state.busy) throw new Error('Backend is not armed and idle');
    this.validatePath(spec, path);
    const commanded = this.state.commands;
    for (const joint of spec.joints) {
      const channel = Number(joint.id.slice(1)) - 1;
      if (commandFor(commanded, channel) !== path[0][joint.id]) {
        throw new Error('Planned start does not match the backend commanded pose; re-plan from the connected state');
      }
    }
    try {
      for (let index = 1; index < path.length; index += 1) {
        if (shouldAbort?.()) return { aborted: true, completed: index - 1 };
        const joint = spec.joints.find((candidate) => path[index][candidate.id] !== path[index - 1][candidate.id]);
        const channel = Number(joint.id.slice(1)) - 1;
        const target = path[index][joint.id];
        await onStep?.({ phase: 'sending', index, total: path.length - 1, joint: joint.id, channel, target, pose: { ...path[index] } });
        await this.post('move', { channel, target, first_clear: false });
        await this.waitForIdle();
        if (!this.state.armed || this.state.outputs_off || commandFor(this.state.commands, channel) !== target) {
          throw new Error(`${joint.id} command ${target} was not confirmed by the backend`);
        }
        await onStep?.({ phase: 'confirmed', index, total: path.length - 1, joint: joint.id, channel, target, pose: { ...path[index] }, state: this.state });
      }
      return { aborted: false, completed: path.length - 1 };
    } catch (error) {
      try { await this.emergencyStop(); } catch { this.setMode('fault-stopped'); }
      throw new Error(`Operational playback stopped: ${error.message}`);
    }
  }

  /** Move all six joints to the controller's own HOME or PARK definition and confirm it. */
  async goPose(name) {
    if (this.mode !== 'operational-armed') throw new Error('Explicit operational arming is required');
    if (name !== 'HOME' && name !== 'PARK') throw new Error('Only the controller poses HOME and PARK can be sent');
    await this.refresh();
    if (!this.state.armed || this.state.outputs_off || this.state.busy) throw new Error('Backend is not armed and idle');
    const key = name.toLowerCase();
    const target = CHANNELS.map((channel) => this.state.joints.find((joint) => joint.channel === channel)?.[key]);
    if (!target.every(Number.isInteger)) throw new Error(`Robot state does not report the controller's ${name} definition`);
    let accepted = false;
    try {
      await this.post('go_pose', { name });
      accepted = true;
      await this.waitForIdle();
      if (!this.state.armed || this.state.outputs_off || CHANNELS.some((channel) => commandFor(this.state.commands, channel) !== target[channel])) {
        throw new Error(`${name} [${target.join(', ')}] was not confirmed by the backend`);
      }
    } catch (error) {
      // A 4xx refusal (gateway policy or controller guard) started no motion; stopping would only drop the holding arm.
      if (!accepted && error.status >= 400 && error.status < 500) throw new Error(`${name} refused: ${error.message}`);
      try { await this.emergencyStop(); } catch { this.setMode('fault-stopped'); }
      throw new Error(`${name} stopped: ${error.message}`);
    }
    return this.state;
  }

  async powerOff() {
    if (!['operational-armed', 'operational-disarmed'].includes(this.mode)) {
      throw new Error('Select operational mode before requesting power_off');
    }
    await this.post('power_off');
    await this.waitForIdle();
    if (!this.state.outputs_off || this.state.armed) throw new Error('Backend did not confirm outputs off');
    this.setMode('connected-read-only');
  }

  async emergencyStop() {
    if (!this.connected && this.mode !== 'fault-stopped') return;
    try { await this.post('stop', { reason: 'simulator emergency stop' }); }
    finally { this.setMode('fault-stopped'); }
  }

  returnToSimulation() {
    this.stopPolling();
    this.state = null;
    this.setMode('simulation');
  }
}
