// Emulation of the lutich/robot-arm controller session, in counts.
// Same rules as the real one (control/domain/motion.py, docs/api.md):
// - targets outside the bounds are rejected, never clamped;
// - one motion at a time, a new one while busy is rejected;
// - a motion moves all selected joints along one cubic smoothstep, its duration is the
//   slowest joint's 1,5 x |delta| / speed;
// - move, jog and go_pose run at 20 counts/s, demo playback at the demo speed (1..100);
// - stop cancels at once and switches the outputs off.
// state() returns the same shape as GET /api/state, so the UI draws sim and live alike.

import { SERVOS, CONTROLLER } from './config.js';

export class ControllerError extends Error {}

export class SimController {
  constructor(servos = SERVOS) {
    this.servos = servos;
    this.bounds = servos.map(s => [s.low, s.high]);
    this.poses = { HOME: servos.map(s => s.home), PARK: servos.map(s => s.park) };
    this.commands = this.poses.HOME.slice();      // integer counts, as reported
    this.position = this.commands.slice();        // float counts while moving, for drawing
    this.armed = true; this.outputsOff = false; this.error = null;
    this.speed = CONTROLLER.manualSpeed;
    this.settings = { step_size: CONTROLLER.stepSize, demo_speed: CONTROLLER.demoSpeed, demo_speed_max: CONTROLLER.demoSpeedMax };
    this.motion = null;   // { start, target, duration, t, channels, done }
    this.queue = [];      // further motions of a demo run or a sequential plan
    this.demo = { name: 'Sim demo', positions: [], cursor: 0, status: 'idle', mode: null, pause_requested: false };
    this.moves = 0;
    this.listeners = new Set();
    this.log = [];
  }

  on(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  emit(kind, detail) {
    this.log.push({ kind, detail });
    if (this.log.length > 200) this.log.shift();
    for (const fn of this.listeners) fn(kind, detail);
  }
  get busy() { return !!this.motion || this.queue.length > 0; }

  // ---------------- checks ----------------

  checkReady() {
    if (this.outputsOff) throw new ControllerError('Outputs are off; power on first');
    if (!this.armed) throw new ControllerError('Not armed');
  }
  checkIdle() { if (this.busy) throw new ControllerError('Rejected: a motion is already running'); }
  checkTarget(ch, target) {
    if (!Number.isInteger(ch) || ch < 0 || ch > 5) throw new ControllerError(`channel: ${ch} is not 0..5`);
    if (!Number.isInteger(target)) throw new ControllerError('target: must be an integer count');
    const [lo, hi] = this.bounds[ch];
    if (target < lo || target > hi) throw new ControllerError(`${this.servos[ch].name} target ${target} outside ${lo}..${hi}`);
  }

  // ---------------- motion core ----------------

  makeMotion(target, speed, label) {
    let duration = 0;
    const channels = [];
    for (let ch = 0; ch < 6; ch++) {
      if (target[ch] === undefined || target[ch] === null) continue;
      channels.push(ch);
      duration = Math.max(duration, CONTROLLER.peakFactor * Math.abs(target[ch] - this.commands[ch]) / speed);
    }
    return { start: null, target, duration, t: 0, channels, speed, label };
  }

  begin(m) {
    m.start = this.commands.slice();
    // Recompute against the actual start (queued motions start where the previous ended).
    m.duration = 0;
    for (const ch of m.channels) m.duration = Math.max(m.duration, CONTROLLER.peakFactor * Math.abs(m.target[ch] - m.start[ch]) / m.speed);
    m.t = 0;
    this.motion = m;
    this.moves++;
    this.emit('motion', m.label);
  }

  /** Advance simulated time by dt seconds. */
  update(dt) {
    if (!this.motion && this.queue.length && !this.outputsOff) this.begin(this.queue.shift());
    const m = this.motion;
    if (!m) return false;
    m.t += dt;
    const f = m.duration <= 0 ? 1 : Math.min(1, m.t / m.duration);
    const p = f * f * (3 - 2 * f);
    for (const ch of m.channels) {
      const v = m.start[ch] + (m.target[ch] - m.start[ch]) * p;
      this.position[ch] = v;
      this.commands[ch] = Math.round(v);
    }
    if (f >= 1) {
      for (const ch of m.channels) { this.commands[ch] = m.target[ch]; this.position[ch] = m.target[ch]; }
      this.motion = null;
      if (m.demoIndex !== undefined) {
        this.demo.cursor = m.demoIndex + 1;
        if (this.demo.pause_requested) { this.queue = []; this.demo.status = 'paused'; this.demo.pause_requested = false; }
        else if (!this.queue.length) this.demo.status = this.demo.cursor >= this.demo.positions.length ? 'complete' : 'idle';
      }
      this.emit('settled', m.label);
      if (!this.queue.length) this.emit('idle');
    }
    return true;
  }

  // ---------------- API actions (same names as POST /api/...) ----------------

  move(channel, target) {
    this.checkReady(); this.checkIdle(); this.checkTarget(channel, target);
    const t = []; t[channel] = target;
    this.begin(this.makeMotion(t, this.speed, `move ${this.servos[channel].name} → ${target}`));
  }

  jog(channel, direction) {
    if (direction !== 1 && direction !== -1) throw new ControllerError('direction: must be 1 or -1');
    this.move(channel, this.commands[channel] + direction * this.settings.step_size);
  }

  go_pose(name) {
    this.checkReady(); this.checkIdle();
    const pose = this.poses[name];
    if (!pose) throw new ControllerError(`Unknown pose ${name}`);
    pose.forEach((c, ch) => this.checkTarget(ch, c));
    this.begin(this.makeMotion(pose.slice(), this.speed, `go ${name}`));
  }

  stop(reason = 'stop') {
    this.motion = null; this.queue = [];
    this.outputsOff = true; this.armed = false;
    this.position = this.commands.slice();
    if (this.demo.status === 'running' || this.demo.status === 'pausing') this.demo.status = 'idle';
    this.error = reason;
    this.emit('stop', reason);
  }

  /** Power on: PARK, then HOME (the real one requires a confirmed, supported PARK). */
  power_on() {
    this.motion = null; this.queue = [];
    this.outputsOff = false; this.armed = true; this.error = null;
    this.commands = this.poses.PARK.slice(); this.position = this.commands.slice();
    this.queue.push(this.makeMotion(this.poses.HOME.slice(), this.speed, 'power on → HOME'));
    this.emit('power', 'on');
  }

  power_off() {
    this.checkIdle();
    this.queue.push(this.makeMotion(this.poses.PARK.slice(), this.speed, 'power off → PARK'));
    const off = () => { this.outputsOff = true; this.armed = false; this.emit('power', 'off'); un(); };
    const un = this.on(kind => { if (kind === 'idle') off(); });
  }

  settings_(step_size, demo_speed) {
    this.checkIdle();
    if (!(Number.isInteger(step_size) && step_size > 0)) throw new ControllerError('step_size: positive integer');
    if (!(Number.isInteger(demo_speed) && demo_speed >= 1 && demo_speed <= CONTROLLER.demoSpeedMax)) throw new ControllerError('demo_speed: 1..100');
    this.settings.step_size = step_size; this.settings.demo_speed = demo_speed;
  }

  limits(channel, low, high) {
    const s = this.servos[channel];
    if (!(low >= s.low && high <= s.high && low <= high)) throw new ControllerError(`limits must stay inside the guards ${s.low}..${s.high}`);
    this.bounds[channel] = [low, high];
  }

  save(name) { this.poses[name] = this.commands.slice(); }

  // ---------------- demo ----------------

  demo_add(name) {
    this.checkIdle();
    this.demo.positions.push({ name: name || `Position ${this.demo.positions.length + 1}`, counts: this.commands.slice() });
    this.demo_restart();
  }
  demo_replace(index) { this.checkIdle(); this.demo.positions[index].counts = this.commands.slice(); this.demo_restart(); }
  demo_remove(index) { this.checkIdle(); this.demo.positions.splice(index, 1); this.demo_restart(); }
  demo_rename(index, name) { this.demo.positions[index].name = name; }
  demo_reorder(index, direction) {
    const j = index + direction, p = this.demo.positions;
    if (j < 0 || j >= p.length) return;
    [p[index], p[j]] = [p[j], p[index]]; this.demo_restart();
  }
  demo_restart() { this.demo.cursor = 0; this.demo.status = 'idle'; this.demo.pause_requested = false; }
  demo_load(name, positions) {
    this.checkIdle();
    this.demo = { name, positions: positions.map(p => ({ name: p.name, counts: p.counts.slice() })), cursor: 0, status: 'idle', mode: null, pause_requested: false };
  }

  validatePositions(positions) {
    positions.forEach((p, i) => p.counts.forEach((c, ch) => {
      try { this.checkTarget(ch, c); } catch (e) { throw new ControllerError(`Position ${i + 1} "${p.name}": ${e.message}`); }
    }));
  }

  demoMotions(from, to) {
    const out = [];
    for (let i = from; i < to; i++) {
      const m = this.makeMotion(this.demo.positions[i].counts.slice(), this.settings.demo_speed, this.demo.positions[i].name);
      m.demoIndex = i; out.push(m);
    }
    return out;
  }

  demo_run() {
    this.checkReady(); this.checkIdle();
    this.validatePositions(this.demo.positions);
    this.demo.cursor = 0; this.demo.status = 'running'; this.demo.mode = 'all';
    this.queue = this.demoMotions(0, this.demo.positions.length);
  }
  demo_next() {
    this.checkReady(); this.checkIdle();
    if (this.demo.cursor >= this.demo.positions.length) this.demo.cursor = 0;
    this.validatePositions([this.demo.positions[this.demo.cursor]]);
    this.demo.status = 'running'; this.demo.mode = 'step';
    this.queue = this.demoMotions(this.demo.cursor, this.demo.cursor + 1);
  }
  demo_pause() { if (this.busy) { this.demo.pause_requested = true; this.demo.status = 'pausing'; } }
  demo_resume() {
    this.checkReady(); this.checkIdle();
    this.demo.status = 'running';
    this.queue = this.demoMotions(this.demo.cursor, this.demo.positions.length);
  }

  // ---------------- sim-only helpers ----------------

  /** Play a list of full poses at `speed` counts/s, like a demo run (synchronised joints). */
  runPoses(poses, speed, labels = []) {
    this.checkReady(); this.checkIdle();
    poses.forEach((p, i) => p.forEach((c, ch) => {
      try { this.checkTarget(ch, c); } catch (e) { throw new ControllerError(`Step ${i + 1}: ${e.message}`); }
    }));
    this.queue = poses.map((p, i) => this.makeMotion(p.slice(), speed, labels[i] || `step ${i + 1}`));
  }

  /** Play single-joint moves in order, the only way the HTTP API reaches a computed pose. */
  runMoves(moves, labels = []) {
    this.checkReady(); this.checkIdle();
    moves.forEach(({ ch, target }) => this.checkTarget(ch, target));
    this.queue = moves.map(({ ch, target }, i) => { const t = []; t[ch] = target; return this.makeMotion(t, this.speed, labels[i] || `move ${this.servos[ch].name} → ${target}`); });
  }

  /** Sim only: switch the outputs back on where the arm is, without the PARK -> HOME move. */
  rearm() {
    this.motion = null; this.queue = [];
    this.outputsOff = false; this.armed = true; this.error = null;
    this.emit('power', 're-armed');
  }

  /** Teleport without motion (sim reset, live mirror). */
  setCommands(counts) {
    this.motion = null; this.queue = [];
    this.commands = counts.slice(); this.position = counts.slice();
  }

  state() {
    const c = {}; this.commands.forEach((v, i) => { c[i] = v; });
    return {
      mode: 'simulation', armed: this.armed, busy: this.busy, outputs_off: this.outputsOff, error: this.error,
      phase: this.motion ? this.motion.label : null, commands: c, moves: this.moves, speed: this.speed,
      joints: this.servos.map((s, i) => ({ channel: i, name: `${s.name} · ${s.joint}`, low: this.bounds[i][0], high: this.bounds[i][1],
        park: this.poses.PARK[i], home: this.poses.HOME[i] })),
      demo: this.demo, settings: this.settings,
    };
  }
}
