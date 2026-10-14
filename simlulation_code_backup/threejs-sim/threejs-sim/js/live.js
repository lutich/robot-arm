// Connection to the real controller (lutich/robot-arm) over its HTTP API.
//
// The controller accepts POSTs only from a page served by itself (Origin check), so this
// works when the simulator is served
//   a) by the controller (integration/ patch: http://<pi>:8765/sim/), or
//   b) by threejs-sim/server.mjs with --controller, which forwards /api/* with the right
//      Host and Origin.
// Reading state is always safe. Anything that moves the arm goes through the controller's
// own checks (bounds, readiness, one motion at a time) and only after the operator enabled it.

export class LiveLink {
  constructor(base = '') {
    this.base = base.replace(/\/$/, '');
    this.state = null;
    this.error = null;
    this.timer = null;
    this.abort = false;
    this.listeners = new Set();
  }

  on(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  emit() { for (const fn of this.listeners) fn(this.state, this.error); }

  async get(path) {
    const r = await fetch(`${this.base}/api/${path}`, { cache: 'no-store' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.error || `${path}: HTTP ${r.status}`);
    return data;
  }

  async post(path, body = {}) {
    const r = await fetch(`${this.base}/api/${path}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
    });
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.error || `${path}: HTTP ${r.status}`);
    return data;
  }

  async poll() {
    try { this.state = await this.get('state'); this.error = null; }
    catch (e) { this.error = e.message; }
    this.emit();
    return this.state;
  }

  start(intervalMs = 500) {
    this.stopPolling();
    const tick = async () => { await this.poll(); this.timer = setTimeout(tick, intervalMs); };
    tick();
  }
  stopPolling() { clearTimeout(this.timer); this.timer = null; }

  /** Commanded counts [c0..c5] from a state object. */
  static counts(state) { return [0, 1, 2, 3, 4, 5].map(i => state.commands?.[i] ?? state.commands?.[String(i)]); }

  // ---- actions (same bodies as the controller's own page) ----
  move(channel, target) { return this.post('move', { channel, target, first_clear: false }); }
  jog(channel, direction) { return this.post('jog', { channel, direction }); }
  goPose(name) { return this.post('go_pose', { name }); }
  powerOn() { return this.post('power_on', {}); }
  /** Fresh state (same name as SimArm.read, see armapi.js). */
  read() { return this.poll(); }
  emergencyStop() { this.abort = true; return this.post('stop', {}); }

  async waitIdle(timeoutS) {
    const t0 = performance.now();
    for (;;) {
      if (this.abort) throw new Error('aborted');
      const s = await this.poll();
      if (s && !s.busy) return s;
      if (s?.outputs_off) throw new Error('controller outputs are off');
      if ((performance.now() - t0) / 1000 > timeoutS) throw new Error('timed out waiting for the controller');
      await new Promise(r => setTimeout(r, 150));
    }
  }

  /**
   * Send single-joint moves one after the other, each after the previous has settled.
   * onStep(i, move) is called before each move. Stops on the first error.
   */
  async runMoves(moves, { onStep = () => {}, speed = 20 } = {}) {
    this.abort = false;
    let s = await this.waitIdle(30);
    for (let i = 0; i < moves.length; i++) {
      if (this.abort) throw new Error('aborted');
      const m = moves[i];
      onStep(i, m);
      const from = LiveLink.counts(s)[m.ch];
      if (from === m.target) continue;
      await this.move(m.ch, m.target);
      s = await this.waitIdle(1.5 * Math.abs(m.target - from) / speed + 10);
      if (LiveLink.counts(s)[m.ch] !== m.target) throw new Error(`${m.label}: controller reports ${LiveLink.counts(s)[m.ch]}`);
    }
  }
}
