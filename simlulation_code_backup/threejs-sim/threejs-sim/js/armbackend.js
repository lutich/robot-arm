// The arm backend behind /api: one shared simulated arm, and optionally the real controller.
//
// Clients (the page, scripts, other tools) use only the robot's low-level API (lutich/robot-arm
// docs/api.md): GET state, POST move / jog / go_pose / stop / power_on / power_off / settings /
// limits / demo/*. Which arm answers is switched for everybody with setTarget('sim' | 'real').
//
//   sim   the shared simulated arm: controller emulation, cubes, physics, collision; a collision
//         stops it like an emergency stop
//   real  the controller on the Pi. Every call that can move the arm is first played on a copy of
//         the simulation, synced to the real arm's state, and blocked if the arm would hit itself,
//         the board or a cube's side; "stop" always goes through. The shared simulation follows the
//         real arm's commanded pose.
//
// Pure JS, no Node APIs: the real controller is reached through `real` = { get(path), post(path, body) }.

import { SimController, ControllerError } from './controller.js';
import { controllerActions } from './controller-api.js';
import { World } from './world.js';

/** Actions that cannot move the arm: never simulated first, never blocked. */
const SAFE = new Set(['stop', 'settings', 'limits', 'demo/add', 'demo/replace', 'demo/remove', 'demo/rename', 'demo/reorder', 'demo/pause', 'demo/restart']);
/** Calls that end in PARK, where the real gripper rests on the base plate by design (docs/model-photo-comparison.md). */
const parks = (path, body) => path === 'power_on' || path === 'power_off' || (path === 'go_pose' && body.name === 'PARK');
// The model's PARK still puts the pads 2 mm into the board and the base (HOME angles not measured yet,
// HANDOVER.md open item 1), so at PARK the gripper's contact with base and board is not a collision.
const restingOnBase = k => /^(base|board) × (left|right) (pad|finger)$|^(left|right) (pad|finger) × (base|board)$/.test(k);

export class Blocked extends Error {}

/** Contacts that end a motion: the arm with itself or the board. Arm against cubes is grasping or pushing. */
const hardContacts = w => w.contacts.map(k => `${k.a} × ${k.b}`).filter(k => !k.endsWith(' cube'));

export const DEFAULT_SCENE = [['orange', 10.8, 14.0, 0, 3], ['blue', -8, 12, 0, 0], ['yellow', 3, 10, 0, 0], ['green', -4, 17, 0, 10]];

export class ArmBackend {
  constructor({ real = null, realName = null, allowMotion = false, scene = DEFAULT_SCENE, log = () => {} } = {}) {
    this.real = real; this.realName = realName; this.allowMotion = allowMotion; this.log = log;
    this.target = 'sim';
    this.ctl = new SimController();
    this.world = new World();
    this.setScene(scene);
    this.lastHard = new Set();
    this.realState = null;
  }

  // ---------------- shared simulation ----------------

  setScene(cubes) {
    this.world.cubes = []; this.world.holding = null;
    for (const [c, x, y, level = 0, yaw = 0, id = null] of cubes) {
      let name = id || c, n = 2; while (this.world.cube(name)) name = `${c}${n++}`;
      this.world.addCube(name, c, x, y, level, yaw);
    }
    this.world.dirty = true;
  }

  /** Advance the shared simulation by dt seconds (the server calls this at 50 Hz). */
  tick(dt) {
    const n = Math.max(1, Math.ceil(dt / (1 / 120)));
    for (let i = 0; i < n; i++) {
      this.ctl.update(dt / n);
      this.world.step(dt / n, this.ctl.position);
      const atPark = /PARK|power/.test(this.ctl.motion?.label || '');
      const hard = hardContacts(this.world), fresh = hard.filter(k => !this.lastHard.has(k) && !(atPark && restingOnBase(k)));
      this.lastHard = new Set(hard);
      if (fresh.length && this.ctl.busy && this.target === 'sim') {
        this.ctl.stop(`collision: ${fresh.join(', ')}`);
        this.log(`sim: stopped, collision ${fresh.join(', ')}`);
      }
    }
  }

  /** Cubes and gripper of the shared simulation, for viewers. */
  worldState() {
    return { cubes: this.world.cubes.map(c => ({ id: c.id, colour: c.colour, p: c.p, R: c.R, state: c.state })), holding: this.world.holding?.id || null,
      gap: this.world.gap, position: this.ctl.position, target: this.target,
      events: this.world.events.slice(-30).map(e => ({ t: e.t, kind: e.kind, text: e.text })) };
  }

  // ---------------- backend switch ----------------

  info() { return { target: this.target, real: this.realName, realAvailable: !!this.real, allowMotion: this.allowMotion }; }

  async setTarget(target) {
    if (target !== 'sim' && target !== 'real') throw new ControllerError('target: "sim" or "real"');
    if (target === 'real') {
      if (!this.real) throw new ControllerError('No real controller: start the server with --controller <url>');
      this.syncFromReal(await this.real.get('state'));
    } else {
      this.ctl.rearm();   // the simulation continues where the real arm was
    }
    this.target = target;
    this.log(`backend: ${target}`);
    return this.info();
  }

  /** Make the shared simulation show the real arm's commanded pose, poses, limits and demo. */
  syncFromReal(s, ctl = this.ctl) {
    this.realState = s;
    const c = [0, 1, 2, 3, 4, 5].map(i => s.commands?.[i] ?? s.commands?.[String(i)]);
    if (c.some(v => typeof v !== 'number')) return;
    if (s.joints) s.joints.forEach((j, i) => {
      if (j.home != null) ctl.poses.HOME[i] = j.home;
      if (j.park != null) ctl.poses.PARK[i] = j.park;
      if (j.low != null && j.high != null) ctl.bounds[i] = [j.low, j.high];
    });
    if (s.settings) Object.assign(ctl.settings, s.settings);
    if (s.demo) ctl.demo = JSON.parse(JSON.stringify(s.demo));
    ctl.setCommands(c);
    ctl.outputsOff = !!s.outputs_off; ctl.armed = !s.outputs_off && s.armed !== false; ctl.error = s.error || null;
  }

  // ---------------- the robot API ----------------

  async get(path) {
    if (path !== 'state') throw new ControllerError(`Unknown path ${path}`);
    if (this.target === 'real') {
      const s = await this.real.get('state');
      if (!this.ctl.busy) this.syncFromReal(s);
      return { ...s, backend: 'real' };
    }
    return { ...this.ctl.state(), mode: 'simulation', backend: 'sim', position: this.ctl.position.map(v => Math.round(v * 10) / 10) };
  }

  async post(path, body = {}) {
    const actions = controllerActions(this.ctl);
    if (!actions[path]) throw new ControllerError(`Unknown path ${path}`);
    if (this.target === 'sim') { actions[path](body); return { ok: true, result: true, backend: 'sim' }; }
    if (path !== 'stop' && !this.allowMotion) throw new Blocked('Motion to the real arm is disabled: restart the server with --allow-motion.');
    if (!SAFE.has(path)) {
      const s = await this.real.get('state');
      const check = this.simulate(path, body, s);
      if (!check.ok) throw new Blocked(`blocked by the simulation: ${check.reason}`);
    }
    const r = await this.real.post(path, body);
    this.log(`real: ${path} ${JSON.stringify(body)}`);
    return { ...r, backend: 'real' };
  }

  /**
   * Play one API call on a copy of the shared simulation synced to the real arm's state `s`.
   * Returns { ok, reason, duration }.
   */
  simulate(path, body, s) {
    const ctl = new SimController(), w = new World();
    this.syncFromReal(s, ctl);
    this.realState = s;
    w.restore(this.world.snapshot());
    w.step(0, ctl.position);
    const before = new Set(hardContacts(w));
    const allowed = k => before.has(k) || (parks(path, body) && restingOnBase(k));
    try { controllerActions(ctl)[path](body); }
    catch (e) { return { ok: false, reason: e.message }; }
    const dt = 1 / 60;
    let t = 0;
    while ((ctl.busy || !w.settled()) && t < 600) {
      ctl.update(dt); w.step(dt, ctl.position); t += dt;
      const fresh = hardContacts(w).filter(k => !allowed(k));
      if (fresh.length) return { ok: false, reason: `collision ${fresh.join(', ')} after ${t.toFixed(1)} s (${ctl.motion?.label || path})` };
    }
    return { ok: true, duration: t };
  }
}
