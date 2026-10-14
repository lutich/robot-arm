// One client for the simulated and the real arm: the controller's HTTP API.
//
//   LiveLink  the real controller over HTTP (live.js), or the mock of server.mjs --mock
//   SimArm    the same client, but its requests go straight to the page's simulated arm through the
//             same backend code the mock uses (controller-api.js), so every call behaves the same
//
// The planner computes a trajectory as a list of API calls, { op: 'move', ch, target, label },
// { op: 'go_pose', name, label } or { op: 'power_on', label }; runCalls() executes it on either arm.
// A run on the real arm is the same list that just ran on the simulated arm.

import { LiveLink } from './live.js';
import { controllerApi } from './controller-api.js';

export class SimArm extends LiveLink {
  constructor(ctl) { super(''); this.ctl = ctl; this.api = controllerApi(ctl, 'simulation'); }
  async get(path) { return this.api.get(path); }
  async post(path, body = {}) { return this.api.post(path, body); }
}

/** Calls for a list of single-joint moves (planner.js toMoves). */
export const movesToCalls = moves => moves.map(m => ({ op: 'move', ch: m.ch, target: m.target, label: m.label }));

/** Run calls one after the other on an arm (SimArm or LiveLink), each after the previous has settled. */
export async function runCalls(arm, calls, { onStep = () => {} } = {}) {
  arm.abort = false;
  if (calls[0]?.op !== 'power_on') {   // power_on is what switches stopped outputs back on
    const s0 = await arm.waitIdle(30);
    if (s0.outputs_off) throw new Error('the outputs are off: power on first');
  }
  for (let i = 0; i < calls.length; i++) {
    if (arm.abort) throw new Error('aborted');
    const c = calls[i];
    onStep(i, c);
    if (c.op === 'move') {
      const from = LiveLink.counts(await arm.read())[c.ch];
      if (from === c.target) continue;
      await arm.move(c.ch, c.target);
      const done = await arm.waitIdle(1.5 * Math.abs(c.target - from) / 20 + 10);
      if (done.outputs_off) throw new Error(`${c.label}: stopped (${done.error || 'outputs off'})`);
      if (LiveLink.counts(done)[c.ch] !== c.target) throw new Error(`${c.label}: the arm reports ${LiveLink.counts(done)[c.ch]}`);
    } else if (c.op === 'go_pose') {
      await arm.goPose(c.name);
      await arm.waitIdle(60);
    } else if (c.op === 'power_on') {
      await arm.powerOn();
      await new Promise(r => setTimeout(r, 300));
      await arm.waitIdle(60);
    } else throw new Error(`unknown call ${c.op}`);
  }
}
