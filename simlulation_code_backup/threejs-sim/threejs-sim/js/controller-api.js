// The controller's HTTP API (lutich/robot-arm docs/api.md) on top of a SimController.
// One backend for every simulated arm: server.mjs --mock serves it over HTTP, and the page's own
// simulated arm (armapi.js SimArm) calls it directly, so both answer exactly like the real controller.

import { ControllerError } from './controller.js';

/** POST /api/<action> handlers for a SimController. */
export function controllerActions(ctl) {
  return {
    move: b => ctl.move(b.channel, b.target),
    jog: b => ctl.jog(b.channel, b.direction),
    go_pose: b => ctl.go_pose(b.name),
    stop: b => ctl.stop(b.reason || 'stop'),
    power_on: () => ctl.power_on(),
    power_off: () => ctl.power_off(),
    settings: b => ctl.settings_(b.step_size, b.demo_speed),
    limits: b => ctl.limits(b.channel, b.low, b.high),
    'demo/add': b => ctl.demo_add(b.name),
    'demo/replace': b => ctl.demo_replace(b.index),
    'demo/remove': b => ctl.demo_remove(b.index),
    'demo/rename': b => ctl.demo_rename(b.index, b.name),
    'demo/reorder': b => ctl.demo_reorder(b.index, b.direction),
    'demo/next': () => ctl.demo_next(),
    'demo/run': () => ctl.demo_run(),
    'demo/pause': () => ctl.demo_pause(),
    'demo/resume': () => ctl.demo_resume(),
    'demo/restart': () => ctl.demo_restart(),
  };
}

/** GET state and POST actions, with the controller's answers: { ok, result } or a ControllerError. */
export function controllerApi(ctl, mode) {
  const actions = controllerActions(ctl);
  return {
    get(path) {
      if (path === 'state') return { ...ctl.state(), mode };
      throw new ControllerError(`Unknown path ${path}`);
    },
    post(path, body = {}) {
      if (!actions[path]) throw new ControllerError(`Unknown path ${path}`);
      actions[path](body);
      return { ok: true, result: true };
    },
  };
}
