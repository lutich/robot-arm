const READ_PATHS = new Set(['/api/state', '/api/camera/status', '/api/camera/snapshot.jpg']);
const STOP_PATH = '/api/stop';
const MOTION_PATHS = new Set(['/api/power_on', '/api/power_off', '/api/move', '/api/go_pose']);

export function proxyDecision(method, pathname, { allowMotion = false } = {}) {
  const verb = String(method).toUpperCase();
  if (verb === 'GET' && READ_PATHS.has(pathname)) return { allowed: true, kind: 'read' };
  if (verb === 'POST' && pathname === STOP_PATH) return { allowed: true, kind: 'stop' };
  if (verb === 'POST' && MOTION_PATHS.has(pathname)) {
    return allowMotion
      ? { allowed: true, kind: 'motion' }
      : { allowed: false, status: 403, error: 'Motion is disabled in this gateway. Restart with --allow-motion for an attended operational session.' };
  }
  return { allowed: false, status: 404, error: 'The gateway exposes only state, camera status/snapshot, stop, power, single-joint move and HOME/PARK pose endpoints.' };
}

export function connectionInfo(controller, allowMotion) {
  return {
    backend: controller ? 'controller-proxy' : 'none',
    controllerConfigured: Boolean(controller),
    motionAllowed: Boolean(controller && allowMotion),
    policy: !controller ? 'simulation-only' : allowMotion ? 'explicit-motion' : 'read-only-plus-stop',
  };
}
