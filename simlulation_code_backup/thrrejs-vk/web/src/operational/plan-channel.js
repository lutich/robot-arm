// Plan channel: Claude (terminal) publishes a task plan; the browser simulates it live,
// reports progress, and only an operator click in the page sends it to the robot.
// Shared by the production gateway (server.mjs) and the Vite dev server (vite.config.js).
// No robot API access happens here.

export const PLAN_LIMITS = Object.freeze({ maxSteps: 3000, maxCubes: 24, maxBodyBytes: 512 * 1024 });
export const PLAN_PHASES = new Set(['sent', 'simulating', 'simulated', 'sim-failed', 'stale', 'cleared',
  'running-live', 'live-complete', 'live-halted', 'live-failed', 'error']);
const ROUTES = new Set(['/simulator/plan', '/simulator/plan/progress', '/simulator/scene']);

const isRecord = (value) => value !== null && typeof value === 'object' && !Array.isArray(value);

function text(value, field, maximum, { optional = false } = {}) {
  if (optional && (value === undefined || value === null)) return null;
  if (typeof value !== 'string' || !value.trim() || value.length > maximum) throw new Error(`${field} must be 1..${maximum} characters`);
  return value.trim();
}

function counts(value, field) {
  if (!Array.isArray(value) || value.length !== 6 || !value.every((count) => Number.isInteger(count) && count >= 0 && count <= 4095)) {
    throw new Error(`${field} must be six integer servo counts`);
  }
  return value.slice();
}

function finiteTriple(value, field) {
  if (!Array.isArray(value) || value.length !== 3 || !value.every(Number.isFinite)) throw new Error(`${field} must be three finite numbers`);
  return value.slice();
}

function sceneSnapshot(value, field) {
  if (!isRecord(value) || !Array.isArray(value.cubes) || value.cubes.length > PLAN_LIMITS.maxCubes) {
    throw new Error(`${field} must contain at most ${PLAN_LIMITS.maxCubes} cubes`);
  }
  value.cubes.forEach((cube, index) => {
    if (!isRecord(cube) || typeof cube.id !== 'string' || typeof cube.colour !== 'string') throw new Error(`${field} cube ${index} needs id and colour`);
    finiteTriple(cube.p, `${field} cube ${cube.id} position`);
  });
  return structuredClone(value);
}

/** Validate a plan from Claude: every step must change exactly one servo, as the robot API executes it. */
export function validatePlan(input) {
  if (!isRecord(input)) throw new Error('Plan must be a JSON object');
  const start = counts(input.start, 'start');
  if (!Array.isArray(input.steps) || !input.steps.length || input.steps.length > PLAN_LIMITS.maxSteps) {
    throw new Error(`steps must contain 1..${PLAN_LIMITS.maxSteps} single-servo steps`);
  }
  let previous = start;
  const steps = input.steps.map((step, index) => {
    if (!isRecord(step)) throw new Error(`step ${index + 1} must be an object`);
    const next = counts(step.counts, `step ${index + 1} counts`);
    const changed = next.filter((count, channel) => count !== previous[channel]).length;
    if (changed !== 1) throw new Error(`step ${index + 1} changes ${changed} servos; exactly one is required`);
    previous = next;
    return { label: text(step.label, `step ${index + 1} label`, 120), counts: next };
  });
  if (!isRecord(input.expect) || !['pick', 'pick-place'].includes(input.expect.type)) throw new Error('expect.type must be pick or pick-place');
  const precheck = isRecord(input.precheck)
    ? { ok: Boolean(input.precheck.ok), offsetCm: Number.isFinite(input.precheck.offsetCm) ? input.precheck.offsetCm : null }
    : null;
  return {
    id: text(input.id, 'id', 80),
    task: text(input.task, 'task', 180),
    description: text(input.description, 'description', 300, { optional: true }),
    createdAt: text(input.createdAt, 'createdAt', 40),
    start,
    steps,
    scene: sceneSnapshot(input.scene, 'scene'),
    expect: { type: input.expect.type, cube: text(input.expect.cube, 'expect.cube', 40), target: finiteTriple(input.expect.target, 'expect.target') },
    waypoints: Array.isArray(input.waypoints) ? input.waypoints.slice(0, 40).map((waypoint, index) => ({
      label: text(waypoint?.label, `waypoint ${index + 1} label`, 60),
      counts: counts(waypoint?.counts, `waypoint ${index + 1} counts`),
    })) : [],
    estimatedSeconds: Number.isFinite(input.estimatedSeconds) ? input.estimatedSeconds : null,
    precheck,
  };
}

export function validateProgress(input, revision) {
  if (!isRecord(input)) throw new Error('Progress must be a JSON object');
  if (input.revision !== revision) throw new Error(`Progress is for plan revision ${input.revision}; the current plan is ${revision}`);
  if (!PLAN_PHASES.has(input.phase)) throw new Error(`Unknown plan phase: ${input.phase}`);
  const step = Number.isInteger(input.step) && input.step >= 0 ? input.step : 0;
  const total = Number.isInteger(input.total) && input.total >= 0 ? input.total : 0;
  return {
    revision,
    phase: input.phase,
    step,
    total,
    ok: typeof input.ok === 'boolean' ? input.ok : null,
    message: text(input.message ?? input.phase, 'message', 500),
    offsetCm: Number.isFinite(input.offsetCm) ? input.offsetCm : null,
    updatedAt: new Date().toISOString(),
  };
}

/** The page publishes what it shows (cubes, commanded counts, calibration) so Claude plans against it. */
export function validateScene(input) {
  if (!isRecord(input)) throw new Error('Scene must be a JSON object');
  return {
    commands: counts(input.commands, 'commands'),
    snapshot: sceneSnapshot(input.snapshot, 'snapshot'),
    calibration: isRecord(input.calibration) ? structuredClone(input.calibration) : null,
    mode: text(input.mode, 'mode', 40),
    source: text(input.source, 'source', 80),
    syncedAt: text(input.syncedAt, 'syncedAt', 40, { optional: true }),
    receivedAt: new Date().toISOString(),
  };
}

function sendJson(response, status, data) {
  response.writeHead(status, {
    'Content-Type': 'application/json; charset=utf-8',
    'Cache-Control': 'no-store',
    'X-Content-Type-Options': 'nosniff',
  });
  response.end(JSON.stringify(data));
}

function readJson(request) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let length = 0;
    request.on('data', (chunk) => {
      length += chunk.length;
      if (length > PLAN_LIMITS.maxBodyBytes) {
        reject(new Error(`Request body exceeds ${PLAN_LIMITS.maxBodyBytes} bytes`));
        request.destroy();
      } else chunks.push(chunk);
    });
    request.on('end', () => {
      try { resolve(JSON.parse(Buffer.concat(chunks).toString('utf8'))); }
      catch (error) { reject(new Error(`Invalid JSON: ${error.message}`)); }
    });
    request.on('error', reject);
  });
}

export function createPlanChannel() {
  let planRevision = 0;
  let plan = null;
  let progress = null;
  let sceneRevision = 0;
  let scene = null;

  async function post(request, response, pathname) {
    const origin = request.headers.origin;
    if (origin && origin !== `http://${request.headers.host}`) return sendJson(response, 403, { error: 'Plan channel updates must be same-origin' });
    if (!String(request.headers['content-type'] ?? '').toLowerCase().startsWith('application/json')) {
      return sendJson(response, 415, { error: 'Content-Type application/json required' });
    }
    let input;
    try { input = await readJson(request); }
    catch (error) { return sendJson(response, 400, { error: error.message }); }
    try {
      if (pathname === '/simulator/plan') {
        plan = validatePlan(input);
        planRevision += 1;
        progress = { revision: planRevision, phase: 'sent', step: 0, total: plan.steps.length, ok: null,
          message: 'Plan published; waiting for an open simulator page.', offsetCm: null, updatedAt: new Date().toISOString() };
        return sendJson(response, 200, { revision: planRevision, steps: plan.steps.length });
      }
      if (pathname === '/simulator/plan/progress') {
        if (!plan) return sendJson(response, 409, { error: 'No plan has been published' });
        progress = validateProgress(input, planRevision);
        return sendJson(response, 200, progress);
      }
      scene = { revision: ++sceneRevision, ...validateScene(input) };
      return sendJson(response, 200, { revision: scene.revision });
    } catch (error) {
      return sendJson(response, 400, { error: error.message });
    }
  }

  return {
    handles: (pathname) => ROUTES.has(pathname),
    /** Node http handler; returns a promise. Only call when handles(pathname) is true. */
    async handle(request, response, pathname) {
      if (request.method === 'GET') {
        if (pathname === '/simulator/plan') return sendJson(response, 200, { revision: planRevision, plan });
        if (pathname === '/simulator/plan/progress') return sendJson(response, 200, progress ?? { revision: planRevision, phase: null });
        return scene ? sendJson(response, 200, scene) : sendJson(response, 404, { error: 'No scene yet; open the simulator page' });
      }
      if (request.method === 'POST') return post(request, response, pathname);
      return sendJson(response, 405, { error: 'GET or POST required' });
    },
  };
}
