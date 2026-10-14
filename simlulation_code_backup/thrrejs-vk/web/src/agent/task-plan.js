// Task planning shared by the page's command box and Claude's terminal planner: the page's
// top-down pick-and-place planner (the one the tower demo uses) and its grasp model, then
// split into single-servo steps, the only motion the robot API executes. Deterministic; no AI.
import { SERVOS, MAT, CONTROLLER } from '../../js/config.js';
import { countsObject, countsArray } from '../../js/advanced.js';
import { planTransfer, parseCommand, verify, PlanError } from '../../js/planner.js';
import { controllerCompatiblePath } from '../core/planning.js';

const COLOURS = ['orange', 'blue', 'yellow', 'green', 'red'];

/** "grab green" picks and holds; everything else is the page grammar ("blue on orange", "green to 5 15" in board cm). */
export function parseTask(text, world) {
  const words = text.toLowerCase().split(/[\s,;]+/).filter(Boolean);
  const colours = words.filter((word) => COLOURS.includes(word));
  const hasNumbers = words.some((word) => Number.isFinite(Number(word)));
  if (/\b(grab|grip|pick|hold)\b/.test(text.toLowerCase()) && colours.length === 1 && !hasNumbers) {
    if (!world.cube(colours[0])) throw new PlanError(`no ${colours[0]} cube in the scene`);
    return { pick: colours[0], place: null };
  }
  return parseCommand(text, world);
}

/** Apply the calibration the page uses (its Calibration panel values), so plan and display agree. */
export function applyPageCalibration(calibration) {
  if (!calibration?.servos?.length) return false;
  calibration.servos.slice(0, 5).forEach((values, index) => Object.assign(SERVOS[index], values));
  if (Number.isFinite(calibration.d)) MAT.sToEdge = calibration.d;
  return true;
}

/** Controller timing for a step sequence: each move takes 1,5 x |delta| / manual speed. */
export function estimateSeconds(start, steps) {
  let previous = start;
  let seconds = 0;
  for (const step of steps) {
    const delta = Math.max(...step.counts.map((count, channel) => Math.abs(count - previous[channel])));
    seconds += CONTROLLER.peakFactor * delta / CONTROLLER.manualSpeed;
    previous = step.counts;
  }
  return seconds;
}

/** Split coordinated poses into single-servo steps of at most `stepCounts` counts. */
export function singleServoSteps(spec, start, poses, stepCounts) {
  const planning = { ...spec, planning: { ...spec.planning, controllerStepCounts: stepCounts } };
  const steps = [];
  let previous = start.slice();
  for (const pose of poses) {
    const segment = [];
    for (const next of controllerCompatiblePath(planning, [countsObject(previous, spec), countsObject(pose.counts, spec)]).path.slice(1)) {
      const counts = countsArray(next, spec);
      if (counts.every((count, channel) => count === (segment.at(-1) ?? previous)[channel])) continue;
      segment.push(counts);
    }
    segment.forEach((counts, index) => steps.push({ label: segment.length > 1 ? `${pose.label} (${index + 1}/${segment.length})` : pose.label, counts }));
    if (segment.length) previous = segment.at(-1);
  }
  return steps;
}

/**
 * Plan `text` in `world` from `start` (six counts). Returns { success, plan, report } or
 * { success: false, reason }. `report` is the headless pre-check of the single-servo steps.
 */
export function buildTaskPlan({ spec, world, start, text, stepCounts = null, precheck = true }) {
  let command; let coordinated;
  try {
    command = parseTask(text, world);
    coordinated = planTransfer(world, command, start);
  } catch (error) {
    if (error instanceof PlanError) return { success: false, reason: error.message };
    throw error;
  }
  const steps = singleServoSteps(spec, start, coordinated.steps, stepCounts ?? spec.planning.controllerStepCounts ?? 8);
  if (!steps.length) return { success: false, reason: 'The arm is already at the task goal; nothing to do' };
  const { info } = coordinated;
  const report = precheck ? verify(world, { steps, info }, start, { mode: 'demo', speed: CONTROLLER.manualSpeed }) : null;
  const where = info.retainGrasp ? 'hold it above its place' : info.onWhat === 'board'
    ? `place it at board (${info.target[0].toFixed(1)}, ${info.target[1].toFixed(1)}) cm` : `place it on ${info.onWhat}`;
  return {
    success: true,
    report,
    plan: {
      id: `plan-${Date.now().toString(36)}`,
      task: text.trim(),
      description: `Pick ${info.pick} and ${where}`,
      createdAt: new Date().toISOString(),
      start: start.slice(),
      steps,
      scene: world.snapshot(),
      expect: { type: info.retainGrasp ? 'pick' : 'pick-place', cube: info.pick, target: [...info.target] },
      waypoints: coordinated.steps.map((step) => ({ label: step.label, counts: step.counts.slice() })),
      estimatedSeconds: estimateSeconds(start, steps),
      precheck: report ? { ok: report.ok, offsetCm: report.offset } : null,
    },
  };
}
