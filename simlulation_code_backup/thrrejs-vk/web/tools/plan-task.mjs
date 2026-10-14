#!/usr/bin/env node
// Claude's planner: plan a task against the scene the simulator page shows, pre-check it
// headless and publish it; the page simulates it live and only the operator can run it on
// the robot. This tool reads robot state but never sends robot commands.
//
//   node tools/plan-task.mjs "blue on orange" [--base http://127.0.0.1:8770] [--step 8] [--wait] [--dry-run] [--force]
//   node tools/plan-task.mjs --status [--base ...]
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { validateArmSpec } from '../src/config/spec.js';
import { SERVOS } from '../js/config.js';
import { configureCanonicalSpec, applyCanonicalServoMetadata } from '../js/advanced.js';
import { World } from '../js/world.js';
import { applyPageCalibration, buildTaskPlan } from '../src/agent/task-plan.js';

const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const args = process.argv.slice(2);
const option = (name, fallback = null) => { const index = args.indexOf(name); return index >= 0 ? args[index + 1] : fallback; };
const flag = (name) => args.includes(name);
const base = option('--base', 'http://127.0.0.1:8770').replace(/\/$/, '');
const text = args.find((arg, index) => !arg.startsWith('--') && !['--base', '--step'].includes(args[index - 1]));

async function getJson(url) {
  const response = await fetch(url, { cache: 'no-store' });
  const body = await response.text();
  let data;
  try { data = JSON.parse(body); } catch { throw new Error(`${url} returned non-JSON (HTTP ${response.status})`); }
  if (!response.ok) throw new Error(data.error || `${url} HTTP ${response.status}`);
  return data;
}

async function postJson(url, data) {
  const response = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(data) });
  const result = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(result.error || `${url} HTTP ${response.status}`);
  return result;
}

const minutes = (seconds) => seconds >= 90 ? `${(seconds / 60).toFixed(1)} min` : `${seconds.toFixed(0)} s`;

async function printStatus() {
  const { revision, plan } = await getJson(`${base}/simulator/plan`);
  const progress = await getJson(`${base}/simulator/plan/progress`);
  if (!plan) { console.log('No plan published.'); return progress; }
  console.log(`Plan r${revision}: "${plan.task}" — ${plan.steps.length} steps`);
  console.log(`Phase: ${progress.phase ?? 'none'} · step ${progress.step}/${progress.total} · ${progress.message}${progress.updatedAt ? ` (${progress.updatedAt})` : ''}`);
  return progress;
}

async function main() {
  if (flag('--status')) { await printStatus(); return; }
  if (!text) throw new Error('Usage: node tools/plan-task.mjs "<task>" [--base URL] [--step N] [--wait] [--dry-run] [--force]');

  const scene = await getJson(`${base}/simulator/scene`);
  let start = scene.commands; let startSource = `page (${scene.mode})`;
  try {
    const state = await getJson(`${base}/api/state`);
    const commands = [0, 1, 2, 3, 4, 5].map((channel) => state.commands?.[channel] ?? state.commands?.[String(channel)]);
    if (commands.every(Number.isInteger)) { start = commands; startSource = `robot controller (${state.mode}, ${state.armed ? 'armed' : 'not armed'})`; }
  } catch { /* no controller behind this server: plan from the page's commanded counts */ }

  const spec = validateArmSpec(JSON.parse(fs.readFileSync(path.join(ROOT, 'public/arm-spec.json'), 'utf8')));
  configureCanonicalSpec(spec);
  applyCanonicalServoMetadata(SERVOS, spec);
  applyPageCalibration(scene.calibration);
  const world = new World(SERVOS);
  world.restore(scene.snapshot);

  console.log(`Scene: ${world.cubes.map((cube) => `${cube.id} (${cube.p[0].toFixed(1)}, ${cube.p[1].toFixed(1)})`).join(', ') || 'no cubes'} · source ${scene.source}${scene.syncedAt ? ` @ ${scene.syncedAt}` : ''}`);
  console.log(`Start: [${start.join(', ')}] from ${startSource}`);
  const result = buildTaskPlan({ spec, world, start, text, stepCounts: option('--step') ? Number(option('--step')) : null });
  if (!result.success) { console.error(`Not planned: ${result.reason}`); process.exitCode = 1; return; }
  const { plan } = result;
  console.log(`Plan: ${plan.description}; ${plan.waypoints.length} poses -> ${plan.steps.length} single-servo steps, ~${minutes(plan.estimatedSeconds)} at robot speed`);
  console.log(`Pre-check (headless grasp model): ${plan.precheck.ok ? 'PASS' : 'FAIL'} · cube ends ${(plan.precheck.offsetCm * 10).toFixed(1)} mm from target`);
  if (!plan.precheck.ok && !flag('--force')) { console.error('Not published: the pre-check failed (use --force to watch it fail in the page).'); process.exitCode = 2; return; }
  if (flag('--dry-run')) return;

  const { revision } = await postJson(`${base}/simulator/plan`, plan);
  console.log(`Published as plan r${revision}; the page simulates it now.`);
  if (!flag('--wait')) return;
  const deadline = Date.now() + Math.max(120, plan.estimatedSeconds * 2) * 1000;
  let last = '';
  while (Date.now() < deadline) {
    await new Promise((resolve) => setTimeout(resolve, 2000));
    const progress = await getJson(`${base}/simulator/plan/progress`);
    if (progress.revision !== revision) { console.log('A newer plan replaced this one.'); return; }
    const line = progress.message;
    if (line !== last) { console.log(line); last = line; }
    if (['simulated', 'sim-failed', 'stale', 'cleared', 'error'].includes(progress.phase)) return;
  }
  console.log('Stopped waiting; is the simulator page open and visible?');
}

main().catch((error) => { console.error(error.message); process.exitCode = 1; });
