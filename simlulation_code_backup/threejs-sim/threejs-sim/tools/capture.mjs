// Saves one frame from each camera and the arm state, taken together, into a folder of its own:
// docs/images/calibration/<date>-<time>[-name]/ with oak.jpg, webcam.jpg, state.json and info.json.
// Read only: one GET per camera and one GET /api/state; nothing is sent to the arm.
//
//   node threejs-sim/tools/capture.mjs                         folder named by date and time
//   node threejs-sim/tools/capture.mjs gripper down blue cube  ... plus a name (spaces become dashes)
//   node threejs-sim/tools/capture.mjs --note "S5 at 480" name a note for info.json
//   node threejs-sim/tools/capture.mjs --controller http://10.102.4.103:8765   (the default)
//   node threejs-sim/tools/capture.mjs --out <dir>             default: docs/images/calibration
//
// For recalibration (docs/recalibration.md) say where the cube in the gripper is: its spot on the mat and its level
//   --at marker3            on mat marker 3 (any marker id of js/calib.js MARKER_CORNERS: its centre)
//   --at 300,0              at X 300, Y 0 mm (camera table frame: X away from the arm, Y to the arm's left)
//   --level 1               the held cube sits on one other cube (0: on the mat, the default)
//   --at 300,0,40           or the height of the cube's centre in mm directly
// e.g.  node threejs-sim/tools/capture.mjs --at marker2 --level 1 stack on marker 2

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { MARKER_CORNERS, markerCentre } from '../js/calib.js';
import { CUBE } from '../js/config.js';

const ROOT = path.dirname(path.dirname(path.dirname(fileURLToPath(import.meta.url))));
const args = process.argv.slice(2);
const opts = {}, words = [];
for (let i = 0; i < args.length; i++) {
  if (args[i].startsWith('--')) opts[args[i].slice(2)] = args[++i];
  else words.push(args[i]);
}
const base = (opts.controller || 'http://10.102.4.103:8765').replace(/\/$/, '');
const outRoot = path.resolve(opts.out || path.join(ROOT, 'docs', 'images', 'calibration'));
const SOURCES = [
  { file: 'oak.jpg', url: '/api/camera/snapshot.jpg', what: 'OAK on the stand' },
  { file: 'webcam.jpg', url: '/api/camera2/snapshot.jpg', what: 'webcam on the monitor' },
  { file: 'state.json', url: '/api/state', what: 'arm state (commanded counts)' },
];

// the held cube: its centre in the camera table frame (mm)
let target = null;
if (opts.at) {
  const m = opts.at.match(/^marker(\d+)$/i), level = Number(opts.level || 0);
  let at;
  if (m) {
    if (!MARKER_CORNERS[m[1]]) { console.error(`no mat marker ${m[1]} (js/calib.js MARKER_CORNERS has ${Object.keys(MARKER_CORNERS).join(', ')})`); process.exit(1); }
    at = markerCentre(m[1]);
  } else {
    at = opts.at.split(',').map(Number);
    if (at.length < 2 || at.some(v => !Number.isFinite(v))) { console.error('--at wants markerN or x,y[,z] in mm'); process.exit(1); }
  }
  const z = at[2] ?? CUBE * 10 * (0.5 + level);
  target = { what: 'centre of the cube held in the gripper', at: [at[0], at[1], +z.toFixed(2)], level, spot: opts.at };
}

const pad = n => String(n).padStart(2, '0');
const now = new Date();
const stamp = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}-${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
const name = words.join(' ').trim().toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
const dir = path.join(outRoot, name ? `${stamp}-${name}` : stamp);

// all three reads at once, so the frames and the counts belong to the same moment
async function get(src) {
  const t0 = Date.now();
  const ctrl = new AbortController(), timer = setTimeout(() => ctrl.abort(), 15000);
  try {
    const r = await fetch(base + src.url, { cache: 'no-store', signal: ctrl.signal });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    return { ...src, data: Buffer.from(await r.arrayBuffer()), ms: Date.now() - t0 };
  } catch (e) {
    return { ...src, error: e.name === 'AbortError' ? 'timeout' : e.message };
  } finally { clearTimeout(timer); }
}
const results = await Promise.all(SOURCES.map(get));
if (results.every(r => r.error)) {
  console.error(`nothing read from ${base}: ${results.map(r => `${r.file} ${r.error}`).join(', ')}`);
  process.exit(1);
}

fs.mkdirSync(dir, { recursive: true });
for (const r of results) if (!r.error) fs.writeFileSync(path.join(dir, r.file), r.data);

let counts = null, armNote = '';
const st = results.find(r => r.file === 'state.json');
if (!st.error) {
  const s = JSON.parse(st.data.toString());
  const cmd = [0, 1, 2, 3, 4, 5].map(i => s.commands?.[i]);
  // with the outputs off the controller has no commands; the last ones still say where the arm is
  counts = cmd.every(v => v != null) ? cmd : [0, 1, 2, 3, 4, 5].map(i => s.last_command?.[i] ?? null);
  armNote = s.outputs_off ? `outputs off (${s.error || 'no reason given'}): counts are the last commanded ones` : s.busy ? 'arm was moving' : 'arm still';
}
const info = {
  taken: now.toISOString(),
  name: words.join(' ') || null,
  note: opts.note || null,
  controller: base,
  counts, arm: armNote,
  target,
  files: Object.fromEntries(results.map(r => [r.file, r.error ? `missing: ${r.error}` : `${r.what}, GET ${r.url}, ${r.ms} ms`])),
};
fs.writeFileSync(path.join(dir, 'info.json'), JSON.stringify(info, null, 2) + '\n');

console.log(path.relative(process.cwd(), dir));
for (const r of results) console.log(`  ${r.file.padEnd(10)} ${r.error ? `MISSING (${r.error})` : `${(r.data.length / 1024).toFixed(0)} kB`}`);
if (counts) console.log(`  counts     ${counts.join(' ')} (${armNote})`);
if (target) console.log(`  target     cube centre at ${target.at.join(' / ')} mm (${target.spot}, level ${target.level})`);
else console.log('  target     none (add --at markerN|x,y --level N to use this capture for recalibration)');
