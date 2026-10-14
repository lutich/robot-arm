#!/usr/bin/env node
// Offline visual validation: model | reference photo | 50% overlay.
// Build first. No controller is configured and no robot API request is made.

import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const REPO = path.dirname(ROOT);
const args = process.argv.slice(2);
const option = (name, fallback) => { const index = args.indexOf(name); return index >= 0 ? args[index + 1] : fallback; };
const only = option('--only', '');
const outputDirectory = path.resolve(option('--out', path.join(REPO, 'docs', 'images', 'compare')));
const look = option('--look', 'office');
const viewsFile = path.resolve(option('--views', path.join(ROOT, 'tools', 'photo-views.json')));
const port = Number(option('--port', '8779'));
const debuggingPort = port + 1;
const chromeCandidates = [
  process.env.CHROME,
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
  '/Applications/Chromium.app/Contents/MacOS/Chromium',
  '/usr/bin/google-chrome',
  '/usr/bin/chromium',
  'C:/Program Files/Google/Chrome/Application/chrome.exe',
].filter(Boolean);
const chromePath = chromeCandidates.find((candidate) => fs.existsSync(candidate));

if (!fs.existsSync(path.join(ROOT, 'dist', 'index.html'))) throw new Error('Run npm run build before npm run compare:photos');
if (!chromePath) throw new Error('Chrome/Chromium was not found; set CHROME to its executable path');
const views = JSON.parse(fs.readFileSync(viewsFile, 'utf8')).views.filter((view) => view.photo.includes(only));
if (!views.length) throw new Error('No matching reference views');
fs.mkdirSync(outputDirectory, { recursive: true });

const sleep = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));
const server = spawn(process.execPath, [path.join(ROOT, 'server.mjs'), '--port', String(port)], { stdio: 'ignore' });
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'px-arm-photo-'));
const chrome = spawn(chromePath, [
  '--headless=new', `--remote-debugging-port=${debuggingPort}`, `--user-data-dir=${profile}`,
  '--remote-allow-origins=*', '--window-size=1400,900', '--use-angle=swiftshader',
  '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check', 'about:blank',
], { stdio: 'ignore' });

function cleanup() {
  try { chrome.kill(); } catch { /* already stopped */ }
  try { server.kill(); } catch { /* already stopped */ }
  try { fs.rmSync(profile, { recursive: true, force: true }); } catch { /* temporary cleanup only */ }
}
process.on('exit', cleanup);
process.on('SIGINT', () => process.exit(130));

for (let attempt = 0; attempt < 40; attempt += 1) {
  try { if ((await fetch(`http://127.0.0.1:${port}/connection-info`)).ok) break; } catch { /* starting */ }
  if (attempt === 39) throw new Error('Comparison server did not start');
  await sleep(100);
}

let targets = [];
for (let attempt = 0; attempt < 60 && !targets.some((target) => target.type === 'page'); attempt += 1) {
  try { targets = await (await fetch(`http://127.0.0.1:${debuggingPort}/json`)).json(); } catch { /* Chrome starting */ }
  await sleep(100);
}
const page = targets.find((target) => target.type === 'page');
if (!page) throw new Error('Headless Chrome did not start');

const socket = new WebSocket(page.webSocketDebuggerUrl);
await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
let sequence = 0;
const pending = new Map();
socket.onmessage = (event) => {
  const message = JSON.parse(event.data);
  if (message.id && pending.has(message.id)) { pending.get(message.id)(message); pending.delete(message.id); }
};
const send = (method, params = {}) => new Promise((resolve) => {
  const id = ++sequence; pending.set(id, resolve); socket.send(JSON.stringify({ id, method, params }));
});
const evaluate = async (expression) => {
  const response = await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
  if (response.result?.exceptionDetails) throw new Error(response.result.exceptionDetails.exception?.description || response.result.exceptionDetails.text);
  return response.result?.result?.value;
};

await send('Runtime.enable');
await send('Page.navigate', { url: `http://127.0.0.1:${port}/` });
for (let attempt = 0; attempt < 80; attempt += 1) {
  if (await evaluate('Boolean(window.sim)').catch(() => false)) break;
  if (attempt === 79) throw new Error('Simulator did not become ready');
  await sleep(100);
}

async function capture(viewDefinition, selectedLook) {
  const simulation = window.sim;
  const byId = (id) => document.getElementById(id);
  for (const id of ['show-path']) {
    const element = byId(id); if (element) { element.checked = false; element.dispatchEvent(new Event('change')); }
  }
  simulation.ctl.rearm();
  simulation.ctl.setCommands(viewDefinition.counts || simulation.ctl.poses[viewDefinition.pose || 'HOME'].slice());
  simulation.world.cubes = []; simulation.world.holding = null; simulation.world.dirty = true;
  await new Promise((resolve) => setTimeout(resolve, 500));

  const photo = new Image(); photo.src = `/reference-photos/${viewDefinition.photo}`; await photo.decode();
  const height = 900; const width = Math.round(height * photo.naturalWidth / photo.naturalHeight);
  const camera = simulation.camera; const renderer = simulation.renderer;
  const saved = { position: camera.position.clone(), quaternion: camera.quaternion.clone(), fov: camera.fov, aspect: camera.aspect,
    size: renderer.getSize(new simulation.THREE.Vector2()), ratio: renderer.getPixelRatio() };
  simulation.view.setLook(selectedLook); renderer.setPixelRatio(1); renderer.setSize(width, height, false);
  let calibrationInfo = null;

  if (viewDefinition.camera.markers) {
    const tools = simulation.photoReference;
    const scale = Math.min(1, 1280 / Math.max(photo.naturalWidth, photo.naturalHeight));
    const canvas = document.createElement('canvas'); canvas.width = Math.round(photo.naturalWidth * scale); canvas.height = Math.round(photo.naturalHeight * scale);
    const context = canvas.getContext('2d', { willReadFrequently: true }); context.drawImage(photo, 0, 0, canvas.width, canvas.height);
    const { markers } = tools.detectMarkers(context.getImageData(0, 0, canvas.width, canvas.height), { maxArea: 400000 });
    const ids = Object.keys(markers).filter((id) => tools.markerCorners[id] && !(viewDefinition.camera.skip || []).includes(Number(id)));
    if (ids.length < 2) throw new Error(`Only ${ids.length} mat markers found in ${viewDefinition.photo}`);
    const world = []; const image = [];
    for (const id of ids) tools.markerCorners[id].forEach((point, index) => {
      world.push([point[0], point[1], 0]); image.push([markers[id][index][0] / scale, markers[id][index][1] / scale]);
    });
    const calibration = tools.calibrate(world, image, { width: photo.naturalWidth, height: photo.naturalHeight, fixPrincipal: true });
    const direction = (row) => new simulation.THREE.Vector3(-row[1], row[0], row[2]);
    camera.position.set(...tools.tableToBoard(calibration.centre));
    camera.quaternion.setFromRotationMatrix(new simulation.THREE.Matrix4().makeBasis(direction(calibration.R[0]), direction(calibration.R[1]).negate(), direction(calibration.R[2]).negate()));
    camera.fov = 2 * Math.atan(photo.naturalHeight / 2 / calibration.f) * 180 / Math.PI;
    camera.aspect = width / height; camera.updateProjectionMatrix();
    calibrationInfo = { markers: ids, rms: Number(calibration.rms.toFixed(2)), fov: Number(camera.fov.toFixed(1)) };
  } else {
    camera.aspect = width / height; camera.fov = viewDefinition.camera.fov; camera.position.set(...viewDefinition.camera.pos); camera.up.set(0, 0, 1);
    camera.lookAt(...viewDefinition.camera.target); camera.rotateZ((viewDefinition.camera.roll || 0) * Math.PI / 180); camera.updateProjectionMatrix();
  }

  simulation.view.render();
  const output = document.createElement('canvas'); output.width = 3 * width; output.height = height;
  const context = output.getContext('2d');
  context.drawImage(renderer.domElement, 0, 0, width, height);
  context.drawImage(photo, width, 0, width, height);
  context.drawImage(photo, 2 * width, 0, width, height); context.globalAlpha = 0.5; context.drawImage(renderer.domElement, 2 * width, 0, width, height); context.globalAlpha = 1;
  context.font = 'bold 22px sans-serif'; context.fillStyle = '#fff'; context.strokeStyle = '#000'; context.lineWidth = 4;
  [['model', 0], [viewDefinition.photo, width], ['overlay 50%', 2 * width]].forEach(([label, x]) => { context.strokeText(label, x + 14, 34); context.fillText(label, x + 14, 34); });

  renderer.setPixelRatio(saved.ratio); renderer.setSize(saved.size.x, saved.size.y, false);
  camera.position.copy(saved.position); camera.quaternion.copy(saved.quaternion); camera.fov = saved.fov; camera.aspect = saved.aspect; camera.updateProjectionMatrix();
  simulation.view.setLook('studio'); simulation.controls.update();
  return { dataUrl: output.toDataURL('image/jpeg', 0.84), calibrationInfo };
}

let failures = 0;
for (const view of views) {
  try {
    const result = await evaluate(`(${capture})(${JSON.stringify(view)}, ${JSON.stringify(look)})`);
    const filename = view.photo.replace(/\.(jpe?g|png)$/i, '.jpg');
    fs.writeFileSync(path.join(outputDirectory, filename), Buffer.from(result.dataUrl.split(',')[1], 'base64'));
    console.log(`saved ${path.relative(process.cwd(), path.join(outputDirectory, filename))}${result.calibrationInfo ? ` ${JSON.stringify(result.calibrationInfo)}` : ''}`);
  } catch (error) {
    failures += 1; console.error(`${view.photo}: ${error.message}`);
  }
}
socket.close();
process.exitCode = failures ? 1 : 0;
