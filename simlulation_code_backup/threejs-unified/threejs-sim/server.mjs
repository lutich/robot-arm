#!/usr/bin/env node
// Static server for the simulation-only three.js application. No dependencies.
//
//   node server.mjs                                   simulator, http://127.0.0.1:8770/
//   node server.mjs --mock                            + local mock API for development/tests
//   --port 8770  --host 127.0.0.1
//
// There is deliberately no controller or camera proxy. This process never connects to the
// Raspberry Pi and cannot forward commands to physical hardware.

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { SimController, ControllerError } from './js/controller.js';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.dirname(ROOT);

const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const flag = name => args.includes(name);
const PORT = Number(opt('--port', 8770));
const HOST = opt('--host', '127.0.0.1');
const MOCK = flag('--mock');
const FORBIDDEN = ['--controller', '--camera', '--allow-motion'].find(name => flag(name));
if (FORBIDDEN) {
  console.error(`${FORBIDDEN} is not supported: this build is simulation-only and never connects to robot hardware.`);
  process.exit(2);
}

const TYPES = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json', '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg',
  '.svg': 'image/svg+xml', '.txt': 'text/plain; charset=utf-8' };

function sendJson(res, code, obj) {
  res.writeHead(code, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
  res.end(JSON.stringify(obj));
}

function serveStatic(req, res) {
  const url = new URL(req.url, 'http://x');
  let rel = decodeURIComponent(url.pathname);
  if (rel === '/') rel = '/index.html';
  const file = path.normalize(path.join(ROOT, rel));
  if (!file.startsWith(ROOT + path.sep) || file.includes(`${path.sep}node_modules${path.sep}`)) return sendJson(res, 404, { error: 'Unknown path' });
  fs.stat(file, (err, st) => {
    if (err || !st.isFile()) return sendJson(res, 404, { error: 'Unknown path' });
    res.writeHead(200, { 'Content-Type': TYPES[path.extname(file).toLowerCase()] || 'application/octet-stream', 'Content-Length': st.size, 'Cache-Control': 'no-cache' });
    fs.createReadStream(file).on('error', () => res.destroy()).pipe(res);
  });
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    const chunks = []; let n = 0;
    req.on('data', c => { n += c.length; if (n > 8192) { reject(new Error('Body over 8192 bytes')); req.destroy(); } else chunks.push(c); });
    req.on('end', () => resolve(Buffer.concat(chunks)));
    req.on('error', reject);
  });
}

// ---------------- mock controller ----------------

const mock = MOCK ? new SimController() : null;
if (mock) {
  let last = performance.now();
  setInterval(() => { const now = performance.now(); mock.update((now - last) / 1000); last = now; }, 20);
}

const MOCK_ACTIONS = {
  move: b => mock.move(b.channel, b.target),
  jog: b => mock.jog(b.channel, b.direction),
  go_pose: b => mock.go_pose(b.name),
  stop: b => mock.stop(b.reason || 'stop'),
  power_on: () => mock.power_on(),
  power_off: () => mock.power_off(),
  settings: b => mock.settings_(b.step_size, b.demo_speed),
  limits: b => mock.limits(b.channel, b.low, b.high),
  'demo/add': b => mock.demo_add(b.name),
  'demo/replace': b => mock.demo_replace(b.index),
  'demo/remove': b => mock.demo_remove(b.index),
  'demo/rename': b => mock.demo_rename(b.index, b.name),
  'demo/reorder': b => mock.demo_reorder(b.index, b.direction),
  'demo/next': () => mock.demo_next(),
  'demo/run': () => mock.demo_run(),
  'demo/pause': () => mock.demo_pause(),
  'demo/resume': () => mock.demo_resume(),
  'demo/restart': () => mock.demo_restart(),
};

async function mockApi(req, res) {
  const p = new URL(req.url, 'http://x').pathname.replace(/^\/api\//, '');
  if (req.method === 'GET' && p === 'state') return sendJson(res, 200, { ...mock.state(), mode: 'mock' });
  if (req.method === 'GET' && p === 'camera/snapshot.jpg') {
    const f = path.join(REPO, 'docs', 'images', 'photos', 'controller-camera-2026-10-12.jpg');
    if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no snapshot' });
    res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-store' });
    return fs.createReadStream(f).on('error', () => res.destroy()).pipe(res);
  }
  // Mock camera: the saved frame as status, snapshot and a 2 fps MJPEG stream (multipart, like the OAK route).
  if (req.method === 'GET' && p === 'camera/status') {
    return sendJson(res, 200, { enabled: true, source: 'mock', available: true, frame_age_ms: 0,
      latest_frame: { delivered_fps: 2, time_us: 30000, iso: 200, lens_position: 65, captured_at: new Date().toISOString() } });
  }
  if (req.method === 'GET' && p === 'camera/stream.mjpg') {
    const f = path.join(REPO, 'docs', 'images', 'photos', 'controller-camera-2026-10-12.jpg');
    if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no frame' });
    const jpg = fs.readFileSync(f);
    res.writeHead(200, { 'Content-Type': 'multipart/x-mixed-replace; boundary=frame', 'Cache-Control': 'no-store' });
    const send = () => res.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpg.length}\r\n\r\n`) && res.write(jpg) && res.write('\r\n');
    send();
    const timer = setInterval(send, 500);
    res.on('close', () => clearInterval(timer));
    return;
  }
  if (req.method !== 'POST' || !MOCK_ACTIONS[p]) return sendJson(res, 404, { error: 'Unknown path' });
  // Same rules as the real controller: same-origin page, JSON body.
  if (req.headers.origin !== `http://${req.headers.host}`) return sendJson(res, 403, { error: 'Same-origin page required' });
  if (!(req.headers['content-type'] || '').startsWith('application/json')) return sendJson(res, 400, { error: 'Content-Type must be application/json' });
  let body;
  try { body = JSON.parse((await readBody(req)).toString() || '{}'); } catch { return sendJson(res, 400, { error: 'Invalid JSON' }); }
  try { MOCK_ACTIONS[p](body); return sendJson(res, 200, { ok: true, result: true }); }
  catch (e) { if (e instanceof ControllerError) return sendJson(res, 400, { error: e.message }); throw e; }
}

// ---------------- server ----------------

const server = http.createServer(async (req, res) => {
  try {
    if (req.url.startsWith('/api/')) {
      if (MOCK) return await mockApi(req, res);
      return sendJson(res, 404, { error: 'No API backend. The application is simulation-only; use --mock for the local development API.' });
    }
    // Repo photos, read only: the model comparison (tools/compare.mjs) overlays them on renders.
    const ph = req.url.match(/^\/photos\/([\w.-]+\.(?:jpe?g|png))$/i);
    if (ph) {
      const f = path.join(REPO, 'docs', 'images', 'photos', ph[1]);
      if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no such photo' });
      res.writeHead(200, { 'Content-Type': /png$/i.test(f) ? 'image/png' : 'image/jpeg', 'Cache-Control': 'no-cache' });
      return fs.createReadStream(f).on('error', () => res.destroy()).pipe(res);
    }
    if (req.url.startsWith('/ref/camera-2026-10-12.jpg')) {
      const f = path.join(REPO, 'docs', 'images', 'photos', 'controller-camera-2026-10-12.jpg');
      res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-cache' });
      return fs.createReadStream(f).on('error', () => res.destroy()).pipe(res);
    }
    if (req.method !== 'GET' && req.method !== 'HEAD') return sendJson(res, 405, { error: 'GET only' });
    serveStatic(req, res);
  } catch (e) {
    console.error(e);
    if (!res.headersSent) sendJson(res, 500, { error: e.message });
  }
});

server.listen(PORT, HOST, () => {
  console.log(`three.js arm simulator: http://${HOST}:${PORT}/`);
  if (MOCK) console.log('  /api -> local mock controller (simulation only)');
});
