#!/usr/bin/env node
// Arm server: the three.js simulator page plus ONE robot API for everybody. No dependencies.
//
// /api is the robot's low-level API (lutich/robot-arm docs/api.md: GET /api/state, POST /api/move,
// /api/jog, /api/go_pose, /api/stop, /api/power_on, /api/demo/*, ...). Behind it sits a switchable backend:
//   sim   a shared simulated arm in this process (controller emulation, cubes, physics, collision)
//   real  the controller on the Pi (--controller); every call that can move the arm is first played
//         in the simulation and blocked (HTTP 409) if it collides; /api/stop always goes through
// GET /api/backend tells which is active, POST /api/backend {"target": "sim" | "real"} switches it.
// /sim/world and /sim/scene show and set the shared simulation's cubes (viewers, scenario resets).
//
//   node server.mjs                                   page + simulated arm, http://127.0.0.1:8770/
//   node server.mjs --controller http://10.102.4.103:8765
//                                                     + the real arm as a second backend, read only:
//                                                       GETs and /api/stop, motion blocked
//   node server.mjs --controller http://10.102.4.103:8765 --allow-motion
//                                                     + motion to the real arm (after the sim check)
//   node server.mjs --camera http://10.102.4.103:8765 the REAL cameras: /api/camera/* (OAK) and
//                                                     /api/camera2/* (webcam), GET only
//   --port 8770  --host 127.0.0.1                     (--mock is accepted and changes nothing)
//
// The real controller only accepts POSTs whose Origin is its own address; this server sends it.
// POSTs to this server need Origin: http://<this host> and Content-Type: application/json, like the
// real controller, so code written against one works against the other.

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { ControllerError } from './js/controller.js';
import { ArmBackend, Blocked } from './js/armbackend.js';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.dirname(ROOT);

const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const flag = name => args.includes(name);
const PORT = Number(opt('--port', 8770));
const HOST = opt('--host', '127.0.0.1');
const CONTROLLER = opt('--controller', null);
const ALLOW_MOTION = flag('--allow-motion');
// Where camera reads go: the controller if given, else --camera, else the mock's saved frame.
const CAMERA = CONTROLLER || opt('--camera', null);

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

// ---------------- proxy to the real controller ----------------

async function proxy(req, res, base = CONTROLLER, cameraOnly = false) {
  const target = new URL(req.url, base);
  const isPost = req.method === 'POST';
  if (cameraOnly && (req.method !== 'GET' || !/^\/api\/camera2?\//.test(target.pathname)))
    return sendJson(res, 403, { error: 'The camera link only reads /api/camera/* and /api/camera2/* (GET).' });
  if (isPost && !ALLOW_MOTION && target.pathname !== '/api/stop')
    return sendJson(res, 403, { error: 'Motion is disabled in this proxy. Restart server.mjs with --allow-motion to send moves.' });
  if (!['GET', 'POST'].includes(req.method)) return sendJson(res, 405, { error: 'GET or POST only' });
  const body = isPost ? await readBody(req) : undefined;
  const upstream = http.request(target, {
    method: req.method,
    headers: {
      Host: target.host, ...(isPost ? { Origin: `${target.protocol}//${target.host}`, 'Content-Type': 'application/json', 'Content-Length': body.length } : {}),
    },
    timeout: 20000,
  }, up => {
    res.writeHead(up.statusCode, { 'Content-Type': up.headers['content-type'] || 'application/octet-stream', 'Cache-Control': 'no-store' });
    up.pipe(res);
  });
  upstream.on('timeout', () => upstream.destroy(new Error('controller timed out')));
  upstream.on('error', e => { if (!res.headersSent) sendJson(res, 502, { error: `controller: ${e.message}` }); else res.end(); });
  // When the browser stops reading (closed tab, stopped video), close the request to the Pi too,
  // so a camera stream never keeps running on its own.
  res.on('close', () => upstream.destroy());
  if (target.pathname.endsWith('stream.mjpg')) console.log(`camera stream opened (read only)`);
  if (isPost) { console.log(`→ ${target.pathname} ${body}`); upstream.end(body); } else upstream.end();
}

// ---------------- arm backend: shared simulation + optional real controller ----------------

// The real controller, reached like the proxy above (its own Host and Origin). GET state, POST actions.
function realRequest(method, p, body) {
  return new Promise((resolve, reject) => {
    const target = new URL(`/api/${p}`, CONTROLLER);
    const data = method === 'POST' ? Buffer.from(JSON.stringify(body || {})) : null;
    const rq = http.request(target, {
      method, timeout: 20000,
      headers: { Host: target.host, ...(data ? { Origin: `${target.protocol}//${target.host}`, 'Content-Type': 'application/json', 'Content-Length': data.length } : {}) },
    }, up => {
      const chunks = [];
      up.on('data', c => chunks.push(c));
      up.on('end', () => {
        let json = {}; try { json = JSON.parse(Buffer.concat(chunks).toString() || '{}'); } catch { /* not JSON */ }
        if (up.statusCode >= 400) { const e = new Error(json.error || json.detail || `controller: HTTP ${up.statusCode}`); e.status = up.statusCode; reject(e); }
        else resolve(json);
      });
    });
    rq.on('timeout', () => rq.destroy(new Error('controller timed out')));
    rq.on('error', reject);
    rq.end(data || undefined);
  });
}

const backend = new ArmBackend({
  real: CONTROLLER ? { get: p => realRequest('GET', p), post: (p, b) => realRequest('POST', p, b) } : null,
  realName: CONTROLLER, allowMotion: ALLOW_MOTION, log: t => console.log(t),
});
{
  let last = performance.now();
  setInterval(() => { const now = performance.now(); backend.tick(Math.min(0.1, (now - last) / 1000)); last = now; }, 20);
}

async function readJson(req) {
  try { return JSON.parse((await readBody(req)).toString() || '{}'); } catch { return null; }
}

function sendError(res, e) {
  if (e instanceof Blocked) return sendJson(res, 409, { error: e.message, blocked: true });
  if (e instanceof ControllerError) return sendJson(res, 400, { error: e.message });
  if (e.status) return sendJson(res, e.status, { error: e.message });
  console.error(e);
  return sendJson(res, 502, { error: e.message });
}

// /api: the robot's low-level API, answered by the active backend; /api/backend switches it.
async function armApi(req, res) {
  const p = new URL(req.url, 'http://x').pathname.replace(/^\/api\//, '');
  if (/^camera2?\//.test(p)) {                     // the OAK (camera) and the webcam (camera2), read only
    if (CAMERA) return await proxy(req, res, CAMERA, true);
    return mockCamera(req, res, p);
  }
  try {
    if (p === 'backend') {
      if (req.method === 'GET') return sendJson(res, 200, backend.info());
      const body = await readJson(req);
      if (!body) return sendJson(res, 400, { error: 'Invalid JSON' });
      return sendJson(res, 200, await backend.setTarget(body.target));
    }
    if (req.method === 'GET') return sendJson(res, 200, await backend.get(p));
    if (req.method !== 'POST') return sendJson(res, 405, { error: 'GET or POST only' });
    // Same rules as the real controller: same-origin caller, JSON body.
    if (req.headers.origin !== `http://${req.headers.host}`) return sendJson(res, 403, { error: 'Same-origin page required (send Origin: http://<this host>)' });
    if (!(req.headers['content-type'] || '').startsWith('application/json')) return sendJson(res, 400, { error: 'Content-Type must be application/json' });
    const body = await readJson(req);
    if (!body) return sendJson(res, 400, { error: 'Invalid JSON' });
    return sendJson(res, 200, await backend.post(p, body));
  } catch (e) { return sendError(res, e); }
}

// /sim: the shared simulation's scene, for viewers and scenario resets (not part of the robot API).
async function simApi(req, res) {
  const p = new URL(req.url, 'http://x').pathname;
  if (p === '/sim/world' && req.method === 'GET') return sendJson(res, 200, backend.worldState());
  if (p === '/sim/scene' && req.method === 'POST') {
    const body = await readJson(req);
    if (!body || !Array.isArray(body.cubes)) return sendJson(res, 400, { error: 'body: { "cubes": [[colour, x, y, level, yaw], ...] }' });
    backend.setScene(body.cubes);
    return sendJson(res, 200, backend.worldState());
  }
  return sendJson(res, 404, { error: 'Unknown path' });
}

// Mock camera: the saved frame as status, snapshot and a 2 fps MJPEG stream (multipart, like the OAK route).
function mockCamera(req, res, p) {
  const f = path.join(REPO, 'docs', 'images', 'photos', 'controller-camera-2026-10-12.jpg');
  if (req.method !== 'GET') return sendJson(res, 405, { error: 'GET only' });
  if (p.startsWith('camera2/')) return sendJson(res, 404, { error: 'No mock frame for camera2 (the webcam): start with --camera http://10.102.4.103:8765, or load an image file' });
  if (p === 'camera/status') {
    return sendJson(res, 200, { enabled: true, source: 'mock', available: true, frame_age_ms: 0,
      latest_frame: { delivered_fps: 2, time_us: 30000, iso: 200, lens_position: 65, captured_at: new Date().toISOString() } });
  }
  if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no frame' });
  if (p === 'camera/snapshot.jpg') {
    res.writeHead(200, { 'Content-Type': 'image/jpeg', 'Cache-Control': 'no-store' });
    return fs.createReadStream(f).on('error', () => res.destroy()).pipe(res);
  }
  if (p === 'camera/stream.mjpg') {
    const jpg = fs.readFileSync(f);
    res.writeHead(200, { 'Content-Type': 'multipart/x-mixed-replace; boundary=frame', 'Cache-Control': 'no-store' });
    const send = () => res.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpg.length}\r\n\r\n`) && res.write(jpg) && res.write('\r\n');
    send();
    const timer = setInterval(send, 500);
    res.on('close', () => clearInterval(timer));
    return;
  }
  return sendJson(res, 404, { error: 'Unknown path' });
}

// ---------------- server ----------------

const server = http.createServer(async (req, res) => {
  try {
    if (req.url.startsWith('/api/')) {
      return await armApi(req, res);
    }
    if (req.url.startsWith('/sim/')) return await simApi(req, res);
    if (req.url === '/local/info') return sendJson(res, 200, { backend: 'arm-server', ...backend.info(), camera: CAMERA || 'mock' });
    // Repo photos, read only: the model comparison (tools/compare.mjs) overlays them on renders.
    const ph = req.url.match(/^\/photos\/([\w.-]+\.(?:jpe?g|png))$/i);
    if (ph) {
      const f = path.join(REPO, 'docs', 'images', 'photos', ph[1]);
      if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no such photo' });
      res.writeHead(200, { 'Content-Type': /png$/i.test(f) ? 'image/png' : 'image/jpeg', 'Cache-Control': 'no-cache' });
      return fs.createReadStream(f).on('error', () => res.destroy()).pipe(res);
    }
    // Calibration captures (tools/capture.mjs), read only: the list with each info.json, and their files
    if (req.url.split('?')[0] === '/calibration/index.json') {
      const root = path.join(REPO, 'docs', 'images', 'calibration');
      const list = fs.existsSync(root) ? fs.readdirSync(root, { withFileTypes: true }).filter(d => d.isDirectory()).map(d => {
        let info = null; try { info = JSON.parse(fs.readFileSync(path.join(root, d.name, 'info.json'), 'utf8')); } catch { /* no info */ }
        return { name: d.name, files: fs.readdirSync(path.join(root, d.name)), info };
      }) : [];
      return sendJson(res, 200, list.sort((a, b) => a.name.localeCompare(b.name)));
    }
    const cf = req.url.split('?')[0].match(/^\/calibration\/([\w.-]+)\/([\w.-]+\.(?:jpe?g|png|json))$/i);
    if (cf) {
      const f = path.join(REPO, 'docs', 'images', 'calibration', cf[1], cf[2]);
      if (!fs.existsSync(f)) return sendJson(res, 404, { error: 'no such file' });
      res.writeHead(200, { 'Content-Type': /json$/i.test(f) ? 'application/json' : /png$/i.test(f) ? 'image/png' : 'image/jpeg', 'Cache-Control': 'no-cache' });
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
  console.log('  /api -> shared simulated arm (switch with POST /api/backend {"target": "sim" | "real"})');
  if (CONTROLLER) console.log(`  real backend: ${CONTROLLER} (${ALLOW_MOTION ? 'MOTION ALLOWED after the sim check' : 'read only, stop allowed'})`);
  if (CAMERA) console.log(`  /api/camera/* -> ${CAMERA} (GET only)`);
});
