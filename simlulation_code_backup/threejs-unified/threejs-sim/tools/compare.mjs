// Renders the model from the viewpoint of each reference photo and saves render | photo | 50 % overlay
// side by side, so the model can be checked against the real arm one photo at a time.
//
//   node tools/compare.mjs                     all views in tools/photo-views.json
//   node tools/compare.mjs --only gripper      views whose photo name contains "gripper"
//   node tools/compare.mjs --out <dir>         default: ../docs/images/compare
//   node tools/compare.mjs --look studio       lighting; default "office" (closer to the photos)
//
// Starts its own simulator server (no controller, nothing is sent anywhere) and a headless Chrome
// (set CHROME to its path if it is not in the default place). How to match a view: docs/model-photo-comparison.md.

import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const only = opt('--only', '');
const outDir = path.resolve(opt('--out', path.join(ROOT, '..', 'docs', 'images', 'compare')));
const look = opt('--look', 'office');
const viewsFile = path.resolve(opt('--views', path.join(ROOT, 'tools', 'photo-views.json')));
const PORT = Number(opt('--port', 8779)), CDP = PORT + 1;
const CHROME = process.env.CHROME || 'C:/Program Files/Google/Chrome/Application/chrome.exe';

const views = JSON.parse(fs.readFileSync(viewsFile, 'utf8')).views.filter(v => v.photo.includes(only));
if (!views.length) { console.error('no matching views'); process.exit(1); }
fs.mkdirSync(outDir, { recursive: true });

const sleep = ms => new Promise(r => setTimeout(r, ms));
const server = spawn(process.execPath, [path.join(ROOT, 'server.mjs'), '--port', String(PORT)], { stdio: 'ignore' });
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arm-compare-'));
const chrome = spawn(CHROME, ['--headless=new', `--remote-debugging-port=${CDP}`, `--user-data-dir=${profile}`, '--window-size=1400,900',
  '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check', 'about:blank'], { stdio: 'ignore' });
const quit = code => { try { chrome.kill(); } catch {} try { server.kill(); } catch {} process.exit(code); };

let targets = [];
for (let i = 0; i < 60 && !targets.some(t => t.type === 'page'); i++) {
  try { targets = await (await fetch(`http://127.0.0.1:${CDP}/json`)).json(); } catch {}
  await sleep(250);
}
const page = targets.find(t => t.type === 'page');
if (!page) { console.error('Chrome did not start'); quit(1); }
const ws = new WebSocket(page.webSocketDebuggerUrl);
await new Promise(r => { ws.onopen = r; });
let id = 0; const pending = new Map();
ws.onmessage = ev => {
  const m = JSON.parse(ev.data);
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
  if (m.method === 'Runtime.exceptionThrown') console.log('[page exception]', m.params.exceptionDetails.exception?.description);
};
const send = (method, params = {}) => new Promise(r => { const i = ++id; pending.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const evaluate = async expr => {
  const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true });
  if (r.result?.exceptionDetails) throw new Error(r.result.exceptionDetails.exception?.description || r.result.exceptionDetails.text);
  return r.result?.result?.value;
};
await send('Runtime.enable');
await send('Page.navigate', { url: `http://127.0.0.1:${PORT}/` });
let ready = false;
for (let i = 0; i < 80 && !ready; i++) { ready = await evaluate('!!window.sim').catch(() => false); if (!ready) await sleep(250); }
if (!ready) { console.error('the simulator page did not load'); quit(1); }

// Runs in the page: pose the arm, render at the photo's aspect ratio from the view's camera,
// compose the three panels and return them as a JPEG data URL.
async function shoot(v, look) {
  const s = window.sim, $ = id => document.getElementById(id);
  for (const id of ['show-path', 'show-shapes']) { $(id).checked = false; $(id).dispatchEvent(new Event('change')); }
  s.ctl.rearm(); s.ctl.setCommands(v.counts || s.ctl.poses[v.pose || 'HOME'].slice());
  s.world.cubes = []; s.world.holding = null;
  for (const [colour, x, y, level = 0, yaw = 0] of v.cubes || []) s.world.addCube(colour, colour, x, y, level, yaw);
  s.world.dirty = true;
  await new Promise(r => setTimeout(r, 700));            // the page's loop applies the pose and the cubes
  const img = new Image(); img.src = `/photos/${v.photo}`; await img.decode();
  const H = 900, W = Math.round(H * img.naturalWidth / img.naturalHeight);
  const cam = s.camera, r = s.renderer;
  const saved = { pos: cam.position.clone(), q: cam.quaternion.clone(), fov: cam.fov, aspect: cam.aspect, size: r.getSize(new s.THREE.Vector2()), ratio: r.getPixelRatio() };
  s.view.setLook(look);
  r.setPixelRatio(1); r.setSize(W, H, false);
  let info = null;
  if (v.camera.markers) {
    // Camera pose and focal length from the mat's ArUco markers in the photo (same code as the Camera tab).
    const [{ detectMarkers }, { calibrate, MARKER_CORNERS }, { tableToBoard }] = await Promise.all([import('/js/aruco.js'), import('/js/calib.js'), import('/js/camview.js')]);
    const pw = img.naturalWidth, ph = img.naturalHeight, sc = Math.min(1, 1280 / Math.max(pw, ph));
    const c = document.createElement('canvas'); c.width = Math.round(pw * sc); c.height = Math.round(ph * sc);
    const cg = c.getContext('2d', { willReadFrequently: true }); cg.drawImage(img, 0, 0, c.width, c.height);
    const { markers } = detectMarkers(cg.getImageData(0, 0, c.width, c.height), { maxArea: 400000 });
    const ids = Object.keys(markers).filter(id => MARKER_CORNERS[id] && !(v.camera.skip || []).includes(Number(id)));
    if (ids.length < 2) throw new Error(`only ${ids.length} mat markers found`);
    const world = [], uv = [];
    for (const id of ids) MARKER_CORNERS[id].forEach((p, k) => { world.push([p[0], p[1], 0]); uv.push([markers[id][k][0] / sc, markers[id][k][1] / sc]); });
    const cal = calibrate(world, uv, { width: pw, height: ph, fixPrincipal: true });
    const dir = row => new s.THREE.Vector3(-row[1], row[0], row[2]);
    cam.position.set(...tableToBoard(cal.centre));
    cam.quaternion.setFromRotationMatrix(new s.THREE.Matrix4().makeBasis(dir(cal.R[0]), dir(cal.R[1]).negate(), dir(cal.R[2]).negate()));
    cam.fov = 2 * Math.atan(ph / 2 / cal.f) * 180 / Math.PI; cam.aspect = W / H; cam.updateProjectionMatrix();
    const ahead = new s.THREE.Vector3(0, 0, -30).applyQuaternion(cam.quaternion).add(cam.position);
    info = { markers: ids.join(' '), rms: +cal.rms.toFixed(2), fov: +cam.fov.toFixed(1), pos: cam.position.toArray().map(x => +x.toFixed(1)), target: ahead.toArray().map(x => +x.toFixed(1)) };
  } else {
    cam.aspect = W / H; cam.fov = v.camera.fov; cam.position.set(...v.camera.pos); cam.up.set(0, 0, 1);
    cam.lookAt(...v.camera.target); cam.rotateZ((v.camera.roll || 0) * Math.PI / 180); cam.updateProjectionMatrix();
  }
  s.view.render();
  const out = document.createElement('canvas'); out.width = 3 * W; out.height = H;
  const g = out.getContext('2d');
  g.drawImage(r.domElement, 0, 0, W, H);
  g.drawImage(img, W, 0, W, H);
  g.drawImage(img, 2 * W, 0, W, H); g.globalAlpha = 0.5; g.drawImage(r.domElement, 2 * W, 0, W, H); g.globalAlpha = 1;
  g.font = 'bold 22px sans-serif'; g.fillStyle = '#fff'; g.strokeStyle = '#000'; g.lineWidth = 4;
  [['model', 0], [v.photo, W], ['overlay 50 %', 2 * W]].forEach(([t, x]) => { g.strokeText(t, x + 14, 34); g.fillText(t, x + 14, 34); });
  r.setPixelRatio(saved.ratio); r.setSize(saved.size.x, saved.size.y, false);
  cam.position.copy(saved.pos); cam.quaternion.copy(saved.q); cam.fov = saved.fov; cam.aspect = saved.aspect; cam.updateProjectionMatrix();
  s.view.setLook('studio'); s.controls.update();
  return { url: out.toDataURL('image/jpeg', 0.82), info };
}

let failed = 0;
for (const v of views) {
  try {
    const { url, info } = await evaluate(`(${shoot})(${JSON.stringify(v)}, ${JSON.stringify(look)})`);
    const file = path.join(outDir, v.photo.replace(/\.(jpe?g|png)$/i, '.jpg'));
    fs.writeFileSync(file, Buffer.from(url.split(',')[1], 'base64'));
    console.log('saved', path.relative(process.cwd(), file), info ? JSON.stringify(info) : '');
  } catch (e) { failed++; console.error(v.photo, e.message); }
}
ws.close();
quit(failed ? 1 : 0);
