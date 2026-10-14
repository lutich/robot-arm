// Procedural textures (canvas): noise, the printed mat, wood, the servo label. No image files.

import * as THREE from 'three';
import { DICT as ARUCO } from '../aruco.js';
import { MARKER_CORNERS } from '../calib.js';

// Marker squares on the mat (camera table frame, mm): id -> [Xmin, Ymin], from calib.js.
const MARKERS = Object.fromEntries(Object.entries(MARKER_CORNERS).map(([id, c]) => [id, [Math.min(...c.map(p => p[0])), Math.min(...c.map(p => p[1]))]]));

export function canvasTexture(w, h, draw, { srgb = true, repeat = null } = {}) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace;
  t.anisotropy = 8;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  if (repeat) t.repeat.set(...repeat);
  return t;
}

export let seed = 1234567;

export const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);

/** Grey noise; `streak` adds row-wise variation (brushed metal), `blur` softens it. */
export function noise(g, w, h, base, amp, { streak = 0, fibres = 0 } = {}) {
  const img = g.createImageData(w, h);
  for (let y = 0; y < h; y++) {
    const row = streak ? (rnd() - 0.5) * streak : 0;
    for (let x = 0; x < w; x++) {
      const v = Math.max(0, Math.min(255, base + (rnd() - 0.5) * amp + row));
      const i = 4 * (y * w + x);
      img.data[i] = img.data[i + 1] = img.data[i + 2] = v; img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  for (let i = 0; i < fibres; i++) {
    g.strokeStyle = `rgba(${rnd() < 0.5 ? '0,0,0' : '255,255,255'},${0.05 + rnd() * 0.08})`;
    g.lineWidth = 1;
    const x = rnd() * w, y = rnd() * h, a = rnd() * Math.PI, l = 3 + rnd() * 10;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + Math.cos(a) * l, y + Math.sin(a) * l); g.stroke();
  }
}

/** The printed reference mat: field 280 x 360 mm on 290 x 375 mm paper, 4 px per mm. */
export function matTexture() {
  const PX = 4, PW = 290, PH = 375, ox = 5, oy = 5;
  const t = canvasTexture(PW * PX, PH * PX, (g, w, h) => {
    g.fillStyle = '#f6f6f2'; g.fillRect(0, 0, w, h);
    // Canvas top = far edge (+X), canvas right = -Y (the arm's right), as seen from the arm.
    const cx = Y => (ox + 140 - Y) * PX;
    const cy = X => h - (oy + X) * PX;
    for (let X = 0; X <= 360; X += 5) {
      g.strokeStyle = X % 50 === 0 ? '#878d94' : X % 10 === 0 ? '#c4c9ce' : '#e2e5e8';
      g.lineWidth = X % 50 === 0 ? 2.4 : 1;
      g.beginPath(); g.moveTo(cx(140), cy(X)); g.lineTo(cx(-140), cy(X)); g.stroke();
    }
    for (let Y = -140; Y <= 140; Y += 5) {
      g.strokeStyle = Y % 50 === 0 ? '#878d94' : Y % 10 === 0 ? '#c4c9ce' : '#e2e5e8';
      g.lineWidth = Y % 50 === 0 ? 2.4 : 1;
      g.beginPath(); g.moveTo(cx(Y), cy(0)); g.lineTo(cx(Y), cy(360)); g.stroke();
    }
    g.strokeStyle = '#4d535a'; g.lineWidth = 3; g.strokeRect(cx(140), cy(360), 280 * PX, 360 * PX);
    // the mat is two A4 sheets, joined at X = 180 with a slight overlap shadow
    g.fillStyle = 'rgba(0,0,0,0.07)'; g.fillRect(0, cy(180) - 2, w, 5);
    g.fillStyle = 'rgba(255,255,255,0.8)'; g.fillRect(0, cy(180) - 4, w, 2);
    g.fillStyle = '#25292d'; g.textAlign = 'center'; g.textBaseline = 'middle';
    const label = (text, X, Y, size = 7, bold = true) => { g.font = `${bold ? 'bold ' : ''}${size * PX}px Helvetica, Arial, sans-serif`; g.fillText(text, cx(Y), cy(X)); };
    for (let X = 50; X <= 350; X += 50) { label(String(X), X, 75); label(String(X), X, -75); }
    for (const Y of [-100, -50, 50, 100]) label((Y > 0 ? '+' : '') + Y, 280, Y);
    label('0', 280, 0); label('Y (mm)', 270, 0, 4.5); label('X (mm)', 340, 75, 4.5); label('X (mm)', 155, 75, 4.5);
    label('X 360 - FAR EDGE', 352, 0, 5); label('+Z UP', 330, 0, 4);
    label('TABLE REFERENCE  |  2/2  |  FAR ARM', 365, 60, 2.6, false);
    label('A4 landscape  |  Actual size / 100%  |  Single-sided', 365, -70, 2.2, false);
    for (const [id, [X0, Y0]] of Object.entries(MARKERS)) {
      const bits = ARUCO[id], cell = 30 / 6;
      g.fillStyle = '#ffffff'; g.fillRect(cx(Y0 + 33), cy(X0 + 33), 36 * PX, 36 * PX);
      for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) {
        const white = r > 0 && r < 5 && c > 0 && c < 5 && bits[r - 1][c - 1] === '1';
        g.fillStyle = white ? '#ffffff' : '#151515';
        const X = X0 + 30 - (r + 1) * cell, Y = Y0 + 30 - c * cell;
        g.fillRect(cx(Y), cy(X + cell), cell * PX + 0.6, cell * PX + 0.6);
      }
      label(`ID ${id}  X ${X0 + 15}  Y ${Y0 + 15}`, X0 - 3.5, Y0 + 15, 2.0, false);
    }
  });
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

export function woodTexture(hex) {
  const col = new THREE.Color(hex);
  return canvasTexture(256, 256, (g, w, h) => {
    g.fillStyle = `#${col.getHexString()}`; g.fillRect(0, 0, w, h);
    for (let i = 0; i < 70; i++) {
      const y = rnd() * h;
      g.strokeStyle = `rgba(0,0,0,${0.025 + rnd() * 0.05})`;
      g.lineWidth = 1 + rnd() * 3;
      g.beginPath(); g.moveTo(0, y);
      for (let x = 0; x <= w; x += 16) g.lineTo(x, y + Math.sin(x / 40 + i) * 4);
      g.stroke();
    }
    const d = g.getImageData(0, 0, w, h);
    for (let i = 0; i < d.data.length; i += 4) { const n = (rnd() - 0.5) * 16; d.data[i] += n; d.data[i + 1] += n; d.data[i + 2] += n; }
    g.putImageData(d, 0, 0);
  });
}

export function servoLabelTexture() {
  return canvasTexture(512, 256, (g, w, h) => {
    g.fillStyle = '#121214'; g.fillRect(0, 0, w, h);
    g.fillStyle = '#f2cf1d'; g.fillRect(70, 70, 372, 120);
    g.fillStyle = '#121214'; g.fillRect(78, 78, 120, 44);
    g.fillStyle = '#f2cf1d'; g.font = 'bold 34px Arial'; g.textAlign = 'center'; g.fillText('DM', 138, 112);
    g.fillStyle = '#121214'; g.font = 'bold 50px Arial'; g.fillText('MG996R', 310, 125);
    g.font = 'bold 22px Arial'; g.fillText('DIGI HI-SPEED', 256, 172);
    g.fillStyle = '#f2cf1d'; g.font = '16px Arial'; g.fillText('TOWER-PRO  ·  13KG  ·  4.8-7.2V', 256, 220);
  });
}
