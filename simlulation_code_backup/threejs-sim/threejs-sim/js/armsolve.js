// The arm's pose from its tags in all cameras, without the page: the Camera tab runs it directly ("Track the
// arm"), live tracking runs it in js/trackworker.js so the page never waits for it.
//   findTags(image, cal, angles, opts)   the arm tags in one camera frame, searched around where `angles` puts them
//   solveArm(views, opts)                triangulated tags, each rigid part alone, wrist + gripper, the joint angles

import { ARM_TAGS, ARM_PARTS, GRIPPER } from './config.js';
import { fk, point, thetaForGap, padDepthForTheta } from './kinematics.js';
import { detectArmTags, triangulate, fitArmPose, tagPoints, projectBoard, tableToBoardPt } from './armtrack.js';
import { fitPartPose } from './rigidtrack.js';
import { fitJointPair } from './jointfit.js';

/**
 * Tags in one frame (ImageData-like), in the calibration's pixel frame (w x h). The search window is the tags'
 * place for `angles`, padded: wide with a full-frame search as well, or (quick) tight and only that window.
 */
export function findTags(img, cal, angles, { quick = false, w = img.width, h = img.height } = {}) {
  const sx = w / img.width, sy = h / img.height;
  const uvs = ARM_TAGS.map(t => projectBoard(cal, tagPoints(t, angles).centre)).map(([u, v]) => [u / sx, v / sy]);
  const pad = (quick ? 0.06 : 0.18) * img.width;
  const roi = [Math.min(...uvs.map(p => p[0])) - pad, Math.min(...uvs.map(p => p[1])) - pad, Math.max(...uvs.map(p => p[0])) + pad, Math.max(...uvs.map(p => p[1])) + pad];
  const found = detectArmTags(img, undefined, { roi, full: !quick });
  return Object.fromEntries(Object.entries(found).map(([id, t]) => [id, { corners: t.corners.map(([x, y]) => [x * sx, y * sy]), centre: [t.centre[0] * sx, t.centre[1] * sy] }]));
}

/**
 * views: [{ cal, tags }] (cameras that saw a tag). commanded: the joint panel's angles; last: the last tracked
 * pose (or null). Returns { tri, parts, pair, result, notes }; result is null with too few tag views.
 */
export function solveArm(views, { commanded, last = null, useCorners = true } = {}) {
  const notes = [];
  // a tag seen by two or more cameras: its 3D position, independent of the arm model
  const tri = {};
  for (const t of ARM_TAGS.filter(x => x.id != null)) {
    const v = views.filter(x => x.tags[t.id]).map(x => ({ cal: x.cal, uv: x.tags[t.id].centre }));
    if (v.length >= 2) { const r = triangulate(v); tri[t.id] = { board: tableToBoardPt(r.X), rms: r.rms }; }
  }
  // each rigid part on its own, from all its tags in all cameras (rigidtrack.js), started from the commanded pose
  const f0 = fk(commanded), parts = {};
  for (const link of Object.keys(ARM_PARTS)) {
    try { const pf = fitPartPose(link, views, { init: f0[link] }); if (pf) parts[link] = pf; } catch (e) { notes.push(`${link}: ${e.message}`); }
  }
  const shifts = Object.assign({}, ...Object.values(parts).map(p => p.shifts));
  // wrist + gripper together: wrist pose and S5 (7 numbers) from the tags of both parts
  let pair = null;
  try { pair = fitJointPair('s5', views, { init: { parent: f0.wrist, child: f0.hand } }); } catch { pair = null; }
  let result = null;
  try {
    if (views.length >= 2 || views.reduce((n, x) => n + Object.keys(x.tags).length, 0) >= 2) {
      // from the last tracked pose and from the joint panel; the better fit wins (either can be far from the real arm)
      const fitFrom = init => fitArmPose(views, { init, useCorners, shifts });
      result = [last, commanded].filter(Boolean).map(fitFrom).sort((x, y) => x.rms - y.rms)[0];
      // both far off (an arm moved by hand): also try the wrist turned in 45 degree steps
      if (result.rms > 6) for (let s5 = -180; s5 < 180; s5 += 45) { const r = fitFrom({ ...result.angles, s5 }); if (r.rms < result.rms) result = r; }
      result.triangulated = tri;
    }
  } catch (e) { notes.push(e.message); }
  return { tri, parts, pair, result, notes };
}

// ---- the jaw opening from the image: two dark finger bars along the jaw axis, at the pads' height

const U_STEP = 0.1, U_MAX = 4.5;                               // the profile across the jaws, cm (hand frame x)

/** Luminance darkness (0 white .. 1 black) at one image point, 3 x 3 px; null outside the frame. */
function darkAt(img, u, v) {
  const x0 = Math.round(u), y0 = Math.round(v), d = img.data;
  if (x0 < 1 || y0 < 1 || x0 >= img.width - 1 || y0 >= img.height - 1) return null;
  let s = 0;
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
    const i = ((y0 + dy) * img.width + x0 + dx) * 4;
    s += 1 - (0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2]) / 255;
  }
  return s / 9;
}

/**
 * Darkness across the jaws at the pads' height (hand frame: x from -U_MAX to U_MAX, a few rows around the pad
 * depth), and how many pixels the pads move apart per cm of gap in this camera (a camera looking along the jaw
 * axis cannot see the opening). Returns { dark: [..] (null where outside the frame), pxPerCm }.
 */
export function jawProfile(img, cal, angles, { w = img.width, h = img.height } = {}) {
  const hand = fk(angles).hand, sx = img.width / w, sy = img.height / h;
  const v0 = padDepthForTheta(thetaForGap(4.5)), v1 = padDepthForTheta(thetaForGap(0.5));
  // rows from 1,4 cm above the pads down to their lower edge: the finger bars, robust to a pose a little off in height
  const vm = (v0 + v1) / 2, rows = [vm - 1.4, vm - 1.0, vm - 0.6, vm - 0.2, vm + 0.2];
  const dark = [];
  for (let u = -U_MAX; u <= U_MAX + 1e-9; u += U_STEP) {
    let s = 0, n = 0;
    for (const v of rows) {
      const [px, py] = projectBoard(cal, point(hand, [u, 0, v]));
      const x = darkAt(img, px * sx, py * sy); if (x != null) { s += x; n++; }
    }
    dark.push(n ? s / n : null);
  }
  const pads = g => [-1, 1].map(k => projectBoard(cal, point(hand, [k * g / 2, 0, padDepthForTheta(thetaForGap(g))])));
  const sep = p => Math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1]);
  return { dark, pxPerCm: Math.max(0, sep(pads(4)) - sep(pads(0))) / 4 };
}

/**
 * Score per gap from one profile: two dark bars (the fingers, about 0,7 cm wide, just outward of the pads at
 * centre +- gap/2) against the light between and beside them; the centre may sit up to 1,5 cm off (the hand
 * pose is a little off). Returns { scores: one per gap (the best centre; null without data), mids: how much
 * lighter the space between the bars is there (null below 0,6 cm, where the bars merge) }.
 */
export function gapScores(profile, gaps) {
  const n = profile.dark.length, at = u => profile.dark[Math.round((u + U_MAX) / U_STEP)];
  const mean = (a, b) => { let s = 0, k = 0; for (let u = a; u <= b + 1e-9; u += U_STEP) { const x = u >= -U_MAX && u <= U_MAX ? at(u) : null; if (x != null) { s += x; k++; } } return k ? s / k : null; };
  const mids = [];
  const scores = gaps.map(g => {
    let best = null, bestMid = null;
    for (let c = -1.5; c <= 1.5 + 1e-9; c += U_STEP) {
      // the bars start just outside the pads (pad 0,27 cm thick on the inner side of each finger)
      const barL = mean(c - g / 2 - 0.95, c - g / 2 - 0.25), barR = mean(c + g / 2 + 0.25, c + g / 2 + 0.95);
      const outL = mean(c - g / 2 - 1.55, c - g / 2 - 1.05), outR = mean(c + g / 2 + 1.05, c + g / 2 + 1.55);
      const mid = g > 0.6 ? mean(c - g / 2 + 0.2, c + g / 2 - 0.2) : null;
      if (barL == null || barR == null || outL == null || outR == null) continue;
      const light = [outL, outR, mid].filter(x => x != null);
      const s = (barL + barR) / 2 - light.reduce((a, x) => a + x, 0) / light.length;
      if (best == null || s > best) { best = s; bestMid = mid == null ? null : (barL + barR) / 2 - mid; }
    }
    mids.push(bestMid);
    return best;
  });
  return { scores, mids };
}

/**
 * The jaw gap from all cameras ([{ scores, mids, pxPerCm }] for the same gaps). A camera counts only when it
 * clearly sees two separate dark bars with light between them (one dark blob can be closed jaws or one finger
 * hidden behind the other; that counts only when very strong, and not when another camera sees two bars). The counted cameras are averaged, weighted by how
 * well they see the opening and how clear their bars are. Without any: the cube width when one is held, else
 * null (unknown). Returns { gap, confidence, cameras }.
 */
export function pickGap(gaps, perCamera, { holding = false, cube = 2.65 } = {}) {
  const votes = [];
  for (const c of perCamera) {
    if (!(c.pxPerCm > 1) || !c.scores.some(x => x != null)) continue;
    const k = c.scores.indexOf(Math.max(...c.scores.filter(x => x != null))), g = gaps[k], s = c.scores[k];
    const clear = g >= 1 ? (c.mids[k] ?? 0) >= 0.06 && s >= 0.06 : s >= 0.15;
    if (clear) votes.push({ g, w: c.pxPerCm * s });
  }
  // two separate bars prove an opening; one blob is also explained by one finger hiding the other
  if (votes.some(v => v.g >= 1)) votes.splice(0, votes.length, ...votes.filter(v => v.g >= 1));
  if (!votes.length) return { gap: holding ? cube : null, confidence: 0, cameras: 0 };
  const w = votes.reduce((a, v) => a + v.w, 0);
  return { gap: votes.reduce((a, v) => a + v.g * v.w, 0) / w, confidence: w / votes.length / 10, cameras: votes.length };
}

// ---- the jaw opening from the screw heads: silver screws on the black gripper plates

/**
 * Screw heads on the gripper (hand frame, cm) for a jaw gap, on the face `side` (+1: hand +Y, -1: -Y) as in
 * model/gripper.js: fixed ones on the gear plate (they do not move with the jaws, so they pin down where the
 * gripper is), and on each finger its pivot, the outer link's end and the two spacers (they move with the gap).
 */
export function gripperScrews(gap, side = 1) {
  const th = thetaForGap(gap) * Math.PI / 180, s = Math.sin(th), c = Math.cos(th);
  const [gu, gv] = GRIPPER.gear, [ou, ov] = GRIPPER.outer, L = GRIPPER.link, [tu, tv] = GRIPPER.tip;
  const fixed = [], moving = [];
  for (const sx of [-1, 1]) {
    fixed.push([sx * ou, side > 0 ? 0.74 : -0.62, ov]);
    if (side > 0) fixed.push([sx * gu, 0.74, gv]);
    const fu = sx * (gu + L * s), fv = gv + L * c;                       // finger pivot (end of the gear link)
    moving.push([fu, side * 0.5, fv], [sx * (ou + L * s), side * 0.5, ov + L * c],
      [fu + sx * -0.3, side * 0.55, fv + 2.8], [fu + sx * (tu + 0.7), side * 0.55, fv + tv - 0.9]);
  }
  return { fixed, moving };
}

/**
 * Small bright spots with a dark ring around them (a screw head on a black plate) in a window of an image.
 * r: the expected head radius in px. A bright area (the mat) has no dark ring and gives nothing.
 * Returns [[u, v] (image px)], brightest first.
 */
export function brightSpots(img, roi, r) {
  const [x0, y0, x1, y1] = [Math.max(2, Math.floor(roi[0])), Math.max(2, Math.floor(roi[1])), Math.min(img.width - 3, Math.ceil(roi[2])), Math.min(img.height - 3, Math.ceil(roi[3]))];
  const d = img.data, W = img.width, lum = (x, y) => { const i = (y * W + x) * 4; return 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2]; };
  const ring = Math.max(3, 2.4 * r), K = 12, found = [];
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    const L = lum(x, y);
    if (L < 120) continue;
    if (lum(x - 1, y) > L || lum(x + 1, y) > L || lum(x, y - 1) > L || lum(x, y + 1) > L) continue;   // local maximum
    let s = 0, n = 0, bright = 0;
    for (let k = 0; k < K; k++) {
      const a = 2 * Math.PI * k / K, u = Math.round(x + ring * Math.cos(a)), v = Math.round(y + ring * Math.sin(a));
      if (u < 0 || v < 0 || u >= W || v >= img.height) continue;
      const l = lum(u, v); s += l; n++; if (l > L - 40) bright++;
    }
    if (n < K / 2 || bright > 2 || s / n > L - 60) continue;               // most of the ring clearly darker
    found.push([x, y, L]);
  }
  found.sort((a, b) => b[2] - a[2]);
  const out = [];
  for (const [x, y] of found) if (!out.some(([u, v]) => Math.hypot(u - x, v - y) < ring)) out.push([x, y]);
  return out;
}

/**
 * Score per candidate gap in one frame from the screw heads: the model's screws projected for that gap, shifted
 * by up to +-20 px (the hand pose is a little off), against the detected spots; score = finger screws matched
 * (+ half a point per fixed screw matched). Returns { scores, shifts, spots, r, side, pxPerCm }.
 */
export function screwScores(img, cal, angles, gaps, { w = img.width, h = img.height } = {}) {
  const hand = fk(angles).hand, sx = img.width / w, sy = img.height / h;
  // the face that looks at this camera, and the screw head's size there
  const C = cal.centre || [0, 1, 2].map(i => -(cal.R[0][i] * cal.t[0] + cal.R[1][i] * cal.t[1] + cal.R[2][i] * cal.t[2]));
  const Cb = tableToBoardPt(C), yAx = [hand.R[1], hand.R[4], hand.R[7]];
  const side = (Cb[0] - hand.p[0]) * yAx[0] + (Cb[1] - hand.p[1]) * yAx[1] + (Cb[2] - hand.p[2]) * yAx[2] >= 0 ? 1 : -1;
  const dist = Math.hypot(Cb[0] - hand.p[0], Cb[1] - hand.p[1], Cb[2] - hand.p[2]);  // cm
  const r = cal.f * 0.27 / dist * sx;                                    // screw head radius ~2,7 mm, in image px
  const proj = pts => pts.map(p => projectBoard(cal, point(hand, p))).map(([u, v]) => [u * sx, v * sy]);
  const sets = gaps.map(g => { const s = gripperScrews(g, side); return { fixed: proj(s.fixed), moving: proj(s.moving) }; });
  const all = sets.flatMap(s => [...s.fixed, ...s.moving]);
  const roi = [Math.min(...all.map(p => p[0])) - 30, Math.min(...all.map(p => p[1])) - 30, Math.max(...all.map(p => p[0])) + 30, Math.max(...all.map(p => p[1])) + 30];
  const spots = brightSpots(img, roi, r), tol = Math.max(3, 1.3 * r);
  const near = (p, du, dv) => spots.some(([u, v]) => Math.abs(u - p[0] - du) < tol && Math.abs(v - p[1] - dv) < tol && Math.hypot(u - p[0] - du, v - p[1] - dv) < tol);
  const shifts = [], scores = sets.map(s => {
    let best = -1, bs = [0, 0];
    for (let du = -20; du <= 20; du += 2) for (let dv = -20; dv <= 20; dv += 2) {
      const sc = s.moving.filter(p => near(p, du, dv)).length + 0.5 * s.fixed.filter(p => near(p, du, dv)).length;
      if (sc > best || (sc === best && Math.hypot(du, dv) < Math.hypot(...bs))) { best = sc; bs = [du, dv]; }
    }
    shifts.push(bs);
    return best;
  });
  const pads = g => [-1, 1].map(k => projectBoard(cal, point(hand, [k * g / 2, 0, padDepthForTheta(thetaForGap(g))])));
  const sep = p => Math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1]);
  return { scores, shifts, spots: spots.map(([u, v]) => [u / sx, v / sy]), r, side, pxPerCm: Math.max(0, sep(pads(4)) - sep(pads(0))) / 4 };
}

/** The gap from the screw scores of all cameras: the most screws matched in all; null when that is not clear. */
export function pickGapFromScrews(gaps, perCamera) {
  const use = perCamera.filter(c => c && c.pxPerCm > 1);
  if (!use.length) return { gap: null, matched: 0 };
  const total = gaps.map((_, k) => use.reduce((s, c) => s + c.scores[k], 0));
  const best = Math.max(...total), k = total.indexOf(best);
  // clear: at least 3 finger screws found, and clearly more than with the gap 1 cm off
  const off = total.filter((_, j) => Math.abs(gaps[j] - gaps[k]) >= 1);
  return { gap: best >= 3 && best - Math.max(...off) >= 1 ? gaps[k] : null, matched: best, index: k };
}
