import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CAMERAS, CUBE } from '../js/config.js';
import { calFromLookAt } from '../js/armtrack.js';
import { locateCubes, visibleCentroidOffset, ray, rayGap } from '../js/locate3d.js';
import { project } from '../js/calib.js';

const cams = CAMERAS.map(c => calFromLookAt(c.posMm, c.lookAtMm, c.fPx, c.w, c.h));
const H = CUBE * 10 / 2;
// cubes in the table frame (mm): on the mat, stacked on it (level 1, 2), and one in the air (gripper)
const cubes = [
  { colour: 'orange', C: [150, 40, H] }, { colour: 'blue', C: [150, 40, 3 * H] }, { colour: 'yellow', C: [150, 40, 5 * H] },
  { colour: 'green', C: [230, -60, H] }, { colour: 'red', C: [120, -30, 95] },
];
// what a camera's blob centroid sees: the area-weighted centre of the cube's visible faces, projected
// (a cube under another one shows no top face)
const covered = c => cubes.some(o => o !== c && o.C[0] === c.C[0] && o.C[1] === c.C[1] && Math.abs(o.C[2] - c.C[2] - 2 * H) < 1);
const blob = (cal, c) => {
  const { C } = ray(cal, [cal.cx, cal.cy]);
  const v = C.map((x, i) => x - c.C[i]), n = Math.hypot(...v);
  const o = visibleCentroidOffset(v.map(x => x / n), 0, { top: !covered(c) });
  return project(cal, c.C.map((x, i) => x + o[i]));
};

test('rays through the same point meet; the gap of skew rays is their distance', () => {
  const P = [150, 0, 30];
  const g = rayGap(ray(cams[0], project(cams[0], P)), ray(cams[1], project(cams[1], P)));
  assert.ok(g.gap < 1e-6 && Math.hypot(...g.mid.map((x, i) => x - P[i])) < 1e-6);
});

test('cubes located in 3D from two cameras: height and stack level, not assumed on the mat', () => {
  const views = cams.map(cal => ({ cal, detections: cubes.map(c => ({ colour: c.colour, uv: blob(cal, c), yaw: 0 })) }));
  const found = locateCubes(views);
  assert.equal(found.length, cubes.length);
  for (const c of cubes) {
    const f = found.find(x => x.colour === c.colour);
    const err = Math.hypot(...f.X.map((x, i) => x - c.C[i]));
    assert.ok(err < 4, `${c.colour}: ${err.toFixed(1)} mm off`);
  }
  assert.deepEqual(['orange', 'blue', 'yellow', 'green'].map(k => found.find(x => x.colour === k).level), [0, 1, 2, 0]);
  assert.ok(found.find(x => x.colour === 'red').X[2] > 80);           // held in the air
});

test('one camera is not enough', () => {
  assert.deepEqual(locateCubes([{ cal: cams[0], detections: [{ colour: 'blue', uv: [600, 400] }] }]), []);
});
