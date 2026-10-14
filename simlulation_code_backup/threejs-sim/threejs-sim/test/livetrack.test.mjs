// node --test test/*.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { gapScores, pickGap } from '../js/armsolve.js';

// a darkness profile across the jaws (-4,5 .. 4,5 cm, 0,1 cm steps) with dark bars where the fingers are
const profile = bars => ({ dark: Array.from({ length: 91 }, (_, k) => { const u = -4.5 + k * 0.1; return bars.some(([a, b]) => u >= a - 1e-9 && u <= b + 1e-9) ? 0.9 : 0.1; }), pxPerCm: 12 });
const gaps = Array.from({ length: 19 }, (_, i) => i * 0.25);

test('jaw opening: two finger bars give the gap, also with the hand pose 1 cm off', () => {
  // fingers just outside pads at +-1,5 cm (gap 3), the whole gripper shifted by 1 cm
  const r = gapScores(profile([[-1.5 - 0.95 + 1, -1.5 - 0.25 + 1], [1.5 + 0.25 + 1, 1.5 + 0.95 + 1]]), gaps);
  assert.equal(gaps[r.scores.indexOf(Math.max(...r.scores))], 3);
  const g = pickGap(gaps, [{ ...r, pxPerCm: 12 }]);
  assert.equal(g.gap, 3); assert.equal(g.cameras, 1);
});

test('jaw opening: a camera that sees one blob (one finger behind the other) does not overrule two bars', () => {
  const two = { ...gapScores(profile([[-1.25 - 0.95, -1.25 - 0.25], [1.25 + 0.25, 1.25 + 0.95]]), gaps), pxPerCm: 12 };
  const blob = { ...gapScores(profile([[-0.9, 0.9]]), gaps), pxPerCm: 12 };
  assert.equal(pickGap(gaps, [two, blob]).gap, 2.5);
  // nothing clear: the cube width when one is held, else unknown
  const flat = { ...gapScores({ dark: new Array(91).fill(0.2), pxPerCm: 12 }, gaps), pxPerCm: 12 };
  assert.equal(pickGap(gaps, [flat], { holding: true }).gap, 2.65);
  assert.equal(pickGap(gaps, [flat]).gap, null);
});
