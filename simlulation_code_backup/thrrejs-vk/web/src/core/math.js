export const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
export const lerp = (start, end, amount) => start + (end - start) * amount;

export function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
export function subtract(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
export function scale(vector, amount) { return vector.map((value) => value * amount); }
export function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
export function length(vector) { return Math.hypot(...vector); }
export function distance(a, b) { return length(subtract(a, b)); }

export function identityTransform() {
  return { rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], translation: [0, 0, 0] };
}

export function rotationX(radians) {
  const c = Math.cos(radians); const s = Math.sin(radians);
  return [[1, 0, 0], [0, c, -s], [0, s, c]];
}

export function rotationZ(radians) {
  const c = Math.cos(radians); const s = Math.sin(radians);
  return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
}

export function multiplyMatrix(a, b) {
  return a.map((row, r) => b[0].map((_, c) => row.reduce((sum, value, k) => sum + value * b[k][c], 0)));
}

export function rotate(matrix, vector) {
  return matrix.map((row) => dot(row, vector));
}

export function compose(transform, rotation, translation = [0, 0, 0]) {
  return {
    rotation: multiplyMatrix(transform.rotation, rotation),
    translation: add(transform.translation, rotate(transform.rotation, translation)),
  };
}

export function transformPoint(transform, point) {
  return add(transform.translation, rotate(transform.rotation, point));
}

export function seededRandom(seed = 1) {
  let state = seed >>> 0;
  return () => {
    state = (1664525 * state + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}
