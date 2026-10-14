import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const webRoot = fileURLToPath(new URL('..', import.meta.url));
const photoRoot = path.resolve(webRoot, '..', 'docs', 'images', 'photos');

test('hardware photo catalog and comparison views stay complete', () => {
  const photos = fs.readdirSync(photoRoot).filter((name) => /\.(?:jpe?g|png)$/i.test(name));
  assert.equal(photos.length, 39);
  const configuration = JSON.parse(fs.readFileSync(path.join(webRoot, 'tools', 'photo-views.json'), 'utf8'));
  assert.ok(configuration.views.length >= 9);
  assert.ok(configuration.views.some((view) => view.camera.markers));
  for (const view of configuration.views) assert.ok(photos.includes(view.photo), `missing reference photo ${view.photo}`);
});

test('the active camera tab loads its saved frame through the guarded photo route', () => {
  const camview = fs.readFileSync(fileURLToPath(new URL('../js/camview.js', import.meta.url)), 'utf8');
  assert.match(camview, /\/reference-photos\/controller-camera-2026-10-12\.jpg/);
  assert.match(camview, /async function scanConnected/);
  assert.match(camview, /onVisionUpdate/);
});
