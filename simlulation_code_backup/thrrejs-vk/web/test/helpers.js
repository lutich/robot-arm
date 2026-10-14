import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

export function loadSpec() {
  return JSON.parse(fs.readFileSync(fileURLToPath(new URL('../public/arm-spec.json', import.meta.url)), 'utf8'));
}
