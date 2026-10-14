import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';
import { createPlanChannel } from './src/operational/plan-channel.js';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const PHOTO_ROOT = path.join(path.dirname(ROOT), 'docs', 'images', 'photos');
const TYPES = { '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.png': 'image/png' };

function registerReferencePhotos(server) {
  server.middlewares.use('/reference-photos', (request, response, next) => {
    const pathname = decodeURIComponent(new URL(request.url, 'http://vite').pathname);
    const filename = pathname.replace(/^\/+/, '');
    if (!/^[A-Za-z0-9._-]+\.(?:jpe?g|png)$/i.test(filename)) return next();
    const file = path.join(PHOTO_ROOT, filename);
    if (!fs.existsSync(file) || !fs.statSync(file).isFile()) return next();
    response.writeHead(200, {
      'Content-Type': TYPES[path.extname(file).toLowerCase()],
      'Content-Length': fs.statSync(file).size,
      'Cache-Control': 'no-cache',
      'X-Content-Type-Options': 'nosniff',
    });
    fs.createReadStream(file).pipe(response);
  });
}

// Claude's plan channel in development too (simulation only; there is no robot proxy here).
function registerPlanChannel(server) {
  const channel = createPlanChannel();
  server.middlewares.use((request, response, next) => {
    const pathname = new URL(request.url, 'http://vite').pathname;
    if (!channel.handles(pathname)) return next();
    channel.handle(request, response, pathname).catch(next);
  });
}

export default defineConfig({
  plugins: [{
    name: 'local-reference-photos',
    configureServer: registerReferencePhotos,
    configurePreviewServer: registerReferencePhotos,
  }, {
    name: 'claude-plan-channel',
    configureServer: registerPlanChannel,
    configurePreviewServer: registerPlanChannel,
  }],
});
