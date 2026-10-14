"""Static server for the 3D view, with an optional proxy to an arm API:

    python -m armsim.web [--port 8780] [--api http://10.102.4.103:8765]

Standard library only. Without --api the view runs standalone with sliders. With --api, requests
to /api/... and /sim/... are forwarded to that arm, so the page can follow it from the same
origin: the robot-arm API refuses cross-origin requests by design, and the real arm has no
/sim routes, which the page treats as "no block positions".
"""
import argparse
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from armsim.client import ArmError, HttpArmClient
from armsim.config import ROOT, load_robot, load_scene
from armsim.mover import Mover

WEB = ROOT / 'web'
TYPES = {'.js': 'text/javascript', '.mjs': 'text/javascript', '.html': 'text/html', '.css': 'text/css', '.json': 'application/json'}
PROXIED = ('/api/', '/sim/', '/openapi.json', '/docs')


class Handler(SimpleHTTPRequestHandler):
    api = None      # base URL of the arm to forward to, or None
    mover = None    # one Mover per proxy process, created on first use

    def reply(self, status, payload):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def move_to(self, method):
        """/move_to lives in the proxy, so the real arm gets the same moves as the sandbox."""
        cls = type(self)
        if cls.mover is None:
            cls.mover = Mover(HttpArmClient(self.api), load_robot(), load_scene())
        path = self.path.split('?')[0]
        try:
            if path == '/move_to/status' and method == 'GET':
                return self.reply(200, cls.mover.status())
            if path == '/move_to' and method == 'POST':
                length = int(self.headers.get('Content-Length') or 0)
                args = json.loads(self.rfile.read(length) or b'{}')
                call = cls.mover.plan if args.get('dry') else cls.mover.start
                result = call(float(args['x']), float(args['y']), float(args.get('z', 0.01325)), float(args.get('yaw', 0.0)), bool(args.get('lower', True)), str(args.get('path', 'direct')))
                return self.reply(200, {'ok': True, 'result': dict(steps=result) if args.get('dry') else cls.mover.status()})
            if path == '/move_to/stop' and method == 'POST':
                cls.mover.stop()
                return self.reply(200, {'ok': True, 'result': cls.mover.status()})
        except (ArmError, ValueError, KeyError, TypeError) as error:
            return self.reply(400, {'error': str(error)})
        return self.reply(404, {'error': 'Unknown path'})

    def guess_type(self, path):
        for suffix, media in TYPES.items():
            if str(path).endswith(suffix):
                return media + '; charset=utf-8'
        return super().guess_type(path)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.api and self.path.startswith('/move_to'):
            return self.move_to('GET')
        if self.api and self.path.startswith(PROXIED):
            return self.proxy('GET')
        path, _, query = self.path.partition('?')
        if path in ('/', '/arm3d'):
            self.path = '/arm3d.html' + ('?' + query if query else '')
        super().do_GET()

    def do_POST(self):
        if self.api and self.path.startswith('/move_to'):
            return self.move_to('POST')
        if self.api and self.path.startswith(PROXIED):
            return self.proxy('POST')
        self.send_error(405)

    def proxy(self, method):
        length = int(self.headers.get('Content-Length') or 0)
        body = self.rfile.read(length) if length else None
        target = urlsplit(self.api)
        headers = {'Origin': f'{target.scheme}://{target.netloc}'}
        if method == 'POST':
            headers['Content-Type'] = 'application/json'
        request = Request(self.api + self.path, data=body, method=method, headers=headers)
        passthrough = None
        try:
            with urlopen(request, timeout=15) as response:
                status, data, ctype = response.status, response.read(), response.headers.get('Content-Type')
                passthrough = response.headers.get('X-Camera-Frame')
        except HTTPError as error:
            status, data, ctype = error.code, error.read(), error.headers.get('Content-Type')
        except URLError as error:
            status, data, ctype = 502, json.dumps({'error': f'proxy: no arm at {self.api} ({error.reason})'}).encode(), 'application/json'
        self.send_response(status)
        self.send_header('Content-Type', ctype or 'application/octet-stream')
        self.send_header('Content-Length', str(len(data)))
        if passthrough:
            self.send_header('X-Camera-Frame', passthrough)
        self.end_headers()
        self.wfile.write(data)


def serve(port=8780, host='127.0.0.1', api=None):
    handler = partial(type('ProxyHandler', (Handler,), {'api': api.rstrip('/') if api else None}), directory=str(WEB))
    return ThreadingHTTPServer((host, port), handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8780)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--api', help='Arm API to follow, e.g. http://10.102.4.103:8765 (real arm) or http://127.0.0.1:8766 (sandbox)')
    args = parser.parse_args()
    server = serve(args.port, args.host, args.api)
    if args.api:
        print(f'3D view following {args.api}: http://{args.host}:{args.port}/arm3d  (API and docs are proxied: /api/state, /docs)', flush=True)
    else:
        print(f'3D view standalone: http://{args.host}:{args.port}/arm3d  (add --api URL to follow an arm)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
