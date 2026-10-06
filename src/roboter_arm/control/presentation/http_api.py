"""HTTP API and browser entry point; loopback and hardware-free preview by default."""
import argparse
from functools import partial
import ipaddress
import signal
import socket
import threading
import time
from urllib.parse import urlsplit

import anyio
from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, Response
from starlette.datastructures import Headers
from starlette.exceptions import HTTPException
import uvicorn

from roboter_arm.control.application.session import Session, STARTUP, OLD_HOME, PARK, HOME, SPEED
from roboter_arm.control.application.camera import CameraService
from roboter_arm.control.infrastructure.demo_store import DemoStore
from roboter_arm.control.infrastructure.drivers import PreviewDriver, HardwareDriver
from roboter_arm.control.infrastructure.pose_store import PoseStore
from roboter_arm.control.presentation import api_models as body
from roboter_arm.control.presentation.camera_api import register as register_camera
from roboter_arm.shared.joints import JOINTS
from roboter_arm.shared.paths import ROOT

FAILURE = 'Driver/server failure; use servo power switch'
ASSETS = {'/':('manual_control.html', 'text/html; charset=utf-8'),
          '/park-illustration-v2.png':('park-illustration-v2.png', 'image/png'),
          '/home-illustration.png':('home-illustration.png', 'image/png'),
          '/manual_control.css':('manual_control.css', 'text/css; charset=utf-8'),
          '/manual_control.js':('manual_control.js', 'text/javascript; charset=utf-8')}
# Swagger groups, in display order.
TAGS = [{'name':'State', 'description':'Read-only status; polling it never prepares hardware.'},
        {'name':'Power', 'description':'Start and end an attended session; Stop disables PWM directly.'},
        {'name':'Motion', 'description':'Move or jog one joint while the others hold.'},
        {'name':'Poses', 'description':'Go to or save the HOME and PARK poses.'},
        {'name':'Session settings', 'description':'Joint bounds, step size and demo speed for this running session.'},
        {'name':'Demo', 'description':'Record, edit, play back, save and load demo positions.'},
        {'name':'Bounded tests', 'description':'Arm a session restricted to fewer joints or narrowed ranges '
         '(manual_control.py --channels/--range), where power_on refuses. No automatic PARK/HOME move; '
         'the first move needs first_clear: true. Not used by the page.'},
        {'name':'Camera', 'description':'Read camera frames and submit supported settings; no servo commands.'}]
# path: (Session method, request body, tag, summary). Stop is separate; see create_app.
ACTIONS = {'power_on':('power_on', body.PowerOn, 'Power', 'Enable PARK, then move to HOME; needs fresh physical PARK/readiness'),
           'power_off':('power_off', body.Empty, 'Power', 'Return known commands to PARK, then disable all outputs'),
           'prepare':('prepare', body.Prepare, 'Bounded tests', 'Step 1: acquire the controller and verify outputs off'),
           'arm':('arm', body.Arm, 'Bounded tests', 'Step 2: arm after fresh attended readiness; no movement'),
           'move':('move', body.Move, 'Motion', 'Move one joint; the others hold'),
           'save':('save_pose', body.SavePose, 'Poses', 'Archive the full pose and make it the active HOME or PARK'),
           'go_pose':('go_pose', body.GoPose, 'Poses', 'Move all six joints to HOME or PARK and hold'),
           'limits':('set_limits', body.Limits, 'Session settings', "Change one joint's session bounds without moving"),
           'settings':('set_settings', body.Settings, 'Session settings', 'Change step size and demo speed while idle'),
           'jog':('jog', body.Jog, 'Motion', 'Add (+1) or subtract (-1) one step from the held command'),
           'demo/name':('demo_name', body.Name, 'Demo', 'Name the demo draft'),
           'demo/add':('demo_add', body.OptionalName, 'Demo', 'Capture all six settled commands as a position'),
           'demo/replace':('demo_replace', body.Index, 'Demo', 'Replace a position with the current settled commands'),
           'demo/rename':('demo_rename', body.Rename, 'Demo', 'Rename a position'),
           'demo/reorder':('demo_reorder', body.Reorder, 'Demo', 'Move a position up (-1) or down (+1)'),
           'demo/remove':('demo_remove', body.Index, 'Demo', 'Remove a position from the draft; saved files stay'),
           'demo/next':('demo_next', body.Empty, 'Demo', 'Execute the next position, then hold'),
           'demo/run':('demo_run', body.Empty, 'Demo', 'Run all positions once from position 1, then hold'),
           'demo/pause':('demo_pause', body.Empty, 'Demo', 'Finish the current step, then hold'),
           'demo/resume':('demo_resume', body.Empty, 'Demo', 'Continue the remaining steps after a pause'),
           'demo/restart':('demo_restart', body.Empty, 'Demo', 'Reset demo progress without moving'),
           'demo/save':('demo_save', body.Empty, 'Demo', 'Write the draft as a new immutable file; returns its name'),
           'demo/load':('demo_load', body.Filename, 'Demo', 'Load a saved draft with progress reset; does not move')}
RESPONSES = {200:{'model':body.Accepted}, 400:{'model':body.Refused}, 403:{'model':body.Refused},
             500:{'model':body.Refused}}


def allowed_host(host):
    """An IP address, a name without dots or a .local name; DNS-rebinding pages use none of these."""
    try:
        name = urlsplit('http://' + (host or '')).hostname
    except ValueError:
        return False
    if not name:
        return False
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        return '.' not in name or name.endswith('.local')


def refusal(method, headers):
    """(error, status) for a request the guard turns away, or None."""
    host = headers.get('host')
    if method == 'POST':
        if not allowed_host(host) or headers.get('origin') != f'http://{host}':
            return 'Same-origin page required', 403
        if headers.get('content-type', '').split(';')[0].strip().lower() != 'application/json':
            return 'Content-Type must be application/json', 400
        length = headers.get('content-length', '0')
        if not length.isdigit() or not 0 < int(length) <= 8192:
            return 'Request too large or empty', 400
    elif not allowed_host(host):
        return 'Use an IP address or local host name', 403
    return None


class Guard:
    """Host and same-origin checks, the POST size limit, and no-store headers on every response."""
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope['type'] != 'http':
            return await self.app(scope, receive, send)
        async def secured(message):
            if message['type'] == 'http.response.start':
                message['headers'] = [*message.get('headers', []), (b'cache-control', b'no-store'),
                                      (b'x-content-type-options', b'nosniff')]
            await send(message)
        refused = refusal(scope['method'], Headers(scope=scope))
        if refused:
            return await JSONResponse({'error':refused[0]}, refused[1])(scope, receive, secured)
        await self.app(scope, receive, secured)


def call(session, method, args):
    try:
        return {'ok':True, 'result':getattr(session, method)(**args.model_dump(exclude_unset=True))}
    except (ValueError, TypeError, KeyError) as error:
        return JSONResponse({'error':str(error)}, 400)
    except Exception:
        session.stop(FAILURE)
        return JSONResponse({'error':FAILURE}, 500)


def action(method, model):
    # A plain def runs in the worker thread pool; Session blocks on locks and would stall the event loop.
    def endpoint(request: Request, args: model):
        return call(request.app.state.session, method, args)
    return endpoint


def asset(name, media_type):
    def endpoint():
        return Response((ROOT / 'web' / name).read_bytes(), media_type=media_type)
    return endpoint


def create_app(session, camera=None):
    app = FastAPI(title='Robot arm control API', docs_url='/docs', openapi_url='/openapi.json',
                  redoc_url=None, redirect_slashes=False, openapi_tags=TAGS)
    app.state.session = session
    app.state.camera = camera if camera is not None else CameraService(session)
    # Stop has its own threads, so it never queues behind requests waiting on the session lock.
    app.state.stop_threads = anyio.CapacityLimiter(2)
    app.add_middleware(Guard)

    @app.exception_handler(RequestValidationError)
    async def invalid(_request, error):
        first = error.errors()[0]
        field = '.'.join(part for part in first['loc'][1:] if isinstance(part, str))
        return JSONResponse({'error':f'{field}: {first["msg"]}' if field else first['msg']}, 400)

    @app.exception_handler(HTTPException)
    async def unknown(_request, error):
        message = 'Unknown path' if error.status_code == 404 else str(error.detail)
        return JSONResponse({'error':message}, error.status_code, headers=error.headers)

    for path, (name, media_type) in ASSETS.items():
        app.add_api_route(path, asset(name, media_type), methods=['GET'], include_in_schema=False)

    @app.get('/api/state', tags=['State'], summary='Read-only session state; never prepares hardware')
    def state(request: Request):
        return request.app.state.session.state()

    @app.post('/api/stop', tags=['Power'], responses=RESPONSES,
              summary='Cancel immediately and disable outputs directly; no recovery movement')
    async def stop(request: Request, args: body.Stop):
        return await anyio.to_thread.run_sync(partial(call, request.app.state.session, 'stop', args),
                                              limiter=request.app.state.stop_threads)

    for path, (method, model, tag, summary) in ACTIONS.items():
        app.add_api_route(f'/api/{path}', action(method, model), methods=['POST'], tags=[tag],
                          summary=summary, responses=RESPONSES)
    register_camera(app, app.state.camera)
    return app


def session_for(driver, channels, *, directory, demo_directory, **options):
    """Session with pose and demo records under explicit directories, scoped to the driver."""
    bounds = {c:(low, high) for c, (_, low, high) in JOINTS.items()}
    return Session(driver, channels, pose_store=PoseStore(directory, driver.scope),
                   demo_store=DemoStore(demo_directory, driver.scope, bounds), **options)


class Uvicorn(uvicorn.Server):
    def __init__(self, config, server):
        super().__init__(config)
        self.server = server

    def handle_exit(self, sig, frame):
        # A signal disables PWM at once, as before; open requests then get at most 1 s.
        self.server.session.stop()
        super().handle_exit(sig, frame)


class Server:
    """Uvicorn on a socket bound at construction, so the port is known before serving starts."""
    def __init__(self, session, port, host='127.0.0.1', *, camera=None):
        self.app = create_app(session, camera)
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.listen(128)
        self.server_address = self.socket.getsockname()
        self.uvicorn = Uvicorn(uvicorn.Config(self.app, access_log=False, log_level='warning',
                                              proxy_headers=False, server_header=False,
                                              timeout_graceful_shutdown=1), self)
        self.thread = None

    @property
    def session(self):
        return self.app.state.session

    @session.setter
    def session(self, session):
        self.app.state.session = session

    def run(self):
        self.uvicorn.run(sockets=[self.socket])

    def start(self, timeout=5):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        deadline = time.monotonic() + timeout
        while not self.uvicorn.started:
            if not self.thread.is_alive() or time.monotonic() > deadline:
                raise RuntimeError('HTTP server did not start')
            time.sleep(.01)

    def stop(self):
        self.uvicorn.should_exit = True
        if self.thread is not None:
            self.thread.join(timeout=5)
        self.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Permit explicit attended UI preparation; no initialization at launch')
    parser.add_argument('--channels', type=int, nargs='+', help='Explicit hardware-session channel set')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', default='127.0.0.1', help='Listen address; 0.0.0.0 serves every network interface')
    parser.add_argument('--camera', choices=['oak'], help='Opt in to OAK-D Lite RGB acquisition; requires the pinned camera runtime')
    parser.add_argument('--range', dest='ranges', action='append', default=[], metavar='CHANNEL:LOW:HIGH')
    parser.add_argument('--session-seconds', type=int, help='Optional bounded-test duration, at most 600 seconds')
    parser.add_argument('--max-moves', type=int, help='Optional bounded-test target count, at most 60')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535 or (args.hardware and not args.channels):
        parser.error('Port must be 1024–65535; hardware mode requires explicit --channels')
    try:
        ranges = {}
        for value in args.ranges:
            channel, low, high = map(int, value.split(':'))
            if channel in ranges:
                raise ValueError('Duplicate session range')
            ranges[channel] = (low, high)
        session = session_for(HardwareDriver() if args.hardware else PreviewDriver(),
                              args.channels if args.channels is not None else list(JOINTS),
                              directory=ROOT / 'artifacts/poses', demo_directory=ROOT / 'artifacts/demos',
                              ranges=ranges, seconds=args.session_seconds, max_moves=args.max_moves)
    except ValueError as error:
        parser.error(str(error))
    from roboter_arm.control.infrastructure.camera_process import CameraProcess
    camera = CameraService(session, CameraProcess() if args.camera == 'oak' else None)
    server = Server(session, args.port, args.host, camera=camera)
    finished = threading.Event()
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt()
    # Uvicorn handles SIGINT/SIGTERM itself, then restores this handler and re-raises the signal.
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, server.uvicorn.handle_exit)
    def monitor():
        while not finished.wait(.25):
            session.watchdog()
    threading.Thread(target=monitor, daemon=True).start()
    print(f'{"HARDWARE commissioning" if args.hardware else "PREVIEW, no hardware"}: http://{args.host}:{args.port}', flush=True)
    print('No startup PWM. Direct stop disables holding torque; keep support and power switch ready.', flush=True)
    if args.camera:
        print('OAK camera enabled; servo driver mode is independent. Metric calibration is not ready.', flush=True)
    try:
        camera.start()
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        finished.set()
        try:
            stopped = session.close()
        finally:
            camera.close()
            server.socket.close()
        if not stopped:
            raise SystemExit('Output disable/readback failed; switch servo power OFF')
