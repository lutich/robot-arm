"""Camera HTTP transport; capture and control remain outside request handlers."""
import base64
import json
import time

import anyio
from fastapi import Request
from fastapi.responses import JSONResponse, Response, StreamingResponse

from roboter_arm.control.domain.camera import CameraConflict, CameraUnavailable
from roboter_arm.control.presentation import api_models as body

CAMERA_PATHS = ('/api/camera/status', '/api/camera/settings',
                '/api/camera/snapshot.jpg', '/api/camera/stream.mjpg', '/api/camera/capture',
                '/api/camera/depth/snapshot.png', '/api/camera/depth/preview.png',
                '/api/camera/depth/stream')
CAMERA_FAILURE = 'Camera request failed'
REFUSALS = {status:{'model':body.Refused} for status in (400, 409, 503, 500)}


def refusal(error):
    if isinstance(error, CameraConflict):
        status = 409
    elif isinstance(error, CameraUnavailable):
        status = 503
    elif isinstance(error, ValueError):
        status = 400
    else:
        return JSONResponse({'error':CAMERA_FAILURE}, 500)
    return JSONResponse({'error':str(error)}, status)


def frame_header(frame):
    return json.dumps(frame.metadata(), separators=(',', ':'), ensure_ascii=True)


def image(frame, kind):
    if kind == 'rgb':
        return frame.jpeg, 'image/jpeg'
    if frame.depth is None:
        raise CameraUnavailable('Stereo depth is unavailable')
    return (frame.depth.png if kind == 'depth' else frame.depth.preview_png), 'image/png'


def part(frame, kind='rgb'):
    data, media_type = image(frame, kind)
    headers = (f'--frame\r\nContent-Type: {media_type}\r\nContent-Length: {len(data)}\r\n'
               f'X-Camera-Frame: {frame_header(frame)}\r\n\r\n').encode('ascii')
    return headers + data + b'\r\n'


def register(app, camera):
    @app.get('/api/camera/status', tags=['Camera'], responses=REFUSALS,
             summary='Camera availability, supported settings and observed frame values')
    def status():
        try:
            return camera.status()
        except Exception as error:
            return refusal(error)

    @app.post('/api/camera/settings', tags=['Camera'], responses={200:{'model':body.Accepted}, **REFUSALS},
              summary='Submit supported camera settings while motion and camera are idle')
    def settings(args: body.CameraSettings):
        try:
            return {'ok':True, 'result':camera.settings(**args.model_dump(exclude_unset=True))}
        except Exception as error:
            return refusal(error)

    async def wait_frame(request, after_run_id, after_sequence, wait_ms):
        if (after_run_id is None) != (after_sequence is None):
            raise ValueError('after_run_id and after_sequence must be supplied together')
        if after_run_id is not None and not after_run_id.strip():
            raise ValueError('after_run_id must not be blank')
        for name, value in (('after_sequence', after_sequence), ('wait_ms', wait_ms)):
            raw = request.query_params.get(name)
            if raw is not None and (not raw.isascii() or not raw.isdecimal()):
                raise ValueError(f'{name} must be a nonnegative integer')
        if after_sequence is not None and after_sequence < 0:
            raise ValueError('after_sequence must be a nonnegative integer')
        if not 0 <= wait_ms <= 2000:
            raise ValueError('wait_ms must be from 0 to 2000')
        deadline = time.monotonic() + wait_ms / 1000
        while True:
            frame = camera.frame(after_run_id, after_sequence)
            if frame is not None:
                return frame
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CameraUnavailable('No newer camera frame before the deadline')
            await anyio.sleep(min(.025, remaining))

    def snapshot_image(kind):
        async def snapshot(request: Request, after_run_id: str | None = None,
                           after_sequence: int | None = None, wait_ms: int = 1000):
            try:
                frame = await wait_frame(request, after_run_id, after_sequence, wait_ms)
                data, media_type = image(frame, kind)
                return Response(data, media_type=media_type,
                                headers={'X-Camera-Frame':frame_header(frame)})
            except Exception as error:
                return refusal(error)
        return snapshot

    for path, kind, media_type, summary in (
        ('snapshot.jpg', 'rgb', 'image/jpeg', 'Fresh RGB JPEG with matching frame metadata'),
        ('depth/snapshot.png', 'depth', 'image/png', 'Fresh lossless 16-bit depth PNG in millimetres; zero means unknown'),
        ('depth/preview.png', 'preview', 'image/png', 'Fresh colour depth preview; display only')):
        app.add_api_route('/api/camera/' + path, snapshot_image(kind), methods=['GET'],
                          tags=['Camera'], response_class=Response, summary=summary,
                          responses={200:{'content':{media_type:{}}}, **REFUSALS})

    @app.get('/api/camera/capture', tags=['Camera'], responses=REFUSALS,
             summary='Capture one RGB/depth pair as base64 images with shared metadata')
    async def capture(request: Request, after_run_id: str | None = None,
                      after_sequence: int | None = None, wait_ms: int = 1000):
        try:
            frame = await wait_frame(request, after_run_id, after_sequence, wait_ms)
            encode = lambda data:base64.b64encode(data).decode('ascii')
            return dict(metadata=frame.metadata(), rgb_jpeg=encode(frame.jpeg),
                        depth_png=encode(frame.depth.png) if frame.depth else None,
                        depth_preview_png=encode(frame.depth.preview_png) if frame.depth else None)
        except Exception as error:
            return refusal(error)

    def stream_image(kind):
        async def stream(request: Request):
            try:
                initial = camera.frame()
                if initial is None:
                    raise CameraUnavailable('No fresh camera frame')
                initial_part = part(initial, kind)
                initial_metadata = initial.metadata()
            except Exception as error:
                return refusal(error)

            async def frames():
                yield initial_part
                run_id, sequence = initial_metadata['run_id'], initial_metadata['sequence']
                while not await request.is_disconnected():
                    await anyio.sleep(.025)
                    try:
                        frame = camera.frame(run_id, sequence)
                        if frame is not None:
                            metadata = frame.metadata()
                            yield part(frame, kind)
                            run_id, sequence = metadata['run_id'], metadata['sequence']
                    except Exception:
                        break
                yield b'--frame--\r\n'

            return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame')
        return stream

    for path, kind, summary in (
        ('stream.mjpg', 'rgb', 'Live MJPEG; slow viewers skip older frames'),
        ('depth/stream', 'preview', 'Live colour depth PNG previews; display only')):
        app.add_api_route('/api/camera/' + path, stream_image(kind), methods=['GET'],
                          tags=['Camera'], response_class=StreamingResponse, summary=summary,
                          responses={200:{'content':{'multipart/x-mixed-replace':{}}}, **REFUSALS})
