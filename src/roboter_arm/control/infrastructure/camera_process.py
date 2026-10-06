"""Keep native OAK failures outside the controller process; one bounded IPC request at a time."""
from dataclasses import replace
import multiprocessing
import pickle
import signal
import socket
import struct
import time

from roboter_arm.control.domain.camera import CameraUnavailable
from roboter_arm.control.infrastructure.oak_camera import OakCamera


def _timeout(channel, deadline):
    remaining = None if deadline is None else deadline-time.monotonic()
    if remaining is not None and remaining <= 0:
        raise TimeoutError('Camera IPC deadline exceeded')
    channel.settimeout(remaining)


def _send(channel, value, deadline=None):
    # This socket is private to the parent and its spawned worker, never an HTTP input.
    payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
    _timeout(channel, deadline)
    channel.sendall(struct.pack('!I', len(payload))+payload)


def _receive(channel, deadline=None):
    def exact(size):
        result = bytearray()
        while len(result) < size:
            _timeout(channel, deadline)
            part = channel.recv(size-len(result))
            if not part:
                raise EOFError('Camera worker disconnected')
            result.extend(part)
        return result
    size, = struct.unpack('!I', exact(4))
    return pickle.loads(exact(size))


def _worker(channel, factory):
    # The parent handles Ctrl-C and asks for cleanup; SIGTERM remains a forced-stop option.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    source = None
    try:
        source = factory()
        while True:
            method, args = _receive(channel)
            if method == 'close':
                break
            try:
                result = getattr(source, method)(*args)
                reply = (True, result, time.monotonic())
            except Exception as error:
                reply = (False, f'{type(error).__name__}: {error}', time.monotonic())
            _send(channel, reply)
    except (EOFError, OSError):
        pass
    finally:
        try:
            if source is not None:
                source.close()
        finally:
            channel.close()


class CameraProcess:
    kind = 'oak'

    def __init__(self, factory=OakCamera, *, timeout=5, close_timeout=3):
        self.factory, self.timeout, self.close_timeout = factory, timeout, close_timeout
        self.process = self.channel = None

    def open(self):
        self.close()
        self.channel, child = socket.socketpair()
        # Never fork the HTTP threads, controller locks or a prepared servo driver.
        self.process = multiprocessing.get_context('spawn').Process(
            target=_worker, args=(child, self.factory), name='oak-camera')
        try:
            self.process.start()
        except Exception:
            self.channel.close()
            self.process.close()
            self.process = self.channel = None
            raise
        finally:
            child.close()
        return self._call('open')[0]

    def _call(self, method, *args):
        deadline = time.monotonic()+self.timeout
        try:
            _send(self.channel, (method, args), deadline)
            ok, result, sampled_at = _receive(self.channel, deadline)
        except (EOFError, OSError) as error:
            self.process.join(0)
            detail = ('deadline exceeded' if isinstance(error, TimeoutError)
                      else f'connection lost (exit code {self.process.exitcode})')
            raise CameraUnavailable(f'Camera worker {detail} during {method}') from error
        if not ok:
            raise CameraUnavailable(f'Camera worker {method} failed: {result}')
        return result, time.monotonic()-sampled_at

    def apply(self, changes):
        self._call('apply', changes)

    def read(self):
        sample, transport_age = self._call('read')
        if sample is None:
            return None
        depth = replace(sample.depth, age=sample.depth.age+transport_age) if sample.depth else None
        return replace(sample, age=sample.age+transport_age, depth=depth)

    def close(self):
        if self.process is None:
            return
        try:
            try:
                if self.process.is_alive():
                    _send(self.channel, ('close', ()), time.monotonic()+.1)
            except OSError:
                pass
        finally:
            self.channel.close()
        self.process.join(self.close_timeout)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(.5)
        if self.process.is_alive():
            raise CameraUnavailable('Camera worker did not exit after forced stop')
        self.process.close()
        self.process = self.channel = None
