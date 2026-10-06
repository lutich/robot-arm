"""One camera owner and one latest-frame cache shared by all HTTP clients."""
from copy import deepcopy
from datetime import datetime, timezone
import math
import threading
import time
from uuid import uuid4

from roboter_arm.control.domain.camera import (
    CameraConflict, CameraFrame, CameraSource, CameraUnavailable, DepthFrame, DEPTH_PROFILE,
    PROFILE, defaults, validate_changes)

FRESH_SECONDS = 1
STALL_SECONDS = 5


def utc_at(seconds):
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


class CameraService:
    def __init__(self, session, source: CameraSource | None = None, *, clock=time.monotonic):
        self.session, self.source, self.clock = session, source, clock
        self.lock = threading.RLock()
        self.finished = threading.Event()
        self.thread = None
        self.run_id = None
        self.revision = 0
        self.requested, self.caps = {}, {}
        self.latest = None
        self.connected = self.pending = False
        self.queued = None
        self.application_status = 'starting' if source else 'disabled'
        self.last_error = None

    def start(self):
        if self.source is not None and self.thread is None:
            self.thread = threading.Thread(target=self._run, name='camera', daemon=False)
            self.thread.start()

    def close(self):
        self.finished.set()
        if self.thread is not None:
            # Native SDK cleanup must finish before Python starts finalizing.
            self.thread.join()

    def _fresh(self):
        if self.latest is None:
            return False
        acquired = self.latest.acquired_monotonic
        if self.latest.depth is not None:
            acquired = min(acquired, self.latest.depth.acquired_monotonic)
        return self.clock()-acquired <= FRESH_SECONDS

    def status(self):
        with self.session.lock, self.lock:
            metadata = self.latest.metadata(self.clock()) if self.latest else None
            available = self.connected and self._fresh()
            return dict(enabled=self.source is not None, source=self.source.kind if self.source else 'disabled',
                        available=available, run_id=self.run_id,
                        settings_revision=self.revision, requested=deepcopy(self.requested),
                        application_status=self.application_status, pending=self.pending,
                        locked=self.pending or self.session.busy, capabilities=deepcopy(self.caps),
                        profile=dict(PROFILE), frame_age_ms=metadata['age_ms'] if metadata else None,
                        depth_profile=dict(DEPTH_PROFILE),
                        depth_available=available and self.latest.depth is not None,
                        latest_frame=metadata, last_error=self.last_error, measurement_ready=False)

    def settings(self, *, expected_run_id, expected_revision, **changes):
        # Motion admission and camera admission share this short lock. No SDK IO here.
        with self.session.lock, self.lock:
            if not self.connected or (not self.pending and not self._fresh()):
                raise CameraUnavailable(self.last_error or 'Camera has no fresh frame')
            if expected_run_id != self.run_id or expected_revision != self.revision:
                raise CameraConflict('Camera changed; refresh status before applying settings')
            if self.pending or self.session.busy:
                raise CameraConflict('Wait for movement or the pending camera change')
            changes = validate_changes(changes, self.caps)
            self.requested.update(changes)
            self.revision += 1
            self.queued = changes
            self.pending = self.session.camera_pending = True
            self.latest = None
            self.application_status, self.last_error = 'queued', None
            return dict(run_id=self.run_id, settings_revision=self.revision,
                        requested=deepcopy(self.requested), application_status='queued')

    def frame(self, after_run_id=None, after_sequence=None):
        with self.lock:
            if (after_run_id is None) != (after_sequence is None):
                raise ValueError('after_run_id and after_sequence must be supplied together')
            if after_run_id is not None and after_run_id != self.run_id:
                raise CameraConflict('Camera restarted; refresh the frame cursor')
            if not self.connected:
                raise CameraUnavailable(self.last_error or 'Camera is disabled or starting')
            if self.latest is None and self.pending:
                return None
            if not self._fresh():
                raise CameraUnavailable('Camera frame is stale or unavailable')
            if after_sequence is not None and self.latest.sequence <= after_sequence:
                return None
            return self.latest

    def _release_change(self, revision=None):
        with self.session.lock, self.lock:
            if revision is not None and (self.revision != revision or self.queued is not None):
                return
            self.pending = self.session.camera_pending = False

    def _run(self):
        while not self.finished.is_set():
            try:
                caps = self.source.open()
                requested = defaults(caps)
                self.source.apply(requested)
                cutoff = self.clock()
                with self.lock:
                    self.caps, self.requested = caps, requested
                    self.run_id, self.revision = uuid4().hex, 0
                    self.connected = True
                    self.application_status, self.last_error = 'sent', None
                sent_revision = 0
                last_fresh, last_received, sequence = self.clock(), None, 0
                while not self.finished.is_set():
                    with self.lock:
                        changes, self.queued = self.queued, None
                        queued_revision = self.revision
                    if changes is not None:
                        self.source.apply(changes)
                        cutoff = self.clock()
                        sent_revision = queued_revision
                        with self.lock:
                            self.application_status = 'sent'
                    sample = self.source.read()
                    now = self.clock()
                    if sample is not None:
                        ages = [sample.age] + ([sample.depth.age] if sample.depth else [])
                        if any(not math.isfinite(age) or age < 0 for age in ages):
                            raise ValueError('Invalid camera acquisition timestamp')
                        acquired = now-sample.age
                        if now-max(ages) >= cutoff and max(ages) <= FRESH_SECONDS:
                            if not sample.jpeg.startswith(b'\xff\xd8') or not sample.jpeg.endswith(b'\xff\xd9'):
                                raise ValueError('Camera output is not a complete JPEG')
                            depth = None
                            if sample.depth is not None:
                                observed_depth = sample.depth.observed
                                skew = observed_depth['sync_delta_ms']
                                if not math.isfinite(skew) or abs(skew) > DEPTH_PROFILE['sync_tolerance_ms']:
                                    raise ValueError('RGB and depth acquisition times do not match')
                                depth = DepthFrame(sample.depth.png, sample.depth.preview_png,
                                    utc_at(time.time()-sample.depth.age), now-sample.depth.age, observed_depth)
                            sequence += 1
                            observed = dict(sample.observed, delivered_fps=round(1/(now-last_received), 2) if last_received and now > last_received else None)
                            wall_time = time.time()
                            with self.lock:
                                # A request can arrive while read() is outside the lock.
                                # Never relabel that earlier frame with the new request.
                                if self.revision != sent_revision or self.queued is not None:
                                    continue
                                self.latest = CameraFrame(sample.jpeg, self.run_id, sequence, self.revision,
                                    utc_at(wall_time-sample.age), utc_at(wall_time), acquired, observed, depth)
                            last_fresh, last_received = now, now
                            if self.pending:
                                self._release_change(sent_revision)
                    if now-last_fresh > STALL_SECONDS:
                        raise CameraUnavailable('No fresh camera frame for five seconds')
                    self.finished.wait(.01)
            except Exception as error:
                with self.lock:
                    self.connected, self.latest, self.queued = False, None, None
                    self.application_status = 'failed'
                    self.last_error = f'{type(error).__name__}: {error}'
            finally:
                self._release_change()
                try:
                    self.source.close()
                except Exception:
                    pass
                with self.lock:
                    self.connected, self.latest = False, None
            self.finished.wait(2)
