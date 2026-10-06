"""Camera settings policy and cached frame values; no camera SDK or HTTP."""
from copy import deepcopy
from dataclasses import dataclass
import time
from typing import Protocol


class CameraConflict(Exception):
    """The caller's camera version or admission state no longer matches."""


class CameraUnavailable(Exception):
    """No fresh camera output can currently be used."""


PROFILE = dict(width=1280, height=720, fps=5, jpeg_quality=85,
               resize_mode='crop', undistortion=True)
DEPTH_PROFILE = dict(width=1280, height=720, fps=5, unit='mm', invalid_value=0,
                     aligned_to='rgb', sync_tolerance_ms=20,
                     preview_near_mm=200, preview_far_mm=3000)


def capabilities(has_autofocus, *, has_depth=False):
    """IMX214 adapter policy limits, not measured sensor guarantees."""
    controls = dict(
        exposure=dict(modes=['auto', 'manual'], time_us=dict(min=100, max=60000),
                      iso=dict(min=100, max=1600)),
        white_balance=dict(modes=['auto', 'manual'], temperature_k=dict(min=1000, max=12000)),
        anti_banding=dict(modes=['off', '50hz', '60hz', 'auto']))
    if has_autofocus:
        controls['focus'] = dict(modes=['once', 'manual'], lens_position=dict(min=0, max=255))
    return dict(has_autofocus=has_autofocus, has_depth=has_depth, controls=controls,
                range_source='application_policy', hardware_validated=False)


def defaults(caps):
    result = dict(exposure=dict(mode='auto'), white_balance=dict(mode='auto'), anti_banding='50hz')
    if caps['has_autofocus']:
        result['focus'] = dict(mode='once')
    return result


def validate_changes(changes, caps):
    """Groups replace whole groups; stale manual fields in Auto are refused."""
    if not changes:
        raise ValueError('Supply at least one camera setting')
    controls = caps['controls']
    for name, group in changes.items():
        if name not in controls:
            raise ValueError(f'Unsupported camera control: {name}')
        if name == 'anti_banding':
            if group not in controls[name]['modes']:
                raise ValueError('Unsupported anti_banding mode')
            continue
        if not isinstance(group, dict) or group.get('mode') not in controls[name]['modes']:
            raise ValueError(f'Unsupported {name} mode')
        fields = set(controls[name]) - {'modes'} if group['mode'] == 'manual' else set()
        if set(group) != {'mode', *fields}:
            raise ValueError(f'{name}: supply only the fields required by its mode')
        for field in fields:
            value, bounds = group[field], controls[name][field]
            if type(value) is not int or not bounds['min'] <= value <= bounds['max']:
                raise ValueError(f'{name}.{field} must be an integer from {bounds["min"]} to {bounds["max"]}')
    return deepcopy(changes)


@dataclass(frozen=True)
class DepthSample:
    png: bytes
    preview_png: bytes
    observed: dict
    age: float


@dataclass(frozen=True)
class CameraSample:
    jpeg: bytes
    observed: dict
    age: float
    depth: DepthSample | None = None


@dataclass(frozen=True)
class DepthFrame:
    png: bytes
    preview_png: bytes
    captured_at: str
    acquired_monotonic: float
    observed: dict

    def metadata(self, now):
        return dict(self.observed, captured_at=self.captured_at,
                    age_ms=round(max(0, now-self.acquired_monotonic)*1000, 1))


@dataclass(frozen=True)
class CameraFrame:
    jpeg: bytes
    run_id: str
    sequence: int
    settings_revision: int
    captured_at: str
    received_at: str
    acquired_monotonic: float
    observed: dict
    depth: DepthFrame | None = None

    def metadata(self, now=None):
        now = time.monotonic() if now is None else now
        result = dict(self.observed, run_id=self.run_id, sequence=self.sequence,
                    settings_revision=self.settings_revision, captured_at=self.captured_at,
                    received_at=self.received_at,
                    age_ms=round(max(0, now-self.acquired_monotonic)*1000, 1),
                    settings_verified=False, measurement_ready=False)
        if self.depth is not None:
            result['depth'] = self.depth.metadata(now)
        return result


class CameraSource(Protocol):
    kind: str

    def open(self) -> dict: ...
    def apply(self, changes: dict) -> None: ...
    def read(self) -> CameraSample | None: ...
    def close(self) -> None: ...
