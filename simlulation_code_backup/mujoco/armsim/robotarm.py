"""Bridge to the robot-arm checkout: its src/ goes on the import path, nothing is copied.

Set ROBOT_ARM_PATH to the checkout; the default is ../robot-arm next to this repository.
"""
import os
from pathlib import Path
import sys

from armsim.config import ROOT

DEFAULT = ROOT.parent / 'robot-arm'


def path():
    candidate = Path(os.environ.get('ROBOT_ARM_PATH', DEFAULT)).expanduser().resolve()
    if not (candidate / 'src' / 'roboter_arm').is_dir():
        raise RuntimeError(f'robot-arm checkout not found at {candidate}; set ROBOT_ARM_PATH')
    return candidate


def install():
    """Put robot-arm's src on sys.path once and return the checkout path."""
    checkout = path()
    src = str(checkout / 'src')
    if src not in sys.path:
        sys.path.insert(0, src)
    return checkout


def commit():
    """Short commit hash of the checkout, for logs and the compatibility note."""
    git = path() / '.git'
    try:
        head = (git / 'HEAD').read_text().strip()
        if head.startswith('ref: '):
            head = (git / head[5:]).read_text().strip()
        return head[:12]
    except OSError:
        return 'unknown'


install()
from roboter_arm.control.application.session import HOME, PARK, SPEED, Session  # noqa: E402
from roboter_arm.control.domain.motion import JointLimit, Trajectory  # noqa: E402
from roboter_arm.control.domain.ports import ServoDriver  # noqa: E402
from roboter_arm.control.infrastructure.demo_store import DemoStore  # noqa: E402
from roboter_arm.control.infrastructure.pose_store import PoseStore  # noqa: E402
from roboter_arm.shared.joints import JOINTS  # noqa: E402

BOUNDS = {channel: (low, high) for channel, (_, low, high) in JOINTS.items()}

__all__ = ['HOME', 'PARK', 'SPEED', 'Session', 'JointLimit', 'Trajectory', 'ServoDriver',
           'DemoStore', 'PoseStore', 'JOINTS', 'BOUNDS', 'path', 'install', 'commit']
