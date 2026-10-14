"""Configuration files are the single source of truth for geometry, calibration and scene."""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config'


def load_robot(path=None):
    return _load(path or CONFIG / 'robot.yaml')


def load_scene(path=None):
    return _load(path or CONFIG / 'scene.yaml')


def _load(path):
    with Path(path).open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a mapping')
    return data
