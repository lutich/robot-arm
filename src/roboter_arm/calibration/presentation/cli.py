"""Append immutable calibration evidence; never command hardware."""
import argparse
from pathlib import Path

from roboter_arm.calibration.infrastructure.evidence_store import record
from roboter_arm.shared.paths import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', type=int, required=True)
    parser.add_argument('--joint', required=True)
    parser.add_argument('--low', type=int, required=True)
    parser.add_argument('--high', type=int, required=True)
    parser.add_argument('--kind', choices=('observation', 'validated_limits'), required=True)
    parser.add_argument('--provenance', required=True, help='Who observed what, when; evidence path')
    parser.add_argument('--direction', help='Physical movement when counts increase')
    parser.add_argument('--zero-count', type=int)
    parser.add_argument('--zero-reference')
    parser.add_argument('--directory', type=Path, default=ROOT / 'artifacts/calibration')
    args = vars(parser.parse_args())
    try:
        print(record(**args))
    except ValueError as error:
        parser.error(str(error))
