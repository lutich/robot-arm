"""Validated calibration evidence files -> JointLimits; the control side of calibration."""
import json
from pathlib import Path

from roboter_arm.calibration.domain.evidence import validate
from roboter_arm.control.domain.motion import JointLimit, Trajectory


def load_limits(paths, speeds):
    """Select explicit validated evidence files; never choose 'latest' implicitly."""
    limits = {}
    for path in paths:
        evidence = json.loads(Path(path).read_text())
        if (evidence.get('schema_version') != 1 or evidence.get('kind') != 'validated_limits'
                or evidence.get('units') != 'PCA9685_counts'
                or evidence.get('nominal_frequency_hz') != 60
                or evidence.get('position_feedback') is not False):
            raise ValueError('General motion requires validated count/zero/direction evidence')
        validate(channel=evidence['channel'], joint=evidence['observed_joint'],
                 low=evidence['low'], high=evidence['high'], kind=evidence['kind'],
                 provenance=evidence['provenance'], direction=evidence['increasing_counts_direction'],
                 zero_count=evidence['zero_count'], zero_reference=evidence['zero_reference'])
        channel = evidence['channel']
        if channel in limits or channel not in speeds:
            raise ValueError('Select one evidence record and explicit speed per channel')
        limits[channel] = JointLimit(evidence['low'], evidence['high'], speeds[channel],
                                    f'{path}: {evidence["provenance"]}', 'validated')
    if not limits or limits.keys() != speeds.keys():
        raise ValueError('Evidence and speed channels must match')
    Trajectory({c:lim.low for c, lim in limits.items()},
               {c:lim.high for c, lim in limits.items()}, limits)
    return limits
