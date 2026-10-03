"""Append immutable calibration evidence records as JSON; never command hardware."""
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

from roboter_arm.calibration.domain.evidence import validate


def record(directory, *, channel, joint, low, high, kind, provenance,
           direction=None, zero_count=None, zero_reference=None):
    validate(channel=channel, joint=joint, low=low, high=high, kind=kind,
             provenance=provenance, direction=direction, zero_count=zero_count,
             zero_reference=zero_reference)
    evidence = dict(schema_version=1, recorded_at=datetime.now(timezone.utc).isoformat(),
                    channel=channel, observed_joint=joint, kind=kind, low=low, high=high,
                    increasing_counts_direction=direction, zero_count=zero_count,
                    zero_reference=zero_reference, provenance=provenance,
                    units='PCA9685_counts', nominal_frequency_hz=60,
                    position_feedback=False)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f'{uuid.uuid4().hex}.json'
    with destination.open('x') as stream:
        stream.write(json.dumps(evidence, indent=2) + '\n')
    return destination
