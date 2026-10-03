"""Scoped PARK/HOME definitions: immutable archive records plus an atomically activated copy."""
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

from roboter_arm.shared.joints import JOINTS


class PoseStore:
    def __init__(self, directory, scope):
        self.directory = Path(directory) / scope
        self.scope = scope

    def load(self, name):
        """Saved counts for name, or None if never saved; invalid records raise ValueError."""
        path = self.directory / f'{name}.json'
        try:
            record = json.loads(path.read_text())
        except FileNotFoundError:
            return None
        except ValueError as error:
            raise ValueError(f'Saved {name} definition is invalid: {path}') from error
        if (not isinstance(record, dict) or type(record.get('schema_version')) is not int
                or record.get('schema_version') != 1 or record.get('scope') != self.scope
                or record.get('name') != name or record.get('units') != 'PCA9685_counts'
                or record.get('nominal_frequency_hz') != 60 or record.get('observed') is not True
                or record.get('validated_limits') is not False or record.get('position_feedback') is not False
                or not isinstance(record.get('provenance'), str) or not record['provenance'].strip()):
            raise ValueError(f'Saved {name} definition metadata is invalid: {path}')
        counts = record.get('counts')
        if (not isinstance(counts, dict) or set(counts) != {str(c) for c in JOINTS}
                or any(type(counts[str(c)]) is not int or not low <= counts[str(c)] <= high
                       for c, (_, low, high) in JOINTS.items())):
            raise ValueError(f'Saved {name} definition needs six counts inside historical bounds: {path}')
        return {c:counts[str(c)] for c in JOINTS}

    def save(self, name, counts, *, provenance):
        """Archive a new record, then activate it as name.json; returns the archive filename."""
        record = dict(schema_version=1, recorded_at=datetime.now(timezone.utc).isoformat(),
            name=name, counts=counts, units='PCA9685_counts',
            nominal_frequency_hz=60, observed=True, provenance=provenance,
            scope=self.scope, validated_limits=False, position_feedback=False)
        self.directory.mkdir(parents=True, exist_ok=True)
        destination = self.directory / f'{uuid.uuid4().hex}.json'
        content = json.dumps(record, indent=2) + '\n'
        with destination.open('x') as stream:
            stream.write(content)
        temporary = destination.with_suffix('.tmp')
        with temporary.open('x') as stream:
            stream.write(content)
        temporary.replace(self.directory / f'{name}.json')
        return destination.name
