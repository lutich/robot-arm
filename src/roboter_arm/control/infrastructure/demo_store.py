"""Scoped, immutable saved command sequences; no hardware access."""
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import uuid


class DemoStore:
    def __init__(self, directory, scope, bounds):
        self.directory = Path(directory) / scope
        self.scope, self.bounds = scope, dict(bounds)

    def _positions(self, positions):
        if not isinstance(positions, list) or not positions:
            raise ValueError('A demo needs at least one recorded position')
        result = []
        for position in positions:
            if not isinstance(position, dict) or not isinstance(position.get('name'), str) or not position['name'].strip():
                raise ValueError('Every position needs a name')
            counts = position.get('counts')
            if (not isinstance(counts, dict)
                    or not all(type(c) is int for c in counts) and not all(type(c) is str for c in counts)
                    or set(counts) not in (set(self.bounds), {str(c) for c in self.bounds})):
                raise ValueError('Every position needs all six joint commands')
            values = {c:counts[c] if c in counts else counts[str(c)] for c in self.bounds}
            if any(type(v) is not int or not self.bounds[c][0] <= v <= self.bounds[c][1] for c,v in values.items()):
                raise ValueError('Saved commands must stay inside the historical outer bounds')
            result.append(dict(name=position['name'].strip(), counts=values))
        return result

    def save(self, name, positions):
        if not isinstance(name, str) or not name.strip():
            raise ValueError('A demo needs a name')
        record = dict(schema_version=1, recorded_at=datetime.now(timezone.utc).isoformat(),
            name=name.strip(), positions=self._positions(positions), scope=self.scope,
            units='PCA9685_counts', nominal_frequency_hz=60,
            validated_limits=False, position_feedback=False)
        self.directory.mkdir(parents=True, exist_ok=True)
        destination = self.directory / f'{uuid.uuid4().hex}.json'
        with destination.open('x') as stream:
            stream.write(json.dumps(record, indent=2) + '\n')
        return destination.name

    def load(self, filename):
        if not isinstance(filename, str) or re.fullmatch(r'[0-9a-f]{32}\.json', filename) is None:
            raise ValueError('Select a saved demo filename')
        try:
            record = json.loads((self.directory / filename).read_text())
        except (OSError, ValueError) as error:
            raise ValueError('Saved demo is unavailable or invalid') from error
        if (not isinstance(record, dict) or type(record.get('schema_version')) is not int or record.get('schema_version') != 1
                or record.get('scope') != self.scope or record.get('units') != 'PCA9685_counts'
                or record.get('nominal_frequency_hz') != 60
                or record.get('position_feedback') is not False or record.get('validated_limits') is not False
                or not isinstance(record.get('name'), str) or not record['name'].strip()):
            raise ValueError('Saved demo scope or command metadata is invalid')
        return dict(name=record['name'].strip(), positions=self._positions(record.get('positions')))

    def saved(self):
        if not self.directory.exists():
            return []
        result = []
        for path in sorted(self.directory.glob('*.json')):
            try:
                record = self.load(path.name)
            except ValueError:
                continue
            result.append(dict(filename=path.name, name=record['name'], positions=len(record['positions'])))
        return result
