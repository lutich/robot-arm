"""Replay of recorded trajectories: robot-arm demo records and the sandbox's own pose logs."""
import json
from pathlib import Path

from armsim.robotarm import BOUNDS

CHANNELS = range(6)


def read_demo_record(path):
    """A robot-arm demo record as saved on the Pi, checked for shape and bounds."""
    path = Path(path)
    try:
        record = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        raise ValueError(f'Cannot read demo record {path}: {error}') from None
    if not isinstance(record, dict) or record.get('schema_version') != 1 or record.get('units') != 'PCA9685_counts' \
            or record.get('nominal_frequency_hz') != 60 or not isinstance(record.get('positions'), list) or not record['positions']:
        raise ValueError(f'{path.name} is not a robot-arm demo record')
    positions = []
    for position in record['positions']:
        counts = position.get('counts', {}) if isinstance(position, dict) else {}
        values = {}
        for channel in CHANNELS:
            value = counts.get(channel, counts.get(str(channel)))
            if type(value) is not int or not BOUNDS[channel][0] <= value <= BOUNDS[channel][1]:
                raise ValueError(f'{path.name}: position {position.get("name")!r} has channel {channel} outside robot-arm bounds')
            values[channel] = value
        positions.append(dict(name=str(position.get('name') or f'Position {len(positions) + 1}'), counts=values))
    return dict(name=str(record.get('name') or path.stem), scope=record.get('scope'), positions=positions)


def import_demo(path, demo_store):
    """Copy a demo record into the sandbox's own scoped store and return its filename.

    The copy carries the sandbox scope ('synthetic'); the source file and scope stay in the name,
    so the provenance remains visible in the app. Records never flow the other way.
    """
    record = read_demo_record(path)
    name = f'{record["name"]} (imported from {Path(path).name}, {record.get("scope") or "unknown scope"})'
    return demo_store.save(name, record['positions'])


def run_demo(client, filename, timeout=600.0):
    """Load a stored demo and run it through the API's own playback."""
    client.demo('load', filename=filename)
    client.demo('run')
    return client.wait_idle(timeout)


def read_pose_log(path):
    entries = []
    with Path(path).open() as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            counts = entry.get('counts')
            if not isinstance(counts, list) or len(counts) != 6 or any(type(c) is not int for c in counts):
                raise ValueError(f'Pose log entry {entry.get("label")!r} needs six integer counts')
            entries.append(entry)
    if not entries:
        raise ValueError('Pose log is empty')
    return entries


def replay_pose_log(client, entries, speed=None):
    """Send every logged pose in order; the same log drives sandbox and real arm."""
    for entry in entries:
        counts = {c: entry['counts'][c] for c in CHANNELS}
        client.move_pose(counts, speed=speed or entry.get('speed'))
    return client.wait_idle()
