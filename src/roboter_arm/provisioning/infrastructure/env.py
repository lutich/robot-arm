"""Read optional secrets from the environment or the project .env file as data."""
import os
from pathlib import Path
import shlex

from roboter_arm.shared.paths import ROOT


def read_password(path=ROOT / '.env'):
    return read_env('PI_PASS', path)


def read_env(name, path=ROOT / '.env'):
    if name in os.environ:
        return os.environ[name]
    if not Path(path).is_file():
        return None
    result = None
    for line in Path(path).read_text().splitlines():
        key, separator, value = line.strip().partition('=')
        if separator and key.strip() == name:
            value = value.strip()
            if value.startswith(('"', "'")):
                parts = shlex.split(value, comments=True)
                if len(parts) != 1:
                    raise ValueError(f'Invalid quoted {name} value')
                value = parts[0]
            result = value
    return result
