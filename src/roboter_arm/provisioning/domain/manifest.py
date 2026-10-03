"""Which repository paths may ever reach the Pi: public operational files only."""
from pathlib import PurePosixPath

PUBLIC = ('scripts', 'src', 'web', 'docs', 'config', 'runtime', 'requirements', 'README.md', 'CONTRIBUTING.md', 'LICENSE', 'pyproject.toml', 'uv.lock')


def check_manifest(files):
    if not isinstance(files, list) or not files or len(files) != len(set(files)):
        raise ValueError('Deployment manifest must contain distinct paths')
    for name in files:
        path = PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or path.parts[0] not in PUBLIC:
            raise ValueError('Manifest path is outside the public operational files')
    return files


def select_files(allowed, files=None):
    """The whole manifest, or a distinct subset of it."""
    selected = allowed if files is None else files
    if not selected or len(selected) != len(set(selected)) or set(selected) - set(allowed):
        raise ValueError('Select distinct files from deploy.json')
    return selected
