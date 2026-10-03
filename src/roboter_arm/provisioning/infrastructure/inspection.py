"""Build a passive Pi inspection payload: files, runtime and optional HTTP GETs."""
import json

from roboter_arm.provisioning.infrastructure.deployment import entries


def inspection(connection, *, app=False):
    files = [{key: entry[key] for key in ('path', 'sha256')} for entry in entries()]
    header = ('import pathlib, json\n'
              f'root = pathlib.Path({connection.remote_dir!r})\n'
              f'files = json.loads({json.dumps(files)!r})\n'
              f'app_port = {connection.app_port!r}\n'
              f'check_app = {app!r}\n')
    body = r'''import hashlib, os, platform, shutil, subprocess, sys, urllib.request
uv = shutil.which('uv')
if uv is None and (pathlib.Path.home() / '.local/bin/uv').is_file():
    uv = str(pathlib.Path.home() / '.local/bin/uv')
report = dict(machine=platform.machine(), python=platform.python_version(),
              uv_available=uv is not None,
              i2c_device=pathlib.Path('/dev/i2c-1').exists(),
              i2c_access=os.access('/dev/i2c-1', os.R_OK | os.W_OK), files={})
for entry in files:
    path = root / entry['path']
    report['files'][entry['path']] = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
report['files_match'] = all(report['files'][entry['path']] == entry['sha256'] for entry in files)
python = root / '.venv-runtime/bin/python'
report['runtime'] = dict(exists=python.is_file(), valid=False)
if python.is_file() and uv is not None:
    versions = subprocess.run([str(python), '-c',
        "import importlib.metadata as m, json, sys; print(json.dumps(dict(python=list(sys.version_info[:2]), packages={d.metadata['Name'].lower().replace('_','-'):d.version for d in m.distributions()})))"],
        capture_output=True, text=True, timeout=30)
    check = subprocess.run([uv, 'pip', 'check', '--python', str(python), '--no-python-downloads'], capture_output=True, text=True, timeout=30)
    lock = root / 'requirements/pi-py313.lock'
    if versions.returncode == 0 and lock.is_file():
        observed = json.loads(versions.stdout)
        expected = dict(line.split()[0].lower().replace('_','-').split('==') for line in lock.read_text().splitlines() if line and not line.startswith('#'))
        report['runtime']['valid'] = check.returncode == 0 and observed['python'] == [3, 13] and all(observed['packages'].get(name) == version for name, version in expected.items())
if check_app:
    expected = {entry['path']:entry['sha256'] for entry in files}
    report['app'] = dict(reachable=False, assets_match=False)
    try:
        base = 'http://127.0.0.1:' + str(app_port)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(base + '/api/state', timeout=5) as response:
            state = json.load(response)
        matched = True
        for route, name in (('/', 'manual_control.html'), ('/manual_control.css', 'manual_control.css'),
                            ('/manual_control.js', 'manual_control.js'), ('/park-illustration-v2.png', 'park-illustration-v2.png'),
                            ('/home-illustration.png', 'home-illustration.png')):
            with opener.open(base + route, timeout=5) as response:
                content = response.read()
            matched = matched and hashlib.sha256(content).hexdigest() == expected['web/' + name]
        report['app'] = dict(reachable=True, assets_match=matched, state=state)
    except (OSError, ValueError):
        report['app']['error'] = 'App unavailable or response invalid'
print(json.dumps(report, indent=2))
'''
    return (header + body).encode()
