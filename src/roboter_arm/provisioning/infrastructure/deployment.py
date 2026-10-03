"""Explicit manifest deployment with conflict checks, SHA256 readback and an app service restart."""
import base64
import hashlib
import json

from roboter_arm.provisioning.domain.manifest import check_manifest, select_files
from roboter_arm.shared.paths import ROOT


def manifest():
    return check_manifest(json.loads((ROOT / 'deploy.json').read_text()))


def entries(files=None, baseline=None):
    selected = select_files(manifest(), files)
    baseline = baseline or {}
    if not isinstance(baseline, dict):
        raise ValueError('Baseline must map paths to SHA256 hashes')
    result = []
    for name in selected:
        data = (ROOT / name).read_bytes()
        result.append(dict(path=name, content=base64.b64encode(data).decode(),
                           sha256=hashlib.sha256(data).hexdigest(), expected=baseline.get(name)))
    return result


def deployment(connection, files=None, baseline=None):
    selected = entries(files, baseline)
    header = ('import base64, hashlib, json, pathlib, subprocess, tempfile, time, os, urllib.request\n'
              f'root = pathlib.Path({connection.remote_dir!r})\n'
              f'entries = json.loads({json.dumps(selected)!r})\n'
              f'state_url = "http://127.0.0.1:{connection.app_port}/api/state"\n')
    remote = '''
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def app_state():
    with opener.open(state_url, timeout=5) as response:
        return json.load(response)
# Restarting an app that holds the arm would drop it limp.
try:
    powered = app_state()['outputs_off'] is False
except (OSError, ValueError, KeyError, TypeError):
    powered = False
if powered:
    raise SystemExit('Arm is powered; press Power off before deploying')
# Hashes the previous deploys wrote; a Pi file still matching them was not changed on the Pi.
record = root / '.deployed.json'
try:
    deployed = {} if record.is_symlink() else json.loads(record.read_text())
except (OSError, ValueError):
    deployed = {}
if not isinstance(deployed, dict):
    deployed = {}
for entry in entries:
    destination = root / entry['path']
    if not destination.resolve().is_relative_to(root.resolve()) or destination.is_symlink():
        raise SystemExit('Refusing destination outside project: ' + entry['path'])
    if destination.exists():
        actual = hashlib.sha256(destination.read_bytes()).hexdigest()
        if entry['path'].startswith('config/reference-arm/') and entry['path'].endswith('.json') and actual != entry['sha256']:
            raise SystemExit('Immutable reference evidence conflict: ' + entry['path'])
        if actual not in (entry['sha256'], entry['expected'], deployed.get(entry['path'])):
            raise SystemExit('Pi-side change; refusing overwrite: ' + entry['path'])
changed = False
for entry in entries:
    destination = root / entry['path']
    data = base64.b64decode(entry['content'])
    if hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise SystemExit('Payload digest mismatch')
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and destination.read_bytes() == data:
        print(entry['path'] + ' already matches ' + entry['sha256'])
        continue
    with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
        stream.write(data)
        temporary = stream.name
    os.replace(temporary, destination)
    changed = True
    actual = hashlib.sha256(destination.read_bytes()).hexdigest()
    if actual != entry['sha256']:
        raise SystemExit('Read-back mismatch: ' + entry['path'])
    print(entry['path'] + ' verified ' + actual)
deployed.update({entry['path']: entry['sha256'] for entry in entries})
with tempfile.NamedTemporaryFile('w', dir=root, delete=False) as stream:
    json.dump(deployed, stream, indent=2, sort_keys=True)
    temporary = stream.name
os.replace(temporary, record)
systemctl = ['systemctl', '--user']
environment = dict(os.environ, XDG_RUNTIME_DIR=f'/run/user/{os.getuid()}')
try:
    running = subprocess.run(systemctl + ['is-active', '--quiet', 'robot-arm.service'], env=environment).returncode == 0
except FileNotFoundError:
    running = False
if not running:
    print('App service not running; deployed files apply at its next start (scripts/service.py install or start)')
elif changed:
    if subprocess.run(systemctl + ['restart', 'robot-arm.service'], env=environment).returncode:
        raise SystemExit('App service restart failed; see scripts/service.py status')
    deadline = time.monotonic() + 10
    while True:
        try:
            app_state()
            break
        except (OSError, ValueError):
            if time.monotonic() > deadline:
                raise SystemExit('App did not answer after restart; see scripts/service.py status')
            time.sleep(.2)
    print('App service restarted')
'''
    return (header + remote).encode()
