"""Connection, setup, deployment, the app service and passive checks."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from roboter_arm.provisioning.domain.report import report_valid
from roboter_arm.provisioning.infrastructure.deployment import deployment
from roboter_arm.provisioning.infrastructure.inspection import inspection
from roboter_arm.provisioning.infrastructure.remote import (
    INSTALL_COMMAND, WIFI_COMMAND, install_payload, service_command, setup_command, wifi_payload)
from roboter_arm.provisioning.infrastructure.ssh import load_connection, setup_connection, ssh


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='Connection JSON; default .pi.json in this checkout')
    commands = parser.add_subparsers(dest='action', required=True)
    connect = commands.add_parser('setup-connection', help='Pin a verified host key and test SSH')
    connect.add_argument('--fingerprint', help='Ed25519 SHA256 fingerprint from an independent trusted source')
    setup = commands.add_parser('setup-pi', help='Check prerequisites and install/reuse the pinned runtime')
    setup.add_argument('--install-system', action='store_true', help='Explicitly install OS prerequisites using sudo -n')
    wifi = commands.add_parser('setup-wifi', help='Save the .env WIFI_SSID/WIFI_PASS profile on the Pi using sudo')
    wifi.add_argument('--connect', action='store_true', help='Also switch to it now; drops an SSH session over Wi-Fi')
    deploy = commands.add_parser('deploy', help='Transfer only reviewed manifest files, with conflict checks')
    deploy.add_argument('--files', nargs='+', help='Subset of deploy.json; default is the full manifest')
    deploy.add_argument('--baseline', type=Path, help='Reviewed remote SHA256 snapshot permitting known changes')
    service = commands.add_parser('service', help='Install, start, stop, restart or show the Pi app service')
    service.add_argument('service_action', choices=('install', 'start', 'stop', 'restart', 'status'),
                         help='install enables it at boot (sudo once); stop and restart switch PWM off')
    service.add_argument('--camera', choices=('oak',), help='Opt in to OAK acquisition with service install')
    check = commands.add_parser('check', help='Read-only connectivity, deployed files and runtime inspection')
    check.add_argument('--app', action='store_true', help='Also GET app state and verify served frontend assets')
    check.add_argument('--snapshot', type=Path, help='Save remote file hashes for review; does not require deployment to match')
    for command in (connect, setup, wifi, deploy, service, check):
        command.add_argument('--config', type=Path, default=argparse.SUPPRESS, help='Override connection JSON')
    args = parser.parse_args(argv)
    if args.action == 'service' and args.camera and args.service_action != 'install':
        parser.error('--camera is only supported with service install')
    try:
        connection = load_connection(args.config)
        if args.action == 'setup-connection':
            result = setup_connection(connection, args.fingerprint)
        elif args.action == 'setup-pi':
            result = ssh(connection, setup_command(connection, install_system=args.install_system), timeout=600)
        elif args.action == 'setup-wifi':
            result = ssh(connection, WIFI_COMMAND, wifi_payload(connect=args.connect))
        elif args.action == 'deploy':
            baseline = json.loads(args.baseline.read_text()) if args.baseline else None
            result = ssh(connection, 'python3 -', deployment(connection, args.files, baseline), timeout=180)
        elif args.action == 'service':
            if args.service_action == 'install':
                result = ssh(connection, INSTALL_COMMAND, install_payload(connection, camera=args.camera))
            else:
                result = ssh(connection, service_command(args.service_action))
            if result.returncode == 0 and args.service_action != 'stop':
                result.stdout += f'App: http://{connection.host}:{connection.app_port}/\n'.encode()
        else:
            result = ssh(connection, 'python3 -', inspection(connection, app=args.app))
            if result.returncode == 0:
                report = json.loads(result.stdout)
                if args.snapshot:
                    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                    with args.snapshot.open('x') as stream:
                        json.dump({name: digest for name, digest in report['files'].items() if digest}, stream, indent=2)
                        stream.write('\n')
                if not args.snapshot and not report_valid(report, app=args.app):
                    result = subprocess.CompletedProcess(result.args, 1, result.stdout, result.stderr)
    except KeyboardInterrupt:
        parser.exit(130, 'Interrupted.\n')
    except (ValueError, TypeError) as error:
        parser.exit(1, str(error) + '\n')
    except (OSError, subprocess.TimeoutExpired):
        parser.exit(1, 'Pi operation failed; check connection, file paths and prerequisites. No automatic retry.\n')
    sys.stdout.buffer.write(result.stdout)
    sys.stderr.buffer.write(result.stderr)
    raise SystemExit(result.returncode)
