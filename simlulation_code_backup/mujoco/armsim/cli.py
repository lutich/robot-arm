"""Text interface for the arm, sandbox or real: python -m armsim.cli [--target URL] [SCRIPT]

Commands use the API's terms. Every motion command is logged with its counts; the log replays
on either target.
"""
import argparse
import json
import shlex
import sys
import time
from pathlib import Path

from armsim.client import ArmError, HttpArmClient
from armsim.config import ROOT, load_robot, load_scene
from armsim.replay import import_demo, read_pose_log, replay_pose_log, run_demo
from armsim.robotarm import BOUNDS, DemoStore
from armsim.tasks import HttpWorld, ManualWorld, PickPlace

HELP = """commands:
  power on | power off | stop            session control; on a hardware target "power on" asks for the PARK confirmation
  power on confirmed                      scripted form of that confirmation (hardware only)
  move S2 450      jog S3 + | jog S3 -   one joint (S1..S6 or channel 0..5)
  pose 320 427 403 220 480 650 [speed]   all six joints (move_pose, or single joints as fallback)
  home | park      speed N               named poses; demo speed in counts/s
  state | blocks | reset                  status; block positions; sandbox blocks back to their slots
  blocks set red 0.110 0.110 [yaw]        tell the task layer where a block is (real arm: measure on the printed grid)
  pick red | place T2 | run red T2        task layer (needs block positions from /sim/blocks)
  stack yellow red                        pick one block and set it down on another
  demo import FILE | demo load NAME | demo run | demo next | demo pause | demo list
  snapshot [CAMERA] [FILE]                render an image (sandbox) or fetch the camera (real arm)
  log FILE | replay FILE                  save the pose log; replay a pose log
  script FILE | help | quit"""


def channel_of(token):
    token = token.upper()
    if token.startswith('S') and token[1:].isdigit():
        return int(token[1:]) - 1
    if token.isdigit():
        return int(token)
    raise ArmError(f'Unknown joint {token}')


class FallbackWorld:
    """Block positions from /sim/blocks when the target has it, otherwise the operator's entries."""

    def __init__(self, http, manual):
        self.http, self.manual = http, manual

    def blocks(self):
        try:
            return self.http.blocks()
        except ArmError:
            if not self.manual.poses:
                raise ArmError('This target has no block positions; enter them with "blocks set NAME X Y [YAW]"')
            return self.manual.blocks()

    def moved(self, *args):
        self.manual.moved(*args)

    def settle(self, seconds=0.5):
        self.http.settle(seconds)


class Shell:
    def __init__(self, target, out=sys.stdout, interactive=False):
        self.client = HttpArmClient(target)
        self.target, self.out, self.interactive = target, out, interactive
        self.robot, self.scene = load_robot(), load_scene()
        self.manual = ManualWorld(self.scene)
        self.task = PickPlace(self.client, FallbackWorld(HttpWorld(self.client), self.manual), self.robot, self.scene, BOUNDS)
        self.log = self.task.log
        self.demo_store = DemoStore(ROOT / 'artifacts' / 'demos', 'synthetic', BOUNDS)

    def say(self, text):
        print(text, file=self.out, flush=True)

    def run_line(self, line):
        words = shlex.split(line)
        if not words:
            return True
        command, args = words[0].lower(), words[1:]
        try:
            return self.dispatch(command, args)
        except ArmError as error:
            self.say(f'refused: {error}')
        except (ValueError, OSError) as error:
            self.say(f'error: {error}')
        return True

    def dispatch(self, command, args):
        c = self.client
        if command in ('quit', 'exit'):
            return False
        if command == 'help':
            self.say(HELP)
        elif command == 'power' and args[:1] == ['on']:
            if c.state()['mode'] == 'hardware' and not self.confirm_hardware(args[1:2] == ['confirmed']):
                return True
            c.power_on()
            self.say('powering on: PARK, then HOME')
            c.wait_idle()
            self.record('power on')
        elif command == 'power' and args[:1] == ['off']:
            c.power_off()
            c.wait_idle()
            self.say('off')
        elif command == 'stop':
            c.stop()
            self.say('stopped, outputs off')
        elif command == 'move':
            c.move(channel_of(args[0]), int(args[1]))
            c.wait_idle()
            self.record(f'move {args[0]} {args[1]}')
        elif command == 'jog':
            c.jog(channel_of(args[0]), 1 if args[1] == '+' else -1)
            c.wait_idle()
            self.record(f'jog {args[0]} {args[1]}')
        elif command == 'pose':
            counts = [int(v) for v in args[:6]]
            speed = int(args[6]) if len(args) > 6 else None
            c.move_pose(dict(enumerate(counts)), speed=speed)
            self.record('pose', speed)
        elif command in ('home', 'park'):
            c.go_pose(command.upper())
            c.wait_idle()
            self.record(command)
        elif command == 'speed':
            c.settings(demo_speed=int(args[0]))
            self.say(f'demo speed {args[0]} counts/s')
        elif command == 'state':
            state = c.state()
            self.say(json.dumps(dict(mode=state['mode'], armed=state['armed'], busy=state['busy'], phase=state['phase'],
                                     error=state['error'], commands=state['commands'], settings=state['settings'])))
            try:
                sim = c.sim('state')
                self.say(json.dumps(dict(joint_angles={k: round(v, 1) for k, v in sim['joint_angles'].items()},
                                         grasp_point=[round(v, 4) for v in sim['grasp_point']], pad_gap=round(sim['pad_gap'], 4), held=sim['held'])))
            except ArmError:
                pass
        elif command == 'blocks' and args[:1] == ['set']:
            self.manual.set(args[1], float(args[2]), float(args[3]), float(args[4]) if len(args) > 4 else 0.0)
            self.say(f'{args[1]} at x={float(args[2]):.3f} y={float(args[3]):.3f}')
        elif command == 'blocks':
            for name, info in self.task.world.blocks().items():
                self.say(f'{name:7s} x={info["x"]:.3f} y={info["y"]:.3f} z={info["z"]:.3f} yaw={info["yaw"]:.0f}')
        elif command == 'reset':
            c.sim('reset', {})
            self.say('blocks reset')
        elif command == 'pick':
            self.task.pick(args[0])
            self.say(f'holding {args[0]}')
        elif command == 'place':
            self.task.place(args[0])
            self.say(f'placed at {args[0]}')
        elif command == 'run':
            self.task.run(args[0], args[1])
            self.say(f'{args[0]} moved to {args[1]}, back at HOME')
        elif command == 'stack':
            self.task.stack(args[0], args[1])
            self.say(f'{args[0]} stacked on {args[1]}, back at HOME')
        elif command == 'demo':
            self.demo(args)
        elif command == 'snapshot':
            self.snapshot(args)
        elif command == 'log':
            self.task.save_log(args[0])
            self.say(f'{len(self.log)} poses written to {args[0]}')
        elif command == 'replay':
            entries = read_pose_log(args[0])
            replay_pose_log(c, entries)
            self.say(f'replayed {len(entries)} poses')
        elif command == 'script':
            for line in Path(args[0]).read_text().splitlines():
                if line.strip() and not line.startswith('#'):
                    self.say(f'> {line}')
                    if not self.run_line(line):
                        return False
        else:
            self.say(f'unknown command {command!r}; type help')
        return True

    def demo(self, args):
        c = self.client
        sub = args[0] if args else 'list'
        if sub == 'import':
            filename = import_demo(args[1], self.demo_store)
            self.say(f'imported as {filename}')
        elif sub == 'load':
            saved = c.state()['demo']['saved']
            match = [d for d in saved if d['filename'] == args[1] or d['name'].startswith(args[1])]
            if not match:
                raise ArmError(f'No saved demo matches {args[1]}')
            c.demo('load', filename=match[0]['filename'])
            self.say(f'loaded {match[0]["name"]} ({match[0]["positions"]} positions)')
        elif sub == 'run':
            c.demo('run')
            c.wait_idle(600)
            self.say('demo finished')
        elif sub in ('next', 'pause', 'resume', 'restart'):
            c.demo(sub)
            if sub == 'next':
                c.wait_idle()
            self.say(sub)
        else:
            for d in c.state()['demo']['saved']:
                self.say(f'{d["filename"]}  {d["name"]}  ({d["positions"]} positions)')

    def snapshot(self, args):
        camera = args[0] if args else 'oak'
        path = Path(args[1]) if len(args) > 1 else ROOT / 'artifacts' / 'snapshots' / f'{time.strftime("%Y%m%d-%H%M%S")}-{camera}.png'
        path.parent.mkdir(parents=True, exist_ok=True)
        from urllib.request import urlopen
        try:
            with urlopen(f'{self.target}/sim/snapshot.png?camera={camera}', timeout=10) as response:
                data = response.read()
        except Exception:
            with urlopen(f'{self.target}/api/camera/snapshot.jpg', timeout=10) as response:
                data = response.read()
                path = path.with_suffix('.jpg')
        path.write_bytes(data)
        self.say(f'saved {path}')

    def confirm_hardware(self, scripted_confirmation):
        """robot-arm's power_on enables the servos at PARK: on hardware that needs an attending operator."""
        if scripted_confirmation:
            self.say('HARDWARE: PARK confirmed by the script')
            return True
        if not self.interactive:
            self.say('refused: hardware target; write "power on confirmed" once the arm is physically in PARK, supported, and the servo power switch is within reach')
            return False
        answer = input('HARDWARE: is the arm physically in PARK, supported, and the servo power switch within reach? Type yes: ')
        if answer.strip().lower() != 'yes':
            self.say('aborted')
            return False
        return True

    def record(self, label, speed=None):
        commands = self.client.commands()
        if all(commands.get(c) is not None for c in range(6)):
            self.log.append(dict(label=label, counts=[commands[c] for c in range(6)], speed=speed or 20, t=time.time()))

    def repl(self):
        self.interactive = True
        try:
            mode = self.client.state()['mode']
        except ArmError as error:
            mode = f'unreachable ({error})'
        self.say(f'arm shell against {self.target} [{mode}]; type help')
        if mode == 'hardware':
            self.say('This is the REAL ARM. Keep the servo power switch within reach; "stop" cuts the outputs.')
        while True:
            try:
                line = input('arm> ')
            except (EOFError, KeyboardInterrupt):
                self.say('')
                return
            if not self.run_line(line):
                return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('script', nargs='?', help='Command script to run instead of the interactive shell')
    parser.add_argument('--target', default='http://127.0.0.1:8765', help='Sandbox or real-arm API address')
    args = parser.parse_args()
    shell = Shell(args.target)
    if args.script:
        shell.run_line(f'script {args.script}')
    else:
        shell.repl()


if __name__ == '__main__':
    main()
