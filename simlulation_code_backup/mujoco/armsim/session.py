"""SandboxSession: robot-arm's Session plus one action, move_pose, that the pick-and-place needs.

move_pose moves all six joints to a full pose at the demo speed, validated with the same batch
check and run by the same motion runner as go_pose. It is the action proposed to robot-arm.
"""
import threading

from armsim.robotarm import BOUNDS, DemoStore, PoseStore, Session, Trajectory


class SandboxSession(Session):
    def move_pose(self, *, counts, speed=None):
        with self.lock:
            start = self._full_commands()
            if speed is None:
                speed = self.demo_speed
            if type(speed) is not int or not 1 <= speed <= 100:
                raise ValueError('Speed must be integer counts/s from 1 to 100')
            target = self._as_pose(counts)
            limits = self._check_batch(start, [target], speed=speed)
            self.demo.reset()
            self.budget.accept()
            self.last_activity = self.clock()
            self.busy, self.phase, self.error = True, 'Moving to pose', None
            self.worker = threading.Thread(target=self._go_pose, args=(start, target, limits), daemon=True)
            self.worker.start()

    @staticmethod
    def _as_pose(counts):
        if isinstance(counts, (list, tuple)):
            if len(counts) != 6:
                raise ValueError('A pose needs six counts')
            counts = dict(enumerate(counts))
        if not isinstance(counts, dict):
            raise ValueError('A pose needs six counts')
        pose = {}
        for channel in range(6):
            value = counts.get(channel, counts.get(str(channel)))
            if type(value) is not int:
                raise ValueError('A pose needs all six integer counts')
            pose[channel] = value
        return pose


def sandbox_session(driver, *, directory, demo_directory, clock=None, sleep=None, **options):
    """A SandboxSession over all six channels with scoped pose and demo stores."""
    timing = {}
    if clock is not None:
        timing['clock'] = clock
    if sleep is not None:
        timing['sleep'] = sleep
    return SandboxSession(driver, list(range(6)), pose_store=PoseStore(directory, driver.scope),
                          demo_store=DemoStore(demo_directory, driver.scope, BOUNDS), **timing, **options)
