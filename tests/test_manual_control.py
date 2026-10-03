"""Offline commissioning app ownership, limits, stop and HTTP-origin tests."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.infrastructure import drivers
from roboter_arm.control.infrastructure import pca9685
from roboter_arm.control.presentation import http_api as app
from test_motion import FakeTime


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.time = FakeTime()
        self.driver = app.PreviewDriver()
        self.directory = Path(tempfile.mkdtemp(prefix='robot-pose-test-'))
        self.session = app.session_for(self.driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos',
                                   clock=self.time.clock, sleep=self.time.sleep)

    def arm(self):
        self.session.prepare(supported=True, power_off=True)
        self.session.arm(ready=True, supported=True, switch_ready=True)

    def move(self, channel=4, target=480, first=True):
        self.session.move(channel=channel, target=target, first_clear=first)
        self.session.worker.join(timeout=2)
        self.assertFalse(self.session.busy)

    def test_power_on_requires_acknowledgment_and_all_park_targets_before_io(self):
        with patch.object(self.driver, 'prepare', wraps=self.driver.prepare) as prepare:
            for flag in (False, 1):
                with self.assertRaises(ValueError):
                    self.session.power_on(park_confirmed=flag)
            self.session.channels = (4,)
            with self.assertRaises(ValueError):
                self.session.power_on(park_confirmed=True)
            self.session.channels = tuple(range(6))
            self.session.ranges[0] = (90,329)
            with self.assertRaises(ValueError):
                self.session.power_on(park_confirmed=True)
            self.session.ranges[0] = (330,660)  # PARK fits, HOME does not.
            with self.assertRaises(ValueError):
                self.session.power_on(park_confirmed=True)
            prepare.assert_not_called()
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_power_on_park_counts_match_the_recorded_physical_observation(self):
        observation = json.loads((app.ROOT / 'config/reference-arm/park.json').read_text())
        self.assertEqual({str(c):v for c,v in app.PARK.items()}, observation['counts'])
        self.assertTrue(observation['observed'])
        self.assertFalse(observation['validated_limits'])
        home = json.loads((app.ROOT / 'config/reference-arm/home.json').read_text())
        self.assertEqual({str(c):v for c,v in app.HOME.items()}, home['counts'])
        self.assertTrue(home['observed'])
        self.assertFalse(home['validated_limits'])

    def test_power_on_moves_park_to_home_then_one_joint_moves_and_stop_clears_authority(self):
        with patch.dict(sys.modules, {'board':None, 'adafruit_pca9685':None}):
            self.session.power_on(park_confirmed=True)
            self.session.worker.join(timeout=2)
            self.assertTrue(self.session.armed)
            self.assertFalse(self.session.busy)
            self.assertEqual(self.session.commands, app.HOME)
            self.assertEqual(self.session.budget.moves, 0)
            with self.assertRaises(ValueError):
                self.session.power_on(park_confirmed=True)
            self.move(target=490, first=False)
            self.assertEqual(self.driver.outputs[4], 490)
            for c in app.PARK:
                if c != 4:
                    self.assertEqual(self.driver.outputs[c], app.HOME[c])
            self.assertTrue(self.session.stop())
            self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))
            self.assertTrue(all(v is None for v in self.session.commands.values()))
            with self.assertRaises(ValueError):
                self.session.power_on(park_confirmed=False)
            self.session.power_on(park_confirmed=True)
            self.session.worker.join(timeout=2)
            self.assertEqual(self.session.commands, app.HOME)

    def test_startup_orders_park_writes_before_coordinated_home_and_park_off_disables(self):
        writes = []
        write = self.driver.write
        def record(channel, count):
            writes.append((channel, count))
            write(channel, count)
        self.driver.write = record
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.assertEqual(writes[:6], list(app.PARK.items()))
        self.assertEqual(self.session.commands, app.HOME)
        self.assertAlmostEqual(self.time.now, 13.275, places=3)
        for channel, count in writes:
            low, high = self.session.ranges[channel]
            self.assertTrue(low <= count <= high)
        self.assertEqual({c:self.driver.outputs[c] for c in app.HOME}, app.HOME)
        self.move(channel=1, target=430, first=False)
        writes.clear()
        self.session.power_off()
        self.session.worker.join(timeout=2)
        self.assertFalse(self.session.armed)
        self.assertFalse(self.session.busy)
        self.assertTrue(self.session.outputs_off)
        self.assertEqual({c:self.session.last_command[c] for c in app.PARK}, app.PARK)
        self.assertEqual({c:count for c,count in writes[-6:]}, app.PARK)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))
        self.assertTrue(all(v is None for v in self.session.commands.values()))

    def test_power_off_is_rejected_during_a_move_and_immediate_stop_still_works(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.session.busy = True
        with self.assertRaises(ValueError):
            self.session.power_off()
        self.session.busy = False
        self.session.stop()
        self.session.worker.join(timeout=2)
        self.assertTrue(self.session.outputs_off)
        with self.assertRaises(ValueError):
            self.session.power_off()

    def test_power_off_failure_stops_directly_without_continuing_park(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        write = self.driver.write
        calls = []
        def fail(channel, count):
            calls.append((channel, count))
            write(channel, count)
            if len(calls) == 2:
                raise OSError('fake return failure')
        self.driver.write = fail
        self.session.power_off()
        self.session.worker.join(timeout=2)
        self.assertEqual(len(calls), 2)
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertIn('PARK return stopped', self.session.error)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_immediate_stop_during_power_off_prevents_later_writes(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        write = self.driver.write
        calls = []
        def stop_after_first(channel, count):
            calls.append((channel, count))
            write(channel, count)
            self.session.stop()
        self.driver.write = stop_after_first
        self.session.power_off()
        self.session.worker.join(timeout=2)
        self.assertEqual(len(calls), 1)
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_partial_park_enable_failure_disables_every_output(self):
        write = self.driver.write
        attempted = []
        def fail(channel, count):
            attempted.append(channel)
            write(channel, count)
            if channel == 2:
                raise OSError('fake PARK write failure')
        self.driver.write = fail
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.assertEqual(attempted, [0,1,2])
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))
        self.assertIn('HOME transition stopped', self.session.error)

    def test_failure_during_park_to_home_disables_all_outputs(self):
        write = self.driver.write
        attempted = []
        def fail(channel, count):
            attempted.append((channel, count))
            write(channel, count)
            if channel == 1 and count < app.PARK[1]:
                raise OSError('fake outbound transition failure')
        self.driver.write = fail
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.assertIn((1, app.PARK[1]), attempted)
        self.assertTrue(any(channel == 1 and count < app.PARK[1] for channel,count in attempted))
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_stop_during_preparation_is_not_cleared_by_power_on(self):
        prepare = self.driver.prepare
        def stop_during_prepare():
            prepare()
            self.session.stop()
        self.driver.prepare = stop_during_prepare
        with self.assertRaises(InterruptedError):
            self.session.power_on(park_confirmed=True)
        self.assertTrue(self.session.cancel.is_set())
        self.assertFalse(self.session.armed)
        self.assertIsNone(self.session.worker)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_stop_during_park_enable_prevents_remaining_nonzero_writes(self):
        write = self.driver.write
        attempted = []
        def stop_after_first(channel, count):
            attempted.append(channel)
            write(channel, count)
            self.session.stop()
        self.driver.write = stop_after_first
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.assertEqual(attempted, [0])
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))

    def test_park_enable_stops_at_session_deadline_between_driver_calls(self):
        write = self.driver.write
        attempted = []
        def slow_write(channel, count):
            attempted.append(channel)
            write(channel, count)
            self.time.sleep(1)
        self.driver.write = slow_write
        self.session.budget.seconds = 1
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.assertEqual(attempted, [0])
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)

    def test_power_on_rejects_unverified_off_after_disable_failure(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        def fail_disable(_):
            raise OSError('fake disable failure')
        self.driver.disable = fail_disable
        self.assertFalse(self.session.stop())
        with self.assertRaises(ValueError):
            self.session.power_on(park_confirmed=True)

    def test_default_has_no_initial_outputs_and_requires_physical_gates(self):
        self.assertEqual(self.session.commands, {c:None for c in range(6)})
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))
        with self.assertRaises(ValueError):
            self.session.arm(ready=True, supported=True, switch_ready=True)
        with self.assertRaises(ValueError):
            self.session.prepare(supported=True, power_off=False)
        self.arm()
        with self.assertRaises(ValueError):
            self.session.move(channel=4, target=480, first_clear=False)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_attended_session_can_enforce_narrow_range_time_and_move_budget(self):
        session = app.session_for(app.PreviewDriver(), [4], directory=self.directory, demo_directory=self.directory / 'demos', ranges={4:(475,485)},
            seconds=120, max_moves=4, clock=self.time.clock, sleep=self.time.sleep)
        session.prepare(supported=True, power_off=True)
        session.arm(ready=True, supported=True, switch_ready=True)
        self.assertEqual(session.budget.deadline, 120)
        self.assertEqual(session.state()['remaining_moves'], 4)
        with self.assertRaises(ValueError):
            session.move(channel=4, target=486, first_clear=True)
        for target in (480,485,475,480):
            session.move(channel=4, target=target, first_clear=True)
            session.worker.join(timeout=2)
        with self.assertRaises(ValueError):
            session.move(channel=4, target=481, first_clear=True)
        self.time.now = 120
        session.watchdog()
        self.assertFalse(session.armed)
        self.assertTrue(session.outputs_off)
        for ranges in ({4:(89,485)}, {3:(245,255)}, {4:(486,485)}):
            with self.assertRaises(ValueError):
                app.session_for(app.PreviewDriver(), [4], directory=self.directory, demo_directory=self.directory / 'demos', ranges=ranges)
    def test_full_historical_range_single_channel_smooth_and_holding(self):
        self.arm()
        self.move()
        self.move(target=660, first=False)  # Beyond the old ±5 commissioning restriction.
        self.assertEqual(self.driver.outputs[4], 660)
        self.assertTrue(all(v == 0 for c, v in self.driver.outputs.items() if c != 4))
        self.assertAlmostEqual(self.time.now, 13.5)
        self.assertEqual(self.session.last_command, {4:660})
        self.session.stop()
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))
        self.assertTrue(all(v is None for v in self.session.commands.values()))
        self.assertEqual(self.session.last_command, {4:660})

    def test_invalid_out_of_session_and_busy_requests_preserve_hold(self):
        self.arm()
        self.move()
        for channel, target in ((True,480), (6,480), (4,661), (4,480.5)):
            with self.assertRaises(ValueError):
                self.session.move(channel=channel, target=target, first_clear=True)
        self.session.channels = (4,)
        with self.assertRaises(ValueError):
            self.session.move(channel=3, target=250, first_clear=True)
        self.session.busy = True
        with self.assertRaises(ValueError):
            self.session.move(channel=4, target=500, first_clear=True)
        self.assertEqual(self.driver.outputs[4], 480)

    def test_watchdog_session_deadline_stops_and_invalidates(self):
        self.session.budget.seconds = 600
        self.arm()
        self.move()
        self.time.now = 601
        self.session.watchdog()
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))

    def test_direct_stop_prevents_any_later_worker_write(self):
        self.arm()
        self.move()
        def stop_sleep(seconds):
            self.time.sleep(seconds)
            self.session.stop()
        self.session.sleep = stop_sleep
        self.move(target=500)
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v == 0 for v in self.driver.outputs.values()))

    def test_move_failure_disables_every_output_and_prevents_rearm_if_disable_fails(self):
        self.arm()
        def fail_write(*_):
            raise OSError('fake driver failure')
        self.driver.write = fail_write
        self.move()
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.session.arm(ready=True, supported=True, switch_ready=True)
        attempted = []
        def fail_disable(channel):
            attempted.append(channel)
            if channel == 0:
                raise OSError('disable fault')
        self.driver.disable = fail_disable
        self.assertFalse(self.session.stop())
        self.assertEqual(attempted, list(range(16)))
        with self.assertRaises(ValueError):
            self.session.arm(ready=True, supported=True, switch_ready=True)

    def test_pose_requires_all_commands_observation_and_preserves_prior_records(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.session.save_pose(name='PARK', observed=True, provenance='synthetic')
        for channel, target in enumerate(app.STARTUP):
            self.move(channel, target)
        with self.assertRaises(ValueError):
            self.session.save_pose(name='PARK', observed=False, provenance='synthetic')
        first = self.session.save_pose(name='PARK', observed=True, provenance='synthetic preview')
        content = (self.directory / 'synthetic' / first).read_bytes()
        second = self.session.save_pose(name='HOME', observed=True, provenance='synthetic preview')
        self.assertNotEqual(first, second)
        self.assertEqual((self.directory / 'synthetic' / first).read_bytes(), content)
        record = json.loads(content)
        self.assertEqual(record['counts']['4'], 480)
        self.assertFalse(record['position_feedback'])
        self.assertFalse(record['validated_limits'])
        self.assertEqual(record['scope'], 'synthetic')

    def test_saved_home_and_park_drive_moves_and_power_transitions_after_restart(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.move(channel=4, target=249, first=False)
        self.move(channel=5, target=552, first=False)
        home = dict(self.session.commands)
        before = dict(self.driver.outputs)
        home_file = self.session.save_pose(name='HOME', observed=True, provenance='synthetic HOME')
        archived = (self.directory / 'synthetic' / home_file).read_bytes()
        self.assertEqual(self.driver.outputs, before)
        self.session.go_pose(name='PARK')
        self.session.worker.join(timeout=2)
        self.assertEqual(self.session.commands, app.PARK)
        self.move(channel=4, target=250, first=False)
        self.move(channel=5, target=553, first=False)
        park = dict(self.session.commands)
        before = dict(self.driver.outputs)
        self.session.save_pose(name='PARK', observed=True, provenance='synthetic PARK')
        self.assertEqual(self.driver.outputs, before)
        self.assertEqual((self.directory / 'synthetic' / home_file).read_bytes(), archived)
        self.assertEqual({j['channel']:j['home'] for j in self.session.state()['joints']}, home)
        self.assertEqual({j['channel']:j['park'] for j in self.session.state()['joints']}, park)
        self.session.set_limits(channel=4, low=240, high=300)  # Excludes the shipped defaults.
        with self.assertRaises(ValueError):
            self.session.set_limits(channel=4, low=249, high=249)
        for name, target in (('HOME', home), ('PARK', park), ('HOME', home)):
            self.session.go_pose(name=name)
            self.session.worker.join(timeout=2)
            self.assertEqual(self.session.commands, target)
        self.session.power_off()
        self.session.worker.join(timeout=2)
        self.assertEqual(self.session.last_command, park)
        self.assertTrue(self.session.outputs_off)
        self.session.close()

        driver = app.PreviewDriver()
        restarted = app.session_for(driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos',
                                clock=self.time.clock, sleep=self.time.sleep)
        self.addCleanup(restarted.close)
        self.assertFalse(restarted.prepared)
        self.assertFalse(restarted.armed)
        self.assertTrue(all(count is None for count in restarted.commands.values()))
        self.assertTrue(all(count == 0 for count in driver.outputs.values()))
        with patch.object(driver, 'prepare', wraps=driver.prepare) as prepare:
            restarted.ranges[4] = (475,485)
            with self.assertRaises(ValueError):
                restarted.power_on(park_confirmed=True)
            prepare.assert_not_called()
        restarted.ranges[4] = (240,300)
        writes = []
        write = driver.write
        def record(channel, count):
            writes.append((channel, count))
            write(channel, count)
        driver.write = record
        restarted.power_on(park_confirmed=True)
        restarted.worker.join(timeout=2)
        self.assertEqual(writes[:6], list(park.items()))
        self.assertEqual(restarted.commands, home)
        restarted.power_off()
        restarted.worker.join(timeout=2)
        self.assertEqual(restarted.last_command, park)
        self.assertTrue(restarted.outputs_off)
        self.assertEqual(app.HOME[4], 480)
        self.assertEqual(app.PARK[4], 480)

    def test_pose_definitions_and_archives_are_isolated_by_driver_scope(self):
        for scope, target in (('synthetic',249), ('commissioning',251)):
            driver = app.PreviewDriver()
            driver.scope = scope  # Fake hardware: no bus access.
            session = app.session_for(driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos',
                                  clock=self.time.clock, sleep=self.time.sleep)
            self.addCleanup(session.close)
            self.assertEqual(session.poses, {'HOME':app.HOME, 'PARK':app.PARK})
            session.power_on(park_confirmed=True)
            session.worker.join(timeout=2)
            session.move(channel=4, target=target, first_clear=False)
            session.worker.join(timeout=2)
            for name in ('HOME', 'PARK'):
                session.save_pose(name=name, observed=True, provenance='scope separation test')
            session.close()
        for scope, target in (('synthetic',249), ('commissioning',251)):
            driver = app.PreviewDriver()
            driver.scope = scope
            loaded = app.session_for(driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos')
            self.addCleanup(loaded.close)
            self.assertEqual(loaded.poses['HOME'][4], target)
            self.assertEqual(loaded.poses['PARK'][4], target)

    def test_records_from_another_driver_scope_are_rejected(self):
        bounds = {c:(low, high) for c, (_, low, high) in app.JOINTS.items()}
        for pose_scope, demo_scope in (('commissioning', 'synthetic'), ('synthetic', 'commissioning')):
            with self.subTest(pose_scope=pose_scope, demo_scope=demo_scope), self.assertRaises(ValueError):
                app.Session(app.PreviewDriver(), list(range(6)),
                            pose_store=app.PoseStore(self.directory, pose_scope),
                            demo_store=app.DemoStore(self.directory, demo_scope, bounds))

    def test_failed_activation_preserves_the_active_pose_and_does_not_move(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        for name in ('HOME', 'PARK'):
            self.session.save_pose(name=name, observed=True, provenance='original definition')
            path = self.directory / 'synthetic' / f'{name}.json'
            original = path.read_bytes()
            original_counts = dict(self.session.poses[name])
            self.move(channel=4, target=481, first=False)
            before = dict(self.driver.outputs)
            with patch.object(Path, 'replace', side_effect=OSError('activation failed')):
                with self.assertRaises(OSError):
                    self.session.save_pose(name=name, observed=True, provenance='failed update')
            self.assertEqual(self.session.poses[name], original_counts)
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(self.driver.outputs, before)

    def test_invalid_persisted_pose_rejects_startup_before_driver_preparation(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.session.save_pose(name='HOME', observed=True, provenance='synthetic definition')
        path = self.directory / 'synthetic' / 'HOME.json'
        original = json.loads(path.read_text())
        cases = [dict(original, scope='commissioning'),
                 dict(original, counts={**original['counts'], '4':True}),
                 dict(original, counts={**original['counts'], '5':489}),
                 dict(original, counts={'4':480})]
        driver = app.PreviewDriver()
        with patch.object(driver, 'prepare') as prepare:
            for record in cases:
                path.write_text(json.dumps(record))
                with self.subTest(record=record), self.assertRaises(ValueError):
                    app.session_for(driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos')
            path.write_text('{invalid')
            with self.assertRaises(ValueError):
                app.session_for(driver, list(range(6)), directory=self.directory, demo_directory=self.directory / 'demos')
            prepare.assert_not_called()

    def test_default_allows_more_than_sixty_moves_after_ten_minutes(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.time.now = 1200
        state = self.session.state()
        self.session.watchdog()
        self.assertTrue(self.session.armed)
        self.assertIsNone(state['remaining_seconds'])
        self.assertIsNone(state['remaining_moves'])
        for _ in range(61):
            self.move(target=480, first=False)
        self.assertEqual(self.session.budget.moves, 61)
        self.assertEqual(self.session.commands, app.HOME)
        self.session.save_pose(name='HOME', observed=True, provenance='synthetic preview')
        self.assertTrue(self.session.armed)

    def test_inactivity_warns_without_stopping_and_polling_does_not_clear_it(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.time.now = 599
        self.assertFalse(self.session.state()['inactivity_warning'])
        self.time.now = 600
        self.assertTrue(self.session.state()['inactivity_warning'])
        self.session.watchdog()
        self.assertTrue(self.session.armed)
        self.assertEqual(self.session.commands, app.HOME)
        self.time.now = 900
        self.assertTrue(self.session.state()['inactivity_warning'])
        self.move(target=481, first=False)
        self.assertFalse(self.session.state()['inactivity_warning'])
        self.time.now += 600
        self.assertTrue(self.session.state()['inactivity_warning'])
        self.session.save_pose(name='HOME', observed=True, provenance='synthetic preview')
        self.assertFalse(self.session.state()['inactivity_warning'])
        self.session.stop()
        self.assertFalse(self.session.state()['inactivity_warning'])

    def test_browser_absence_does_not_stop_the_session(self):
        self.session.power_on(park_confirmed=True)
        self.session.worker.join(timeout=2)
        self.time.now = 1000  # No browser request since power on.
        self.session.watchdog()
        self.assertTrue(self.session.armed)
        self.assertFalse(self.session.outputs_off)
        self.assertEqual(self.session.commands, app.HOME)
        self.assertIsNone(self.session.error)

    def test_import_and_default_preview_never_load_board_driver(self):
        with patch.dict(sys.modules, {'board':None, 'adafruit_pca9685':None}):
            self.arm()
            self.move()
            self.session.close()

    def test_real_adapter_count_conversion_and_explicit_initialization_with_fake_context(self):
        from contextlib import contextmanager
        from test_joint_control import Output
        import types
        pwm = types.SimpleNamespace(channels=[Output() for _ in range(16)])
        @contextmanager
        def fake_controller():
            yield pwm
        with patch.object(drivers, 'controller', fake_controller):
            driver = app.HardwareDriver()
            self.assertIsNone(driver.pwm)
            driver.prepare()
            driver.write(4, 480)
            self.assertEqual(pwm.channels[4].value, 480 << 4)
            self.assertTrue(all(not c.writes for i,c in enumerate(pwm.channels) if i != 4))
            driver.disable(4)
            driver.verify_off()
            driver.close()
            self.assertIsNone(driver.pwm)

    def test_shared_hardware_context_lock_rejects_second_owner_before_bus_access(self):
        from contextlib import contextmanager
        @contextmanager
        def fake_controller():
            yield object()
        with patch.object(pca9685, '_controller', fake_controller):
            with pca9685.controller():
                with self.assertRaises(RuntimeError):
                    with pca9685.controller():
                        self.fail('Second controller owner must not acquire hardware')


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.records = Path(self.enterContext(tempfile.TemporaryDirectory(prefix='robot-http-test-')))
        self.session = app.session_for(app.PreviewDriver(), [4], directory=self.records / 'poses',
                                       demo_directory=self.records / 'demos')
        self.server = app.Server(self.session, 0)
        self.server.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.stop()
        self.session.close()

    def request(self, method, path, data=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        headers = ({'Content-Type':'application/json'} if data is not None else {}) | (headers or {})
        connection.request(method, path, json.dumps(data) if data is not None else None, headers)
        response = connection.getresponse()
        content = response.read()
        connection.close()
        return response.status, content

    def test_page_state_and_same_origin_guards(self):
        status, page = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertNotIn(b'robot-token', page)
        self.assertIn(b'/park-illustration-v2.png', page)
        self.assertIn(b'/home-illustration.png', page)
        self.assertEqual(page.count(b'data-joint='), 6)
        self.assertIn(b'EMERGENCY STOP', page)
        for path in ('/manual_control.css', '/manual_control.js'):
            self.assertEqual(self.request('GET', path)[0], 200)
        for path in ('/park-illustration-v2.png', '/home-illustration.png'):
            status, illustration = self.request('GET', path)
            self.assertEqual(status, 200)
            self.assertTrue(illustration.startswith(b'\x89PNG\r\n\x1a\n'))
        status, content = self.request('GET', '/api/state')
        self.assertEqual(json.loads(content)['mode'], 'preview')
        self.assertEqual(self.request('GET', '/', headers={'Host':'evil.example'})[0], 403)
        for host in ('192.168.0.120', 'raspberrypi.local', 'raspberrypi', 'localhost', '[fe80::1]'):
            with self.subTest(host=host):
                self.assertEqual(self.request('GET', '/api/state', headers={'Host':f'{host}:{self.port}'})[0], 200)
        args = dict(supported=True, power_off=True)
        self.assertEqual(self.request('POST', '/api/prepare', args)[0], 403)
        headers = {'Origin':f'http://127.0.0.1:{self.port}'}
        self.assertEqual(self.request('POST', '/api/prepare', args, headers)[0], 200)
        self.assertEqual(self.request('POST', '/api/arm', dict(ready=True,supported=True,switch_ready=True), headers)[0], 200)
        headers['Origin'] = 'http://evil.example'
        self.assertEqual(self.request('POST', '/api/stop', {}, headers)[0], 403)
        rebound = {'Host':'evil.example', 'Origin':'http://evil.example'}
        self.assertEqual(self.request('POST', '/api/stop', {}, rebound)[0], 403)
        network = {'Host':f'raspberrypi.local:{self.port}', 'Origin':f'http://127.0.0.1:{self.port}'}
        self.assertEqual(self.request('POST', '/api/stop', {}, network)[0], 403)
        self.assertTrue(self.session.armed)
        network['Origin'] = f'http://raspberrypi.local:{self.port}'
        self.assertEqual(self.request('POST', '/api/stop', {}, network)[0], 200)
        self.assertFalse(self.session.armed)

    def test_simple_power_endpoint_goes_home_then_park_and_stop_remains_direct(self):
        fake = FakeTime()
        self.session = app.session_for(app.PreviewDriver(), list(range(6)), directory=self.records / 'poses',
                                       demo_directory=self.records / 'demos', clock=fake.clock, sleep=fake.sleep)
        self.server.session = self.session
        headers = {'Origin':f'http://127.0.0.1:{self.port}'}
        self.assertEqual(self.request('POST', '/api/power_on', {'park_confirmed':True})[0], 403)
        self.assertEqual(self.request('POST', '/api/power_on', {'park_confirmed':False}, headers)[0], 400)
        self.assertEqual(self.request('POST', '/api/power_on', {'park_confirmed':True}, headers)[0], 200)
        self.session.worker.join(timeout=2)
        self.assertEqual(self.session.commands, app.HOME)
        self.assertEqual(self.request('POST', '/api/power_off', {}, headers)[0], 200)
        self.session.worker.join(timeout=2)
        self.assertTrue(self.session.outputs_off)
        self.assertEqual({c:self.session.last_command[c] for c in app.PARK}, app.PARK)
        self.assertEqual(self.request('POST', '/api/power_on', {'park_confirmed':True}, headers)[0], 200)
        self.session.worker.join(timeout=2)
        self.assertEqual(self.request('POST', '/api/stop', {}, headers)[0], 200)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))


if __name__ == '__main__':
    unittest.main()
