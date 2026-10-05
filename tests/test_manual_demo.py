"""Offline full-pose, scoped recording and finite Demo playback checks."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation import http_api as app
from roboter_arm.control.infrastructure.demo_store import DemoStore
from test_motion import FakeTime


class DemoTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='robot-demo-test-')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.time = FakeTime()
        self.driver = app.PreviewDriver()
        self.session = app.session_for(self.driver, list(range(6)), directory=self.root/'poses',
            demo_directory=self.root/'demos', clock=self.time.clock, sleep=self.time.sleep)
        self.addCleanup(self.session.close)

    def join(self):
        self.session.worker.join(timeout=2)
        self.assertFalse(self.session.worker.is_alive())
        self.assertFalse(self.session.busy)

    def power_on(self):
        self.session.power_on(park_confirmed=True)
        self.join()

    def move(self, target):
        self.session.move(channel=0, target=target, first_clear=False)
        self.join()

    def record(self):
        self.power_on()
        self.session.demo_add(name='HOME')
        self.move(340)
        self.session.demo_add(name='Turn')
        self.move(350)
        self.session.demo_add(name='Further')

    def test_offline_state_and_file_operations_never_initialize_controller(self):
        with patch.dict(sys.modules, {'board':None, 'adafruit_pca9685':None}):
            hardware = app.session_for(app.HardwareDriver(), list(range(6)), directory=self.root/'poses', demo_directory=self.root/'hardware')
            self.assertEqual(hardware.state()['demo']['saved'], [])
            self.assertFalse((self.root/'hardware').exists())
            hardware.demo_name(name='Offline draft')
            hardware.demo_restart()
            for method in (hardware.demo_add, hardware.demo_next, hardware.demo_run):
                with self.assertRaises(ValueError):
                    method()
            self.assertIsNone(hardware.driver.pwm)
            hardware.close()

    def test_concurrent_stop_cannot_be_erased_by_power_on_or_arm_clear(self):
        for action_name in ('power_on','arm'):
            with self.subTest(action=action_name):
                driver=app.PreviewDriver()
                session=app.session_for(driver,list(range(6)),directory=self.root/'poses',demo_directory=self.root/action_name,
                    clock=self.time.clock,sleep=self.time.sleep)
                if action_name=='arm':
                    session.prepare(supported=True,power_off=True)
                entered,resume=threading.Event(),threading.Event()
                original_clear=session.cancel.clear
                def delayed_clear():
                    entered.set()
                    resume.wait(timeout=2)
                    original_clear()
                session.cancel.clear=delayed_clear
                errors=[]
                def begin():
                    try:
                        if action_name=='power_on':
                            session.power_on(park_confirmed=True)
                        else:
                            session.arm(ready=True,supported=True,switch_ready=True)
                    except BaseException as error:
                        errors.append(error)
                startup=threading.Thread(target=begin)
                startup.start()
                self.assertTrue(entered.wait(timeout=2))
                stopping=threading.Thread(target=lambda:session.stop('Concurrent emergency stop'))
                stopping.start()
                try:
                    self.assertTrue(session.cancel.wait(timeout=2))
                finally:
                    resume.set()
                startup.join(timeout=2)
                stopping.join(timeout=2)
                self.assertFalse(startup.is_alive())
                self.assertFalse(stopping.is_alive())
                self.assertEqual(len(errors),1)
                self.assertIsInstance(errors[0],InterruptedError)
                self.assertTrue(session.cancel.is_set())
                self.assertFalse(session.armed)
                self.assertIsNone(session.worker)
                self.assertTrue(all(v==0 for v in driver.outputs.values()))
                session.close()

    def test_startup_worker_retains_stop_generation_even_if_event_is_cleared(self):
        execute=self.session.runner.execute
        def stopped_before_execution(start,target,limits,**kwargs):
            self.session.stop()
            self.session.cancel.clear()
            self.session.armed=True
            return execute(start,target,limits,**kwargs)
        self.session.runner.execute=stopped_before_execution
        self.session.power_on(park_confirmed=True)
        self.join()
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v==0 for v in self.driver.outputs.values()))

    def test_stale_watchdog_decision_does_not_stop_a_fresh_session(self):
        self.power_on()
        self.session.budget.deadline=self.time.now  # Bounded test session just expired.
        original_stop=self.session._stop
        replaced=False
        def new_session_before_watchdog_stop(reason=None,*,expected_generation=None):
            nonlocal replaced
            if expected_generation is not None and not replaced:
                replaced=True
                original_stop()
                self.session.power_on(park_confirmed=True)
                self.join()
            return original_stop(reason,expected_generation=expected_generation)
        self.session._stop=new_session_before_watchdog_stop
        self.session.watchdog()
        self.assertTrue(replaced)
        self.assertTrue(self.session.armed)
        self.assertEqual(self.session.commands,app.HOME)
        self.assertFalse(self.session.outputs_off)

    def test_go_pose_moves_every_joint_and_holds_without_disabling(self):
        self.power_on()
        manual_moves = self.session.budget.moves
        start = self.time.now
        self.session.go_pose(name='PARK')
        self.join()
        self.assertEqual(self.session.commands, app.PARK)
        self.assertAlmostEqual(self.time.now-start, 13.275)
        self.assertTrue(self.session.armed)
        self.assertFalse(self.session.outputs_off)
        self.assertEqual(self.session.budget.moves, manual_moves)
        self.session.go_pose(name='HOME')
        self.join()
        self.assertEqual(self.session.commands, app.HOME)
        self.assertEqual({c:self.driver.outputs[c] for c in app.HOME}, app.HOME)
        with self.assertRaises(ValueError):
            self.session.go_pose(name='OTHER')

    def test_jog_settings_are_integer_atomic_and_do_not_initialize_or_move(self):
        initial = dict(step_size=10, demo_speed=20, demo_speed_max=100)
        self.assertEqual(self.session.state()['settings'], initial)
        before = dict(self.driver.outputs)
        for step, speed in ((0,20), (True,20), (2.5,20), (10,0), (10,101), (10,True), (10,5.5)):
            with self.assertRaises(ValueError):
                self.session.set_settings(step_size=step, demo_speed=speed)
            self.assertEqual(self.session.state()['settings'], initial)
        self.session.set_settings(step_size=7, demo_speed=4)
        self.assertEqual(self.session.state()['settings']['step_size'], 7)
        self.assertEqual(self.driver.outputs, before)
        self.assertFalse(self.session.prepared)

    def test_jog_arrows_use_held_count_for_each_joint_and_leave_others_holding(self):
        self.power_on()
        for channel in range(6):
            before = dict(self.session.commands)
            self.session.jog(channel=channel, direction=1)
            self.join()
            self.assertEqual(self.session.commands[channel], before[channel]+10)
            self.assertEqual({c:v for c,v in self.session.commands.items() if c!=channel},
                             {c:v for c,v in before.items() if c!=channel})
            self.session.jog(channel=channel, direction=-1)
            self.join()
            self.assertEqual(self.session.commands, before)
        self.assertEqual(self.session.budget.moves, 12)
        self.session.set_settings(step_size=3, demo_speed=1)
        begun = self.time.now
        self.session.jog(channel=0, direction=1)
        self.join()
        self.assertEqual(self.session.commands[0], app.HOME[0]+3)
        self.assertAlmostEqual(self.time.now-begun, 1.5*3/app.SPEED)

    def test_jog_rejects_unknown_invalid_and_out_of_range_without_writes(self):
        self.session.prepare(supported=True, power_off=True)
        self.session.arm(ready=True, supported=True, switch_ready=True)
        with self.assertRaises(ValueError):
            self.session.jog(channel=0, direction=1)
        self.assertTrue(all(value==0 for value in self.driver.outputs.values()))
        self.session.stop()
        self.power_on()
        self.session.jog(channel=5, direction=1)
        self.join()
        before = dict(self.driver.outputs)
        moves = self.session.budget.moves
        for channel, direction in ((5,1), (True,1), (6,1), (0,True), (0,0), (0,2)):
            with self.assertRaises(ValueError):
                self.session.jog(channel=channel, direction=direction)
        self.assertEqual(self.driver.outputs, before)
        self.assertEqual(self.session.budget.moves, moves)

    def test_settings_and_another_jog_are_rejected_while_motion_is_running(self):
        self.power_on()
        reached, release = threading.Event(), threading.Event()
        def blocked_sleep(seconds):
            reached.set()
            release.wait(timeout=2)
            self.time.sleep(seconds)
        self.session.sleep = blocked_sleep
        self.session.jog(channel=0, direction=1)
        self.assertTrue(reached.wait(timeout=2))
        try:
            with self.assertRaises(ValueError):
                self.session.set_settings(step_size=5, demo_speed=5)
            with self.assertRaises(ValueError):
                self.session.jog(channel=1, direction=1)
            self.assertEqual(self.session.step_size, 10)
            self.assertEqual(self.session.demo_speed, 20)
            self.session.stop()
        finally:
            release.set()
        self.join()
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(value is None for value in self.session.commands.values()))

    def test_demo_speed_controls_next_run_and_resume_but_not_named_poses(self):
        self.record()
        self.session.set_settings(step_size=10, demo_speed=10)
        begun = self.time.now
        self.session.demo_next()
        self.join()
        self.assertAlmostEqual(self.time.now-begun, 4.5)
        begun = self.time.now
        self.session.demo_run()
        self.join()
        self.assertAlmostEqual(self.time.now-begun, 4.5)
        requested = False
        def pause(seconds):
            nonlocal requested
            self.time.sleep(seconds)
            if not requested:
                requested = True
                self.session.demo_pause()
        self.session.sleep = pause
        self.session.demo_run()
        self.join()
        self.assertEqual(self.session.demo.cursor, 1)
        self.session.set_settings(step_size=10, demo_speed=5)
        self.assertEqual(self.session.demo.status, 'paused')
        self.session.sleep = self.time.sleep
        begun = self.time.now
        self.session.demo_resume()
        self.join()
        self.assertAlmostEqual(self.time.now-begun, 9)
        self.assertEqual(self.session.commands[0], 350)
        begun = self.time.now
        self.session.go_pose(name='PARK')
        self.join()
        self.assertAlmostEqual(self.time.now-begun, 13.275)

    def test_slower_demo_speed_is_used_for_deadline_preflight(self):
        self.record()
        self.session.set_settings(step_size=10, demo_speed=1)
        self.session.budget.deadline = self.time.now+20
        before = dict(self.driver.outputs)
        targets = self.session.budget.accepted_targets
        for method in (self.session.demo_next, self.session.demo_run):
            with self.assertRaises(ValueError):
                method()
        self.assertEqual(self.driver.outputs, before)
        self.assertEqual(self.session.budget.accepted_targets, targets)
        self.assertFalse(self.session.busy)

    def test_editable_limits_retain_poses_and_held_command_without_motion(self):
        self.power_on()
        self.move(350)
        before = dict(self.driver.outputs)
        for low,high in ((True,660),(90,661),(331,660),(90,349),(330,320)):
            with self.assertRaises(ValueError):
                self.session.set_limits(channel=0, low=low, high=high)
        self.session.set_limits(channel=0, low=320, high=350)
        self.assertEqual(self.session.ranges[0], (320,350))
        self.assertEqual(self.driver.outputs, before)
        with self.assertRaises(ValueError):
            self.session.move(channel=0, target=351, first_clear=False)

    def test_capture_copies_all_six_commands_and_edits_are_ordered(self):
        self.power_on()
        self.session.demo_add()
        original = dict(self.session.demo.positions[0]['counts'])
        self.move(340)
        self.session.demo_add(name='Second')
        self.assertEqual(self.session.demo.positions[0]['counts'], original)
        self.assertEqual(set(original), set(range(6)))
        self.session.demo_rename(index=0,name='First')
        self.session.demo_reorder(index=1,direction=-1)
        self.assertEqual([p['name'] for p in self.session.demo.positions], ['Second','First'])
        self.session.demo_replace(index=1)
        self.assertEqual(self.session.demo.positions[1]['counts'], self.session.commands)
        self.session.demo_remove(index=0)
        self.assertEqual(len(self.session.demo.positions), 1)
        self.session.stop()
        self.session.demo_rename(index=0,name='Offline edit')
        with self.assertRaises(ValueError):
            self.session.demo_replace(index=0)
        for operation in (lambda:self.session.demo_remove(index=True),
                          lambda:self.session.demo_rename(index=-1,name='No'),
                          lambda:self.session.demo_reorder(index=0,direction=True),
                          lambda:self.session.demo_name(name='')):
            with self.assertRaises(ValueError):
                operation()

    def test_next_advances_and_run_all_restarts_first_once(self):
        self.record()
        manual_moves = self.session.budget.moves
        self.session.demo_next()
        self.join()
        self.assertEqual(self.session.commands, app.HOME)
        self.assertEqual(self.session.state()['demo']['cursor'], 1)
        self.session.demo_next()
        self.join()
        self.assertEqual(self.session.commands[0], 340)
        self.assertEqual(self.session.demo.cursor, 2)
        seen = []
        execute = self.session.runner.execute
        def traced(start,target,limits,**kwargs):
            seen.append(dict(target))
            return execute(start,target,limits,**kwargs)
        self.session.runner.execute = traced
        self.session.demo_run()
        self.join()
        self.assertEqual([target[0] for target in seen], [320,340,350])
        self.assertEqual(self.session.demo.cursor, 3)
        self.assertEqual(self.session.demo.status, 'complete')
        self.assertTrue(self.session.armed)
        self.assertEqual(self.session.budget.moves, manual_moves)
        with self.assertRaises(ValueError):
            self.session.demo_next()
        before = dict(self.driver.outputs)
        self.session.demo_restart()
        self.assertEqual(self.session.demo.cursor, 0)
        self.assertEqual(self.driver.outputs, before)

    def test_pause_finishes_current_step_then_resume_continues(self):
        self.record()
        requested = False
        def pause_during_move(seconds):
            nonlocal requested
            self.time.sleep(seconds)
            if not requested:
                requested = True
                self.session.demo_pause()
                self.assertEqual(self.session.demo.status, 'pausing')
        self.session.sleep = pause_during_move
        self.session.demo_run()
        self.join()
        self.assertEqual(self.session.demo.cursor, 1)
        self.assertEqual(self.session.demo.status, 'paused')
        self.assertEqual(self.session.commands, app.HOME)
        self.assertTrue(self.session.armed)
        with self.assertRaises(ValueError):
            self.session.demo_next()
        self.session.sleep = self.time.sleep
        self.session.demo_resume()
        self.join()
        self.assertEqual(self.session.demo.cursor, 3)
        self.assertEqual(self.session.demo.status, 'complete')

    def test_edit_and_manual_actions_reset_progress(self):
        self.record()
        for edit in (lambda:self.session.demo_name(name='Edited'),
                     lambda:self.session.demo_rename(index=0,name='Start'),
                     lambda:self.session.demo_reorder(index=0,direction=1),
                     lambda:self.session.demo_replace(index=0),
                     lambda:self.session.set_limits(channel=0,low=90,high=660),
                     lambda:self.move(350)):
            self.session.demo_next()
            self.join()
            self.assertEqual(self.session.demo.cursor, 1)
            edit()
            self.assertEqual(self.session.demo.cursor, 0)
            self.assertEqual(self.session.demo.status, 'idle')
        self.session.demo_next()
        self.join()
        self.session.go_pose(name='HOME')
        self.join()
        self.assertEqual(self.session.demo.cursor, 0)

    def test_limits_invalidate_records_and_block_replay_without_clamping(self):
        self.record()
        self.move(320)
        self.session.set_limits(channel=0,low=320,high=340)
        demo = self.session.state()['demo']
        self.assertTrue(demo['positions'][0]['valid'])
        self.assertFalse(demo['positions'][2]['valid'])
        self.assertEqual(demo['positions'][2]['counts'][0], 350)
        before = dict(self.driver.outputs)
        for method in (self.session.demo_next,self.session.demo_run):
            with self.assertRaises(ValueError):
                method()
        self.assertEqual(self.driver.outputs,before)

    def test_playback_busy_rejects_every_conflicting_action(self):
        self.record()
        reached,release = threading.Event(),threading.Event()
        def blocked_sleep(seconds):
            reached.set()
            release.wait(timeout=2)
            self.time.sleep(seconds)
        self.session.sleep = blocked_sleep
        self.session.demo_run()
        self.assertTrue(reached.wait(timeout=2))
        operations = [lambda:self.session.go_pose(name='HOME'),
            lambda:self.session.set_limits(channel=0,low=90,high=660),
            lambda:self.session.move(channel=0,target=350,first_clear=False),
            self.session.demo_add,lambda:self.session.demo_remove(index=0),
            self.session.demo_restart,self.session.demo_save,self.session.demo_next]
        try:
            for operation in operations:
                with self.assertRaises(ValueError):
                    operation()
        finally:
            self.session.stop()
            release.set()
        self.join()

    def test_stop_mid_step_prevents_later_steps_and_clears_cursor_authority(self):
        self.record()
        targets = []
        execute = self.session.runner.execute
        def traced(start,target,limits,**kwargs):
            targets.append(target[0])
            return execute(start,target,limits,**kwargs)
        self.session.runner.execute = traced
        stopped = False
        def stop_during_move(seconds):
            nonlocal stopped
            self.time.sleep(seconds)
            if not stopped:
                stopped=True
                self.session.stop()
        self.session.sleep = stop_during_move
        self.session.demo_run()
        self.join()
        self.assertEqual(targets, [320])
        self.assertEqual(self.session.demo.cursor, 0)
        self.assertEqual(self.session.demo.status, 'idle')
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))
        with self.assertRaises(ValueError):
            self.session.demo_resume()

    def test_driver_failure_does_not_advance_or_run_recovery_pose(self):
        self.record()
        calls=[]
        def fail(channel,count):
            calls.append((channel,count))
            raise OSError('Synthetic I2C failure')
        self.driver.write=fail
        self.session.demo_run()
        self.join()
        self.assertEqual(len(calls),1)
        self.assertFalse(self.session.armed)
        self.assertEqual(self.session.demo.cursor,0)
        self.assertTrue(self.session.outputs_off)
        self.assertIn('Demo stopped',self.session.error)

    def test_stop_after_endpoint_still_prevents_progress_and_next_step(self):
        self.record()
        targets=[]
        execute=self.session.runner.execute
        def stop_at_boundary(start,target,limits,**kwargs):
            targets.append(dict(target))
            result=execute(start,target,limits,**kwargs)
            self.session.stop()
            return result
        self.session.runner.execute=stop_at_boundary
        self.session.demo_run()
        self.join()
        self.assertEqual(len(targets),1)
        self.assertEqual(self.session.demo.cursor,0)
        self.assertTrue(self.session.outputs_off)
        self.assertTrue(all(v is None for v in self.session.commands.values()))

    def test_pause_on_final_position_completes_without_a_resume_cursor(self):
        self.power_on()
        self.session.demo_add()
        self.move(340)
        requested=False
        def pause(seconds):
            nonlocal requested
            self.time.sleep(seconds)
            if not requested:
                requested=True
                self.session.demo_pause()
        self.session.sleep=pause
        self.session.demo_run()
        self.join()
        self.assertEqual(self.session.demo.cursor,1)
        self.assertEqual(self.session.demo.status,'complete')
        self.assertFalse(self.session.demo.pause_requested)
        with self.assertRaises(ValueError):
            self.session.demo_resume()

    def test_finite_move_and_duration_caps_preflight_entire_run(self):
        self.record()
        before=dict(self.driver.outputs)
        self.session.budget.max_moves=self.session.budget.accepted_targets+2
        with self.assertRaises(ValueError):
            self.session.demo_run()
        self.assertEqual(self.driver.outputs,before)
        self.session.budget.max_moves=None
        self.session.budget.deadline=self.time.now+.1
        with self.assertRaises(ValueError):
            self.session.demo_run()
        self.assertEqual(self.driver.outputs,before)
        self.session.budget.deadline=None
        self.session.budget.max_moves=self.session.budget.accepted_targets+3
        self.session.demo_run()
        self.join()
        self.assertEqual(self.session.state()['remaining_moves'],0)
        with self.assertRaises(ValueError):
            self.session.go_pose(name='HOME')

    def test_deadline_reached_during_move_disables_and_aborts_remaining_steps(self):
        self.record()
        self.session.budget.deadline=self.time.now+100
        fired=False
        def expire(seconds):
            nonlocal fired
            self.time.sleep(seconds)
            if not fired:
                fired=True
                self.time.now=self.session.budget.deadline
        self.session.sleep=expire
        self.session.demo_run()
        self.join()
        self.assertFalse(self.session.armed)
        self.assertTrue(self.session.outputs_off)
        self.assertEqual(self.session.demo.cursor,0)

    def test_saved_demos_are_immutable_scoped_and_reload_without_motion(self):
        self.record()
        self.session.demo_name(name='Saved sequence')
        first=self.session.demo_save()
        path=self.root/'demos/synthetic'/first
        original=path.read_bytes()
        second=self.session.demo_save()
        self.assertNotEqual(first,second)
        self.assertEqual(path.read_bytes(),original)
        record=json.loads(original)
        self.assertEqual(record['scope'],'synthetic')
        self.assertFalse(record['position_feedback'])
        self.assertFalse(record['validated_limits'])
        self.session.demo_next()
        self.join()
        before=dict(self.driver.outputs)
        self.session.demo_load(filename=first)
        self.assertEqual(self.session.demo.title,'Saved sequence')
        self.assertEqual(self.session.demo.cursor,0)
        self.assertEqual(self.driver.outputs,before)
        self.assertEqual(len(self.session.state()['demo']['saved']),2)
        self.session.stop()
        self.session.demo_load(filename=first)
        self.session.set_limits(channel=0,low=320,high=340)
        self.assertFalse(self.session.state()['demo']['positions'][2]['valid'])
        self.assertEqual(self.session.demo.positions[2]['counts'][0],350)

    def test_scoped_load_rejects_traversal_cross_scope_and_malformed_records(self):
        self.record()
        filename=self.session.demo_save()
        store=DemoStore(self.root/'demos','commissioning',{c:(v[1],v[2]) for c,v in app.JOINTS.items()})
        store.directory.mkdir(parents=True)
        (store.directory/filename).write_bytes((self.root/'demos/synthetic'/filename).read_bytes())
        for name in (filename,'../synthetic/'+filename,True):
            with self.assertRaises(ValueError):
                store.load(name)
        (store.directory/('0'*32+'.json')).write_text('{bad json')
        self.assertEqual(store.saved(),[])
        original=list(self.session.demo.positions)
        with self.assertRaises(ValueError):
            self.session.demo_load(filename='../bad.json')
        self.assertEqual(self.session.demo.positions,original)

    def test_commissioning_storage_is_separate_and_loaded_state_is_a_copy(self):
        self.record()
        preview_filename=self.session.demo_save()
        commissioning_driver=app.PreviewDriver()
        commissioning_driver.scope='commissioning'
        session=app.session_for(commissioning_driver,list(range(6)),directory=self.root/'poses',demo_directory=self.root/'demos',
            clock=self.time.clock,sleep=self.time.sleep)
        self.assertEqual(session.state()['demo']['saved'],[])
        with self.assertRaises(ValueError):
            session.demo_load(filename=preview_filename)
        session.power_on(park_confirmed=True)
        session.worker.join(timeout=2)
        session.demo_add(name='Hardware-scoped fake')
        filename=session.demo_save()
        self.assertTrue((self.root/'demos/commissioning'/filename).exists())
        self.assertEqual(len(session.state()['demo']['saved']),1)
        state=session.state()
        state['demo']['positions'][0]['counts'][0]=90
        self.assertEqual(session.demo.positions[0]['counts'][0],app.HOME[0])
        session.close()

    def test_partial_session_cannot_capture_or_execute_full_demo(self):
        partial=app.session_for(app.PreviewDriver(),[4],directory=self.root/'poses',demo_directory=self.root/'partial')
        partial.prepare(supported=True,power_off=True)
        partial.arm(ready=True,supported=True,switch_ready=True)
        for operation in (partial.demo_add,lambda:partial.go_pose(name='HOME')):
            with self.assertRaises(ValueError):
                operation()
        self.assertTrue(all(v==0 for v in partial.driver.outputs.values()))
        partial.close()


if __name__ == '__main__':
    unittest.main()
