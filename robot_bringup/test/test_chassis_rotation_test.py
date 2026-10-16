"""Offline lifecycle checks: no ROS master, CAN socket or motor is used."""
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/chassis_rotation_test.py'
spec = importlib.util.spec_from_file_location('chassis_rotation_test', SCRIPT)
test = importlib.util.module_from_spec(spec)
spec.loader.exec_module(test)


def healthy(now, state='CACHE_READY'):
    feedback = test.Feedback()
    feedback.ready = True
    feedback.ready_at = feedback.rpm_at = now
    feedback.rpm = [0.0] * 4
    feedback.robot_state = state
    return feedback


class RotationTest(unittest.TestCase):
    def test_duration_starts_after_physical_start_and_then_stops_commands(self):
        rotation = test.Rotation(8, 3, 30)
        self.assertEqual([8] * 4, rotation.command(healthy(10), 10))
        self.assertEqual([8] * 4, rotation.command(healthy(20), 20))
        self.assertEqual([8] * 4, rotation.command(healthy(21, 'RUNNING'), 21))
        self.assertEqual([8] * 4, rotation.command(healthy(23.9, 'RUNNING'), 23.9))
        self.assertIsNone(rotation.command(healthy(24, 'RUNNING'), 24))

    def test_button_timeout_and_failed_start_never_return_another_target(self):
        rotation = test.Rotation(8, 3, 5)
        rotation.command(healthy(10), 10)
        with self.assertRaisesRegex(RuntimeError, '按钮超时'):
            rotation.command(healthy(15), 15)
        for state in ('START_FAILED', 'ESTOP_LATCHED', 'HEARTBEAT_LOST'):
            with self.subTest(state=state), self.assertRaises(RuntimeError):
                test.Rotation(8, 3, 30).command(healthy(10, state), 10)

    def test_feedback_loss_estop_overspeed_and_running_state_loss_abort(self):
        rotation = test.Rotation(8, 3, 30)
        rotation.command(healthy(10, 'RUNNING'), 10)
        for feedback in (healthy(9), healthy(10, 'CACHE_READY')):
            with self.assertRaises(RuntimeError):
                rotation.command(feedback, 10)
        feedback = healthy(10, 'RUNNING')
        feedback.estop = True
        with self.assertRaisesRegex(RuntimeError, '急停'):
            rotation.command(feedback, 10)
        for rpm in ([0, 0, 16, 0], [0, float('nan'), 0, 0], [0, 0, 0]):
            feedback = healthy(10, 'RUNNING')
            feedback.rpm = rpm
            with self.assertRaises(RuntimeError):
                rotation.command(feedback, 10)

    def test_stop_requires_new_fresh_finite_feedback_for_all_four_wheels(self):
        feedback = healthy(10)
        self.assertFalse(feedback.stopped(10.1, 10))
        feedback.rpm_at = 10.1
        self.assertTrue(feedback.stopped(10.2, 10))
        self.assertFalse(feedback.stopped(10.7, 10))
        for rpm in ([0, 0, 2, 0], [0, 0, float('nan'), 0], [0, 0, 0]):
            feedback.rpm = rpm
            self.assertFalse(feedback.stopped(10.2, 10))

    def test_up_interface_is_reused_and_incorrect_active_config_is_not_reset(self):
        link = {'flags': ['UP'], 'linkinfo': {'info_kind': 'can', 'info_data': {
            'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 500000}}}}
        with mock.patch.object(test.subprocess, 'check_output', return_value=json.dumps([link])), \
                mock.patch.object(test.subprocess, 'run') as run:
            test.configure_can('can0')
            run.assert_not_called()
            link['linkinfo']['info_data']['state'] = 'BUS-OFF'
            with mock.patch.object(test.subprocess, 'check_output', return_value=json.dumps([link])):
                with self.assertRaises(RuntimeError):
                    test.configure_can('can0')
            run.assert_not_called()

    def test_other_controller_is_rejected_before_launch(self):
        publishers = [['/motor_velocity_cmd', ['/planner_node']]]
        with mock.patch.object(test, 'master_call', return_value=[1, '', [publishers, [], []]]):
            with self.assertRaisesRegex(RuntimeError, '现有底盘控制程序'):
                test.check_existing_controllers('http://localhost:11311')

    def test_down_interface_configuration_uses_sudo_only_for_ip_commands(self):
        link = {'flags': [], 'linkinfo': {'info_kind': 'can', 'info_data': {}}}
        with mock.patch.object(test.subprocess, 'check_output', return_value=json.dumps([link])), \
                mock.patch.object(test.subprocess, 'run') as run, \
                mock.patch.object(test.os, 'geteuid', return_value=1000):
            test.configure_can('can0')
        self.assertEqual([
            mock.call(['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate',
                       '500000', 'sample-point', '0.833', 'loopback', 'off',
                       'listen-only', 'off'], check=True),
            mock.call(['sudo', 'ip', 'link', 'set', 'can0', 'up'], check=True)], run.call_args_list)

    def test_owned_launch_group_cleanup_escalates_if_it_does_not_exit(self):
        proc = mock.Mock(pid=12345)
        proc.poll.return_value = None
        proc.wait.side_effect = [subprocess.TimeoutExpired('roslaunch', 8), None]
        with mock.patch.object(test.os, 'killpg') as kill:
            test.shutdown_launch(proc)
        self.assertEqual([mock.call(12345, signal.SIGINT), mock.call(12345, signal.SIGTERM)],
                         kill.call_args_list)


if __name__ == '__main__':
    unittest.main()
