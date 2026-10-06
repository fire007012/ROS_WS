"""Offline mocks only: never access GPIO, start ROS, or install a service."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

spec = importlib.util.spec_from_file_location('button_autostart',
    Path(__file__).resolve().parents[1] / 'scripts/button_autostart.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class ButtonAutostart(unittest.TestCase):
    def test_wait_requires_exact_node_name(self):
        result = SimpleNamespace(stdout='/not_mission_controller_node\n/mission_controller_node_extra\n')
        with patch.object(m.time, 'monotonic', side_effect=[0, 0, 2]), \
             patch.object(m.time, 'sleep'), patch.object(m.subprocess, 'run', return_value=result):
            self.assertFalse(m.wait_for_node('/mission_controller_node', 1, {}))
        result.stdout = '/mission_controller_node\n'
        with patch.object(m.time, 'monotonic', side_effect=[0, 0]), \
             patch.object(m.subprocess, 'run', return_value=result):
            self.assertTrue(m.wait_for_node('/mission_controller_node', 1, {}))

    def test_wait_handles_subprocess_timeout(self):
        result = SimpleNamespace(stdout='/mission_controller_node\n')
        with patch.object(m.time, 'monotonic', side_effect=[0, 0, 0.1]), \
             patch.object(m.time, 'sleep'), \
             patch.object(m.subprocess, 'run', side_effect=[subprocess.TimeoutExpired('rosnode', 3), result]):
            self.assertTrue(m.wait_for_node('/mission_controller_node', 1, {}))

    def test_start_pulse_and_failure(self):
        with patch.object(m.subprocess, 'run', return_value=SimpleNamespace(returncode=0)) as run:
            self.assertTrue(m.publish_start(0x5A17, {}))
            self.assertEqual(run.call_args[0][0], ['rostopic', 'pub', '-1', '/start_signal/physical',
                'std_msgs/UInt32', '{data: 23063}'])
        with patch.object(m.subprocess, 'run', side_effect=OSError('not installed')):
            self.assertFalse(m.publish_start(0x5A17, {}))

    def test_unknown_gpio_read_is_not_a_press(self):
        button = MagicMock()
        button.read.side_effect = [-1, 0, 0, 0]
        with patch.object(m.time, 'monotonic', side_effect=[0, 0.1, 0.2]), \
             patch.object(m.time, 'sleep') as sleep:
            m.confirmed_press(button, True, 0.05, 0.02)
            sleep.assert_called_once_with(0.02)

    def test_launcher_is_opt_in_and_disables_duplicate_listeners(self):
        button, led, process = MagicMock(), MagicMock(), MagicMock()
        button.read.return_value = 1
        with patch.object(sys, 'argv', ['button_autostart.py']), \
             patch.object(m, 'SysfsGpio', side_effect=[button, led]), \
             patch.object(m, 'confirmed_press', side_effect=[None, KeyboardInterrupt]), \
             patch.object(m, 'wait_for_node', return_value=True), \
             patch.object(m, 'publish_start', return_value=True) as start, \
             patch.object(m, 'terminate_process'), \
             patch.object(m.subprocess, 'Popen', return_value=process) as launch:
            self.assertEqual(m.main(), 0)
            self.assertEqual(launch.call_args[0][0], ['roslaunch', 'robot_navigation',
                'mission_complete.launch', 'enable_physical_start:=false', 'enable_can_start_signal:=false'])
            self.assertTrue(launch.call_args[1]['start_new_session'])
            start.assert_called_once()
            process.wait.assert_called_once()
            button.close.assert_called_once()
            led.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
