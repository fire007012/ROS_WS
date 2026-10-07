import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('keyboard_entry', ROOT / 'scripts/y42_keyboard.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class KeyboardDefaults(unittest.TestCase):
    def test_exec_preserves_terminal_and_defaults(self):
        with patch.object(module.os, 'execvp') as execute, patch.object(module.sys, 'argv', ['y42_keyboard.py']):
            module.main()
        execute.assert_called_once_with('rosrun', ['rosrun', 'teleop_twist_keyboard',
            'teleop_twist_keyboard.py', 'cmd_vel:=/cmd_vel_teleop', '_speed:=0.10',
            '_turn:=0.30', '_repeat_rate:=10.0', '_key_timeout:=0.6'])

    def test_optional_overrides_are_appended(self):
        with patch.object(module.os, 'execvp') as execute, patch.object(module.sys, 'argv', ['y42_keyboard.py', '_speed:=0.05']):
            module.main()
        self.assertEqual(execute.call_args[0][1][-1], '_speed:=0.05')

    def test_bridge_launch_keeps_known_wheel_map(self):
        launch = ET.parse(ROOT / 'launch/y42_stm32_keyboard_test.launch').getroot()
        args = {item.attrib['name']: item.attrib.get('default') for item in launch.findall('arg')}
        self.assertEqual(args['can_device'], 'can0')
        self.assertEqual(args['max_rpm'], '90.0')
        self.assertEqual(args['motor_ids'], '[2, 1, 4, 3]')
        monitor = launch.find("param[@name='/y42_direct_node/monitor_stm32_events']")
        self.assertEqual(monitor.attrib['value'], 'true')
        forwarded = {item.attrib['name'] for item in launch.find('include').findall('arg')}
        self.assertEqual(forwarded, set(args))


if __name__ == '__main__':
    unittest.main()
