"""Offline checks for the single-CANable upstream interface defaults."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

SRC = Path(__file__).resolve().parents[2]


class CanInterfaceDefaults(unittest.TestCase):
    def test_medicine_box_launch_default_and_forwarding(self):
        root = ET.parse(SRC / 'robot_navigation/launch/mission_complete.launch').getroot()
        arg = root.find("arg[@name='medicine_box_can_device']")
        self.assertEqual(arg.get('default'), 'can0')
        node = next(n for n in root.iter('node') if n.get('name') == 'open_medicine_box_node')
        self.assertEqual(node.find("param[@name='can_device']").get('value'),
                         '$(arg medicine_box_can_device)')

    def test_standalone_cpp_node_fallbacks(self):
        for name in ['open_medicine_box.cpp', 'display_node.cpp']:
            with self.subTest(node=name):
                text = (SRC / 'robot_navigation/src' / name).read_text(encoding='utf-8')
                self.assertIn('can_device_("can0")', text)
                self.assertIn('pnh_.param<std::string>("can_device", can_device_, "can0")', text)

    def test_yaml_can_device_defaults(self):
        for relative in ['robot_navigation/config/mission_params.yaml',
                         'robot_navigation/config/navigation_params.yaml',
                         'arm_and_gripper/config/arm_and_gripper.yaml']:
            with self.subTest(config=relative):
                text = (SRC / relative).read_text(encoding='utf-8')
                values = re.findall(r'^\s*can_device:\s*[\"\']?([\w]+)', text, re.M)
                self.assertTrue(values)
                self.assertEqual(set(values), {'can0'})

    def test_keyboard_bridge_default_unchanged(self):
        root = ET.parse(SRC / 'robot_bringup/launch/y42_stm32_keyboard_test.launch').getroot()
        self.assertEqual(root.find("arg[@name='can_device']").get('default'), 'can0')
        self.assertEqual(root.find("arg[@name='motor_ids']").get('default'), '[2, 1, 4, 3]')

    def test_legacy_examples_use_current_interface(self):
        for relative in ['arm_and_gripper/README.md', 'arm_and_gripper/launch/arm_and_gripper.launch',
                         'docs/mission_system_testing.md', 'docs/机器人导航系统架构说明.md',
                         'robot_navigation/include/robot_navigation/can_utils.h']:
            with self.subTest(example=relative):
                text = (SRC / relative).read_text(encoding='utf-8')
                self.assertIsNone(re.search(r'\bcan' + '1' + r'\b', text))


if __name__ == '__main__':
    unittest.main()
