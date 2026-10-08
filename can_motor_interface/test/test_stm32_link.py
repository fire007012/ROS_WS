"""Real roslaunch expansion and read-only link diagnosis, without hardware."""
import importlib.util
from pathlib import Path
import socket
import unittest
from unittest.mock import patch

import roslaunch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('check_link', ROOT / 'can_motor_interface/scripts/check_stm32_can.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


class LinkDiagnosis(unittest.TestCase):
    def link(self):
        return {'flags': ['UP'], 'linkinfo': {'info_kind': 'can', 'info_data': {
            'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 500000}}}}

    def test_missing_down_busoff_wrong_bitrate(self):
        link = self.link()
        self.assertEqual(check.link_problems(link, 500000), [])
        self.assertTrue(check.link_problems(link, 1000000))
        link['flags'] = []
        link['linkinfo']['info_data']['state'] = 'BUS-OFF'
        self.assertEqual(len(check.link_problems(link, 1000000)), 3)
        link['linkinfo']['info_kind'] = 'vcan'
        self.assertEqual(len(check.link_problems(link, 1000000)), 4)

    def test_local_echo_is_never_stm32_feedback(self):
        self.assertTrue(check.is_stm32_reply(check.EFF | 0x181, 8, 0, 0x181, True))
        self.assertFalse(check.is_stm32_reply(check.EFF | 0x181, 8, socket.MSG_DONTROUTE, 0x181, True))
        self.assertFalse(check.is_stm32_reply(0x181, 8, 0, 0x181, True))
        self.assertFalse(check.is_stm32_reply(check.EFF | 0x181, 8, 0, 0x181, False))
        for flag in (check.RTR, check.ERR):
            self.assertFalse(check.is_stm32_reply(check.EFF | flag | 0x181, 8, 0, 0x181, True))

    def test_controller_modes_for_iproute2_formats(self):
        for modes in (['LISTEN-ONLY', 'LOOPBACK'], {'listen-only': True, 'loopback': True}):
            link = self.link()
            link['linkinfo']['info_data']['ctrlmode'] = modes
            self.assertEqual(len(check.link_problems(link, 500000)), 2)


class LaunchProtocol(unittest.TestCase):
    def load(self, package, launch, argv):
        original = roslaunch.substitution_args._find

        def find(resolved, expression, args, context):
            if (ROOT / args[0] / 'package.xml').is_file():
                return resolved.replace('$(' + expression + ')', str(ROOT / args[0]))
            return original(resolved, expression, args, context)

        config = roslaunch.config.ROSLaunchConfig()
        with patch.object(roslaunch.substitution_args, '_find', find):
            roslaunch.xmlloader.XmlLoader().load(str(ROOT / package / 'launch' / launch),
                                                config, argv=argv, verbose=False)
        return config

    def assert_protocol(self, config, tx, rx, extended, device='can0'):
        nodes = [n for n in config.nodes if n.package == 'can_motor_interface']
        self.assertEqual([n.type for n in nodes], ['can_interface_node'])
        for name, value in [('tx_can_id', tx), ('rx_can_id', rx),
                            ('use_extended_frame', extended), ('can_device', device)]:
            self.assertEqual(config.params['/can_interface_node/' + name].value, value)

    def test_documented_custom_parser_entry_not_native_y42(self):
        config = self.load('robot_bringup', 'stm32_custom_keyboard_test.launch', [])
        self.assert_protocol(config, 0x100, 0x101, False)
        self.assertEqual(config.params['/can_interface_node/report_mask'].value, 7)
        self.assertEqual(config.params['/can_interface_node/acceleration_rpm_s'].value, 50)
        self.assertEqual(config.params['/can_interface_node/max_rpm'].value, 30.0)
        self.assertEqual(config.params['/planner_node/max_rpm'].value, 30.0)
        self.assertFalse(any(n.package in ('arm_interface', 'arm_and_gripper') for n in config.nodes))

    def test_interface_and_limits_can_be_selected_explicitly(self):
        config = self.load('robot_bringup', 'stm32_custom_keyboard_test.launch',
                           ['tx_can_id:=256', 'rx_can_id:=257', 'use_extended_frame:=false',
                            'can_device:=vcan9', 'max_rpm:=20', 'acceleration_rpm_s:=60'])
        self.assert_protocol(config, 0x100, 0x101, False, 'vcan9')
        self.assertEqual(config.params['/planner_node/max_rpm'].value, 20)
        self.assertEqual(config.params['/can_interface_node/acceleration_rpm_s'].value, 60)

    def test_all_custom_launches_forward_firmware_configuration(self):
        entries = [('can_motor_interface', 'can_interface.launch', []),
                   ('robot_bringup', 'robot_bringup.launch', ['enable_arm:=false', 'enable_teleop:=false']),
                   ('robot_navigation', 'complete_navigation.launch', []),
                   ('robot_navigation', 'mission_complete.launch', []),
                   ('robot_navigation', 'dynamic_navigation.launch', ['map_file:=/tmp/arena.yaml'])]
        for package, launch, extra in entries:
            with self.subTest(launch=launch):
                config = self.load(package, launch, extra + ['can_device:=vcan9',
                                   'motor_indices:=[0,1,2,3]', 'direction_signs:=[-1,1,-1,1]'])
                self.assert_protocol(config, 0x100, 0x101, False, 'vcan9')
                self.assertEqual(config.params['/can_interface_node/motor_indices'].value, [0, 1, 2, 3])
                self.assertEqual(config.params['/can_interface_node/direction_signs'].value, [-1, 1, -1, 1])

    def test_default_custom_wheel_order_matches_native_test(self):
        config = self.load('robot_bringup', 'stm32_custom_keyboard_test.launch', [])
        self.assertEqual(config.params['/can_interface_node/motor_indices'].value, [1, 0, 3, 2])
        self.assertEqual(config.params['/can_interface_node/direction_signs'].value, [1, 1, 1, 1])

    def test_arm_launch_selects_single_axis_on_same_bus(self):
        config = self.load('robot_bringup', 'robot_bringup.launch',
                           ['enable_teleop:=false', 'can_device:=vcan9'])
        self.assertEqual(config.params['/arm_controller_node/can_device'].value, 'vcan9')
        self.assertEqual(config.params['/arm_controller_node/joint_name'].value, 'joint_1')
        self.assertEqual(config.params['/arm/joint_count'].value, 1)
        self.assertNotIn('/arm_controller_node/tx_can_base_id', config.params)

    def test_arm_joint_and_sequence_control_do_not_start_together(self):
        for package, launch, args in [
                ('robot_navigation', 'mission_complete.launch', []),
                ('robot_navigation', 'mission_complete.launch', ['arm_joint_control:=true']),
                ('robot_navigation', 'complete_navigation.launch', ['enable_arm:=true', 'enable_arm_and_gripper:=true']),
                ('robot_bringup', 'robot_bringup.launch', ['enable_arm_and_gripper:=true', 'enable_teleop:=false'])]:
            with self.subTest(launch=launch, args=args):
                config = self.load(package, launch, args)
                owners = [n for n in config.nodes if n.type in ('arm_controller_node', 'arm_and_gripper_node')]
                self.assertEqual(len(owners), 1)


if __name__ == '__main__':
    unittest.main()
