"""Check shared AV settings and, when available, the real ROS launch loader.

No ROS master, camera, audio device or running node is needed.
"""
from pathlib import Path
import os
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

try:
    import roslaunch.config
    import roslaunch.substitution_args
    import roslaunch.xmlloader
    HAS_ROSLAUNCH = True
except ImportError:
    HAS_ROSLAUNCH = False

PACKAGE = Path(__file__).resolve().parents[1]
SETTINGS = PACKAGE / 'config/audio_video_defaults.launch'
PEER = PACKAGE / 'launch/audio_video_peer.launch'
NETWORK = dict(ROS_MASTER_URI='http://10.161.170.93:11311',
               ROS_IP='10.161.170.93', ROBOT_PI_IP='10.161.170.93',
               ROBOT_VM_IP='10.161.170.94')


class AVSettingsStructure(unittest.TestCase):
    def test_settings_define_every_public_peer_argument(self):
        settings = ET.parse(SETTINGS).getroot()
        peer = ET.parse(PEER).getroot()
        names = [arg.get('name') for arg in settings.findall('arg')]
        self.assertEqual(len(names), len(set(names)))
        required = {arg.get('name') for arg in peer.findall('arg')
                    if arg.get('value') is None}
        self.assertEqual(set(names), required)
        self.assertEqual(settings.find("arg[@name='enable_http']").get('default'), 'false')
        self.assertEqual(settings.find('include').get('pass_all_args'), 'true')

    def test_wrappers_fix_roles_and_forward_cli(self):
        for role in ('master', 'slave'):
            with self.subTest(role=role):
                root = ET.parse(PACKAGE / ('launch/audio_video_' + role + '.launch')).getroot()
                self.assertEqual(root.find("arg[@name='role']").get('value'), role)
                self.assertEqual(root.find('include').get('pass_all_args'), 'true')
                self.assertEqual(root.find("arg[@name='config_file']").get('default'),
                                 '$(find robot_navigation)/config/audio_video_defaults.launch')
                self.assertFalse(root.findall('node'))

    def test_audio_parameter_names_and_configurable_values(self):
        peer = ET.parse(PEER).getroot()
        audio = peer.find("node[@name='audio_chat_$(arg role)']")
        params = {p.get('name'): p.get('value') for p in audio.findall('param')}
        for name in ('role', 'local_ip', 'remote_ip', 'rx_port', 'tx_port',
                     'mic_device', 'speaker_device'):
            self.assertEqual(params[name], '$(arg ' + name + ')')
        self.assertFalse(any('$(' in name for name in params))


@unittest.skipUnless(HAS_ROSLAUNCH, 'ROS launch loader is not installed')
class AVLaunchLoading(unittest.TestCase):
    def load_peer(self, role, argv=None, environment=None):
        env = dict(NETWORK)
        if role == 'master':
            env['ROS_IP'] = env['ROBOT_VM_IP']
        if environment:
            env.update(environment)

        def find_package(resolved, expression, args, context):
            if args != ['robot_navigation']:
                raise ValueError('Unexpected package lookup: ' + str(args))
            return resolved.replace('$(' + expression + ')', str(PACKAGE))

        config = roslaunch.config.ROSLaunchConfig()
        # Adapt package lookup for source-only checkouts; all XML, substitution,
        # include forwarding, conditionals, types and nodes use the real loader.
        with patch.dict(os.environ, env, clear=True), \
                patch.object(roslaunch.substitution_args, '_find', find_package):
            roslaunch.xmlloader.XmlLoader().load(
                str(PACKAGE / ('launch/audio_video_' + role + '.launch')),
                config, argv=argv or [], verbose=False)
        return config

    def value(self, config, node, param):
        return config.params['/' + node + '/' + param].value

    def test_default_nodes_and_network_on_both_roles(self):
        for role, camera, view, local, remote in (
            ('slave', 'lower_usb_cam', 'view_upper_cam', '10.161.170.93', '10.161.170.94'),
            ('master', 'upper_usb_cam', 'view_lower_cam', '10.161.170.94', '10.161.170.93'),
        ):
            with self.subTest(role=role):
                c = self.load_peer(role)
                self.assertEqual({n.name for n in c.nodes},
                                 {camera, 'video_sender_' + role, 'video_stream_' + role,
                                  view, 'audio_chat_' + role})
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'role'), role)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'local_ip'), local)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'remote_ip'), remote)
                self.assertEqual(self.value(c, 'video_stream_' + role, 'enable_http'), False)
                for n in c.nodes:
                    self.assertEqual(dict(n.env_args)['ROS_IP'], local)
                    self.assertEqual(dict(n.env_args)['ROS_MASTER_URI'], NETWORK['ROS_MASTER_URI'])

    def test_topics_and_namespaces_remain_compatible(self):
        for role, local, remote in (('slave', 'lower', 'upper'), ('master', 'upper', 'lower')):
            with self.subTest(role=role):
                c = self.load_peer(role)
                self.assertEqual(self.value(c, 'video_sender_' + role, 'camera_topic'),
                                 '/' + local + '_cam/' + local + '_usb_cam/image_raw')
                self.assertEqual(self.value(c, 'video_sender_' + role, 'compressed_topic'),
                                 '/' + local + '_cam/network/compressed')
                self.assertEqual(self.value(c, 'video_stream_' + role, 'camera_topic'),
                                 '/' + remote + '_cam/network/compressed')
                self.assertEqual(self.value(c, 'video_stream_' + role, 'local_display_topic'),
                                 '/camera/' + remote + '_display')
                camera = next(n for n in c.nodes if n.name == local + '_usb_cam')
                self.assertEqual(camera.namespace, '/' + local + '_cam/')

    def test_audio_only_cli_override(self):
        for role in ('slave', 'master'):
            self.assertEqual([n.name for n in self.load_peer(role, ['enable_video:=false']).nodes],
                             ['audio_chat_' + role])

    def test_camera_off_cli_override(self):
        for role, camera_arg in (('slave', 'start_lower_cam'), ('master', 'start_upper_cam')):
            c = self.load_peer(role, [camera_arg + ':=false'])
            self.assertNotIn('video_sender_' + role, [n.name for n in c.nodes])
            self.assertIn('video_stream_' + role, [n.name for n in c.nodes])

    def test_simulated_camera_and_video_only(self):
        for role in ('slave', 'master'):
            c = self.load_peer(role, ['use_test_camera:=true', 'enable_audio:=false'])
            camera = next(n for n in c.nodes if n.name.endswith('_usb_cam'))
            self.assertEqual(camera.type, 'test_camera.py')
            self.assertFalse(any(n.package == 'usb_cam' for n in c.nodes))
            self.assertNotIn('audio_chat_' + role, [n.name for n in c.nodes])

    def test_no_display_with_http_and_browser(self):
        for role in ('slave', 'master'):
            c = self.load_peer(role, ['enable_display:=false', 'enable_http:=true', 'enable_browser:=true'])
            self.assertFalse(any(n.package == 'image_view' for n in c.nodes))
            self.assertEqual(self.value(c, 'video_stream_' + role, 'enable_http'), True)
            self.assertIn('video_stream_' + role + '_http2', [n.name for n in c.nodes])

    def test_camera_audio_and_network_parameter_overrides(self):
        for role, device_arg, camera_ns in (
            ('slave', 'lower_cam_device', 'lower_cam'), ('master', 'upper_cam_device', 'upper_cam')
        ):
            with self.subTest(role=role):
                c = self.load_peer(role, [device_arg + ':=/dev/video2', 'cam_width:=640',
                                         'frame_duration_ms:=10', 'rx_port:=6004', 'tx_port:=6005',
                                         'mic_device:=test_mic', 'remote_ip:=10.0.0.20'])
                self.assertEqual(c.params['/' + camera_ns + '/' + camera_ns.replace('_cam', '_usb_cam')
                                          + '/video_device'].value, '/dev/video2')
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'frame_duration_ms'), 10)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'rx_port'), 6004)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'tx_port'), 6005)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'mic_device'), 'test_mic')
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'remote_ip'), '10.0.0.20')

    def test_custom_settings_file(self):
        root = ET.parse(SETTINGS).getroot()
        root.find("arg[@name='cam_width']").set('default', '640')
        root.find("arg[@name='enable_display']").set('default', 'false')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'custom.launch'
            ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
            c = self.load_peer('slave', ['config_file:=' + str(path)])
        self.assertEqual(self.value(c, 'video_sender_slave', 'stream_width'), 1280)
        self.assertEqual(c.params['/lower_cam/lower_usb_cam/image_width'].value, 640)
        self.assertNotIn('view_upper_cam', [n.name for n in c.nodes])

    def test_changed_network_from_environment(self):
        c = self.load_peer('slave', environment=dict(ROS_MASTER_URI='http://10.0.0.10:11311',
                           ROS_IP='10.0.0.10', ROBOT_VM_IP='10.0.0.11'))
        self.assertEqual(self.value(c, 'audio_chat_slave', 'remote_ip'), '10.0.0.11')
        self.assertEqual(dict(c.nodes[0].env_args)['ROS_MASTER_URI'], 'http://10.0.0.10:11311')

    def test_missing_required_environment_has_clear_error(self):
        for name in NETWORK:
            # Only the role-specific remote variable is required by the slave.
            if name == 'ROBOT_PI_IP':
                continue
            env = dict(NETWORK)
            del env[name]
            with self.subTest(variable=name), patch.dict(os.environ, env, clear=True), \
                    patch.object(roslaunch.substitution_args, '_find',
                                 lambda resolved, expr, args, ctx: resolved.replace('$(' + expr + ')', str(PACKAGE))):
                with self.assertRaises(roslaunch.xmlloader.XmlParseException) as error:
                    roslaunch.xmlloader.XmlLoader().load(
                        str(PACKAGE / 'launch/audio_video_slave.launch'),
                        roslaunch.config.ROSLaunchConfig(), argv=[], verbose=False)
                self.assertIn(name, str(error.exception))


if __name__ == '__main__':
    unittest.main()
