"""Check shared AV settings and, when available, the real ROS launch loader.

No ROS master, camera, audio device or running node is needed.
"""
from pathlib import Path
import os
import tempfile
import shutil
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
               ROBOT_VM_IP='10.161.170.41')


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
        self.assertEqual(settings.find("arg[@name='stream_framerate']").get('default'), '10.0')
        self.assertEqual(settings.find("arg[@name='video_display_fps']").get('default'), '10.0')
        self.assertEqual(settings.find('include').get('pass_all_args'), 'true')
        camera = PEER.read_text(encoding='utf-8')
        self.assertIn("<arg name=\"capture_wait_for_ready\" value=\"$(eval not arg('always_on'))\"/>", camera)
        self.assertIn('<arg name="capture_ready_topic" value="$(arg effective_ready_topic)"/>', camera)

    def test_status_gui_defaults_are_explicit_and_not_part_of_peer_nodes(self):
        settings = ET.parse(SETTINGS).getroot()
        self.assertEqual(settings.find("arg[@name='enable_status_gui']").get('default'), 'false')
        master = ET.parse(PACKAGE / 'launch/audio_video_master.launch').getroot()
        self.assertEqual(master.find("arg[@name='enable_status_gui']").get('default'), 'true')
        self.assertIsNotNone(master.find("node[@name='robot_status_display_node']"))

    def test_call_stream_defaults_match_480p_25fps_pi_test(self):
        settings = ET.parse(SETTINGS).getroot()
        for name, expected in [('cam_width','640'),('cam_height','480'),
                               ('cam_framerate','25'),('stream_width','640'),
                               ('stream_height','480'),('stream_framerate','10.0'),
                               ('mic_device','default')]:
            self.assertEqual(settings.find("arg[@name='%s']" % name).get('default'), expected)

    def test_wrappers_fix_roles_and_forward_cli(self):
        for role in ('master', 'slave'):
            with self.subTest(role=role):
                root = ET.parse(PACKAGE / ('launch/audio_video_' + role + '.launch')).getroot()
                self.assertEqual(root.find("arg[@name='role']").get('value'), role)
                self.assertEqual(root.find('include').get('pass_all_args'), 'true')
                self.assertEqual(root.find("arg[@name='config_file']").get('default'),
                                 '$(find robot_navigation)/config/audio_video_defaults.launch')
                if role == 'slave':
                    self.assertFalse(root.findall('node'))
                else:
                    self.assertIsNotNone(root.find("node[@name='robot_status_display_node']"))

    def test_role_ip_defaults_match_pi_master_and_vm_slave(self):
        settings = ET.parse(SETTINGS).getroot()
        self.assertIn("'10.161.170.93' if arg('role') == 'master' else '10.161.170.41'",
                      settings.find("arg[@name='local_ip']").get('default'))
        self.assertIn("'10.161.170.41' if arg('role') == 'master' else '10.161.170.93'",
                      settings.find("arg[@name='remote_ip']").get('default'))

    def test_role_camera_routing_matches_pi_master_and_vm_slave(self):
        peer = ET.parse(PEER).getroot()
        args = {arg.get('name'): arg.get('value') for arg in peer.findall('arg')}
        self.assertEqual(args['local_camera_ns'],
                         "$(eval 'lower_cam' if arg('role') == 'master' else 'upper_cam')")
        self.assertEqual(args['remote_camera_ns'],
                         "$(eval 'upper_cam' if arg('role') == 'master' else 'lower_cam')")
        camera = next(node for node in peer.iter('include')
                      if node.get('file', '').endswith('/config/camera_devices.launch'))
        camera_args = {arg.get('name'): arg.get('value') for arg in camera.findall('arg')}
        self.assertEqual(camera_args['camera_role'],
                         "$(eval 'robot_audio' if arg('role') == 'master' else 'doctor_audio')")

    def test_robot_status_display_uses_robot_master_camera_and_audio_nodes(self):
        mission = ET.parse(PACKAGE / 'launch/mission_complete.launch').getroot()
        status = ET.parse(PACKAGE / 'launch/status_display.launch').getroot()
        include = next(i for i in mission.findall('include')
                       if i.get('file', '').endswith('/launch/status_display.launch'))
        forwarded = {a.get('name'): a.get('value') for a in include.findall('arg')}
        self.assertEqual(forwarded['enable_vitals_bridge'], 'false')
        self.assertEqual(forwarded['audio_role'], '$(arg audio_role)')
        gui = status.find("node[@name='robot_status_display_node']")
        params = {p.get('name'): p.get('value') for p in gui.findall('param')}
        self.assertIn('/camera/upper_display', params['video_topic'])
        self.assertIn('/lower_cam/lower_usb_cam/capture_status', params['call_camera_status_topic'])
        display = (PACKAGE / 'scripts/robot_status_display_node.py').read_text(encoding='utf-8')
        self.assertIn("stream = '/video_sender_' + role", display)
        self.assertIn("audio = '/audio_chat_' + role", display)

    def test_mission_launch_defaults_to_pi_robot_role(self):
        mission = ET.parse(PACKAGE / 'launch/mission_complete.launch').getroot()
        self.assertEqual(mission.find("arg[@name='audio_role']").get('default'), 'master')
        include = mission.find("group[@if=\"$(arg enable_audio_video)\"]/include")
        self.assertEqual(include.find("arg[@name='role']").get('value'), '$(arg audio_role)')

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
        # These cases isolate the AV peers; the dedicated GUI cases enable it explicitly.
        argv = ['enable_status_gui:=false'] + (argv or [])
        env = dict(NETWORK)
        env['ROS_IP'] = env['ROBOT_PI_IP'] if role == 'master' else env['ROBOT_VM_IP']
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
            ('master', 'lower_usb_cam', 'view_upper_cam', '10.161.170.93', '10.161.170.41'),
            ('slave', 'upper_usb_cam', 'view_lower_cam', '10.161.170.41', '10.161.170.93'),
        ):
            with self.subTest(role=role):
                c = self.load_peer(role)
                expected_nodes = {camera, 'video_sender_' + role,
                                  'video_stream_' + role, 'audio_chat_' + role, 'audio_video_ready_' + role}
                if role == 'slave':
                    expected_nodes.add(view)
                self.assertEqual({n.name for n in c.nodes}, expected_nodes)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'role'), role)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'local_ip'), local)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'remote_ip'), remote)
                self.assertEqual(self.value(c, 'video_stream_' + role, 'enable_http'), False)
                self.assertEqual(self.value(c, 'audio_chat_' + role, 'ready_topic'), '/audio_video/ready')
                for n in c.nodes:
                    self.assertEqual(dict(n.env_args)['ROS_IP'], local)
                    self.assertEqual(dict(n.env_args)['ROS_MASTER_URI'], NETWORK['ROS_MASTER_URI'])

    def test_topics_and_namespaces_remain_compatible(self):
        for role, local, remote in (('master', 'lower', 'upper'), ('slave', 'upper', 'lower')):
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
                             ['audio_video_ready_' + role, 'audio_chat_' + role])

    def test_camera_off_cli_override(self):
        for role, camera_arg in (('master', 'start_lower_cam'), ('slave', 'start_upper_cam')):
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
            ('master', 'lower_cam_device', 'lower_cam'), ('slave', 'upper_cam_device', 'upper_cam')
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
        self.assertEqual(self.value(c, 'video_sender_slave', 'stream_width'), 640)
        self.assertEqual(c.params['/upper_cam/upper_usb_cam/low_width'].value, 640)
        self.assertNotIn('view_lower_cam', [n.name for n in c.nodes])

    def test_changed_network_from_environment(self):
        c = self.load_peer('slave', environment=dict(ROS_MASTER_URI='http://10.0.0.10:11311',
                           ROS_IP='10.0.0.10', ROBOT_VM_IP='10.0.0.11'))
        self.assertEqual(self.value(c, 'audio_chat_slave', 'remote_ip'), '10.161.170.93')
        self.assertEqual(dict(c.nodes[0].env_args)['ROS_MASTER_URI'], 'http://10.0.0.10:11311')

    def test_network_defaults_can_still_be_overridden(self):
        c = self.load_peer('master', ['local_ip:=10.1.1.1', 'remote_ip:=10.1.1.2'])
        self.assertEqual(self.value(c, 'audio_chat_master', 'local_ip'), '10.1.1.1')
        self.assertEqual(self.value(c, 'audio_chat_master', 'remote_ip'), '10.1.1.2')


    def load_mission(self, argv=None):
        def find_package(resolved, expression, args, context):
            package = PACKAGE.parent / args[0]
            if len(args) != 1 or not package.is_dir():
                raise ValueError('Unexpected package lookup: ' + str(args))
            return resolved.replace('$(' + expression + ')', package.as_posix())
        config = roslaunch.config.ROSLaunchConfig()
        with patch.dict(os.environ, NETWORK, clear=True), \
                patch.object(roslaunch.substitution_args, '_find', find_package):
            roslaunch.xmlloader.XmlLoader().load(
                str(PACKAGE / 'launch/mission_complete.launch'), config,
                argv=argv or [], verbose=False)
        return config

    def test_full_mission_has_each_business_node_once_and_scans_by_default(self):
        config = self.load_mission()
        names = [n.name for n in config.nodes]
        for name in ['arm_and_gripper_node', 'open_medicine_box_node',
                     'barcode_qr_detector_node', 'robot_status_display_node',
                     'esp32_vitals_bridge_node', 'speak_node', 'bedside_session_node',
                     'audio_chat_master', 'video_sender_master', 'video_stream_master']:
            self.assertEqual(names.count(name), 1, name)
        self.assertFalse(self.value(config, 'mission_controller_node', 'skip_bedside_scan'))
        self.assertTrue(self.value(config, 'mission_controller_node', 'competition_mode'))
        self.assertEqual(self.value(config, 'bedside_session_node', 'audio_role'), 'master')
        self.assertTrue(self.value(config, 'open_medicine_box_node', 'selection_only'))
        self.assertEqual(self.value(config, 'arm_and_gripper_node', 'can_device'), 'can0')
        # No obsolete Y42 arm node or Pi pigpio servo controller.
        self.assertNotIn('arm_interface_node', names)
        self.assertNotIn('servo_controller_node', names)

    def test_mission_explicit_scan_setting_wins_over_yaml(self):
        # Deliberately supply the old skip=true config so ordering regressions fail.
        text = (PACKAGE / 'config/mission_params.yaml').read_text(encoding='utf-8')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.yaml'
            path.write_text(text.replace('skip_bedside_scan: false',
                                         'skip_bedside_scan: true'), encoding='utf-8')
            for override, expected in [('false', False), ('true', True)]:
                config = self.load_mission(['mission_config:=' + str(path),
                                           'skip_bedside_scan:=' + override])
                self.assertEqual(self.value(config, 'mission_controller_node', 'skip_bedside_scan'), expected)

    def test_mission_routes_serial_topics_and_gui_overrides_to_actual_consumers(self):
        config = self.load_mission(['vitals_serial_device:=/dev/test-esp32',
            'vitals_baudrate:=57600', 'vitals_frame_topic:=/custom/frame',
            'heart_rate_topic:=/custom/hr', 'temperature_topic:=/custom/temp',
            'gui_fullscreen:=false', 'audio_ready_topic:=/custom/ready'])
        self.assertEqual(self.value(config, 'esp32_vitals_bridge_node', 'serial_device'), '/dev/test-esp32')
        self.assertEqual(self.value(config, 'esp32_vitals_bridge_node', 'baudrate'), 57600)
        self.assertFalse(self.value(config, 'robot_status_display_node', 'fullscreen'))
        self.assertEqual(self.value(config, 'bedside_session_node', 'vitals_frame_topic'), '/custom/frame')
        for name in ['esp32_vitals_bridge_node', 'robot_status_display_node']:
            node = next(n for n in config.nodes if n.name == name)
            self.assertEqual(dict(node.remap_args)['/vitals/frame'], '/custom/frame')
            self.assertEqual(dict(node.remap_args)['/vitals/heart_rate'], '/custom/hr')
            self.assertEqual(dict(node.remap_args)['/vitals/temperature'], '/custom/temp')
        node = next(n for n in config.nodes if n.name == 'mission_controller_node')
        self.assertEqual(dict(node.remap_args)['/Ready'], '/custom/ready')
        self.assertEqual(self.value(config, 'audio_chat_master', 'ready_topic'), '/audio_video/ready')

    def test_mission_role_and_av_devices_forward_to_bedside_and_gui(self):
        config = self.load_mission(['audio_role:=slave', 'call_camera_device:=/dev/test-camera',
            'audio_mic_device:=test-mic', 'audio_speaker_device:=test-speaker'])
        self.assertEqual(self.value(config, 'bedside_session_node', 'audio_role'), 'slave')
        self.assertEqual(self.value(config, 'robot_status_display_node', 'video_topic'), '/camera/lower_display')
        self.assertEqual(config.params['/upper_cam/upper_usb_cam/video_device'].value, '/dev/test-camera')
        self.assertEqual(self.value(config, 'audio_chat_slave', 'mic_device'), 'test-mic')
        self.assertEqual(self.value(config, 'audio_chat_slave', 'speaker_device'), 'test-speaker')
        self.assertEqual(self.value(config, 'speak_node', 'speaker_device'), 'test-speaker')

    def test_mission_can_disable_optional_devices_without_duplicates(self):
        config = self.load_mission(['enable_gui:=false', 'enable_audio_video:=false',
                                   'enable_patient_measurement:=false'])
        names = [n.name for n in config.nodes]
        for name in ['robot_status_display_node', 'esp32_vitals_bridge_node', 'audio_chat_master']:
            self.assertNotIn(name, names)
        self.assertFalse(self.value(config, 'bedside_session_node', 'audio_video_enabled'))
        self.assertFalse(self.value(config, 'bedside_session_node', 'patient_measurement_enabled'))
        self.assertIn('speak_node', names)
        self.assertIn('barcode_qr_detector_node', names)

    def test_continuous_media_and_independent_vitals_defaults(self):
        config = self.load_mission()
        self.assertIn('audio_video_ready_master', [n.name for n in config.nodes])
        self.assertFalse(config.params['/lower_cam/lower_usb_cam/wait_for_ready'].value)
        self.assertEqual(self.value(config, 'video_sender_master', 'ready_topic'), '/audio_video/ready')
        self.assertFalse(self.value(config, 'bedside_session_node', 'patient_measurement_enabled'))
        self.assertIn('esp32_vitals_bridge_node', [n.name for n in config.nodes])
        self.assertFalse(self.value(config, 'mission_controller_node', 'enable_bedside_distance_tuning'))
        self.assertEqual(config.params['/adaptive_scan_camera_node/video_device'].value, '/dev/video2')
        self.assertEqual(config.params['/lower_cam/lower_usb_cam/video_device'].value, '/dev/video0')

    def test_legacy_ready_can_still_gate_media_explicitly(self):
        config = self.load_peer('master', ['always_on:=false'])
        self.assertNotIn('audio_video_ready_master', [n.name for n in config.nodes])
        self.assertEqual(self.value(config, 'audio_chat_master', 'ready_topic'), '/Ready')
        self.assertTrue(config.params['/lower_cam/lower_usb_cam/wait_for_ready'].value)

    def test_mission_landscape_video_override_and_bounded_default_rate(self):
        config = self.load_mission()
        self.assertEqual(self.value(config, 'video_sender_master', 'framerate'), 10.0)
        self.assertEqual(self.value(config, 'video_stream_master', 'framerate'), 10.0)
        config = self.load_mission(['call_stream_width:=640', 'call_stream_height:=360'])
        self.assertEqual(self.value(config, 'video_sender_master', 'stream_width'), 640)
        self.assertEqual(self.value(config, 'video_sender_master', 'stream_height'), 360)
        # GUI derives aspect from these receiver dimensions at runtime.
        self.assertEqual(self.value(config, 'video_stream_master', 'stream_height'), 360)
        self.assertNotIn('/robot_status_display_node/video_aspect', config.params)

    def test_persistent_camera_and_video_defaults_need_no_cli_device_arguments(self):
        # Use real config files edited in a temporary minimal source tree.
        # No scan_camera_device, call_camera_device or dimensions are passed.
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            packages = ['robot_navigation', 'robot_bringup', 'fine_tuning',
                        'arm_and_gripper', 'path_manager']
            for package in packages:
                destination = source / package
                destination.mkdir()
                for name in ['launch', 'config']:
                    original = PACKAGE.parent / package / name
                    if original.is_dir():
                        shutil.copytree(original, destination / name)
            package = source / 'robot_navigation'
            path = package / 'config/camera_devices.launch'
            text = path.read_text(encoding='utf-8')
            text = text.replace('name="scan_device" default="/dev/video2"',
                                'name="scan_device" default="/dev/test-scan"')
            text = text.replace('name="robot_audio_device" default="/dev/video0"',
                                'name="robot_audio_device" default="/dev/test-call"')
            text = text.replace('name="doctor_audio_device" default="/dev/video0"',
                                'name="doctor_audio_device" default="/dev/test-doctor"')
            path.write_text(text, encoding='utf-8')
            path = package / 'config/audio_video_defaults.launch'
            text = path.read_text(encoding='utf-8').replace(
                'name="stream_height" default="480"', 'name="stream_height" default="360"')
            path.write_text(text, encoding='utf-8')
            with patch.dict(globals(), PACKAGE=package):
                config = self.load_mission()
                doctor = self.load_peer('slave')
            self.assertEqual(config.params['/adaptive_scan_camera_node/video_device'].value,
                             '/dev/test-scan')
            self.assertEqual(config.params['/lower_cam/lower_usb_cam/video_device'].value,
                             '/dev/test-call')
            self.assertEqual(doctor.params['/upper_cam/upper_usb_cam/video_device'].value,
                             '/dev/test-doctor')
            for node in ['video_sender_master', 'video_stream_master']:
                self.assertEqual(self.value(config, node, 'stream_width'), 640)
                self.assertEqual(self.value(config, node, 'stream_height'), 360)
            self.assertEqual(self.value(doctor, 'video_sender_slave', 'stream_height'), 360)
            self.assertNotIn('/robot_status_display_node/video_aspect', config.params)

    def test_mission_field_uses_same_waypoints_and_can_be_hidden(self):
        config = self.load_mission()
        self.assertTrue(self.value(config, 'robot_status_display_node', 'show_map'))
        self.assertEqual(self.value(config, 'robot_status_display_node', 'odom_topic'), '/odom')
        self.assertEqual(self.value(config, 'robot_status_display_node', 'map_frame'), 'odom')
        self.assertEqual(config.params['/robot_status_display_node/routes/waypoints/P4/x'].value, 4.0)
        config = self.load_mission(['show_map:=false'])
        self.assertFalse(self.value(config, 'robot_status_display_node', 'show_map'))

    def test_mission_pre_recorded_speech_files_exist(self):
        config = self.load_mission()
        for name in ['bed1_audio_file', 'bed3_audio_file']:
            path = Path(self.value(config, 'speak_node', name))
            self.assertTrue(path.is_file(), str(path))


if __name__ == '__main__':
    unittest.main()
