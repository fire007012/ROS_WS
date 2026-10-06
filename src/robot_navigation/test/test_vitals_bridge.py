#!/usr/bin/env python3
"""No ROS master/devices needed: python3 -m unittest discover -s test -p test_vitals*.py"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest

sys.dont_write_bytecode = True
from unittest.mock import patch


class Publisher:
    def __init__(self, *args, **kwargs):
        self.values = []
    def publish(self, value):
        self.values.append(value)


rospy = types.ModuleType('rospy')
rospy.is_shutdown = lambda: False
rospy.logwarn_throttle = lambda *args: None
rospy.loginfo = lambda *args: None
std = types.ModuleType('std_msgs.msg')
std.Bool = std.Float32 = std.String = type('Message', (), {})
robot = types.ModuleType('robot_navigation.msg')
robot.QrResult = type('QrResult', (), {})
sys.modules.update({'rospy': rospy, 'std_msgs': types.ModuleType('std_msgs'),
                    'std_msgs.msg': std, 'robot_navigation': types.ModuleType('robot_navigation'),
                    'robot_navigation.msg': robot})


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / 'scripts' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load('vitals_bridge', 'esp32_vitals_bridge_node.py')
display = load('vitals_display', 'robot_status_display_node.py')


def frame(**kwargs):
    result = dict(type='vitals', version=1, seq=10, uptime_ms=10000, mode='button',
                  measuring=True, heart_rate=76, heart_rate_valid=True,
                  temperature=36.7, temperature_valid=True)
    result.update(kwargs)
    return result


def make_bridge():
    obj = bridge.VitalsBridge.__new__(bridge.VitalsBridge)
    obj.lock = threading.RLock()
    obj.last_rx = obj.last_seq = obj.last_uptime = None
    obj.online = False
    obj.timeout = 3.0
    for name in ('hr', 'temp', 'hr_valid', 'temp_valid', 'measuring', 'status', 'frame'):
        setattr(obj, name, Publisher())
    return obj


class ProtocolTests(unittest.TestCase):
    def test_valid_frame(self):
        self.assertEqual(bridge.validate_frame(frame()), (76.0, 36.7, True, True))

    def test_invalid_or_idle_measurements_become_nan(self):
        for packet in (frame(measuring=False), frame(heart_rate_valid=False,
                       temperature_valid=False, heart_rate=None, temperature=None)):
            hr, temp, hv, tv = bridge.validate_frame(packet)
            self.assertTrue(math.isnan(hr) and math.isnan(temp))
            self.assertFalse(hv or tv)

    def test_malformed_frames(self):
        cases = [[], None, 7, dict(), frame(type='other'), frame(version=True),
                 frame(version=2), frame(seq=True), frame(seq=-1),
                 frame(uptime_ms='10'), frame(mode='bad'), frame(measuring='false'),
                 frame(heart_rate_valid=1), frame(temperature_valid='true'),
                 frame(heart_rate=True), frame(heart_rate='76'),
                 frame(heart_rate=float('inf')), frame(temperature=float('nan')),
                 frame(temperature=None), frame(temperature=1000),
                 frame(heart_rate=10**1000), frame(heart_sensor_ready='false')]
        for packet in cases:
            with self.subTest(packet=str(packet)[:80]):
                with self.assertRaises(ValueError):
                    bridge.validate_frame(packet)

    def test_split_merged_and_crlf(self):
        buf = bridge.LineBuffer(32)
        self.assertEqual(buf.feed(b'one'), [])
        self.assertEqual(buf.feed(b'\r\ntwo\nthree'), [b'one\r', b'two'])
        self.assertEqual(buf.feed(b'\n'), [b'three'])

    def test_oversized_line_discarded_until_newline(self):
        buf = bridge.LineBuffer(8)
        self.assertEqual(buf.feed(b'x' * 100000), [])
        self.assertEqual(len(buf.buffer), 0)
        self.assertEqual(buf.feed(b'good\nnext\n'), [b'next'])

    def test_invalid_utf8_json_and_wrong_shape_do_not_publish(self):
        obj = make_bridge()
        for raw in (b'\xff\n', b'{', b'[]', b'null', b'{"measuring":"false"}'):
            self.assertFalse(obj.text_handle(raw))
        self.assertIsNone(obj.last_rx)
        self.assertEqual(obj.hr.values, [])

    def test_atomic_update_no_partial_publish(self):
        obj = make_bridge()
        self.assertTrue(obj.handle(frame()))
        previous = obj.last_rx
        with self.assertRaises(ValueError):
            obj.handle(frame(seq=11, temperature='bad'))
        self.assertEqual(obj.hr.values, [76.0])
        self.assertEqual(obj.last_rx, previous)
        atomic = json.loads(obj.frame.values[-1])
        self.assertEqual(atomic['heart_rate'], 76)
        self.assertEqual(atomic['status'], 'online')

    def test_duplicate_out_of_order_reboot_and_wrap(self):
        obj = make_bridge()
        obj.handle(frame())
        previous = obj.last_rx
        self.assertFalse(obj.handle(frame()))
        self.assertFalse(obj.handle(frame(seq=9, uptime_ms=11000)))
        self.assertEqual(obj.last_rx, previous)
        self.assertTrue(obj.handle(frame(seq=0, uptime_ms=1)))
        obj.last_seq = 0xFFFFFFFF
        self.assertTrue(obj.handle(frame(seq=0, uptime_ms=12000)))

    def test_watchdog_wall_clock_clears_stale_values(self):
        obj = make_bridge()
        obj.handle(frame())
        with patch.object(bridge.time, 'monotonic', return_value=obj.last_rx + 3.01):
            obj.watchdog(None)
        self.assertEqual(obj.status.values[-1], 'offline')
        self.assertFalse(obj.measuring.values[-1])
        self.assertTrue(math.isnan(obj.hr.values[-1]))
        self.assertIsNone(json.loads(obj.frame.values[-1])['temperature'])

    def test_stop_and_partial_validity(self):
        obj = make_bridge()
        obj.handle(frame(heart_rate_valid=False, heart_rate=None))
        self.assertTrue(math.isnan(obj.hr.values[-1]))
        self.assertEqual(obj.temp.values[-1], 36.7)
        self.assertEqual(obj.status.values[-1], 'online_invalid')
        obj.handle(frame(seq=11, measuring=False))
        self.assertEqual(obj.status.values[-1], 'online_idle')
        self.assertTrue(math.isnan(obj.temp.values[-1]))


class SerialLifecycleTests(unittest.TestCase):
    def test_closed_port_read_race_reconnects(self):
        obj = make_bridge()
        obj.stop = threading.Event()
        obj.retry_delay = .001
        obj.baudrate = 115200
        obj.device = 'test'
        ports = []
        class Port:
            def __init__(self, **kwargs):
                self.is_open = False
                ports.append(self)
            def open(self):
                self.is_open = True
            def close(self):
                self.is_open = False
            def read(self, count):
                if len(ports) == 1:
                    self.close()
                    raise TypeError('overlapped handle cleared during read')
                obj.stop.set()
                return b''
        obj.serial_module = types.SimpleNamespace(Serial=Port, SerialException=OSError)
        obj.serial_loop()
        self.assertEqual(len(ports), 2)
        self.assertTrue(all(not port.is_open for port in ports))

    def test_shutdown_cancels_read_instead_of_racy_close(self):
        obj = make_bridge()
        obj.stop = threading.Event()
        obj.server = None
        obj.transport = 'serial'
        port = types.SimpleNamespace(is_open=True, cancelled=False)
        def cancel():
            port.cancelled = True
        port.cancel_read = cancel
        obj.connection = port
        obj.worker = threading.Thread(target=lambda: obj.stop.wait())
        obj.worker.start()
        obj.shutdown()
        self.assertTrue(port.cancelled)
        self.assertFalse(obj.worker.is_alive())


class Label:
    def __init__(self):
        self.text = None
        self.threads = []
    def config(self, text):
        self.text = text
        self.threads.append(threading.get_ident())


class Root:
    def after(self, *_):
        pass


def make_window():
    obj = display.StatusWindow.__new__(display.StatusWindow)
    obj.lock = threading.Lock()
    obj.pending = {}
    obj.sensor_hint = ''
    obj.atomic_source = False
    obj.last_vitals = time.monotonic()
    obj.stale_timeout = 3.0
    obj.vitals_state = 'offline'
    obj.measuring_state = False
    obj.labels = {key: Label() for key in ('hr', 'temp', 'esp', 'measure', 'task', 'scan')}
    obj.root = Root()
    return obj


class DisplayTests(unittest.TestCase):
    def test_nan_not_displayed_as_a_measurement(self):
        self.assertIn('--', display.measurement_text(float('nan'), 'HR:', 'bpm'))
        self.assertIn('76.0', display.measurement_text(76, 'HR:', 'bpm'))

    def test_ros_callback_never_touches_tk(self):
        obj = make_window()
        worker = threading.Thread(target=lambda: obj.frame(types.SimpleNamespace(
            data=json.dumps(frame(status='online')))))
        worker.start(); worker.join()
        self.assertIsNone(obj.labels['hr'].text)
        obj.pump()
        self.assertIn('76.0', obj.labels['hr'].text)
        self.assertEqual(obj.labels['hr'].threads, [threading.get_ident()])

    def test_atomic_frame_ignores_late_legacy_topics(self):
        obj = make_window()
        obj.frame(types.SimpleNamespace(data=json.dumps(frame(status='online'))))
        obj.legacy_set('_measuring', False)
        obj.vital('hr', 150, 'HR:', 'bpm')
        obj.pump()
        self.assertTrue(obj.measuring_state)
        self.assertIn('76.0', obj.labels['hr'].text)

    def test_idle_and_stale_clear_values(self):
        obj = make_window()
        obj.frame(types.SimpleNamespace(data=json.dumps(frame(status='online_idle', measuring=False))))
        obj.pump()
        self.assertIn('--', obj.labels['hr'].text)
        obj.frame(types.SimpleNamespace(data=json.dumps(frame(status='online'))))
        with patch.object(display.time, 'monotonic', return_value=obj.last_vitals + 4):
            obj.pump()
        self.assertIn('--', obj.labels['temp'].text)
        self.assertIn('离线', obj.labels['esp'].text)

    def test_missing_sensor_hint(self):
        obj = make_window()
        obj.frame(types.SimpleNamespace(data=json.dumps(frame(
            status='online_idle', measuring=False, temperature_sensor_ready=False))))
        obj.pump()
        self.assertIn('温度传感器未识别', obj.labels['esp'].text)

    def test_callback_bursts_are_coalesced(self):
        obj = make_window()
        for i in range(10000):
            obj.set('task', str(i))
        self.assertEqual(obj.pending, {'task': '9999'})


if __name__ == '__main__':
    unittest.main()
