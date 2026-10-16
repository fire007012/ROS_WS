"""Exercise the production bedside service with a virtual clock; no devices/ROS master."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


class Response:
    def __init__(self, success, message):
        self.success, self.message = success, message


class BedsideTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.tick = lambda: None
        self.params = {'~call_duration_s': .3, '~measurement_valid_hold_s': .2,
                       '~timeout_s': 1.0, '~freshness_s': .15}
        self.subscriptions = {}
        self.messages = []
        ros = ModuleType('rospy')
        ros.get_param = lambda key, default=None: self.params.get(key, default)
        ros.is_shutdown = lambda: False
        ros.Publisher = lambda *a, **kw: SimpleNamespace(publish=self.messages.append)
        ros.Service = lambda *a, **kw: None
        def subscribe(topic, kind, callback):
            self.subscriptions[topic] = callback
        ros.Subscriber = subscribe
        modules = {'rospy': ros}
        for package, fields in [('sensor_msgs', ['CompressedImage']),
                                ('std_msgs', ['Bool', 'String', 'Empty'])]:
            modules[package] = ModuleType(package)
            mod = ModuleType(package + '.msg')
            for name in fields:
                setattr(mod, name, object)
            modules[package + '.msg'] = mod
        modules['std_srvs'] = ModuleType('std_srvs')
        srv = ModuleType('std_srvs.srv')
        srv.Trigger, srv.TriggerResponse = object, Response
        modules['std_srvs.srv'] = srv
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        spec = importlib.util.spec_from_file_location('bedside_under_test',
            Path(__file__).parents[1] / 'scripts/bedside_session_node.py')
        self.mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.mod)
        # Replace only this module's clock, not the interpreter's time module.
        self.mod.time = SimpleNamespace(monotonic=lambda: self.now, sleep=self.advance)
        self.seq = 0

    def advance(self, seconds):
        self.now += seconds
        self.tick()

    def session(self, **params):
        self.params.update({'~' + k: v for k, v in params.items()})
        return self.mod.BedsideSession()

    def feed(self, obj, av=True, vitals=True):
        if av:
            obj.on_peer(SimpleNamespace(data=True))
            obj.on_video(SimpleNamespace(data=b'jpeg', format='jpeg',
                                        header=SimpleNamespace(frame_id='doctor')))
        if vitals:
            self.seq += 1
            obj.on_vitals(SimpleNamespace(data=json.dumps(dict(seq=self.seq,
                status='online', measuring=True, heart_rate_valid=True,
                temperature_valid=True, heart_rate=75, temperature=36.5))))

    def test_master_and_slave_consume_their_actual_peer_and_remote_camera(self):
        for role, camera in [('master', 'upper'), ('slave', 'lower')]:
            with self.subTest(role=role):
                self.subscriptions.clear()
                obj = self.session(audio_role=role, vitals_frame_topic='/custom/vitals')
                self.assertIn('/audio_chat/' + role + '/peer_ready', self.subscriptions)
                self.assertIn('/' + camera + '_cam/network/compressed', self.subscriptions)
                self.assertIn('/custom/vitals', self.subscriptions)
                self.tick = lambda: self.feed(obj)
                self.assertTrue(obj.complete(None).success)

    def test_each_bed_requires_new_samples_not_previous_bed_results(self):
        obj = self.session()
        self.tick = lambda: self.feed(obj)
        self.assertTrue(obj.complete(None).success)
        self.now += .01
        self.tick = lambda: None
        self.assertFalse(obj.complete(None).success)
        self.tick = lambda: self.feed(obj)
        self.assertTrue(obj.complete(None).success)

    def test_audio_without_real_video_or_vitals_cannot_complete(self):
        obj = self.session()
        self.tick = lambda: obj.on_peer(SimpleNamespace(data=True))
        self.assertFalse(obj.complete(None).success)

    def test_vitals_alone_or_video_alone_cannot_complete(self):
        for av, vitals in [(False, True), (True, False)]:
            obj = self.session()
            self.tick = lambda: self.feed(obj, av, vitals)
            self.assertFalse(obj.complete(None).success)

    def test_gap_restarts_continuous_av_hold(self):
        obj = self.session(patient_measurement_enabled=False)
        began = self.now
        def tick():
            if self.now-began < .2 or self.now-began > .5:
                self.feed(obj, vitals=False)
        self.tick = tick
        self.assertTrue(obj.complete(None).success)
        self.assertGreaterEqual(self.now-began, .8)

    def test_estop_cancels_and_reset_does_not_resume_old_service(self):
        obj = self.session()
        self.tick = lambda: obj.on_estop(SimpleNamespace(data=True))
        self.assertFalse(obj.complete(None).success)
        obj.on_reset(SimpleNamespace(data=True))
        self.assertFalse(obj.complete(None).success)
        obj.on_enabled(SimpleNamespace(data=True))
        self.tick = lambda: self.feed(obj)
        self.assertTrue(obj.complete(None).success)

    def test_invalid_or_repeated_vitals_do_not_count(self):
        obj = self.session()
        self.feed(obj)
        self.assertTrue(obj.vitals_valid)
        for data in ['[]', 'null', '{bad', json.dumps(dict(seq=self.seq)),
                     json.dumps(dict(seq=999, status='online', measuring=True,
                        heart_rate_valid=True, temperature_valid=True,
                        heart_rate=True, temperature=36.5))]:
            obj.on_vitals(SimpleNamespace(data=data))
            self.assertFalse(obj.vitals_valid)

    def test_disabled_features_are_explicit_and_timeout_is_reported(self):
        obj = self.session(audio_video_enabled=False, patient_measurement_enabled=False)
        self.assertTrue(obj.complete(None).success)
        obj = self.session(audio_video_enabled=True, patient_measurement_enabled=True)
        self.tick = lambda: None
        result = obj.complete(None)
        self.assertFalse(result.success)
        self.assertIn('timeout', result.message)


if __name__ == '__main__':
    unittest.main()
