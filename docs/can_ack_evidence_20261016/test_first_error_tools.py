import importlib.util
import pathlib
import socket
import sys
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location('first_error', '/tmp/can_ack_first_error.py')
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)
sys.modules['can_ack_first_error'] = trace
fresh_spec = importlib.util.spec_from_file_location('fresh_start', '/tmp/can_ack_fresh_start.py')
fresh = importlib.util.module_from_spec(fresh_spec)
fresh_spec.loader.exec_module(fresh)


class FirstErrorExperiment(unittest.TestCase):
    def run_experiment(self, fail_mode=False, fail_send=False):
        calls = []
        sock = mock.MagicMock()
        sock.__enter__.return_value = sock
        def configure(*args):
            calls.append(('ip', args))
            if fail_mode and args == ('type', 'can', 'one-shot', 'on'):
                raise RuntimeError('unsupported mode')
        def capture(_, stage, seconds):
            calls.append(('capture', stage))
        def send(frame):
            calls.append(('send', trace.FRAME.unpack(frame)))
            if fail_send:
                raise OSError('injected ENOBUFS')
            return len(frame)
        sock.send.side_effect = send
        original = {'flags': ['UP'], 'linkinfo': {'info_kind': 'can',
                    'info_data': {'bittiming': {'bitrate': 500000}}}}
        with mock.patch.object(trace.os, 'geteuid', return_value=0), \
                mock.patch.object(trace, 'snapshot', return_value=original), \
                mock.patch.object(trace.subprocess, 'check_output', return_value=b'[]'), \
                mock.patch.object(trace.socket, 'socket', return_value=sock), \
                mock.patch.object(trace, 'capture', side_effect=capture), \
                mock.patch.object(trace, 'ip', side_effect=configure), \
                mock.patch.object(trace, 'event'):
            if fail_mode:
                with self.assertRaisesRegex(RuntimeError, 'unsupported'):
                    trace.main()
            else:
                trace.main()
        return calls

    def test_subscribes_and_captures_before_exactly_one_non_motion_tx(self):
        calls = self.run_experiment()
        sends = [x for x in calls if x[0] == 'send']
        self.assertEqual(len(sends), 1)
        self.assertEqual(sends[0][1], (0x100, 8, bytes.fromhex('09ff000000000000')))
        self.assertLess(calls.index(('capture', 'single_shot_before_tx')), calls.index(sends[0]))
        self.assertEqual(calls[-4:-1], [('ip', ('down',)),
            ('ip', ('type', 'can', 'one-shot', 'off')), ('ip', ('up',))])

    def test_unsupported_mode_still_restores_normal_and_sends_nothing(self):
        calls = self.run_experiment(fail_mode=True)
        self.assertFalse(any(x[0] == 'send' for x in calls))
        self.assertIn(('ip', ('type', 'can', 'one-shot', 'off')), calls)
        self.assertEqual(calls[-1], ('capture', 'restored_receive_only'))

    def test_send_failure_is_not_retried_and_still_restores(self):
        calls = self.run_experiment(fail_send=True)
        self.assertEqual(len([x for x in calls if x[0] == 'send']), 1)
        self.assertIn(('ip', ('type', 'can', 'one-shot', 'off')), calls)


class FreshStartExperiment(unittest.TestCase):
    def run_experiment(self, baseline_errors=0, old_tx=0):
        calls = []
        trace.report = {'events': [], 'counts': {}}
        trace.records = []
        sock = mock.MagicMock()
        sock.__enter__.return_value = sock
        sock.bind.side_effect = lambda address: calls.append(('bind', address))
        def configure(*args):
            calls.append(('ip', args))
        def capture(_, stage, seconds):
            calls.append(('capture', stage))
            trace.report['counts'][stage] = {'error_reports': baseline_errors}
        def send(frame):
            calls.append(('send', trace.FRAME.unpack(frame)))
            return len(frame)
        sock.send.side_effect = send
        original = {'flags': [], 'stats64': {'tx': {'packets': old_tx}, 'rx': {'packets': 0}}}
        with mock.patch.object(fresh.os, 'geteuid', return_value=0), \
                mock.patch.object(trace, 'snapshot', return_value=original), \
                mock.patch.object(fresh.socket, 'socket', return_value=sock), \
                mock.patch.object(trace, 'capture', side_effect=capture), \
                mock.patch.object(trace, 'ip', side_effect=configure), \
                mock.patch.object(trace, 'event'):
            if old_tx:
                with self.assertRaisesRegex(RuntimeError, 'freshly replugged'):
                    fresh.main()
            else:
                fresh.main()
        return calls

    def test_errors_before_host_tx_suppress_probe(self):
        calls = self.run_experiment(baseline_errors=1)
        self.assertFalse(any(x[0] == 'send' for x in calls))

    def test_clean_baseline_emits_one_exact_heartbeat_after_subscribing_and_capture(self):
        calls = self.run_experiment()
        sends = [x for x in calls if x[0] == 'send']
        self.assertEqual(sends, [('send', (0x100, 8, bytes.fromhex('09ff000000000000')))])
        self.assertLess(calls.index(('bind', ('can0',))), calls.index(('ip', ('up',))))
        self.assertLess(calls.index(('capture', 'fresh_before_first_tx')), calls.index(sends[0]))

    def test_nonfresh_interface_is_rejected_without_configuration_or_transmission(self):
        self.assertEqual(self.run_experiment(old_tx=1), [])


if __name__ == '__main__':
    unittest.main()
