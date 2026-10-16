"""Offline verification of diagnostic timing selection and rollback."""
import json
import pathlib
import runpy
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

SCRIPT = '/tmp/can_ack_timing_origin.py'

class TimingExperiment(unittest.TestCase):
    def experiment(self, outcomes, fail_candidate=False):
        calls = []
        snapshot = {'flags': ['UP'], 'linkinfo': {'info_kind': 'can', 'info_data': {
            'state': 'ERROR-ACTIVE', 'bittiming': {
                'bitrate': 500000, 'sample_point': .875, 'sjw': 1}}}}
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            def ip(*args):
                calls.append(args)
                if fail_candidate and '480000' in args:
                    raise RuntimeError('injected configuration failure')
            trace = types.SimpleNamespace(DEVICE='can0', ROOT=root, ip=ip,
                                          snapshot=lambda: snapshot)
            def probe(args, **kwargs):
                clean = next(outcomes)
                label = args[3]
                record = {'can_id': '0x20000004' if clean else '0x20000024',
                          'data': bytes(8).hex()}
                (root / (label + '.jsonl')).write_text(json.dumps(record) + '\n')
                report = {'counts': {'heartbeat_ack_ok': 1},
                          'before': snapshot, 'after': snapshot}
                return subprocess.CompletedProcess(args, 0, json.dumps(report))
            with mock.patch.dict(sys.modules, {'can_ack_first_error': trace}), \
                 mock.patch('os.geteuid', return_value=0), \
                 mock.patch.object(subprocess, 'run', side_effect=probe), \
                 mock.patch.object(sys, 'argv', [SCRIPT, '--clock-offset-test']), \
                 mock.patch('builtins.print'):
                if fail_candidate:
                    with self.assertRaisesRegex(RuntimeError, 'injected'):
                        runpy.run_path(SCRIPT, run_name='__main__')
                else:
                    runpy.run_path(SCRIPT, run_name='__main__')
            result = json.loads((root/'clock_origin.json').read_text())
        return calls, result

    def test_all_failures_restore_original_rate_and_keep_nothing(self):
        calls, result = self.experiment(iter([False] * 5))
        rates = [x[3] for x in calls if x[:3] == ('type', 'can', 'bitrate')]
        self.assertEqual(rates, ['500000', '480000', '489796', '505263', '521739', '500000'])
        self.assertNotIn('kept_timing', result)
        self.assertTrue(all(not x['clean'] for x in result['phases']))

    def test_candidate_requires_failed_original_control_and_clean_confirmation(self):
        calls, result = self.experiment(iter([False, True, False, True]))
        self.assertEqual(result['kept_timing'], [.8, 2, 480000])
        self.assertEqual([x['clean'] for x in result['phases']], [False, True, False, True])
        self.assertEqual([x[3] for x in calls if x[:3] == ('type', 'can', 'bitrate')],
                         ['500000', '480000', '500000', '480000'])

    def test_failed_confirmation_is_not_kept(self):
        _, result = self.experiment(iter([False, True, False, False, False, False, False]))
        self.assertNotIn('kept_timing', result)

    def test_clean_original_prevents_candidate_changes(self):
        calls, result = self.experiment(iter([True]))
        self.assertEqual(len(result['phases']), 1)
        self.assertFalse(any('480000' in x for x in calls))

    def test_configuration_failure_also_restores_original(self):
        calls, result = self.experiment(iter([False]), fail_candidate=True)
        self.assertEqual(calls[-3:], [('down',), ('type', 'can', 'bitrate', '500000',
                        'sample-point', '0.875', 'sjw', '1'), ('up',)])
        self.assertNotIn('kept_timing', result)

if __name__ == '__main__':
    unittest.main()
