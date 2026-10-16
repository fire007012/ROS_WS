"""A/B CAN resynchronization test; one heartbeat per phase, no motion.

Tests host sampling/SJW, or nearby bitrates with --clock-offset-test, as a
cause of errors preceding ROS traffic. Keeps a
candidate only after candidate/original/candidate confirmation; otherwise
restores the original 500 kbit/s, 87.5%, SJW=1 configuration.
"""
import json
import os
import subprocess
import sys
import time

import can_ack_first_error as trace

clock_test = '--clock-offset-test' in sys.argv
original = (0.875, 1, 500000)
candidates = ([(0.8, 2, 480000), (0.8, 2, 489796),
               (0.8, 2, 505263), (0.75, 2, 521739)] if clock_test else
              [(0.875, 2, 500000), (0.75, 4, 500000), (0.833333, 2, 500000)])
prefix = 'clock' if clock_test else 'sjw'
result = {'phases': []}
keep = False


def configure(timing):
    trace.ip('down')
    trace.ip('type', 'can', 'bitrate', str(timing[2]), 'sample-point', str(timing[0]),
             'sjw', str(timing[1]))
    trace.ip('up')


def check(label, timing):
    configure(timing)
    proc = subprocess.run([sys.executable, '/tmp/can_ack_probe.py', trace.DEVICE,
                           label, '6', '--probe'], check=True, stdout=subprocess.PIPE,
                          universal_newlines=True, timeout=15)
    report = json.loads(proc.stdout)
    faults = []
    for line in (trace.ROOT / (label + '.jsonl')).read_text().splitlines():
        frame = json.loads(line)
        can_id = int(frame['can_id'], 16)
        if not can_id & 0x20000000:
            continue
        errors = can_id & 0x1fffffff
        flags = bytes.fromhex(frame['data'])[1]
        # Old firmware also reports ordinary error-active state notifications.
        if errors == 4 and flags in (0, 0x40):
            continue
        faults.append(frame)
    clean = (report['after']['linkinfo']['info_data']['state'] == 'ERROR-ACTIVE'
             and not faults and report['counts'].get('heartbeat_ack_ok', 0) > 0)
    item = dict(label=label, sample_point=timing[0], sjw=timing[1], nominal_bitrate=timing[2],
                counts=report['counts'], actual_fault_reports=len(faults), clean=clean,
                actual_bittiming=report['before']['linkinfo']['info_data']['bittiming'])
    result['phases'].append(item)
    print(json.dumps(item), flush=True)
    return clean


if __name__ == '__main__':
    if os.geteuid() != 0:
        sys.exit('Local sudo is required for host CAN timing configuration')
    before = trace.snapshot()
    data = before['linkinfo']['info_data']
    if (before['linkinfo']['info_kind'] != 'can' or data.get('ctrlmode') or
            data['bittiming']['bitrate'] != 500000 or
            data['bittiming']['sample_point'] != original[0] or
            data['bittiming']['sjw'] != original[1]):
        sys.exit('Expected normal 500 kbit/s, 87.5%, SJW=1; no changes made')
    result['before'] = before
    try:
        baseline = check(prefix + '_original', original)
        if baseline:
            result['conclusion'] = 'Original timing is clean; no timing change justified'
        else:
            for index, candidate in enumerate(candidates):
                if check(prefix + '_candidate_%d' % index, candidate):
                    control = check(prefix + '_control_original', original)
                    confirmed = check(prefix + '_confirm_candidate', candidate)
                    if confirmed and not control:
                        keep = True
                        result['conclusion'] = 'Candidate clean twice and original failed twice'
                        result['kept_timing'] = candidate
                        break
            if not keep:
                result['conclusion'] = 'No verified timing repair; restored original'
    finally:
        if not keep:
            configure(original)
        result['after'] = trace.snapshot()
        filename = 'clock_origin.json' if clock_test else 'timing_origin.json'
        (trace.ROOT / filename).write_text(json.dumps(result, indent=2) + '\n')
        print(result.get('conclusion', 'Experiment interrupted; original restored'), flush=True)
