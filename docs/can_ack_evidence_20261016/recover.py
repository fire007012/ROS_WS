"""A/B check CANable timing using only one non-motion heartbeat per phase."""
import json
import os
import pathlib
import subprocess
import sys

EVIDENCE = pathlib.Path('/home/fire/ros_ws/src/docs/can_ack_evidence_20261016')

def link():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']))[0]

def ip(*args):
    subprocess.run(['ip', 'link', 'set', 'dev', 'can0', *args], check=True, timeout=15)

def capture(label):
    subprocess.run([sys.executable, '/tmp/can_ack_probe.py', 'can0', label, '8', '--probe'], check=True)
    return json.loads((EVIDENCE / (label + '.json')).read_text())

def healthy(report):
    counts = report['counts']
    return (report['after']['linkinfo']['info_data']['state'] == 'ERROR-ACTIVE'
            and counts.get('error_frames', 0) == 0
            and counts.get('heartbeat_ack_ok', 0) > 0)

def configure(sample_point):
    ip('down')
    ip('type', 'can', 'bitrate', '500000', 'sample-point', str(sample_point), 'sjw', '1')
    ip('up')

if os.geteuid() != 0:
    sys.exit('Run with sudo: interface reset requires CAP_NET_ADMIN.')
original = link()
data = original['linkinfo']['info_data']
if original['linkinfo']['info_kind'] != 'can' or data['bittiming']['bitrate'] != 500000:
    sys.exit('Expected the current physical 500 kbit/s can0; no changes made.')
if data.get('ctrlmode'):
    sys.exit('Nondefault controller modes require manual review; no changes made.')
old_sample_point = data['bittiming']['sample_point']
keep_change = False
try:
    configure(old_sample_point)
    reset = capture('reset_original_timing')
    if healthy(reset):
        keep_change = True
        print('PASS: reset with original timing restored clean heartbeat ACKs.')
    else:
        configure(0.833333)
        matched = capture('sample_833')
        if healthy(matched):
            # Reproduce the failure at the original setting before attributing it to timing.
            configure(old_sample_point)
            control = capture('control_original_timing')
            configure(0.833333)
            confirm = capture('confirm_sample_833')
            if healthy(confirm):
                keep_change = True
                print('PASS: 83.3% is clean; original timing healthy:', healthy(control))
finally:
    if not keep_change:
        configure(old_sample_point)
        print('No verified repair: restored original timing. Evidence saved to', EVIDENCE)
if not keep_change:
    sys.exit(1)
