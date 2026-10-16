"""Isolate adapter loopback from the external bus; sends no motion command."""
import json
import os
import pathlib
import subprocess
import sys

root = pathlib.Path('/home/fire/ros_ws/src/docs/can_ack_evidence_20261016')
if os.geteuid() != 0:
    sys.exit('Run with sudo: controller mode changes require CAP_NET_ADMIN.')

def ip(*args):
    subprocess.run(['ip', 'link', 'set', 'dev', 'can0', *args], check=True, timeout=15)

def configure(loopback, listen_only):
    ip('down')
    ip('type', 'can', 'bitrate', '500000', 'sample-point', '0.875', 'sjw', '1',
       'loopback', loopback, 'listen-only', listen_only)
    ip('up')

def capture(label, probe=False):
    args = [sys.executable, '/tmp/can_ack_probe.py', 'can0', label, '8']
    if probe:
        args.append('--probe')
    subprocess.run(args, check=True)
    return json.loads((root / (label + '.json')).read_text())

try:
    configure('on', 'off')
    loopback = capture('adapter_loopback', True)
    print('INTERNAL LOOPBACK:', loopback['counts'])
    configure('off', 'on')
    silent = capture('external_listen_only')
    print('EXTERNAL LISTEN ONLY:', silent['counts'])
finally:
    configure('off', 'off')
    normal = capture('normal_after_selftest')
    print('RESTORED NORMAL MODE:', normal['counts'])
