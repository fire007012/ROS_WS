#!/usr/bin/env python3
"""Opt-in real serial test. ROS publishers are mocked; this is NOT a ROS GUI test.
python manual_vitals_serial_smoke.py --device COM8 --report usb_worker_test.json
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import types

sys.dont_write_bytecode = True
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--device', required=True)
parser.add_argument('--report')
args = parser.parse_args()

class Publisher:
    def __init__(self, name, *args, **kwargs):
        self.name = name
        self.values = []
    def publish(self, value):
        self.values.append(value)

class Timer:
    def __init__(self, *args):
        pass

params = {'~serial_device': args.device, '~transport': 'serial'}
r = types.ModuleType('rospy')
r.get_param = lambda name, default=None: params.get(name, default)
r.Publisher = Publisher
r.is_shutdown = lambda: False
r.loginfo = lambda *args: print('INFO', args)
r.logwarn_throttle = lambda *args: print('WARN', args)
r.on_shutdown = lambda *args: None
r.Duration = lambda value: value
r.Timer = Timer
messages = types.ModuleType('std_msgs.msg')
messages.Float32 = messages.Bool = messages.String = type('Message', (), {})
sys.modules.update({'rospy': r, 'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': messages})
source = Path(__file__).parents[1] / 'scripts/esp32_vitals_bridge_node.py'
spec = importlib.util.spec_from_file_location('vitals_bridge', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
obj = module.VitalsBridge()
try:
    time.sleep(5)
    first = len(obj.frame.values)
    if not obj.online or obj.connection is None:
        raise RuntimeError('no valid vitals frames received')
    print('before close:', obj.frame.values[-1])
    # Deliberately exercises close/read/reopen race. Not a physical unplug test.
    obj.connection.close()
    time.sleep(5)
    second = len(obj.frame.values)
    report = {'test': 'real device -> actual bridge worker -> mocked ROS publishers',
              'device': args.device, 'frames_before_close': first,
              'frames_after_reopen': second,
              'recovered': second > first and obj.online,
              'latest': json.loads(obj.frame.values[-1])}
finally:
    obj.shutdown()
report['worker_stopped'] = not obj.worker.is_alive()
print(json.dumps(report, indent=2))
if args.report:
    Path(args.report).write_text(json.dumps(report, indent=2), encoding='utf-8')
if not report['recovered'] or not report['worker_stopped']:
    raise SystemExit('serial test FAILED')
