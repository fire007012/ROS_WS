"""Run the saved bed1 -> bed3 medical path on explicitly lifted wheels.

Uses real path manager/tracker/mux/kinematics/odometry and an isolated build
of the actual CAN node. Real feedback and ACKs are required throughout.
"""
import collections
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client

import rospy
import yaml
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_manager.msg import PathPoint
from path_manager.srv import SelectPath
from std_msgs.msg import Bool, Empty, Float32MultiArray

ROOT = Path(__file__).resolve().parent
TEST = Path('/tmp/chassis_medical_route_test')
ROUTES = ['nurse_station', 'bed1_circle', 'bed1_to_bed3', 'bed3_to_home']


def snapshot():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']))[0]


def main():
    started = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    stem = 'medical_route_stand_' + started.strftime('%Y%m%d_%H%M%S')
    before = snapshot()
    if (before['linkinfo']['info_data']['state'] != 'ERROR-ACTIVE'
            or before['linkinfo']['info_data']['bittiming']['bitrate'] != 500000
            or 'UP' not in before['flags']):
        raise SystemExit('CAN link not ready; no nodes started')
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11329', ROS_IP='127.0.0.1',
               ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=str(TEST / stem))
    os.environ.update(env)
    handles = []
    processes = []
    state = {'odom': None, 'rpm': None, 'ready': False, 'estop': False,
             'done': False, 'saw_unfinished': False, 'active': None}
    timings = {}
    ack_counts = collections.Counter()
    results = []
    samples = []
    errors = []
    interrupted = False
    test_start = time.monotonic()
    logfile = (ROOT / (stem + '.jsonl')).open('w')
    current_points = []
    expected = []
    transition_faults = []

    def interrupt(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def record(item):
        logfile.write(json.dumps({'time': time.time(), **item}) + '\n')
        logfile.flush()

    def spawn(command, label):
        handle = (ROOT / (stem + '_' + label + '.log')).open('w')
        handles.append(handle)
        proc = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    def pose():
        msg = state['odom']
        if msg is None:
            return None
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        return [p.x, p.y, yaw]

    def reached(point, slack=0.0):
        p = pose()
        if p is None or math.hypot(p[0]-point['x'], p[1]-point['y']) > point['tolerance'] + slack:
            return False
        return ('yaw' not in point or abs(math.atan2(math.sin(p[2]-point['yaw']),
                                                    math.cos(p[2]-point['yaw']))) <= 0.08 + slack)

    def update(key, msg):
        state[key] = msg
        timings[key] = time.monotonic()

    def finished(msg):
        if not msg.data:
            state['saw_unfinished'] = True
        state['done'] = msg.data

    def point_callback(msg):
        if not state['active']:
            return
        point = {'x': msg.x, 'y': msg.y, 'tolerance': msg.tolerance}
        if msg.has_yaw:
            point['yaw'] = msg.yaw
        if current_points and point == current_points[-1]['point']:
            return
        index = len(current_points)
        if index >= len(expected) or point != expected[index]:
            transition_faults.append('Unexpected waypoint: ' + str(point))
        if index and not reached(expected[index-1], slack=0.015):
            transition_faults.append('Previous waypoint advanced outside tolerance')
        item = {'point': point, 'odom_at_selection': pose(), 'elapsed': time.monotonic()-test_start}
        current_points.append(item)
        record({'route': state['active'], 'waypoint': item})
        print('TARGET %s %s' % (state['active'], point), flush=True)

    def native_callback(msg):
        if not msg.is_extended and msg.id in (0x102, 0x103, 0x101, 0x110):
            if msg.id != 0x101 or msg.data[0] == 6:
                record({'can_id': hex(msg.id), 'data': bytes(msg.data[:msg.dlc]).hex()})
        if msg.is_extended and msg.dlc == 3 and msg.data[0] == 0xf6:
            ack_counts[str(msg.id >> 8)] += 1

    def diagnostic(msg):
        for status in msg.status:
            if status.name == 'STM32 Motor/CAN':
                state['diagnostics'] = {x.key: x.value for x in status.values}

    def check(ready_required=True):
        now = time.monotonic()
        if interrupted:
            raise RuntimeError('Interrupted by user')
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('A test process exited unexpectedly')
        if state['estop']:
            raise RuntimeError('CAN/software emergency stop latched')
        if transition_faults:
            raise RuntimeError(transition_faults[0])
        if now - test_start > 480:
            raise RuntimeError('480 second stand-test deadline exceeded')
        if ready_required:
            if not state['ready']:
                raise RuntimeError('Real motor link is not ready')
            if any(now - timings.get(k, 0) > 0.7 for k in ('odom', 'rpm', 'ready')):
                raise RuntimeError('Odometry/motor feedback/link freshness expired')
            if any(abs(v) > 28 for v in state['rpm'].data):
                raise RuntimeError('Unexpected actual wheel overspeed')
            p = pose()
            if p and (abs(p[0]) > 5 or abs(p[1]) > 3.2):
                raise RuntimeError('Odometry outside planned route bounds')

    zero_pub = estop_pub = cancel_pub = None
    stopped = None
    fault = ''
    try:
        spawn(['roscore', '-p', '11329'], 'master')
        deadline = time.monotonic() + 10
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        while True:
            try:
                if master.getPid('/stand_test')[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError('ROS master did not start')
            time.sleep(0.1)
        rospy.init_node('medical_route_stand_supervisor', disable_signals=True)
        rospy.Subscriber('/odom', Odometry, lambda m: update('odom', m), queue_size=1)
        rospy.Subscriber('/motor_state', Float32MultiArray, lambda m: update('rpm', m), queue_size=1)
        rospy.Subscriber('/motor_link_ready', Bool,
                         lambda m: (state.update(ready=m.data), timings.update(ready=time.monotonic())), queue_size=1)
        rospy.Subscriber('/emergency_stop', Bool,
                         lambda m: state.update(estop=state['estop'] or m.data), queue_size=1)
        rospy.Subscriber('/path_finished', Bool, finished, queue_size=10)
        rospy.Subscriber('/path_points', PathPoint, point_callback, queue_size=10)
        rospy.Subscriber('/can_rx', Frame, native_callback, queue_size=100)
        rospy.Subscriber('/diagnostics', DiagnosticArray, diagnostic, queue_size=10)
        zero_pub = rospy.Publisher('/cmd_vel_safety', Twist, queue_size=1)
        estop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1, latch=True)
        cancel_pub = rospy.Publisher('/mission/cancel', Empty, queue_size=1, latch=True)
        spawn(['roslaunch', str(TEST / 'stand_route.launch')], 'nodes')
        rospy.wait_for_service('/select_path', timeout=15)
        select_path = rospy.ServiceProxy('/select_path', SelectPath)
        deadline = time.monotonic() + 70
        stable_since = None
        while True:
            check(False)
            fresh = state['ready'] and all(time.monotonic()-timings.get(k, 0) < 0.5 for k in ('odom', 'rpm'))
            stable_since = (stable_since or time.monotonic()) if fresh else None
            if stable_since and time.monotonic() - stable_since > 60:
                break
            if time.monotonic() > deadline + 5:
                raise RuntimeError('Real feedback did not become continuously ready')
            time.sleep(0.05)
        config = yaml.safe_load((ROOT.parents[1] / 'robot_bringup/config/paths.yaml').read_text())
        definitions = {x['name']: x for x in config['paths']}
        if not reached({'x': 0, 'y': 0, 'yaw': 0, 'tolerance': 0.02}):
            raise RuntimeError('Stand-test odometry did not initialize at P0')
        last_progress = 0
        for name in ROUTES:
            state.update(active=None, done=False, saw_unfinished=False)
            current_points.clear()
            expected = definitions[name]['points']
            state['active'] = name
            begin = time.monotonic()
            response = select_path(name)
            if not response.success:
                raise RuntimeError('Path selection failed: ' + response.message)
            print('START ' + name, flush=True)
            while True:
                check()
                sample = {'route': name, 'odom': pose(), 'rpm': list(state['rpm'].data),
                          'elapsed': time.monotonic()-test_start}
                record(sample)
                if time.monotonic() - last_progress > 15:
                    last_progress = time.monotonic()
                    print('PROGRESS ' + json.dumps(sample), flush=True)
                if state['done'] and state['saw_unfinished']:
                    if len(current_points) != len(expected) or not reached(expected[-1], slack=0.005):
                        raise RuntimeError('Path completion does not match all saved waypoints')
                    break
                if time.monotonic() - begin > 200:
                    raise RuntimeError('Route segment exceeded 200 seconds')
                time.sleep(0.1)
            result = {'name': name, 'elapsed': time.monotonic()-begin,
                      'final_odom': pose(), 'waypoints': list(current_points), 'passed': True}
            results.append(result)
            print('COMPLETE ' + json.dumps(result), flush=True)
            record({'completed_route': result})
        hold = time.monotonic()
        while time.monotonic() - hold < 5:
            check()
            if not reached(expected[-1], slack=0.005):
                raise RuntimeError('Home position drifted during final hold')
            time.sleep(0.05)
    except Exception as exc:
        fault = str(exc)
        print('STOP: ' + fault, flush=True)
    finally:
        if zero_pub:
            stop_started = time.monotonic()
            if cancel_pub:
                cancel_pub.publish(Empty())
            stationary_since = None
            while time.monotonic() - stop_started < 4 and not rospy.is_shutdown():
                zero_pub.publish(Twist())
                estop_pub.publish(Bool(data=True))
                msg = state['rpm']
                now = time.monotonic()
                fresh = (msg is not None and timings.get('rpm', 0) > stop_started
                         and now-timings['rpm'] < 0.5 and max(abs(x) for x in msg.data) <= 1)
                stationary_since = (stationary_since or now) if fresh else None
                if stationary_since and now-stationary_since >= 0.5:
                    stopped = list(msg.data)
                    break
                time.sleep(0.05)
            if stopped is None:
                fault = fault or 'Final stop feedback was not confirmed'
        final_pose = pose()
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
        for handle in handles:
            handle.close()
        logfile.close()
        report = {'started': started.isoformat(), 'wheels_lifted': True, 'preflight_seconds': 60,
                  'command_policy': 'one wheel per 12.5ms slot, integer RPM coalescing, 200ms refresh',
                  'route_sequence': ROUTES, 'results': results, 'fault': fault,
                  'final_odom': final_pose, 'final_stop_rpm': stopped,
                  'elapsed': time.monotonic()-test_start, 'motion_ack_counts': dict(ack_counts),
                  'last_can_diagnostics': state.get('diagnostics'), 'before': before,
                  'after': snapshot(), 'raw_samples': stem + '.jsonl',
                  'passed': not fault and len(results) == len(ROUTES)}
        (ROOT / (stem + '.json')).write_text(json.dumps(report, indent=2) + '\n')
        print('RESULT ' + json.dumps(report), flush=True)
    return bool(fault)


if __name__ == '__main__':
    raise SystemExit(main())
