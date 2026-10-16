#!/usr/bin/env python3
"""Bounded lifted-wheel rotation test using the production CAN start interlock."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


class Feedback:
    def __init__(self):
        self.ready = False
        self.ready_at = None
        self.rpm = None
        self.rpm_at = None
        self.robot_state = ''
        self.estop = False

    def fresh(self, now):
        return (self.ready and self.ready_at is not None and now - self.ready_at < 0.5
                and self.rpm_at is not None and now - self.rpm_at < 0.5
                and self.rpm is not None and len(self.rpm) == 4
                and all(math.isfinite(v) and abs(v) <= 15 for v in self.rpm))

    def stopped(self, now, stop_started):
        return (self.rpm_at is not None and self.rpm_at > stop_started
                and now - self.rpm_at < 0.5 and self.rpm is not None
                and len(self.rpm) == 4
                and all(math.isfinite(v) and abs(v) <= 1 for v in self.rpm))


class Rotation:
    def __init__(self, rpm, duration, button_timeout):
        self.rpm = rpm
        self.duration = duration
        self.button_timeout = button_timeout
        self.prepared_at = None
        self.started_at = None

    def command(self, feedback, now):
        if feedback.estop:
            raise RuntimeError('底盘已触发急停')
        if not feedback.fresh(now):
            raise RuntimeError('底盘链路或四轮反馈未就绪/已过期')
        if self.prepared_at is None:
            self.prepared_at = now
        if feedback.robot_state == 'RUNNING':
            if self.started_at is None:
                self.started_at = now
            if now - self.started_at >= self.duration:
                return None
        elif self.started_at is not None:
            raise RuntimeError('运动启动状态退出 RUNNING')
        elif feedback.robot_state in ('START_FAILED', 'ESTOP_LATCHED', 'HEARTBEAT_LOST'):
            raise RuntimeError('底盘启动失败：' + feedback.robot_state)
        elif now - self.prepared_at >= self.button_timeout:
            raise RuntimeError('等待实体启动按钮超时')
        return [self.rpm] * 4


def master_call(uri, method):
    # Limit XMLRPC probes even when ROS_MASTER_URI points at an offline host.
    previous = socket.getdefaulttimeout()
    socket.setdefaulttimeout(1.0)
    try:
        with xmlrpc.client.ServerProxy(uri) as master:
            return getattr(master, method)('/chassis_rotation_test')
    finally:
        socket.setdefaulttimeout(previous)


def check_existing_controllers(uri):
    try:
        result = master_call(uri, 'getSystemState')
    except (OSError, xmlrpc.client.Error):
        return
    if result[0] != 1:
        raise RuntimeError('无法检查现有 ROS 控制节点：' + str(result[1]))
    reserved = {'/motor_velocity_cmd', '/motor_state', '/can_rx', '/cmd_vel_muxed'}
    busy = [(topic, nodes) for topic, nodes in result[2][0] if topic in reserved and nodes]
    if busy:
        raise RuntimeError('请先关闭现有底盘控制程序：' + str(busy))


def configure_can(device):
    links = json.loads(subprocess.check_output(
        ['ip', '-json', '-details', 'link', 'show', 'dev', device], text=True))
    if len(links) != 1 or links[0].get('linkinfo', {}).get('info_kind') != 'can':
        raise RuntimeError(device + ' 不是物理 CAN 接口')
    link = links[0]
    data = link['linkinfo']['info_data']
    if 'UP' in link['flags']:
        modes = data.get('ctrlmode', [])
        invalid_mode = (any(modes.get(k, False) for k in ('listen-only', 'loopback'))
                        if isinstance(modes, dict)
                        else any(str(k).upper() in ('LISTEN-ONLY', 'LOOPBACK') for k in modes))
        if (data.get('bittiming', {}).get('bitrate') != 500000
                or data.get('state') != 'ERROR-ACTIVE' or invalid_mode):
            raise RuntimeError(device + ' 已启用但状态/位速率不正确；请先停止其他程序并检查接口')
        print(device + ' 已就绪，保留现有 500 kbit/s 配置。', flush=True)
        return
    prefix = [] if os.geteuid() == 0 else ['sudo']
    subprocess.run(prefix + ['ip', 'link', 'set', device, 'type', 'can', 'bitrate',
                              '500000', 'sample-point', '0.833', 'loopback', 'off',
                              'listen-only', 'off'], check=True)
    subprocess.run(prefix + ['ip', 'link', 'set', device, 'up'], check=True)
    print(device + ' 已配置为 500 kbit/s。', flush=True)


def shutdown_launch(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser(description='架空底盘低速旋转，按实体启动按钮后计时，结束自动停车。')
    parser.add_argument('--device', default='can0')
    parser.add_argument('--rpm', type=float, default=8, help='四轮目标 RPM，负数反向，范围 ±1..10，默认 8')
    parser.add_argument('--duration', type=float, default=3, help='实际开始旋转后的秒数，范围 0.5..10，默认 3')
    parser.add_argument('--button-timeout', type=float, default=30, help='等待按钮秒数，范围 5..60，默认 30')
    args = parser.parse_args()
    if (not math.isfinite(args.rpm) or not 1 <= abs(args.rpm) <= 10
            or not math.isfinite(args.duration) or not 0.5 <= args.duration <= 10
            or not math.isfinite(args.button_timeout) or not 5 <= args.button_timeout <= 60):
        parser.error('RPM/时间参数超出允许范围')

    # Import after --help/argument validation; the shell entry loads ROS first.
    import rospy
    from std_msgs.msg import Bool, Float32MultiArray, String

    feedback = Feedback()
    interrupted = False

    def interrupt(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    # Prevent a second instance of this helper from using the same chassis.
    with open('/tmp/chassis_rotation_test.lock', 'a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有一键旋转测试在运行')
        check_existing_controllers(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
        configure_can(args.device)
        if interrupted:
            return 130
        log_dir = Path(tempfile.mkdtemp(prefix='chassis_rotation_test_'))
        env = dict(os.environ, ROS_LOG_DIR=str(log_dir / 'ros'))
        proc = cmd_pub = estop_pub = None
        fault = ''
        stopped = False
        with (log_dir / 'launch.log').open('w') as launch_log:
            try:
                proc = subprocess.Popen(
                    ['roslaunch', 'can_motor_interface', 'can_interface.launch',
                     'can_device:=' + args.device, 'max_rpm:=10'], env=env,
                    stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 15
                uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
                while True:
                    if interrupted or proc.poll() is not None:
                        raise RuntimeError('启动已中断或 CAN 节点启动失败')
                    try:
                        if master_call(uri, 'getPid')[0] == 1:
                            break
                    except (OSError, xmlrpc.client.Error):
                        pass
                    if time.monotonic() >= deadline:
                        raise RuntimeError('等待 ROS master 超时')
                    time.sleep(0.1)
                rospy.init_node('chassis_rotation_test', disable_signals=True)

                def ready(msg):
                    feedback.ready = msg.data
                    feedback.ready_at = time.monotonic()

                def rpm(msg):
                    feedback.rpm = list(msg.data)
                    feedback.rpm_at = time.monotonic()

                def estop(msg):
                    feedback.estop = feedback.estop or msg.data

                rospy.Subscriber('/motor_link_ready', Bool, ready, queue_size=1)
                rospy.Subscriber('/motor_state', Float32MultiArray, rpm, queue_size=1)
                rospy.Subscriber('/robot_state', String,
                                 lambda msg: setattr(feedback, 'robot_state', msg.data), queue_size=1)
                rospy.Subscriber('/emergency_stop', Bool, estop, queue_size=1)
                cmd_pub = rospy.Publisher('/motor_velocity_cmd', Float32MultiArray, queue_size=1)
                estop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1, latch=True)
                print('等待四轮反馈就绪；Ctrl+C 可随时停车退出。', flush=True)
                preflight_started = time.monotonic()
                deadline = preflight_started + 15
                stationary_since = None
                while True:
                    if interrupted or rospy.is_shutdown() or proc.poll() is not None or feedback.estop:
                        raise RuntimeError('等待反馈时中断、节点退出或触发急停')
                    if time.monotonic() >= deadline:
                        raise RuntimeError('四轮未在 15 秒内就绪并确认静止')
                    cmd_pub.publish(Float32MultiArray(data=[0.0] * 4))
                    now = time.monotonic()
                    stationary = (feedback.fresh(now) and feedback.stopped(now, preflight_started)
                                  and cmd_pub.get_num_connections() > 0)
                    stationary_since = ((stationary_since if stationary_since is not None else now)
                                        if stationary else None)
                    if stationary_since is not None and now - stationary_since >= 0.5:
                        break
                    time.sleep(0.05)
                rotation = Rotation(args.rpm, args.duration, args.button_timeout)
                announced = False
                prompted = False
                while True:
                    if interrupted or rospy.is_shutdown() or proc.poll() is not None:
                        raise RuntimeError('用户中断或 CAN 节点退出')
                    command = rotation.command(feedback, time.monotonic())
                    if rotation.started_at is not None and not announced:
                        print('检测到真实启动，开始计时。', flush=True)
                        announced = True
                    if command is None:
                        print('测试时间到，正在停车。', flush=True)
                        break
                    cmd_pub.publish(Float32MultiArray(data=command))
                    if not prompted and time.monotonic() - rotation.prepared_at >= 0.2:
                        print('已就绪：请按 STM32 实体启动按钮。目标 %.1f RPM，转动 %.1f 秒。'
                              % (args.rpm, args.duration), flush=True)
                        prompted = True
                    time.sleep(0.05)
            except Exception as exc:
                fault = str(exc)
                print('停止测试：' + fault, flush=True)
            finally:
                try:
                    if cmd_pub is not None:
                        stop_started = time.monotonic()
                        stable_since = None
                        while time.monotonic() - stop_started < 4 and not rospy.is_shutdown():
                            cmd_pub.publish(Float32MultiArray(data=[0.0] * 4))
                            estop_pub.publish(Bool(data=True))
                            now = time.monotonic()
                            stable_since = ((stable_since if stable_since is not None else now)
                                            if feedback.stopped(now, stop_started) else None)
                            if stable_since is not None and now - stable_since >= 0.5:
                                stopped = True
                                print('已确认四轮停车：' + str(feedback.rpm) + ' RPM', flush=True)
                                break
                            time.sleep(0.05)
                        if not stopped:
                            fault = fault or '未能用新速度反馈确认停车，请检查底盘'
                            print('未能用新速度反馈确认停车，请检查底盘。', flush=True)
                finally:
                    shutdown_launch(proc)
                    if cmd_pub is not None:
                        rospy.signal_shutdown('旋转测试结束')
                    print('日志：' + str(log_dir), flush=True)
        return 1 if fault else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print('启动失败：' + str(exc), flush=True)
        raise SystemExit(1)
