#!/usr/bin/env python3
"""Native Y42 X firmware, fixed 6B checksum, standard angle/speed scaling.
Only chassis addresses 1..4 are permitted. No broadcast or auto-enable.
"""
import math
import struct
import time
import threading
import socket
import select
import errno
from collections import deque

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
FRAME = struct.Struct('=IB3x8s')
STOP = bytes.fromhex('FE 98 00 6B')


def speed_payload(rpm, accel=50, limit=30):
    if not math.isfinite(rpm):
        raise ValueError('nonfinite RPM')
    rpm = max(-limit, min(limit, rpm))
    raw = int(abs(rpm) * 10 + 0.5)
    return bytes([0xF6, int(rpm < 0), accel >> 8, accel & 255,
                  raw >> 8, raw & 255, 0, 0x6B])


def parse_reply(can_id, payload):
    if not can_id & CAN_EFF_FLAG or can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    raw_id = can_id & 0x1FFFFFFF
    addr = raw_id >> 8
    if raw_id & 255 or addr not in (1, 2, 3, 4):
        return None
    if len(payload) == 5 and payload[0] == 0x35 and payload[1] in (0, 1) and payload[-1] == 0x6B:
        rpm = ((payload[2] << 8) | payload[3]) / 10.0
        return addr, 'speed', -rpm if payload[1] else rpm
    if len(payload) == 3 and payload[0] == 0x3A and payload[-1] == 0x6B:
        return addr, 'status', payload[1]
    return None


def parse_stm32_stop(can_id, payload):
    """STM32 standard-frame events; native Y42 replies remain separate."""
    if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG) or len(payload) != 8:
        return ''
    if can_id == 0x101 and payload[0] == 0x06:
        return 'STM32 emergency event: motor=%d reason=0x%02X' % (payload[1], payload[2])
    if can_id == 0x112 and payload[:3] == bytes([0x02, 0x01, 0x01]):
        return 'STM32 physical emergency-stop button'
    return ''


class Feedback:
    def __init__(self, ids, stall_warning_timeout=0.5, feedback_timeout=1.0):
        if not math.isfinite(stall_warning_timeout) or not 0 <= stall_warning_timeout <= 1.0:
            raise ValueError('stall_warning_timeout must be finite and within 0..1 seconds')
        self.stall_warning_timeout = stall_warning_timeout
        self.feedback_timeout = feedback_timeout
        self.stall_since = {}
        self.ids = ids
        self.speed = {i: 0.0 for i in ids}
        self.status = {i: 0 for i in ids}
        self.seen = {}

    def update(self, reply, now):
        if reply is None or reply[0] not in self.ids:
            return
        addr, kind, value = reply
        if kind == 'status':
            previous = self.seen.get((addr, kind))
            if value & 0x04:
                if addr not in self.stall_since or previous is None or now - previous > self.feedback_timeout:
                    self.stall_since[addr] = now
            else:
                self.stall_since.pop(addr, None)
        (self.speed if kind == 'speed' else self.status)[addr] = value
        self.seen[addr, kind] = now

    def problem(self, now, timeout, strict_warning=False):
        for addr in self.ids:
            for kind in ('speed', 'status'):
                if (addr, kind) not in self.seen or now - self.seen[addr, kind] > timeout:
                    return 'address %d: missing/stale %s reply' % (addr, kind)
            # Vendor manual V1.1 section 5.5.15: bit7 Oac_TF is a
            # sticky power-cycle record, DEFAULT 1, not a live power fault.
            # 0x83 is explicitly illustrated as enabled + reached + history.
            # Driver protection is immediate. Only raw stall detection gets a
            # bounded confirmation window while already armed; no driver writes.
            if self.status[addr] & 0x08:
                return 'address %d: driver stall PROTECTION fault flags 0x%02X' % (addr, self.status[addr])
            if not self.status[addr] & 1:
                return 'address %d: motor disabled (enable on driver before arming)' % addr
            if self.status[addr] & 0x04:
                duration = max(0.0, now - self.stall_since.get(addr, now))
                if strict_warning or duration >= self.stall_warning_timeout:
                    return ('address %d: stall warning fault flags 0x%02X, continuous %.3fs (limit %.3fs)'
                            % (addr, self.status[addr], duration, self.stall_warning_timeout))
        return ''


class TxSchedule:
    """No queued velocity commands, no catch-up bursts. One CAN frame per slot.

    Running: each motor speed ~20 Hz; each of eight queries ~10 Hz.
    Disarmed: each motor stop ~4 Hz. TX fault: only two bounded stop rounds,
    no queries/motion until process restart. App pacing cannot repair a dead bus.
    """
    def __init__(self, ids):
        self.ids = list(ids)
        self.queries = [(a, func) for a in ids for func in (0x35, 0x3A)]
        self.control_index = self.query_index = 0
        self.next_slot = self.next_control = self.next_query = 0.0
        self.pending_stops = deque()
        self.tx_fault = False
        self.failures = 0

    def request_stop(self):
        if not self.pending_stops and not self.tx_fault:
            self.pending_stops.extend(self.ids * 2)

    def failed(self, now):
        self.failures += 1
        if not self.tx_fault:
            self.tx_fault = True
            self.pending_stops = deque(self.ids * 2)
        # On a full queue never immediately retry another send.
        self.next_slot = now + 0.05

    def next(self, now, armed):
        if now < self.next_slot:
            return None
        if self.pending_stops:
            self.next_slot = now + (0.05 if self.tx_fault else 0.004)
            return self.pending_stops.popleft(), 'stop'
        if self.tx_fault:
            return None
        if now >= self.next_control:
            self.next_slot = now + 0.004
            self.next_control = now + (0.0125 if armed else 0.0625)
            addr = self.ids[self.control_index]
            self.control_index = (self.control_index + 1) % len(self.ids)
            return addr, 'control' if armed else 'stop'
        if now >= self.next_query:
            self.next_slot = now + 0.004
            self.next_query = now + 0.0125
            addr, func = self.queries[self.query_index]
            self.query_index = (self.query_index + 1) % len(self.queries)
            return addr, func
        return None


class DirectNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import Float32MultiArray, UInt8MultiArray, Bool, String
        from std_srvs.srv import Trigger, TriggerResponse
        self.ros = rospy
        self.FloatArray, self.ByteArray, self.Bool, self.String = Float32MultiArray, UInt8MultiArray, Bool, String
        self.Response = TriggerResponse
        self.lock = threading.RLock()
        self.monitor_stm32_events = bool(rospy.get_param('~monitor_stm32_events', False))
        self.ids = rospy.get_param('~motor_ids', [1, 2, 3, 4])
        self.signs = rospy.get_param('~direction_signs', [1, 1, 1, 1])
        self.limit = float(rospy.get_param('~max_rpm', 30.0))
        self.accel = int(rospy.get_param('~acceleration_rpm_s', 50))
        self.timeout = float(rospy.get_param('~feedback_timeout', 1.0))
        self.cmd_timeout = float(rospy.get_param('~command_timeout', 0.5))
        self.stall_warning_timeout = float(rospy.get_param('~stall_warning_timeout', 0.5))
        self.startup_zero_duration = float(rospy.get_param('~startup_zero_duration', 1.0))
        self.inactive_wheel_mode = rospy.get_param('~inactive_wheel_mode', 'stop')
        if (len(self.ids) != 4 or set(self.ids) != {1, 2, 3, 4}
                or len(self.signs) != 4 or any(s not in (-1, 1) for s in self.signs)
                or not 0 < self.limit <= 3000 or not 0 <= self.accel <= 65535
                or not 0.2 <= self.timeout <= 5 or not 0.05 <= self.cmd_timeout <= 1
                or not 0.0 <= self.startup_zero_duration <= 5.0
                or self.inactive_wheel_mode not in ('stop', 'speed_zero')):
            raise ValueError('Invalid IDs/signs/limits/timeouts; chassis IDs must be a permutation of 1..4')
        self.feedback = Feedback(self.ids, self.stall_warning_timeout, self.timeout)
        self.tx_schedule = TxSchedule(self.ids)
        self.target = [0.0] * 4
        self.cmd_time = None
        self.armed = False
        self.zero_init_until = 0.0
        self.external_stop = False
        self.fault = ''
        self.running = True
        self.reason = 'disarmed: wait for all feedback, then call /y42_direct/arm with zero keyboard input'
        self.rpm_pub = rospy.Publisher('/motor_state', Float32MultiArray, queue_size=1)
        self.flags_pub = rospy.Publisher('/motor_status_flags', UInt8MultiArray, queue_size=1)
        self.ready_pub = rospy.Publisher('/y42_direct/ready', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/y42_direct/status', String, queue_size=1, latch=True)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1, latch=True)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Do not treat locally transmitted queries (including other sockets) as feedback.
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        self.sock.bind((rospy.get_param('~can_device', 'can0'),))
        self.sock.setblocking(False)
        rospy.Subscriber('/motor_velocity_cmd', Float32MultiArray, self.command, queue_size=1)
        rospy.Subscriber('/emergency_stop', Bool, self.estop, queue_size=1)
        rospy.Service('/y42_direct/arm', Trigger, self.arm)
        rospy.Service('/y42_direct/disarm', Trigger, self.disarm)
        rospy.logwarn('Stall warning grace %.3fs; 0x08 protection remains immediate; driver settings unchanged.', self.stall_warning_timeout)
        rospy.logwarn('Y42 native X (STM32 event monitor=%s): IDs=%s max_rpm=%.1f, NOT armed. No auto-enable, no broadcast, no IDs 5/6.', self.monitor_stm32_events, self.ids, self.limit)

    def send(self, addr, payload):
        assert addr in self.ids and 0 < len(payload) <= 8
        frame = FRAME.pack(CAN_EFF_FLAG | (addr << 8), len(payload), payload.ljust(8, b'\x00'))
        try:
            if self.sock.send(frame) != len(frame):
                raise OSError('short CAN write')
            return True
        except OSError as exc:
            self.armed = False
            self.tx_schedule.failed(time.monotonic())
            # Preserve the FIRST failure for diagnostics and never clear via /arm.
            if not self.fault:
                self.fault = 'CAN TX failed: ' + str(exc)
                if exc.errno in (errno.ENOBUFS, errno.EAGAIN, errno.EWOULDBLOCK):
                    self.fault += '; TX backpressure: no motion replay; check link/ACK/USB, restart after repair'
            self.reason = self.fault
            self.ros.logerr_throttle(2, self.fault)
            return False

    def stop_all(self):
        # Schedule stops, rather than bursting four additional writes into a
        # possibly full queue. Motion is disabled by caller before this request.
        self.tx_schedule.request_stop()

    def transmit_one(self, now):
        job = self.tx_schedule.next(now, self.armed and not self.external_stop and not self.fault)
        if job is None:
            return
        addr, kind = job
        if kind == 'control':
            idx = self.ids.index(addr)
            if now < self.zero_init_until:
                # Initialize Y42's X-firmware speed loop with an explicit F6
                # zero-speed command after arming. FE 98 is an immediate-stop
                # command, not a speed-mode initialization.
                payload = speed_payload(0.0, self.accel, self.limit)
            else:
                if abs(self.target[idx]) < 0.01:
                    active_others = any(abs(value) >= 0.01 for value in self.target)
                    payload = (speed_payload(0.0, self.accel, self.limit)
                               if self.inactive_wheel_mode == 'speed_zero' and active_others else STOP)
                else:
                    payload = speed_payload(self.target[idx]*self.signs[idx], self.accel, self.limit)
        elif kind == 'stop':
            payload = STOP
        else:
            payload = bytes([kind, 0x6B])
        self.send(addr, payload)

    def trip(self, reason):
        self.armed = False
        self.zero_init_until = 0.0
        self.reason = reason
        self.target = [0.0] * 4
        self.stop_all()

    def receive_frame(self, can_id, payload, now):
        if self.monitor_stm32_events:
            reason = parse_stm32_stop(can_id, payload)
            if reason:
                # A one-shot STM32 stop must not be overwritten by the next F6.
                self.external_stop = True
                self.trip(reason + '; latched, correct cause and restart test launch')
                self.emergency_pub.publish(self.Bool(data=True))
                return
        self.feedback.update(parse_reply(can_id, payload), now)

    def command(self, msg):
        with self.lock:
            if len(msg.data) != 4 or not all(math.isfinite(x) for x in msg.data):
                self.fault = 'invalid wheel RPM command'
                self.trip(self.fault)
                return
            self.target = list(msg.data)
            self.cmd_time = time.monotonic()

    def estop(self, msg):
        if msg.data:
            with self.lock:
                self.external_stop = True
                self.trip('external emergency stop latched: correct cause and restart test launch')

    def arm(self, _):
        with self.lock:
            now = time.monotonic()
            why = self.feedback.problem(now, self.timeout, strict_warning=True)
            if self.external_stop:
                why = 'external emergency stop latched; cannot reset via arm service'
            elif self.fault:
                why = self.fault + '; correct cause and restart'
            elif self.cmd_time is None or now-self.cmd_time > self.cmd_timeout or any(abs(x)>0.01 for x in self.target):
                why = 'need fresh ZERO command; stop keyboard before arming'
            elif any(abs(v)>1.0 for v in self.feedback.speed.values()):
                why = 'wheels are not stationary'
            if why:
                return self.Response(False, why)
            self.armed = True
            self.zero_init_until = now + self.startup_zero_duration
            self.reason = 'armed; initializing zero-speed control for %.2fs' % self.startup_zero_duration
            self.ros.loginfo('Y42 speed-loop initialization: sending F6 zero-speed commands for %.2fs',
                             self.startup_zero_duration)
            return self.Response(True, 'armed: chassis IDs 1..4 only')

    def disarm(self, _):
        with self.lock:
            self.trip('manually disarmed')
            return self.Response(True, self.reason)

    def run(self):
        next_report = 0
        try:
            with self.lock:
                self.stop_all()
            while not self.ros.is_shutdown():
                readable, _, _ = select.select([self.sock], [], [], 0.002)
                with self.lock:
                    if readable:
                        for _ in range(64):
                            try:
                                raw = self.sock.recv(16)
                            except BlockingIOError:
                                break
                            if len(raw) != 16:
                                continue
                            cid, dlc, data = FRAME.unpack(raw)
                            if dlc <= 8:
                                self.receive_frame(cid, data[:dlc], time.monotonic())
                    now = time.monotonic()
                    why = self.feedback.problem(now, self.timeout)
                    if self.armed and why:
                        self.trip(why + '; stopped, explicit re-arm required')
                    if self.armed and (self.cmd_time is None or now-self.cmd_time > self.cmd_timeout):
                        self.trip('wheel command timeout; explicit re-arm required')
                    self.transmit_one(now)
                    if now >= next_report:
                        next_report = now + 0.2
                        strict_problem = self.feedback.problem(now, self.timeout, strict_warning=True)
                        ready = not strict_problem and not self.external_stop and not self.fault
                        self.ready_pub.publish(self.Bool(data=ready))
                        warnings = []
                        for i, addr in enumerate(self.ids):
                            if self.feedback.status[addr] & 0x04:
                                warnings.append('id=%d flags=0x%02X target=%.1f actual=%.1f RPM duration=%.3fs/%.3fs' % (
                                    addr, self.feedback.status[addr], self.target[i]*self.signs[i],
                                    self.feedback.speed[addr], max(0.0, now-self.feedback.stall_since.get(addr, now)),
                                    self.feedback.stall_warning_timeout))
                        text = ('armed; transient stall WARNING: ' + '; '.join(warnings)) if self.armed and warnings else (
                            'armed' if self.armed else (self.fault or strict_problem or self.reason))
                        self.status_pub.publish(self.String(data=text))
                        if not ready:
                            self.ros.logwarn_throttle(0.5 if self.armed else 2, text)
                        # Never publish stale RPM as valid odometry feedback.
                        if not why:
                            self.rpm_pub.publish(self.FloatArray(data=[self.feedback.speed[a]*self.signs[i] for i,a in enumerate(self.ids)]))
                            self.flags_pub.publish(self.ByteArray(data=[self.feedback.status[a] for a in self.ids]))
        finally:
            with self.lock:
                self.armed = False
                self.target = [0.0] * 4
                self.stop_all()
                # Bounded best-effort shutdown. A disconnected CAN cannot be
                # stopped by software; use the hardware emergency stop.
                deadline = time.monotonic() + 0.6
                while self.tx_schedule.pending_stops and time.monotonic() < deadline:
                    self.transmit_one(time.monotonic())
                    time.sleep(0.004)
                self.sock.close()


if __name__ == '__main__':
    import rospy
    rospy.init_node('y42_direct_node')
    try:
        DirectNode().run()
    except (OSError, ValueError) as exc:
        rospy.logfatal('Y42 direct startup/runtime failed: %s', exc)
        raise SystemExit(1)
