#!/usr/bin/env python3
"""USB/UART JSON-line bridge. TCP remains an explicitly selected fallback."""
import json
import math
import socket
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32, String


class LineBuffer:
    """Bound memory and discard the entire oversized line, not just its prefix."""
    def __init__(self, limit=1024):
        self.limit = limit
        self.buffer = bytearray()
        self.discarding = False

    def feed(self, data):
        lines = []
        for value in data:
            if value == 10:
                if not self.discarding and self.buffer:
                    lines.append(bytes(self.buffer))
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                if len(self.buffer) >= self.limit:
                    self.buffer.clear()
                    self.discarding = True
                else:
                    self.buffer.append(value)
        return lines


def finite_number(value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('measurement must be a JSON number')
    try:
        value = float(value)
    except (OverflowError, ValueError):
        raise ValueError('measurement is out of range')
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError('measurement is not finite/in sensor range')
    return value


def validate_frame(obj):
    if not isinstance(obj, dict) or obj.get('type') != 'vitals':
        raise ValueError('expected a vitals JSON object')
    if type(obj.get('version')) is not int or obj['version'] != 1:
        raise ValueError('unsupported protocol version')
    for key in ('seq', 'uptime_ms'):
        if type(obj.get(key)) is not int or not 0 <= obj[key] <= 0xFFFFFFFF:
            raise ValueError('invalid ' + key)
    for key in ('measuring', 'heart_rate_valid', 'temperature_valid'):
        if type(obj.get(key)) is not bool:
            raise ValueError(key + ' must be boolean')
    if obj.get('mode') not in ('button', 'continuous'):
        raise ValueError('invalid acquisition mode')
    for key in ('finger_present', 'heart_sensor_ready', 'temperature_sensor_ready'):
        if key in obj and type(obj[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    hr_valid = obj['measuring'] and obj['heart_rate_valid']
    temp_valid = obj['measuring'] and obj['temperature_valid']
    # These limits are protocol/sensor sanity checks, not clinical validation.
    heart_rate = finite_number(obj.get('heart_rate'), 1, 300) if hr_valid else float('nan')
    temperature = finite_number(obj.get('temperature'), -70, 380) if temp_valid else float('nan')
    return heart_rate, temperature, hr_valid, temp_valid


class VitalsBridge:
    def __init__(self):
        self.transport = rospy.get_param('~transport', 'serial')
        self.device = rospy.get_param('~serial_device', '/dev/ttyUSB0')
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.timeout = float(rospy.get_param('~client_timeout_s', 3.0))
        self.retry_delay = float(rospy.get_param('~reconnect_interval_s', 1.0))
        if self.timeout <= 0 or self.retry_delay <= 0 or self.baudrate <= 0:
            raise ValueError('timeouts/baudrate must be positive')
        self.host = rospy.get_param('~bind_address', '0.0.0.0')
        self.port = int(rospy.get_param('~tcp_port', 8899))
        self.hr = rospy.Publisher('/vitals/heart_rate', Float32, queue_size=10)
        self.temp = rospy.Publisher('/vitals/temperature', Float32, queue_size=10)
        self.hr_valid = rospy.Publisher('/vitals/heart_rate_valid', Bool, queue_size=10, latch=True)
        self.temp_valid = rospy.Publisher('/vitals/temperature_valid', Bool, queue_size=10, latch=True)
        self.measuring = rospy.Publisher('/vitals/measuring', Bool, queue_size=10, latch=True)
        self.status = rospy.Publisher('/vitals/status', String, queue_size=10, latch=True)
        # One atomic packet prevents cross-topic ordering from mixing sessions.
        self.frame = rospy.Publisher('/vitals/frame', String, queue_size=10, latch=True)
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.connection = None
        self.server = None
        self.last_rx = None
        self.last_seq = None
        self.last_uptime = None
        self.online = False
        self.set_offline()
        if self.transport == 'serial':
            import serial  # TCP fallback need not depend on pyserial.
            self.serial_module = serial
            target = self.serial_loop
        elif self.transport == 'tcp':
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((self.host, self.port))
            self.server.listen(1)
            self.server.settimeout(0.5)
            target = self.tcp_loop
        else:
            raise ValueError('transport must be serial or tcp')
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=target, daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(0.25), self.watchdog)

    def set_offline(self):
        with self.lock:
            self.online = False
            self.last_rx = self.last_seq = self.last_uptime = None
            self.hr.publish(float('nan'))
            self.temp.publish(float('nan'))
            self.hr_valid.publish(False)
            self.temp_valid.publish(False)
            self.measuring.publish(False)
            self.status.publish('offline')
            self.frame.publish(json.dumps({
                'status': 'offline', 'measuring': False, 'heart_rate': None,
                'temperature': None, 'heart_rate_valid': False,
                'temperature_valid': False}, allow_nan=False))

    def watchdog(self, _):
        with self.lock:
            if self.online and time.monotonic() - self.last_rx > self.timeout:
                self.set_offline()

    def handle(self, obj):
        # Validate the ENTIRE frame before updating freshness or publishing anything.
        heart_rate, temperature, hr_valid, temp_valid = validate_frame(obj)
        with self.lock:
            if self.last_seq is not None:
                rebooted = obj['uptime_ms'] < self.last_uptime
                delta = (obj['seq'] - self.last_seq) & 0xFFFFFFFF
                if not rebooted and (delta == 0 or delta >= 0x80000000):
                    return False  # repeated/out-of-order data is not a heartbeat
            self.last_seq = obj['seq']
            self.last_uptime = obj['uptime_ms']
            self.last_rx = time.monotonic()
            self.online = True
            self.hr.publish(heart_rate)
            self.temp.publish(temperature)
            self.hr_valid.publish(hr_valid)
            self.temp_valid.publish(temp_valid)
            self.measuring.publish(obj['measuring'])
            state = 'online_idle' if not obj['measuring'] else (
                'online' if hr_valid and temp_valid else 'online_invalid')
            self.status.publish(state)
            packet = dict(obj, status=state, heart_rate_valid=hr_valid,
                          temperature_valid=temp_valid,
                          heart_rate=heart_rate if hr_valid else None,
                          temperature=temperature if temp_valid else None)
            # Only publish protocol fields we have actually validated.
            packet = {key: packet[key] for key in (
                'type', 'version', 'seq', 'uptime_ms', 'mode', 'status',
                'measuring', 'heart_rate', 'temperature',
                'heart_rate_valid', 'temperature_valid')}
            for key in ('finger_present', 'heart_sensor_ready', 'temperature_sensor_ready'):
                if key in obj:
                    packet[key] = obj[key]
            self.frame.publish(json.dumps(packet, allow_nan=False))
        return True

    def text_handle(self, raw):
        try:
            text = raw.decode('utf-8') if isinstance(raw, bytes) else raw
            return self.handle(json.loads(text))
        except (ValueError, TypeError, UnicodeError, RecursionError) as exc:
            rospy.logwarn_throttle(5.0, '[vitals] discarded invalid frame: %s', exc)
            return False

    def read_stream(self, read):
        lines = LineBuffer()
        while not self.stop.is_set() and not rospy.is_shutdown():
            for line in lines.feed(read()):
                self.text_handle(line)

    def serial_loop(self):
        serial = self.serial_module
        while not self.stop.is_set() and not rospy.is_shutdown():
            port = None
            try:
                # Set control lines before opening to minimize auto-reset of ESP32.
                port = serial.Serial(port=None, baudrate=self.baudrate, timeout=0.2,
                                     write_timeout=0.2, rtscts=False, dsrdtr=False)
                port.dtr = False
                port.rts = False
                port.port = self.device
                self.connection = port
                port.open()
                rospy.loginfo('[vitals] serial opened: %s @ %d', self.device, self.baudrate)
                def read():
                    try:
                        return port.read(256)
                    except (TypeError, ValueError) as exc:
                        # Windows can clear overlapped IO handles before
                        # pyserial updates is_open during a concurrent close.
                        raise serial.SerialException('serial read backend: %s' % exc) from exc
                self.read_stream(read)
            except (serial.SerialException, OSError) as exc:
                if not self.stop.is_set():
                    rospy.logwarn_throttle(5.0, '[vitals] serial disconnected: %s', exc)
            finally:
                if port is not None:
                    port.close()
                self.connection = None
                self.set_offline()
            self.stop.wait(self.retry_delay)

    def tcp_loop(self):
        # One source owns the stream; parallel ESP32 clients cannot mix readings.
        while not self.stop.is_set() and not rospy.is_shutdown():
            client = None
            try:
                try:
                    client, addr = self.server.accept()
                except socket.timeout:
                    continue
                self.connection = client
                client.settimeout(0.2)
                rospy.loginfo('[vitals] TCP source connected: %s', addr)
                def read():
                    try:
                        data = client.recv(256)
                    except socket.timeout:
                        return b''
                    if not data:
                        raise ConnectionError('TCP EOF')
                    return data
                self.read_stream(read)
            except OSError as exc:
                if not self.stop.is_set():
                    rospy.logwarn_throttle(5.0, '[vitals] TCP disconnected: %s', exc)
            finally:
                if client is not None:
                    client.close()
                    self.connection = None
                    self.set_offline()

    def shutdown(self):
        self.stop.set()
        if self.server is not None:
            self.server.close()
        connection = self.connection
        if connection is not None:
            try:
                if self.transport == 'serial':
                    # Let the worker own close(); closing overlapped IO from a
                    # second thread races with serialwin32.read().
                    if connection.is_open:
                        connection.cancel_read()
                else:
                    connection.close()
            except OSError:
                pass
        if threading.current_thread() is not self.worker:
            self.worker.join(timeout=2.0)


if __name__ == '__main__':
    rospy.init_node('esp32_vitals_bridge_node')
    VitalsBridge()
    rospy.spin()
