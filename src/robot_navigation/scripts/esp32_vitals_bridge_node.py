#!/usr/bin/env python3
import json, socket, threading, time
import rospy
from std_msgs.msg import Float32, Bool, String

class VitalsBridge:
    def __init__(self):
        self.host = rospy.get_param('~bind_address', '0.0.0.0')
        self.port = int(rospy.get_param('~tcp_port', 8899))
        self.timeout = float(rospy.get_param('~client_timeout_s', 10.0))
        self.hr = rospy.Publisher('/vitals/heart_rate', Float32, queue_size=10)
        self.temp = rospy.Publisher('/vitals/temperature', Float32, queue_size=10)
        self.measuring = rospy.Publisher('/vitals/measuring', Bool, queue_size=10, latch=True)
        self.status = rospy.Publisher('/vitals/status', String, queue_size=10, latch=True)
        self.last_rx = rospy.Time.now()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port)); self.server.listen(2)
        rospy.loginfo('[esp32_vitals_bridge] TCP listening on %s:%d', self.host, self.port)
        self.status.publish('offline'); self.measuring.publish(False)
        threading.Thread(target=self.accept_loop, daemon=True).start()
        rospy.Timer(rospy.Duration(1.0), self.watchdog)
    def watchdog(self, _):
        if (rospy.Time.now()-self.last_rx).to_sec() > self.timeout:
            self.status.publish('offline')
    def accept_loop(self):
        while not rospy.is_shutdown():
            try:
                client, addr = self.server.accept()
                rospy.loginfo('[esp32_vitals_bridge] client %s connected', addr)
                threading.Thread(target=self.client_loop, args=(client,), daemon=True).start()
            except Exception as e:
                if not rospy.is_shutdown(): rospy.logwarn('%s', e)
    def handle(self, obj):
        self.last_rx=rospy.Time.now(); self.status.publish('online')
        try:
            if 'heart_rate' in obj: self.hr.publish(float(obj['heart_rate']))
            if 'temperature' in obj: self.temp.publish(float(obj['temperature']))
            self.measuring.publish(bool(obj.get('measuring', True)))
            if obj.get('heart_rate_valid') is False or obj.get('temperature_valid') is False:
                self.status.publish('online_invalid')
        except (TypeError, ValueError) as e: rospy.logwarn_throttle(2.0, 'invalid vitals: %s', e)
    def text_handle(self, line):
        line=line.strip()
        if not line: return
        try: self.handle(json.loads(line)); return
        except ValueError: pass
        # Compatibility with current ESP32 text protocol.
        import re
        m=re.search(r'平均心率\s*:\s*([0-9.]+)',line)
        if m: self.hr.publish(float(m.group(1)))
        m=re.search(r'体温\s*:\s*([0-9.]+)',line)
        if m: self.temp.publish(float(m.group(1)))
        if m: self.status.publish('online_text')
    def client_loop(self, client):
        client.settimeout(1.0); buf=b''
        try:
            while not rospy.is_shutdown():
                try: data=client.recv(1024)
                except socket.timeout: continue
                if not data: break
                buf+=data
                while b'\n' in buf:
                    raw,buf=buf.split(b'\n',1); self.text_handle(raw.decode('utf-8','replace'))
        finally: client.close()
if __name__=='__main__':
    rospy.init_node('esp32_vitals_bridge_node'); VitalsBridge(); rospy.spin()
