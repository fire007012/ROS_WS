#!/usr/bin/env python3
"""ROS callbacks only enqueue state; Tk widgets are updated on the GUI thread."""
import json
import math
import threading
import time
import tkinter as tk

import rospy
from std_msgs.msg import Bool, Float32, String
from robot_navigation.msg import QrResult


def measurement_text(value, label, unit):
    return label + ('%.1f %s' % (value, unit) if math.isfinite(value) else '-- ' + unit)


class StatusWindow:
    def __init__(self):
        self.lock = threading.Lock()
        self.pending = {}  # coalesce bursts; memory does not grow with message rate
        self.last_vitals = time.monotonic()
        self.stale_timeout = float(rospy.get_param('~vitals_timeout_s', 3.0))
        self.sensor_hint = ''
        self.atomic_source = False
        self.vitals_state = 'offline'
        self.measuring_state = False
        self.root = tk.Tk()
        self.root.title('送药巡诊机器人状态')
        self.root.configure(bg='#101820')
        if rospy.get_param('~fullscreen', True):
            self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', lambda _: self.root.attributes('-fullscreen', False))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.labels = {}
        self.make('title', '送药巡诊机器人', 28, '#4dd0e1')
        for key, text in [('task', '当前任务：等待启动'), ('scan', '识别状态：未开始'),
                          ('bed1', '1号床条码：---'), ('bed3', '3号床条码：---'),
                          ('hr', '心率：-- bpm'), ('temp', '体温：-- °C'),
                          ('measure', '测量状态：等待按键'), ('esp', 'ESP32：离线')]:
            self.make(key, text, 18, '#f5f5f5')
        rospy.Subscriber('/robot_display', String, lambda m: self.set('task', '当前任务：' + m.data.replace('\n', ' | ')))
        rospy.Subscriber('/qr_result', QrResult, self.qr)
        rospy.Subscriber('/barcode_bed1', String, lambda m: self.set('bed1', '1号床条码：' + m.data))
        rospy.Subscriber('/barcode_bed3', String, lambda m: self.set('bed3', '3号床条码：' + m.data))
        rospy.Subscriber('/vitals/frame', String, self.frame)
        rospy.Subscriber('/vitals/heart_rate', Float32, lambda m: self.vital('hr', m.data, '心率：', 'bpm'))
        rospy.Subscriber('/vitals/temperature', Float32, lambda m: self.vital('temp', m.data, '体温：', '°C'))
        rospy.Subscriber('/vitals/heart_rate_valid', Bool, lambda m: self.invalid('hr', m.data, '心率：-- bpm'))
        rospy.Subscriber('/vitals/temperature_valid', Bool, lambda m: self.invalid('temp', m.data, '体温：-- °C'))
        rospy.Subscriber('/vitals/measuring', Bool, lambda m: self.legacy_set('_measuring', m.data))
        rospy.Subscriber('/vitals/status', String, lambda m: self.legacy_set('_status', m.data))
        self.root.after(50, self.pump)

    def make(self, key, text, size, color):
        label = tk.Label(self.root, text=text,
                         font=('DejaVu Sans', size, 'bold' if key == 'title' else 'normal'),
                         fg=color, bg='#101820', anchor='w', padx=30, pady=8)
        label.pack(fill='x')
        self.labels[key] = label

    def set(self, key, value):
        with self.lock:
            self.pending[key] = value
            if key in ('hr', 'temp', '_status'):
                self.last_vitals = time.monotonic()

    def legacy_set(self, key, value):
        with self.lock:
            if self.atomic_source:
                return
            self.pending[key] = value
            self.last_vitals = time.monotonic()

    def frame(self, msg):
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict) or obj.get('status') not in (
                    'online', 'online_idle', 'online_invalid', 'offline'):
                return
            for key in ('measuring', 'heart_rate_valid', 'temperature_valid'):
                if type(obj.get(key)) is not bool:
                    return
            values = {}
            for key, valid in (('heart_rate', 'heart_rate_valid'),
                               ('temperature', 'temperature_valid')):
                value = obj.get(key)
                if obj[valid] and obj['measuring'] and obj['status'] != 'offline':
                    if isinstance(value, bool) or not isinstance(value, (int, float)):
                        return
                    value = float(value)
                    if not math.isfinite(value):
                        return
                    values[key] = value
                else:
                    values[key] = float('nan')
            missing = []
            if obj.get('heart_sensor_ready') is False:
                missing.append('心率传感器未识别')
            if obj.get('temperature_sensor_ready') is False:
                missing.append('温度传感器未识别')
            with self.lock:
                self.atomic_source = True
                self.last_vitals = time.monotonic()
                self.pending.update({
                    '_status': obj['status'], '_measuring': obj['measuring'],
                    '_sensor_hint': '／' + '、'.join(missing) if missing else '',
                    'hr': measurement_text(values['heart_rate'], '心率：', 'bpm'),
                    'temp': measurement_text(values['temperature'], '体温：', '°C')})
        except (ValueError, TypeError, OverflowError, RecursionError):
            return

    def vital(self, key, value, label, unit):
        self.legacy_set(key, measurement_text(value, label, unit))

    def invalid(self, key, valid, text):
        if not valid:
            self.legacy_set(key, text)

    def qr(self, msg):
        self.set('scan', '识别状态：护士台二维码 %d%d，下一站 %d%d' %
                 (msg.first_bed, msg.first_box, msg.second_bed, msg.second_box))

    def pump(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
        with self.lock:
            updates, self.pending = self.pending, {}
            stale = time.monotonic() - self.last_vitals > self.stale_timeout
        self.sensor_hint = updates.pop('_sensor_hint', self.sensor_hint)
        self.vitals_state = updates.pop('_status', self.vitals_state)
        self.measuring_state = updates.pop('_measuring', self.measuring_state)
        for key, text in updates.items():
            self.labels[key].config(text=text)
        offline = stale or self.vitals_state == 'offline'
        status_names = {'online': '在线', 'online_idle': '在线／等待测量',
                        'online_invalid': '在线／部分数据无效', 'offline': '离线'}
        self.labels['esp'].config(text='ESP32：' + ('离线' if offline else
                                 status_names.get(self.vitals_state, self.vitals_state) + self.sensor_hint))
        self.labels['measure'].config(text='测量状态：' + ('连接中断' if offline else
                                      '测量中' if self.measuring_state else '已停止／按键开始'))
        if offline or not self.measuring_state:
            self.labels['hr'].config(text='心率：-- bpm')
            self.labels['temp'].config(text='体温：-- °C')
        self.root.after(50, self.pump)

    def close(self):
        rospy.signal_shutdown('status window closed')
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    rospy.init_node('robot_status_display_node')
    StatusWindow().run()
