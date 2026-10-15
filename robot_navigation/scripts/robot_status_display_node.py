#!/usr/bin/env python3
"""ROS callbacks only enqueue state; Tk widgets are updated on the GUI thread."""
import json
import base64
import math
import threading
import time
import tkinter as tk

try:
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage
except ImportError:
    cv2 = None
    CvBridge = None
    RosImage = object

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
        self.video_topic = rospy.get_param('~video_topic', '/camera/upper_display')
        self.scan_mode_topic = rospy.get_param('~scan_mode_topic', '/barcode_scan/camera_mode')
        self.call_camera_status_topic = rospy.get_param(
            '~call_camera_status_topic', '/lower_cam/lower_usb_cam/capture_status')
        self.video_display_fps = max(1.0, min(25.0, float(rospy.get_param('~video_display_fps', 15.0))))
        if CvBridge is None:
            raise RuntimeError('video display requires cv_bridge and OpenCV')
        self.bridge = CvBridge()
        self.video_lock = threading.Lock()
        self.latest_video = None
        self.last_video_time = 0.0
        self.last_video_render = 0.0
        self.video_photo = None
        self.video_has_signal = False
        self.scan_camera_state = '等待扫码摄像头'
        self.call_camera_state = '等待PC1任务启动'
        self.scan_summary = self.scan_parameters()
        self.call_summary = self.call_parameters()
        self.root = tk.Tk()
        self.root.title('送药巡诊机器人状态')
        self.root.configure(bg='#101820')
        if rospy.get_param('~fullscreen', True):
            self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', lambda _: self.root.attributes('-fullscreen', False))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=4, minsize=340)
        self.root.grid_columnconfigure(1, weight=6, minsize=440)
        self.left_panel = tk.Frame(self.root, bg='#101820', padx=16, pady=12)
        self.left_panel.grid(row=0, column=0, sticky='nsew')
        self.left_panel.grid_columnconfigure(0, weight=1)
        self.right_panel = tk.Frame(self.root, bg='#05080b', padx=8, pady=8)
        self.right_panel.grid(row=0, column=1, sticky='nsew')
        self.right_panel.grid_rowconfigure(1, weight=1)
        self.right_panel.grid_columnconfigure(0, weight=1)
        self.labels = {}
        self.make('title', '送药巡诊机器人', 25, '#4dd0e1')
        for key, text in [('task', '当前任务：等待启动'), ('scan', '识别状态：未开始'),
                          ('bed1', '1号床条码：---'), ('bed3', '3号床条码：---'),
                          ('hr', '心率：-- bpm'), ('temp', '体温：-- °C'),
                          ('measure', '测量状态：等待按键'), ('esp', 'ESP32：离线')]:
            self.make(key, text, 17, '#f5f5f5')
        tk.Label(self.right_panel, text='音视频通话', font=('DejaVu Sans', 17, 'bold'),
                 fg='#4dd0e1', bg='#05080b', anchor='w', padx=10, pady=6).grid(
                     row=0, column=0, sticky='ew')
        self.video_label = tk.Label(self.right_panel, text='等待医生端视频画面',
                                    font=('DejaVu Sans', 17), fg='#d8e0e5',
                                    bg='#05080b', anchor='center')
        self.video_label.grid(row=1, column=0, sticky='nsew')
        self.video_info = tk.Label(self.right_panel, text='', justify='right',
                                   font=('DejaVu Sans', 11), fg='#e4eaee',
                                   bg='#18232b', anchor='e', padx=10, pady=7)
        self.video_info.grid(row=2, column=0, sticky='ew')
        self.root.bind('<Configure>', self.resize_text)
        self.right_panel.bind('<Configure>', self.resize_text)
        rospy.Subscriber('/robot_display', String, lambda m: self.set('task', '当前任务：' + m.data.replace('\n', ' | ')))
        rospy.Subscriber('/qr_result', QrResult, self.qr)
        rospy.Subscriber('/barcode_bed1', String, lambda m: self.set('bed1', '1号床条码：' + (m.data or '---')))
        rospy.Subscriber('/barcode_bed3', String, lambda m: self.set('bed3', '3号床条码：' + (m.data or '---')))
        rospy.Subscriber('/vitals/frame', String, self.frame)
        rospy.Subscriber('/vitals/heart_rate', Float32, lambda m: self.vital('hr', m.data, '心率：', 'bpm'))
        rospy.Subscriber('/vitals/temperature', Float32, lambda m: self.vital('temp', m.data, '体温：', '°C'))
        rospy.Subscriber('/vitals/heart_rate_valid', Bool, lambda m: self.invalid('hr', m.data, '心率：-- bpm'))
        rospy.Subscriber('/vitals/temperature_valid', Bool, lambda m: self.invalid('temp', m.data, '体温：-- °C'))
        rospy.Subscriber('/vitals/measuring', Bool, lambda m: self.legacy_set('_measuring', m.data))
        rospy.Subscriber('/vitals/status', String, lambda m: self.legacy_set('_status', m.data))
        rospy.Subscriber(self.video_topic, RosImage, self.video, queue_size=1,
                         buff_size=8 * 1024 * 1024)
        rospy.Subscriber(self.scan_mode_topic, String, self.scan_camera_status, queue_size=1)
        rospy.Subscriber(self.call_camera_status_topic, String, self.call_camera_status, queue_size=1)
        self.root.after(33, self.pump)

    def make(self, key, text, size, color):
        label = tk.Label(self.left_panel, text=text,
                         font=('DejaVu Sans', size, 'bold' if key == 'title' else 'normal'),
                         fg=color, bg='#101820', anchor='w', justify='left',
                         wraplength=600, padx=12, pady=9)
        label.pack(fill='x')
        self.labels[key] = label

    def resize_text(self, event):
        if event.widget == self.root:
            width = max(260, self.left_panel.winfo_width() - 36)
            for label in self.labels.values():
                label.configure(wraplength=width)
        if event.widget == self.right_panel:
            self.video_info.configure(wraplength=max(320, self.right_panel.winfo_width() - 28))

    @staticmethod
    def param(name, default):
        return rospy.get_param(name, default)

    def scan_parameters(self):
        root = '/adaptive_scan_camera_node/'
        mode = self.param(root + 'mode', 'auto')
        device = self.param(root + 'video_device', '/dev/video0')
        low = '%sx%s@%sfps %s' % (
            self.param(root + 'low_width', 640), self.param(root + 'low_height', 480),
            self.param(root + 'low_fps', 25), self.param(root + 'low_pixel_format', 'mjpeg'))
        high = '%sx%s@%sfps %s' % (
            self.param(root + 'high_width', 1280), self.param(root + 'high_height', 720),
            self.param(root + 'high_fps', 5), self.param(root + 'high_pixel_format', 'mjpeg'))
        timeout = self.param(root + 'switch_after_s', 5.0)
        profile = ('低清 %s；%.0fs无识别切高清 %s' % (low, timeout, high)
                   if mode == 'auto' else '%s模式 %s' % (mode, high if mode == 'high' else low))
        return '扫码 %s | %s' % (device, profile)

    def call_parameters(self):
        camera = '/lower_cam/lower_usb_cam/'
        stream = '/video_sender_slave/'
        audio = '/audio_chat_slave/'
        device = self.param(camera + 'video_device', '/dev/video2')
        capture = '%sx%s@%sfps %s' % (
            self.param(camera + 'low_width', 640), self.param(camera + 'low_height', 480),
            self.param(camera + 'low_fps', 25), self.param(camera + 'low_pixel_format', 'mjpeg'))
        output = '%sx%s@%sfps' % (
            self.param(stream + 'stream_width', 640), self.param(stream + 'stream_height', 480),
            self.param(stream + 'framerate', 25))
        rate = self.param(audio + 'sample_rate', 48000)
        mic = self.param(audio + 'mic_device', 'default')
        return '音视频 %s | 采集 %s → 推流 %s\n音频 %sHz mono | 麦克风 %s' % (
            device, capture, output, rate, mic)

    def video(self, msg):
        with self.video_lock:
            self.last_video_time = time.monotonic()
            self.video_has_signal = msg.header.frame_id != 'no_signal'
            self.latest_video = msg if self.video_has_signal else None

    def scan_camera_status(self, msg):
        self.scan_camera_state = msg.data

    def call_camera_status(self, msg):
        self.call_camera_state = msg.data

    def render_video(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            panel_w = max(1, self.video_label.winfo_width())
            panel_h = max(1, self.video_label.winfo_height())
            scale = min(1.25, panel_w / frame.shape[1], panel_h / frame.shape[0])
            width = max(1, int(frame.shape[1] * scale))
            height = max(1, int(frame.shape[0] * scale))
            if (width, height) != (frame.shape[1], frame.shape[0]):
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ppm = ('P6\n%d %d\n255\n' % (width, height)).encode('ascii') + rgb.tobytes()
            self.video_photo = tk.PhotoImage(data=base64.b64encode(ppm), format='PPM')
            self.video_label.configure(image=self.video_photo, text='')
        except (cv2.error, tk.TclError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, '[status_display] video render failed: %s', exc)

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
        with self.video_lock:
            video, self.latest_video = self.latest_video, None
            video_age = time.monotonic() - self.last_video_time
            video_has_signal = self.video_has_signal
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
        now = time.monotonic()
        if video is not None and now - self.last_video_render >= 1.0 / self.video_display_fps:
            self.render_video(video)
            self.last_video_render = now
        if video_age > 1.5 or not video_has_signal:
            self.video_label.configure(text='等待音视频通话画面', image='')
            self.video_photo = None
        self.video_info.configure(text='%s\n%s\n扫码状态：%s | 通话摄像头：%s | 画面：%s' % (
            self.scan_summary, self.call_summary, self.scan_camera_state,
            self.call_camera_state,
            '接收中' if video_age <= 1.5 and video_has_signal else '等待画面'))
        self.root.after(33, self.pump)

    def close(self):
        rospy.signal_shutdown('status window closed')
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    rospy.init_node('robot_status_display_node')
    StatusWindow().run()
