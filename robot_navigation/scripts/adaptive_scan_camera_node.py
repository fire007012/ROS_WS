#!/usr/bin/env python3
"""One owner of the scan camera; callbacks never open/release a capture handle."""
import math
import threading
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8, Int32, String
from scan_camera_policy import ScanCameraPolicy

class AdaptiveScanCamera:
    def __init__(self):
        self.device = rospy.get_param('~video_device', '/dev/video0')
        self.topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.frame_id = rospy.get_param('~frame_id', 'scan_camera')
        self.low = self.profile('low', 640, 480, 25)
        self.high = self.profile('high', 1280, 720, 5)
        self.policy = ScanCameraPolicy(rospy.get_param('~mode', 'auto'),
                                      float(rospy.get_param('~switch_after_s', 5.0)),
                                      bool(rospy.get_param('~continuous', False)))
        self.lock = threading.Lock()
        self.capture = None
        self.current = None
        self.failed_modes = set()
        self.wait_for_ready = bool(rospy.get_param('~wait_for_ready', False))
        self.ready_topic = rospy.get_param('~ready_topic', '/Ready')
        self.ready = not self.wait_for_ready
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.topic, Image, queue_size=1)
        self.status = rospy.Publisher(rospy.get_param('~status_topic','/barcode_scan/camera_mode'), String, queue_size=1, latch=True)
        self.ready_subscriber = (rospy.Subscriber(self.ready_topic, Int32, self.ready_callback, queue_size=1)
                                 if self.wait_for_ready else None)
        self.subscribers = [rospy.Subscriber('/barcode_scan/target_bed', Int8, self.target, queue_size=1),
                            rospy.Subscriber('/barcode_scan/qr_enabled', Bool, self.qr_enable, queue_size=1),
                            rospy.Subscriber('/barcode_scan/completed', Int8, self.completed, queue_size=1)]

    @staticmethod
    def profile(prefix, width, height, fps):
        result = dict(width=int(rospy.get_param('~'+prefix+'_width', width)),
                      height=int(rospy.get_param('~'+prefix+'_height', height)),
                      fps=float(rospy.get_param('~'+prefix+'_fps', fps)),
                      pixel_format=str(rospy.get_param('~'+prefix+'_pixel_format', 'mjpeg')))
        if result['width'] <= 0 or result['height'] <= 0 or not math.isfinite(result['fps']) or result['fps'] <= 0:
            raise ValueError('invalid camera dimensions/fps')
        if result['pixel_format'] not in ('mjpeg', 'yuyv'):
            raise ValueError('camera pixel format must be mjpeg or yuyv')
        return result

    def ready_callback(self, msg):
        with self.lock:
            self.ready = msg.data == 1

    def target(self, msg):
        with self.lock:
            self.policy.target(msg.data, time.monotonic())
            if msg.data in (1, 3):
                self.failed_modes.clear()

    def qr_enable(self, msg):
        with self.lock:
            self.policy.enable_qr(msg.data, time.monotonic())
            if msg.data:
                self.failed_modes.clear()

    def completed(self, msg):
        with self.lock:
            if msg.data == 0:
                self.policy.decoded('qr',time.monotonic())
            elif msg.data == self.policy.bed and msg.data in (1,3):
                self.policy.decoded('barcode',time.monotonic())

    def close(self):
        # Called only on capture thread, including its finally block.
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.current = None

    def open(self, mode):
        self.close()
        config = self.low if mode == 'low' else self.high
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            self.status.publish('error: unable to open '+self.device)
            rospy.logerr_throttle(5.0, '[scan_camera] cannot open %s; check permissions/device ownership', self.device)
            return False
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*('MJPG' if config['pixel_format']=='mjpeg' else 'YUYV')))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config['width'])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config['height'])
        capture.set(cv2.CAP_PROP_FPS, config['fps'])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv2.CAP_PROP_FPS)
        fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
        actual_format = ''.join(chr((fourcc >> (8*i)) & 255) for i in range(4))
        if (width,height) != (config['width'],config['height']):
            rospy.logwarn('[scan_camera] %s requested %dx%d but driver returned %dx%d; profile rejected',
                          mode,config['width'],config['height'],width,height)
            capture.release()
            return False
        self.capture = capture
        self.current = mode
        self.status.publish('%s:%dx%d@%.1f:%s' % (mode,width,height,fps,actual_format))
        rospy.loginfo('[scan_camera] %s %dx%d requested %.1ffps; driver %.1ffps %s',
                      mode,width,height,config['fps'],fps,actual_format)
        return True

    def run(self):
        next_publish = 0.0
        read_failures = 0
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    desired = self.policy.requested(time.monotonic()) if self.ready else None
                    if desired == 'high' and 'high' in self.failed_modes:
                        desired = 'low' # Automatic mode recovers without retrying unsupported HD forever.
                if desired is None:
                    if self.capture is not None:
                        self.close()
                        if self.wait_for_ready and not self.ready:
                            self.status.publish('waiting for /Ready: camera device closed')
                        else:
                            self.status.publish('idle: camera released between scan stages')
                    time.sleep(0.05)
                    continue
                if desired != self.current:
                    if not self.open(desired):
                        with self.lock:
                            if desired == 'high' and self.policy.mode == 'auto':
                                self.failed_modes.add('high')
                        time.sleep(0.5)
                        continue
                    next_publish = 0.0
                    read_failures = 0
                try:
                    ok, frame = self.capture.read()
                except cv2.error as exc:
                    rospy.logwarn_throttle(5.0,'[scan_camera] decode failed: %s',exc)
                    ok,frame=False,None
                if not ok or frame is None:
                    read_failures += 1
                    rospy.logwarn_throttle(5.0, '[scan_camera] invalid frame; dropping it, never publishing stale pixels')
                    if read_failures >= 3:
                        with self.lock:
                            if self.current == 'high' and self.policy.mode == 'auto':
                                self.failed_modes.add('high')
                        self.close()
                        time.sleep(0.5)
                    continue
                read_failures = 0
                now = time.monotonic()
                if now < next_publish:
                    continue
                next_publish = now + 0.9/(self.low if self.current=='low' else self.high)['fps']
                message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = self.frame_id
                self.publisher.publish(message)
        finally:
            self.close()

if __name__ == '__main__':
    rospy.init_node('adaptive_scan_camera_node')
    AdaptiveScanCamera().run()
