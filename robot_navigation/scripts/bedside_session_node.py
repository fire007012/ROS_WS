#!/usr/bin/env python3
"""Wait for actual AV receive and fresh, valid ESP32 data at each bed."""
import json
import math
import threading
import time
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String, Empty
from std_srvs.srv import Trigger, TriggerResponse

class BedsideSession:
    def __init__(self):
        self.audio = bool(rospy.get_param('~audio_video_enabled', True))
        self.measure = bool(rospy.get_param('~patient_measurement_enabled', True))
        role = rospy.get_param('~audio_role', 'master')
        if role not in ('master', 'slave'):
            raise ValueError('audio_role must be master or slave')
        self.peer_topic = rospy.get_param('~peer_ready_topic', '/audio_chat/' + role + '/peer_ready')
        self.video_topic = rospy.get_param('~video_topic',
            '/upper_cam/network/compressed' if role == 'master' else '/lower_cam/network/compressed')
        self.vitals_topic = rospy.get_param('~vitals_frame_topic', '/vitals/frame')
        self.call_duration = float(rospy.get_param('~call_duration_s', 5.0))
        self.timeout = float(rospy.get_param('~timeout_s', 30.0))
        self.freshness = float(rospy.get_param('~freshness_s', 2.0))
        self.measure_duration = float(rospy.get_param('~measurement_valid_hold_s', 2.0))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.call_duration, self.timeout, self.freshness, self.measure_duration)):
            raise ValueError('bedside durations must be positive and finite')
        self.lock = threading.Lock()
        self.enabled = True
        self.estop = False
        self.generation = 0
        self.peer_ready = False
        self.peer_time = self.video_time = self.vitals_time = 0.0
        self.vitals_valid = False
        self.last_seq = None
        self.status = rospy.Publisher('/bedside/status', String, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber(self.peer_topic, Bool, self.on_peer),
                     rospy.Subscriber(self.video_topic, CompressedImage, self.on_video),
                     rospy.Subscriber(self.vitals_topic, String, self.on_vitals),
                     rospy.Subscriber('/mission/cancel', Empty, self.cancel),
                     rospy.Subscriber('/emergency_stop', Bool, self.on_estop),
                     rospy.Subscriber('/emergency_stop/reset', Bool, self.on_reset),
                     rospy.Subscriber('/mission/actions_enabled', Bool, self.on_enabled)]
        self.server = rospy.Service('/bedside/complete', Trigger, self.complete)

    def on_peer(self, msg):
        self.peer_ready = msg.data
        self.peer_time = time.monotonic()

    def on_video(self, msg):
        if msg.data and 'jpeg' in msg.format.lower() and msg.header.frame_id != 'no_signal':
            self.video_time = time.monotonic()

    def on_vitals(self, msg):
        try:
            obj = json.loads(msg.data)
            if not isinstance(obj, dict):
                raise ValueError('expected vitals object')
            seq = obj.get('seq')
            if seq == self.last_seq or type(seq) is not int:
                self.vitals_valid = False
                return
            self.last_seq = seq
            valid = (obj.get('status') == 'online' and obj.get('measuring') is True
                     and obj.get('heart_rate_valid') is True and obj.get('temperature_valid') is True)
            hr, temp = obj.get('heart_rate'), obj.get('temperature')
            self.vitals_valid = (valid and type(hr) in (int, float) and type(temp) in (int, float)
                                 and math.isfinite(hr) and math.isfinite(temp)
                                 and 1 <= hr <= 300 and -70 <= temp <= 380)
            self.vitals_time = time.monotonic()
        except (ValueError, TypeError, OverflowError):
            self.vitals_valid = False

    def cancel(self, _):
        self.generation += 1
        self.enabled = False

    def on_estop(self, msg):
        if msg.data:
            self.estop = True
            self.cancel(msg)

    def on_reset(self, msg):
        if msg.data:
            self.estop = False

    def on_enabled(self, msg):
        if msg.data:
            self.enabled = True
        else:
            self.cancel(msg)

    def complete(self, _):
        if not self.lock.acquire(False):
            return TriggerResponse(False, 'bedside session busy')
        try:
            if not self.enabled or self.estop:
                return TriggerResponse(False, 'bedside session disabled')
            generation = self.generation
            started = time.monotonic()
            call_since = measure_since = None
            call_done = not self.audio
            measure_done = not self.measure
            self.status.publish('waiting for doctor video/audio and fresh bedside measurement' if self.measure
                                else 'waiting for doctor video/audio')
            while not rospy.is_shutdown():
                now = time.monotonic()
                if generation != self.generation or not self.enabled or self.estop:
                    return TriggerResponse(False, 'bedside session cancelled')
                if now-started >= self.timeout:
                    return TriggerResponse(False, 'bedside timeout: check doctor Ready, UDP, video and ESP32 valid readings')
                av_ok = (self.peer_ready and self.peer_time >= started
                         and now-self.peer_time < self.freshness and self.video_time >= started
                         and now-self.video_time < self.freshness)
                if not call_done:
                    call_since = (call_since if call_since is not None else now) if av_ok else None
                    call_done = call_since is not None and now-call_since >= self.call_duration
                valid = self.vitals_valid and self.vitals_time >= started and now-self.vitals_time < self.freshness
                if not measure_done:
                    measure_since = (measure_since if measure_since is not None else now) if valid else None
                    measure_done = measure_since is not None and now-measure_since >= self.measure_duration
                if call_done and measure_done:
                    self.status.publish('bedside call and measurement completed' if self.measure else 'bedside call completed')
                    return TriggerResponse(True, 'fresh receive and valid measurement gates completed')
                time.sleep(0.05)
            return TriggerResponse(False, 'ROS shutdown')
        finally:
            self.lock.release()

if __name__ == '__main__':
    rospy.init_node('bedside_session_node')
    BedsideSession()
    rospy.spin()
