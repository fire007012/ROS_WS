#!/usr/bin/env python3
"""Route one QR scan and two distinct Code 128 scans in bed order."""

import threading
import math

import rospy
from robot_navigation.msg import QrResult
from std_msgs.msg import Int8, String, UInt32


class ScanSequence:
    def __init__(self):
        self.lock = threading.Lock()
        self.first_bed = None
        self.expected_bed = None
        self.first_code = None
        self.session_id = 0
        self.generation = 0
        self.retry_pending = False
        self.retry_sec = float(rospy.get_param('~duplicate_retry_sec', 0.5))
        self.allow_qr_updates = bool(rospy.get_param('~allow_qr_updates', False))
        self.require_distinct = bool(rospy.get_param('~require_distinct_barcodes', True))
        self.qr_key = None
        if not math.isfinite(self.retry_sec) or self.retry_sec <= 0:
            raise ValueError('duplicate_retry_sec must be positive and finite')

        self.target_pub = rospy.Publisher('/barcode_scan/target_bed', Int8, queue_size=1, latch=True)
        self.bed_pubs = {
            1: rospy.Publisher('/barcode_bed1', String, queue_size=1, latch=True),
            3: rospy.Publisher('/barcode_bed3', String, queue_size=1, latch=True),
        }
        self.display_pub = rospy.Publisher('/robot_display', String, queue_size=1, latch=True)
        self.qr_sub = rospy.Subscriber('/qr_result', QrResult, self.on_qr, queue_size=1)
        self.bed_subs = [
            rospy.Subscriber('/barcode_scan/raw_bed1', String, self.on_barcode, callback_args=1, queue_size=1),
            rospy.Subscriber('/barcode_scan/raw_bed3', String, self.on_barcode, callback_args=3, queue_size=1),
        ]
        test_bed = int(rospy.get_param('~barcode_test_bed', 0))
        if test_bed in (1, 3):
            self.first_bed = self.expected_bed = test_bed
            self.target_pub.publish(Int8(data=test_bed))
        self.session_sub = rospy.Subscriber('/barcode_scan/session', UInt32, self.on_session, queue_size=1)

    def on_qr(self, msg):
        if (msg.first_bed, msg.second_bed) not in ((1, 3), (3, 1)):
            return  # An empty QR timeout result must not start a scan.
        key = (msg.first_bed, msg.first_box, msg.second_bed, msg.second_box)
        if msg.first_box not in (1, 3) or msg.second_box not in (1, 3) or msg.first_box == msg.second_box:
            return
        with self.lock:
            if self.qr_key == key or (self.first_bed is not None and not self.allow_qr_updates):
                return
            self.qr_key = key
            self.generation += 1
            self.first_code = None
            self.retry_pending = False
            self.bed_pubs[1].publish(String(data=''))
            self.bed_pubs[3].publish(String(data=''))
            self.first_bed = msg.first_bed
            self.expected_bed = msg.first_bed
            self.target_pub.publish(Int8(data=self.expected_bed))
            rospy.loginfo('[scan_sequence] QR selects bed %d first, bed %d second',
                          msg.first_bed, msg.second_bed)

    def on_barcode(self, msg, bed):
        with self.lock:
            if not msg.data or len(msg.data) > 256:
                return
            if bed != self.expected_bed:
                return
            if self.first_code is None:
                self.first_code = msg.data
                self.bed_pubs[bed].publish(msg)
                self.display_pub.publish(String(data='%d床药品:%s' % (bed, msg.data)))
                self.expected_bed = 3 if bed == 1 else 1
                self.target_pub.publish(Int8(data=self.expected_bed))
                rospy.loginfo('[scan_sequence] bed %d scanned; now scan bed %d',
                              bed, self.expected_bed)
            elif self.require_distinct and msg.data == self.first_code:
                # The first barcode may remain in view after switching beds.
                # The detector suppresses further output until its target is set again.
                if not self.retry_pending:
                    self.retry_pending = True
                    generation = self.generation
                    rospy.Timer(rospy.Duration(self.retry_sec),
                                lambda event: self.retry_duplicate(generation), oneshot=True)
            else:
                self.bed_pubs[bed].publish(msg)
                self.display_pub.publish(String(data='%d床药品:%s' % (bed, msg.data)))
                self.expected_bed = None
                self.target_pub.publish(Int8(data=0))
                rospy.loginfo('[scan_sequence] bed %d scanned; both beds complete', bed)

    def retry_duplicate(self, generation):
        with self.lock:
            if generation != self.generation or self.expected_bed is None:
                return
            self.retry_pending = False
            self.target_pub.publish(Int8(data=self.expected_bed))

    def on_session(self, msg):
        with self.lock:
            if msg.data == self.session_id:
                return
            self.qr_key = None
            self.session_id = msg.data
            self.generation += 1
            self.first_bed = None
            self.expected_bed = None
            self.first_code = None
            self.retry_pending = False
            self.target_pub.publish(Int8(data=0))
            # Clear latched outputs so a new scan session cannot inherit the
            # previous bed codes or display text in a late-joining GUI.
            self.bed_pubs[1].publish(String(data=''))
            self.bed_pubs[3].publish(String(data=''))
            self.display_pub.publish(String(data='识别状态：等待扫码'))


if __name__ == '__main__':
    rospy.init_node('scan_sequence_node')
    ScanSequence()
    rospy.spin()


