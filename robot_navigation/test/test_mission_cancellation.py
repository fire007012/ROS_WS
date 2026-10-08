#!/usr/bin/env python3
"""Slow services must not prevent cancellation, stage deadlines or global stop."""
import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse
from path_manager.srv import SelectPath, SelectPathResponse
from robot_navigation.msg import QrResult
from robot_navigation.srv import OpenMedicineBox, OpenMedicineBoxResponse, Speak, SpeakResponse
from arm_and_gripper.srv import ArmPlaceMedicine, ArmPlaceMedicineResponse


class MissionCancellation(unittest.TestCase):
    def test_cancel_and_deadlines(self):
        self.guard = threading.RLock()
        self.selected = []
        self.pose = (0., 0.)
        self.pending_path = self.pending_fine = None
        self.slow = self.entered = ''
        self.cancelled = 0
        self.locked = False
        self.displays = []
        self.global_case = False
        self.start = rospy.Publisher('/start_signal/physical', UInt32, queue_size=1)
        self.estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        self.qr = rospy.Publisher('/qr_result', QrResult, queue_size=1)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path_done = rospy.Publisher('/path_finished', Bool, queue_size=1)
        self.fine_done = rospy.Publisher('/fine_tuning_done', Bool, queue_size=1)
        subs = [rospy.Subscriber('/mission/cancel', Empty, self.on_cancel),
                rospy.Subscriber('/chassis_lock', Bool, self.on_lock),
                rospy.Subscriber('/robot_display', String, self.on_display)]
        services = [rospy.Service('/select_path', SelectPath, self.select),
                    rospy.Service('/fine_tuning/start', Trigger, self.fine),
                    rospy.Service('/open_medicine_box', OpenMedicineBox,
                                  lambda req: OpenMedicineBoxResponse(self.service('box'), 'mock')),
                    rospy.Service('/arm_place_medicine', ArmPlaceMedicine,
                                  lambda req: ArmPlaceMedicineResponse(self.service('arm'), 'mock')),
                    rospy.Service('/speak', Speak,
                                  lambda req: SpeakResponse(self.service('speech'), 'mock'))]
        timer = rospy.Timer(rospy.Duration(.02), self.publish)
        try:
            self.wait(lambda: self.start.get_num_connections() and self.estop.get_num_connections())
            for slow in ['box', 'arm', 'speech']:
                self.new_case(slow)
                self.start.publish(UInt32(0x5A17))
                self.wait(lambda: self.entered == slow)
                before = self.cancelled
                sent = time.monotonic()
                self.estop.publish(Bool(True))
                self.wait(lambda: self.cancelled > before and self.locked, timeout=.4)
                self.assertLess(time.monotonic()-sent, .4)
                paths = list(self.selected)
                self.reset.publish(Bool(True))
                time.sleep(.15)
                self.assertTrue(self.locked, 'reset must wait for an explicit new mission')
                # A still-running old service prevents restart and cannot advance the mission.
                self.start.publish(UInt32(0x5A17))
                time.sleep(1.5)
                self.assertEqual(self.selected, paths)
                self.assertEqual(paths, ['nurse_station', 'bed1_circle'])

            self.new_case('box')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.entered == 'box')
            before = self.cancelled
            self.wait(lambda: self.cancelled > before, timeout=1)
            self.assertTrue(self.locked)
            self.assertTrue(any('阶段超时' in d for d in self.displays))
            time.sleep(1)
            self.assertEqual(self.selected, ['nurse_station', 'bed1_circle'])

            self.estop.publish(Bool(True))
            time.sleep(.1)
            self.reset.publish(Bool(True))
            time.sleep(.1)
            self.new_case('')
            self.global_case = True
            before = self.cancelled
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.cancelled > before, timeout=3.5)
            self.assertTrue(self.locked)
            self.assertEqual(self.selected, ['nurse_station'], 'global timeout must not select a return path')
        finally:
            timer.shutdown()
            for service in services:
                service.shutdown()
            for sub in subs:
                sub.unregister()

    def new_case(self, slow):
        with self.guard:
            self.slow, self.entered = slow, ''
            self.selected, self.displays = [], []
            self.pending_path = self.pending_fine = None

    def service(self, name):
        with self.guard:
            slow = self.slow == name
            if slow:
                self.entered = name
        if slow:
            time.sleep(1.5)
        return True

    def select(self, req):
        with self.guard:
            self.selected.append(req.path_name)
            self.pose = (1.3, 0.) if req.path_name == 'nurse_station' else (4., 2.2)
            if not self.global_case:
                self.pending_path = time.monotonic()+.1
        return SelectPathResponse(True, 'mock')

    def fine(self, req):
        with self.guard:
            self.pending_fine = time.monotonic()+.1
        return TriggerResponse(True, 'mock')

    def on_cancel(self, msg):
        with self.guard:
            self.cancelled += 1

    def on_lock(self, msg):
        with self.guard:
            self.locked = msg.data

    def on_display(self, msg):
        with self.guard:
            self.displays.append(msg.data)

    def publish(self, event):
        with self.guard:
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x, msg.pose.pose.position.y = self.pose
            msg.pose.pose.orientation.w = 1.0
            self.odom.publish(msg)
            self.qr.publish(QrResult(1, 1, 3, 3))
            now = time.monotonic()
            if self.pending_path and now >= self.pending_path:
                self.path_done.publish(Bool(True))
                self.pending_path = None
            if self.pending_fine and now >= self.pending_fine:
                self.fine_done.publish(Bool(True))
                self.pending_fine = None

    def wait(self, predicate, timeout=4):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timed out: '+str(self.selected)+' '+str(self.displays))


if __name__ == '__main__':
    rospy.init_node('test_mission_cancellation')
    rostest.rosrun('robot_navigation', 'mission_cancellation', MissionCancellation)
