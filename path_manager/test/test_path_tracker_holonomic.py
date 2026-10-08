#!/usr/bin/env python3
"""Verify real tracker translation, frame conversion, speed limits and final yaw."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_manager.msg import PathPoint
from std_msgs.msg import Bool, Empty, UInt32


class HolonomicTracker(unittest.TestCase):
    def test_motion_and_heading(self):
        self.guard = threading.Lock()
        self.velocity = Twist()
        self.finished = False
        self.yaw = 0.0
        self.send_odom = True
        self.point = PathPoint(x=0.0, y=1.0, yaw=0.0, has_yaw=True,
                               tolerance=.02, frame_id='odom', has_next=False)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path = rospy.Publisher('/path_points', PathPoint, queue_size=1)
        estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        cancel = rospy.Publisher('/mission/cancel', Empty, queue_size=1)
        revision = rospy.Publisher('/path_manager/path_revision', UInt32, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_external', Twist, self.on_velocity),
                rospy.Subscriber('/path_finished', Bool, self.on_finished)]
        timer = rospy.Timer(rospy.Duration(.02), self.publish)
        try:
            self.wait(lambda: self.velocity.linear.y > .2)
            self.assertAlmostEqual(self.velocity.linear.x, 0.0, places=6)
            self.assertAlmostEqual(self.velocity.angular.z, 0.0, places=6)
            with self.guard:
                self.point.x, self.point.y = -1.0, 0.0
            self.wait(lambda: self.velocity.linear.x < -.2)
            self.assertAlmostEqual(self.velocity.angular.z, 0.0, places=6)
            with self.guard:
                self.point.x, self.point.y = 1.0, 1.0
            self.wait(lambda: self.velocity.linear.x > .1 and self.velocity.linear.y > .1)
            self.assertAlmostEqual(math.hypot(self.velocity.linear.x, self.velocity.linear.y),
                                   .25, places=6)
            with self.guard:
                self.yaw = self.point.yaw = math.pi/2
                self.point.x, self.point.y = 1.0, 0.0
            self.wait(lambda: self.velocity.linear.y < -.2)
            self.assertAlmostEqual(self.velocity.linear.x, 0.0, places=6)
            self.assertAlmostEqual(self.velocity.angular.z, 0.0, places=6)
            # Reaching position still requires final yaw alignment.
            with self.guard:
                self.point.x, self.point.y, self.point.yaw = 0.0, 0.0, 0.0
            self.wait(lambda: self.velocity.angular.z < -.1)
            self.assertAlmostEqual(math.hypot(self.velocity.linear.x, self.velocity.linear.y), 0.0)
            self.assertFalse(self.finished)
            with self.guard:
                self.yaw = 0.0
            self.wait(lambda: self.finished)
            with self.guard:
                self.point.x = 1.0
            self.wait(lambda: self.velocity.linear.x > .2)
            estop.publish(Bool(True))
            self.wait(lambda: self.velocity.linear.x == 0)
            reset.publish(Bool(True))
            self.hold_stopped(.6)
            revision.publish(UInt32(1))
            self.wait(lambda: self.velocity.linear.x > .2)
            cancel.publish(Empty())
            self.wait(lambda: self.velocity.linear.x == 0)
            self.hold_stopped(.3)
            revision.publish(UInt32(2))
            self.wait(lambda: self.velocity.linear.x > .2)
            self.send_odom = False
            self.wait(lambda: self.velocity.linear.x == 0)
            self.hold_stopped(.3)
        finally:
            timer.shutdown()
            for sub in subs:
                sub.unregister()

    def on_velocity(self, msg):
        with self.guard:
            self.velocity = msg

    def on_finished(self, msg):
        with self.guard:
            self.finished = msg.data

    def publish(self, event):
        with self.guard:
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.orientation.z = math.sin(self.yaw/2)
            msg.pose.pose.orientation.w = math.cos(self.yaw/2)
            if self.send_odom:
                self.odom.publish(msg)
            self.path.publish(self.point)

    def wait(self, predicate):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('tracker condition timed out')

    def hold_stopped(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            with self.guard:
                self.assertEqual(self.velocity.linear.x, 0.0)
                self.assertEqual(self.velocity.linear.y, 0.0)
                self.assertEqual(self.velocity.angular.z, 0.0)
            time.sleep(.01)


if __name__ == '__main__':
    rospy.init_node('test_path_tracker_holonomic')
    rostest.rosrun('path_manager', 'path_tracker_holonomic', HolonomicTracker)
