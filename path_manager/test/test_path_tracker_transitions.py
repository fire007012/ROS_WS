#!/usr/bin/env python3
"""Exercise the real tracker with delayed path messages; no CAN or hardware."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_manager.msg import PathPoint
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


class TrackerTransitions(unittest.TestCase):
    def test_delayed_point_arrival_and_heading_changes(self):
        self.guard = threading.Lock()
        self.calls = 0
        self.velocity = Twist()
        self.finished = False
        self.pose = [0.0, 0.0, 0.0]
        self.point = None
        rospy.Subscriber('/cmd_vel_external', Twist, self.on_velocity)
        rospy.Subscriber('/path_finished', Bool, self.on_finished)
        service = rospy.Service('/next_point', Trigger, self.next_point)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path = rospy.Publisher('/path_points', PathPoint, queue_size=1)
        timer = rospy.Timer(rospy.Duration(.02), self.publish)
        try:
            self.wait(lambda: self.odom.get_num_connections() and self.path.get_num_connections())
            self.set_point(0.0, True)
            self.wait(lambda: self.calls == 1)
            # Continue publishing the OLD point after /next_point returns.
            time.sleep(.5)
            self.assertEqual(self.calls, 1, 'delayed completed point requested another advance')
            self.set_point(.15, True)
            self.wait(lambda: self.velocity.linear.x > .01)
            time.sleep(.4)
            self.assertEqual(self.calls, 1, 'new target inherited old arrival state')

            # Change target while the old target is in its arrival hold.
            self.set_pose(.15)
            self.wait(lambda: abs(self.velocity.linear.x) < 1e-8)
            self.set_point(.3, False)
            self.wait(lambda: self.velocity.linear.x > .01)
            time.sleep(.4)
            self.assertFalse(self.finished)
            self.assertEqual(self.calls, 1)

            # Drifting out of tolerance during the hold must resume movement.
            self.set_pose(.3)
            self.wait(lambda: abs(self.velocity.linear.x) < 1e-8)
            self.set_pose(.2)
            self.wait(lambda: self.velocity.linear.x > .01)
            time.sleep(.35)
            self.assertFalse(self.finished, 'arrival hold ignored position drift')
            self.set_pose(.3)
            self.wait(lambda: self.finished)

            # Same position with a different heading is a new target.
            self.set_point(.3, False, yaw=1.0)
            self.wait(lambda: not self.finished and self.velocity.angular.z > .1)
            self.set_pose(.3, yaw=1.0)
            self.wait(lambda: self.finished)
        finally:
            timer.shutdown()
            service.shutdown()

    def wait(self, predicate):
        deadline = time.monotonic()+3
        while time.monotonic()<deadline and not rospy.is_shutdown():
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('tracker condition timed out')

    def on_velocity(self, msg):
        with self.guard:
            self.velocity = msg

    def on_finished(self, msg):
        with self.guard:
            self.finished = msg.data

    def next_point(self, req):
        with self.guard:
            self.calls += 1
        return TriggerResponse(success=True, message='advanced; next publication delayed')

    def set_point(self, x, has_next, yaw=None):
        with self.guard:
            self.point = PathPoint(x=x, y=0.0, tolerance=.02, frame_id='odom',
                                   has_next=has_next, has_yaw=yaw is not None,
                                   yaw=0.0 if yaw is None else yaw)

    def set_pose(self, x, yaw=0.0):
        with self.guard:
            self.pose = [x, 0.0, yaw]

    def publish(self, event):
        with self.guard:
            x, y, yaw = self.pose
            point = self.point
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw/2)
        msg.pose.pose.orientation.w = math.cos(yaw/2)
        self.odom.publish(msg)
        if point is not None:
            self.path.publish(point)


if __name__ == '__main__':
    rospy.init_node('test_path_tracker_transitions')
    rostest.rosrun('path_manager', 'path_tracker_transitions', TrackerTransitions)
