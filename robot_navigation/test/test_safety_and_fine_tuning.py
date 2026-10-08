#!/usr/bin/env python3
"""Exercise real mux, directional braking, and fine tuning without hardware."""
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger


class SafetyAndFineTuning(unittest.TestCase):
    def test_controls(self):
        self.guard = threading.RLock()
        self.command = Twist()
        self.output = Twist()
        self.state = ''
        self.ranges = dict(front=.8, left=.8, right=.8)
        self.distance = 400.0
        self.pose_x = 0.0
        self.send_command = self.send_data = True
        self.done, self.failed, self.status, self.velocities = {}, {}, {}, {}
        self.cmd_pub = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=1)
        self.range_pubs = {d: rospy.Publisher('/'+d+'/range', Range, queue_size=1)
                           for d in ['front', 'left', 'right', 'rear']}
        self.distance_pub = rospy.Publisher('/vl53l1x_distance', Float32, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        cancel = rospy.Publisher('/mission/cancel', Empty, queue_size=1)
        reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        enabled = rospy.Publisher('/mission/actions_enabled', Bool, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_muxed', Twist, self.on_output),
                rospy.Subscriber('/distance_safety/state', String, self.on_state)]
        starts = {}
        for name in ['normal_fine', 'signed_fine', 'timeout_fine', 'conflict_fine']:
            self.velocities[name] = []
            for field, msg_type in [('done', Bool), ('failed', Bool), ('status', String)]:
                subs.append(rospy.Subscriber('/'+name+'/'+field, msg_type,
                            lambda msg, n=name, f=field: self.record(n, f, msg.data)))
            subs.append(rospy.Subscriber('/'+name+'/velocity', Twist,
                        lambda msg, n=name: self.record_velocity(n, msg)))
            rospy.wait_for_service('/'+name+'/start', timeout=8)
            starts[name] = rospy.ServiceProxy('/'+name+'/start', Trigger)
        timer = rospy.Timer(rospy.Duration(.02), self.publish)
        try:
            self.command.linear.x = .2
            self.wait(lambda: self.state == 'CLEAR' and self.output.linear.x > .19)
            self.ranges['front'] = .2
            self.wait(lambda: self.state == 'STOP' and self.output.linear.x == 0)
            self.hold(lambda: self.output.linear.x == 0, .8)
            self.ranges['front'] = .45
            self.wait(lambda: self.state == 'WARNING' and abs(self.output.linear.x-.06) < .001)
            self.hold(lambda: abs(self.output.linear.x-.06) < .001, .8)
            self.ranges['front'] = .8
            self.wait(lambda: self.state == 'CLEAR' and self.output.linear.x > .19)
            for direction, sign in [('left', 1), ('right', -1)]:
                self.command.linear.x = 0
                self.command.linear.y = sign*.2
                self.ranges[direction] = .2
                self.wait(lambda: self.state == 'STOP' and self.output.linear.y == 0)
                self.ranges[direction] = .8
                self.wait(lambda: abs(self.output.linear.y-sign*.2) < .001)
                del self.ranges[direction]
                self.wait(lambda: self.state == 'RANGE_TIMEOUT' and self.output.linear.y == 0)
                self.ranges[direction] = .8
            self.command.linear.y = 0
            self.command.linear.x = -.2
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.linear.x == 0)
            self.hold(lambda: self.output.linear.x == 0, .6)
            # Only a real simulated rear sample can permit reverse in this test.
            self.ranges['rear'] = .8
            self.wait(lambda: self.state == 'CLEAR' and self.output.linear.x < -.19)
            self.command.linear.x = float('nan')
            self.wait(lambda: self.output.linear.x == 0)
            self.command.linear.x = .2
            self.wait(lambda: self.output.linear.x > .19)
            self.send_command = False
            self.wait(lambda: self.output.linear.x == 0)

            response = starts['conflict_fine']()
            self.assertFalse(response.success)
            self.assertIn('braking', response.message)
            self.assertTrue(starts['normal_fine']().success)
            self.wait(lambda: self.done.get('normal_fine') is True)

            self.distance = 300.0
            time.sleep(.1)
            self.assertTrue(starts['normal_fine']().success)
            self.wait(lambda: self.failed.get('normal_fine') is True)
            self.assertIn('rear sensor', self.status['normal_fine'])
            self.assertFalse(self.done['normal_fine'])

            self.assertTrue(starts['signed_fine']().success)
            self.wait(lambda: self.failed.get('signed_fine') is True)
            moving = [v for v in self.velocities['signed_fine'] if abs(v) > .001]
            self.assertGreater(len(moving), 3)
            self.assertTrue(all(-.050001 <= v < 0 for v in moving), moving)
            self.assertIn('step limit', self.status['signed_fine'])
            self.assertFalse(self.done['signed_fine'])

            self.distance = 500.0
            time.sleep(.1)
            self.assertTrue(starts['timeout_fine']().success)
            self.wait(lambda: self.failed.get('timeout_fine') is True)
            self.assertEqual(self.status['timeout_fine'], 'timeout')
            self.assertFalse(self.done['timeout_fine'])

            self.assertTrue(starts['normal_fine']().success)
            self.wait(lambda: self.failed.get('normal_fine') is False)
            self.pose_x = .051
            self.wait(lambda: self.failed.get('normal_fine') is True)
            self.assertIn('translation limit', self.status['normal_fine'])
            self.pose_x = 0
            time.sleep(.1)
            self.assertTrue(starts['normal_fine']().success)
            self.wait(lambda: self.failed.get('normal_fine') is False)
            self.send_data = False
            self.wait(lambda: self.failed.get('normal_fine') is True)
            self.assertIn('stale', self.status['normal_fine'])
            self.assertFalse(self.done['normal_fine'])
            self.send_data = True
            self.distance = 400.0
            time.sleep(.1)
            cancel.publish(Empty())
            time.sleep(.1)
            self.assertFalse(starts['normal_fine']().success, 'late service after cancel')
            reset.publish(Bool(True))
            time.sleep(.1)
            self.assertFalse(starts['normal_fine']().success, 'reset alone must not enable actions')
            enabled.publish(Bool(True))
            time.sleep(.1)
            self.assertTrue(starts['normal_fine']().success)
            self.wait(lambda: self.done.get('normal_fine') is True)
        finally:
            timer.shutdown()
            for sub in subs:
                sub.unregister()

    def record(self, name, field, value):
        with self.guard:
            getattr(self, field)[name] = value

    def record_velocity(self, name, msg):
        with self.guard:
            self.velocities[name].append(msg.linear.x)

    def on_output(self, msg):
        with self.guard:
            self.output = msg

    def on_state(self, msg):
        with self.guard:
            self.state = msg.data

    def publish(self, event):
        with self.guard:
            if self.send_command:
                self.cmd_pub.publish(self.command)
            if not self.send_data:
                return
            for direction, distance in list(self.ranges.items()):
                msg = Range()
                msg.header.stamp = rospy.Time.now()
                msg.min_range, msg.max_range, msg.range = .04, 4.0, distance
                self.range_pubs[direction].publish(msg)
            self.distance_pub.publish(Float32(self.distance))
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = self.pose_x
            msg.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(msg)

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timed out: state='+self.state+' status='+str(self.status))

    def hold(self, predicate, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            with self.guard:
                self.assertTrue(predicate())
            time.sleep(.01)


if __name__ == '__main__':
    rospy.init_node('test_safety_and_fine_tuning')
    rostest.rosrun('robot_navigation', 'safety_and_fine_tuning', SafetyAndFineTuning)
