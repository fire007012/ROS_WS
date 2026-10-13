#!/usr/bin/env python3
"""Raw rear CAN frames -> production bridge -> braking -> mux, without hardware."""
import math
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, String, UInt8MultiArray


class RearCanSafety(unittest.TestCase):
    def test_rear_protection(self):
        self.lock = threading.RLock()
        self.command, self.output = Twist(), Twist()
        self.state = ''
        self.rear = None
        self.last_range = Range()
        self.distance = float('nan')
        self.diagnostic = None
        self.sequence = 0
        self.rear_sequence = 0
        self.repeat = False
        self.running = True
        self.frames = rospy.Publisher('/test/range_can', UInt8MultiArray, queue_size=50)
        self.commands = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_muxed', Twist, lambda m: self.store('output', m)),
                rospy.Subscriber('/distance_safety/state', String, lambda m: self.store('state', m.data)),
                rospy.Subscriber('/rear/range', Range, lambda m: self.store('last_range', m)),
                rospy.Subscriber('/vl53l1x_distance_rear', Float32, lambda m: self.store('distance', m.data)),
                rospy.Subscriber('/diagnostics', DiagnosticArray, self.on_diagnostics)]
        thread = threading.Thread(target=self.send_loop, daemon=True)
        thread.start()
        try:
            self.command.linear.x = -.2
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.linear.x == 0)
            self.rear = (800, 1)
            self.wait_reverse()
            self.wait(lambda: self.distance == 800)
            self.assertEqual(self.last_range.header.frame_id, 'rear_range_link')
            self.assertEqual(self.last_range.radiation_type, Range.INFRARED)
            self.assertAlmostEqual(self.last_range.range, .8, places=5)
            self.assertEqual(self.distance, 800)
            self.rear = (200, 1)
            self.wait(lambda: self.state == 'STOP' and self.output.linear.x == 0)
            # At 0.2 m/s the threshold is 360 mm, release requires >390 mm.
            self.rear = (380, 1)
            time.sleep(.12)
            self.assertEqual(self.state, 'STOP')
            self.rear = (450, 1)
            self.wait(lambda: self.state == 'WARNING' and abs(self.output.linear.x + .06) < .001)
            self.rear = (800, 1)
            self.wait_reverse()
            for distance, status in [(65535, 4), (800, 8), (49, 1), (3001, 1)]:
                self.rear = (distance, status)
                self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.linear.x == 0)
                self.wait(lambda: math.isnan(self.last_range.range) and math.isnan(self.distance))
                self.rear = (800, 1)
                self.wait_reverse()
            # Repeated sequences cannot keep old clearance alive with frozen ROS time.
            self.repeat = True
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.linear.x == 0)
            self.wait(lambda: self.diagnostic == 2)
            self.repeat = False
            self.wait_reverse()
            self.rear = None
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.linear.x == 0)
            self.rear = (800, 1)
            self.wait_reverse()
            # Diagnostic failure and malformed rear frames invalidate the last good sample.
            with self.lock:
                self.rear = None
                self.frames.publish(UInt8MultiArray(data=[0x11, 1, 1, 5, 3, 4, 0x20, 3, 4, 1]))
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and math.isnan(self.last_range.range))
            self.rear = (800, 1)
            self.wait_reverse()
            with self.lock:
                self.rear = None
                self.frames.publish(UInt8MultiArray(data=[0x10, 1, 1, 5, 0x20, 3]))
            self.wait(lambda: self.state == 'REAR_UNOBSERVED' and math.isnan(self.last_range.range))
            # Both turn directions check the rear, including turn while moving forward.
            for forward, rotation in [(0, .2), (0, -.2), (.2, .2)]:
                with self.lock:
                    self.command.linear.x = forward
                    self.command.angular.z = rotation
                    self.rear = (200, 1)
                self.wait(lambda: self.state == 'STOP' and self.output.angular.z == 0)
                self.rear = (800, 1)
                self.wait(lambda: self.state == 'CLEAR' and abs(self.output.angular.z) > .19)
                self.rear = None
                self.wait(lambda: self.state == 'REAR_UNOBSERVED' and self.output.angular.z == 0)
            # A rear failure does not prevent straight forward escape.
            with self.lock:
                self.command.linear.x = .2
                self.command.angular.z = 0
            self.wait(lambda: self.state == 'CLEAR' and self.output.linear.x > .19)
        finally:
            self.running = False
            thread.join(timeout=1)
            for sub in subs:
                sub.unregister()

    def store(self, name, value):
        with self.lock:
            setattr(self, name, value)

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'STM32 Range/rear_vl53l1x':
                self.store('diagnostic', status.level)

    def send_loop(self):
        while self.running and not rospy.is_shutdown():
            with self.lock:
                self.sequence = (self.sequence + 1) % 256
                if not self.repeat:
                    self.rear_sequence = (self.rear_sequence + 1) % 256
                for sensor_id in (0, 1, 2, 5):
                    sample = self.rear if sensor_id == 5 else (800, 1)
                    if sample is None:
                        continue
                    distance, status = sample
                    sequence = self.rear_sequence if sensor_id == 5 else self.sequence
                    self.frames.publish(UInt8MultiArray(data=[0x10, 1, 1, sensor_id,
                        distance & 255, distance >> 8, 255, 255, status, sequence]))
                self.commands.publish(self.command)
            time.sleep(.02)

    def wait_reverse(self):
        self.wait(lambda: self.state == 'CLEAR' and self.output.linear.x < -.19
                  and math.isfinite(self.last_range.range))

    def wait(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timed out: safety=' + self.state)


if __name__ == '__main__':
    rospy.init_node('test_rear_can_safety')
    rostest.rosrun('robot_navigation', 'rear_can_safety', RearCanSafety)
