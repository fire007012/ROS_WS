#!/usr/bin/env python3
"""Raw CAN injection -> production bridge -> mux -> braking -> wheel mapping.
ROS time is frozen. This test never connects to CAN or physical actuators.
"""
import math
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String, UInt8MultiArray


class UpperAlignmentRos(unittest.TestCase):
    def test_pipeline(self):
        self.lock = threading.RLock()
        self.upper = {3: None, 4: None}
        self.seq = {i: 0 for i in range(5)}
        self.fixed_seq = set()
        self.original = {0: 800, 1: 800, 2: 800}
        self.output = Twist()
        self.wheels = []
        self.ranges, self.distances, self.diagnostics = {}, {}, {}
        self.protocol_sequences = {}
        self.active = self.done = False
        self.source = ''
        self.samples = []
        self.run_sender = True
        self.nav_enabled = False
        self.frames = rospy.Publisher('/test/range_can', UInt8MultiArray, queue_size=50)
        self.stage = rospy.Publisher('/mission/state', String, queue_size=1)
        self.enabled = rospy.Publisher('/mission/actions_enabled', Bool, queue_size=1, latch=True)
        self.cancel = rospy.Publisher('/mission/cancel', Empty, queue_size=1)
        self.estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        self.nav = rospy.Publisher('/cmd_vel_external', Twist, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_muxed', Twist, self.on_output),
                rospy.Subscriber('/motor_velocity_cmd', Float32MultiArray,
                                 lambda m: self.store('wheels', m.data)),
                rospy.Subscriber('/arm_upper_alignment/active', Bool,
                                 lambda m: self.store('active', m.data)),
                rospy.Subscriber('/arm_upper_alignment/done', Bool,
                                 lambda m: self.store('done', m.data)),
                rospy.Subscriber('/cmd_vel_mux/selected_source', String,
                                 lambda m: self.store('source', m.data)),
                rospy.Subscriber('/diagnostics', DiagnosticArray, self.on_diagnostics)]
        for i, topic in enumerate(['/front/range', '/left/range', '/right/range',
                                   '/arm_upper_left/range', '/arm_upper_right/range']):
            subs.append(rospy.Subscriber(topic, Range, lambda m, i=i: self.on_range(i, m)))
        for i, topic in enumerate(['/vl53l1x_distance', '/vl53l1x_distance_left',
                                   '/vl53l1x_distance_right']):
            subs.append(rospy.Subscriber(topic, Float32, lambda m, i=i: self.on_distance(i, m)))
        thread = threading.Thread(target=self.send_loop, daemon=True)
        thread.start()
        try:
            self.wait(lambda: self.frames.get_num_connections() and self.stage.get_num_connections())
            self.wait(lambda: all(i in self.ranges for i in range(3)))
            for i in range(3):
                self.assertAlmostEqual(self.ranges[i].range, .8, places=5)
                self.assertEqual(self.ranges[i].radiation_type, Range.INFRARED)
                self.assertEqual(self.distances[i], 800)
            self.set_pair(500, 1000)
            self.hold(lambda: self.output.linear.y == 0 and not self.active, .25)
            self.enabled.publish(Bool(True))
            time.sleep(.05)
            self.set_pair(None, None)
            self.stage.publish(String('ARM_UPPER_ALIGN_A'))
            self.wait(lambda: self.active)
            self.hold(lambda: self.output.linear.y == 0, .25)
            self.set_pair(500, None)
            self.hold(lambda: self.output.linear.y == 0, .12)
            # Original left/right readings cannot fill the missing upper right.
            self.set_pair(500, 1000)
            self.wait(lambda: self.output.linear.y > .049)
            self.assert_lateral()
            self.wait(lambda: len(self.wheels) == 4 and self.wheels[1] > 0)
            self.assertEqual(self.wheels[0], 0)
            self.assertEqual(self.wheels[2], 0)
            self.assertAlmostEqual(self.wheels[1], -self.wheels[3], places=5)
            # Navigation keeps transmitting, but the exclusive stage owns motion.
            self.nav_enabled = True
            self.hold(lambda: self.output.linear.x == 0 and self.output.angular.z == 0, .2)
            self.set_pair(1000, 500)
            self.wait(lambda: self.output.linear.y < -.049)
            self.assert_lateral()
            self.set_pair(500, 500)
            self.wait(lambda: self.output.linear.y == 0)
            self.set_pair(500, 530)
            self.hold(lambda: self.output.linear.y == 0, .15)
            self.set_pair(530, 500)
            self.hold(lambda: self.output.linear.y == 0, .15)
            self.set_pair(500, 550)
            self.wait(lambda: self.output.linear.y > .049)
            self.set_pair(500, 530)
            self.hold(lambda: self.output.linear.y > 0, .15)
            self.set_pair(530, 500)
            self.wait(lambda: self.output.linear.y == 0)
            self.hold(lambda: self.output.linear.y == 0, .12)
            # All invalid/fault statuses must stop rather than retain last range.
            for distance, status in [(65535, 0), (1000, 4), (1000, 8), (1000, 17), (3001, 1)]:
                self.set_pair(500, 1000)
                self.wait(lambda: self.output.linear.y > .049)
                with self.lock:
                    self.upper[4] = (distance, status)
                self.wait(lambda: self.output.linear.y == 0)
                if status != 17:
                    self.wait(lambda: math.isnan(self.ranges[4].range))
            self.set_pair(500, 1000)
            self.wait(lambda: self.output.linear.y > .049)
            with self.lock:
                self.fixed_seq.add(4)
            self.wait(lambda: self.output.linear.y == 0)
            self.hold(lambda: self.output.linear.y == 0, .15)
            with self.lock:
                self.fixed_seq.clear()
            self.wait(lambda: self.output.linear.y > .049)
            with self.lock:
                self.fixed_seq.update([3, 4])
                self.seq[3] = self.seq[4] = 255
            self.wait(lambda: self.protocol_sequences.get('STM32 Range/arm_upper_left_vl53l1x') == 255 and self.protocol_sequences.get('STM32 Range/arm_upper_right_vl53l1x') == 255)
            with self.lock:
                self.seq[3] = self.seq[4] = 0
            self.wait(lambda: self.protocol_sequences.get('STM32 Range/arm_upper_left_vl53l1x') == 0 and self.protocol_sequences.get('STM32 Range/arm_upper_right_vl53l1x') == 0)
            self.wait(lambda: self.output.linear.y > 0)
            with self.lock:
                self.fixed_seq.clear()
            self.set_pair(500, None)
            self.wait(lambda: self.output.linear.y == 0)
            self.set_pair(500, 1000)
            self.wait(lambda: self.output.linear.y > .049)
            # A real diagnostic frame invalidates the side immediately.
            with self.lock:
                self.upper[4] = None
                self.frames.publish(UInt8MultiArray(data=[0x11, 1, 1, 4, 1, 3, 0xff, 0xff, 8, 9]))
            self.wait(lambda: self.output.linear.y == 0)
            self.wait(lambda: self.diagnostics.get('STM32 Range/arm_upper_right_vl53l1x', 0) == 2)
            self.set_pair(500, 1000)
            self.wait(lambda: self.output.linear.y > .049)
            # Existing side braking retains priority. Upper sensor 200 mm by
            # itself is a valid comparison input, not a 300 mm obstacle beam.
            self.set_pair(200, 1000)
            self.hold(lambda: self.output.linear.y > 0, .12)
            with self.lock:
                self.original[1] = 190
            self.wait(lambda: self.source == 'safety' and self.output.linear.y == 0)
            with self.lock:
                self.original[1] = 800
            self.wait(lambda: self.output.linear.y > .049)
            # Only motion toward a valid chassis beam brakes. Front/right
            # 250 mm must not recreate the removed firmware-wide 300 mm stop.
            for original_id in [0, 2]:
                with self.lock:
                    self.original[original_id] = 250
                self.hold(lambda: self.output.linear.y > .049, .15)
                with self.lock:
                    self.original[original_id] = 800
            with self.lock:
                self.original[1] = 190
            self.wait(lambda: self.output.linear.y == 0)
            with self.lock:
                self.original[1] = 800
            self.wait(lambda: self.output.linear.y > .049)
            self.nav_enabled = False
            self.stage.publish(String('POSITION_IN_CIRCLE_A'))
            self.wait(lambda: not self.active and self.output.linear.y == 0)
            self.set_pair(None, None)
            self.stage.publish(String('ARM_UPPER_ALIGN_B'))
            self.wait(lambda: self.active)
            self.hold(lambda: self.output.linear.y == 0, .15)
            self.set_pair(500, 1000)
            self.wait(lambda: self.output.linear.y > .049)
            self.nav_enabled = True
            self.cancel.publish(Empty())
            self.wait(lambda: self.output.linear.y == 0 and not self.active)
            self.hold(lambda: self.output.linear.x == 0 and self.output.linear.y == 0, .2)
            self.nav_enabled = False
            self.stage.publish(String('IDLE'))
            time.sleep(.05)
            self.enabled.publish(Bool(True))
            time.sleep(.05)
            self.stage.publish(String('ARM_UPPER_ALIGN_A'))
            self.wait(lambda: self.output.linear.y > .049)
            self.estop.publish(Bool(True))
            self.wait(lambda: self.output.linear.y == 0 and not self.active)
            self.hold(lambda: self.output.linear.y == 0, .15)
            self.reset.publish(Bool(True))
            self.estop.publish(Bool(False))
            self.hold(lambda: self.output.linear.y == 0, .15)
            # A stopped/crashed alignment producer cannot hand control to a
            # navigation publisher that continues transmitting in this mode.
            self.isolated_output = Twist()
            expiry_sub = rospy.Subscriber('/expiry/output', Twist,
                lambda m: self.store('isolated_output', m))
            subs.append(expiry_sub)
            mode = rospy.Publisher('/expiry/active', Bool, queue_size=1)
            command = rospy.Publisher('/expiry/alignment_command', Twist, queue_size=1)
            navigation = rospy.Publisher('/expiry/nav', Twist, queue_size=1)
            self.wait(lambda: mode.get_num_connections() and command.get_num_connections())
            mode.publish(Bool(True))
            time.sleep(.05)
            lateral = Twist()
            lateral.linear.x, lateral.linear.y, lateral.angular.z = .2, .05, .2
            for _ in range(5):
                command.publish(lateral)
                time.sleep(.03)
            self.wait(lambda: self.isolated_output.linear.y > .049)
            self.assertEqual(self.isolated_output.linear.x, 0)
            self.assertEqual(self.isolated_output.angular.z, 0)
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                navigation.publish(lateral)
                time.sleep(.02)
            self.assertEqual(self.isolated_output.linear.x, 0)
            self.assertEqual(self.isolated_output.linear.y, 0)
            self.assertEqual(self.isolated_output.angular.z, 0)
        finally:
            self.run_sender = False
            thread.join(1)
            for s in subs:
                s.unregister()

    def assert_lateral(self):
        self.assertEqual(self.output.linear.x, 0)
        self.assertEqual(self.output.angular.z, 0)
        self.assertLessEqual(abs(self.output.linear.y), .050001)

    def set_pair(self, left, right):
        with self.lock:
            self.upper = {3: None if left is None else (left, 1),
                          4: None if right is None else (right, 1)}

    def store(self, attr, value):
        with self.lock:
            setattr(self, attr, value)

    def on_output(self, msg):
        self.store('output', msg)

    def on_range(self, i, msg):
        with self.lock:
            self.ranges[i] = msg

    def on_distance(self, i, msg):
        with self.lock:
            self.distances[i] = msg.data

    def on_diagnostics(self, msg):
        with self.lock:
            self.diagnostics = {s.name: s.level for s in msg.status}
            self.protocol_sequences = {s.name: int(v.value) for s in msg.status
                                       for v in s.values if v.key == 'sequence'}

    def send_loop(self):
        while self.run_sender and not rospy.is_shutdown():
            with self.lock:
                values = {i: (d, 1) for i, d in self.original.items()}
                values.update(self.upper)
                for i, value in values.items():
                    if value is None:
                        continue
                    if i not in self.fixed_seq:
                        self.seq[i] = (self.seq[i]+1) & 255
                    distance, status = value
                    self.frames.publish(UInt8MultiArray(data=[0x10, 1, 1, i,
                        distance & 255, distance >> 8, 255, 255, status, self.seq[i]]))
                if self.nav_enabled:
                    nav = Twist()
                    nav.linear.x, nav.angular.z = .2, .2
                    self.nav.publish(nav)
            time.sleep(.03)

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timeout, active=%s source=%s velocity=%s' %
                  (self.active, self.source, self.output))

    def hold(self, predicate, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            with self.lock:
                self.assertTrue(predicate(), 'unexpected command: '+str(self.output))
            time.sleep(.01)


if __name__ == '__main__':
    rospy.init_node('test_upper_alignment')
    rostest.rosrun('robot_navigation', 'upper_alignment', UpperAlignmentRos)
