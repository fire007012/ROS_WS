#!/usr/bin/env python3
"""Run both complete forward-only routes through the real ROS control chain."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt32
from robot_navigation.msg import QrResult
from robot_navigation.srv import OpenMedicineBox, OpenMedicineBoxResponse, Speak, SpeakResponse
from arm_and_gripper.srv import ArmPlaceMedicine, ArmPlaceMedicineResponse


class ClosedLoop(unittest.TestCase):
    def test_both_routes(self):
        self.guard = threading.RLock()
        self.running = True
        self.velocity = Twist()
        self.pose = [0., 0., 0.]
        self.sim_time = .001
        self.advance = False
        self.result = QrResult(1, 1, 3, 3)
        self.finished = self.failed = False
        self.boxes, self.placed, self.displays = [], [], []
        self.minimum_vx = 0.
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.measured = rospy.Publisher('/base_velocity', Twist, queue_size=1)
        self.distance = rospy.Publisher('/vl53l1x_distance', Float32, queue_size=1)
        self.motor = rospy.Publisher('/motor_state', Float32MultiArray, queue_size=1)
        self.ranges = [rospy.Publisher('/'+name+'/range', Range, queue_size=1)
                       for name in ['front', 'left', 'right']]
        self.qr = rospy.Publisher('/qr_result', QrResult, queue_size=1)
        self.start = rospy.Publisher('/start_signal/physical', UInt32, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_muxed', Twist, self.on_velocity),
                rospy.Subscriber('/mission_finished', Bool, self.on_finished),
                rospy.Subscriber('/emergency_stop', Bool, self.on_estop),
                rospy.Subscriber('/robot_display', String, self.on_display)]
        services = [rospy.Service('/open_medicine_box', OpenMedicineBox, self.open_box),
                    rospy.Service('/arm_place_medicine', ArmPlaceMedicine, self.place),
                    rospy.Service('/speak', Speak, lambda req: SpeakResponse(True, 'mock'))]
        thread = threading.Thread(target=self.simulate, daemon=True)
        thread.start()
        try:
            self.wait(lambda: self.start.get_num_connections() and self.qr.get_num_connections()
                      and self.odom.get_num_connections() >= 4
                      and self.motor.get_num_connections() >= 1
                      and all(pub.get_num_connections() >= 3 for pub in self.ranges), 8)
            time.sleep(.3)
            self.advance = True
            for bed in [1, 3]:
                with self.guard:
                    self.pose = [0., 0., 0.]
                    self.result = QrResult(bed, 1, 4-bed, 3)
                    self.finished = False
                    self.boxes, self.placed, self.displays = [], [], []
                    started = self.sim_time
                self.start.publish(UInt32(0x5A17))
                self.wait(lambda: self.finished or self.failed, 30)
                self.assertFalse(self.failed, str(self.displays))
                self.assertEqual(self.boxes, [1, 3])
                self.assertEqual(self.placed, [(bed, 1), (4-bed, 3)])
                self.assertGreaterEqual(self.minimum_vx, -.000001)
                self.assertLess(self.sim_time-started, 180.)
                rospy.loginfo('forward-only branch %d completed in %.2f simulated seconds', bed,
                              self.sim_time-started)
        finally:
            self.running = False
            thread.join(timeout=2)
            for sub in subs:
                sub.unregister()
            for service in services:
                service.shutdown()

    def simulate(self):
        # Advance 50 ms of ideal chassis physics per 5 ms wall time. Sensor
        # receipt freshness still uses the actual monotonic wall clock.
        while self.running and not rospy.is_shutdown():
            with self.guard:
                # Give all subscribers time to connect before accelerated time
                # starts; missing startup data must not be hidden by disabling health checks.
                dt = .05 if self.advance else 0.0
                self.sim_time += dt
                vx, vy, omega = self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z
                yaw = self.pose[2]
                self.pose[0] += (vx*math.cos(yaw)-vy*math.sin(yaw))*dt
                self.pose[1] += (vx*math.sin(yaw)+vy*math.cos(yaw))*dt
                self.pose[2] += omega*dt
                stamp = rospy.Time.from_sec(self.sim_time)
                self.clock.publish(Clock(stamp))
                msg = Odometry()
                msg.header.stamp = stamp
                msg.pose.pose.position.x, msg.pose.pose.position.y = self.pose[:2]
                msg.pose.pose.orientation.z = math.sin(self.pose[2]/2)
                msg.pose.pose.orientation.w = math.cos(self.pose[2]/2)
                msg.twist.twist = self.velocity
                self.odom.publish(msg)
                self.measured.publish(self.velocity)
                for pub in self.ranges:
                    msg = Range()
                    msg.header.stamp = stamp
                    msg.min_range, msg.max_range, msg.range = .04, 4., .8
                    pub.publish(msg)
                # Fine-tuning alignment is ideal; obstacle geometry is not modeled.
                self.distance.publish(Float32(400.))
                self.motor.publish(Float32MultiArray(data=[0., 0., 0., 0.]))
                self.qr.publish(self.result)
            time.sleep(.005)

    def on_velocity(self, msg):
        with self.guard:
            self.velocity = msg
            self.minimum_vx = min(self.minimum_vx, msg.linear.x)

    def on_finished(self, msg):
        with self.guard:
            self.finished = msg.data

    def on_estop(self, msg):
        with self.guard:
            self.failed |= msg.data

    def on_display(self, msg):
        with self.guard:
            self.displays.append(msg.data)
            self.failed |= '失败' in msg.data or '超时' in msg.data

    def open_box(self, req):
        with self.guard:
            self.boxes.append(req.box_id)
        return OpenMedicineBoxResponse(True, 'mock')

    def place(self, req):
        with self.guard:
            self.placed.append((req.bed_id, req.box_id))
        return ArmPlaceMedicineResponse(True, 'mock')

    def wait(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timeout pose='+str(self.pose)+' displays='+str(self.displays[-5:]))


if __name__ == '__main__':
    rospy.init_node('test_mission_closed_loop')
    rostest.rosrun('robot_navigation', 'mission_closed_loop', ClosedLoop)
