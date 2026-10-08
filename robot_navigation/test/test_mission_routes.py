#!/usr/bin/env python3
"""Run the real mission state machine with simulated navigation and task services."""
from pathlib import Path
import threading
import time
import unittest

import rospy
import rostest
import yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse
from path_manager.srv import SelectPath, SelectPathResponse
from robot_navigation.msg import QrResult
from robot_navigation.srv import OpenMedicineBox, OpenMedicineBoxResponse, Speak, SpeakResponse
from arm_and_gripper.srv import ArmPlaceMedicine, ArmPlaceMedicineResponse


class MissionRoutes(unittest.TestCase):
    def test_qr_routes_projection_and_home_hold(self):
        source = Path(__file__).resolve().parents[2]
        with (source / 'robot_bringup/config/paths.yaml').open() as stream:
            self.paths = {p['name']: p['points'] for p in yaml.safe_load(stream)['paths']}
        self.guard = threading.RLock()
        self.reset()
        self.start = rospy.Publisher('/start_signal/physical', UInt32, queue_size=1)
        self.qr = rospy.Publisher('/qr_result', QrResult, queue_size=1)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path_done = rospy.Publisher('/path_finished', Bool, queue_size=1)
        self.fine_done = rospy.Publisher('/fine_tuning_done', Bool, queue_size=1)
        subscribers = [rospy.Subscriber('/mission_finished', Bool, self.on_finished),
                       rospy.Subscriber('/robot_display', String, self.on_display)]
        services = [rospy.Service('/select_path', SelectPath, self.select_path),
                    rospy.Service('/fine_tuning/start', Trigger, self.fine_tuning),
                    rospy.Service('/open_medicine_box', OpenMedicineBox, self.open_box),
                    rospy.Service('/arm_place_medicine', ArmPlaceMedicine, self.place),
                    rospy.Service('/speak', Speak, lambda req: SpeakResponse(True, 'simulated'))]
        timer = rospy.Timer(rospy.Duration(.02), self.publish)
        try:
            self.wait(lambda: self.start.get_num_connections() and self.qr.get_num_connections())
            for first_bed, first_box in [(1, 1), (1, 3), (3, 1), (3, 3)]:
                with self.subTest(qr=f'{first_bed}{first_box}'):
                    self.reset()
                    self.result = QrResult(first_bed, first_box, 4-first_bed, 4-first_box)
                    self.block_projection = True
                    self.start.publish(UInt32(0x5A17))
                    self.wait(lambda: self.fine_calls == 1)
                    time.sleep(.3)
                    self.assertEqual(self.boxes, [], 'partial chassis projection accepted')
                    with self.guard:
                        self.pose[0] -= .15
                        self.block_projection = False
                    self.wait(lambda: len(self.selected) == 4)
                    # Within the home rectangle but still moving: do not count hold time.
                    time.sleep(.4)
                    self.assertFalse(self.finished)
                    with self.guard:
                        self.home_velocity = 0.0
                        stationary_since = time.monotonic()
                    self.wait(lambda: self.finished, timeout=7)
                    self.assertGreaterEqual(time.monotonic()-stationary_since, 5.0)
                    expected = (['nurse_station', 'bed1_circle', 'bed1_to_bed3', 'bed3_to_home']
                                if first_bed == 1 else
                                ['nurse_station', 'bed3_circle', 'bed3_to_bed1', 'bed1_to_home'])
                    self.assertEqual(self.selected, expected)
                    self.assertEqual(self.boxes, [first_box, 4-first_box])
                    self.assertEqual(self.placed, [(first_bed, first_box),
                                                   (4-first_bed, 4-first_box)])
            # If a dedicated return path is absent, stop instead of cutting to P7.
            self.reset()
            self.result = QrResult(1, 1, 3, 3)
            self.reject_return = True
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.failed)
            self.assertEqual(self.selected[-1], 'bed3_to_home')
            self.assertNotIn('HOME', self.selected)
        finally:
            timer.shutdown()
            for service in services:
                service.shutdown()
            for sub in subscribers:
                sub.unregister()

    def reset(self):
        with self.guard:
            self.selected, self.boxes, self.placed = [], [], []
            self.pose = [0.0, 0.0]
            self.finished = self.failed = False
            self.pending_path = self.pending_fine = None
            self.fine_calls = 0
            self.result = None
            self.home_velocity = .02
            self.block_projection = self.reject_return = False

    def select_path(self, req):
        with self.guard:
            self.selected.append(req.path_name)
            if self.reject_return and req.path_name.endswith('_to_home'):
                return SelectPathResponse(False, 'simulated missing route')
            endpoint = self.paths[req.path_name][-1]
            self.pose = [endpoint['x'], endpoint['y']]
            if len(self.selected) == 2 and self.block_projection:
                self.pose[0] += .15
            self.pending_path = time.monotonic()+.15
        return SelectPathResponse(True, 'simulated arrival')

    def fine_tuning(self, req):
        with self.guard:
            self.fine_calls += 1
            self.pending_fine = time.monotonic()+.1
        return TriggerResponse(True, 'simulated fine tuning')

    def open_box(self, req):
        with self.guard:
            self.boxes.append(req.box_id)
        return OpenMedicineBoxResponse(True, 'simulated')

    def place(self, req):
        with self.guard:
            self.placed.append((req.bed_id, req.box_id))
        return ArmPlaceMedicineResponse(True, 'simulated')

    def on_finished(self, msg):
        with self.guard:
            self.finished = msg.data

    def on_display(self, msg):
        with self.guard:
            self.failed |= msg.data.startswith('任务失败:')

    def publish(self, event):
        with self.guard:
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x, msg.pose.pose.position.y = self.pose
            msg.pose.pose.orientation.w = 1.0
            if len(self.selected) == 4:
                msg.twist.twist.linear.x = self.home_velocity
            self.odom.publish(msg)
            if self.result is not None:
                self.qr.publish(self.result)
            now = time.monotonic()
            if self.pending_path is not None and now >= self.pending_path:
                self.path_done.publish(Bool(True))
                self.pending_path = None
            if self.pending_fine is not None and now >= self.pending_fine:
                self.fine_done.publish(Bool(True))
                self.pending_fine = None

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.guard:
                if predicate():
                    return
            time.sleep(.01)
        self.fail(f'condition timed out; routes={self.selected}, boxes={self.boxes}')


if __name__ == '__main__':
    rospy.init_node('test_mission_routes')
    rostest.rosrun('robot_navigation', 'mission_routes', MissionRoutes)
