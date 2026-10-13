#!/usr/bin/env python3
"""Verify release-event ordering, exactly-once speech, and locked failure paths."""
import threading
import time
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Int8, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse
from path_manager.srv import SelectPath, SelectPathResponse
from robot_navigation.msg import QrResult
from robot_navigation.srv import OpenMedicineBox, OpenMedicineBoxResponse, Speak, SpeakResponse
from arm_and_gripper.srv import ArmPlaceMedicine, ArmPlaceMedicineResponse


class MissionMedicineEvent(unittest.TestCase):
    def test_event_gate(self):
        self.guard = threading.RLock()
        self.state = ''
        self.locked = False
        self.cancelled = 0
        self.arm_return = threading.Event()
        self.bedside_return = threading.Event()
        self.new_case('late')
        self.start = rospy.Publisher('/start_signal/physical', UInt32, queue_size=1)
        self.estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        self.reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        self.placed = rospy.Publisher('/medicine/placed', Int8, queue_size=10)
        self.qr = rospy.Publisher('/qr_result', QrResult, queue_size=1)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.path_done = rospy.Publisher('/path_finished', Bool, queue_size=1)
        self.fine_done = rospy.Publisher('/fine_tuning_done', Bool, queue_size=1)
        subs = [rospy.Subscriber('/mission/state', String, lambda m: self.store('state', m.data)),
                rospy.Subscriber('/chassis_lock', Bool, lambda m: self.store('locked', m.data)),
                rospy.Subscriber('/mission/cancel', Empty, self.on_cancel)]
        services = [rospy.Service('/select_path', SelectPath, self.select),
                    rospy.Service('/fine_tuning/start', Trigger, self.fine),
                    rospy.Service('/open_medicine_box', OpenMedicineBox,
                                  lambda r: OpenMedicineBoxResponse(True, 'simulated')),
                    rospy.Service('/arm_place_medicine', ArmPlaceMedicine, self.arm),
                    rospy.Service('/speak', Speak, self.speak),
                    rospy.Service('/bedside/complete', Trigger, self.bedside)]
        timer = rospy.Timer(rospy.Duration(.01), self.publish)
        try:
            self.wait(lambda: self.start.get_num_connections() and self.placed.get_num_connections())
            # Real arm behavior: release precedes home-return/service completion.
            # Verify both bed IDs, and prevent duplicate events from speaking twice.
            for bed in [1, 3]:
                self.new_case('early', bed)
                self.start.publish(UInt32(0x5A17))
                self.wait(lambda: len(self.spoken) == 1)
                self.assertFalse(self.arm_finished)
                self.assertEqual(self.state, 'PLACE_MEDICINE_A')
                self.assertTrue(self.locked)
                self.assertEqual(self.spoken, [f'{bed}床病人请取药'])
                self.arm_return.set()
                self.wait(lambda: self.bedside_started)
                self.assertEqual(len(self.spoken), 1)
                self.stop_case()

            self.new_case('late')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'VOICE_A' and self.arm_finished)
            self.assertTrue(self.locked)
            self.assertEqual(self.spoken, [])
            self.placed.publish(Int8(3))  # Wrong bed is not a release for bed 1.
            time.sleep(.07)
            self.assertEqual(self.spoken, [])
            self.assertEqual(self.state, 'VOICE_A')
            self.placed.publish(Int8(1))
            self.wait(lambda: self.bedside_started)
            self.assertEqual(self.spoken, ['1床病人请取药'])
            self.stop_case()

            self.new_case('late')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'VOICE_A')
            self.wait(lambda: self.state == 'FAILED', timeout=1)
            self.assertTrue(self.locked)
            self.assertEqual(self.spoken, [])
            # After timeout, a late release cannot re-enable speech or motion.
            self.placed.publish(Int8(1))
            time.sleep(.1)
            self.assertEqual(self.spoken, [])
            self.stop_case()

            self.new_case('late')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'VOICE_A')
            self.stop_case()
            self.placed.publish(Int8(1))
            time.sleep(.1)
            self.assertEqual(self.spoken, [])
            self.assertEqual(self.state, 'IDLE')
            self.assertTrue(self.locked)

            self.new_case('speech_failure')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'VOICE_A')
            self.placed.publish(Int8(1))
            self.wait(lambda: self.state == 'FAILED')
            self.assertEqual(self.spoken, ['1床病人请取药'])
            self.assertFalse(self.bedside_started)
            self.assertTrue(self.locked)
            self.stop_case()

            self.new_case('arm_failure')
            self.start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'FAILED')
            self.placed.publish(Int8(1))
            time.sleep(.1)
            self.assertEqual(self.spoken, [])
            self.assertTrue(self.locked)
        finally:
            self.arm_return.set()
            self.bedside_return.set()
            timer.shutdown()
            for sub in subs:
                sub.unregister()
            for service in services:
                service.shutdown()

    def new_case(self, mode, bed=1):
        with self.guard:
            self.mode, self.bed = mode, bed
            self.pose = (0., 0.)
            self.pending_path = self.pending_fine = None
            self.spoken = []
            self.arm_finished = self.bedside_started = False
            self.arm_return.clear()
            self.bedside_return.clear()

    def stop_case(self):
        before = self.cancelled
        self.estop.publish(Bool(True))
        self.wait(lambda: self.state == 'STOP' and self.cancelled > before)
        self.arm_return.set()
        self.bedside_return.set()
        time.sleep(.08)  # Let cancelled service workers finish before restart.
        self.reset.publish(Bool(True))
        self.wait(lambda: self.state == 'IDLE')

    def on_cancel(self, _):
        with self.guard:
            self.cancelled += 1

    def select(self, req):
        with self.guard:
            self.pose = (1.3, 0.) if req.path_name == 'nurse_station' else (
                4., 2.2 if self.bed == 1 else -2.2)
            self.pending_path = time.monotonic()+.06
        return SelectPathResponse(True, 'simulated')

    def fine(self, _):
        with self.guard:
            self.pending_fine = time.monotonic()+.06
        return TriggerResponse(True, 'simulated')

    def arm(self, req):
        if self.mode == 'early':
            self.placed.publish(Int8(req.bed_id))
            self.placed.publish(Int8(req.bed_id))
            self.arm_return.wait(2)
        with self.guard:
            self.arm_finished = True
            success = self.mode != 'arm_failure'
        return ArmPlaceMedicineResponse(success, 'simulated arm return')

    def speak(self, req):
        with self.guard:
            self.spoken.append(req.text)
            success = self.mode != 'speech_failure'
        return SpeakResponse(success, 'simulated speech')

    def bedside(self, _):
        self.store('bedside_started', True)
        self.bedside_return.wait(2)
        return TriggerResponse(False, 'simulated cancellation')

    def store(self, attr, value):
        with self.guard:
            setattr(self, attr, value)

    def publish(self, _):
        with self.guard:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose
            odom.pose.pose.orientation.w = 1.
            self.odom.publish(odom)
            self.qr.publish(QrResult(self.bed, 1, 4-self.bed, 3))
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
            time.sleep(.005)
        self.fail(f'timed out: state={self.state} speech={self.spoken}')


if __name__ == '__main__':
    rospy.init_node('test_mission_medicine_event')
    rostest.rosrun('robot_navigation', 'mission_medicine_event', MissionMedicineEvent)
