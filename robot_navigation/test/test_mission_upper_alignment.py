#!/usr/bin/env python3
"""Verify the optional state entry and completion with the real mission node."""
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse
from path_manager.srv import SelectPath, SelectPathResponse
from robot_navigation.msg import QrResult
from robot_navigation.srv import OpenMedicineBox, OpenMedicineBoxResponse, Speak, SpeakResponse
from arm_and_gripper.srv import ArmPlaceMedicine, ArmPlaceMedicineResponse


class MissionUpperAlignment(unittest.TestCase):
    def test_stage_entry(self):
        self.state = ''
        self.fine_calls = 0
        def fine(req):
            self.fine_calls += 1
            return TriggerResponse(True, 'simulated')
        services = [rospy.Service('/select_path', SelectPath,
                                  lambda r: SelectPathResponse(True, 'simulated')),
                    rospy.Service('/fine_tuning/start', Trigger, fine),
                    rospy.Service('/open_medicine_box', OpenMedicineBox,
                                  lambda r: OpenMedicineBoxResponse(True, 'simulated')),
                    rospy.Service('/arm_place_medicine', ArmPlaceMedicine,
                                  lambda r: ArmPlaceMedicineResponse(True, 'simulated')),
                    rospy.Service('/speak', Speak, lambda r: SpeakResponse(True, 'simulated'))]
        sub = rospy.Subscriber('/mission/state', String, lambda m: setattr(self, 'state', m.data))
        start = rospy.Publisher('/start_signal/physical', UInt32, queue_size=1)
        path_done = rospy.Publisher('/path_finished', Bool, queue_size=1)
        qr = rospy.Publisher('/qr_result', QrResult, queue_size=1)
        done = rospy.Publisher('/arm_upper_alignment/done', Bool, queue_size=1)
        estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        try:
            self.wait(lambda: start.get_num_connections() and done.get_num_connections())
            start.publish(UInt32(0x5A17))
            self.wait(lambda: self.state == 'GOTO_NURSE')
            time.sleep(.15)
            path_done.publish(Bool(True))
            self.wait(lambda: self.state == 'SCAN_QR')
            time.sleep(.1)
            qr.publish(QrResult(1, 1, 3, 3))
            self.wait(lambda: self.state == 'GOTO_BED_A')
            time.sleep(.15)
            path_done.publish(Bool(True))
            self.wait(lambda: self.state == 'ARM_UPPER_ALIGN_A')
            self.assertEqual(self.fine_calls, 0)
            # An old completed result alone cannot skip a newly entered stage.
            done.publish(Bool(True))
            time.sleep(.2)
            self.assertEqual(self.state, 'ARM_UPPER_ALIGN_A')
            self.assertEqual(self.fine_calls, 0)
            done.publish(Bool(False))
            time.sleep(.05)
            done.publish(Bool(True))
            self.wait(lambda: self.state == 'POSITION_IN_CIRCLE_A' and self.fine_calls == 1)
            estop.publish(Bool(True))
            self.wait(lambda: self.state == 'STOP')
        finally:
            sub.unregister()
            for s in services:
                s.shutdown()

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        self.fail('timed out; state='+self.state)


if __name__ == '__main__':
    rospy.init_node('test_mission_upper_alignment')
    rostest.rosrun('robot_navigation', 'mission_upper_alignment', MissionUpperAlignment)
