#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import Int8
from arm_and_gripper.srv import ArmPlaceMedicine
from robot_navigation.srv import Speak

if __name__ == '__main__':
    rospy.init_node('arm_demo_once')
    bed = int(rospy.get_param('~bed_id', 1))
    box = int(rospy.get_param('~box_id', 1))
    done = threading.Event()
    result = []
    def placed(msg):
        if msg.data != bed or done.is_set():
            return
        done.set()
        try:
            response = rospy.ServiceProxy('/speak', Speak)('%d床病人请取药' % bed)
            result.append(response.success)
        except rospy.ServiceException:
            result.append(False)
    subscriber = rospy.Subscriber('/medicine/placed', Int8, placed, queue_size=1)
    try:
        rospy.wait_for_service('/speak', timeout=20)
        rospy.wait_for_service('/arm_place_medicine', timeout=20)
        # Give publisher/subscriber negotiation time before the only motion call.
        rospy.sleep(0.5)
        response = rospy.ServiceProxy('/arm_place_medicine', ArmPlaceMedicine)(bed, box)
        if not response.success:
            raise RuntimeError(response.message)
        deadline = rospy.Time.now() + rospy.Duration(15)
        while not rospy.is_shutdown() and not result and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        if not result or not result[0]:
            raise RuntimeError('arm returned but speech missing/failed')
        rospy.loginfo('[arm_demo] one sequence and speech completed; no automatic repetition')
    except (rospy.ROSException, RuntimeError) as exc:
        rospy.logerr('[arm_demo] %s; no retry', exc)
        raise SystemExit(1)
