#!/usr/bin/env python3
"""Keep media ready independently of mission start, stop, or measurement."""
import rospy
from std_msgs.msg import Int32

if __name__ == '__main__':
    rospy.init_node('audio_video_ready_node')
    topic = rospy.get_param('~ready_topic', '/audio_video/ready')
    publisher = rospy.Publisher(topic, Int32, queue_size=1, latch=True)
    publisher.publish(Int32(1))
    rospy.loginfo('[audio_video] continuous media enabled on %s; independent of mission /Ready', topic)
    rospy.spin()
