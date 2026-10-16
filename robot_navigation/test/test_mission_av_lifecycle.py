"""Guard legacy mission Ready ownership while media can operate independently."""
from pathlib import Path
import unittest
import runpy
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
ROOT = Path(__file__).resolve().parents[1]

class MissionAvLifecycle(unittest.TestCase):
    def test_only_mission_controller_owns_ready_for_full_mission(self):
        mission = (ROOT / 'src/mission_controller.cpp').read_text(encoding='utf-8')
        bedside = (ROOT / 'scripts/bedside_session_node.py').read_text(encoding='utf-8')
        capture = (ROOT / 'scripts/adaptive_scan_camera_node.py').read_text(encoding='utf-8')
        peer = (ROOT / 'launch/audio_video_peer.launch').read_text(encoding='utf-8')
        self.assertIn('advertise<std_msgs::Int32>("/Ready",1,true)', mission)
        self.assertIn('av_on.data=1; av_ready_pub_.publish(av_on)', mission)
        self.assertGreaterEqual(mission.count('av_off.data=0; av_ready_pub_.publish(av_off)'), 3)
        self.assertNotIn("Publisher('/Ready'", bedside)
        self.assertNotIn('self.ready.publish', bedside)
        self.assertIn('desired = self.policy.requested(time.monotonic()) if self.ready else None', capture)
        self.assertIn("<arg name=\"capture_wait_for_ready\" value=\"$(eval not arg('always_on'))\"/>", peer)

class ContinuousMediaStartup(unittest.TestCase):
    def test_ready_node_enables_only_media_without_mission_start(self):
        events, publishers = [], []
        ros = ModuleType('rospy')
        ros.init_node = lambda *a: None
        ros.get_param = lambda key, default: default
        ros.loginfo = lambda *a: None
        ros.spin = lambda: None
        def publish(topic, kind, **kwargs):
            publishers.append((topic, kwargs))
            return SimpleNamespace(publish=lambda msg: events.append(msg.data))
        ros.Publisher = publish
        std = ModuleType('std_msgs.msg')
        std.Int32 = lambda value: SimpleNamespace(data=value)
        with patch.dict(sys.modules, {'rospy': ros, 'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': std}):
            runpy.run_path(str(ROOT/'scripts/audio_video_ready_node.py'), run_name='__main__')
        self.assertEqual(publishers, [('/audio_video/ready', dict(queue_size=1, latch=True))])
        self.assertEqual(events, [1])


if __name__ == '__main__':
    unittest.main()
