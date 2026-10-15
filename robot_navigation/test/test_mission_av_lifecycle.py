"""Guard mission-wide Ready ownership and per-bed service behavior."""
from pathlib import Path
import unittest
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
        self.assertIn('<arg name="capture_wait_for_ready" value="true"/>', peer)

if __name__ == '__main__':
    unittest.main()
