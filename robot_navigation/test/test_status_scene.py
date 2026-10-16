"""Offline field geometry and pose validity; no ROS or desktop required."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from status_scene import PoseTrack, field_transform, draw_field, task_text


class Canvas:
    def __init__(self):
        self.items = []
    def delete(self, *_):
        self.items = []
    def __getattr__(self, method):
        def record(*args, **kwargs):
            self.items.append((method, args, kwargs))
        return record


class SceneTests(unittest.TestCase):
    def test_arena_axes_and_scale_match_odom(self):
        point, scale = field_transform(620, 280, [-.5, 6.5, -3.5, 3.5])
        home, nurse = point(0, 0), point(1.3, 0)
        self.assertLess(nurse[1], home[1])
        self.assertLess(point(4, 2.2)[0], point(4, -2.2)[0])
        self.assertAlmostEqual(abs(point(1,0)[1]-home[1]), scale)
        self.assertAlmostEqual(abs(point(0,1)[0]-home[0]), scale)

    def test_invalid_frames_timestamps_and_quaternions_do_not_replace_pose(self):
        obj = PoseTrack()
        self.assertIsNone(obj.pose)
        self.assertTrue(obj.update('/odom', 0, 0, (0,0,0,1), 10))
        for frame, x, q, age in [('map', 1, (0,0,0,1),0),
                                 ('odom', float('nan'), (0,0,0,1),0),
                                 ('odom', 1, (0,0,0,0),0),
                                 ('odom', 1, (0,0,0,1),2)]:
            self.assertFalse(obj.update(frame, x, 1, q, 11, age))
            self.assertEqual(obj.pose, (0,0,0))
            self.assertEqual(obj.received, 10)
        self.assertTrue(obj.update('odom', .1, .1, (0,0,2,2), 12))
        self.assertAlmostEqual(obj.pose[2], math.pi/2)
        self.assertEqual(obj.problem, '')

    def test_trace_has_bounded_memory_and_pose_reset_breaks_line(self):
        obj=PoseTrack(limit=20)
        for i in range(100):
            obj.update('odom', i*.03, 0, (0,0,0,1), i)
        self.assertEqual(len(obj.trail),20)
        obj.update('odom', 0, 0, (0,0,0,1), 101)
        self.assertEqual(list(obj.trail),[(0,0)])

    def test_waiting_stale_fresh_and_frame_error_are_visible(self):
        obj=PoseTrack()
        canvas=Canvas()
        def texts():
            return [kw['text'] for _,_,kw in canvas.items if 'text' in kw]
        draw_field(canvas,620,280,obj.snapshot(),{},10)
        self.assertTrue(any('等待定位' in text for text in texts()))
        obj.update('odom',0,0,(0,0,0,1),10)
        draw_field(canvas,620,280,obj.snapshot(),{},10.5)
        self.assertTrue(any('位置估计' in text for text in texts()))
        draw_field(canvas,620,280,obj.snapshot(),{},12)
        self.assertTrue(any('定位已过期' in text for text in texts()))
        obj.update('map',0,0,(0,0,0,1),12)
        draw_field(canvas,620,280,obj.snapshot(),{},12)
        self.assertTrue(any('坐标系不匹配' in text for text in texts()))

    def test_specific_task_and_box_are_not_visit_numbers(self):
        assignment=dict(first_bed=3,first_box=1,second_bed=1,second_box=3)
        self.assertIn('正在前往3号床',task_text('GOTO_BED_A',assignment))
        self.assertIn('向1号床放药（3号药箱）',task_text('PLACE_MEDICINE_B',assignment))
        self.assertIn('正在等待二维码',task_text('SCAN_QR'))
        self.assertNotIn('None',task_text('SCAN_BARCODE_A'))
        self.assertIn('任务完成',task_text('STOP',finished=True))
        self.assertNotIn('任务完成',task_text('STOP',finished=False))


if __name__ == '__main__':
    unittest.main()
