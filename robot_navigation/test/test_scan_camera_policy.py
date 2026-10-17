from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scan_camera_policy import ScanCameraPolicy, capture_dimensions_acceptable

class ScanCameraPolicyTests(unittest.TestCase):
    def test_exact_five_seconds_each_scan(self):
        p=ScanCameraPolicy(); self.assertIsNone(p.requested(0))
        p.enable_qr(True,10); self.assertEqual(p.requested(14.999),'low')
        self.assertEqual(p.requested(15),'high')
        p.decoded('qr',16); self.assertIsNone(p.requested(20))
        p.target(1,30); self.assertEqual(p.requested(34.999),'low')
        self.assertEqual(p.requested(35),'high')
        p.decoded('barcode',36); self.assertIsNone(p.requested(37))
        p.target(3,40); self.assertEqual(p.requested(44.999),'low')
        self.assertEqual(p.requested(45),'high')

    def test_disabling_qr_does_not_disable_barcode_capture(self):
        p=ScanCameraPolicy();p.target(1,0);p.enable_qr(False,1)
        self.assertEqual(p.requested(2),'low')

    def test_fixed_modes_never_switch(self):
        for mode in ['low','high']:
            p=ScanCameraPolicy(mode=mode,continuous=True)
            for now in [0,5,1000]:self.assertEqual(p.requested(now),mode)
            p.decoded('qr',1000);self.assertEqual(p.requested(1001),mode)

    def test_continuous_returns_low_after_result(self):
        p=ScanCameraPolicy(continuous=True)
        p.requested(0);self.assertEqual(p.requested(6),'high')
        p.decoded('barcode',7);self.assertEqual(p.requested(11.999),'low')
        self.assertEqual(p.requested(12),'high')

    def test_driver_negotiated_low_mode_is_used_but_hd_fallback_is_rejected(self):
        self.assertTrue(capture_dimensions_acceptable('low',(640,480),(640,360)))
        self.assertTrue(capture_dimensions_acceptable('low',(640,480),(640,480)))
        self.assertTrue(capture_dimensions_acceptable('high',(1280,720),(1920,1080)))
        self.assertFalse(capture_dimensions_acceptable('high',(1280,720),(640,360)))
        self.assertFalse(capture_dimensions_acceptable('low',(640,480),(0,0)))

    def test_scan_low_mode_requests_the_camera_native_640x480_mjpeg_rate(self):
        from pathlib import Path
        import re
        config = Path(__file__).resolve().parents[1] / 'config/scan_camera.yaml'
        values = dict(re.findall(r'^(low_\w+):\s*([^#\s]+)', config.read_text(encoding='utf-8'), re.M))
        self.assertEqual((values['low_width'], values['low_height'], values['low_fps'], values['low_pixel_format']),
                         ('640', '480', '30.0', 'mjpeg'))

    def test_timeout_validation(self):
        for value in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):ScanCameraPolicy(timeout=value)
        with self.assertRaises(ValueError):ScanCameraPolicy(mode='typo')

if __name__=='__main__':unittest.main()
