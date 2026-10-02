import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('direct',Path(__file__).resolve().parents[1]/'scripts/y42_direct_node.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class TestProtocol(unittest.TestCase):
    def test_speed(self):
        self.assertEqual(m.speed_payload(10),bytes.fromhex('F6 00 00 32 00 64 00 6B'))
        self.assertEqual(m.speed_payload(-10),bytes.fromhex('F6 01 00 32 00 64 00 6B'))
        self.assertEqual(m.speed_payload(999)[4:6],bytes.fromhex('01 2C'))
        self.assertEqual(m.speed_payload(0)[4:6],b'\0\0')
        for v in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):m.speed_payload(v)
    def test_parse(self):
        cid=m.CAN_EFF_FLAG|0x100
        self.assertEqual(m.parse_reply(cid,bytes.fromhex('35 01 00 64 6B')),(1,'speed',-10.0))
        self.assertEqual(m.parse_reply(cid,bytes.fromhex('3A 03 6B')),(1,'status',3))
        for bad in (0x100,cid|m.CAN_RTR_FLAG,cid|m.CAN_ERR_FLAG,cid|1,m.CAN_EFF_FLAG|0x500,m.CAN_EFF_FLAG|0x600):
            self.assertIsNone(m.parse_reply(bad,bytes.fromhex('3A 03 6B')))
        for data in ('35 6B','F6 02 6B','35 01 00 64 00','3A 6B','35 E2 6B'):
            self.assertIsNone(m.parse_reply(cid,bytes.fromhex(data)))
    def test_each_motor_freshness(self):
        f=m.Feedback([1,2,3,4]);self.assertTrue(f.problem(0,1))
        for a in (1,2,3,4):
            f.update((a,'speed',0),10);f.update((a,'status',3),10)
        self.assertFalse(f.problem(10.5,1))
        f.update((1,'speed',0),11);f.update((1,'status',3),11)
        self.assertIn('address 2',f.problem(11.1,1))
        f.update((2,'status',0),10);self.assertIn('disabled',f.problem(10.5,1))
        f.update((2,'status',0x0D),10);self.assertIn('fault',f.problem(10.5,1))
    def test_power_history_is_not_live_fault(self):
        f=m.Feedback([1,2,3,4])
        for flags in (0x81, 0x83):
            for a in f.ids:
                f.update((a,'speed',0),10)
                f.update((a,'status',flags),10)
            self.assertEqual(f.problem(10.5,1), '')
            self.assertEqual(f.status[1], flags)  # preserve raw diagnostic bits
        for flags in (0x85,0x89,0x8D):
            f.update((1,'status',flags),10)
            self.assertIn('fault',f.problem(10.5,1))
        f.update((1,'status',0x80),10)
        self.assertIn('disabled',f.problem(10.5,1))
        f.update((1,'status',0x83),10)
        self.assertIn('stale',f.problem(12,1))

    def test_frame(self):
        frame=m.FRAME.pack(m.CAN_EFF_FLAG|0x100,4,m.STOP.ljust(8,b'\0'))
        self.assertEqual(len(frame),16)
        self.assertEqual(m.FRAME.unpack(frame)[0],0x80000100)

class TestStallGrace(unittest.TestCase):
    def feedback(self, grace=0.5):
        f=m.Feedback([1,2,3,4],grace)
        for a in f.ids:
            f.update((a,'speed',0),10)
            f.update((a,'status',0x83),10)
        return f
    def test_warning_window_and_strict_arm(self):
        f=self.feedback();f.update((2,'status',0x85),10)
        self.assertFalse(f.problem(10.49,1))
        self.assertTrue(f.problem(10.01,1,strict_warning=True))
        self.assertIn('continuous',f.problem(10.51,1))
    def test_repeated_warning_does_not_restart_timer(self):
        f=self.feedback()
        for t in (10,10.1,10.2,10.3,10.4,10.5):f.update((2,'status',0x85),t)
        self.assertTrue(f.problem(10.51,1))
    def test_clear_resets_only_affected_motor(self):
        f=self.feedback();f.update((1,'status',0x85),10);f.update((2,'status',0x85),10)
        f.update((1,'status',0x83),10.3);f.update((1,'status',0x85),10.4)
        self.assertIn('address 2',f.problem(10.51,1))
    def test_protection_immediate_even_with_grace(self):
        for flag in (0x89,0x8D):
            f=self.feedback(1.0);f.update((2,'status',flag),10)
            self.assertIn('PROTECTION',f.problem(10,1))
    def test_disabled_and_timeout_still_immediate(self):
        f=self.feedback(1.0);f.update((2,'status',0x84),10)
        self.assertIn('disabled',f.problem(10,1))
        f=self.feedback(1.0);f.update((2,'status',0x85),10)
        self.assertIn('stale',f.problem(11.01,1))
    def test_zero_grace_and_validation(self):
        f=self.feedback(0);f.update((2,'status',0x85),10);self.assertTrue(f.problem(10,1))
        for v in (-1,1.01,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.feedback(v)
    def test_status_gap_breaks_continuity(self):
        f=self.feedback();f.update((2,'status',0x85),10)
        for a in f.ids:
            f.update((a,'speed',0),12)
            f.update((a,'status',0x85 if a==2 else 0x83),12)
        self.assertFalse(f.problem(12.1,1))

class TestArmGuard(unittest.TestCase):
    def node(self):
        import threading
        from types import SimpleNamespace
        n=m.DirectNode.__new__(m.DirectNode)
        n.lock=threading.RLock();n.ids=[1,2,3,4];n.feedback=m.Feedback(n.ids)
        n.timeout=1;n.cmd_timeout=0.5;n.external_stop=False;n.fault='';n.armed=False
        n.Response=lambda success,message:SimpleNamespace(success=success,message=message)
        n.target=[0]*4;n.cmd_time=m.time.monotonic()
        for a in n.ids:
            n.feedback.update((a,'speed',0),n.cmd_time)
            n.feedback.update((a,'status',3),n.cmd_time)
        return n
    def test_arm_zero_and_fresh_only(self):
        n=self.node();self.assertTrue(n.arm(None).success)
        n=self.node();n.target[0]=1;self.assertFalse(n.arm(None).success)
        n=self.node();n.feedback.seen.pop((4,'status'));self.assertFalse(n.arm(None).success)
        n=self.node();n.cmd_time-=2;self.assertFalse(n.arm(None).success)
        n=self.node();n.feedback.speed[2]=10;self.assertFalse(n.arm(None).success)
    def test_arm_rejects_transient_warning(self):
        n=self.node();n.feedback.update((2,'status',0x85),m.time.monotonic())
        self.assertFalse(n.arm(None).success)

    def test_arm_with_83_feedback(self):
        n=self.node()
        for a in n.ids:
            n.feedback.update((a,'status',0x83),m.time.monotonic())
        self.assertTrue(n.arm(None).success)

    def test_no_clear_hard_fault_or_external_estop(self):
        n=self.node();n.external_stop=True;self.assertFalse(n.arm(None).success)
        n=self.node();n.fault='bus fault';self.assertFalse(n.arm(None).success)
    def test_stop_only_chassis(self):
        n=self.node();sent=[];n.send=lambda addr,data:sent.append((addr,data))
        n.stop_all();self.assertEqual(sent,[(a,m.STOP) for a in (1,2,3,4)])
        n.armed=True;n.target=[10]*4;n.trip('test');self.assertFalse(n.armed);self.assertEqual(n.target,[0]*4)
if __name__=='__main__':unittest.main()

