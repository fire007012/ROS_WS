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
        n.inactive_wheel_mode='stop'
        n.startup_zero_duration=0;n.zero_init_until=0
        n.ros=SimpleNamespace(loginfo=lambda *args:None)
        n.tx_schedule=m.TxSchedule(n.ids)
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
        n.stop_all();self.assertFalse(sent)  # asynchronous, no burst
        jobs=[]
        for i in range(8):jobs.append(n.tx_schedule.next(i*0.01,False))
        self.assertEqual(jobs,[(a,'stop') for a in (1,2,3,4,1,2,3,4)])
        n.armed=True;n.target=[10]*4;n.trip('test');self.assertFalse(n.armed);self.assertEqual(n.target,[0]*4)

class TestTxSchedule(unittest.TestCase):
    def test_one_per_slot_no_catchup(self):
        q=m.TxSchedule([1,2,3,4]);self.assertIsNotNone(q.next(10,True))
        self.assertIsNone(q.next(10,True));self.assertIsNone(q.next(10.003,True))
        self.assertIsNotNone(q.next(50,True));self.assertIsNone(q.next(50,True))
    def test_queries_and_controls_distributed(self):
        q=m.TxSchedule([1,2,3,4]);counts={};times=[]
        for i in range(1000):
            job=q.next(i*0.002,True)
            if job:counts[job]=counts.get(job,0)+1;times.append(i*0.002)
        for a in (1,2,3,4):
            self.assertGreater(counts[a,'control'],20)
            for func in (0x35,0x3A):self.assertGreater(counts[a,func],10)
        self.assertTrue(all(b-a>=0.004-1e-8 for a,b in zip(times,times[1:])))
    def test_stop_priority_and_coalescing(self):
        q=m.TxSchedule([1,2,3,4]);q.request_stop();q.request_stop()
        self.assertEqual(len(q.pending_stops),8)
        self.assertEqual(q.next(0,True),(1,'stop'))
    def test_failure_stops_bounded_no_query_or_motion(self):
        q=m.TxSchedule([1,2,3,4]);q.failed(0)
        self.assertIsNone(q.next(0.01,True))
        jobs=[]
        for i in range(1,25):
            job=q.next(i*0.1,True)
            if job:jobs.append(job);q.failed(i*0.1)
        self.assertEqual(jobs,[(a,'stop') for a in (1,2,3,4,1,2,3,4)])
        q.request_stop();self.assertIsNone(q.next(100,True))
    def test_latest_command_not_saved_in_scheduler(self):
        n=TestArmGuard().node();n.signs=[1]*4;n.accel=50;n.limit=30;n.armed=True
        sent=[];n.send=lambda addr,data:sent.append((addr,data))
        n.target=[10]*4;n.transmit_one(10)
        n.target=[0]*4;n.transmit_one(10.02)
        self.assertEqual(sent[0],(1,m.speed_payload(10)))
        self.assertEqual(sent[1],(2,m.STOP))
    def test_inactive_wheel_zero_only_while_other_wheels_move(self):
        n=TestArmGuard().node();n.signs=[1]*4;n.accel=100;n.limit=90
        n.zero_init_until=0;n.armed=True;n.inactive_wheel_mode='speed_zero'
        n.target=[0,10,0,-10]
        sent=[];n.send=lambda addr,data:sent.append((addr,data))
        for i in range(4):n.transmit_one(10+i*.02)
        self.assertEqual(sent[0],(1,m.speed_payload(0,100,90)))
        self.assertEqual(sent[2],(3,m.speed_payload(0,100,90)))
        n.target=[0]*4
        for i in range(4):n.transmit_one(11+i*.02)
        self.assertTrue(all(data==m.STOP for addr,data in sent[4:8]))
        n.trip('test fault')
        self.assertFalse(n.armed)
        self.assertEqual(n.tx_schedule.next(12,False),(1,'stop'))
    def test_real_send_enobufs_latches(self):
        import errno
        from types import SimpleNamespace
        n=TestArmGuard().node();n.armed=True
        class Broken:
            def send(self, frame):raise OSError(errno.ENOBUFS,'No buffer space available')
        n.sock=Broken();n.ros=SimpleNamespace(logerr_throttle=lambda *args:None)
        self.assertFalse(n.send(1,m.STOP));self.assertFalse(n.armed)
        self.assertIn('backpressure',n.fault);self.assertTrue(n.tx_schedule.tx_fault)
        self.assertFalse(n.arm(None).success)
        for _ in range(4):n.send(1,m.STOP)
        self.assertEqual(len(n.tx_schedule.pending_stops),8)  # not replenished
class TestSTM32BridgeEvents(unittest.TestCase):
    def test_standard_stop_events(self):
        p=bytes([6,255,0x81,0,0,0,0,0])
        self.assertIn('reason=0x81',m.parse_stm32_stop(0x101,p))
        self.assertTrue(m.parse_stm32_stop(0x101,bytes([6,255,0x80,0,0,0,0,0])))
        self.assertTrue(m.parse_stm32_stop(0x101,bytes([6,0,0x89,0,0,0,0,0])))
        self.assertTrue(m.parse_stm32_stop(0x112,bytes([2,1,1,0,0,0,0,0])))
        self.assertFalse(m.parse_stm32_stop(0x112,bytes([1,1,1,0,0,0,0,0])))
        self.assertFalse(m.parse_stm32_stop(0x101,bytes([1,0,0,0,0,0,0,0])))
        for flag in (m.CAN_EFF_FLAG,m.CAN_RTR_FLAG,m.CAN_ERR_FLAG):
            self.assertFalse(m.parse_stm32_stop(0x101|flag,p))
        for size in range(8):
            self.assertFalse(m.parse_stm32_stop(0x101,p[:size]))
        self.assertFalse(m.parse_stm32_stop(0x101,p+b'\0'))

    def test_bridge_event_latches_and_blocks_rearm(self):
        from types import SimpleNamespace
        n=TestArmGuard().node();n.armed=True;n.monitor_stm32_events=True
        messages=[]
        n.Bool=lambda **kw:SimpleNamespace(**kw)
        n.emergency_pub=SimpleNamespace(publish=messages.append)
        n.receive_frame(0x101,bytes([6,255,0x81,0,0,0,0,0]),m.time.monotonic())
        self.assertFalse(n.armed);self.assertTrue(n.external_stop)
        self.assertEqual(n.target,[0]*4)
        self.assertTrue(messages[0].data)
        self.assertFalse(n.arm(None).success)
        self.assertEqual(len(n.tx_schedule.pending_stops),8)

    def test_direct_mode_ignores_stm32_standard_events(self):
        n=TestArmGuard().node();n.armed=True;n.monitor_stm32_events=False
        n.receive_frame(0x101,bytes([6,255,0x81,0,0,0,0,0]),m.time.monotonic())
        self.assertTrue(n.armed);self.assertFalse(n.external_stop)

    def test_native_feedback_still_works_in_bridge_mode(self):
        n=TestArmGuard().node();n.monitor_stm32_events=True
        n.receive_frame(m.CAN_EFF_FLAG|0x200,bytes.fromhex('35 00 00 64 6B'),10)
        self.assertEqual(n.feedback.speed[2],10)
        n.receive_frame(m.CAN_EFF_FLAG|0x200,bytes.fromhex('3A 83 6B'),10)
        self.assertEqual(n.feedback.status[2],0x83)

if __name__=='__main__':unittest.main()
