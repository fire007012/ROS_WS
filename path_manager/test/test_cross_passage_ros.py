#!/usr/bin/env python3
"""Exercise a mid-passage blockage through the real route and control chain."""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_manager.srv import SelectPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


class CrossPassage(unittest.TestCase):
    def test_mid_passage_fallback(self):
        self.lock = threading.RLock()
        self.running = True
        self.pose = [4., 2.2, 0.]
        self.velocity = Twist()
        self.sim_time = 1.
        self.active = self.state = ''
        self.finished = False
        self.block = False
        self.bad_rear = False
        self.visited = set()
        self.block_pose = None
        self.direction = 1
        self.points = {'P3L': (3., 2.), 'P2L': (2., 1.1), 'P1': (1.3, 0.),
                       'P2R': (2., -1.1), 'P3R': (3., -2.)}
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.measured = rospy.Publisher('/base_velocity', Twist, queue_size=1)
        self.ranges = [rospy.Publisher('/'+name+'/range', Range, queue_size=1)
                       for name in ['front', 'left', 'right', 'rear']]
        self.cancel = rospy.Publisher('/mission/cancel', Empty, queue_size=1)
        self.enabled = rospy.Publisher('/mission/actions_enabled', Bool, queue_size=1, latch=True)
        subscribers = [
            rospy.Subscriber('/cmd_vel_muxed', Twist, lambda m: self.store('velocity', m)),
            rospy.Subscriber('/path_manager/active_path', String, lambda m: self.store('active', m.data)),
            rospy.Subscriber('/path_tracker/avoidance_state', String, lambda m: self.store('state', m.data)),
            rospy.Subscriber('/path_finished', Bool, lambda m: self.store('finished', m.data))]
        thread = threading.Thread(target=self.simulate, daemon=True)
        thread.start()
        try:
            rospy.wait_for_service('/select_path', timeout=10)
            self.select = rospy.ServiceProxy('/select_path', SelectPath)
            reroute = rospy.ServiceProxy('/path_manager/reroute_blocked_path', Trigger)
            self.wait(lambda: self.odom.get_num_connections() and
                      all(pub.get_num_connections() >= 2 for pub in self.ranges))
            self.enabled.publish(Bool(True))
            for direction, name in [(1, 'bed1_to_bed3'), (-1, 'bed3_to_bed1')]:
                self.begin(direction, name)
                # Before entering the protected segment, a blockage cannot
                # select a fallback that starts on the other side of the robot.
                self.assertFalse(reroute().success)
                self.wait(lambda: self.block_pose is not None, timeout=12)
                self.assertLess(abs(self.block_pose[1]), 1.05, 'must block mid-passage')
                # Keep the rear invalid throughout retreat and passage via P1.
                # The valid front obstacle must still select the alternate route.
                self.wait(lambda: self.active == name+'_via_p1')
                self.assertFalse(self.finished)
                self.assertFalse(reroute().success, 'fallback must not recursively reroute')
                self.wait(lambda: self.finished, timeout=24)
                self.assertEqual(self.visited, set(self.points), 'must retreat and physically pass P1')
                self.assertAlmostEqual(self.pose[0], 4., delta=.04)
                self.assertAlmostEqual(self.pose[1], -direction*2.2, delta=.04)

            # Cancel while retreating: no path completion or resume on stale goals.
            self.begin(1, 'bed1_to_bed3')
            self.wait(lambda: self.block_pose is not None, timeout=12)
            with self.lock:
                self.bad_rear = False
            self.wait(lambda: self.active.endswith('_via_p1'))
            self.cancel.publish(Empty())
            self.wait(lambda: self.state == 'CANCELLED' and self.stopped())
            self.hold(lambda: self.stopped() and not self.finished, .4)

            # With a clear passage, keep the direct route and never visit P1.
            self.begin(1, 'bed1_to_bed3', blocked=False)
            self.wait(lambda: self.finished, timeout=18)
            self.assertEqual(self.active, 'bed1_to_bed3')
            self.assertNotIn('P1', self.visited)
        finally:
            self.running = False
            thread.join(timeout=2)
            for sub in subscribers:
                sub.unregister()

    def begin(self, direction, name, blocked=True):
        self.cancel.publish(Empty())
        time.sleep(.1)  # Drain this cancellation even if the previous state was already CANCELLED.
        self.wait(lambda: self.state == 'CANCELLED' and self.stopped())
        with self.lock:
            self.pose = [4., direction*2.2, 0.]
            self.velocity = Twist()
            self.direction = direction
            self.block = blocked
            self.bad_rear = False
            self.block_pose = None
            self.visited.clear()
            self.finished = False
        self.enabled.publish(Bool(True))
        time.sleep(.1)
        self.assertTrue(self.select(name).success)
        self.wait(lambda: self.active == name and self.state != 'CANCELLED')

    def simulate(self):
        while self.running and not rospy.is_shutdown():
            with self.lock:
                dt = .04
                self.sim_time += dt
                vx, vy, wz = self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z
                x, y, yaw = self.pose
                self.pose = [x+(vx*math.cos(yaw)-vy*math.sin(yaw))*dt,
                             y+(vx*math.sin(yaw)+vy*math.cos(yaw))*dt, yaw+wz*dt]
                x, y, yaw = self.pose
                if self.active.endswith('_via_p1'):
                    for name, (px, py) in self.points.items():
                        if math.hypot(x-px, y-py) < .07:
                            self.visited.add(name)
                # Inject a front obstruction only after travelling >1 m into
                # the cross passage; leave it present throughout the fallback.
                blocked = (self.block and abs(x-3.) < .10 and abs(y) < 1.0 and
                           -self.direction*math.sin(yaw) > .98)
                if blocked and self.block_pose is None:
                    self.block_pose = list(self.pose)
                    self.bad_rear = True
                stamp = rospy.Time.from_sec(self.sim_time)
                self.clock.publish(Clock(stamp))
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
                odom.pose.pose.orientation.z = math.sin(yaw/2.)
                odom.pose.pose.orientation.w = math.cos(yaw/2.)
                odom.twist.twist = self.velocity
                self.odom.publish(odom)
                self.measured.publish(self.velocity)
                for i, pub in enumerate(self.ranges):
                    msg = Range()
                    msg.header.stamp = stamp
                    msg.min_range, msg.max_range = .05, 3.
                    msg.range = .65 if blocked and i == 0 else float('nan')
                    if i == 3 and self.bad_rear:
                        msg.range = float('nan')
                    pub.publish(msg)
            time.sleep(.005)

    def store(self, name, value):
        with self.lock:
            setattr(self, name, value)

    def stopped(self):
        return max(abs(self.velocity.linear.x), abs(self.velocity.linear.y),
                   abs(self.velocity.angular.z)) < .001

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timeout: active=%s state=%s pose=%s visited=%s' %
                  (self.active, self.state, self.pose, self.visited))

    def hold(self, predicate, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            with self.lock:
                self.assertTrue(predicate())
            time.sleep(.01)


if __name__ == '__main__':
    rospy.init_node('test_cross_passage')
    rostest.rosrun('path_manager', 'cross_passage', CrossPassage)
