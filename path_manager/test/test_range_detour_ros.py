#!/usr/bin/env python3
"""Closed-loop four-beam detours through the real tracker, mux and brake.
Finite returns come from ray casts against room walls/boxes. No CAN is opened.
"""
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_manager.msg import PathPoint
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, String, UInt32
from std_srvs.srv import Trigger, TriggerResponse


def ray_box(x, y, dx, dy, box):
    near, far = 0.0, float('inf')
    for origin, direction, lower, upper in [(x, dx, box[0], box[1]), (y, dy, box[2], box[3])]:
        if abs(direction) < 1e-10:
            if origin < lower or origin > upper:
                return float('inf')
        else:
            a, b = (lower-origin)/direction, (upper-origin)/direction
            near, far = max(near, min(a, b)), min(far, max(a, b))
    return near if far >= near and far >= 0 else float('inf')


class RangeDetourRos(unittest.TestCase):
    def test_closed_loop(self):
        self.lock = threading.RLock()
        self.running = True
        self.pose = [0., 0., 0.]
        self.velocity = Twist()
        self.clock_time = 1.0
        self.advance = True
        self.revision = 0
        self.point = None
        self.angle = 0.0
        self.obstacles = []
        self.fault = None
        self.state = ''
        self.states = set()
        self.finished = False
        self.collided = False
        self.next_calls = 0
        self.cross_track = []
        self.send_odom = True
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.measured = rospy.Publisher('/base_velocity', Twist, queue_size=1)
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.path = rospy.Publisher('/path_points', PathPoint, queue_size=1)
        self.revisions = rospy.Publisher('/path_manager/path_revision', UInt32, queue_size=1)
        self.range_pubs = {name: rospy.Publisher('/'+name+'/range', Range, queue_size=1)
                           for name in ['front', 'left', 'right', 'rear']}
        self.cancel = rospy.Publisher('/mission/cancel', Empty, queue_size=1)
        self.reset = rospy.Publisher('/emergency_stop/reset', Bool, queue_size=1)
        self.enabled = rospy.Publisher('/mission/actions_enabled', Bool, queue_size=1)
        self.estop = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        subs = [rospy.Subscriber('/cmd_vel_muxed', Twist, lambda m: self.store('velocity', m)),
                rospy.Subscriber('/path_finished', Bool, lambda m: self.store('finished', m.data)),
                rospy.Subscriber('/path_tracker/avoidance_state', String, self.on_state)]
        service = rospy.Service('/next_point', Trigger, self.on_next)
        thread = threading.Thread(target=self.simulate, daemon=True)
        thread.start()
        try:
            self.wait(lambda: self.path.get_num_connections() and self.cancel.get_num_connections() >= 2)
            for angle, right in [(0., False), (math.pi/2, True)]:
                self.new_route(angle=angle, right=right)
                self.wait(lambda: self.finished, timeout=28)
                self.assertFalse(self.collided, 'chassis intersected an obstacle')
                self.assertEqual(self.next_calls, 0, 'detour must not advance/skip the waypoint')
                self.assertTrue({'SIDESTEP', 'BYPASS', 'EDGE_CLEAR', 'RETURN'} <= self.states, self.states)
                self.assertGreater(max(abs(y) for y in self.cross_track), .6)
                if right:
                    self.assertLess(min(self.cross_track), -.6)
                else:
                    self.assertGreater(max(self.cross_track), .6)
                x, y = self.local_pose()
                self.assertAlmostEqual(x, 2.7, delta=.05)
                self.assertAlmostEqual(y, 0., delta=.05)

            # NaN and missing rear samples stop lateral detours with frozen ROS time.
            self.new_route()
            self.wait(lambda: self.state == 'SIDESTEP' and abs(self.velocity.linear.y) > .05)
            with self.lock:
                self.fault = 'nan_rear'
                self.advance = False
            self.wait(lambda: self.state == 'WAIT_RANGES' and self.stopped())
            self.assertFalse(self.finished)
            with self.lock:
                self.fault = None
                self.advance = True
            self.wait(lambda: self.state == 'SIDESTEP' and abs(self.velocity.linear.y) > .05)
            with self.lock:
                self.fault = 'missing_rear'
                self.advance = False
            self.wait(lambda: self.state == 'WAIT_RANGES' and self.stopped())
            self.assertFalse(self.finished)
            with self.lock:
                self.fault = None
                self.advance = True
            self.wait(lambda: abs(self.velocity.linear.y) > .05)
            # Cancel/estop cannot resume an old detour on reset alone.
            self.cancel.publish(Empty())
            self.wait(lambda: self.state == 'CANCELLED' and self.stopped())
            self.reset.publish(Bool(True))
            self.hold(lambda: self.stopped() and not self.finished, .35)

            self.new_route(blocked=True)
            self.wait(lambda: self.state == 'NO_SIDE_SPACE')
            self.assertTrue(self.stopped())
            self.wait(lambda: self.state == 'FAILED_STALLED', timeout=4)
            with self.lock:
                self.obstacles = []
            self.hold(lambda: self.stopped() and not self.finished, .3)

            # An occupied endpoint is a failure, never an implicit /next_point.
            self.new_route(goal=1.5)
            self.wait(lambda: self.state == 'FAILED_GOAL_BLOCKED', timeout=20)
            self.assertFalse(self.finished)
            self.assertEqual(self.next_calls, 0)
            self.assertFalse(self.collided)

            self.new_route()
            self.wait(lambda: self.state == 'SIDESTEP' and abs(self.velocity.linear.y) > .05)
            self.estop.publish(Bool(True))
            self.wait(lambda: self.state == 'CANCELLED' and self.stopped())
            self.reset.publish(Bool(True))
            self.hold(lambda: self.stopped() and not self.finished, .3)
        finally:
            self.running = False
            thread.join(timeout=1)
            for sub in subs:
                sub.unregister()
            service.shutdown()

    def new_route(self, angle=0., right=False, blocked=False, goal=2.7):
        self.cancel.publish(Empty())
        self.wait(lambda: self.state == 'CANCELLED' and self.stopped())
        self.reset.publish(Bool(True))
        time.sleep(.08)
        with self.lock:
            self.pose = [0., 0., angle]
            self.velocity = Twist()
            self.angle = angle
            self.obstacles = [(1.2, 1.7, -.25, .25)]
            if right or blocked:
                self.obstacles.append((-.48, 3.3, .65, 1.8))
            if blocked:
                self.obstacles.append((-.48, 3.3, -1.8, -.65))
            self.fault = None
            self.advance = self.send_odom = True
            self.states.clear()
            self.finished = self.collided = False
            self.cross_track.clear()
            self.point = None
        self.enabled.publish(Bool(True))
        time.sleep(.08)
        with self.lock:
            self.revision += 1
            self.point = PathPoint(x=goal*math.cos(angle), y=goal*math.sin(angle),
                                   tolerance=.04, frame_id='odom', has_next=False)
        self.wait(lambda: self.state != 'CANCELLED')

    def simulate(self):
        walls = [(-.50, -.48, -1.82, 1.82), (3.3, 3.32, -1.82, 1.82),
                 (-.50, 3.32, 1.8, 1.82), (-.50, 3.32, -1.82, -1.8)]
        beams = [(1., 0.), (0., 1.), (0., -1.), (-1., 0.)]
        while self.running and not rospy.is_shutdown():
            with self.lock:
                dt = .025 if self.advance else 0.
                self.clock_time += dt
                vx, vy, omega = self.velocity.linear.x, self.velocity.linear.y, self.velocity.angular.z
                yaw = self.pose[2]
                self.pose[0] += (vx*math.cos(yaw)-vy*math.sin(yaw))*dt
                self.pose[1] += (vx*math.sin(yaw)+vy*math.cos(yaw))*dt
                self.pose[2] += omega*dt
                x, y = self.local_pose()
                self.cross_track.append(y)
                radius = .20 * (abs(math.cos(yaw-self.angle)) + abs(math.sin(yaw-self.angle)))
                for box in self.obstacles:
                    if box[0]-radius < x < box[1]+radius and box[2]-radius < y < box[3]+radius:
                        self.collided = True
                stamp = rospy.Time.from_sec(self.clock_time)
                self.clock.publish(Clock(stamp))
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
                odom.pose.pose.orientation.z = math.sin(self.pose[2]/2)
                odom.pose.pose.orientation.w = math.cos(self.pose[2]/2)
                odom.twist.twist = self.velocity
                if self.send_odom:
                    self.odom.publish(odom)
                self.measured.publish(self.velocity)
                for name, (bx, by) in zip(self.range_pubs, beams):
                    if name == 'rear' and self.fault == 'missing_rear':
                        continue
                    local_angle = yaw-self.angle
                    dx = math.cos(local_angle)*bx-math.sin(local_angle)*by
                    dy = math.sin(local_angle)*bx+math.cos(local_angle)*by
                    ox, oy = x+.20*dx, y+.20*dy
                    distances = []
                    for spread in [-.235, -.1175, 0., .1175, .235]:
                        rx = math.cos(spread)*dx-math.sin(spread)*dy
                        ry = math.sin(spread)*dx+math.cos(spread)*dy
                        distances.extend(ray_box(ox, oy, rx, ry, b) for b in walls+self.obstacles)
                    distance = min(distances)
                    msg = Range()
                    msg.header.stamp = stamp
                    msg.min_range, msg.max_range = .05, 3.
                    msg.field_of_view = .47
                    msg.range = distance if .05 <= distance <= 3. else float('nan')
                    if name == 'rear' and self.fault == 'nan_rear':
                        msg.range = float('nan')
                    self.range_pubs[name].publish(msg)
                self.revisions.publish(UInt32(self.revision))
                if self.point is not None:
                    self.path.publish(self.point)
            time.sleep(.01)

    def local_pose(self):
        x, y = self.pose[:2]
        return math.cos(self.angle)*x+math.sin(self.angle)*y, -math.sin(self.angle)*x+math.cos(self.angle)*y

    def stopped(self):
        return max(abs(self.velocity.linear.x), abs(self.velocity.linear.y), abs(self.velocity.angular.z)) < .001

    def store(self, name, value):
        with self.lock:
            setattr(self, name, value)

    def on_state(self, msg):
        with self.lock:
            self.state = msg.data
            self.states.add(msg.data)

    def on_next(self, req):
        with self.lock:
            self.next_calls += 1
        return TriggerResponse(True, 'unexpected advance')

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('timeout: state=%s pose=%s phases=%s' % (self.state, self.pose, self.states))

    def hold(self, predicate, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            with self.lock:
                self.assertTrue(predicate())
            time.sleep(.01)


if __name__ == '__main__':
    rospy.init_node('test_range_detour')
    rostest.rosrun('path_manager', 'range_detour', RangeDetourRos)
