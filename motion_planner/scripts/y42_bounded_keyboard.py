#!/usr/bin/env python3
"""Bounded lateral keyboard control for Y42 chassis tests."""
import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist


def command_from_keys(keys):
    if any(key in keys for key in (ord(' '), ord('k'), ord('K'), 3)):
        return 0
    for key in reversed(keys):
        if key == ord('J'):
            return 1
        if key == ord('L'):
            return -1
    return 0


def main():
    rospy.init_node('y42_bounded_keyboard')
    speed = float(rospy.get_param('~speed', 0.10))
    timeout = float(rospy.get_param('~key_timeout', 0.6))
    if not 0.02 <= speed <= 0.20 or not 0.2 <= timeout <= 0.8:
        raise ValueError('speed must be 0.02..0.20m/s and key_timeout 0.2..0.8s')
    publisher = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=1)
    fd = sys.stdin.fileno()
    old_terminal = termios.tcgetattr(fd)
    direction = 0
    last_key = 0.0
    try:
        tty.setraw(fd)
        print('Shift+J left | Shift+L right | Space/K stop | Ctrl+C exit', flush=True)
        while not rospy.is_shutdown():
            if select.select([fd], [], [], 1.0 / 30.0)[0]:
                keys = os.read(fd, 4096)
                direction = command_from_keys(keys)
                last_key = time.monotonic()
                if direction == 0:
                    termios.tcflush(fd, termios.TCIFLUSH)
                if 3 in keys:
                    break
            if time.monotonic() - last_key > timeout:
                direction = 0
            twist = Twist()
            twist.linear.y = direction * speed
            publisher.publish(twist)
    finally:
        for _ in range(3):
            publisher.publish(Twist())
            time.sleep(0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_terminal)


if __name__ == '__main__':
    main()
