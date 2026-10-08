#!/usr/bin/env python3
"""TTY-preserving keyboard entry point; edit the tested defaults below."""
import os
import sys

# Optional ROS CLI overrides supplied to this wrapper are appended last.
DEFAULT_ARGS = [
    'cmd_vel:=/cmd_vel_teleop',
    '_speed:=0.10',
    '_turn:=0.30',
    '_repeat_rate:=10.0',
    '_key_timeout:=0.6',
]


def main():
    # Do not start a new terminal or a roslaunch child: retain the user's stdin.
    os.execvp('rosrun', ['rosrun', 'teleop_twist_keyboard',
                        'teleop_twist_keyboard.py'] + DEFAULT_ARGS + sys.argv[1:])


if __name__ == '__main__':
    main()
