"""Build an isolated copy of the real CAN node for the authorized stand test.

Only the physical-start gate changes. Feedback, ACK, fault and estop guards
remain active. The production source and executable are not overwritten.
"""
from pathlib import Path
import difflib
import shlex
import subprocess

ROOT = Path('/home/fire/ros_ws/src')
OUT = Path('/tmp/chassis_medical_route_test')
OUT.mkdir(exist_ok=True)
header = ROOT / 'can_motor_interface/include/can_motor_interface/can_interface.h'
source = ROOT / 'can_motor_interface/src/can_interface_node.cpp'
h = header.read_text().replace('  bool have_command_;',
                               '  bool have_command_;\n  bool test_immediate_motion_ = false;')
s = source.read_text()
needle = '  pnh_.getParam("motor_indices", wheel_map_.indices);'
s = s.replace(needle, '''  pnh_.param("test_immediate_motion", test_immediate_motion_, false);
  if (test_immediate_motion_ && max_rpm_ > 20)
    throw std::invalid_argument("Stand test requires max_rpm <= 20");
  if (test_immediate_motion_)
    ROS_WARN("STAND TEST: immediate standard speed commands; physical start bypassed, all fault guards retained");
''' + needle)
s = s.replace('if (start_state_.canSendCachedMotion()) {',
              'if (!test_immediate_motion_ && start_state_.canSendCachedMotion()) {')
s = s.replace('motion_ack_expired && !motion_.latched() && start_state_.state() == MotionStartState::RUNNING',
              'motion_ack_expired && !motion_.latched() && (test_immediate_motion_ || start_state_.state() == MotionStartState::RUNNING)')
s = s.replace('const std::string state_name = motionStartStateName(start_state_.state());',
              'const std::string state_name = test_immediate_motion_ && start_state_.state() == MotionStartState::IDLE ? "STAND_TEST_IMMEDIATE" : motionStartStateName(start_state_.state());')
inc = OUT / 'include/can_motor_interface'
inc.mkdir(parents=True, exist_ok=True)
(inc / 'can_interface.h').write_text(h)
cpp = OUT / 'can_interface_test.cpp'
cpp.write_text(s)
patch = ''.join(difflib.unified_diff(source.read_text().splitlines(True), s.splitlines(True),
                                     fromfile=str(source), tofile=str(cpp)))
patch += ''.join(difflib.unified_diff(header.read_text().splitlines(True), h.splitlines(True),
                                      fromfile=str(header), tofile=str(inc / 'can_interface.h')))
(OUT / 'test_only_changes.patch').write_text(patch)
link = shlex.split((ROOT.parent / 'build/can_motor_interface/CMakeFiles/can_interface_node.dir/link.txt').read_text())
libs = link[link.index('-Wl,-rpath,/opt/ros/noetic/lib:'):]
subprocess.run(['c++', '-std=c++14', '-Wall', '-Wextra', '-pthread',
                '-I' + str(OUT / 'include'), '-I' + str(ROOT / 'can_motor_interface/include'),
                '-I/opt/ros/noetic/include', '-I/usr/include/xmlrpcpp', str(cpp),
                '-o', str(OUT / 'can_interface_test'), *libs], check=True)
wrapper = OUT / 'run_test_can'
wrapper.write_text('#!/bin/sh\nshift\nexec /tmp/chassis_medical_route_test/can_interface_test "$@"\n')
wrapper.chmod(0o755)
print(OUT / 'can_interface_test')
