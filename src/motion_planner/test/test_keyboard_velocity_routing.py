"""Offline regression: compile the production mux with tiny ROS API stubs.

This verifies local mux logic/topic wiring, NOT a real ROS graph or hardware.
Run: python3 src/motion_planner/test/test_keyboard_velocity_routing.py
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


class KeyboardVelocityRouting(unittest.TestCase):
    def test_downstream_topics(self):
        planner = (ROOT / 'motion_planner/src/planner_node.cpp').read_text(encoding='utf-8')
        safety = (ROOT / 'robot_navigation/src/distance_safety_node.cpp').read_text(encoding='utf-8')
        self.assertIn('nh_.subscribe("/cmd_vel_muxed",', planner)
        self.assertIn('nh_.subscribe("/cmd_vel_muxed",', safety)
        self.assertNotIn('nh_.subscribe("/cmd_vel",', planner)
        self.assertNotIn('nh_.subscribe("/cmd_vel",', safety)

    def test_launch_teleop_remaps(self):
        for relative, node_type in (
            ('motion_planner/launch/planner.launch', 'twist_test_node'),
            ('robot_bringup/launch/robot_bringup.launch', 'teleop_twist_keyboard.py'),
            ('robot_navigation/launch/complete_navigation.launch', 'teleop_twist_keyboard.py'),
        ):
            with self.subTest(launch=relative):
                root = ET.parse(ROOT / relative).getroot()
                nodes = [n for n in root.iter('node') if n.get('type') == node_type]
                self.assertTrue(nodes)
                for node in nodes:
                    self.assertTrue(any(n.get('from') == 'cmd_vel' and n.get('to') == '/cmd_vel_teleop'
                                        for n in node.findall('remap')))

    def test_production_mux_preserves_local_safety(self):
        compiler = shutil.which('g++')
        if not compiler:
            self.skipTest('host g++ unavailable; run on a host with g++')
        with tempfile.TemporaryDirectory(prefix='ros-mux-host-') as directory:
            tmp = Path(directory)
            def write(relative, text):
                p = tmp / relative
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(text, encoding='utf-8')
            write('geometry_msgs/Twist.h', r'''#pragma once
#include <memory>
namespace geometry_msgs {
struct Vector3 { double x=0, y=0, z=0; };
struct Twist {
  using ConstPtr=std::shared_ptr<const Twist>;
  Vector3 linear, angular;
};
}
''')
            for name, data in [('Bool', 'bool data=false;'), ('String', 'std::string data;'),
                               ('Empty', ''), ('UInt32', 'uint32_t data=0;')]:
                write('std_msgs/' + name + '.h', '#pragma once\n#include <memory>\n#include <string>\n#include <cstdint>\n'
                    + 'namespace std_msgs { struct ' + name + ' { using ConstPtr=std::shared_ptr<const '
                    + name + '>; ' + data + ' }; }\n')
            write('ros/ros.h', r'''#pragma once
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <geometry_msgs/Twist.h>
namespace ros {
inline double& clock() { static double value=10; return value; }
struct Duration { double v; explicit Duration(double value=0):v(value){} double toSec()const{return v;} };
struct Time {
  double v; explicit Time(double value=0):v(value){}
  static Time now(){return Time(clock());}
  Duration operator-(Time other)const{return Duration(v-other.v);}
};
struct TimerEvent { Time current_real, last_real; };
struct Subscriber { std::string topic; };
struct Timer {};
inline std::vector<std::pair<std::string,geometry_msgs::Twist>>& outputs(){
  static std::vector<std::pair<std::string,geometry_msgs::Twist>> value; return value;
}
struct Publisher {
  std::string topic;
  void publish(const geometry_msgs::Twist& msg)const{outputs().emplace_back(topic,msg);}
  template<class M> void publish(const M&)const{}
};
struct NodeHandle {
  template<class T> void param(const std::string&,T& value,const T& fallback){value=fallback;}
  template<class O,class M> Subscriber subscribe(const std::string& topic,int,void(O::*)(const std::shared_ptr<const M>&),O*){return Subscriber{topic};}
  template<class M> Publisher advertise(const std::string& topic,int,bool=false){return Publisher{topic};}
  template<class O> Timer createTimer(Duration,void(O::*)(const TimerEvent&),O*){return Timer{};}
};
}
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
''')
            write('check.cpp', r'''#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#define private public
#include "motion_planner/cmd_vel_mux_node.h"
#undef private
int main(){
  ros::NodeHandle nh,pnh;
  motion_planner::CmdVelMuxNode mux(nh,pnh);
  assert(mux.direct_cmd_sub_.topic=="/cmd_vel");
  assert(mux.teleop_sub_.topic=="/cmd_vel_teleop");
  assert(mux.cmd_vel_pub_.topic=="/cmd_vel_muxed");
  assert(mux.direct_cmd_sub_.topic!=mux.cmd_vel_pub_.topic);
  auto tick=[&](){
    ros::TimerEvent event;
    event.current_real=ros::Time::now();
    event.last_real=ros::Time(ros::clock()-1.0/30.0);
    mux.timerCallback(event);
    assert(ros::outputs().back().first=="/cmd_vel_muxed");
    return ros::outputs().back().second;
  };
  assert(tick().linear.x==0);
  auto motion=std::make_shared<geometry_msgs::Twist>();motion->linear.x=0.1;
  mux.teleopCallback(motion);
  assert(tick().linear.x>0);
  auto zero=std::make_shared<geometry_msgs::Twist>();
  mux.teleopCallback(zero);
  assert(tick().linear.x==0); // local immediate keyboard-stop fix must survive
  mux.teleopCallback(motion);assert(tick().linear.x>0);
  mux.safetyCallback(zero);assert(tick().linear.x==0); // highest priority stop
  ros::clock()+=1;
  mux.teleopCallback(motion);assert(tick().linear.x>0);
  ros::clock()+=1;
  assert(tick().linear.x==0); // no active input -> timeout stop, not /cmd_vel
  mux.teleopCallback(motion);assert(tick().linear.x>0);
  auto yes=std::make_shared<std_msgs::Bool>();yes->data=true;
  auto no=std::make_shared<std_msgs::Bool>();no->data=false;
  mux.estopCallback(yes);assert(tick().linear.x==0);
  mux.estopCallback(no);assert(mux.estop_latched_); // false must not clear latch
  auto start=std::make_shared<std_msgs::UInt32>();start->data=0x5A17;
  mux.physicalStartCallback(start);assert(mux.estop_latched_);
  assert(tick().linear.x==0); // authenticated start must NOT reset emergency stop
  mux.emergencyResetCallback(yes);
  assert(!mux.estop_latched_ && tick().linear.x==0); // reset clears stale commands
  mux.teleopCallback(motion);assert(tick().linear.x>0);
  mux.chassisLockCallback(yes);assert(tick().linear.x==0);
  return 0;
}
''')
            executable = tmp / ('check.exe' if os.name == 'nt' else 'check')
            subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-I', str(tmp),
                            '-I', str(ROOT / 'motion_planner/include'),
                            str(ROOT / 'motion_planner/src/cmd_vel_mux_node.cpp'),
                            str(tmp / 'check.cpp'), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
