#pragma once

#include <mutex>
#include <string>
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include "can_motor_interface/link_health.h"

namespace arm_interface {

// Four chassis axes and one arm axis (STM32 index 4).
class ArmControllerNode {
 public:
  ArmControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ArmControllerNode();
 private:
  void armCmdCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void canRxCallback(const can_msgs::Frame::ConstPtr& msg);
  void readyCallback(const std_msgs::Bool::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  void timerCallback(const ros::WallTimerEvent&);
  bool send(const can_frame& frame);
  bool openSocket();
  void closeSocket();
  bool ready() const;
  void fail(const char* reason);
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joint_cmd_sub_, can_rx_sub_, ready_sub_, estop_sub_;
  ros::Publisher arm_state_pub_, estop_pub_;
  ros::WallTimer timer_;
  std::string can_device_, joint_name_;
  int socket_fd_ = -1;
  std::mutex socket_mutex_;
  can_motor_interface::MotionInterlock motion_;
  bool link_ready_ = false;
  bool emergency_active_ = false;
  double link_ready_time_ = 0;
  double position_time_ = 0;
  double status_time_ = 0;
  uint8_t status_flags_ = 0;
  bool query_status_ = false;
  int speed_rpm_ = 100;
};

}  // namespace arm_interface
