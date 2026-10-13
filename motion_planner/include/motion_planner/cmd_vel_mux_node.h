#pragma once

#include <map>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>

namespace motion_planner {

class CmdVelMuxNode {
 public:
  CmdVelMuxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void fixedRouteCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void externalCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void safetyCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void alignmentCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void alignmentModeCallback(const std_msgs::Bool::ConstPtr& msg);
  void missionCancelCallback(const std_msgs::Empty::ConstPtr& msg);
  void actionsEnabledCallback(const std_msgs::Bool::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  void emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg);
  void chassisLockCallback(const std_msgs::Bool::ConstPtr& msg);
  void physicalStartCallback(const std_msgs::UInt32::ConstPtr& msg);
  void timerCallback(const ros::SteadyTimerEvent& event);
  void updateSourceCommand(const std::string& source_name, const geometry_msgs::Twist& msg);
  bool sourceActive(const std::string& source_name, const ros::SteadyTime& now) const;
  geometry_msgs::Twist clampTwist(const geometry_msgs::Twist& input, double dt,
                                  bool bypass_acceleration_limit = false) const;
  void publishSelected(const geometry_msgs::Twist& twist, const std::string& source_name);
  void publishStop(const std::string& reason);

 private:
  struct SourceState {
    geometry_msgs::Twist twist;
    ros::SteadyTime stamp;
    bool has_msg;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber fixed_route_sub_;
  ros::Subscriber teleop_sub_;
  ros::Subscriber direct_cmd_sub_;  // Backward-compatible direct /cmd_vel teleop input.
  ros::Subscriber external_sub_;
  ros::Subscriber safety_sub_;
  ros::Subscriber alignment_sub_, alignment_mode_sub_;
  bool alignment_active_ = false;
  bool mission_cancelled_ = false;
  double alignment_cmd_timeout_sec_ = 0.2;
  ros::Subscriber mission_cancel_sub_, actions_enabled_sub_;
  ros::Subscriber estop_sub_;
  ros::Subscriber emergency_reset_sub_;
  ros::Subscriber chassis_lock_sub_;
  ros::Subscriber physical_start_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher requested_vel_pub_;
  ros::Publisher selected_source_pub_;
  ros::Publisher estop_state_pub_;
  ros::SteadyTimer timer_;

  std::map<std::string, SourceState> sources_;
  geometry_msgs::Twist last_output_;

  double publish_rate_hz_;
  double cmd_timeout_sec_;
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;
  double max_angular_accel_;
  bool estop_active_;
  bool estop_latched_;
  bool timeout_reported_;
  uint32_t start_auth_token_;
  bool chassis_locked_;
};

}  // namespace motion_planner
