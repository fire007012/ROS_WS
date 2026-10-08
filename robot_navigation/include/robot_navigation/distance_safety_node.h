#pragma once
#include <array>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

namespace robot_navigation {
// Directional braking only; three ToF beams do not constitute an obstacle map.
class DistanceSafetyNode {
 public:
  DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool init();
 private:
  struct Sample {
    double distance_mm = 0.0;
    ros::SteadyTime received;
    bool valid = false;
  };
  void rangeCallback(size_t index, const sensor_msgs::Range::ConstPtr& msg);
  void requestedCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void measuredCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void safetyTimerCallback(const ros::TimerEvent&);
  void publishState(const std::string& state);
  ros::NodeHandle nh_, pnh_;
  std::array<ros::Subscriber, 4> range_subs_;
  ros::Subscriber requested_sub_, measured_sub_;
  ros::Publisher cmd_pub_, state_pub_;
  ros::Timer timer_;
  std::array<Sample, 4> samples_;
  geometry_msgs::Twist requested_, measured_;
  ros::SteadyTime requested_time_, measured_time_;
  bool has_request_ = false, has_measured_ = false;
  std::array<bool, 4> stopped_{{false, false, false, false}};
  double safety_distance_mm_ = 300.0;
  double critical_distance_mm_ = 100.0;
  double braking_deceleration_mps2_ = 1.0;
  double reaction_time_sec_ = 0.2;
  double stop_margin_mm_ = 300.0;
  double distance_timeout_sec_ = 0.5;
  double command_timeout_sec_ = 0.5;
  double release_hysteresis_mm_ = 30.0;
  double projection_radius_m_ = 0.25;
  double rotation_clearance_mm_ = 300.0;
  std::string state_;
};
}  // namespace robot_navigation
