#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace fine_tuning {
class FineTuningController {
 public:
  FineTuningController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  bool init();
 private:
  enum class State { IDLE, FINE_TUNING, DONE, FAILED };
  void pathFinishedCallback(const std_msgs::Bool::ConstPtr& msg);
  void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  void resetCallback(const std_msgs::Bool::ConstPtr& msg);
  void cancelCallback(const std_msgs::Empty::ConstPtr&);
  void revisionCallback(const std_msgs::UInt32::ConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent&);
  bool fineTuningStartCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  bool fresh() const;
  void finish(bool success, const std::string& reason);
  void publishVelocity(double vx, double vy);
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber path_sub_, distance_sub_, odom_sub_, estop_sub_, reset_sub_, cancel_sub_, revision_sub_;
  ros::Subscriber actions_enabled_sub_;
  ros::Publisher velocity_pub_, done_pub_, failed_pub_, status_pub_;
  ros::ServiceServer start_srv_;
  ros::Timer timer_;
  State state_ = State::IDLE;
  bool estop_latched_ = false, path_finished_ = false, require_new_path_ = false;
  uint32_t revision_ = 0;
  bool auto_start_ = false, allow_reverse_ = false;
  bool actions_enabled_ = true;
  double distance_mm_ = 0.0, x_ = 0.0, y_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0;
  bool has_distance_ = false, has_odom_ = false;
  ros::SteadyTime distance_time_, odom_time_, start_time_, phase_time_;
  bool moving_ = false;
  double step_vx_ = 0.0, step_vy_ = 0.0;
  double target_mm_ = 400.0, tolerance_mm_ = 10.0, axis_ = 1.0;
  int direction_ = 1, max_steps_ = 50, steps_ = 0;
  double max_speed_ = 0.05, step_duration_ = 0.2, settle_time_ = 0.3;
  double max_time_ = 25.0, kp_ = 0.0005, sensor_timeout_ = 0.5, odom_timeout_ = 0.5;
  double max_translation_ = 0.05, minimum_target_mm_ = 350.0;
};
}  // namespace fine_tuning
