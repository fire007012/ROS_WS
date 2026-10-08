#include "fine_tuning/fine_tuning_controller.h"
#include <algorithm>
#include <cmath>

namespace fine_tuning {
FineTuningController::FineTuningController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {}

bool FineTuningController::init() {
  pnh_.param("target_distance_mm", target_mm_, 400.0);
  pnh_.param("tolerance_mm", tolerance_mm_, 10.0);
  pnh_.param("move_axis", axis_, 1.0);
  pnh_.param("move_direction", direction_, 1);
  pnh_.param("step_velocity", max_speed_, 0.05);
  double global_limit;
  nh_.param("/robot/max_linear_vel", global_limit, max_speed_);
  max_speed_ = std::min(max_speed_, global_limit);
  pnh_.param("step_duration", step_duration_, 0.2);
  pnh_.param("settle_time", settle_time_, 0.3);
  pnh_.param("max_fine_tuning_time", max_time_, 25.0);
  pnh_.param("kp_distance", kp_, 0.0005);
  pnh_.param("max_steps", max_steps_, 50);
  pnh_.param("distance_timeout_sec", sensor_timeout_, 0.5);
  pnh_.param("odom_timeout_sec", odom_timeout_, 0.5);
  pnh_.param("max_translation_m", max_translation_, 0.05);
  pnh_.param("allow_reverse", allow_reverse_, false);
  pnh_.param("auto_start_on_path_finished", auto_start_, false);
  double safety_floor, margin, reaction, deceleration, hysteresis;
  nh_.param("/distance_safety_node/safety_distance_mm", safety_floor, 300.0);
  nh_.param("/distance_safety_node/stop_margin_mm", margin, 300.0);
  nh_.param("/distance_safety_node/reaction_time_sec", reaction, 0.2);
  nh_.param("/distance_safety_node/braking_deceleration_mps2", deceleration, 1.0);
  nh_.param("/distance_safety_node/release_hysteresis_mm", hysteresis, 30.0);
  minimum_target_mm_ = std::max(safety_floor, margin + 1000.0 *
      (max_speed_ * max_speed_ / (2.0 * std::max(0.01, deceleration)) + max_speed_ * reaction))
      + hysteresis + tolerance_mm_;
  for (double value : {target_mm_, tolerance_mm_, max_speed_, step_duration_, max_time_, kp_,
                       sensor_timeout_, odom_timeout_, max_translation_}) {
    if (!std::isfinite(value) || value <= 0.0) return false;
  }
  if ((axis_ != 1.0 && axis_ != 2.0) || (direction_ != 1 && direction_ != -1) ||
      !std::isfinite(settle_time_) || settle_time_ < 0.0 || max_steps_ < 0) return false;
  std::string distance_topic;
  pnh_.param<std::string>("distance_topic", distance_topic, "/vl53l1x_distance");
  path_sub_ = nh_.subscribe("/path_finished", 1, &FineTuningController::pathFinishedCallback, this);
  distance_sub_ = nh_.subscribe(distance_topic, 1, &FineTuningController::distanceCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &FineTuningController::odomCallback, this);
  estop_sub_ = nh_.subscribe("/emergency_stop", 1, &FineTuningController::estopCallback, this);
  reset_sub_ = nh_.subscribe("/emergency_stop/reset", 1, &FineTuningController::resetCallback, this);
  cancel_sub_ = nh_.subscribe("/mission/cancel", 1, &FineTuningController::cancelCallback, this);
  actions_enabled_sub_ = nh_.subscribe<std_msgs::Bool>("/mission/actions_enabled", 1,
      [this](const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) actions_enabled_ = true; else cancelCallback(std_msgs::Empty::ConstPtr());
      });
  revision_sub_ = nh_.subscribe("/path_manager/path_revision", 1, &FineTuningController::revisionCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_external", 1);
  done_pub_ = nh_.advertise<std_msgs::Bool>("/fine_tuning_done", 1, true);
  failed_pub_ = nh_.advertise<std_msgs::Bool>("/fine_tuning_failed", 1, true);
  status_pub_ = nh_.advertise<std_msgs::String>("/fine_tuning/status", 1, true);
  start_srv_ = nh_.advertiseService("/fine_tuning/start", &FineTuningController::fineTuningStartCallback, this);
  double rate;
  pnh_.param("control_rate_hz", rate, 20.0);
  if (!std::isfinite(rate) || rate <= 0.0) return false;
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &FineTuningController::controlTimerCallback, this);
  std_msgs::Bool no;
  done_pub_.publish(no);
  failed_pub_.publish(no);
  return true;
}

bool FineTuningController::fresh() const {
  const auto now = ros::SteadyTime::now();
  return has_distance_ && has_odom_ &&
      (now - distance_time_).toSec() <= sensor_timeout_ &&
      (now - odom_time_).toSec() <= odom_timeout_;
}
void FineTuningController::distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  has_distance_ = std::isfinite(msg->data) && msg->data >= 40.0 && msg->data <= 4000.0;
  if (!has_distance_) return;
  distance_mm_ = msg->data;
  distance_time_ = ros::SteadyTime::now();
  if (auto_start_ && path_finished_ && !require_new_path_ && state_ == State::IDLE) {
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    fineTuningStartCallback(request, response);
  }
}
void FineTuningController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  const double age = msg->header.stamp.isZero() ? 0.0 : (ros::Time::now()-msg->header.stamp).toSec();
  has_odom_ = std::isfinite(msg->pose.pose.position.x) && std::isfinite(msg->pose.pose.position.y) &&
      age >= -0.05 && age <= odom_timeout_;
  if (!has_odom_) return;
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  odom_time_ = ros::SteadyTime::now();
}
void FineTuningController::pathFinishedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (estop_latched_ || require_new_path_) return;
  path_finished_ = msg->data;
  if (auto_start_ && path_finished_ && fresh() && state_ == State::IDLE) {
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    fineTuningStartCallback(request, response);
  }
}
void FineTuningController::estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  estop_latched_ = true;
  cancelCallback(std_msgs::Empty::ConstPtr());
}
void FineTuningController::resetCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  estop_latched_ = false;
  path_finished_ = false;
  require_new_path_ = true;
  state_ = State::IDLE;
}
void FineTuningController::cancelCallback(const std_msgs::Empty::ConstPtr&) {
  actions_enabled_ = false;
  require_new_path_ = true;
  path_finished_ = false;
  if (state_ == State::FINE_TUNING) finish(false, "cancelled");
}
void FineTuningController::revisionCallback(const std_msgs::UInt32::ConstPtr& msg) {
  if (msg->data == revision_) return;
  revision_ = msg->data;
  if (estop_latched_) return;
  require_new_path_ = false;
  path_finished_ = false;
  if (state_ == State::DONE || state_ == State::FAILED) state_ = State::IDLE;
}
bool FineTuningController::fineTuningStartCallback(std_srvs::Trigger::Request&,
                                                  std_srvs::Trigger::Response& res) {
  if (estop_latched_) res.message = "ESTOP latched";
  else if (!actions_enabled_) res.message = "cancelled: a new accepted mission is required";
  else if (state_ == State::FINE_TUNING) res.message = "fine tuning already running";
  else if (!fresh()) res.message = "fresh distance and odometry required";
  else if (target_mm_ < minimum_target_mm_) res.message = "target conflicts with braking clearance";
  else {
    state_ = State::FINE_TUNING;
    require_new_path_ = false;
    steps_ = 0;
    origin_x_ = x_;
    origin_y_ = y_;
    start_time_ = phase_time_ = ros::SteadyTime::now();
    moving_ = false;
    publishVelocity(0.0, 0.0);
    std_msgs::Bool no;
    done_pub_.publish(no);
    failed_pub_.publish(no);
    res.success = true;
    res.message = "fine tuning started";
    return true;
  }
  res.success = false;
  return true;
}
void FineTuningController::finish(bool success, const std::string& reason) {
  publishVelocity(0.0, 0.0);
  state_ = success ? State::DONE : State::FAILED;
  moving_ = false;
  std_msgs::Bool done, failed;
  done.data = success;
  failed.data = !success;
  done_pub_.publish(done);
  failed_pub_.publish(failed);
  std_msgs::String status;
  status.data = reason;
  status_pub_.publish(status);
  if (!success) ROS_WARN("[fine_tuning] failed: %s", reason.c_str());
}
void FineTuningController::controlTimerCallback(const ros::TimerEvent&) {
  if (state_ != State::FINE_TUNING) return;
  const auto now = ros::SteadyTime::now();
  if (estop_latched_) { finish(false, "ESTOP"); return; }
  if (!fresh()) { finish(false, "stale/invalid sensor or odometry"); return; }
  if ((now-start_time_).toSec() >= max_time_) { finish(false, "timeout"); return; }
  if (std::hypot(x_-origin_x_, y_-origin_y_) >= max_translation_) {
    finish(false, "translation limit reached"); return;
  }
  if (moving_) {
    if ((now-phase_time_).toSec() < step_duration_) {
      // Preserve the signed velocity and magnitude selected for THIS step.
      publishVelocity(step_vx_, step_vy_);
    } else {
      publishVelocity(0.0, 0.0);
      moving_ = false;
      phase_time_ = now;
    }
    return;
  }
  publishVelocity(0.0, 0.0);
  if ((now-phase_time_).toSec() < settle_time_) return;
  const double error = distance_mm_-target_mm_;
  if (std::abs(error) <= tolerance_mm_) { finish(true, "distance aligned"); return; }
  if (max_steps_ > 0 && steps_ >= max_steps_) { finish(false, "step limit reached"); return; }
  const double speed = std::min(max_speed_, std::max(0.005, kp_*std::abs(error)));
  const double signed_speed = (error > 0.0 ? direction_ : -direction_)*speed;
  step_vx_ = axis_ == 1.0 ? signed_speed : 0.0;
  step_vy_ = axis_ == 2.0 ? signed_speed : 0.0;
  if (step_vx_ < 0.0 && !allow_reverse_) { finish(false, "rear sensor required for reverse correction"); return; }
  if (std::hypot(x_-origin_x_, y_-origin_y_) + speed*step_duration_ > max_translation_) {
    finish(false, "next step would exceed translation limit"); return;
  }
  moving_ = true;
  phase_time_ = now;
  ++steps_;
  publishVelocity(step_vx_, step_vy_);
}
void FineTuningController::publishVelocity(double vx, double vy) {
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  velocity_pub_.publish(msg);
}
}  // namespace fine_tuning
int main(int argc, char** argv) {
  ros::init(argc, argv, "fine_tuning_node");
  ros::NodeHandle nh, pnh("~");
  fine_tuning::FineTuningController controller(nh, pnh);
  if (!controller.init()) return 1;
  ros::spin();
  return 0;
}
