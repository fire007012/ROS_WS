#include "robot_navigation/distance_safety_node.h"
#include <algorithm>
#include <cmath>

namespace robot_navigation {
DistanceSafetyNode::DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {}

bool DistanceSafetyNode::init() {
  pnh_.param("safety_distance_mm", safety_distance_mm_, 150.0);
  pnh_.param("critical_distance_mm", critical_distance_mm_, 100.0);
  pnh_.param("braking_deceleration_mps2", braking_deceleration_mps2_, 1.0);
  pnh_.param("reaction_time_sec", reaction_time_sec_, 0.2);
  pnh_.param("stop_margin_mm", stop_margin_mm_, 100.0);
  pnh_.param("distance_timeout_sec", distance_timeout_sec_, 0.5);
  pnh_.param("command_timeout_sec", command_timeout_sec_, 0.5);
  pnh_.param("release_hysteresis_mm", release_hysteresis_mm_, 30.0);
  pnh_.param("rotation_clearance_mm", rotation_clearance_mm_, 100.0);
  double rate;
  pnh_.param("control_rate_hz", rate, 20.0);
  for (double value : {safety_distance_mm_, critical_distance_mm_, braking_deceleration_mps2_,
                       distance_timeout_sec_, command_timeout_sec_, rate}) {
    if (!std::isfinite(value) || value <= 0.0) return false;
  }
  for (double value : {reaction_time_sec_, stop_margin_mm_, release_hysteresis_mm_, rotation_clearance_mm_}) {
    if (!std::isfinite(value) || value < 0.0) return false;
  }
  const std::array<std::string, 4> names{{"front", "left", "right", "rear"}};
  for (size_t i = 0; i < names.size(); ++i) {
    nh_.param("/chassis_geometry/" + names[i] + "_inset_m", sensor_insets_m_[i], i == 0 ? 0.0 : 0.05);
    if (!std::isfinite(sensor_insets_m_[i]) || sensor_insets_m_[i] < 0.0) return false;
    std::string topic;
    pnh_.param(names[i] + "_range_topic", topic, "/" + names[i] + "/range");
    range_subs_[i] = nh_.subscribe<sensor_msgs::Range>(topic, 10,
        [this, i](const sensor_msgs::Range::ConstPtr& msg) { rangeCallback(i, msg); });
  }
  requested_sub_ = nh_.subscribe("/cmd_vel_requested", 1, &DistanceSafetyNode::requestedCallback, this);
  measured_sub_ = nh_.subscribe("/base_velocity", 1, &DistanceSafetyNode::measuredCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_safety", 1);
  state_pub_ = nh_.advertise<std_msgs::String>("/distance_safety/state", 1, true);
  timer_ = nh_.createSteadyTimer(ros::WallDuration(1.0 / rate), &DistanceSafetyNode::safetyTimerCallback, this);
  publishState("CLEAR");
  ROS_INFO("[distance_safety] four-direction braking on valid obstacle detections; missing returns ignored");
  return true;
}

void DistanceSafetyNode::rangeCallback(size_t index, const sensor_msgs::Range::ConstPtr& msg) {
  auto& sample = samples_[index];
  const double age = msg->header.stamp.isZero() ? 0.0 : (ros::Time::now() - msg->header.stamp).toSec();
  sample.valid = std::isfinite(msg->range) && std::isfinite(msg->min_range) &&
      std::isfinite(msg->max_range) && msg->range >= msg->min_range &&
      msg->range <= msg->max_range && msg->range > 0.0 &&
      age >= -0.05 && age <= distance_timeout_sec_;
  if (!sample.valid) return;
  // Convert sensor-origin range into clearance outside the circular body.
  // An inset sensor can report a valid obstacle at/below the body edge.
  sample.distance_mm = std::max(0.0, msg->range - sensor_insets_m_[index]) * 1000.0;
  sample.received = ros::SteadyTime::now();
}
void DistanceSafetyNode::requestedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  requested_ = *msg;
  requested_time_ = ros::SteadyTime::now();
  has_request_ = true;
}
void DistanceSafetyNode::measuredCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  measured_ = *msg;
  measured_time_ = ros::SteadyTime::now();
  has_measured_ = true;
}
void DistanceSafetyNode::publishState(const std::string& state) {
  if (state_ == state) return;
  state_ = state;
  std_msgs::String msg;
  msg.data = state;
  state_pub_.publish(msg);
}
void DistanceSafetyNode::safetyTimerCallback(const ros::SteadyTimerEvent&) {
  const auto now = ros::SteadyTime::now();
  geometry_msgs::Twist output;
  auto stop = [this, &output](const std::string& reason) {
    cmd_pub_.publish(output);
    publishState(reason);
  };
  if (!has_request_ || (now - requested_time_).toSec() > command_timeout_sec_) {
    stop("COMMAND_TIMEOUT");
    return;
  }
  const auto& r = requested_;
  if (!std::isfinite(r.linear.x) || !std::isfinite(r.linear.y) || !std::isfinite(r.angular.z)) {
    stop("INVALID_COMMAND");
    return;
  }
  geometry_msgs::Twist measured;
  if (has_measured_ && (now - measured_time_).toSec() <= command_timeout_sec_ &&
      std::isfinite(measured_.linear.x) && std::isfinite(measured_.linear.y) &&
      std::isfinite(measured_.angular.z)) measured = measured_;
  const double rotation = std::max(std::abs(r.angular.z), std::abs(measured.angular.z));
  const std::array<double, 4> toward{{
    std::max({0.0, r.linear.x, measured.linear.x}),
    std::max({0.0, r.linear.y, measured.linear.y}),
    std::max({0.0, -r.linear.y, -measured.linear.y}),
    std::max({0.0, -r.linear.x, -measured.linear.x})}};
  double factor = 1.0;
  for (size_t i = 0; i < toward.size(); ++i) {
    // Check every available beam during rotation; only detected obstacles brake.
    const bool turning_check = rotation > 0.001;
    if (toward[i] <= 0.001 && !turning_check) {
      stopped_[i] = false;
      continue;
    }
    const auto& sample = samples_[i];
    if (!sample.valid || (now - sample.received).toSec() > distance_timeout_sec_) {
      stopped_[i] = false;
      continue;
    }
    // A circular footprint sweeps the same area while turning in place.
    // Only translation closes the gap; all beams still enforce turn clearance.
    const double speed = toward[i];
    const bool rotation_only = turning_check && toward[i] <= 0.001;
    const double margin = rotation_only ? rotation_clearance_mm_ : stop_margin_mm_;
    const double floor = rotation_only ? rotation_clearance_mm_ : safety_distance_mm_;
    const double threshold = std::max({critical_distance_mm_, floor,
        1000.0 * (speed * speed / (2.0 * braking_deceleration_mps2_) + speed * reaction_time_sec_) + margin});
    if (sample.distance_mm <= threshold ||
        (stopped_[i] && sample.distance_mm <= threshold + release_hysteresis_mm_)) {
      stopped_[i] = true;
      stop("STOP");
      return;
    }
    stopped_[i] = false;
    if (sample.distance_mm < threshold * 1.5) factor = std::min(factor, 0.3);
  }
  if (factor < 1.0) {
    output = r;
    output.linear.x *= factor;
    output.linear.y *= factor;
    output.angular.z *= factor;
    cmd_pub_.publish(output);
    publishState("WARNING");
  } else {
    // A silent clear channel lets the mux expire the previous safety command.
    publishState("CLEAR");
  }
}
}  // namespace robot_navigation
int main(int argc, char** argv) {
  ros::init(argc, argv, "distance_safety_node");
  ros::NodeHandle nh, pnh("~");
  robot_navigation::DistanceSafetyNode node(nh, pnh);
  if (!node.init()) return 1;
  ros::spin();
  return 0;
}
