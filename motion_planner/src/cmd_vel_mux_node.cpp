#include "motion_planner/cmd_vel_mux_node.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_planner {

CmdVelMuxNode::CmdVelMuxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      publish_rate_hz_(30.0),
      cmd_timeout_sec_(0.5),
      max_linear_vel_(1.0),
      max_angular_vel_(2.0),
      max_linear_accel_(1.0),
      max_angular_accel_(2.0),
      estop_active_(false),
      estop_latched_(false),
      timeout_reported_(false),
      start_auth_token_(0x5A17),
      chassis_locked_(false) {
  nh_.param("/robot/max_linear_vel", max_linear_vel_, max_linear_vel_);
  nh_.param("/robot/max_angular_vel", max_angular_vel_, max_angular_vel_);
  nh_.param("/robot/max_linear_accel", max_linear_accel_, max_linear_accel_);
  nh_.param("/robot/max_angular_accel", max_angular_accel_, max_angular_accel_);
  pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
  pnh_.param("cmd_timeout_sec", cmd_timeout_sec_, cmd_timeout_sec_);
  pnh_.param("alignment_cmd_timeout_sec", alignment_cmd_timeout_sec_, 0.2);
  if (!std::isfinite(alignment_cmd_timeout_sec_) || alignment_cmd_timeout_sec_ <= 0 ||
      alignment_cmd_timeout_sec_ > 0.2) throw std::invalid_argument("alignment_cmd_timeout_sec must be in (0, 0.2]");
  int start_token = static_cast<int>(start_auth_token_); pnh_.param<int>("start_auth_token", start_token, start_token); start_auth_token_ = static_cast<uint32_t>(start_token);

  fixed_route_sub_ = nh_.subscribe("/cmd_vel_fixed_route", 10, &CmdVelMuxNode::fixedRouteCallback, this);
  teleop_sub_ = nh_.subscribe("/cmd_vel_teleop", 10, &CmdVelMuxNode::teleopCallback, this);
  // Keep /cmd_vel as an input alias for rosrun teleop_twist_keyboard and manual tests.
  // The mux output is published on /cmd_vel_muxed, so this cannot form a feedback loop.
  direct_cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &CmdVelMuxNode::teleopCallback, this);
  external_sub_ = nh_.subscribe("/cmd_vel_external", 10, &CmdVelMuxNode::externalCallback, this);
  safety_sub_ = nh_.subscribe("/cmd_vel_safety", 10, &CmdVelMuxNode::safetyCallback, this);
  alignment_sub_ = nh_.subscribe("/cmd_vel_arm_alignment", 1, &CmdVelMuxNode::alignmentCallback, this);
  alignment_mode_sub_ = nh_.subscribe("/arm_upper_alignment/active", 1, &CmdVelMuxNode::alignmentModeCallback, this);
  mission_cancel_sub_ = nh_.subscribe("/mission/cancel", 1, &CmdVelMuxNode::missionCancelCallback, this);
  actions_enabled_sub_ = nh_.subscribe("/mission/actions_enabled", 1, &CmdVelMuxNode::actionsEnabledCallback, this);
  estop_sub_ = nh_.subscribe("/emergency_stop", 10, &CmdVelMuxNode::estopCallback, this);
  emergency_reset_sub_ = nh_.subscribe("/emergency_stop/reset", 2, &CmdVelMuxNode::emergencyResetCallback, this);
  chassis_lock_sub_ = nh_.subscribe("/chassis_lock", 10, &CmdVelMuxNode::chassisLockCallback, this);
  physical_start_sub_ = nh_.subscribe("/start_signal/physical", 1, &CmdVelMuxNode::physicalStartCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_muxed", 10);
  requested_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_requested", 1);
  selected_source_pub_ = nh_.advertise<std_msgs::String>("/cmd_vel_mux/selected_source", 10, true);
  estop_state_pub_ = nh_.advertise<std_msgs::Bool>("/cmd_vel_mux/estop_active", 10, true);

  sources_["fixed_route"] = SourceState();
  sources_["teleop"] = SourceState();
  sources_["external"] = SourceState();
  sources_["safety"] = SourceState();
  sources_["arm_alignment"] = SourceState();

  const double period = 1.0 / std::max(1.0, publish_rate_hz_);
  timer_ = nh_.createSteadyTimer(ros::WallDuration(period), &CmdVelMuxNode::timerCallback, this);
}

void CmdVelMuxNode::fixedRouteCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("fixed_route", *msg);
}

void CmdVelMuxNode::teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("teleop", *msg);
}

void CmdVelMuxNode::externalCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("external", *msg);
}

void CmdVelMuxNode::safetyCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("safety", *msg);
}

void CmdVelMuxNode::alignmentCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  geometry_msgs::Twist lateral;
  lateral.linear.y = msg->linear.y;
  updateSourceCommand("arm_alignment", lateral);
  if (alignment_active_ && sources_["arm_alignment"].twist.linear.y == 0)
    publishStop("arm_alignment_stop");
}
void CmdVelMuxNode::alignmentModeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (alignment_active_ == msg->data) return;
  alignment_active_ = msg->data;
  // A mode transition must never resume a cached navigation/teleop command.
  for (auto& source : sources_) source.second.has_msg = false;
  publishStop("arm_alignment_mode_change");
}

void CmdVelMuxNode::missionCancelCallback(const std_msgs::Empty::ConstPtr&) {
  mission_cancelled_ = true;
  for (auto& source : sources_) source.second.has_msg = false;
  publishStop("mission_cancel");
}
void CmdVelMuxNode::actionsEnabledCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && mission_cancelled_) {
    mission_cancelled_ = false;
    for (auto& source : sources_) source.second.has_msg = false;
  }
}

void CmdVelMuxNode::estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) { estop_latched_ = true; }
  estop_active_ = estop_latched_;
  std_msgs::Bool state;
  state.data = estop_active_;
  estop_state_pub_.publish(state);
  if (estop_active_) { publishStop("emergency_stop"); }
}

void CmdVelMuxNode::emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  estop_latched_ = false;
  estop_active_ = false;
  for (auto& source : sources_) source.second.has_msg = false;
  last_output_ = geometry_msgs::Twist();
  std_msgs::Bool state; state.data = false; estop_state_pub_.publish(state);
  ROS_WARN("[cmd_vel_mux] ESTOP reset accepted by upper safety policy");
}

void CmdVelMuxNode::physicalStartCallback(const std_msgs::UInt32::ConstPtr& msg) {
  if (msg->data != start_auth_token_) return;
  ROS_INFO("[cmd_vel_mux] physical start received; ESTOP latch remains unchanged");
}

void CmdVelMuxNode::chassisLockCallback(const std_msgs::Bool::ConstPtr& msg) {
  chassis_locked_ = msg->data;
  if (chassis_locked_) {
    publishStop("chassis_lock");
  }
}

void CmdVelMuxNode::timerCallback(const ros::SteadyTimerEvent& event) {
  const ros::SteadyTime now = ros::SteadyTime::now();
  const double dt = std::max(1e-3, (event.current_real - event.last_real).toSec());

  // Safety must inspect the requested motion, never its own reduced/stopped
  // output. Otherwise a stop releases itself and a warning compounds to zero.
  geometry_msgs::Twist requested;
  if (!estop_active_ && !chassis_locked_ && !mission_cancelled_) {
    if (alignment_active_) {
      if (sourceActive("arm_alignment", now)) requested = sources_["arm_alignment"].twist;
    } else for (const char* source : {"external", "teleop", "fixed_route"}) {
      if (sourceActive(source, now)) {
        requested = clampTwist(sources_[source].twist, dt, true);
        break;
      }
    }
  }
  requested_vel_pub_.publish(requested);

  if (estop_active_) {
    publishStop("emergency_stop");
    return;
  }

  if (mission_cancelled_) {
    publishStop("mission_cancel");
    return;
  }

  if (chassis_locked_) {
    publishStop("chassis_lock");
    return;
  }

  if (alignment_active_) {
    // Remain in this mode on publisher loss: stop, never fall back to navigation.
    if (!sourceActive("arm_alignment", now) || requested.linear.y == 0.0) {
      publishStop("arm_alignment_wait_or_timeout");
      return;
    }
    if (sourceActive("safety", now)) {
      geometry_msgs::Twist safe;
      const double y = sources_["safety"].twist.linear.y;
      if (y * requested.linear.y > 0)
        safe.linear.y = std::copysign(std::min(std::abs(y), std::abs(requested.linear.y)), y);
      publishSelected(safe, "safety");
      return;
    }
    auto output = clampTwist(requested, dt);
    output.linear.x = 0;
    output.angular.z = 0;
    publishSelected(output, "arm_alignment");
    return;
  }

  const char* priority_order[] = {"safety", "external", "teleop", "fixed_route"};
  for (const char* source_name : priority_order) {
    if (!sourceActive(source_name, now)) {
      continue;
    }
    // A safety command must be able to stop immediately. Applying the normal
    // acceleration ramp here would turn a safety stop into a delayed stop.
    const bool is_safety_source = std::string(source_name) == "safety";
    const bool is_teleop_stop = std::string(source_name) == "teleop" &&
        sources_[source_name].twist.linear.x == 0.0 &&
        sources_[source_name].twist.linear.y == 0.0 &&
        sources_[source_name].twist.linear.z == 0.0 &&
        sources_[source_name].twist.angular.x == 0.0 &&
        sources_[source_name].twist.angular.y == 0.0 &&
        sources_[source_name].twist.angular.z == 0.0;
    const geometry_msgs::Twist clamped = clampTwist(
        sources_[source_name].twist, dt, is_safety_source || is_teleop_stop);
    if (timeout_reported_) {
      ROS_INFO("[cmd_vel_mux] command source recovered: %s", source_name);
      timeout_reported_ = false;
    }
    publishSelected(clamped, source_name);
    return;
  }

  if (!timeout_reported_) {
    ROS_WARN("[cmd_vel_mux] command timeout: no active cmd_vel source for %.3f s; publishing zero velocity",
             cmd_timeout_sec_);
    timeout_reported_ = true;
  }
  publishStop("timeout_stop");
}

void CmdVelMuxNode::updateSourceCommand(const std::string& source_name, const geometry_msgs::Twist& msg) {
  SourceState& source = sources_[source_name];
  // Invalid commands must never turn into maximum speed through std::min/max.
  source.twist = geometry_msgs::Twist();
  if (std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
      std::isfinite(msg.angular.z)) {
    source.twist.linear.x = msg.linear.x;
    source.twist.linear.y = msg.linear.y;
    source.twist.angular.z = msg.angular.z;
  }
  source.stamp = ros::SteadyTime::now();
  source.has_msg = true;
}

bool CmdVelMuxNode::sourceActive(const std::string& source_name, const ros::SteadyTime& now) const {
  const auto it = sources_.find(source_name);
  if (it == sources_.end() || !it->second.has_msg) {
    return false;
  }
  const double timeout = source_name == "arm_alignment" ? alignment_cmd_timeout_sec_ : cmd_timeout_sec_;
  return (now - it->second.stamp).toSec() <= timeout;
}

geometry_msgs::Twist CmdVelMuxNode::clampTwist(const geometry_msgs::Twist& input, double dt,
                                               bool bypass_acceleration_limit) const {
  geometry_msgs::Twist output = input;
  output.linear.x = std::max(-max_linear_vel_, std::min(max_linear_vel_, output.linear.x));
  output.linear.y = std::max(-max_linear_vel_, std::min(max_linear_vel_, output.linear.y));
  output.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, output.angular.z));

  if (bypass_acceleration_limit) {
    return output;
  }

  const double linear_step = max_linear_accel_ * dt;
  const double angular_step = max_angular_accel_ * dt;

  auto clampStep = [](double target, double last, double step) {
    const double delta = target - last;
    if (std::fabs(delta) <= step) {
      return target;
    }
    return last + std::copysign(step, delta);
  };

  output.linear.x = clampStep(output.linear.x, last_output_.linear.x, linear_step);
  output.linear.y = clampStep(output.linear.y, last_output_.linear.y, linear_step);
  output.angular.z = clampStep(output.angular.z, last_output_.angular.z, angular_step);
  return output;
}

void CmdVelMuxNode::publishSelected(const geometry_msgs::Twist& twist, const std::string& source_name) {
  last_output_ = twist;
  cmd_vel_pub_.publish(twist);

  std_msgs::String source_msg;
  source_msg.data = source_name;
  selected_source_pub_.publish(source_msg);
}

void CmdVelMuxNode::publishStop(const std::string& reason) {
  geometry_msgs::Twist stop_cmd;
  last_output_ = stop_cmd;
  cmd_vel_pub_.publish(stop_cmd);

  std_msgs::String source_msg;
  source_msg.data = reason;
  selected_source_pub_.publish(source_msg);
}

}  // namespace motion_planner
