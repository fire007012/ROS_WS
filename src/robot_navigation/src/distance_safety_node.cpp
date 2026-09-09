/**
 * @file distance_safety_node.cpp
 * @brief 距离安全检测节点 — 障碍物过近或传感器失效时自动制动
 *
 * 通过 cmd_vel_mux 的 safety 通道（最高优先级）介入底盘控制，
 * 在 VL53L1X 检测到障碍物距离过近时持续占用安全通道，直到风险解除。
 */

#include "robot_navigation/distance_safety_node.h"

#include <cmath>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
DistanceSafetyNode::DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , safety_distance_mm_(300.0)
    , critical_distance_mm_(100.0)
    , reverse_speed_(0.05)
    , allow_emergency_reverse_(false)
    , braking_deceleration_mps2_(1.0)
    , reaction_time_sec_(0.2)
    , stop_margin_mm_(300.0)
    , distance_timeout_sec_(0.5)
    , control_rate_hz_(20.0)
    , check_forward_only_(true)
    , current_distance_mm_(0.0)
    , current_vx_(0.0)
    , current_vy_(0.0)
    , has_distance_(false)
    , has_cmd_vel_(false)
    , last_distance_time_(ros::Time::now())
    , safety_state_(SafetyState::CLEAR)
{}

// ── 初始化 ─────────────────────────────────────────────────
bool DistanceSafetyNode::init() {
  // ── 参数读取 ──
  pnh_.param<double>("safety_distance_mm", safety_distance_mm_, 300.0);
  pnh_.param<double>("critical_distance_mm", critical_distance_mm_, 100.0);
  pnh_.param<double>("reverse_speed", reverse_speed_, 0.05);
  pnh_.param<bool>("allow_emergency_reverse", allow_emergency_reverse_, false);
  pnh_.param<double>("braking_deceleration_mps2", braking_deceleration_mps2_, 1.0);
  pnh_.param<double>("reaction_time_sec", reaction_time_sec_, 0.2);
  pnh_.param<double>("stop_margin_mm", stop_margin_mm_, 300.0);
  pnh_.param<double>("distance_timeout_sec", distance_timeout_sec_, 0.5);
  pnh_.param<double>("control_rate_hz", control_rate_hz_, 20.0);
  pnh_.param<bool>("check_forward_only", check_forward_only_, true);

  // 确保 threshold 大小逻辑正确
  if (critical_distance_mm_ >= safety_distance_mm_) {
    ROS_WARN("[distance_safety] critical_distance_mm (%.0f) >= safety_distance_mm (%.0f), "
             "自动调整 critical = safety * 0.33",
             critical_distance_mm_, safety_distance_mm_);
    critical_distance_mm_ = safety_distance_mm_ * 0.33;
  }
  braking_deceleration_mps2_ = std::max(0.01, braking_deceleration_mps2_);
  reaction_time_sec_ = std::max(0.0, reaction_time_sec_);
  stop_margin_mm_ = std::max(0.0, stop_margin_mm_);

  // ── 订阅 ──
  distance_sub_ = nh_.subscribe("/vl53l1x_distance", 10,
                                &DistanceSafetyNode::distanceCallback, this);

  // 订阅最终下发的 cmd_vel（经过 mux 后），用于判断运动方向
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10,
                               &DistanceSafetyNode::cmdVelCallback, this);

  // ── 发布 ──
  // 发布到 cmd_vel_mux 的 safety 通道（最高优先级）
  cmd_vel_safety_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_safety", 10);
  safety_state_pub_   = nh_.advertise<std_msgs::String>("/distance_safety/state", 10, true);

  // ── 定时器 ──
  const double period = 1.0 / std::max(1.0, control_rate_hz_);
  safety_timer_ = nh_.createTimer(ros::Duration(period),
                                  &DistanceSafetyNode::safetyTimerCallback, this);

  // ── 初始发布安全状态 ──
  {
    std_msgs::String init_state;
    init_state.data = "CLEAR";
    safety_state_pub_.publish(init_state);
  }
  // 初始发布一帧空指令，让 mux 知道 safety 通道存在
  publishSafetyClear();

  ROS_INFO("[distance_safety] 初始化完成");
  ROS_INFO("[distance_safety]   安全停止: %.0f mm, 紧急后退: %.0f mm, 后退速度: %.2f m/s",
           safety_distance_mm_, critical_distance_mm_, reverse_speed_);
  ROS_INFO("[distance_safety]   传感器超时: %.1f s, 控制频率: %.1f Hz",
           distance_timeout_sec_, control_rate_hz_);
  ROS_INFO("[distance_safety]   模式: %s", check_forward_only_ ? "仅检测前方" : "全方向检测");
  return true;
}

// ── 距离回调 ───────────────────────────────────────────────
void DistanceSafetyNode::distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_distance_mm_ = static_cast<double>(msg->data);
  has_distance_ = true;
  last_distance_time_ = ros::Time::now();
}

// ── cmd_vel 回调（用于判断运动方向）────────────────────────
void DistanceSafetyNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  current_vx_ = msg->linear.x;
  current_vy_ = msg->linear.y;
  has_cmd_vel_ = true;
}

// ── 安全检测定时器 ─────────────────────────────────────────
void DistanceSafetyNode::safetyTimerCallback(const ros::TimerEvent& /*event*/) {
  const ros::Time now = ros::Time::now();

  // ── 检查传感器超时 ──
  double time_since_last = (now - last_distance_time_).toSec();
  if (!has_distance_ || time_since_last > distance_timeout_sec_) {
    if (safety_state_ != SafetyState::TIMEOUT) {
      safety_state_ = SafetyState::TIMEOUT;
      std_msgs::String state;
      state.data = "TIMEOUT";
      safety_state_pub_.publish(state);
      ROS_WARN_THROTTLE(1.0, "[distance_safety] 传感器数据超时 (%.1f s)，触发安全停止",
                        time_since_last);
    }
    // Refresh every cycle so the mux cannot time out the safety source.
    publishSafetyStop();
    return;
  }

  // ── 方向判断 ──
  double effective_vx = current_vx_;
  if (!has_cmd_vel_) {
    // 尚无 cmd_vel 数据，保守假设静止
    effective_vx = 0.0;
  }

  // 仅检测前方时，如果机器人不向前则不干预
  if (check_forward_only_ && effective_vx <= 0.001) {
    if (safety_state_ != SafetyState::CLEAR) {
      safety_state_ = SafetyState::CLEAR;
      publishSafetyClear();
      std_msgs::String state;
      state.data = "CLEAR";
      safety_state_pub_.publish(state);
    }
    return;
  }

  // ── 距离判定 ──
  // The base threshold is a floor.  At higher speed, use a conservative
  // stopping-distance estimate so the commanded stop is issued early enough.
  const double forward_speed = std::max(0.0, effective_vx);
  const double stopping_distance_mm = 1000.0 * (
      forward_speed * forward_speed / (2.0 * braking_deceleration_mps2_) +
      forward_speed * reaction_time_sec_) + stop_margin_mm_;
  const double effective_safety_distance_mm =
      std::max(safety_distance_mm_, stopping_distance_mm);
  if (current_distance_mm_ <= critical_distance_mm_) {
    // A rear sensor is not present, so reverse is opt-in rather than the
    // default response to a front emergency.
    const SafetyState next_state = allow_emergency_reverse_ ? SafetyState::REVERSE
                                                             : SafetyState::STOP;
    if (safety_state_ != next_state) {
      safety_state_ = next_state;
      std_msgs::String state;
      state.data = allow_emergency_reverse_ ? "REVERSE" : "STOP";
      safety_state_pub_.publish(state);
      ROS_WARN("[distance_safety] 紧急制动: 距离 %.0f mm < %.0f mm",
               current_distance_mm_, critical_distance_mm_);
    }
    if (allow_emergency_reverse_) publishSafetyReverse(reverse_speed_);
    else publishSafetyStop();
  } else if (current_distance_mm_ <= effective_safety_distance_mm) {
    // Keep refreshing the stop command while blocked.
    if (safety_state_ != SafetyState::STOP) {
      safety_state_ = SafetyState::STOP;
      std_msgs::String state;
      state.data = "STOP";
      safety_state_pub_.publish(state);
      ROS_WARN("[distance_safety] ⛔ 安全停止！距离 %.0f mm < %.0f mm",
               current_distance_mm_, effective_safety_distance_mm);
    }
    publishSafetyStop();
  } else if (current_distance_mm_ <= effective_safety_distance_mm * 1.5) {
    // Keep refreshing the reduced command while warning is active.
    if (safety_state_ != SafetyState::WARNING) {
      safety_state_ = SafetyState::WARNING;
      std_msgs::String state;
      state.data = "WARNING";
      safety_state_pub_.publish(state);
    }
    geometry_msgs::Twist slow;
    slow.linear.x = current_vx_ * 0.3;
    slow.linear.y = current_vy_ * 0.3;
    slow.angular.z = 0.0;
    cmd_vel_safety_pub_.publish(slow);
  } else {
    // 安全，清除干预
    if (safety_state_ != SafetyState::CLEAR) {
      safety_state_ = SafetyState::CLEAR;
      publishSafetyClear();
      std_msgs::String state;
      state.data = "CLEAR";
      safety_state_pub_.publish(state);
    }
  }
}

// ── 安全停止 ───────────────────────────────────────────────
void DistanceSafetyNode::publishSafetyStop() {
  geometry_msgs::Twist stop;
  stop.linear.x = 0.0;
  stop.linear.y = 0.0;
  stop.angular.z = 0.0;
  cmd_vel_safety_pub_.publish(stop);
}

// ── 安全后退 ───────────────────────────────────────────────
void DistanceSafetyNode::publishSafetyReverse(double reverse_speed) {
  geometry_msgs::Twist reverse;
  reverse.linear.x = -std::abs(reverse_speed);
  reverse.linear.y = 0.0;
  reverse.angular.z = 0.0;
  cmd_vel_safety_pub_.publish(reverse);
}

// ── 清除安全干预 ───────────────────────────────────────────
void DistanceSafetyNode::publishSafetyClear() {
  // 发布一个"空"Twist，让 mux 知道 safety 通道无有效指令
  // mux 的 sourceActive 依赖超时判断 — 发布零速度让 mux 知道 safety 通道存在
  // 但不要在 safety CLEAR 时持续发布零速度覆盖其他通道！
  // 解决方案：发布一次速度，然后依赖 mux 的 cmd_timeout_sec 超时回退
  geometry_msgs::Twist clear;
  clear.linear.x = 0.0;
  clear.linear.y = 0.0;
  clear.angular.z = 0.0;
  // 仅发布一次就够了 — mux 的 sourceActive 检查 has_msg 和时间戳
  // 发布零速度后，mux 会切换到其他通道
  cmd_vel_safety_pub_.publish(clear);
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "distance_safety_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::DistanceSafetyNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[distance_safety] 初始化失败");
    return 1;
  }

  ROS_INFO("[distance_safety] 距离安全检测节点运行中...");
  ros::spin();

  return 0;
}
