#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

namespace robot_navigation {

/**
 * @brief 距离安全检测节点 — 持续占用安全通道的近距离制动层
 *
 * 订阅前向 VL53L1X 距离数据和当前底盘速度，在障碍物过近时通过
 * /cmd_vel_safety 通道（cmd_vel_mux 最高优先级）下发：
 *   1. 减速/停止指令（基础安全）
 *
 * 三个单点 ToF 不能构建可靠的局部障碍地图，因此本节点只做失效即停的
 * 近距离安全制动。需要绕障时由 move_base 的局部规划器使用 /scan 完成。
 */
class DistanceSafetyNode {
 public:
  DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~DistanceSafetyNode() = default;

  bool init();

 private:
  // ── 回调 ──
  void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void safetyTimerCallback(const ros::TimerEvent& event);

  // ── 工具 ──
  void publishSafetyStop();
  void publishSafetyReverse(double reverse_speed);
  void publishSafetyClear();

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber distance_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher  cmd_vel_safety_pub_;
  ros::Publisher  safety_state_pub_;
  ros::Timer      safety_timer_;

  // ── 参数 ──
  double safety_distance_mm_;       // 安全停止距离 (mm)
  double critical_distance_mm_;     // 紧急后退距离 (mm)
  double reverse_speed_;            // 后退速度 (m/s)
  bool   allow_emergency_reverse_;  // 允许在无后向测距时紧急后退
  double braking_deceleration_mps2_;
  double reaction_time_sec_;
  double stop_margin_mm_;
  double distance_timeout_sec_;     // 传感器数据超时
  double control_rate_hz_;          // 控制频率
  bool   check_forward_only_;       // 仅检测前方

  // ── 状态 ──
  double current_distance_mm_;
  double current_vx_;
  double current_vy_;
  bool   has_distance_;
  bool   has_cmd_vel_;
  ros::Time last_distance_time_;

  // ── 安全状态 ──
  enum class SafetyState {
    CLEAR,      // 安全，距离足够
    WARNING,    // 距离接近安全阈值，减速
    STOP,       // 距离过近，停止
    REVERSE,    // 距离极近，后退
    TIMEOUT     // 传感器数据超时
  };
  SafetyState safety_state_;

};

}  // namespace robot_navigation
