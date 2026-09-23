#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <map>
#include <string>
#include <vector>

namespace robot_navigation {

/**
 * @brief 系统健康监控节点
 *
 * 功能：
 *   1. 监听关键节点的 /stop_all 信号并转发紧急停止
 *   2. 定期汇总系统状态发布到 /system_health
 *   3. 提供 /system_health/reset 服务用于重置监控状态
 *   4. 监听各节点心跳（可选）
 *
 * 发布话题:
 *   /system_health      (String):  系统状态报告
 *   /system_health/ok   (Bool):    系统是否正常
 *
 * 服务:
 *   /system_health/reset (Trigger): 重置监控
 */
class HealthMonitor {
 public:
  HealthMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~HealthMonitor() = default;

  bool init();

 private:
  void stopAllCallback(const std_msgs::Empty::ConstPtr& msg);
  void healthTimerCallback(const ros::TimerEvent& event);
  bool resetCallback(std_srvs::Trigger::Request& req,
                     std_srvs::Trigger::Response& res);

  std::string generateHealthReport() const;
  void checkCriticalTopics();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void frontRangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void leftRangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void rightRangeCallback(const sensor_msgs::Range::ConstPtr& msg);
  void pathCallback(const std_msgs::Bool::ConstPtr& msg);
  void motorStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 订阅 /stop_all（转发给所有需要知道的组件）
  ros::Subscriber stop_all_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber front_range_sub_;
  ros::Subscriber left_range_sub_;
  ros::Subscriber right_range_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber motor_state_sub_;

  // 发布
  ros::Publisher health_pub_;
  ros::Publisher health_ok_pub_;
  ros::Publisher emergency_stop_pub_;

  // 服务
  ros::ServiceServer reset_srv_;

  // 定时器
  ros::Timer health_timer_;

  // 参数
  double publish_rate_hz_;
  std::vector<std::string> critical_topics_;

  // 状态
  struct TopicStatus {
    bool seen;
    ros::Time last_seen;
    double timeout_sec;
  };
  std::map<std::string, TopicStatus> topic_status_;
  std::map<std::string, ros::Time> last_message_time_;
  bool system_ok_;
  int consecutive_failures_;
  int max_consecutive_failures_;
};

}  // namespace robot_navigation
