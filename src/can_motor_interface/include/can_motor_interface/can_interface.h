#pragma once

#include <atomic>
#include <mutex>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <can_msgs/Frame.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include "can_motor_interface/can_protocol.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>

namespace can_motor_interface {

class CanInterfaceNode {
 public:
  CanInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~CanInterfaceNode();

 private:
  void cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void commandTimerCallback(const ros::TimerEvent& event);
  void timerCallback(const ros::TimerEvent& event);
  void canReceiveThread();
  void softwareEstopCallback(const std_msgs::Bool::ConstPtr& msg);
  void emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg);
  void remoteStartCallback(const std_msgs::Empty::ConstPtr& msg);

  bool openSocket();
  void closeSocket();
  bool sendSpeedCommand(uint8_t motor_index, float target_rpm);
  bool sendControlCommand(uint8_t command);
  bool sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  uint8_t calcChecksum(const uint8_t* data, uint8_t len) const;
  void parseStatusFrame(const can_msgs::Frame& frame);
  void parseSafetyEvent(const can_msgs::Frame& frame);
  void handleCanEvent(const CanEvent& event);
  bool isDuplicateEvent(uint8_t key, const ros::Time& now);
  void triggerEmergencyStop(const char* reason);
  void publishStopSignals();
  void publishTelemetry();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber software_estop_sub_;
  ros::Subscriber emergency_reset_sub_;
  ros::Subscriber remote_start_sub_;
  ros::Publisher motor_state_pub_;
  ros::Publisher motor_status_flag_pub_;
  ros::Publisher emergency_stop_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher display_pub_;
  ros::Publisher stop_all_pub_;
  ros::Publisher chassis_lock_pub_;
  ros::Publisher fixed_route_hold_pub_;
  ros::Publisher move_base_cancel_pub_;
  ros::Publisher can_rx_pub_;
  ros::Timer monitor_timer_;
  ros::Timer command_timer_;

  std::string can_device_;
  int socket_fd_;
  std::atomic<bool> running_;
  std::atomic<bool> estop_latched_;
  std::thread rx_thread_;

  std::vector<int> motor_ids_;
  std::vector<float> motor_state_rpm_;
  std::vector<uint8_t> motor_status_flags_;
  std::mutex telemetry_mutex_;

  std::vector<float> target_rpm_;
  ros::Time last_command_time_;
  bool have_command_;
  std::mutex command_mutex_;

  uint32_t tx_can_id_;
  uint32_t rx_can_id_;
  bool rx_can_id_filter_enable_;
  bool use_extended_frame_;
  bool payload_little_endian_;
  bool checksum_use_sum8_;

  uint8_t speed_cmd_code_;
  uint8_t status_cmd_code_;
  uint8_t broadcast_index_;
  float max_rpm_;
  int motor_count_;

  double heartbeat_timeout_sec_;
  double event_dedup_window_sec_;
  double command_publish_rate_hz_;
  double command_timeout_sec_;
  ros::Time last_rx_time_;
  std::mutex event_mutex_;
  std::map<uint8_t, ros::Time> last_event_times_;
};

}  // namespace can_motor_interface
