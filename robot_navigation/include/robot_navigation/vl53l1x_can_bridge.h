#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "robot_navigation/can_utils.h"
#include "robot_navigation/upper_alignment.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

namespace robot_navigation {

class Vl53l1xCanBridge {
 public:
  Vl53l1xCanBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~Vl53l1xCanBridge();

  bool init();

 private:
  friend class Vl53l1xBridgeTestPeer;
  bool configure();
  void handleFrame(const struct can_frame& frame);
  struct SensorState {
    std::string name;
    std::string range_topic;
    std::string distance_topic;
    std::string frame_id;
    std::string device_name;
    uint8_t radiation_type = sensor_msgs::Range::INFRARED;
    double min_range = 0.05;
    double max_range = 3.0;
    double field_of_view = 0.47;
    ros::Publisher range_pub;
    ros::Publisher distance_pub;
    ros::Time last_receive;
    ros::SteadyTime last_valid;
    ros::SteadyTime last_receive_steady;
    uint16_t distance_mm = 0xFFFF;
    uint16_t sigma_mm = 0xFFFF;
    uint8_t status = 0;
    uint8_t diagnostic_error_code = 0;
    uint8_t sequence = 0;
    bool has_sequence = false;
    uint8_t diagnostic_sequence = 0;
    bool has_diagnostic_sequence = false;
    bool has_valid = false;
    uint64_t received = 0;
    uint64_t invalid_samples = 0;
    uint64_t dropped_frames = 0;
    uint64_t duplicate_frames = 0;
    uint64_t diagnostic_dropped_frames = 0;
    uint64_t diagnostic_duplicate_frames = 0;
    uint64_t invalid_frames = 0;
    uint64_t consecutive_failures = 0;
  };

  void pollTimerCallback(const ros::SteadyTimerEvent& event);
  void diagnosticsTimerCallback(const ros::SteadyTimerEvent& event);
  void handleSample(const Vl53l1xSample& sample, const ros::Time& stamp);
  void handleDiagnostic(const Vl53l1xDiagnosticFrame& diagnostic, const ros::Time& stamp);
  void publishDiagnostics(const ros::Time& now);
  bool updateSequence(SensorState& state, uint8_t sequence);
  bool updateDiagnosticSequence(SensorState& state, uint8_t sequence);
  void loadSensorParams(SensorState& state, const std::string& prefix,
                        const std::string& default_name,
                        const std::string& default_range_topic,
                        const std::string& default_distance_topic,
                        const std::string& default_frame, double default_min_range,
                        double default_max_range, uint8_t radiation_type,
                        const std::string& device_name);

  void publishRange(SensorState& state, const ros::Time& stamp, bool valid);
  void publishAlignment();
  void stageCallback(const std_msgs::String::ConstPtr& msg);
  void cancelCallback(const std_msgs::Empty::ConstPtr& msg);
  void enabledCallback(const std_msgs::Bool::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  UpperAlignment alignment_;
  bool alignment_enabled_ = false;
  bool actions_enabled_ = false;
  bool estop_latched_ = false;
  std::string stage_;
  std::vector<std::string> alignment_stages_;
  ros::Subscriber stage_sub_, cancel_sub_, enabled_sub_, estop_sub_, reset_sub_;
  ros::Publisher alignment_cmd_pub_, alignment_mode_pub_, alignment_done_pub_, emergency_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  CanInterface can_;
  ros::SteadyTimer poll_timer_;
  ros::SteadyTimer diagnostics_timer_;
  ros::Publisher diagnostics_pub_;
  SensorState sensors_[kStm32RangeSensorCount];
  uint32_t distance_can_id_ = 0x110;
  uint32_t diagnostic_can_id_ = 0x111;
  std::string can_interface_ = "can0";
  int poll_rate_hz_ = 200;
  int sensor_timeout_ms_ = 200;
  uint64_t invalid_frame_count_ = 0;
  uint64_t diagnostic_frame_count_ = 0;
};

}  // namespace robot_navigation
