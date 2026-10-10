#pragma once

#include <cstdint>
#include <string>

#include <linux/can.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "robot_navigation/can_utils.h"

namespace robot_navigation {

// STM32 range-frame status bits.  Keep these tied to the firmware protocol,
// rather than inferring validity from the distance sentinel alone.
constexpr uint8_t kStm32RangeValid = 0x01;
constexpr uint8_t kStm32RangeOutOfRange = 0x02;
constexpr uint8_t kStm32RangeTimeout = 0x04;
constexpr uint8_t kStm32RangeI2cError = 0x08;
constexpr uint8_t kStm32RangeLowQuality = 0x10;
constexpr uint8_t kStm32RangeEmergency = 0x20;

struct Vl53l1xSample {
  uint8_t sensor_id = 0;
  uint16_t distance_mm = 0xFFFF;
  uint16_t sigma_mm = 0xFFFF;
  uint8_t status = 0;
  uint8_t sequence = 0;
};

struct Vl53l1xDiagnosticFrame {
  uint8_t sensor_id = 0xFF;
  uint8_t error_code = 0;
  uint8_t consecutive_errors = 0;
  uint16_t last_valid_distance_mm = 0xFFFF;
  uint8_t last_status = 0;
  uint8_t sequence = 0;
};

inline bool stm32RangeSampleValid(const Vl53l1xSample& sample) {
  const uint8_t failure_flags = kStm32RangeOutOfRange | kStm32RangeTimeout |
                                kStm32RangeI2cError;
  return (sample.status & kStm32RangeValid) != 0 &&
         (sample.status & failure_flags) == 0 && sample.distance_mm != 0xFFFF;
}

inline bool stm32RangeStatusHasFailure(uint8_t status) {
  return (status & (kStm32RangeOutOfRange | kStm32RangeTimeout |
                    kStm32RangeI2cError)) != 0;
}

inline uint8_t stm32RangeRadiationType(uint8_t sensor_id) {
  return sensor_id == 0 ? sensor_msgs::Range::ULTRASOUND : sensor_msgs::Range::INFRARED;
}

inline const char* stm32RangeDeviceName(uint8_t sensor_id) {
  switch (sensor_id) {
    case 0: return "HC-SR04 ultrasonic";
    case 1: return "VL53L1X ToF left";
    case 2: return "VL53L1X ToF right";
    default: return "unknown range sensor";
  }
}

inline uint8_t vl53l1xSequenceDelta(uint8_t previous, uint8_t current) {
  return static_cast<uint8_t>(current - previous);
}

inline bool vl53l1xSampleFresh(bool has_valid, const ros::Time& now,
                               const ros::Time& last_valid, int timeout_ms) {
  return has_valid && timeout_ms > 0 && (now - last_valid).toSec() * 1000.0 <= timeout_ms;
}

// Pure protocol helpers are kept separate so they can be tested without CAN hardware.
inline bool parseVl53l1xSample(const struct can_frame& frame, Vl53l1xSample& sample) {
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
      frame.can_id != 0x110 || frame.can_dlc != 8 || frame.data[0] != 0x01 || frame.data[1] > 2) {
    return false;
  }
  sample.sensor_id = frame.data[1];
  sample.distance_mm = static_cast<uint16_t>(frame.data[2]) |
                       (static_cast<uint16_t>(frame.data[3]) << 8);
  sample.sigma_mm = static_cast<uint16_t>(frame.data[4]) |
                    (static_cast<uint16_t>(frame.data[5]) << 8);
  sample.status = frame.data[6];
  sample.sequence = frame.data[7];
  return true;
}

inline bool parseVl53l1xDiagnostic(const struct can_frame& frame, Vl53l1xDiagnosticFrame& diagnostic) {
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
      frame.can_id != 0x111 || frame.can_dlc != 8 || frame.data[0] != 0x01 ||
      (frame.data[1] != 0xFF && frame.data[1] > 2)) {
    return false;
  }
  diagnostic.sensor_id = frame.data[1];
  diagnostic.error_code = frame.data[2];
  diagnostic.consecutive_errors = frame.data[3];
  diagnostic.last_valid_distance_mm = static_cast<uint16_t>(frame.data[4]) |
                                      (static_cast<uint16_t>(frame.data[5]) << 8);
  diagnostic.last_status = frame.data[6];
  diagnostic.sequence = frame.data[7];
  return true;
}

class Vl53l1xCanBridge {
 public:
  Vl53l1xCanBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~Vl53l1xCanBridge();

  bool init();

 private:
  struct SensorState {
    std::string name;
    std::string range_topic;
    std::string distance_topic;
    std::string frame_id;
    std::string device_name;
    uint8_t radiation_type = sensor_msgs::Range::INFRARED;
    double min_range = 0.05;
    double max_range = 4.0;
    double field_of_view = 0.47;
    ros::Publisher range_pub;
    ros::Publisher distance_pub;
    ros::Time last_receive;
    ros::Time last_valid;
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

  void pollTimerCallback(const ros::TimerEvent& event);
  void diagnosticsTimerCallback(const ros::TimerEvent& event);
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  CanInterface can_;
  ros::Timer poll_timer_;
  ros::Timer diagnostics_timer_;
  ros::Publisher diagnostics_pub_;
  SensorState sensors_[3];
  uint32_t distance_can_id_ = 0x110;
  uint32_t diagnostic_can_id_ = 0x111;
  std::string can_interface_ = "can0";
  int poll_rate_hz_ = 200;
  int sensor_timeout_ms_ = 200;
  uint64_t invalid_frame_count_ = 0;
  uint64_t diagnostic_frame_count_ = 0;
};

}  // namespace robot_navigation
