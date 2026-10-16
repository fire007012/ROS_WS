#pragma once
#include <cstdint>
#include <linux/can.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
namespace robot_navigation {
// IDs 0..4 retain their firmware mapping; the rear chassis sensor is ID 5.
constexpr uint8_t kStm32RearSensorId = 5;
constexpr uint8_t kStm32RangeSensorCount = 6;
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
         (sample.status & failure_flags) == 0 && sample.distance_mm >= 50 && sample.distance_mm <= 3000;
}

inline bool stm32RangeStatusHasFailure(uint8_t status) {
  return (status & (kStm32RangeTimeout | kStm32RangeI2cError)) != 0;
}

// An acquired sample with no target is normal. Timeout/I2C errors still carry
// acquisition failure semantics and are diagnosed separately from no return.
inline bool stm32RangeNoReturn(const Vl53l1xSample& sample) {
  return sample.distance_mm == 0xFFFF && !(sample.status & kStm32RangeValid) &&
      !stm32RangeStatusHasFailure(sample.status) && !(sample.status & kStm32RangeEmergency);
}

inline uint8_t stm32RangeRadiationType(uint8_t /*sensor_id*/) {
  return sensor_msgs::Range::INFRARED;
}

inline const char* stm32RangeDeviceName(uint8_t sensor_id) {
  switch (sensor_id) {
    case 0: return "VL53L1X ToF front";
    case 1: return "VL53L1X ToF left";
    case 2: return "VL53L1X ToF right";
    case 3: return "VL53L1X ToF arm upper left";
    case 4: return "VL53L1X ToF arm upper right";
    case kStm32RearSensorId: return "VL53L1X ToF rear";
    default: return "unknown range sensor";
  }
}

inline uint8_t vl53l1xSequenceDelta(uint8_t previous, uint8_t current) {
  return static_cast<uint8_t>(current - previous);
}

inline bool vl53l1xSampleFresh(bool has_valid, const ros::SteadyTime& now,
                               const ros::SteadyTime& last_valid, int timeout_ms) {
  return has_valid && timeout_ms > 0 && now >= last_valid && (now - last_valid).toSec() * 1000.0 <= timeout_ms;
}

// Pure protocol helpers are kept separate so they can be tested without CAN hardware.
inline bool parseVl53l1xSample(const struct can_frame& frame, Vl53l1xSample& sample) {
  if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
      frame.can_id != 0x110 || frame.can_dlc != 8 || frame.data[0] != 0x01 ||
      frame.data[1] >= kStm32RangeSensorCount) {
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
      (frame.data[1] != 0xFF && frame.data[1] >= kStm32RangeSensorCount)) {
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

}  // namespace robot_navigation
