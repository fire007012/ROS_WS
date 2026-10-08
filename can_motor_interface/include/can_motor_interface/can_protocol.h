#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <linux/can.h>
#include <stdexcept>

namespace can_motor_interface {

// SocketCAN distinguishes standard/extended IDs with CAN_EFF_FLAG, even when
// the numeric ID fits into 11 bits. Never silently mask a misconfigured ID.
inline uint32_t socketCanId(uint32_t id, bool extended) {
  if (id > (extended ? CAN_EFF_MASK : CAN_SFF_MASK)) {
    throw std::invalid_argument("CAN ID exceeds the configured frame format");
  }
  return id | (extended ? CAN_EFF_FLAG : 0U);
}

inline bool matchesCanReply(uint32_t raw_id, uint32_t expected_id, bool extended) {
  if (raw_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) return false;
  return ((raw_id & CAN_EFF_FLAG) != 0) == extended &&
      (raw_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK)) == expected_id;
}

// STM32 CAN1 parser: exactly 8 bytes, no payload checksum. The last two bytes
// are acceleration (RPM/s) and synchronous-start flags, not a sum8 checksum.
inline can_frame stm32Command(uint32_t id, bool extended, uint8_t command,
                              uint8_t index, int32_t value = 0,
                              uint8_t param0 = 0, uint8_t param1 = 0) {
  can_frame frame{};
  frame.can_id = socketCanId(id, extended);
  frame.can_dlc = 8;
  frame.data[0] = command;
  frame.data[1] = index;
  const uint32_t raw = static_cast<uint32_t>(value);
  for (unsigned i = 0; i < 4; ++i) frame.data[2 + i] = static_cast<uint8_t>(raw >> (8 * i));
  frame.data[6] = param0;
  frame.data[7] = param1;
  return frame;
}

// The fifth motor is the arm axis. Commands use 3200 pulses/revolution,
// mode=1 absolute coordinate and sync=0 immediate execution.
inline can_frame stm32ArmPosition(double radians) {
  const double pulses = radians * (3200.0 / (2.0 * std::acos(-1.0)));
  if (!std::isfinite(pulses) || pulses < std::numeric_limits<int32_t>::min() ||
      pulses > std::numeric_limits<int32_t>::max())
    throw std::invalid_argument("Arm position is nonfinite or exceeds int32 pulse range");
  return stm32Command(0x100, false, 2, 4, static_cast<int32_t>(std::llround(pulses)), 0, 1);
}

struct Stm32Telemetry {
  uint8_t type = 0;
  uint8_t index = 0;
  uint8_t flags = 0;
  float rpm = 0;
};

inline bool decodeStm32Telemetry(uint8_t dlc, const uint8_t* data, Stm32Telemetry* out) {
  if (!data || !out || dlc != 8 || (data[0] != 0x01 && data[0] != 0x03)) return false;
  out->type = data[0];
  out->index = data[1];
  out->flags = data[2];
  const uint32_t raw = static_cast<uint32_t>(data[2]) |
      (static_cast<uint32_t>(data[3]) << 8) | (static_cast<uint32_t>(data[4]) << 16) |
      (static_cast<uint32_t>(data[5]) << 24);
  out->rpm = static_cast<int32_t>(raw) / 10.0f;
  return true;
}

enum class CanEventType { NONE, PHYSICAL_START, PHYSICAL_ESTOP, HEARTBEAT_TIMEOUT, DRIVER_FAULT };

struct CanEvent {
  CanEventType type = CanEventType::NONE;
  uint8_t detail = 0;
};

inline bool decodeCanEvent(uint32_t can_id, bool is_extended, uint8_t dlc,
                           const uint8_t* data, CanEvent* event) {
  if (!event || !data || is_extended) return false;
  event->type = CanEventType::NONE;
  event->detail = 0;
  if (can_id == 0x112U) {
    if (dlc < 3 || data[1] != 0x01 || data[2] != 0x01) return false;
    if (data[0] == 0x01) event->type = CanEventType::PHYSICAL_START;
    else if (data[0] == 0x02) event->type = CanEventType::PHYSICAL_ESTOP;
    else return false;
    return true;
  }
  if (can_id == 0x101U && dlc == 8 && data[0] == 0x06 && (data[1] < 5 || data[1] == 0xFF)) {
    event->detail = data[2];
    if (data[1] == 0xFF && data[2] == 0x81) event->type = CanEventType::PHYSICAL_ESTOP;
    else if (data[1] == 0xFF && data[2] == 0x80) event->type = CanEventType::HEARTBEAT_TIMEOUT;
    else event->type = CanEventType::DRIVER_FAULT;
    return true;
  }
  return false;
}

}  // namespace can_motor_interface
