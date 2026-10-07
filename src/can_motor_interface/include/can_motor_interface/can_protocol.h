#pragma once

#include <cstdint>

namespace can_motor_interface {

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
  if (can_id == 0x101U && dlc >= 3 && data[0] == 0x06 && data[1] == 0xFF) {
    event->detail = data[2];
    if (data[2] == 0x81) event->type = CanEventType::PHYSICAL_ESTOP;
    else if (data[2] == 0x80) event->type = CanEventType::HEARTBEAT_TIMEOUT;
    else event->type = CanEventType::DRIVER_FAULT;
    return true;
  }
  return false;
}

}  // namespace can_motor_interface
