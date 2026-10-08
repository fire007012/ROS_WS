#include <algorithm>
#include "can_motor_interface/can_protocol.h"
#include "can_motor_interface/link_health.h"

extern "C" void ros_encode_command(uint8_t command, uint8_t index, int32_t value,
                                    uint8_t accel, uint32_t* id, uint8_t* dlc, uint8_t* data) {
  const auto frame = can_motor_interface::stm32Command(0x100, false, command, index, value, accel, 0);
  *id = frame.can_id;
  *dlc = frame.can_dlc;
  std::copy(frame.data, frame.data + 8, data);
}

extern "C" void ros_encode_wheel(uint8_t wheel, int32_t rpm,
                                  uint32_t* id, uint8_t* dlc, uint8_t* data) {
  can_motor_interface::WheelMap map;
  ros_encode_command(1, map.indices.at(wheel), rpm * map.signs.at(wheel), 50, id, dlc, data);
}

extern "C" void ros_encode_arm(double radians, uint32_t* id, uint8_t* dlc, uint8_t* data) {
  const auto frame = can_motor_interface::stm32ArmPosition(radians);
  *id = frame.can_id; *dlc = frame.can_dlc;
  std::copy(frame.data, frame.data + 8, data);
}
