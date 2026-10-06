#include <gtest/gtest.h>
#include <array>
#include "can_motor_interface/can_protocol.h"

using can_motor_interface::CanEvent;
using can_motor_interface::CanEventType;
using can_motor_interface::decodeCanEvent;

TEST(CanProtocol, PhysicalStartRequiresButtonSourceAndPress) {
  const std::array<uint8_t, 8> data{{0x01, 0x01, 0x01, 0, 0, 0, 0, 0}};
  CanEvent event;
  EXPECT_TRUE(decodeCanEvent(0x112, false, 8, data.data(), &event));
  EXPECT_EQ(CanEventType::PHYSICAL_START, event.type);
  EXPECT_FALSE(decodeCanEvent(0x112, true, 8, data.data(), &event));
  const std::array<uint8_t, 8> invalid{{0x01, 0x00, 0x01, 0, 0, 0, 0, 0}};
  EXPECT_FALSE(decodeCanEvent(0x112, false, 8, invalid.data(), &event));
}

TEST(CanProtocol, StatusSafetyReasonsAreDistinguished) {
  const std::array<uint8_t, 8> physical{{0x06, 0xFF, 0x81, 0, 0, 0, 0, 0}};
  const std::array<uint8_t, 8> heartbeat{{0x06, 0xFF, 0x80, 0, 0, 0, 0, 0}};
  const std::array<uint8_t, 8> fault{{0x06, 0xFF, 0x42, 0, 0, 0, 0, 0}};
  CanEvent event;
  ASSERT_TRUE(decodeCanEvent(0x101, false, 8, physical.data(), &event));
  EXPECT_EQ(CanEventType::PHYSICAL_ESTOP, event.type);
  ASSERT_TRUE(decodeCanEvent(0x101, false, 8, heartbeat.data(), &event));
  EXPECT_EQ(CanEventType::HEARTBEAT_TIMEOUT, event.type);
  ASSERT_TRUE(decodeCanEvent(0x101, false, 8, fault.data(), &event));
  EXPECT_EQ(CanEventType::DRIVER_FAULT, event.type);
  EXPECT_EQ(0x42, event.detail);
}
