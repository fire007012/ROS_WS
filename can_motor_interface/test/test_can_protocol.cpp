#include <gtest/gtest.h>
#include <array>
#include <condition_variable>
#include <future>
#include <thread>
#include "can_motor_interface/can_protocol.h"
#include "can_motor_interface/link_health.h"

using can_motor_interface::CanEvent;
using can_motor_interface::CanEventType;
using can_motor_interface::decodeCanEvent;
using can_motor_interface::socketCanId;
using can_motor_interface::matchesCanReply;
using can_motor_interface::stm32Command;

TEST(CanProtocol, CommandMatchesStm32FixedEightByteParser) {
  const auto frame = stm32Command(0x100, false, 1, 0, 100, 50, 0);
  const std::array<uint8_t, 8> expected{{1, 0, 0x64, 0, 0, 0, 0x32, 0}};
  EXPECT_EQ(0x100U, frame.can_id);
  EXPECT_EQ(8, frame.can_dlc);
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), frame.data));
  const auto reverse = stm32Command(0x100, false, 1, 3, -100, 50, 0);
  const std::array<uint8_t, 8> negative{{1, 3, 0x9C, 0xFF, 0xFF, 0xFF, 0x32, 0}};
  EXPECT_TRUE(std::equal(negative.begin(), negative.end(), reverse.data));
  const auto heartbeat = stm32Command(0x100, false, 9, 0xFF);
  EXPECT_EQ(8, heartbeat.can_dlc);
  EXPECT_EQ(9, heartbeat.data[0]);
  const auto reports = stm32Command(0x100, false, 7, 0xFF, 7);
  EXPECT_EQ(7, reports.data[2]);
}

TEST(CanProtocol, FeedbackUsesActualVelocitySubframeAndTenthsOfRpm) {
  can_motor_interface::Stm32Telemetry telemetry;
  const std::array<uint8_t, 8> speed{{3, 2, 0x18, 0xFC, 0xFF, 0xFF, 0, 0}}; // -1000 = -100 RPM
  EXPECT_TRUE(can_motor_interface::decodeStm32Telemetry(8, speed.data(), &telemetry));
  EXPECT_EQ(2, telemetry.index);
  EXPECT_FLOAT_EQ(-100.0f, telemetry.rpm);
  const std::array<uint8_t, 8> status{{1, 1, 0x83, 0, 0, 0, 0, 0}};
  EXPECT_TRUE(can_motor_interface::decodeStm32Telemetry(8, status.data(), &telemetry));
  EXPECT_EQ(0x83, telemetry.flags);
  EXPECT_FALSE(can_motor_interface::decodeStm32Telemetry(7, speed.data(), &telemetry));
  const std::array<uint8_t, 8> target{{4, 1, 0, 0, 0, 0, 0, 0}};
  EXPECT_FALSE(can_motor_interface::decodeStm32Telemetry(8, target.data(), &telemetry));
}

TEST(CanProtocol, PerMotorDriverFaultsAreNotIgnored) {
  CanEvent event;
  const std::array<uint8_t, 8> fault{{6, 2, 0x89, 0, 0, 0, 0, 0}};
  EXPECT_TRUE(decodeCanEvent(0x101, false, 8, fault.data(), &event));
  EXPECT_EQ(CanEventType::DRIVER_FAULT, event.type);
  EXPECT_FALSE(decodeCanEvent(0x101, false, 7, fault.data(), &event));
}

TEST(CanProtocol, ExtendedIdMustKeepIdeBitEvenForSmallNumericIds) {
  EXPECT_EQ(0x201U, socketCanId(0x201, false));
  EXPECT_EQ(CAN_EFF_FLAG | 0x201U, socketCanId(0x201, true));
  EXPECT_EQ(CAN_EFF_FLAG | CAN_EFF_MASK, socketCanId(CAN_EFF_MASK, true));
  EXPECT_THROW(socketCanId(0x800, false), std::invalid_argument);
  EXPECT_THROW(socketCanId(CAN_EFF_MASK + 1U, true), std::invalid_argument);
  EXPECT_THROW(socketCanId(0xFFFFFFFFU, false), std::invalid_argument);
}

TEST(CanProtocol, HeartbeatAcceptsConfiguredExtendedReplies) {
  EXPECT_TRUE(matchesCanReply(CAN_EFF_FLAG | 0x181, 0x181, true));
  EXPECT_FALSE(matchesCanReply(0x181, 0x181, true));
  EXPECT_FALSE(matchesCanReply(CAN_EFF_FLAG | 0x181, 0x181, false));
  EXPECT_TRUE(matchesCanReply(0x101, 0x101, false));
  EXPECT_FALSE(matchesCanReply(CAN_EFF_FLAG | 0x101, 0x101, false));
  EXPECT_FALSE(matchesCanReply(CAN_EFF_FLAG | 0x182, 0x181, true));
  EXPECT_FALSE(matchesCanReply(CAN_RTR_FLAG | 0x101, 0x101, false));
  EXPECT_FALSE(matchesCanReply(CAN_ERR_FLAG | 0x101, 0x101, false));
}

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

TEST(LinkHealth, NativeRepliesAreStrictAndUseTenthsRpm) {
  using namespace can_motor_interface;
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | 0x200;
  frame.can_dlc = 5;
  std::copy_n(std::array<uint8_t, 5>{{0x35, 1, 3, 0xE8, 0x6B}}.begin(), 5, frame.data);
  DriverTelemetry reply;
  ASSERT_TRUE(decodeDriverTelemetry(frame, &reply));
  EXPECT_EQ(2, reply.address);
  EXPECT_FLOAT_EQ(-100, reply.rpm);
  frame.can_id = 0x200;
  EXPECT_FALSE(decodeDriverTelemetry(frame, &reply));
  frame.can_id = CAN_EFF_FLAG | 0x201;
  EXPECT_FALSE(decodeDriverTelemetry(frame, &reply));
  frame.can_id = CAN_EFF_FLAG | 0x200;
  frame.data[4] = 0;
  EXPECT_FALSE(decodeDriverTelemetry(frame, &reply));
  frame = driverRead(2, 0x35);
  EXPECT_EQ(CAN_EFF_FLAG | 0x200U, frame.can_id);
  EXPECT_EQ(2, frame.can_dlc);
  EXPECT_FALSE(decodeDriverTelemetry(frame, &reply)); // local TX query is not a sample
  frame.can_id = 0x101; frame.can_dlc = 8; frame.data[0] = 3;
  EXPECT_FALSE(decodeDriverTelemetry(frame, &reply)); // STM32 cached speed
}

TEST(LinkHealth, AllFourRealDriversMustStayFreshAndEnabled) {
  using namespace can_motor_interface;
  DriverHealth health;
  EXPECT_FALSE(health.healthy(10, 1));
  DriverTelemetry speed; speed.function = 0x35;
  DriverTelemetry status; status.function = 0x3A; status.flags = 0x83;
  for (size_t i = 0; i < 4; ++i) {
    health.observe(i, speed, 10); health.observe(i, status, 10);
  }
  EXPECT_TRUE(health.healthy(10.5, 1)); // bit7 is power-cycle history
  for (size_t i = 0; i < 3; ++i) {
    health.observe(i, speed, 11); health.observe(i, status, 11);
  }
  EXPECT_FALSE(health.healthy(11.1, 1)); // one missing wheel cannot be hidden by others
  health.observe(3, speed, 11); health.observe(3, status, 11);
  EXPECT_TRUE(health.healthy(11.1, 1));
  status.flags = 0x8B;
  health.observe(2, status, 11.1);
  EXPECT_FALSE(health.healthy(11.1, 1));
  status.flags = 0x82;
  health.observe(2, status, 11.1);
  EXPECT_FALSE(health.healthy(11.1, 1)); // disabled
}

TEST(LinkHealth, FirmwareFailureCountersDetectChangesAndWrap) {
  using namespace can_motor_interface;
  FirmwareStats stats;
  can_frame frame{}; frame.can_id = 0x103; frame.can_dlc = 8;
  for (int type = 1; type <= 3; ++type) {
    frame.data[0] = type;
    EXPECT_FALSE(stats.observe(frame));
  }
  EXPECT_TRUE(stats.ready());
  for (int type = 1; type <= 3; ++type) {
    frame.data[0] = type; frame.data[3] = 1;
    EXPECT_TRUE(stats.observe(frame));
    EXPECT_FALSE(stats.observe(frame));
    frame.data[3] = 0;
  }
  frame.data[0] = 2; frame.data[7] = 1;
  EXPECT_TRUE(stats.observe(frame));
  FirmwareStats wrapping;
  frame = {}; frame.can_id = 0x103; frame.can_dlc = 8; frame.data[0] = 1;
  frame.data[5] = frame.data[6] = 0xFF;
  EXPECT_FALSE(wrapping.observe(frame));
  frame.data[5] = frame.data[6] = 0;
  EXPECT_FALSE(wrapping.observe(frame));
  EXPECT_TRUE(wrapping.responseTimeoutChanged());
}

TEST(LinkHealth, MissingOptionalAxisDoesNotHideChassisMotionAckLoss) {
  using namespace can_motor_interface;
  FirmwareStats stats;
  can_frame frame{}; frame.can_id = 0x103; frame.can_dlc = 8; frame.data[0] = 1;
  EXPECT_FALSE(stats.observe(frame));
  frame.data[5] = 1; // fifth-axis background read timeout, no index in statistics
  EXPECT_FALSE(stats.observe(frame));
  EXPECT_TRUE(stats.responseTimeoutChanged());
  frame.data[3] = 1; // queue drop remains a hard failure
  EXPECT_TRUE(stats.observe(frame));
  MotionAckHealth acks;
  EXPECT_TRUE(acks.healthy(10, 0.5)); // no motion submitted
  for (size_t wheel = 0; wheel < 4; ++wheel) acks.sent(wheel, 10);
  EXPECT_TRUE(acks.healthy(10.4, 0.5));
  for (size_t wheel = 0; wheel < 3; ++wheel) acks.acknowledge(wheel, 10.5);
  EXPECT_FALSE(acks.healthy(10.6, 0.5)); // fourth driver reads alone cannot hide missing F6 ACK
  acks.acknowledge(3, 10.5);
  EXPECT_TRUE(acks.healthy(10.6, 0.5));
  for (size_t wheel = 0; wheel < 4; ++wheel) acks.sent(wheel, 11);
  EXPECT_FALSE(acks.healthy(11.1, 0.5)); // repeated TX never extends the ACK deadline
}

TEST(LinkHealth, WheelPermutationAndSignsAreSymmetric) {
  using namespace can_motor_interface;
  WheelMap map;
  EXPECT_NO_THROW(map.validate());
  EXPECT_EQ((std::vector<int>{1, 0, 3, 2}), map.indices);
  map.signs = {-1, 1, 1, -1};
  for (int wheel = 0; wheel < 4; ++wheel) {
    const float wire_rpm = 30 * map.signs[wheel];
    EXPECT_EQ(wheel, map.logicalIndex(map.indices[wheel]));
    EXPECT_FLOAT_EQ(30, wire_rpm * map.signs[wheel]);
  }
  map.indices = {0, 1, 2, 4}; EXPECT_THROW(map.validate(), std::invalid_argument);
  map.indices = {0, 0, 2, 3}; EXPECT_THROW(map.validate(), std::invalid_argument);
  map.indices = {0, 1, 2, 3}; map.signs[0] = 0; EXPECT_THROW(map.validate(), std::invalid_argument);
}

TEST(LinkHealth, StopCannotBeOvertakenByAnotherNonzeroMotionWrite) {
  using namespace can_motor_interface;
  MotionInterlock interlock;
  std::promise<void> writing, finish;
  auto release = finish.get_future();
  std::vector<int> writes;
  auto sender = std::async(std::launch::async, [&] {
    interlock.run([&] {
      writing.set_value(); release.wait(); writes.push_back(1); return true;
    });
  });
  writing.get_future().wait();
  auto stopper = std::async(std::launch::async, [&] { interlock.stop([&] { writes.push_back(3); }); });
  // The latch is set before STOP waits for the active write's mutex.
  while (!interlock.latched()) std::this_thread::yield();
  finish.set_value();
  sender.get(); stopper.get();
  EXPECT_TRUE(interlock.run([&] { writes.push_back(1); return true; }));
  EXPECT_EQ((std::vector<int>{1, 3}), writes);
  EXPECT_FALSE(interlock.reset([] { return false; }));
  EXPECT_TRUE(interlock.latched());
  EXPECT_TRUE(interlock.reset([] { return true; }));
  interlock.run([&] { writes.push_back(1); return true; });
  EXPECT_EQ((std::vector<int>{1, 3, 1}), writes);
}

TEST(CanProtocol, ArmUsesFifthMotorAbsolutePulsePosition) {
  using namespace can_motor_interface;
  const auto position = stm32ArmPosition(std::acos(-1.0));
  EXPECT_EQ(0x100U, position.can_id);
  const std::array<uint8_t, 8> expected{{2, 4, 0x40, 6, 0, 0, 0, 1}};
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), position.data)); // pi rad = 1600 pulses
  EXPECT_THROW(stm32ArmPosition(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(stm32ArmPosition(1e20), std::invalid_argument);
}
