#include <gtest/gtest.h>
#include <algorithm>
#include <initializer_list>

#include "robot_navigation/vl53l1x_can_bridge.h"

namespace {
struct can_frame makeFrame(uint32_t id, std::initializer_list<uint8_t> data) {
  struct can_frame frame{};
  frame.can_id = id;
  frame.can_dlc = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), frame.data);
  return frame;
}
}

TEST(Vl53l1xCanProtocol, DecodesLittleEndianAndRoutesSensors) {
  for (uint8_t id = 0; id < 5; ++id) {
    const auto frame = makeFrame(0x110, {0x01, id, 0x52, 0x03, 0x0A, 0x00, 0x01, id});
    robot_navigation::Vl53l1xSample sample;
    ASSERT_TRUE(robot_navigation::parseVl53l1xSample(frame, sample));
    EXPECT_EQ(id, sample.sensor_id);
    EXPECT_EQ(850, sample.distance_mm);
    EXPECT_EQ(10, sample.sigma_mm);
  }
}

TEST(Vl53l1xCanProtocol, RejectsInvalidFramesAndStatusIsPreserved) {
  robot_navigation::Vl53l1xSample sample;
  EXPECT_FALSE(robot_navigation::parseVl53l1xSample(makeFrame(0x110, {0x02, 0, 1, 0, 0, 0, 1, 0}), sample));
  EXPECT_FALSE(robot_navigation::parseVl53l1xSample(makeFrame(0x110, {0x01, 5, 1, 0, 0, 0, 1, 0}), sample));
  EXPECT_FALSE(robot_navigation::parseVl53l1xSample(makeFrame(0x110, {0x01, 0, 1, 0}), sample));
  const auto timeout = makeFrame(0x110, {0x01, 1, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x2D});
  ASSERT_TRUE(robot_navigation::parseVl53l1xSample(timeout, sample));
  EXPECT_EQ(0x04, sample.status);
  EXPECT_EQ(0xFFFF, sample.distance_mm);
}

TEST(Vl53l1xCanProtocol, ValidityUsesStatusAndDistanceTogether) {
  robot_navigation::Vl53l1xSample sample;
  sample.distance_mm = 850;
  sample.sigma_mm = 0xFFFF;  // Firmware currently has no sigma estimate.
  sample.status = robot_navigation::kStm32RangeValid;
  EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));

  sample.status = robot_navigation::kStm32RangeValid | robot_navigation::kStm32RangeEmergency;
  EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));
  sample.status = robot_navigation::kStm32RangeValid | robot_navigation::kStm32RangeOutOfRange;
  EXPECT_FALSE(robot_navigation::stm32RangeSampleValid(sample));
  EXPECT_TRUE(robot_navigation::stm32RangeStatusHasFailure(sample.status));
  sample.status = robot_navigation::kStm32RangeTimeout;
  EXPECT_FALSE(robot_navigation::stm32RangeSampleValid(sample));
  sample.status = robot_navigation::kStm32RangeEmergency;
  EXPECT_FALSE(robot_navigation::stm32RangeStatusHasFailure(sample.status));
  sample.status = robot_navigation::kStm32RangeValid;
  sample.distance_mm = 0xFFFF;
  EXPECT_FALSE(robot_navigation::stm32RangeSampleValid(sample));
}

TEST(Vl53l1xCanProtocol, AllFiveSensorsAreInfrared) {
  EXPECT_EQ(sensor_msgs::Range::INFRARED, robot_navigation::stm32RangeRadiationType(0));
  EXPECT_EQ(sensor_msgs::Range::INFRARED, robot_navigation::stm32RangeRadiationType(1));
  EXPECT_EQ(sensor_msgs::Range::INFRARED, robot_navigation::stm32RangeRadiationType(2));
  EXPECT_STREQ("VL53L1X ToF front", robot_navigation::stm32RangeDeviceName(0));
}

TEST(Vl53l1xCanProtocol, RejectsExtendedAndRemoteFrames) {
  robot_navigation::Vl53l1xSample sample;
  EXPECT_FALSE(robot_navigation::parseVl53l1xSample(makeFrame(0x110 | CAN_EFF_FLAG,
      {0x01, 0, 1, 0, 0, 0, 1, 0}), sample));
  EXPECT_FALSE(robot_navigation::parseVl53l1xSample(makeFrame(0x110 | CAN_RTR_FLAG,
      {0x01, 0, 1, 0, 0, 0, 1, 0}), sample));
}

TEST(Vl53l1xCanProtocol, DecodesDiagnosticsIncludingBusFrame) {
  robot_navigation::Vl53l1xDiagnosticFrame diagnostic;
  ASSERT_TRUE(robot_navigation::parseVl53l1xDiagnostic(
      makeFrame(0x111, {0x01, 0xFF, 2, 4, 0x52, 0x03, 0x20, 7}), diagnostic));
  EXPECT_EQ(0xFF, diagnostic.sensor_id);
  EXPECT_EQ(850, diagnostic.last_valid_distance_mm);
  EXPECT_EQ(4, diagnostic.consecutive_errors);
}

TEST(Vl53l1xCanProtocol, SequenceWrapAndDroppedFramesAreDetectable) {
  EXPECT_EQ(1, robot_navigation::vl53l1xSequenceDelta(0xFF, 0x00));
  EXPECT_EQ(3, robot_navigation::vl53l1xSequenceDelta(10, 13));
}

TEST(Vl53l1xCanProtocol, WatchdogRejectsMissingOrOldSamples) {
  const ros::SteadyTime last(10.0);
  EXPECT_TRUE(robot_navigation::vl53l1xSampleFresh(true, ros::SteadyTime(10.15), last, 200));
  EXPECT_FALSE(robot_navigation::vl53l1xSampleFresh(true, ros::SteadyTime(10.21), last, 200));
  EXPECT_FALSE(robot_navigation::vl53l1xSampleFresh(false, ros::SteadyTime(10.0), last, 200));
}

TEST(Vl53l1xCanProtocol, FirmwareExamplesAndUpperDiagnostics) {
  robot_navigation::Vl53l1xSample sample;
  ASSERT_TRUE(robot_navigation::parseVl53l1xSample(makeFrame(0x110,
      {1, 3, 0xF4, 1, 0xFF, 0xFF, 1, 1}), sample));
  EXPECT_EQ(500, sample.distance_mm);
  EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));
  ASSERT_TRUE(robot_navigation::parseVl53l1xSample(makeFrame(0x110,
      {1, 4, 0xE8, 3, 0xFF, 0xFF, 1, 1}), sample));
  EXPECT_EQ(1000, sample.distance_mm);
  EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));
  for (uint8_t id : {3, 4}) {
    robot_navigation::Vl53l1xDiagnosticFrame diagnostic;
    ASSERT_TRUE(robot_navigation::parseVl53l1xDiagnostic(makeFrame(0x111,
        {1, id, 3, 4, 0xFF, 0xFF, 4, 255}), diagnostic));
    EXPECT_EQ(id, diagnostic.sensor_id);
    EXPECT_EQ(3, diagnostic.error_code);
  }
  sample.distance_mm = 50; EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));
  sample.distance_mm = 3000; EXPECT_TRUE(robot_navigation::stm32RangeSampleValid(sample));
  sample.distance_mm = 49; EXPECT_FALSE(robot_navigation::stm32RangeSampleValid(sample));
  sample.distance_mm = 3001; EXPECT_FALSE(robot_navigation::stm32RangeSampleValid(sample));
}
