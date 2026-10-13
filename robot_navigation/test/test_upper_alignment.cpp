#include <gtest/gtest.h>
#include "robot_navigation/upper_alignment.h"

using robot_navigation::UpperAlignment;
using robot_navigation::Vl53l1xSample;
namespace {
Vl53l1xSample sample(uint8_t id, uint16_t distance, uint8_t seq = 1, uint8_t status = 1) {
  Vl53l1xSample s; s.sensor_id = id; s.distance_mm = distance;
  s.sequence = seq; s.status = status; return s;
}
void pair(UpperAlignment& c, double t, uint16_t left, uint16_t right, uint8_t seq = 1) {
  c.update(sample(3, left, seq), t); c.update(sample(4, right, seq), t);
}
}
TEST(UpperAlignment, DisabledAndNewStageFence) {
  UpperAlignment c;
  pair(c, 1, 500, 1000);
  EXPECT_EQ(0, c.velocity(1));
  c.enter(2);
  EXPECT_EQ(0, c.velocity(2));
  // Replay of prior sequence cannot satisfy the stage fence.
  pair(c, 2.01, 500, 1000);
  EXPECT_EQ(0, c.velocity(2.01));
  c.update(sample(3, 500, 2), 2.02);
  EXPECT_EQ(0, c.velocity(2.02));
  c.update(sample(4, 1000, 2), 2.03);
  EXPECT_DOUBLE_EQ(.05, c.velocity(2.03));
}
TEST(UpperAlignment, Rep103DirectionsAndOriginalSensorsExcluded) {
  UpperAlignment c; c.enter(1);
  for (uint8_t i = 0; i < 3; ++i) EXPECT_FALSE(c.update(sample(i, 100), 1.01));
  EXPECT_EQ(0, c.velocity(1.01));
  pair(c, 1.02, 500, 1000); EXPECT_DOUBLE_EQ(.05, c.velocity(1.02));
  pair(c, 1.03, 1000, 500, 2); EXPECT_EQ(0, c.velocity(1.03));
  EXPECT_DOUBLE_EQ(-.05, c.velocity(1.04));
  pair(c, 1.05, 500, 500, 3); EXPECT_EQ(0, c.velocity(1.05));
}
TEST(UpperAlignment, HysteresisAndSmallSignReversals) {
  UpperAlignment c; c.enter(0);
  pair(c, .01, 500, 549); EXPECT_EQ(0, c.velocity(.01));
  pair(c, .02, 500, 550, 2); EXPECT_GT(c.velocity(.02), 0);
  pair(c, .03, 500, 530, 3); EXPECT_GT(c.velocity(.03), 0);
  pair(c, .04, 530, 500, 4); EXPECT_EQ(0, c.velocity(.04));
  pair(c, .05, 500, 530, 5); EXPECT_EQ(0, c.velocity(.05));
  pair(c, .06, 550, 500, 6); EXPECT_LT(c.velocity(.06), 0);
  pair(c, .07, 520, 500, 7); EXPECT_EQ(0, c.velocity(.07));
  for (uint8_t seq = 8; seq < 18; ++seq) {
    pair(c, seq * .01, 500 + (seq % 2 ? 10 : 0), 505, seq);
    EXPECT_EQ(0, c.velocity(seq * .01));
  }
}
TEST(UpperAlignment, MissingInvalidSentinelQualityAndRange) {
  for (auto s : {sample(4, 0xFFFF, 2), sample(4, 1000, 2, 0),
                 sample(4, 1000, 2, 3), sample(4, 1000, 2, 5),
                 sample(4, 1000, 2, 9), sample(4, 1000, 2, 17),
                 sample(4, 1000, 2, 33), sample(4, 49, 2), sample(4, 3001, 2)}) {
    UpperAlignment c; c.enter(0); pair(c, .01, 500, 1000);
    EXPECT_GT(c.velocity(.01), 0);
    c.update(s, .02); EXPECT_EQ(0, c.velocity(.02));
  }
  UpperAlignment c; c.enter(0); c.update(sample(3, 500), .01);
  EXPECT_EQ(0, c.velocity(.01));
}
TEST(UpperAlignment, TimeoutDuplicateWrapSkewAndDiagnosticFailure) {
  UpperAlignment c; c.enter(1); pair(c, 1.01, 500, 1000, 255);
  EXPECT_GT(c.velocity(1.01), 0);
  pair(c, 1.15, 500, 1000, 255);  // duplicate cannot refresh age
  EXPECT_EQ(0, c.velocity(1.22));
  pair(c, 1.23, 500, 1000, 0); EXPECT_GT(c.velocity(1.23), 0);
  c.update(sample(3, 500, 1), 1.35); EXPECT_EQ(0, c.velocity(1.35));
  c.update(sample(4, 1000, 1), 1.36); EXPECT_GT(c.velocity(1.36), 0);
  c.invalidate(4); EXPECT_EQ(0, c.velocity(1.36));
}
TEST(UpperAlignment, ExitCancelEstopRequireFreshReentryAndSettling) {
  UpperAlignment c; c.enter(1); pair(c, 1.01, 500, 1000);
  EXPECT_GT(c.velocity(1.01), 0);
  c.exit(); EXPECT_EQ(0, c.velocity(1.02));
  c.enter(2); EXPECT_EQ(0, c.velocity(2));
  pair(c, 2.01, 500, 500, 2); EXPECT_EQ(0, c.velocity(2.01));
  EXPECT_FALSE(c.done(2.01));
  pair(c, 2.16, 500, 500, 3); c.velocity(2.16);
  pair(c, 2.32, 500, 500, 4); c.velocity(2.32);
  EXPECT_TRUE(c.done(2.32));
  EXPECT_FALSE(c.done(2.6));
}
TEST(UpperAlignment, UnsafeParametersRejected) {
  UpperAlignment c; EXPECT_TRUE(c.parametersValid());
  c.stop_mm = c.start_mm; EXPECT_FALSE(c.parametersValid());
  c.stop_mm = 20; c.timeout_sec = .201; EXPECT_FALSE(c.parametersValid());
  c.timeout_sec = .2; c.skew_sec = .101; EXPECT_FALSE(c.parametersValid());
}
