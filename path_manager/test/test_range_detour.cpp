#include <gtest/gtest.h>
#include "path_manager/range_detour.h"

using Detour = path_manager::RangeDetour;

namespace {
struct Fixture {
  Detour detour;
  Detour::Pose pose, goal;
  Detour::Command nominal;
  double now = 0.0;
  Fixture() { goal.x = 4.0; nominal.x = 0.25; refresh(); }
  void refresh() {
    for (auto& r : detour.ranges) { r.valid = true; r.distance = 2.5; r.received = now; }
  }
  Detour::Output step(double time, double speed = 0.0) {
    now = time;
    for (auto& r : detour.ranges) r.received = now;
    return detour.step(pose, goal, nominal, speed, 0.0, now);
  }
  void start() {
    detour.ranges[0].distance = .65;
    EXPECT_EQ("BRAKING", step(0.0, .25).state);
    EXPECT_EQ("BRAKING", step(.1, .25).state);
    EXPECT_EQ("BRAKING", step(.2).state);
    EXPECT_EQ("SIDESTEP", step(.5).state);
  }
};
}

TEST(RangeDetour, RejectsInvalidParametersAndAllowsDefaults) {
  Detour d;
  EXPECT_TRUE(d.parametersValid());
  d.params.speed_mps = .5; EXPECT_FALSE(d.parametersValid());
  d.params.speed_mps = .12;
  d.params.front_clear_m = .60; EXPECT_FALSE(d.parametersValid());
  d.params.front_clear_m = .9;
  d.params.max_offset_m = .5; EXPECT_FALSE(d.parametersValid());
}

TEST(RangeDetour, FullDetourWaitsForObservedWallAndTrailingClearance) {
  Fixture f;
  f.start();
  EXPECT_GT(f.step(.6).command.y, 0.0);
  f.detour.ranges[0].distance = 2.5;
  f.pose.y = .10;
  EXPECT_EQ("SIDESTEP", f.step(.7).state);
  // Losing the front obstacle alone cannot authorize return to the route.
  f.pose.y = .70;
  EXPECT_EQ("BRAKING", f.step(1.1).state);
  f.step(1.2);
  EXPECT_EQ("BYPASS", f.step(1.5).state);
  f.pose.x = .4;
  EXPECT_EQ("BYPASS", f.step(1.6).state);
  f.pose.x = .8;
  f.detour.ranges[2].distance = .55;
  EXPECT_EQ("BYPASS", f.step(1.7).state);
  f.pose.x = 1.5;
  f.detour.ranges[2].distance = 2.5;
  f.step(1.8);
  EXPECT_EQ("EDGE_CLEAR", f.step(2.1).state);
  f.pose.x = 1.7;
  EXPECT_EQ("EDGE_CLEAR", f.step(2.2).state);
  f.pose.x = 2.0;
  EXPECT_EQ("BRAKING", f.step(2.3).state);
  f.step(2.4);
  EXPECT_EQ("RETURN", f.step(2.7).state);
  EXPECT_LT(f.step(2.8).command.y, 0.0);
  f.detour.ranges[2].distance = .5;
  EXPECT_EQ("RETURN_BLOCKED", f.step(2.9).state);
  f.detour.ranges[2].distance = 2.5;
  f.pose.y = .01;
  EXPECT_EQ("BRAKING", f.step(3.0).state);
  f.step(3.1);
  EXPECT_EQ("TRACKING", f.step(3.4).state);
  EXPECT_FALSE(f.step(3.5).override_command);
  EXPECT_FALSE(f.detour.active());
}

TEST(RangeDetour, ChoosesOtherSideAndNeverMovesIntoInsufficientSpace) {
  Fixture f;
  f.detour.ranges[1].distance = .45;
  f.start();
  EXPECT_LT(f.step(.6).command.y, 0.0);
  f.detour.ranges[2].distance = .30;
  const auto blocked = f.step(.7);
  EXPECT_EQ("SIDE_BLOCKED", blocked.state);
  EXPECT_EQ(0.0, blocked.command.y);
}

TEST(RangeDetour, BlockedSidesStopAndStallFailureRequiresNewTarget) {
  Fixture f;
  f.detour.params.stall_sec = 1.0;
  f.detour.ranges[0].distance = .65;
  f.detour.ranges[1].distance = f.detour.ranges[2].distance = .5;
  f.step(0.0); f.step(.1);
  EXPECT_EQ("NO_SIDE_SPACE", f.step(.4).state);
  EXPECT_EQ("FAILED_STALLED", f.step(1.1).state);
  f.refresh();
  EXPECT_EQ("FAILED_STALLED", f.step(1.2).state);
  f.detour.reset();
  EXPECT_FALSE(f.step(1.3).override_command);
}

TEST(RangeDetour, MissingRangesDoNotStopTrackingOrAnExistingConeDetour) {
  Fixture f;
  for (auto& r : f.detour.ranges) r.valid = false;
  EXPECT_FALSE(f.step(0).override_command);
  f.refresh();
  f.start();
  f.detour.ranges[3].valid = false;
  EXPECT_GT(f.step(.6).command.y, 0.0);
  f.detour.ranges[3].valid = true;
  EXPECT_GT(f.step(.7).command.y, 0.0);
  f.now = 1.0;
  EXPECT_GT(f.detour.step(f.pose, f.goal, f.nominal, 0, 0, f.now).command.y, 0.0);
}

TEST(RangeDetour, LostConeReturnUsesRememberedPositionBeforeReturningToRoute) {
  Fixture f;
  f.start();
  for (auto& r : f.detour.ranges) r.valid = false;
  f.pose.y = .1; f.step(.6);
  f.pose.y = .7;
  EXPECT_EQ("BRAKING", f.step(.9).state);
  f.step(1.0);
  EXPECT_EQ("BYPASS", f.step(1.3).state);
  f.pose.x = .5;
  EXPECT_EQ("BYPASS", f.step(1.4).state);  // Cone still ahead, no early return.
  f.pose.x = 1.6;
  f.step(1.5);
  EXPECT_EQ("EDGE_CLEAR", f.step(1.8).state);
  f.pose.x = 2.1;
  EXPECT_EQ("BRAKING", f.step(1.9).state);
  f.step(2.0);
  EXPECT_EQ("RETURN", f.step(2.3).state);
  EXPECT_LT(f.step(2.4).command.y, 0.0);
  f.pose.y = .01;
  f.step(2.5); f.step(2.6);
  EXPECT_EQ("TRACKING", f.step(2.9).state);
  EXPECT_FALSE(f.step(3.0).override_command);
}

TEST(RangeDetour, OnlyFreshDetectionsIncludingZeroBodyClearanceBlockMovement) {
  Fixture f;
  f.detour.ranges[0].distance = .2;
  f.detour.ranges[0].valid = false;
  EXPECT_FALSE(f.step(0).override_command);
  f.detour.ranges[0].valid = true;
  EXPECT_FALSE(f.detour.step(f.pose, f.goal, f.nominal, 0, 0, 1.0).override_command);
  f.refresh(); f.start();
  f.detour.ranges[1].distance = 0.0;  // Valid inset reading normalized to the body edge.
  EXPECT_EQ("SIDE_BLOCKED", f.step(.6).state);
  f.detour.ranges[1].valid = false;
  EXPECT_GT(f.step(.7).command.y, 0.0);
}

TEST(RangeDetour, BoundsAndHeadingDriftLatchRatherThanCuttingAcrossObstacle) {
  Fixture f;
  f.start();
  f.pose.y = 1.4;
  EXPECT_EQ("FAILED_OFFSET_LIMIT", f.step(.6).state);
  f.detour.reset(); f.pose = Detour::Pose{}; f.start();
  f.pose.yaw = .3;
  EXPECT_EQ("FAILED_HEADING_DRIFT", f.step(.6).state);
}

TEST(RangeDetour, ObstacleBeyondEndpointDoesNotStartDetour) {
  Fixture f;
  f.goal.x = .4;
  f.detour.ranges[0].distance = .65;
  EXPECT_FALSE(f.step(0).override_command);
}

TEST(RangeDetour, MapsSideAndRearTravelToPhysicalBeams) {
  const double pi = std::acos(-1.0);
  for (int beam : {1, 2, 3}) {
    Fixture f;
    const std::array<double, 4> angles{{0, pi/2, -pi/2, pi}};
    f.goal.x = 4 * std::cos(angles[beam]); f.goal.y = 4 * std::sin(angles[beam]);
    f.nominal.x = .25 * std::cos(angles[beam]); f.nominal.y = .25 * std::sin(angles[beam]);
    f.detour.ranges[beam].distance = .65;
    f.step(0); f.step(.1); f.step(.4);
    const auto out = f.step(.5);
    EXPECT_TRUE(out.override_command);
    // Equal free space chooses left relative to travel, not chassis-left.
    EXPECT_NEAR(-.12 * std::sin(angles[beam]), out.command.x, 1e-9);
    EXPECT_NEAR(.12 * std::cos(angles[beam]), out.command.y, 1e-9);
  }
}
