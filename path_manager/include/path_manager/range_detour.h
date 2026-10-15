#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace path_manager {

// A bounded, reactive detour for an omnidirectional chassis. These four beams
// are observations along the chassis axes, not a reconstructed obstacle map.
class RangeDetour {
 public:
  struct Parameters {
    double trigger_m = 0.70;
    double front_clear_m = 0.90;
    double motion_clearance_m = 0.40;
    double side_space_m = 0.80;
    double min_offset_m = 0.70;
    double edge_margin_m = 0.35;
    double pass_margin_m = 0.45;
    double obstacle_length_m = 0.50;  // Confirmed cone base diameter along travel.
    double body_radius_m = 0.25;     // Samples already measure clearance from the body edge.
    double wall_detect_m = 1.30;
    double max_offset_m = 1.25;
    double max_forward_m = 3.0;
    double speed_mps = 0.12;
    double timeout_sec = 45.0;
    double stall_sec = 6.0;
    double range_timeout_sec = 0.20;
    double clear_hold_sec = 0.25;
    double settle_sec = 0.25;
    double line_tolerance_m = 0.03;
    double heading_tolerance_rad = 0.15;
    bool prefer_left = true;
  } params;
  struct Pose { double x = 0.0, y = 0.0, yaw = 0.0; };
  struct Command { double x = 0.0, y = 0.0, omega = 0.0; };
  struct Sample { double distance = 0.0, received = 0.0; bool valid = false; };
  struct Output {
    bool override_command = false;
    Command command;
    std::string state = "TRACKING";
  };
  std::array<Sample, 4> ranges;  // front, left, right, rear

  bool parametersValid() const {
    for (double value : {params.trigger_m, params.front_clear_m, params.motion_clearance_m,
         params.side_space_m, params.min_offset_m, params.edge_margin_m, params.pass_margin_m,
         params.obstacle_length_m, params.body_radius_m,
         params.wall_detect_m, params.max_offset_m, params.max_forward_m, params.speed_mps,
         params.timeout_sec, params.stall_sec, params.range_timeout_sec, params.clear_hold_sec,
         params.settle_sec, params.line_tolerance_m, params.heading_tolerance_rad})
      if (!std::isfinite(value) || value <= 0.0) return false;
    return params.trigger_m > params.motion_clearance_m &&
        params.front_clear_m > params.trigger_m && params.side_space_m > params.motion_clearance_m &&
        params.min_offset_m < params.max_offset_m && params.edge_margin_m < params.max_offset_m &&
        params.wall_detect_m > params.motion_clearance_m && params.speed_mps <= 0.15 &&
        params.range_timeout_sec <= 0.5 && params.line_tolerance_m < params.min_offset_m &&
        params.heading_tolerance_rad <= 0.25;
  }

  void reset() {
    phase_ = Phase::TRACKING;
    reason_.clear();
    clear_since_ = settled_since_ = -1.0;
    wall_seen_ = false;
    side_ = 0;
    // Keep received ranges; resetting a target never creates a valid sample.
  }

  bool active() const { return phase_ != Phase::TRACKING; }
  bool failed() const { return phase_ == Phase::FAILED; }

  Output step(const Pose& pose, const Pose& goal, const Command& nominal,
              double measured_speed, double measured_rotation, double now) {
    if (failed()) return stopped(reason_);
    if (!finite(pose) || !finite(goal) || !std::isfinite(now) ||
        !std::isfinite(measured_speed) || !std::isfinite(measured_rotation))
      return stopped("WAIT_ODOMETRY");
    if (active()) {
      if (now - started_ > params.timeout_sec) return fail("TIMEOUT");
      if (std::hypot(pose.x - checkpoint_.x, pose.y - checkpoint_.y) >= 0.03) {
        checkpoint_ = pose;
        progressed_ = now;
      }
      if (now - progressed_ > params.stall_sec) return fail("STALLED");
    }
    if (phase_ == Phase::TRACKING) {
      const std::array<double, 4> toward{{nominal.x, nominal.y, -nominal.y, -nominal.x}};
      travel_ = std::max_element(toward.begin(), toward.end()) - toward.begin();
      if (toward[travel_] <= 0.001 || std::abs(nominal.omega) > 0.15 ||
          !near(travel_, params.trigger_m, now)) return Output{};
      const double dx = goal.x - pose.x, dy = goal.y - pose.y;
      const double distance = std::hypot(dx, dy);
      const double bearing = std::atan2(dy, dx);
      const double beam_heading = pose.yaw + beamAngle(travel_);
      // Only detour along the requested route. Turning into an obstruction is
      // still handled by the independent braking node.
      if (distance < params.line_tolerance_m ||
          std::abs(angle(bearing - beam_heading)) > params.heading_tolerance_rad) return Output{};
      // An obstacle beyond a nearby endpoint is not a blocked route segment.
      if (distance <= ranges[travel_].distance) return Output{};
      origin_ = checkpoint_ = pose;
      heading_ = pose.yaw;
      ux_ = dx / distance; uy_ = dy / distance;
      goal_distance_ = distance;
      obstacle_end_along_ = ranges[travel_].distance + params.body_radius_m + params.obstacle_length_m;
      started_ = progressed_ = now;
      wall_seen_ = false;
      side_ = 0;
      beginBrake(Phase::SIDESTEP);
      return stopped("BRAKING");
    }

    if (std::abs(angle(pose.yaw - heading_)) > params.heading_tolerance_rad)
      return fail("HEADING_DRIFT");
    const double dx = pose.x - origin_.x, dy = pose.y - origin_.y;
    const double along = dx * ux_ + dy * uy_;
    const double offset = side_ * (-dx * uy_ + dy * ux_);
    if (std::abs(offset) > params.max_offset_m + params.line_tolerance_m)
      return fail("OFFSET_LIMIT");
    if (along > params.max_forward_m) return fail("BYPASS_LIMIT");
    if (along >= goal_distance_ - params.line_tolerance_m && phase_ != Phase::RETURN)
      return fail("GOAL_BLOCKED");

    if (phase_ == Phase::BRAKE) {
      if (measured_speed > 0.02 || std::abs(measured_rotation) > 0.02) {
        settled_since_ = -1.0;
        return stopped("BRAKING");
      }
      if (settled_since_ < 0.0) settled_since_ = now;
      if (now - settled_since_ < params.settle_sec) return stopped("BRAKING");
      if (side_ == 0) {
        const auto left = leftBeam(travel_), right = rightBeam(travel_);
        const bool left_free = !observed(left, now) || ranges[left].distance >= params.side_space_m;
        const bool right_free = !observed(right, now) || ranges[right].distance >= params.side_space_m;
        if (!left_free && !right_free) return stopped("NO_SIDE_SPACE");
        const double left_space = observed(left, now) ? ranges[left].distance : params.side_space_m;
        const double right_space = observed(right, now) ? ranges[right].distance : params.side_space_m;
        side_ = left_free && (!right_free || left_space > right_space + 0.05 ||
                    (std::abs(left_space - right_space) <= 0.05 && params.prefer_left)) ? 1 : -1;
        outer_ = side_ > 0 ? left : right;
        inner_ = side_ > 0 ? right : left;
      }
      phase_ = after_brake_;
      clear_since_ = -1.0;
      return stopped(phaseName());
    }

    if (phase_ == Phase::SIDESTEP) {
      if (!near(travel_, params.front_clear_m, now)) {
        if (clear_since_ < 0.0) { clear_since_ = now; clear_offset_ = offset; }
        if (offset >= std::max(params.min_offset_m, clear_offset_ + params.edge_margin_m) &&
            now - clear_since_ >= params.clear_hold_sec) {
          beginBrake(Phase::BYPASS);
          return stopped("BRAKING");
        }
      } else clear_since_ = -1.0;
      if (offset >= params.max_offset_m) return fail("OFFSET_LIMIT");
      if (near(outer_, params.motion_clearance_m, now)) return stopped("SIDE_BLOCKED");
      return motion(pose, 0.0, side_ * params.speed_mps, now);
    }

    if (phase_ == Phase::BYPASS || phase_ == Phase::EDGE_CLEAR) {
      const double return_space = offset + params.motion_clearance_m + params.line_tolerance_m;
      if (observed(inner_, now) && ranges[inner_].distance <= params.wall_detect_m &&
          ranges[inner_].distance <= return_space) wall_seen_ = true;
      if (near(travel_, params.motion_clearance_m, now)) return stopped("FRONT_BLOCKED");
      // Increase separation when the inner beam first reaches the obstacle;
      // never steer toward it based on a single distant reading.
      if (near(inner_, params.motion_clearance_m + 0.03, now)) {
        clear_since_ = -1.0;
        if (offset >= params.max_offset_m) return fail("OFFSET_LIMIT");
        if (near(outer_, params.motion_clearance_m, now)) return stopped("SIDE_BLOCKED");
        return motion(pose, 0.0, side_ * params.speed_mps, now);
      }
      // A cone may leave every beam after the sidestep. Keep its remembered
      // position and footprint until odometry confirms we have passed it.
      // Missing returns alone never mean the remembered cone has been passed.
      const bool return_corridor_clear = !near(inner_, return_space, now) &&
          ((wall_seen_ && observed(inner_, now)) || along >= obstacle_end_along_);
      if (phase_ == Phase::BYPASS) {
        if (return_corridor_clear) {
          if (clear_since_ < 0.0) clear_since_ = now;
          if (now - clear_since_ >= params.clear_hold_sec) {
            edge_along_ = along;
            phase_ = Phase::EDGE_CLEAR;
          }
        } else clear_since_ = -1.0;
      } else if (!return_corridor_clear) {
        phase_ = Phase::BYPASS;
        clear_since_ = -1.0;
      } else if (along - edge_along_ >= params.pass_margin_m) {
        beginBrake(Phase::RETURN);
        return stopped("BRAKING");
      }
      return motion(pose, params.speed_mps, 0.0, now);
    }

    if (phase_ == Phase::RETURN) {
      if (std::abs(offset) <= params.line_tolerance_m) {
        beginBrake(Phase::TRACKING);
        return stopped("BRAKING");
      }
      if (near(inner_, std::abs(offset) + params.motion_clearance_m, now))
        return stopped("RETURN_BLOCKED");
      const double speed = std::min(params.speed_mps, 0.8 * std::abs(offset));
      return motion(pose, 0.0, -side_ * std::copysign(speed, offset), now);
    }
    return Output{};
  }

 private:
  enum class Phase { TRACKING, BRAKE, SIDESTEP, BYPASS, EDGE_CLEAR, RETURN, FAILED };
  Phase phase_ = Phase::TRACKING, after_brake_ = Phase::TRACKING;
  Pose origin_, checkpoint_;
  int travel_ = 0, outer_ = 1, inner_ = 2, side_ = 0;
  double ux_ = 1.0, uy_ = 0.0, heading_ = 0.0, goal_distance_ = 0.0;
  double started_ = 0.0, progressed_ = 0.0;
  double obstacle_end_along_ = 0.0;
  double clear_since_ = -1.0, settled_since_ = -1.0, clear_offset_ = 0.0, edge_along_ = 0.0;
  bool wall_seen_ = false;
  std::string reason_;

  bool observed(size_t index, double now) const {
    const auto& r = ranges[index];
    return r.valid && std::isfinite(r.distance) && r.distance >= 0.0 &&
        std::isfinite(r.received) && now >= r.received && now - r.received <= params.range_timeout_sec;
  }
  bool near(size_t index, double limit, double now) const {
    return observed(index, now) && ranges[index].distance <= limit;
  }

  static bool finite(const Pose& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw); }
  static double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
  static double beamAngle(int beam) {
    const double pi = std::acos(-1.0);
    return std::array<double, 4>{{0.0, pi / 2.0, -pi / 2.0, pi}}[beam];
  }
  static int leftBeam(int beam) { return std::array<int, 4>{{1, 3, 0, 2}}[beam]; }
  static int rightBeam(int beam) { return std::array<int, 4>{{2, 0, 3, 1}}[beam]; }
  static Output stopped(const std::string& reason) {
    Output out; out.override_command = true; out.state = reason; return out;
  }
  Output fail(const std::string& reason) {
    phase_ = Phase::FAILED; reason_ = "FAILED_" + reason; return stopped(reason_);
  }
  void beginBrake(Phase next) {
    phase_ = Phase::BRAKE; after_brake_ = next;
    settled_since_ = clear_since_ = -1.0;
  }
  const char* phaseName() const {
    switch (phase_) {
      case Phase::TRACKING: return "TRACKING";
      case Phase::SIDESTEP: return "SIDESTEP";
      case Phase::BYPASS: return "BYPASS";
      case Phase::EDGE_CLEAR: return "EDGE_CLEAR";
      case Phase::RETURN: return "RETURN";
      case Phase::BRAKE: return "BRAKING";
      default: return "FAILED";
    }
  }
  Output motion(const Pose& pose, double forward, double lateral, double now) const {
    Output out; out.override_command = true; out.state = phaseName();
    const double wx = ux_ * forward - uy_ * lateral;
    const double wy = uy_ * forward + ux_ * lateral;
    out.command.x = std::cos(pose.yaw) * wx + std::sin(pose.yaw) * wy;
    out.command.y = -std::sin(pose.yaw) * wx + std::cos(pose.yaw) * wy;
    const std::array<double, 4> toward{{out.command.x, out.command.y, -out.command.y, -out.command.x}};
    for (size_t i = 0; i < toward.size(); ++i)
      if (toward[i] > 0.001 && near(i, params.motion_clearance_m, now))
        return stopped("DIRECTION_BLOCKED");
    return out;
  }
};
}  // namespace path_manager
