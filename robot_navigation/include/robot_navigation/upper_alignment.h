#pragma once
#include <array>
#include <cmath>
#include "robot_navigation/range_protocol.h"

namespace robot_navigation {
// All times are seconds from the host monotonic clock, never ROS/simulation time.
class UpperAlignment {
 public:
  double start_mm = 50.0, stop_mm = 20.0, speed_mps = 0.05;
  double timeout_sec = 0.2, skew_sec = 0.1, settled_sec = 0.3;

  bool parametersValid() const {
    return std::isfinite(start_mm) && std::isfinite(stop_mm) &&
        std::isfinite(speed_mps) && std::isfinite(timeout_sec) &&
        std::isfinite(skew_sec) && std::isfinite(settled_sec) &&
        start_mm > stop_mm && stop_mm >= 0 && speed_mps > 0 &&
        timeout_sec > 0 && timeout_sec <= 0.2 && skew_sec >= 0 &&
        skew_sec <= 0.1 && settled_sec > 0;
  }
  void enter(double now) {
    active_ = true;
    entered_ = now;
    for (auto& s : samples_) s.valid = false;
    direction_ = 0;
    settled_since_ = -1;
  }
  void stopMotion() { direction_ = 0; settled_since_ = -1; }
  void exit() { active_ = false; stopMotion(); }
  bool active() const { return active_; }
  // IDs 0/1/2 are never comparison inputs. Keep sequence across stage changes.
  bool update(const Vl53l1xSample& sample, double received) {
    if (sample.sensor_id < 3 || sample.sensor_id > 4) return false;
    auto& s = samples_[sample.sensor_id - 3];
    const bool valid = stm32RangeSampleValid(sample) &&
        !(sample.status & (kStm32RangeLowQuality | kStm32RangeEmergency));
    if (!valid) s.valid = false;
    if (s.has_sequence && sample.sequence == s.sequence) return false;
    s.has_sequence = true;
    s.sequence = sample.sequence;  // uint8 sequence naturally permits 255 -> 0.
    s.received = received;
    s.distance = sample.distance_mm;
    s.valid = valid && (!active_ || received > entered_);
    return true;
  }
  void invalidate(uint8_t id) {
    if (id >= 3 && id <= 4) samples_[id - 3].valid = false;
  }
  bool ready(double now) const {
    if (!active_) return false;
    for (const auto& s : samples_)
      if (!s.valid || s.received <= entered_ || now < s.received ||
          now - s.received > timeout_sec) return false;
    return std::abs(samples_[0].received - samples_[1].received) <= skew_sec;
  }
  double velocity(double now) {
    if (!ready(now)) { direction_ = 0; settled_since_ = -1; return 0; }
    const double delta = samples_[1].distance - samples_[0].distance;
    if (std::abs(delta) <= stop_mm) {
      direction_ = 0;
      if (settled_since_ < 0) settled_since_ = now;
    } else {
      settled_since_ = -1;
      // Any sign reversal first emits a stop, even if the new difference is large.
      if (direction_ != 0 && delta * direction_ < 0) direction_ = 0;
      else if (std::abs(delta) >= start_mm) direction_ = delta > 0 ? 1 : -1;
      // The intermediate band retains motion only with the same sign.
    }
    return direction_ * speed_mps;
  }
  bool done(double now) const {
    return ready(now) && settled_since_ >= 0 && now - settled_since_ >= settled_sec;
  }
 private:
  struct Sample {
    bool valid = false, has_sequence = false;
    uint8_t sequence = 0;
    double distance = 0, received = 0;
  };
  std::array<Sample, 2> samples_{};
  bool active_ = false;
  int direction_ = 0;
  double entered_ = 0, settled_since_ = -1;
};
}  // namespace robot_navigation
