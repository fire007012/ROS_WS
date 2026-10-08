#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <mutex>
#include <vector>
#include "can_motor_interface/can_protocol.h"

namespace can_motor_interface {

// Logical [front, right, back, left] -> STM32 index and physical CAN2 address.
struct WheelMap {
  std::vector<int> indices{1, 0, 3, 2};
  std::vector<int> signs{1, 1, 1, 1};
  void validate() const {
    if (indices.size() != 4 || signs.size() != 4)
      throw std::invalid_argument("Require four motor_indices and direction_signs");
    std::array<bool, 4> used{};
    for (size_t i = 0; i < 4; ++i) {
      if (indices[i] < 0 || indices[i] >= 4 || used[indices[i]] || std::abs(signs[i]) != 1)
        throw std::invalid_argument("motor_indices must permute 0..3; direction_signs must be +/-1");
      used[indices[i]] = true;
    }
  }
  int logicalIndex(int stm32_index) const {
    for (size_t i = 0; i < indices.size(); ++i)
      if (indices[i] == stm32_index) return static_cast<int>(i);
    return -1;
  }
};

// A stop and each motion write share a mutex. A sender that has already entered
// finishes before STOP; every later sender sees the latch and is suppressed.
class MotionInterlock {
 public:
  bool latched() const { return latched_.load(); }
  template<class Send> bool run(Send send) {
    std::lock_guard<std::mutex> lock(mutex_);
    return latched() ? true : send();
  }
  template<class Send> bool stop(Send send) {
    const bool first = !latched_.exchange(true);
    std::lock_guard<std::mutex> lock(mutex_);
    latched_.store(true);
    send();
    return first;
  }
  template<class Healthy> bool reset(Healthy healthy) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latched() || !healthy()) return false;
    latched_.store(false);
    return true;
  }
 private:
  std::atomic<bool> latched_{false};
  std::mutex mutex_;
};

struct DriverTelemetry {
  uint8_t address = 0;
  uint8_t function = 0;
  uint8_t flags = 0;
  float rpm = 0;
};

inline can_frame driverRead(uint8_t address, uint8_t function) {
  can_frame frame{};
  frame.can_id = socketCanId(static_cast<uint32_t>(address) << 8, true);
  frame.can_dlc = 2;
  frame.data[0] = function;
  frame.data[1] = 0x6B;
  return frame;
}

inline bool decodeDriverTelemetry(const can_frame& frame, DriverTelemetry* out) {
  if (!out || !(frame.can_id & CAN_EFF_FLAG) ||
      (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) ||
      (frame.can_id & CAN_EFF_MASK) > 0xFF00 || (frame.can_id & 0xFF) != 0)
    return false;
  if (frame.data[0] == 0x35 && frame.can_dlc == 5 &&
      frame.data[1] <= 1 && frame.data[4] == 0x6B) {
    out->rpm = ((frame.data[2] << 8) | frame.data[3]) / 10.0f;
    if (frame.data[1]) out->rpm = -out->rpm;
  } else if (frame.data[0] == 0x3A && frame.can_dlc == 3 && frame.data[2] == 0x6B &&
             frame.data[1] != 0xE2 && frame.data[1] != 0xEE) {
    out->flags = frame.data[1];
  } else return false;
  out->address = static_cast<uint8_t>((frame.can_id & CAN_EFF_MASK) >> 8);
  out->function = frame.data[0];
  return out->address != 0;
}

// Only real CAN2 replies may call observe(). STM32 0x101 cache reports must not.
class DriverHealth {
 public:
  void observe(size_t wheel, const DriverTelemetry& reply, double now) {
    if (wheel >= 4) return;
    if (reply.function == 0x35) speed_time[wheel] = now;
    else if (reply.function == 0x3A) {
      status_time[wheel] = now;
      flags[wheel] = reply.flags;
    }
  }
  bool fresh(double now, double timeout) const {
    for (size_t i = 0; i < 4; ++i)
      if (speed_time[i] <= 0 || status_time[i] <= 0 ||
          now - speed_time[i] > timeout || now - status_time[i] > timeout) return false;
    return true;
  }
  bool healthy(double now, double timeout) const {
    if (!fresh(now, timeout)) return false;
    for (auto flag : flags) if (!(flag & 1) || (flag & 8)) return false;
    return true;
  }
  std::array<double, 4> speed_time{};
  std::array<double, 4> status_time{};
  std::array<uint8_t, 4> flags{};
};

// Firmware sends three independently tagged snapshots of 16-bit counters.
// The first snapshot is a baseline (historical failures); changes include wrap.
class FirmwareStats {
 public:
  bool observe(const can_frame& frame) {
    response_timeout_changed_ = false;
    if (frame.can_id != 0x103 || frame.can_dlc != 8 || frame.data[0] < 1 || frame.data[0] > 3)
      return false;
    const size_t type = frame.data[0] - 1;
    std::array<uint16_t, 3> counts{};
    size_t size = 0;
    if (type == 0) { counts[0] = le16(frame.data + 3); counts[1] = le16(frame.data + 5); size = 2; }
    if (type == 1) { counts[0] = le16(frame.data + 3); size = 1; }
    if (type == 2) {
      counts[0] = le16(frame.data + 1); counts[1] = le16(frame.data + 3);
      counts[2] = le16(frame.data + 5); size = 3;
    }
    bool failed = type == 1 && frame.data[7] != 0;
    if (seen_[type]) for (size_t i = 0; i < size; ++i) {
      if (counts[i] != previous_[type][i]) {
        // This aggregate includes background reads of the optional fifth axis.
        // It has no address/command tag. Selected-wheel feedback and F6 ACKs
        // independently detect chassis communication failures.
        if (type == 0 && i == 1) response_timeout_changed_ = true;
        else failed = true;
      }
    }
    previous_[type] = counts;
    seen_[type] = true;
    return failed;
  }
  bool ready() const { return seen_[0] && seen_[1] && seen_[2]; }
  bool responseTimeoutChanged() const { return response_timeout_changed_; }
 private:
  static uint16_t le16(const uint8_t* p) { return p[0] | (static_cast<uint16_t>(p[1]) << 8); }
  std::array<bool, 3> seen_{};
  std::array<std::array<uint16_t, 3>, 3> previous_{};
  bool response_timeout_changed_ = false;
};

// Real driver motion acknowledgements, not STM32's local enqueue ACK (0x102).
class MotionAckHealth {
 public:
  void sent(size_t wheel, double now) {
    if (wheel < 4 && first_send_[wheel] == 0) first_send_[wheel] = now;
  }
  void acknowledge(size_t wheel, double now) {
    if (wheel < 4) last_ack_[wheel] = now;
  }
  bool healthy(double now, double timeout) const {
    for (size_t i = 0; i < 4; ++i) {
      if (first_send_[i] == 0) continue;
      const double stamp = last_ack_[i] >= first_send_[i] ? last_ack_[i] : first_send_[i];
      if (now - stamp > timeout) return false;
    }
    return true;
  }
 private:
  std::array<double, 4> first_send_{};
  std::array<double, 4> last_ack_{};
};

}  // namespace can_motor_interface
