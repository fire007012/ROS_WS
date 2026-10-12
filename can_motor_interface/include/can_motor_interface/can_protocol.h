#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <linux/can.h>
#include <stdexcept>

namespace can_motor_interface {

// SocketCAN distinguishes standard/extended IDs with CAN_EFF_FLAG, even when
// the numeric ID fits into 11 bits. Never silently mask a misconfigured ID.
inline uint32_t socketCanId(uint32_t id, bool extended) {
  if (id > (extended ? CAN_EFF_MASK : CAN_SFF_MASK)) {
    throw std::invalid_argument("CAN ID exceeds the configured frame format");
  }
  return id | (extended ? CAN_EFF_FLAG : 0U);
}

inline bool matchesCanReply(uint32_t raw_id, uint32_t expected_id, bool extended) {
  if (raw_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) return false;
  return ((raw_id & CAN_EFF_FLAG) != 0) == extended &&
      (raw_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK)) == expected_id;
}

// STM32 CAN1 parser: exactly 8 bytes, no payload checksum. The last two bytes
// are acceleration (RPM/s) and synchronous-start flags, not a sum8 checksum.
inline can_frame stm32Command(uint32_t id, bool extended, uint8_t command,
                              uint8_t index, int32_t value = 0,
                              uint8_t param0 = 0, uint8_t param1 = 0) {
  can_frame frame{};
  frame.can_id = socketCanId(id, extended);
  frame.can_dlc = 8;
  frame.data[0] = command;
  frame.data[1] = index;
  const uint32_t raw = static_cast<uint32_t>(value);
  for (unsigned i = 0; i < 4; ++i) frame.data[2 + i] = static_cast<uint8_t>(raw >> (8 * i));
  frame.data[6] = param0;
  frame.data[7] = param1;
  return frame;
}

// The fifth motor is the arm axis. Commands use 3200 pulses/revolution,
// mode=1 absolute coordinate and sync=0 immediate execution.
inline can_frame stm32ArmPosition(double radians) {
  const double pulses = radians * (3200.0 / (2.0 * std::acos(-1.0)));
  if (!std::isfinite(pulses) || pulses < std::numeric_limits<int32_t>::min() ||
      pulses > std::numeric_limits<int32_t>::max())
    throw std::invalid_argument("Arm position is nonfinite or exceeds int32 pulse range");
  // bit2 of param1 marks a position command as cached for the next
  // synchronous physical start.  mode=1 is absolute position.
  return stm32Command(0x100, false, 2, 4, static_cast<int32_t>(std::llround(pulses)), 0, 0x01 | 0x04);
}

inline can_frame stm32CachedSpeed(uint8_t index, int32_t rpm, uint8_t acceleration) {
  // param1=1 means enqueue only; the physical start key releases the queue.
  return stm32Command(0x100, false, 0x01, index, rpm, acceleration, 0x01);
}

inline can_frame stm32CachedPosition(uint8_t index, int32_t pulse, uint8_t acceleration,
                                     uint8_t mode_bits = 0x01) {
  return stm32Command(0x100, false, 0x02, index, pulse, acceleration, mode_bits | 0x04);
}

struct Stm32Telemetry {
  uint8_t type = 0;
  uint8_t index = 0;
  uint8_t flags = 0;
  float rpm = 0;
};

inline bool decodeStm32Telemetry(uint8_t dlc, const uint8_t* data, Stm32Telemetry* out) {
  if (!data || !out || dlc != 8 || (data[0] != 0x01 && data[0] != 0x03)) return false;
  out->type = data[0];
  out->index = data[1];
  out->flags = data[2];
  const uint32_t raw = static_cast<uint32_t>(data[2]) |
      (static_cast<uint32_t>(data[3]) << 8) | (static_cast<uint32_t>(data[4]) << 16) |
      (static_cast<uint32_t>(data[5]) << 24);
  out->rpm = static_cast<int32_t>(raw) / 10.0f;
  return true;
}

enum class CanEventType { NONE, PHYSICAL_START, PHYSICAL_ESTOP, HEARTBEAT_TIMEOUT, DRIVER_FAULT };

struct CanEvent {
  CanEventType type = CanEventType::NONE;
  uint8_t detail = 0;
  uint16_t sequence = 0;
  uint8_t result = 0;
};

enum class MotionStartState {
  IDLE,
  CACHE_READY,
  START_PENDING,
  RUNNING,
  ESTOP_LATCHED,
  HEARTBEAT_LOST,
  START_FAILED
};

inline const char* motionStartStateName(MotionStartState state) {
  switch (state) {
    case MotionStartState::IDLE: return "IDLE";
    case MotionStartState::CACHE_READY: return "CACHE_READY";
    case MotionStartState::START_PENDING: return "START_PENDING";
    case MotionStartState::RUNNING: return "RUNNING";
    case MotionStartState::ESTOP_LATCHED: return "ESTOP_LATCHED";
    case MotionStartState::HEARTBEAT_LOST: return "HEARTBEAT_LOST";
    case MotionStartState::START_FAILED: return "START_FAILED";
  }
  return "IDLE";
}

// Correlates the priority-ordered 0x102 ACK with the local sequence in 0x112.
// A result=0 ACK means that the cached trigger was accepted, not that motion
// has started; the owner must call motionStarted() after observing telemetry.
class MotionStartStateMachine {
 public:
  explicit MotionStartStateMachine(double ack_timeout_sec = 0.4)
      : ack_timeout_sec_(ack_timeout_sec) {}

  void setAckTimeout(double timeout_sec) {
    std::lock_guard<std::mutex> lock(mutex_);
    ack_timeout_sec_ = timeout_sec;
  }

  MotionStartState state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }
  bool canSendCachedMotion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == MotionStartState::IDLE || state_ == MotionStartState::CACHE_READY;
  }
  bool canSendRunningMotion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == MotionStartState::RUNNING;
  }
  bool startPending() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == MotionStartState::START_PENDING;
  }
  uint16_t pendingSequence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_sequence_;
  }
  bool ackAccepted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ack_accepted_;
  }

  void cacheReady() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == MotionStartState::IDLE) state_ = MotionStartState::CACHE_READY;
  }

  // Returns true when the event is an accepted start request and motion writes
  // must be paused immediately. Rejection leaves cached commands intact.
  bool physicalStart(uint8_t result, uint16_t sequence, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result == 0) {
      pending_sequence_ = sequence;
      pending_since_sec_ = now;
      ack_accepted_ = false;
      state_ = MotionStartState::START_PENDING;
      const auto it = early_acks_.find(sequence);
      if (it != early_acks_.end() && it->second == 0) ack_accepted_ = true;
      if (it != early_acks_.end() && it->second != 0) state_ = MotionStartState::START_FAILED;
      return true;
    }
    if (result == 2) state_ = MotionStartState::START_FAILED;
    // result=1 is a safety/communication rejection. Cancel any old pending
    // request and preserve the prepared cache for the next human press, with
    // no automatic retry.
    if (result == 1) {
      pending_sequence_ = 0;
      pending_since_sec_ = 0;
      ack_accepted_ = false;
      if (state_ != MotionStartState::IDLE) state_ = MotionStartState::CACHE_READY;
    }
    return false;
  }

  // Returns true only for an ACK that matches the pending local sequence and
  // command 0x05. ACKs received first are retained for later correlation.
  bool acknowledge(uint16_t sequence, uint8_t command, uint8_t result) {
    if (command != 0x05) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    early_acks_[sequence] = result;
    if (state_ != MotionStartState::START_PENDING || sequence != pending_sequence_)
      return false;
    if (result == 0) {
      ack_accepted_ = true;
    } else {
      state_ = MotionStartState::START_FAILED;
    }
    return true;
  }

  bool timeout(double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == MotionStartState::START_PENDING && now - pending_since_sec_ >= ack_timeout_sec_) {
      state_ = MotionStartState::START_FAILED;
      return true;
    }
    return false;
  }

  void motionStarted() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == MotionStartState::START_PENDING && ack_accepted_)
      state_ = MotionStartState::RUNNING;
  }

  void heartbeatLost() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != MotionStartState::ESTOP_LATCHED) state_ = MotionStartState::HEARTBEAT_LOST;
  }

  void estopLatched() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = MotionStartState::ESTOP_LATCHED;
  }

  // Called only after the operator has released/reset the physical estop and
  // the caller has cleared all cached motion commands.
  void resetForReprepare() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_sequence_ = 0;
    pending_since_sec_ = 0;
    ack_accepted_ = false;
    early_acks_.clear();
    state_ = MotionStartState::IDLE;
  }

 private:
  MotionStartState state_ = MotionStartState::IDLE;
  double ack_timeout_sec_ = 0.4;
  double pending_since_sec_ = 0;
  uint16_t pending_sequence_ = 0;
  bool ack_accepted_ = false;
  std::map<uint16_t, uint8_t> early_acks_;
  mutable std::mutex mutex_;
};

inline bool decodeCanEvent(uint32_t can_id, bool is_extended, uint8_t dlc,
                           const uint8_t* data, CanEvent* event) {
  if (!event || !data || is_extended) return false;
  event->type = CanEventType::NONE;
  event->detail = 0;
  event->sequence = 0;
  event->result = 0;
  if (can_id == 0x112U) {
    if (dlc != 8 || data[1] != 0x01 || data[2] != 0x01) return false;
    event->result = data[3];
    event->sequence = static_cast<uint16_t>(data[4] << 8 | data[5]);
    if (data[0] == 0x01) event->type = CanEventType::PHYSICAL_START;
    else if (data[0] == 0x02) event->type = CanEventType::PHYSICAL_ESTOP;
    else return false;
    return true;
  }
  if (can_id == 0x101U && dlc == 8 && data[0] == 0x06) {
    event->detail = data[2];
    if (data[2] == 0x81) event->type = CanEventType::PHYSICAL_ESTOP;
    else if (data[2] == 0x80) event->type = CanEventType::HEARTBEAT_TIMEOUT;
    else event->type = CanEventType::DRIVER_FAULT;
    return true;
  }
  return false;
}

}  // namespace can_motor_interface
