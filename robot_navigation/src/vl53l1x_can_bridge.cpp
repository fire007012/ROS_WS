#include "robot_navigation/vl53l1x_can_bridge.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <limits>

namespace robot_navigation {
namespace {
std::string toString(uint64_t value) { return std::to_string(value); }

std::string rangeStatusText(uint8_t status) {
  std::ostringstream stream;
  const auto add = [&stream](const char* text) {
    if (stream.tellp() > 0) stream << '|';
    stream << text;
  };
  if (status & kStm32RangeValid) add("VALID");
  if (status & kStm32RangeOutOfRange) add("OUT_OF_RANGE");
  if (status & kStm32RangeTimeout) add("TIMEOUT");
  if (status & kStm32RangeI2cError) add("I2C_ERROR");
  if (status & kStm32RangeLowQuality) add("LOW_QUALITY");
  if (status & kStm32RangeEmergency) add("EMERGENCY");
  return stream.tellp() > 0 ? stream.str() : "NONE";
}
}  // namespace

Vl53l1xCanBridge::Vl53l1xCanBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  sensors_[0].name = "front_vl53l1x";
  sensors_[1].name = "left_vl53l1x";
  sensors_[2].name = "right_vl53l1x";
}

Vl53l1xCanBridge::~Vl53l1xCanBridge() {
  poll_timer_.stop();
  diagnostics_timer_.stop();
  alignment_.exit();
  publishAlignment();
  can_.close();
}

void Vl53l1xCanBridge::loadSensorParams(SensorState& state, const std::string& prefix,
                                        const std::string& default_name,
                                        const std::string& default_range_topic,
                                        const std::string& default_distance_topic,
                                        const std::string& default_frame,
                                        double default_min_range, double default_max_range,
                                        uint8_t radiation_type,
                                        const std::string& device_name) {
  pnh_.param(prefix + "/name", state.name, default_name);
  pnh_.param(prefix + "/range_topic", state.range_topic, default_range_topic);
  pnh_.param(prefix + "/distance_topic", state.distance_topic, default_distance_topic);
  pnh_.param(prefix + "/frame_id", state.frame_id, default_frame);
  pnh_.param(prefix + "/min_range", state.min_range, default_min_range);
  pnh_.param(prefix + "/max_range", state.max_range, default_max_range);
  pnh_.param(prefix + "/field_of_view", state.field_of_view, 0.47);
  state.radiation_type = radiation_type;
  state.device_name = device_name;
  state.min_range = std::max(0.0, state.min_range);
  state.max_range = std::max(state.min_range, state.max_range);
  state.field_of_view = std::max(0.001, state.field_of_view);
}

bool Vl53l1xCanBridge::configure() {
  pnh_.param("can_interface", can_interface_, std::string("can0"));
  int distance_can_id = static_cast<int>(distance_can_id_);
  int diagnostic_can_id = static_cast<int>(diagnostic_can_id_);
  pnh_.param("distance_can_id", distance_can_id, distance_can_id);
  pnh_.param("diagnostic_can_id", diagnostic_can_id, diagnostic_can_id);
  distance_can_id_ = static_cast<uint32_t>(std::max(0, distance_can_id));
  diagnostic_can_id_ = static_cast<uint32_t>(std::max(0, diagnostic_can_id));
  pnh_.param("poll_rate_hz", poll_rate_hz_, 200);
  pnh_.param("sensor_timeout_ms", sensor_timeout_ms_, 200);
  distance_can_id_ &= CAN_SFF_MASK;
  diagnostic_can_id_ &= CAN_SFF_MASK;
  poll_rate_hz_ = std::max(10, poll_rate_hz_);
  sensor_timeout_ms_ = std::max(1, sensor_timeout_ms_);

  loadSensorParams(sensors_[0], "front", "front_vl53l1x", "/front/range", "/vl53l1x_distance",
                   "front_range_link", 0.05, 3.0, stm32RangeRadiationType(0),
                   stm32RangeDeviceName(0));
  loadSensorParams(sensors_[1], "left", "left_vl53l1x", "/left/range", "/vl53l1x_distance_left",
                   "left_range_link", 0.05, 3.0, stm32RangeRadiationType(1),
                   stm32RangeDeviceName(1));
  loadSensorParams(sensors_[2], "right", "right_vl53l1x", "/right/range", "/vl53l1x_distance_right",
                   "right_range_link", 0.05, 3.0, stm32RangeRadiationType(2),
                   stm32RangeDeviceName(2));
  loadSensorParams(sensors_[3], "arm_upper_left", "arm_upper_left_vl53l1x",
                   "/arm_upper_left/range", "/arm_upper_left/distance_mm",
                   "arm_upper_left_range_link", 0.05, 3.0, stm32RangeRadiationType(3),
                   stm32RangeDeviceName(3));
  loadSensorParams(sensors_[4], "arm_upper_right", "arm_upper_right_vl53l1x",
                   "/arm_upper_right/range", "/arm_upper_right/distance_mm",
                   "arm_upper_right_range_link", 0.05, 3.0, stm32RangeRadiationType(4),
                   stm32RangeDeviceName(4));
  loadSensorParams(sensors_[kStm32RearSensorId], "rear", "rear_vl53l1x",
                   "/rear/range", "/vl53l1x_distance_rear", "rear_range_link",
                   0.05, 3.0, stm32RangeRadiationType(kStm32RearSensorId),
                   stm32RangeDeviceName(kStm32RearSensorId));
  pnh_.param("upper_alignment/enabled", alignment_enabled_, false);
  pnh_.param("upper_alignment/start_difference_mm", alignment_.start_mm, 50.0);
  pnh_.param("upper_alignment/stop_difference_mm", alignment_.stop_mm, 20.0);
  pnh_.param("upper_alignment/speed_mps", alignment_.speed_mps, 0.05);
  pnh_.param("upper_alignment/data_timeout_sec", alignment_.timeout_sec, 0.2);
  pnh_.param("upper_alignment/max_skew_sec", alignment_.skew_sec, 0.1);
  pnh_.param("upper_alignment/settled_sec", alignment_.settled_sec, 0.3);
  pnh_.param("upper_alignment/stages", alignment_stages_,
             std::vector<std::string>{"ARM_UPPER_ALIGN_A", "ARM_UPPER_ALIGN_B"});
  if (!alignment_.parametersValid() || (alignment_enabled_ && alignment_stages_.empty())) {
    ROS_ERROR("[stm32_range_bridge] invalid upper_alignment parameters");
    return false;
  }
  alignment_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_arm_alignment", 1);
  alignment_mode_pub_ = nh_.advertise<std_msgs::Bool>("/arm_upper_alignment/active", 1, true);
  alignment_done_pub_ = nh_.advertise<std_msgs::Bool>("/arm_upper_alignment/done", 1);
  emergency_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1);
  stage_sub_ = nh_.subscribe("/mission/state", 1, &Vl53l1xCanBridge::stageCallback, this);
  cancel_sub_ = nh_.subscribe("/mission/cancel", 1, &Vl53l1xCanBridge::cancelCallback, this);
  enabled_sub_ = nh_.subscribe("/mission/actions_enabled", 1, &Vl53l1xCanBridge::enabledCallback, this);
  estop_sub_ = nh_.subscribe("/emergency_stop", 1, &Vl53l1xCanBridge::estopCallback, this);
  reset_sub_ = nh_.subscribe<std_msgs::Bool>("/emergency_stop/reset", 1,
      [this](const std_msgs::Bool::ConstPtr& msg) { if (msg->data) estop_latched_ = false; });
  publishAlignment();
  for (auto& sensor : sensors_) {
    sensor.range_pub = nh_.advertise<sensor_msgs::Range>(sensor.range_topic, 10);
    sensor.distance_pub = nh_.advertise<std_msgs::Float32>(sensor.distance_topic, 10);
  }
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  return true;
}

bool Vl53l1xCanBridge::init() {
  if (!configure()) return false;
  if (!can_.open(can_interface_)) {
    ROS_ERROR("[stm32_range_bridge] unable to open SocketCAN interface %s", can_interface_.c_str());
    return false;
  }
  poll_timer_ = nh_.createSteadyTimer(ros::WallDuration(1.0 / poll_rate_hz_),
                                &Vl53l1xCanBridge::pollTimerCallback, this);
  diagnostics_timer_ = nh_.createSteadyTimer(ros::WallDuration(0.1),
                                        &Vl53l1xCanBridge::diagnosticsTimerCallback, this);
  ROS_INFO("[stm32_range_bridge] listening on %s (distance=0x%03X, diagnostics=0x%03X)",
           can_interface_.c_str(), distance_can_id_, diagnostic_can_id_);
  return true;
}

bool Vl53l1xCanBridge::updateSequence(SensorState& state, uint8_t sequence) {
  if (!state.has_sequence) {
    state.has_sequence = true;
    state.sequence = sequence;
    return true;
  }
  const uint8_t delta = vl53l1xSequenceDelta(state.sequence, sequence);
  state.sequence = sequence;
  if (delta == 0) {
    ++state.duplicate_frames;
    return false;
  }
  if (delta > 1) state.dropped_frames += delta - 1;
  return true;
}

bool Vl53l1xCanBridge::updateDiagnosticSequence(SensorState& state, uint8_t sequence) {
  if (!state.has_diagnostic_sequence) {
    state.has_diagnostic_sequence = true;
    state.diagnostic_sequence = sequence;
    return true;
  }
  const uint8_t delta = vl53l1xSequenceDelta(state.diagnostic_sequence, sequence);
  state.diagnostic_sequence = sequence;
  if (delta == 0) {
    ++state.diagnostic_duplicate_frames;
    return false;
  }
  if (delta > 1) state.diagnostic_dropped_frames += delta - 1;
  return true;
}

void Vl53l1xCanBridge::handleSample(const Vl53l1xSample& sample, const ros::Time& stamp) {
  SensorState& state = sensors_[sample.sensor_id];
  ++state.received;
  const auto received = ros::SteadyTime::now();
  alignment_.update(sample, received.toSec());
  if (sample.status & kStm32RangeEmergency) {
    estop_latched_ = true;
    actions_enabled_ = false;
    alignment_.exit();
    std_msgs::Bool emergency; emergency.data = true; emergency_pub_.publish(emergency);
  }
  if (!updateSequence(state, sample.sequence)) {
    if (!stm32RangeSampleValid(sample)) {
      state.has_valid = false;
      publishRange(state, stamp, false);
    }
    publishAlignment();
    return;
  }
  state.last_receive = stamp;
  state.last_receive_steady = received;
  state.status = sample.status;
  state.sigma_mm = sample.sigma_mm;
  state.distance_mm = sample.distance_mm;
  const bool valid = stm32RangeSampleValid(sample) &&
                     sample.distance_mm >= state.min_range * 1000.0 &&
                     sample.distance_mm <= state.max_range * 1000.0;
  if (sample.status & kStm32RangeEmergency) {
    ROS_ERROR_THROTTLE(1.0, "[stm32_range_bridge] %s reports EMERGENCY: %u mm (status=%s)",
                       state.device_name.c_str(), static_cast<unsigned int>(sample.distance_mm),
                       rangeStatusText(sample.status).c_str());
  }
  if (!valid) alignment_.invalidate(sample.sensor_id);
  state.has_valid = valid;
  if (!valid) {
    publishRange(state, stamp, false);
    publishAlignment();
    ++state.invalid_samples;
    ++state.consecutive_failures;
    ROS_WARN_THROTTLE(1.0, "[stm32_range_bridge] invalid %s sample: %u mm (status=%s)",
                      state.device_name.c_str(), static_cast<unsigned int>(sample.distance_mm),
                      rangeStatusText(sample.status).c_str());
    return;
  }
  state.has_valid = true;
  state.last_valid = received;
  state.consecutive_failures = 0;
  state.diagnostic_error_code = 0;
  publishRange(state, stamp, true);
  publishAlignment();
  if (sample.status & kStm32RangeLowQuality) {
    ROS_WARN_THROTTLE(1.0, "[stm32_range_bridge] %s sample has LOW_QUALITY status",
                      state.device_name.c_str());
  }
}

void Vl53l1xCanBridge::publishRange(SensorState& state, const ros::Time& stamp, bool valid) {
  sensor_msgs::Range range;
  range.header.stamp = stamp;
  range.header.frame_id = state.frame_id;
  range.radiation_type = state.radiation_type;
  range.field_of_view = state.field_of_view;
  range.min_range = state.min_range;
  range.max_range = state.max_range;
  range.range = valid ? state.distance_mm / 1000.0 : std::numeric_limits<float>::quiet_NaN();
  state.range_pub.publish(range);
  // Float32 distance topics use millimetres, including invalidation.
  std_msgs::Float32 distance;
  distance.data = valid ? state.distance_mm : std::numeric_limits<float>::quiet_NaN();
  state.distance_pub.publish(distance);
}

void Vl53l1xCanBridge::publishAlignment() {
  if (!alignment_mode_pub_) return;
  const double now = ros::SteadyTime::now().toSec();
  // Mirror the firmware's existing local stop scope even for a lateral
  // request. Upper sensors are comparison inputs and never enter this check.
  bool local_obstacle = false;
  for (size_t id = 0; id < 3; ++id)
    local_obstacle = local_obstacle || (sensors_[id].has_valid && sensors_[id].distance_mm <= 300);
  geometry_msgs::Twist command;
  if (local_obstacle) alignment_.stopMotion();
  else command.linear.y = alignment_.velocity(now);
  alignment_cmd_pub_.publish(command);
  std_msgs::Bool active; active.data = alignment_.active(); alignment_mode_pub_.publish(active);
  std_msgs::Bool done; done.data = !local_obstacle && alignment_.done(now); alignment_done_pub_.publish(done);
}

void Vl53l1xCanBridge::stageCallback(const std_msgs::String::ConstPtr& msg) {
  if (stage_ == msg->data) return;
  stage_ = msg->data;
  alignment_.exit();
  if (alignment_enabled_ && actions_enabled_ && !estop_latched_ &&
      std::find(alignment_stages_.begin(), alignment_stages_.end(), stage_) != alignment_stages_.end()) {
    // Consume buffered frames before setting the entry fence. Their sequences
    // are retained, but neither side's preceding-stage value can drive motion.
    struct can_frame buffered;
    int drained = 0;
    while (drained < 1000 && can_.receiveFrame(buffered, 0)) {
      handleFrame(buffered);
      ++drained;
    }
    if (drained == 1000) {
      ROS_ERROR("[stm32_range_bridge] excessive CAN backlog; alignment entry refused");
    } else if (actions_enabled_ && !estop_latched_) {
      alignment_.enter(ros::SteadyTime::now().toSec());
    }
  }
  publishAlignment();
}
void Vl53l1xCanBridge::cancelCallback(const std_msgs::Empty::ConstPtr&) {
  actions_enabled_ = false;
  alignment_.exit();
  publishAlignment();
}
void Vl53l1xCanBridge::enabledCallback(const std_msgs::Bool::ConstPtr& msg) {
  actions_enabled_ = msg->data;
  if (!actions_enabled_) { alignment_.exit(); publishAlignment(); }
}
void Vl53l1xCanBridge::estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  estop_latched_ = true;
  actions_enabled_ = false;
  alignment_.exit();
  publishAlignment();
}

void Vl53l1xCanBridge::handleDiagnostic(const Vl53l1xDiagnosticFrame& diagnostic, const ros::Time& stamp) {
  ++diagnostic_frame_count_;
  if (diagnostic.sensor_id == 0xFF) {
    ROS_WARN_THROTTLE(1.0, "[stm32_range_bridge] STM32 bus diagnostic: error=%u consecutive=%u",
                      diagnostic.error_code, diagnostic.consecutive_errors);
    return;
  }
  SensorState& state = sensors_[diagnostic.sensor_id];
  if (!updateDiagnosticSequence(state, diagnostic.sequence)) return;
  state.consecutive_failures = diagnostic.consecutive_errors;
  state.diagnostic_error_code = diagnostic.error_code;
  state.status = diagnostic.last_status;
  state.distance_mm = diagnostic.last_valid_distance_mm;
  if (diagnostic.error_code != 0 || diagnostic.consecutive_errors != 0 ||
      stm32RangeStatusHasFailure(diagnostic.last_status) ||
      !(diagnostic.last_status & kStm32RangeValid)) {
    state.has_valid = false;
    alignment_.invalidate(diagnostic.sensor_id);
    publishRange(state, stamp, false);
  }
  if (diagnostic.last_status & kStm32RangeEmergency) {
    estop_latched_ = true;
    actions_enabled_ = false;
    alignment_.exit();
    std_msgs::Bool emergency; emergency.data = true; emergency_pub_.publish(emergency);
  }
  publishAlignment();
  if (diagnostic.error_code != 0 || diagnostic.consecutive_errors != 0) {
    ROS_WARN_THROTTLE(1.0, "[stm32_range_bridge] %s diagnostic error=%u consecutive=%u",
                      state.device_name.c_str(), diagnostic.error_code, diagnostic.consecutive_errors);
  }
  if (diagnostic.last_status & kStm32RangeEmergency) {
    ROS_ERROR_THROTTLE(1.0, "[stm32_range_bridge] %s diagnostic reports EMERGENCY",
                       state.device_name.c_str());
  }
}

void Vl53l1xCanBridge::handleFrame(const struct can_frame& frame) {
  const ros::Time stamp = ros::Time::now();
  Vl53l1xSample sample;
  Vl53l1xDiagnosticFrame diagnostic;
  if ((frame.can_id & CAN_SFF_MASK) == distance_can_id_) {
    struct can_frame protocol_frame = frame;
    protocol_frame.can_id = (frame.can_id & ~CAN_SFF_MASK) | 0x110;
    if (!parseVl53l1xSample(protocol_frame, sample)) {
      ++invalid_frame_count_;
      if (frame.can_dlc >= 2 && frame.data[1] < kStm32RangeSensorCount) ++sensors_[frame.data[1]].invalid_frames;
      if (frame.can_dlc >= 2 && frame.data[1] < kStm32RangeSensorCount) {
        auto& state = sensors_[frame.data[1]];
        state.has_valid = false;
        alignment_.invalidate(frame.data[1]);
        publishRange(state, stamp, false);
      }
      ROS_WARN_THROTTLE(1.0, "[stm32_range_bridge] discarded malformed distance CAN frame");
    } else {
      handleSample(sample, stamp);
    }
  } else if ((frame.can_id & CAN_SFF_MASK) == diagnostic_can_id_) {
    struct can_frame protocol_frame = frame;
    protocol_frame.can_id = (frame.can_id & ~CAN_SFF_MASK) | 0x111;
    if (!parseVl53l1xDiagnostic(protocol_frame, diagnostic)) {
      ++invalid_frame_count_;
      if (frame.can_dlc >= 2 && frame.data[1] < kStm32RangeSensorCount) {
        auto& state = sensors_[frame.data[1]];
        state.has_valid = false;
        alignment_.invalidate(frame.data[1]);
        publishRange(state, stamp, false);
      }
    } else {
      handleDiagnostic(diagnostic, stamp);
    }
  } else {
    ++invalid_frame_count_;
  }
  publishAlignment();
}

void Vl53l1xCanBridge::pollTimerCallback(const ros::SteadyTimerEvent&) {
  struct can_frame frame;
  int processed = 0;
  while (processed++ < 100 && can_.receiveFrame(frame, 0)) {
    handleFrame(frame);
  }
  const auto now = ros::SteadyTime::now();
  for (auto& state : sensors_) {
    if (state.has_valid && !vl53l1xSampleFresh(true, now, state.last_valid, sensor_timeout_ms_)) {
      state.has_valid = false;
      publishRange(state, ros::Time::now(), false);
    }
  }
  publishAlignment();
}

void Vl53l1xCanBridge::diagnosticsTimerCallback(const ros::SteadyTimerEvent&) {
  publishDiagnostics(ros::Time::now());
}

void Vl53l1xCanBridge::publishDiagnostics(const ros::Time& now) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = now;
  for (const auto& state : sensors_) {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "STM32 Range/" + state.name;
    const auto steady_now = ros::SteadyTime::now();
    const bool fresh = vl53l1xSampleFresh(state.has_valid, steady_now, state.last_valid, sensor_timeout_ms_);
    if (!fresh || state.consecutive_failures != 0 || state.diagnostic_error_code != 0 ||
        stm32RangeStatusHasFailure(state.status)) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = fresh ? "measurement error" : (state.has_valid ? "timeout" : "no valid sample");
    } else if (state.status & kStm32RangeEmergency) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "emergency distance";
    } else if (state.status & kStm32RangeLowQuality) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "low quality";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "valid";
    }
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
    };
    add("data_age_ms", state.has_valid ? toString(static_cast<uint64_t>((steady_now - state.last_valid).toSec() * 1000.0)) : "inf");
    add("receive_age_ms", state.has_sequence ? std::to_string((steady_now - state.last_receive_steady).toSec() * 1000.0) : "inf");
    add("last_receive_time", state.last_receive.isZero() ? "never" : std::to_string(state.last_receive.toSec()));
    add("valid", state.has_valid && fresh ? "true" : "false");
    add("status", toString(state.status));
    add("status_text", rangeStatusText(state.status));
    add("diagnostic_error_code", toString(state.diagnostic_error_code));
    add("distance_mm", toString(state.distance_mm));
    add("sigma_mm", toString(state.sigma_mm));
    add("radiation_type", state.radiation_type == sensor_msgs::Range::ULTRASOUND ? "ultrasound" : "infrared");
    add("min_range_m", std::to_string(state.min_range));
    add("max_range_m", std::to_string(state.max_range));
    add("sequence", toString(state.sequence));
    add("dropped_frames", toString(state.dropped_frames));
    add("duplicate_frames", toString(state.duplicate_frames));
    add("diagnostic_sequence", toString(state.diagnostic_sequence));
    add("diagnostic_dropped_frames", toString(state.diagnostic_dropped_frames));
    add("diagnostic_duplicate_frames", toString(state.diagnostic_duplicate_frames));
    add("invalid_samples", toString(state.invalid_samples));
    add("invalid_frames", toString(state.invalid_frames));
    add("consecutive_failures", toString(state.consecutive_failures));
    status.hardware_id = state.device_name + " (" + state.frame_id + ")";
    array.status.push_back(status);
  }
  diagnostic_msgs::DiagnosticStatus bus;
  bus.name = "STM32 Range/CAN";
  bus.level = can_.isOpen() ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
  bus.message = can_.isOpen() ? "connected" : "disconnected";
  diagnostic_msgs::KeyValue invalid; invalid.key = "invalid_frames"; invalid.value = toString(invalid_frame_count_); bus.values.push_back(invalid);
  diagnostic_msgs::KeyValue received; received.key = "diagnostic_frames"; received.value = toString(diagnostic_frame_count_); bus.values.push_back(received);
  array.status.push_back(bus);
  diagnostics_pub_.publish(array);
}

}  // namespace robot_navigation

#ifndef VL53L1X_BRIDGE_NO_MAIN
int main(int argc, char** argv) {
  ros::init(argc, argv, "vl53l1x_can_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  robot_navigation::Vl53l1xCanBridge bridge(nh, pnh);
  if (!bridge.init()) return 1;
  ros::spin();
  return 0;
}

#endif
