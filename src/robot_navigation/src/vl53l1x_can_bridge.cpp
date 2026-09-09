#include "robot_navigation/vl53l1x_can_bridge.h"

#include <algorithm>
#include <cmath>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Float32.h>

namespace robot_navigation {
namespace {
std::string toString(uint64_t value) { return std::to_string(value); }
}  // namespace

Vl53l1xCanBridge::Vl53l1xCanBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  sensors_[0].name = "front";
  sensors_[1].name = "left";
  sensors_[2].name = "right";
}

Vl53l1xCanBridge::~Vl53l1xCanBridge() {
  poll_timer_.stop();
  diagnostics_timer_.stop();
  can_.close();
}

void Vl53l1xCanBridge::loadSensorParams(SensorState& state, const std::string& prefix,
                                        const std::string& default_name,
                                        const std::string& default_range_topic,
                                        const std::string& default_distance_topic,
                                        const std::string& default_frame) {
  pnh_.param(prefix + "/name", state.name, default_name);
  pnh_.param(prefix + "/range_topic", state.range_topic, default_range_topic);
  pnh_.param(prefix + "/distance_topic", state.distance_topic, default_distance_topic);
  pnh_.param(prefix + "/frame_id", state.frame_id, default_frame);
  pnh_.param(prefix + "/min_range", state.min_range, 0.04);
  pnh_.param(prefix + "/max_range", state.max_range, 4.0);
  pnh_.param(prefix + "/field_of_view", state.field_of_view, 0.47);
  state.min_range = std::max(0.0, state.min_range);
  state.max_range = std::max(state.min_range, state.max_range);
  state.field_of_view = std::max(0.001, state.field_of_view);
}

bool Vl53l1xCanBridge::init() {
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

  loadSensorParams(sensors_[0], "front", "front", "/front/range", "/vl53l1x_distance", "front_range_link");
  loadSensorParams(sensors_[1], "left", "left", "/left/range", "/vl53l1x_distance_left", "left_range_link");
  loadSensorParams(sensors_[2], "right", "right", "/right/range", "/vl53l1x_distance_right", "right_range_link");
  for (auto& sensor : sensors_) {
    sensor.range_pub = nh_.advertise<sensor_msgs::Range>(sensor.range_topic, 10);
    sensor.distance_pub = nh_.advertise<std_msgs::Float32>(sensor.distance_topic, 10);
  }
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  if (!can_.open(can_interface_)) {
    ROS_ERROR("[vl53l1x_can_bridge] unable to open SocketCAN interface %s", can_interface_.c_str());
    return false;
  }
  poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_hz_),
                                &Vl53l1xCanBridge::pollTimerCallback, this);
  diagnostics_timer_ = nh_.createTimer(ros::Duration(0.1),
                                        &Vl53l1xCanBridge::diagnosticsTimerCallback, this);
  ROS_INFO("[vl53l1x_can_bridge] listening on %s (distance=0x%03X, diagnostics=0x%03X)",
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

void Vl53l1xCanBridge::handleSample(const Vl53l1xSample& sample, const ros::Time& stamp) {
  SensorState& state = sensors_[sample.sensor_id];
  ++state.received;
  state.last_receive = stamp;
  updateSequence(state, sample.sequence);
  state.status = sample.status;
  state.sigma_mm = sample.sigma_mm;
  state.distance_mm = sample.distance_mm;
  const bool valid = (sample.status & 0x01) != 0 && sample.distance_mm != 0xFFFF &&
                     sample.distance_mm >= state.min_range * 1000.0 &&
                     sample.distance_mm <= state.max_range * 1000.0;
  if (!valid) {
    ++state.invalid_samples;
    ++state.consecutive_failures;
    if (sample.status & 0x20) ROS_ERROR_THROTTLE(1.0, "[vl53l1x_can_bridge] sensor %s reports EMERGENCY", state.name.c_str());
    return;
  }
  state.has_valid = true;
  state.last_valid = stamp;
  state.consecutive_failures = 0;
  sensor_msgs::Range range;
  range.header.stamp = stamp;
  range.header.frame_id = state.frame_id;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.field_of_view = state.field_of_view;
  range.min_range = state.min_range;
  range.max_range = state.max_range;
  range.range = sample.distance_mm / 1000.0;
  state.range_pub.publish(range);
  std_msgs::Float32 distance;
  distance.data = static_cast<float>(sample.distance_mm);
  state.distance_pub.publish(distance);
}

void Vl53l1xCanBridge::handleDiagnostic(const Vl53l1xDiagnosticFrame& diagnostic, const ros::Time& stamp) {
  ++diagnostic_frame_count_;
  if (diagnostic.sensor_id == 0xFF) {
    ROS_WARN_THROTTLE(1.0, "[vl53l1x_can_bridge] STM32 bus diagnostic: error=%u consecutive=%u",
                      diagnostic.error_code, diagnostic.consecutive_errors);
    return;
  }
  SensorState& state = sensors_[diagnostic.sensor_id];
  state.consecutive_failures = diagnostic.consecutive_errors;
  state.status = diagnostic.last_status;
  state.distance_mm = diagnostic.last_valid_distance_mm;
  (void)stamp;
  if (diagnostic.error_code != 0 || diagnostic.consecutive_errors != 0) {
    ROS_WARN_THROTTLE(1.0, "[vl53l1x_can_bridge] sensor %s diagnostic error=%u consecutive=%u",
                      state.name.c_str(), diagnostic.error_code, diagnostic.consecutive_errors);
  }
}

void Vl53l1xCanBridge::pollTimerCallback(const ros::TimerEvent&) {
  struct can_frame frame;
  int processed = 0;
  while (processed++ < 100 && can_.receiveFrame(frame, 0)) {
    const ros::Time stamp = ros::Time::now();
    Vl53l1xSample sample;
    Vl53l1xDiagnosticFrame diagnostic;
    if ((frame.can_id & CAN_SFF_MASK) == distance_can_id_) {
      struct can_frame protocol_frame = frame;
      protocol_frame.can_id = (frame.can_id & ~CAN_SFF_MASK) | 0x110;
      if (!parseVl53l1xSample(protocol_frame, sample)) {
        ++invalid_frame_count_;
        if (frame.can_dlc >= 2 && frame.data[1] < 3) ++sensors_[frame.data[1]].invalid_frames;
        ROS_WARN_THROTTLE(1.0, "[vl53l1x_can_bridge] discarded malformed distance CAN frame");
      } else {
        handleSample(sample, stamp);
      }
    } else if ((frame.can_id & CAN_SFF_MASK) == diagnostic_can_id_) {
      struct can_frame protocol_frame = frame;
      protocol_frame.can_id = (frame.can_id & ~CAN_SFF_MASK) | 0x111;
      if (!parseVl53l1xDiagnostic(protocol_frame, diagnostic)) {
        ++invalid_frame_count_;
      } else {
        handleDiagnostic(diagnostic, stamp);
      }
    } else {
      ++invalid_frame_count_;
    }
  }
}

void Vl53l1xCanBridge::diagnosticsTimerCallback(const ros::TimerEvent&) {
  publishDiagnostics(ros::Time::now());
}

void Vl53l1xCanBridge::publishDiagnostics(const ros::Time& now) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = now;
  for (const auto& state : sensors_) {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "VL53L1X/" + state.name;
    const bool fresh = vl53l1xSampleFresh(state.has_valid, now, state.last_valid, sensor_timeout_ms_);
    status.level = fresh && state.consecutive_failures == 0 ? diagnostic_msgs::DiagnosticStatus::OK
                                                            : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = fresh ? "valid" : (state.has_valid ? "timeout" : "no valid sample");
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
    };
    add("data_age_ms", state.has_valid ? toString(static_cast<uint64_t>((now - state.last_valid).toSec() * 1000.0)) : "inf");
    add("last_receive_time", state.last_receive.isZero() ? "never" : std::to_string(state.last_receive.toSec()));
    add("valid", state.has_valid && fresh ? "true" : "false");
    add("status", toString(state.status));
    add("distance_mm", toString(state.distance_mm));
    add("sigma_mm", toString(state.sigma_mm));
    add("sequence", toString(state.sequence));
    add("dropped_frames", toString(state.dropped_frames));
    add("duplicate_frames", toString(state.duplicate_frames));
    add("invalid_samples", toString(state.invalid_samples));
    add("invalid_frames", toString(state.invalid_frames));
    add("consecutive_failures", toString(state.consecutive_failures));
    status.hardware_id = state.frame_id;
    array.status.push_back(status);
  }
  diagnostic_msgs::DiagnosticStatus bus;
  bus.name = "VL53L1X/CAN";
  bus.level = can_.isOpen() ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
  bus.message = can_.isOpen() ? "connected" : "disconnected";
  diagnostic_msgs::KeyValue invalid; invalid.key = "invalid_frames"; invalid.value = toString(invalid_frame_count_); bus.values.push_back(invalid);
  diagnostic_msgs::KeyValue received; received.key = "diagnostic_frames"; received.value = toString(diagnostic_frame_count_); bus.values.push_back(received);
  array.status.push_back(bus);
  diagnostics_pub_.publish(array);
}

}  // namespace robot_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "vl53l1x_can_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  robot_navigation::Vl53l1xCanBridge bridge(nh, pnh);
  if (!bridge.init()) return 1;
  ros::spin();
  return 0;
}
