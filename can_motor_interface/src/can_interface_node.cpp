#include "can_motor_interface/can_interface.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace can_motor_interface {

CanInterfaceNode::CanInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      socket_fd_(-1),
      running_(true),
      have_command_(false),
      tx_can_id_(0x100),
      rx_can_id_(0x101),
      rx_can_id_filter_enable_(true),
      use_extended_frame_(false),
      broadcast_index_(0xFF),
      acceleration_rpm_s_(50),
      report_mask_(7),
      report_configured_(false),
      last_report_request_sec_(0),
      max_rpm_(3000.0f),
      motor_count_(4),
      heartbeat_timeout_sec_(1.0),
      event_dedup_window_sec_(0.5),
      command_publish_rate_hz_(50.0),
      command_timeout_sec_(0.5) {
  pnh_.param<std::string>("can_device", can_device_, "can0");

  int tx_can_id_i = static_cast<int>(tx_can_id_);
  int rx_can_id_i = static_cast<int>(rx_can_id_);
  pnh_.param("tx_can_id", tx_can_id_i, tx_can_id_i);
  pnh_.param("rx_can_id", rx_can_id_i, rx_can_id_i);
  tx_can_id_ = static_cast<uint32_t>(tx_can_id_i);
  rx_can_id_ = static_cast<uint32_t>(rx_can_id_i);
  pnh_.param("rx_can_id_filter_enable", rx_can_id_filter_enable_, rx_can_id_filter_enable_);
  pnh_.param("use_extended_frame", use_extended_frame_, use_extended_frame_);
  // Validate both IDs before any socket/thread is created.
  socketCanId(tx_can_id_, use_extended_frame_);
  socketCanId(rx_can_id_, use_extended_frame_);
  if (tx_can_id_ != 0x100 || rx_can_id_ != 0x101 || use_extended_frame_ || !rx_can_id_filter_enable_)
    throw std::invalid_argument("STM32_Motor_Controller requires standard TX=0x100 RX=0x101 with filtering enabled");
  ROS_INFO("STM32 custom protocol: device=%s TX=0x%X RX=0x%X %s frames. "
           "This is NOT native Y42 passthrough; CAN bitrate must match STM32 CAN1.",
           can_device_.c_str(), tx_can_id_, rx_can_id_,
           use_extended_frame_ ? "extended" : "standard");

  pnh_.param("acceleration_rpm_s", acceleration_rpm_s_, acceleration_rpm_s_);
  pnh_.param("report_mask", report_mask_, report_mask_);
  if (acceleration_rpm_s_ < 0 || acceleration_rpm_s_ > 255 || report_mask_ < 0 ||
      report_mask_ > 15 || (report_mask_ & 5) != 5) {
    throw std::invalid_argument("Require acceleration_rpm_s=0..255 and report_mask including basic + velocity (bits 0 and 2)");
  }

  int broadcast_index_i = static_cast<int>(broadcast_index_);
  pnh_.param("broadcast_index", broadcast_index_i, broadcast_index_i);
  broadcast_index_ = static_cast<uint8_t>(broadcast_index_i & 0xFF);

  nh_.param("/robot/max_rpm", max_rpm_, max_rpm_);
  pnh_.param("max_rpm", max_rpm_, max_rpm_);
  pnh_.param("motor_count", motor_count_, motor_count_);
  pnh_.param("heartbeat_timeout_sec", heartbeat_timeout_sec_, heartbeat_timeout_sec_);
  pnh_.param("event_dedup_window_sec", event_dedup_window_sec_, event_dedup_window_sec_);
  pnh_.param("command_publish_rate_hz", command_publish_rate_hz_, command_publish_rate_hz_);
  pnh_.param("command_timeout_sec", command_timeout_sec_, command_timeout_sec_);
  command_publish_rate_hz_ = std::max(1.0, command_publish_rate_hz_);
  command_timeout_sec_ = std::max(0.05, command_timeout_sec_);

  if (motor_count_ != 4 || !std::isfinite(max_rpm_) || max_rpm_ <= 0 || max_rpm_ > 3000 ||
      !std::isfinite(heartbeat_timeout_sec_) || heartbeat_timeout_sec_ < 0.6 ||
      !std::isfinite(command_publish_rate_hz_) || !std::isfinite(command_timeout_sec_))
    throw std::invalid_argument("Require four chassis motors, max_rpm=0..3000 and feedback timeout >=0.6s");
  pnh_.getParam("motor_indices", wheel_map_.indices);
  pnh_.getParam("direction_signs", wheel_map_.signs);
  wheel_map_.validate();
  for (size_t i = 0; i < 4; ++i)
    ROS_INFO("Logical wheel %zu -> STM32 index %d / CAN2 address %d, sign=%d", i,
             wheel_map_.indices[i], wheel_map_.indices[i] + 1, wheel_map_.signs[i]);
  motor_state_rpm_.assign(4, 0.0f);
  motor_status_flags_.assign(4, 0);
  target_rpm_.assign(4, 0.0f);

  cmd_sub_ = nh_.subscribe("/motor_velocity_cmd", 20, &CanInterfaceNode::cmdCallback, this);
  software_estop_sub_ = nh_.subscribe("/emergency_stop", 10, &CanInterfaceNode::softwareEstopCallback, this);
  emergency_reset_sub_ = nh_.subscribe("/emergency_stop/reset", 2, &CanInterfaceNode::emergencyResetCallback, this);
  remote_start_sub_ = nh_.subscribe("/start_signal/remote", 2, &CanInterfaceNode::remoteStartCallback, this);
  motor_state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor_state", 20);
  motor_status_flag_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/motor_status_flags", 20);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 10, true);
  robot_state_pub_ = nh_.advertise<std_msgs::String>("/robot_state", 10, true);
  display_pub_ = nh_.advertise<std_msgs::String>("/robot_display", 10, true);
  stop_all_pub_ = nh_.advertise<std_msgs::Empty>("/stop_all", 2, true);
  chassis_lock_pub_ = nh_.advertise<std_msgs::Bool>("/chassis_lock", 2, true);
  fixed_route_hold_pub_ = nh_.advertise<std_msgs::Bool>("/fixed_route/hold", 2, true);
  move_base_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 2, false);
  can_rx_pub_ = nh_.advertise<can_msgs::Frame>("/can_rx", 50);
  link_ready_pub_ = nh_.advertise<std_msgs::Bool>("/motor_link_ready", 2, true);
  monitor_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &CanInterfaceNode::timerCallback, this);
  command_timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / command_publish_rate_hz_),
      &CanInterfaceNode::commandTimerCallback, this);
  probe_timer_ = nh_.createWallTimer(ros::WallDuration(0.05), &CanInterfaceNode::probeTimerCallback, this);

  if (!openSocket()) {
    ROS_WARN("CAN socket open failed at startup, will retry in timer.");
  }

  last_rx_time_sec_.store(ros::SteadyTime::now().toSec());
  std_msgs::Bool initial_estop; initial_estop.data = false; emergency_stop_pub_.publish(initial_estop);
  std_msgs::String initial_state; initial_state.data = "RUNNING"; robot_state_pub_.publish(initial_state);
  std_msgs::Bool ready; ready.data = false; link_ready_pub_.publish(ready);
  rx_thread_ = std::thread(&CanInterfaceNode::canReceiveThread, this);
}

CanInterfaceNode::~CanInterfaceNode() {
  running_.store(false);
  closeSocket();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

bool CanInterfaceNode::openSocket() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ >= 0) {
    return true;
  }

  // Nonblocking I/O keeps a full TX queue or a silent/disconnected peer from
  // blocking command expiry, shutdown, or socket reconnection.
  int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR_THROTTLE(1.0, "socket(PF_CAN, SOCK_RAW) failed: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR_THROTTLE(1.0, "ioctl(SIOCGIFINDEX) failed for %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0 || !(ifr.ifr_flags & IFF_UP)) {
    ROS_ERROR_THROTTLE(1.0, "CAN interface %s is not UP; check ip -details link show %s "
                       "and configure the same bitrate as STM32 CAN1.",
                       can_device_.c_str(), can_device_.c_str());
    close(fd);
    return false;
  }

  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR_THROTTLE(1.0, "bind() failed on %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  socket_fd_ = fd;
  report_configured_.store(false);
  heartbeat_ack_time_sec_.store(0);
  link_open_time_sec_.store(ros::SteadyTime::now().toSec());
  {
    std::lock_guard<std::mutex> telemetry_lock(telemetry_mutex_);
    driver_health_ = DriverHealth{};
    motion_ack_health_ = MotionAckHealth{};
    firmware_stats_ = FirmwareStats{};
    stats_time_sec_.fill(0);
  }
  ROS_INFO("SocketCAN bound on %s; binding/queued TX does not prove STM32 reception.", can_device_.c_str());
  return true;
}

void CanInterfaceNode::closeSocket() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanInterfaceNode::socketAvailable() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  return socket_fd_ >= 0;
}

bool CanInterfaceNode::sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ < 0) {
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  const uint8_t safe_dlc = std::min<uint8_t>(dlc, 8);
  frame.can_dlc = safe_dlc;
  std::memcpy(frame.data, data, safe_dlc);

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    ROS_ERROR_THROTTLE(1.0, "CAN TX failed on %s ID=0x%X: %s; check link state, "
                       "bitrate, STM32 ACK and bus wiring.", can_device_.c_str(),
                       can_id & CAN_EFF_MASK, std::strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  ROS_INFO_ONCE("First CAN TX queued on %s: ID=0x%X %s DLC=%u. "
                "Use STM32 RX/feedback to confirm delivery.", can_device_.c_str(),
                can_id & CAN_EFF_MASK, (can_id & CAN_EFF_FLAG) ? "extended" : "standard", safe_dlc);
  return true;
}

bool CanInterfaceNode::sendSpeedCommand(uint8_t motor_index, float target_rpm) {
  const float rpm = std::max(-max_rpm_, std::min(max_rpm_, target_rpm));
  const int32_t rpm_i = static_cast<int32_t>(std::lround(rpm));

  return motion_.run([&] {
    if (!linkHealthy()) return true;
    const can_frame frame = stm32Command(tx_can_id_, false, 0x01,
        static_cast<uint8_t>(wheel_map_.indices[motor_index]),
        rpm_i * wheel_map_.signs[motor_index], static_cast<uint8_t>(acceleration_rpm_s_), 0);
    if (!sendCanFrame(frame.can_id, frame.data, frame.can_dlc)) return false;
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    motion_ack_health_.sent(motor_index, ros::SteadyTime::now().toSec());
    return true;
  });
}

bool CanInterfaceNode::sendControlCommand(uint8_t command, int32_t value) {
  const can_frame frame = stm32Command(tx_can_id_, use_extended_frame_, command, broadcast_index_, value);
  return sendCanFrame(frame.can_id, frame.data, frame.can_dlc);
}

void CanInterfaceNode::softwareEstopCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && !motion_.latched()) triggerEmergencyStop("software estop");
}

void CanInterfaceNode::emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  if (!motion_.reset([&] {
    if (!linkHealthy() || ros::SteadyTime::now().toSec() - last_failure_time_sec_.load() < 1.0) return false;
    std::lock_guard<std::mutex> lock(command_mutex_);
    have_command_ = false;
    std::fill(target_rpm_.begin(), target_rpm_.end(), 0.0f);
    std::lock_guard<std::mutex> telemetry_lock(telemetry_mutex_);
    motion_ack_health_ = MotionAckHealth{};
    return true;
  })) {
    ROS_WARN("ESTOP reset denied: require fresh driver replies, heartbeat ACK and stable firmware diagnostics");
    return;
  }
  ROS_WARN("[can_interface] ESTOP reset accepted by upper safety policy");
  std_msgs::Bool estop; estop.data = false; emergency_stop_pub_.publish(estop);
  std_msgs::String state; state.data = "RUNNING"; robot_state_pub_.publish(state);
  std_msgs::Bool lock; lock.data = false; chassis_lock_pub_.publish(lock);
  std_msgs::Bool hold; hold.data = false; fixed_route_hold_pub_.publish(hold);
}

void CanInterfaceNode::remoteStartCallback(const std_msgs::Empty::ConstPtr&) {
  if (!motion_.run([&] { return !linkHealthy() || sendControlCommand(0x05); }))
    triggerEmergencyStop("synchronous start transmit failure");
}

void CanInterfaceNode::cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (std::any_of(msg->data.begin(), msg->data.end(), [](float rpm) { return !std::isfinite(rpm); })) {
    triggerEmergencyStop("nonfinite motor velocity command");
    return;
  }
  if (msg->data.size() != static_cast<size_t>(motor_count_)) {
    triggerEmergencyStop("motor velocity command must have exactly four values");
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  std::fill(target_rpm_.begin(), target_rpm_.end(), 0.0f);
  for (size_t i = 0; i < target_rpm_.size(); ++i) {
    target_rpm_[i] = msg->data[i];
  }
  last_command_time_ = ros::SteadyTime::now();
  have_command_ = true;
}

void CanInterfaceNode::commandTimerCallback(const ros::WallTimerEvent&) {
  if (motion_.latched() || !linkHealthy()) return;
  std::vector<float> targets;
  bool command_fresh = false;
  bool command_received = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    targets = target_rpm_;
    command_received = have_command_;
    command_fresh = have_command_ &&
                    (ros::SteadyTime::now() - last_command_time_).toSec() <= command_timeout_sec_;
  }

  if (!command_received) {
    ROS_WARN_THROTTLE(5.0, "Waiting for /motor_velocity_cmd; no speed frames sent. "
                       "Start planner + cmd_vel_mux or the custom-protocol keyboard test.");
    return;
  }

  // Keep the latest target on the wire at a fixed cadence; expire to an explicit stop.
  if (!command_fresh) {
    std::fill(targets.begin(), targets.end(), 0.0f);
  }
  for (size_t i = 0; i < targets.size(); ++i) {
    if (!sendSpeedCommand(static_cast<uint8_t>(i), targets[i])) {
      triggerEmergencyStop("CAN transmit failure; explicit reset required after link repair");
      break;
    }
  }
}

bool CanInterfaceNode::isDuplicateEvent(uint8_t key, const ros::SteadyTime& now) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  const auto it = last_event_times_.find(key);
  if (it != last_event_times_.end() && (now - it->second).toSec() < event_dedup_window_sec_) return true;
  last_event_times_[key] = now;
  return false;
}

void CanInterfaceNode::publishStopSignals() {
  std_msgs::Bool lock; lock.data = true; chassis_lock_pub_.publish(lock);
  std_msgs::Bool hold; hold.data = true; fixed_route_hold_pub_.publish(hold);
  std_msgs::Empty stop; stop_all_pub_.publish(stop);
  actionlib_msgs::GoalID cancel; move_base_cancel_pub_.publish(cancel);
}

void CanInterfaceNode::triggerEmergencyStop(const char* reason) {
  last_failure_time_sec_.store(ros::SteadyTime::now().toSec());
  const bool send_stop = !motion_.latched();
  const bool first = motion_.stop([&] { if (send_stop) sendControlCommand(0x03); });
  if (first) {
    ROS_ERROR("[can_interface] ESTOP latched: %s", reason);
    publishStopSignals();
    std_msgs::String state; state.data = "ESTOP"; robot_state_pub_.publish(state);
    std_msgs::String display; display.data = "急停: " + std::string(reason); display_pub_.publish(display);
  } else {
    ROS_WARN_THROTTLE(1.0, "[can_interface] duplicate ESTOP ignored: %s", reason);
    return;
  }
  std_msgs::Bool estop; estop.data = true; emergency_stop_pub_.publish(estop);
}

void CanInterfaceNode::handleCanEvent(const CanEvent& event) {
  switch (event.type) {
    case CanEventType::PHYSICAL_START: {
      if (isDuplicateEvent(0x12, ros::SteadyTime::now())) return;
      ROS_INFO("[can_interface] physical start button pressed; STM32 already executed 0x05");
      std_msgs::String state; state.data = motion_.latched() ? "ESTOP" : "RUNNING"; robot_state_pub_.publish(state);
      std_msgs::String display; display.data = motion_.latched() ? "急停中" : "物理启动"; display_pub_.publish(display);
      break;
    }
    case CanEventType::PHYSICAL_ESTOP:
      if (!isDuplicateEvent(0x20, ros::SteadyTime::now())) triggerEmergencyStop("physical estop button");
      break;
    case CanEventType::HEARTBEAT_TIMEOUT:
      if (!isDuplicateEvent(0x21, ros::SteadyTime::now())) triggerEmergencyStop("STM32 heartbeat timeout");
      break;
    case CanEventType::DRIVER_FAULT:
      if (!isDuplicateEvent(static_cast<uint8_t>(0x30U + event.detail), ros::SteadyTime::now())) {
        ROS_ERROR("[can_interface] STM32 driver fault: status=0x%02X", event.detail);
        triggerEmergencyStop("STM32 driver fault");
      }
      break;
    case CanEventType::NONE: break;
  }
}

void CanInterfaceNode::parseSafetyEvent(const can_msgs::Frame& frame) {
  CanEvent event;
  if (decodeCanEvent(frame.id, frame.is_extended, frame.dlc, frame.data.data(), &event)) handleCanEvent(event);
}

void CanInterfaceNode::parseStatusFrame(const can_msgs::Frame& frame) {
  if (frame.is_rtr || frame.is_error || frame.dlc != 8) {
    return;
  }

  if (rx_can_id_filter_enable_) {
    if (frame.is_extended != use_extended_frame_) return;
    const uint32_t rx_id = frame.is_extended ? (frame.id & CAN_EFF_MASK) : (frame.id & CAN_SFF_MASK);
    const uint32_t cfg_id = use_extended_frame_ ? (rx_can_id_ & CAN_EFF_MASK) : (rx_can_id_ & CAN_SFF_MASK);
    if (rx_id != cfg_id) {
      return;
    }
  }

  Stm32Telemetry telemetry;
  if (!decodeStm32Telemetry(frame.dlc, frame.data.data(), &telemetry) ||
      telemetry.index >= 5) return;

  // These are STM32 cache reports, not evidence of a fresh CAN2 sample.
  // Keep their fault indication as a second stop path if the 0x06 event is lost.
  if (telemetry.type == 0x01 && ((telemetry.flags & 8) || frame.data[6]))
    triggerEmergencyStop("STM32 cached status reports driver fault");
}

void CanInterfaceNode::parseDriverFrame(const can_frame& frame) {
  DriverTelemetry reply;
  if (!decodeDriverTelemetry(frame, &reply)) return;
  const int logical = wheel_map_.logicalIndex(reply.address - 1);
  if (logical < 0) return;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    driver_health_.observe(logical, reply, ros::SteadyTime::now().toSec());
    if (reply.function == 0x35)
      motor_state_rpm_[logical] = reply.rpm * wheel_map_.signs[logical];
    else motor_status_flags_[logical] = reply.flags;
  }
  if (reply.function == 0x3A && (!(reply.flags & 1) || (reply.flags & 8)))
    triggerEmergencyStop("CAN2 driver disabled or stall protection active");
  if (reply.function == 0x35) publishTelemetry();
}

void CanInterfaceNode::parseDiagnostics(const can_frame& frame) {
  const double now = ros::SteadyTime::now().toSec();
  const uint32_t id = frame.can_id & CAN_EFF_MASK;
  if ((frame.can_id & CAN_EFF_FLAG) && !(id & 0xFF) && id >= 0x100 && id <= 0x400 &&
      frame.can_dlc == 3 && frame.data[0] == 0xF6 && frame.data[2] == 0x6B &&
      (frame.data[1] == 0x02 || frame.data[1] == 0x9F)) {
    const int logical = wheel_map_.logicalIndex(static_cast<int>(id >> 8) - 1);
    if (logical >= 0) {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      motion_ack_health_.acknowledge(logical, now);
    }
  }
  if ((frame.can_id & CAN_EFF_FLAG) && !(id & 0xFF) && id >= 0x100 && id <= 0x500 &&
      frame.can_dlc == 3 && frame.data[2] == 0x6B &&
      (frame.data[1] == 0xE2 || frame.data[1] == 0xEE))
    triggerEmergencyStop("CAN2 driver rejected a command/read");
  if (frame.can_id == 0x102 && frame.can_dlc == 8) {
    if (frame.data[3] != 0) {
      ROS_ERROR_THROTTLE(1.0, "STM32 rejected command 0x%02X (result=%u)", frame.data[2], frame.data[3]);
      triggerEmergencyStop("STM32 negative command ACK");
    } else if (frame.data[2] == 0x09) heartbeat_ack_time_sec_.store(now);
    else if (frame.data[2] == 0x07) report_configured_.store(true);
  }
  if (frame.can_id == 0x103 && frame.can_dlc == 8 && frame.data[0] >= 1 && frame.data[0] <= 3) {
    bool failure, response_timeout;
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      failure = firmware_stats_.observe(frame);
      response_timeout = firmware_stats_.responseTimeoutChanged();
      stats_time_sec_[frame.data[0] - 1] = now;
    }
    if (response_timeout)
      ROS_WARN_THROTTLE(2.0, "STM32 aggregate response timeouts changed (includes optional motor 5 reads); "
                            "chassis protection uses per-wheel real feedback and motion ACK deadlines");
    if (failure) triggerEmergencyStop("STM32 queue/execution/transport failure counter changed");
  }
}

bool CanInterfaceNode::linkHealthy() {
  const double now = ros::SteadyTime::now().toSec();
  if (!report_configured_.load() || heartbeat_ack_time_sec_.load() == 0 ||
      now - heartbeat_ack_time_sec_.load() > heartbeat_timeout_sec_) return false;
  std::lock_guard<std::mutex> lock(telemetry_mutex_);
  if (!firmware_stats_.ready() || !driver_health_.healthy(now, heartbeat_timeout_sec_)) return false;
  for (double time : stats_time_sec_)
    if (time == 0 || now - time > heartbeat_timeout_sec_) return false;
  link_was_healthy_.store(true);
  return true;
}

void CanInterfaceNode::publishTelemetry() {
  std::vector<float> rpm_snapshot;
  std::vector<uint8_t> flags_snapshot;
  bool rpm_fresh;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    rpm_snapshot = motor_state_rpm_;
    flags_snapshot = motor_status_flags_;
    rpm_fresh = driver_health_.fresh(ros::SteadyTime::now().toSec(), heartbeat_timeout_sec_);
  }

  std_msgs::Float32MultiArray rpm;
  rpm.data = rpm_snapshot;
  if (rpm_fresh) motor_state_pub_.publish(rpm);

  std_msgs::UInt8MultiArray mflag;
  mflag.data = flags_snapshot;
  motor_status_flag_pub_.publish(mflag);
}

void CanInterfaceNode::canReceiveThread() {
  while (running_.load() && ros::ok()) {
    struct can_frame raw_frame;
    int nbytes = -1;
    {
      // Protect fd lifetime across RX, TX-error close, and timer reopen. An old
      // blocking read could otherwise outlive close and miss all future frames.
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (socket_fd_ >= 0) {
        nbytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));
        if (nbytes < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
          ROS_ERROR_THROTTLE(1.0, "CAN read error on %s: %s", can_device_.c_str(), std::strerror(errno));
          close(socket_fd_);
          socket_fd_ = -1;
        }
      }
    }
    if (nbytes != static_cast<int>(sizeof(raw_frame))) {
      ros::WallDuration(0.005).sleep();
      continue;
    }
    if (raw_frame.can_dlc > 8 || (raw_frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG))) continue;

    const ros::Time frame_time = ros::Time::now();

    can_msgs::Frame frame_msg;
    frame_msg.header.stamp = frame_time;
    frame_msg.id = (raw_frame.can_id & CAN_EFF_FLAG) ? (raw_frame.can_id & CAN_EFF_MASK)
                             : (raw_frame.can_id & CAN_SFF_MASK);
    frame_msg.is_rtr = (raw_frame.can_id & CAN_RTR_FLAG) != 0;
    frame_msg.is_extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0;
    frame_msg.is_error = (raw_frame.can_id & CAN_ERR_FLAG) != 0;
    CanEvent event;
    const bool is_safety_event = decodeCanEvent(frame_msg.id, frame_msg.is_extended,
        raw_frame.can_dlc, raw_frame.data, &event);
    const bool is_ack_or_stats = !frame_msg.is_extended && raw_frame.can_dlc == 8 &&
        (frame_msg.id == 0x102 || frame_msg.id == 0x103);
    if (matchesCanReply(raw_frame.can_id, rx_can_id_, use_extended_frame_) || is_safety_event || is_ack_or_stats) {
      last_rx_time_sec_.store(ros::SteadyTime::now().toSec());
    }
    ROS_INFO_ONCE("First CAN RX on %s: ID=0x%X %s DLC=%u; configured STM32 RX=0x%X %s.",
                  can_device_.c_str(), frame_msg.id, frame_msg.is_extended ? "extended" : "standard",
                  raw_frame.can_dlc, rx_can_id_, use_extended_frame_ ? "extended" : "standard");
    frame_msg.dlc = raw_frame.can_dlc;
    std::copy(raw_frame.data, raw_frame.data + 8, frame_msg.data.begin());
    can_rx_pub_.publish(frame_msg);

    parseSafetyEvent(frame_msg);
    parseStatusFrame(frame_msg);
    parseDriverFrame(raw_frame);
    parseDiagnostics(raw_frame);
  }
}

void CanInterfaceNode::probeTimerCallback(const ros::WallTimerEvent&) {
  if (!socketAvailable()) return;
  // Interleave speed/status: each of four motors is checked every 400 ms.
  const size_t wheel = (probe_slot_ / 2) % 4;
  const auto frame = driverRead(static_cast<uint8_t>(wheel_map_.indices[wheel] + 1),
                               probe_slot_ % 2 ? 0x3A : 0x35);
  probe_slot_ = (probe_slot_ + 1) % 8;
  if (!sendCanFrame(frame.can_id, frame.data, frame.can_dlc))
    triggerEmergencyStop("CAN2 feedback probe transmit failure");
}

void CanInterfaceNode::timerCallback(const ros::WallTimerEvent&) {
  std_msgs::Bool ready;
  ready.data = linkHealthy() && !motion_.latched();
  link_ready_pub_.publish(ready);
  if (!socketAvailable()) {
    if (!motion_.latched()) triggerEmergencyStop("CAN socket unavailable");
    openSocket();
    return;
  }
  if (!sendControlCommand(0x09)) {
    triggerEmergencyStop("CAN heartbeat transmit failure");
    return;
  }
  const double now = ros::SteadyTime::now().toSec();
  // Firmware boots with basic+position only (mask=3), so velocity feedback must
  // be requested explicitly. Retry until ACK and reconfigure after reconnect.
  if (!report_configured_.load() && now - last_report_request_sec_ >= 1.0) {
    last_report_request_sec_ = now;
    sendControlCommand(0x07, report_mask_);
  }
  if (now - last_rx_time_sec_.load() > heartbeat_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "CAN heartbeat timeout: no STM32 reply on %s ID=0x%X %s for %.2f s. "
                       "Check firmware protocol, bitrate and CAN1 filtering.", can_device_.c_str(),
                       rx_can_id_, use_extended_frame_ ? "extended" : "standard", heartbeat_timeout_sec_);
    if (!isDuplicateEvent(0x22, ros::SteadyTime::now())) triggerEmergencyStop("CAN receive timeout");
  }
  if ((link_was_healthy_.load() || now - link_open_time_sec_.load() > 2.0) && !linkHealthy() && !motion_.latched())
    triggerEmergencyStop("CAN2 motor feedback, heartbeat ACK or firmware statistics expired");
  bool motion_ack_expired;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    motion_ack_expired = !motion_ack_health_.healthy(now, 0.5);
  }
  if (motion_ack_expired && !motion_.latched())
    triggerEmergencyStop("CAN2 per-wheel F6 motion ACK timeout");
  // Retry the stop after a lost write/reconnect; never emit F6 while latched.
  if (motion_.latched()) motion_.stop([&] { sendControlCommand(0x03); });
}

}  // namespace can_motor_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    can_motor_interface::CanInterfaceNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("CAN interface configuration/startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
