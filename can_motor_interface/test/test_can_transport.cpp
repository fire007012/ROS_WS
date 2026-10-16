#include "can_motor_interface/can_interface.h"

#include <gtest/gtest.h>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <deque>
#include <memory>
#include <set>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
struct Attempt { can_frame frame; int error; };
struct FakeCan {
  std::mutex mutex;
  std::set<int> descriptors;
  std::set<int> active;
  int next_fd = 100000;
  int opens = 0;
  int closes = 0;
  int default_error = 0;
  bool device_present = true;
  can_err_mask_t error_mask = 0;
  std::deque<int> write_errors;
  std::deque<can_frame> received;
  std::vector<Attempt> writes;
} bus;
}  // namespace

// Wrap PF_CAN only; ROS's TCP/XMLRPC sockets continue to use real syscalls.
extern "C" {
int __real_socket(int, int, int);
int __real_ioctl(int, unsigned long, ...);
int __real_bind(int, const sockaddr*, socklen_t);
int __real_setsockopt(int, int, int, const void*, socklen_t);
ssize_t __real_read(int, void*, size_t);
ssize_t __real_write(int, const void*, size_t);
int __real_close(int);

int __wrap_socket(int domain, int type, int protocol) {
  if (domain != PF_CAN) return __real_socket(domain, type, protocol);
  std::lock_guard<std::mutex> lock(bus.mutex);
  const int fd = bus.next_fd++;
  bus.descriptors.insert(fd); bus.active.insert(fd); ++bus.opens;
  return fd;
}
int __wrap_ioctl(int fd, unsigned long request, ...) {
  va_list args; va_start(args, request); void* value = va_arg(args, void*); va_end(args);
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_ioctl(fd, request, value);
  if (!bus.device_present) { errno = ENODEV; return -1; }
  auto* ifr = static_cast<ifreq*>(value);
  if (request == SIOCGIFINDEX) ifr->ifr_ifindex = 7;
  else if (request == SIOCGIFFLAGS) ifr->ifr_flags = IFF_UP;
  else { errno = EINVAL; return -1; }
  return 0;
}
int __wrap_bind(int fd, const sockaddr* address, socklen_t length) {
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_bind(fd, address, length);
  return 0;
}
int __wrap_setsockopt(int fd, int level, int option, const void* value, socklen_t length) {
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_setsockopt(fd, level, option, value, length);
  if (level == SOL_CAN_RAW && option == CAN_RAW_ERR_FILTER && length == sizeof(can_err_mask_t))
    std::memcpy(&bus.error_mask, value, length);
  return 0;
}
ssize_t __wrap_read(int fd, void* data, size_t size) {
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_read(fd, data, size);
  if (!bus.active.count(fd)) { errno = EBADF; return -1; }
  if (bus.received.empty()) { errno = EAGAIN; return -1; }
  if (size < sizeof(can_frame)) { errno = EINVAL; return -1; }
  std::memcpy(data, &bus.received.front(), sizeof(can_frame)); bus.received.pop_front();
  return sizeof(can_frame);
}
ssize_t __wrap_write(int fd, const void* data, size_t size) {
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_write(fd, data, size);
  if (!bus.active.count(fd)) { errno = EBADF; return -1; }
  if (size != sizeof(can_frame)) { errno = EINVAL; return -1; }
  const int error = bus.write_errors.empty() ? bus.default_error : bus.write_errors.front();
  if (!bus.write_errors.empty()) bus.write_errors.pop_front();
  Attempt attempt{}; std::memcpy(&attempt.frame, data, size); attempt.error = error;
  bus.writes.push_back(attempt);
  if (error) { errno = error; return -1; }
  return size;
}
int __wrap_close(int fd) {
  std::lock_guard<std::mutex> lock(bus.mutex);
  if (!bus.descriptors.count(fd)) return __real_close(fd);
  bus.active.erase(fd); ++bus.closes; return 0;
}
}  // extern "C"

namespace can_motor_interface {
class CanInterfaceNodeTestPeer {
 public:
  static void stopWorker(CanInterfaceNode& node) {
    node.monitor_timer_.stop(); node.probe_timer_.stop(); node.command_timer_.stop();
    node.running_.store(false);
    if (node.rx_thread_.joinable()) node.rx_thread_.join();
  }
  static bool send(CanInterfaceNode& node) {
    const auto frame = driverRead(1, 0x35);
    return node.sendCanFrame(frame.can_id, frame.data, frame.can_dlc);
  }
  static void tick(CanInterfaceNode& node) { node.timerCallback(ros::WallTimerEvent{}); }
  static void probe(CanInterfaceNode& node) { node.probeTimerCallback(ros::WallTimerEvent{}); }
  static void stop(CanInterfaceNode& node) { node.triggerEmergencyStop("injected fault"); }
  static bool latched(CanInterfaceNode& node) { return node.motion_.latched(); }
  static bool pending(CanInterfaceNode& node) { return node.stop_pending_.load(); }
  static int fd(CanInterfaceNode& node) { return node.socket_fd_; }
  static void stopDue(CanInterfaceNode& node) { node.last_stop_attempt_sec_ = 0; }
  static void openDue(CanInterfaceNode& node) { node.next_open_attempt_sec_ = 0; }
  static void probeDue(CanInterfaceNode& node) { node.last_probe_attempt_sec_ = 0; }
  static double delay(CanInterfaceNode& node) { return node.reconnect_delay_sec_; }
  static uint64_t attempts(CanInterfaceNode& node) { return node.socket_open_attempts_.load(); }
  static uint64_t pressure(CanInterfaceNode& node) { return node.tx_backpressure_count_.load(); }
  static double heartbeat(CanInterfaceNode& node) { return node.heartbeat_ack_time_sec_.load(); }
  static bool statsReady(CanInterfaceNode& node) { return node.firmware_stats_.ready(); }
  static double lastRx(CanInterfaceNode& node) { return node.last_rx_time_sec_.load(); }
  static bool controllerFault(CanInterfaceNode& node) { return node.controller_fault_.load(); }
  static std::array<uint64_t, 5> errorCounts(CanInterfaceNode& node) {
    return {{node.can_error_count_.load(), node.can_crc_error_count_.load(),
             node.can_format_error_count_.load(), node.can_stuff_error_count_.load(),
             node.can_bit_error_count_.load()}};
  }
  static uint8_t protocolError(CanInterfaceNode& node) { return node.last_protocol_error_.load(); }
  static void receive(CanInterfaceNode& node, const std::vector<can_frame>& frames) {
    {
      std::lock_guard<std::mutex> lock(bus.mutex);
      bus.received.insert(bus.received.end(), frames.begin(), frames.end());
    }
    node.running_.store(true);
    std::thread worker([&] { node.canReceiveThread(); });
    const auto deadline = ros::WallTime::now() + ros::WallDuration(1.0);
    while (ros::WallTime::now() < deadline) {
      { std::lock_guard<std::mutex> lock(bus.mutex); if (bus.received.empty()) break; }
      ros::WallDuration(0.001).sleep();
    }
    node.running_.store(false); worker.join();
    std::lock_guard<std::mutex> lock(bus.mutex);
    ASSERT_TRUE(bus.received.empty());
  }
  static void freshHealth(CanInterfaceNode& node) {
    const double now = ros::SteadyTime::now().toSec();
    node.last_failure_time_sec_.store(now - 2.0);
    node.last_tx_failure_sec_.store(now - 2.0);
    node.tx_congested_.store(false);
    node.report_configured_.store(true);
    node.heartbeat_ack_time_sec_.store(now);
    node.last_rx_time_sec_.store(now);
    for (size_t i = 0; i < 4; ++i) {
      DriverTelemetry sample; sample.function = 0x35;
      node.driver_health_.observe(i, sample, now);
      sample.function = 0x3A; sample.flags = 1;
      node.driver_health_.observe(i, sample, now);
    }
    for (uint8_t type = 1; type <= 3; ++type) {
      can_frame stats{}; stats.can_id = 0x103; stats.can_dlc = 8; stats.data[0] = type;
      node.firmware_stats_.observe(stats); node.stats_time_sec_[type - 1] = now;
    }
  }
  static void command(CanInterfaceNode& node) {
    std_msgs::Float32MultiArray::Ptr command(new std_msgs::Float32MultiArray);
    command->data = {10, 10, 10, 10}; node.cmdCallback(command);
    node.commandTimerCallback(ros::WallTimerEvent{});
  }
  static void commandTick(CanInterfaceNode& node) { node.commandTimerCallback(ros::WallTimerEvent{}); }
  static void running(CanInterfaceNode& node) {
    node.start_state_.physicalStart(0, 123, ros::SteadyTime::now().toSec());
    node.start_state_.acknowledge(123, 0x05, 0);
    node.start_state_.motionStarted();
  }
  static void target(CanInterfaceNode& node, float rpm) {
    std_msgs::Float32MultiArray::Ptr msg(new std_msgs::Float32MultiArray);
    msg->data.assign(4, rpm); node.cmdCallback(msg);
  }
  static void expireCommand(CanInterfaceNode& node) {
    node.last_command_time_ = ros::SteadyTime::now() - ros::WallDuration(1.0);
  }
  static void refreshDue(CanInterfaceNode& node) {
    for (auto& stamp : node.last_speed_sent_sec_) stamp -= 0.3;
  }
  static void missingMotionAck(CanInterfaceNode& node) {
    node.motion_ack_health_ = MotionAckHealth{};
    node.motion_ack_health_.sent(0, ros::SteadyTime::now().toSec() - 0.6);
  }
  static void cacheDue(CanInterfaceNode& node) { node.last_cached_batch_sec_ = 0; }
  static void reset(CanInterfaceNode& node) {
    std_msgs::Bool::Ptr msg(new std_msgs::Bool); msg->data = true; node.emergencyResetCallback(msg);
  }
  static bool haveCommand(CanInterfaceNode& node) { return node.have_command_; }
  static std::string healthIssue(CanInterfaceNode& node) { return node.linkHealthIssue(); }
  static void ageSample(CanInterfaceNode& node, size_t wheel, uint8_t function, double age = 0.6) {
    auto& stamp = function == 0x35 ? node.driver_health_.speed_time[wheel]
                                  : node.driver_health_.status_time[wheel];
    stamp = ros::SteadyTime::now().toSec() - age;
  }
};
}  // namespace can_motor_interface

namespace {
using Peer = can_motor_interface::CanInterfaceNodeTestPeer;
class CanTransport : public ::testing::Test {
 protected:
  ros::NodeHandle nh;
  ros::NodeHandle pnh{"~"};
  std::unique_ptr<can_motor_interface::CanInterfaceNode> node;
  void SetUp() override {
    {
      std::lock_guard<std::mutex> lock(bus.mutex);
      bus.active.clear(); bus.writes.clear(); bus.write_errors.clear(); bus.received.clear();
      bus.opens = bus.closes = bus.default_error = 0; bus.device_present = true; bus.error_mask = 0;
    }
    node.reset(new can_motor_interface::CanInterfaceNode(nh, pnh));
    Peer::stopWorker(*node);
    ASSERT_GE(Peer::fd(*node), 0);
  }
  void TearDown() override { node.reset(); }
  void clearWrites() { bus.writes.clear(); bus.write_errors.clear(); }
};

TEST_F(CanTransport, BackpressurePreservesSocketAndReceivesRealHeartbeat) {
  Peer::freshHealth(*node);
  Peer::ageSample(*node, 0, 0x35);
  const int fd = Peer::fd(*node);
  bus.default_error = ENOBUFS;
  Peer::probe(*node);
  EXPECT_EQ(fd, Peer::fd(*node)); EXPECT_EQ(0, bus.closes);
  EXPECT_TRUE(Peer::latched(*node)); EXPECT_TRUE(Peer::pending(*node));
  EXPECT_GE(Peer::pressure(*node), 2u);
  EXPECT_TRUE(Peer::statsReady(*node));
  can_frame ack{}; ack.can_id = 0x102; ack.can_dlc = 8; ack.data[2] = 0x09;
  Peer::receive(*node, {ack});
  EXPECT_GT(Peer::heartbeat(*node), 0);
  for (int i = 0; i < 20; ++i) { Peer::tick(*node); Peer::probe(*node); }
  EXPECT_EQ(1, bus.opens); EXPECT_EQ(0, bus.closes);
  clearWrites(); Peer::stopDue(*node); Peer::tick(*node);
  ASSERT_GE(bus.writes.size(), 2u);
  EXPECT_EQ(0x03, bus.writes[0].frame.data[0]);
  EXPECT_EQ(0x09, bus.writes[1].frame.data[0]);
}

TEST_F(CanTransport, EagainAndInterruptedWritesDoNotReopenSocket) {
  bus.write_errors = {EAGAIN}; EXPECT_FALSE(Peer::send(*node));
  EXPECT_EQ(0, bus.closes);
  bus.write_errors = {EINTR, 0}; EXPECT_TRUE(Peer::send(*node));
  EXPECT_EQ(3u, bus.writes.size()); EXPECT_EQ(0, bus.closes);
  bus.write_errors = {EINTR, EINTR}; EXPECT_FALSE(Peer::send(*node));
  EXPECT_EQ(5u, bus.writes.size()); EXPECT_EQ(0, bus.closes);
}

TEST_F(CanTransport, StopIsAttemptedBeforeFailingHeartbeatAndSuppressesQueries) {
  Peer::stop(*node); clearWrites(); Peer::stopDue(*node);
  bus.write_errors = {0, ENOBUFS};
  Peer::tick(*node);
  ASSERT_GE(bus.writes.size(), 2u);
  EXPECT_EQ(0x03, bus.writes[0].frame.data[0]);
  EXPECT_EQ(0x09, bus.writes[1].frame.data[0]);
  const auto count = bus.writes.size(); Peer::probe(*node);
  EXPECT_EQ(count, bus.writes.size()); EXPECT_EQ(0, bus.closes);
}

TEST_F(CanTransport, FirstHeartbeatFailureStillAttemptsStopInSameTick) {
  bus.write_errors = {ENOBUFS, 0};
  Peer::tick(*node);
  ASSERT_GE(bus.writes.size(), 2u);
  EXPECT_EQ(0x09, bus.writes[0].frame.data[0]);
  EXPECT_EQ(0x03, bus.writes[1].frame.data[0]);
  EXPECT_TRUE(Peer::latched(*node));
}

TEST_F(CanTransport, DeviceLossBacksOffAndReconnectSubmitsStopFirst) {
  bus.write_errors = {ENODEV}; EXPECT_FALSE(Peer::send(*node));
  EXPECT_LT(Peer::fd(*node), 0); EXPECT_EQ(1, bus.closes);
  for (int i = 0; i < 20; ++i) Peer::tick(*node);
  EXPECT_EQ(1u, Peer::attempts(*node));
  Peer::openDue(*node); Peer::stopDue(*node); clearWrites();
  Peer::tick(*node);
  ASSERT_GE(bus.writes.size(), 2u);
  EXPECT_EQ(0x03, bus.writes[0].frame.data[0]);
  EXPECT_EQ(0x09, bus.writes[1].frame.data[0]);
  EXPECT_EQ(2u, Peer::attempts(*node)); EXPECT_TRUE(Peer::latched(*node));
}

TEST_F(CanTransport, FirstDisconnectedTickDoesNotConsumeStopRetryBeforeReopen) {
  bus.write_errors = {ENODEV}; EXPECT_FALSE(Peer::send(*node));
  EXPECT_FALSE(Peer::latched(*node));
  Peer::openDue(*node); clearWrites(); Peer::tick(*node);
  ASSERT_GE(bus.writes.size(), 2u);
  EXPECT_EQ(0x03, bus.writes[0].frame.data[0]);
  EXPECT_EQ(0x09, bus.writes[1].frame.data[0]);
  EXPECT_TRUE(Peer::latched(*node));
}

TEST_F(CanTransport, RepeatedMissingDeviceAttemptsAreBoundedAndBackoffIsCapped) {
  bus.write_errors = {ENODEV}; EXPECT_FALSE(Peer::send(*node)); bus.device_present = false;
  for (int i = 0; i < 8; ++i) {
    const auto attempts = Peer::attempts(*node);
    for (int j = 0; j < 10; ++j) Peer::tick(*node);
    EXPECT_EQ(attempts, Peer::attempts(*node));
    Peer::openDue(*node); Peer::tick(*node);
    EXPECT_EQ(attempts + 1, Peer::attempts(*node));
    EXPECT_LE(Peer::delay(*node), 5.0);
  }
  EXPECT_DOUBLE_EQ(5.0, Peer::delay(*node));
}

TEST_F(CanTransport, PassiveAndProtocolErrorsAreDiagnosedWithoutRefreshingHealth) {
  EXPECT_NE(0u, bus.error_mask & CAN_ERR_PROT); EXPECT_NE(0u, bus.error_mask & CAN_ERR_BUSOFF);
  const double last_rx = Peer::lastRx(*node);
  can_frame error{}; error.can_id = CAN_ERR_FLAG | CAN_ERR_CRTL | CAN_ERR_PROT;
  error.can_dlc = CAN_ERR_DLC; error.data[1] = CAN_ERR_CRTL_RX_PASSIVE;
  error.data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
  can_frame format = error; format.data[3] = 0; format.data[2] = CAN_ERR_PROT_FORM;
  can_frame stuff = format; stuff.data[2] = CAN_ERR_PROT_STUFF;
  can_frame bit = format; bit.data[1] = 0; bit.data[2] = CAN_ERR_PROT_BIT1;
  Peer::receive(*node, {error, format, stuff, bit});
  EXPECT_EQ((std::array<uint64_t, 5>{{4, 1, 1, 1, 1}}), Peer::errorCounts(*node));
  EXPECT_DOUBLE_EQ(last_rx, Peer::lastRx(*node)); EXPECT_DOUBLE_EQ(0, Peer::heartbeat(*node));
  EXPECT_TRUE(Peer::controllerFault(*node)); EXPECT_TRUE(Peer::latched(*node));
  Peer::freshHealth(*node); Peer::reset(*node); EXPECT_TRUE(Peer::latched(*node));
  can_frame recovery{}; recovery.can_id = CAN_ERR_FLAG | CAN_ERR_RESTARTED; recovery.can_dlc = CAN_ERR_DLC;
  Peer::receive(*node, {recovery});
  EXPECT_FALSE(Peer::controllerFault(*node)); EXPECT_TRUE(Peer::latched(*node));
  EXPECT_EQ(CAN_ERR_PROT_BIT1, Peer::protocolError(*node));
}

TEST_F(CanTransport, BusOffNeedsRecoveryAndDoesNotTriggerSocketChurn) {
  can_frame error{}; error.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF; error.can_dlc = CAN_ERR_DLC;
  Peer::receive(*node, {error});
  EXPECT_TRUE(Peer::controllerFault(*node)); EXPECT_TRUE(Peer::latched(*node));
  clearWrites(); Peer::probe(*node); EXPECT_TRUE(bus.writes.empty());
  EXPECT_EQ(1, bus.opens); EXPECT_EQ(0, bus.closes);
}

TEST_F(CanTransport, LatchedQueriesAreReducedAndRecoveryRequiresResetAndNewCommand) {
  Peer::stop(*node); clearWrites(); Peer::probeDue(*node);
  Peer::probe(*node); Peer::probe(*node); ASSERT_EQ(1u, bus.writes.size());
  Peer::probeDue(*node); Peer::probe(*node); ASSERT_EQ(2u, bus.writes.size());
  Peer::freshHealth(*node); clearWrites(); Peer::command(*node);
  EXPECT_TRUE(bus.writes.empty()); EXPECT_TRUE(Peer::latched(*node));
  Peer::reset(*node); EXPECT_FALSE(Peer::latched(*node)); EXPECT_FALSE(Peer::haveCommand(*node));
  Peer::commandTick(*node); EXPECT_TRUE(bus.writes.empty());
}

TEST_F(CanTransport, FreshNativeRepliesSuppressRedundantCan1Requests) {
  Peer::freshHealth(*node);
  clearWrites();
  for (int i = 0; i < 16; ++i) Peer::probe(*node);
  EXPECT_TRUE(bus.writes.empty());
  // One aging status must be queried directly, without resending seven fresh reads.
  Peer::ageSample(*node, 2, 0x3A);
  Peer::probe(*node);
  ASSERT_EQ(1u, bus.writes.size());
  EXPECT_EQ(CAN_EFF_FLAG | 0x400u, bus.writes[0].frame.can_id);
  EXPECT_EQ(0x3A, bus.writes[0].frame.data[0]);
  can_frame reply{}; reply.can_id = CAN_EFF_FLAG | 0x400u; reply.can_dlc = 3;
  reply.data[0] = 0x3A; reply.data[1] = 0x83; reply.data[2] = 0x6B;
  Peer::receive(*node, {reply});
  clearWrites(); Peer::probe(*node);
  EXPECT_TRUE(bus.writes.empty());
}

TEST_F(CanTransport, CachedReportsCannotSuppressMissingNativeProbes) {
  std::vector<can_frame> reports;
  for (uint8_t wheel = 0; wheel < 4; ++wheel) {
    for (uint8_t type : {uint8_t(1), uint8_t(3)}) {
      can_frame report{}; report.can_id = 0x101; report.can_dlc = 8;
      report.data[0] = type; report.data[1] = wheel;
      report.data[2] = type == 1 ? 0x83 : 0;
      reports.push_back(report);
    }
  }
  Peer::receive(*node, reports);
  clearWrites();
  const std::array<uint32_t, 4> addresses{{2, 1, 4, 3}};
  for (size_t i = 0; i < 8; ++i) {
    Peer::probe(*node);
    ASSERT_EQ(i + 1, bus.writes.size());
    EXPECT_EQ(CAN_EFF_FLAG | (addresses[i / 2] << 8), bus.writes[i].frame.can_id);
    EXPECT_EQ(i % 2 ? 0x3A : 0x35, bus.writes[i].frame.data[0]);
  }
}

TEST_F(CanTransport, HealthDiagnosisIdentifiesMissingNativeSampleAndConfigurationAck) {
  EXPECT_NE(std::string::npos, Peer::healthIssue(*node).find("configuration ACK"));
  Peer::freshHealth(*node);
  EXPECT_TRUE(Peer::healthIssue(*node).empty());
  // Still fresh enough to be safe, but old enough to request a new sample.
  Peer::ageSample(*node, 3, 0x35);
  EXPECT_TRUE(Peer::healthIssue(*node).empty());
  Peer::ageSample(*node, 3, 0x35, 1.1);
  EXPECT_EQ("logical wheel 3 / CAN2 address 3: native speed missing or expired", Peer::healthIssue(*node));
}

TEST_F(CanTransport, RunningTargetsAreStaggeredAndRepeatedWireValuesAreCoalesced) {
  Peer::freshHealth(*node); Peer::running(*node); Peer::target(*node, 10.1f); clearWrites();
  const uint8_t indices[] = {1, 0, 3, 2};
  for (size_t i = 0; i < 4; ++i) {
    Peer::commandTick(*node);
    ASSERT_EQ(i + 1, bus.writes.size());
    EXPECT_EQ(indices[i], bus.writes.back().frame.data[1]);
    EXPECT_EQ(0, bus.writes.back().frame.data[7]);
    EXPECT_EQ(10, bus.writes.back().frame.data[2]);
  }
  Peer::target(*node, 10.4f);  // Same integer RPM, new ROS message.
  for (int i = 0; i < 4; ++i) Peer::commandTick(*node);
  EXPECT_EQ(4u, bus.writes.size());
  Peer::refreshDue(*node);
  for (int i = 0; i < 4; ++i) Peer::commandTick(*node);
  EXPECT_EQ(8u, bus.writes.size());
}

TEST_F(CanTransport, SlotsUseNewestCommandAndExpiredCommandBecomesZero) {
  Peer::freshHealth(*node); Peer::running(*node); Peer::target(*node, 10); clearWrites();
  Peer::commandTick(*node);
  Peer::target(*node, 12); Peer::target(*node, 15);
  Peer::commandTick(*node);
  ASSERT_EQ(2u, bus.writes.size());
  EXPECT_EQ(15, bus.writes.back().frame.data[2]);
  Peer::expireCommand(*node);
  for (int i = 0; i < 4; ++i) Peer::commandTick(*node);
  ASSERT_EQ(6u, bus.writes.size());
  for (size_t i = 2; i < 6; ++i) {
    EXPECT_EQ(0, bus.writes[i].frame.data[2]);
    EXPECT_EQ(0, bus.writes[i].frame.data[3]);
  }
  Peer::stop(*node); clearWrites(); Peer::commandTick(*node);
  EXPECT_TRUE(bus.writes.empty());
}

TEST_F(CanTransport, CoalescingPreservesMissingMotionAckStop) {
  Peer::freshHealth(*node); Peer::running(*node); Peer::target(*node, 10);
  Peer::commandTick(*node); Peer::missingMotionAck(*node);
  Peer::tick(*node);
  EXPECT_TRUE(Peer::latched(*node));
}

TEST_F(CanTransport, PrestartCacheDoesNotRepeatOnNewIdenticalMessagesOrFasterTicks) {
  Peer::freshHealth(*node); Peer::target(*node, 10); clearWrites();
  Peer::commandTick(*node); ASSERT_EQ(4u, bus.writes.size());
  for (const auto& attempt : bus.writes) EXPECT_EQ(1, attempt.frame.data[7]);
  Peer::target(*node, 10.4f); Peer::commandTick(*node);
  EXPECT_EQ(4u, bus.writes.size());
  Peer::target(*node, 15); Peer::commandTick(*node);
  EXPECT_EQ(4u, bus.writes.size());
  Peer::cacheDue(*node); Peer::commandTick(*node);
  ASSERT_EQ(8u, bus.writes.size());
  EXPECT_EQ(15, bus.writes.back().frame.data[2]);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "can_transport_fault_test");
  return RUN_ALL_TESTS();
}
