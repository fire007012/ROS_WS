#include "arm_interface/arm_controller.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace arm_interface {
using can_motor_interface::driverRead;
using can_motor_interface::stm32Command;

ArmControllerNode::ArmControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("can_device", can_device_, "can0");
  pnh_.param<std::string>("joint_name", joint_name_, "joint_1");
  pnh_.param("speed_rpm", speed_rpm_, 100);
  if (speed_rpm_ < 1 || speed_rpm_ > 3000) throw std::invalid_argument("arm speed_rpm must be 1..3000");
  motion_.stop([] {});
  joint_cmd_sub_ = nh_.subscribe("/arm_joint_cmd", 2, &ArmControllerNode::armCmdCallback, this);
  can_rx_sub_ = nh_.subscribe("/can_rx", 100, &ArmControllerNode::canRxCallback, this);
  ready_sub_ = nh_.subscribe("/motor_link_ready", 2, &ArmControllerNode::readyCallback, this);
  estop_sub_ = nh_.subscribe("/emergency_stop", 2, &ArmControllerNode::estopCallback, this);
  arm_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm_state", 10);
  estop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 2, true);
  timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &ArmControllerNode::timerCallback, this);
  ROS_INFO("Arm uses STM32 standard 0x100, index=4/address=5, one absolute joint in radians");
}

ArmControllerNode::~ArmControllerNode() { closeSocket(); }

bool ArmControllerNode::openSocket() {
  if (socket_fd_ >= 0) return true;
  int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) return false;
  struct ifreq ifr{};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return false; }
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) { close(fd); return false; }
  socket_fd_ = fd;
  return true;
}

void ArmControllerNode::closeSocket() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ >= 0) close(socket_fd_);
  socket_fd_ = -1;
}

bool ArmControllerNode::send(const can_frame& frame) {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (!openSocket()) return false;
  if (write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) return true;
  ROS_ERROR_THROTTLE(1.0, "Arm CAN TX failed: %s", std::strerror(errno));
  close(socket_fd_);
  socket_fd_ = -1;
  return false;
}

bool ArmControllerNode::ready() const {
  const double now = ros::SteadyTime::now().toSec();
  return !emergency_active_ && link_ready_ && now - link_ready_time_ <= 0.5 &&
         position_time_ > 0 && now - position_time_ <= 1.0 &&
         status_time_ > 0 && now - status_time_ <= 1.0 && (status_flags_ & 1) && !(status_flags_ & 8);
}

void ArmControllerNode::readyCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data && !motion_.latched()) fail("chassis link became unavailable");
  link_ready_ = msg->data;
  link_ready_time_ = ros::SteadyTime::now().toSec();
  if (!link_ready_) motion_.stop([] {});
  else motion_.reset([&] { return ready(); });
}

void ArmControllerNode::estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  emergency_active_ = msg->data;
  if (msg->data) {
    link_ready_ = false;
    motion_.stop([] {}); // chassis node owns stop transmission and retry
  }
}

void ArmControllerNode::fail(const char* reason) {
  emergency_active_ = true;
  motion_.stop([&] { send(stm32Command(0x100, false, 3, 0xFF)); });
  std_msgs::Bool stop; stop.data = true; estop_pub_.publish(stop);
  ROS_ERROR("Arm ESTOP: %s", reason);
}

void ArmControllerNode::armCmdCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg->position.size() != 1 || (!msg->name.empty() &&
      (msg->name.size() != 1 || msg->name[0] != joint_name_))) {
    ROS_ERROR("This firmware accepts one arm joint (%s) only; refusing multi-axis command", joint_name_.c_str());
    return;
  }
  if (!ready() || motion_.latched()) { ROS_WARN("Arm command refused: link/arm feedback not ready or ESTOP"); return; }
  try {
    const auto position = can_motor_interface::stm32ArmPosition(msg->position[0]);
    if (!motion_.run([&] {
      return send(stm32Command(0x100, false, 4, 4, speed_rpm_)) && send(position);
    })) fail("arm transmit failure");
  } catch (const std::exception& error) { ROS_ERROR("Arm command refused: %s", error.what()); }
}

void ArmControllerNode::canRxCallback(const can_msgs::Frame::ConstPtr& msg) {
  if (msg->is_extended && !msg->is_rtr && !msg->is_error && msg->id == 0x500 &&
      msg->dlc == 3 && msg->data[0] == 0x3A && msg->data[2] == 0x6B &&
      msg->data[1] != 0xE2 && msg->data[1] != 0xEE) {
    status_flags_ = msg->data[1];
    status_time_ = ros::SteadyTime::now().toSec();
    if ((!(status_flags_ & 1) || (status_flags_ & 8)) && !emergency_active_)
      fail("arm driver disabled or faulted");
    return;
  }
  // Driver 0x36 replies use degrees*10, not the command's pulse units.
  if (!msg->is_extended || msg->is_rtr || msg->is_error || msg->id != 0x500 || msg->dlc != 7 ||
      msg->data[0] != 0x36 || msg->data[1] > 1 || msg->data[6] != 0x6B) return;
  uint32_t raw = (static_cast<uint32_t>(msg->data[2]) << 24) |
      (static_cast<uint32_t>(msg->data[3]) << 16) | (static_cast<uint32_t>(msg->data[4]) << 8) | msg->data[5];
  double radians = raw * (std::acos(-1.0) / 1800.0);
  if (msg->data[1]) radians = -radians;
  position_time_ = ros::SteadyTime::now().toSec();
  sensor_msgs::JointState state;
  state.header.stamp = ros::Time::now();
  state.name = {joint_name_};
  state.position = {radians};
  arm_state_pub_.publish(state);
}

void ArmControllerNode::timerCallback(const ros::WallTimerEvent&) {
  if (!ready()) {
    const bool was_active = !motion_.latched();
    if (was_active) fail("arm feedback or chassis readiness expired");
  }
  if (!send(driverRead(5, query_status_ ? 0x3A : 0x36)) && !motion_.latched())
    fail("arm feedback query transmit failure");
  query_status_ = !query_status_;
}
}  // namespace arm_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_controller_node");
  ros::NodeHandle nh, pnh("~");
  try {
    arm_interface::ArmControllerNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& error) { ROS_FATAL("Arm configuration failed: %s", error.what()); return 1; }
  return 0;
}
