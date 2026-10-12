/**
 * @file start_signal_pub.cpp
 * @brief 启动信号转发节点
 *
 * 通过 CAN 接收 STM32 物理按键事件，并转发到 ROS 话题 /start_signal/physical。
 * 同时也支持通过 ROS 参数或服务手动触发。
 *
 * STM32 协议：标准帧 0x112、DLC=8，byte0=0x01、byte1=0x01、byte2=0x01
 * 表示物理启动键按下；byte3=0 表示固件接受，byte4..5 是本地序号。
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_navigation {

class StartSignalPub {
 public:
  StartSignalPub(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , socket_fd_(-1)
      , running_(false)
      , can_device_("can0")
      , can_id_(0x112)
  {
    pnh_.param<std::string>("can_device", can_device_, "can0");
    int can_id_int = static_cast<int>(can_id_);
    pnh_.param("can_id", can_id_int, can_id_int);
    can_id_ = static_cast<uint32_t>(can_id_int);
  }

  ~StartSignalPub() {
    running_.store(false);
    if (rx_thread_.joinable()) rx_thread_.join();
    if (socket_fd_ >= 0) close(socket_fd_);
  }

  bool init() {
    pub_ = nh_.advertise<std_msgs::UInt32>("/start_signal/physical", 1, false);

    // 初始化 CAN 接收
    if (initCan()) {
      running_.store(true);
      rx_thread_ = std::thread(&StartSignalPub::canReceiveThread, this);
      ROS_INFO("[start_signal_pub] CAN 接收线程已启动: %s, ID=0x%03X",
               can_device_.c_str(), can_id_);
    } else {
      ROS_WARN("[start_signal_pub] CAN 初始化失败，比赛模式不启用 CAN 启动入口");
    }

    ROS_INFO("[start_signal_pub] 初始化完成，等待 STM32 启动信号...");
    return true;
  }

 private:
  bool initCan() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      ROS_ERROR("[start_signal_pub] CAN socket 创建失败: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      ROS_ERROR("[start_signal_pub] ioctl 失败: %s", std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      ROS_ERROR("[start_signal_pub] bind 失败: %s", std::strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    return true;
  }

  void canReceiveThread() {
    struct can_frame frame;
    while (running_.load() && ros::ok()) {
      int nbytes = read(socket_fd_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EAGAIN) continue;
        ROS_ERROR_THROTTLE(5.0, "[start_signal_pub] CAN 读取错误: %s", std::strerror(errno));
        break;
      }
      if (nbytes != static_cast<int>(sizeof(struct can_frame))) continue;

      // 检查是否是启动信号
      const uint32_t raw_id = frame.can_id & CAN_SFF_MASK;
      if (!(frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) &&
          raw_id == can_id_ && frame.can_dlc == 8 &&
          frame.data[0] == 0x01 && frame.data[1] == 0x01 &&
          frame.data[2] == 0x01 && frame.data[3] == 0x00) {
        const uint16_t seq = static_cast<uint16_t>(frame.data[4] << 8 | frame.data[5]);
        ROS_INFO("[start_signal_pub] physical STM32 start accepted, local seq=%u", seq);
        std_msgs::UInt32 msg; msg.data = 0x5A17;
        pub_.publish(msg);

        // 只触发一次（发送后退出监听？不，继续运行以支持多次触发）
        // 但也可在此处停止以避免重复触发。这里保持运行。
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;

  int socket_fd_;
  std::atomic<bool> running_;
  std::thread rx_thread_;
  std::string can_device_;
  uint32_t can_id_;
};

}  // namespace robot_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "start_signal_pub_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::StartSignalPub node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[start_signal_pub] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
