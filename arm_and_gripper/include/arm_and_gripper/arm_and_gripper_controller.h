#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <functional>
#include <condition_variable>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <arm_and_gripper/PlaceMedicine.h>
#include <arm_and_gripper/ArmPlaceMedicine.h>

namespace arm_and_gripper {
class ArmAndGripperController {
 public:
  using FrameSender = std::function<bool(uint32_t, const uint8_t*, uint8_t, bool)>;
  ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh, FrameSender sender = {});
  ~ArmAndGripperController();
  bool init();
 private:
  void fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg);
  bool placeMedicineCallback(PlaceMedicine::Request&, PlaceMedicine::Response&);
  bool armPlaceMedicineCallback(ArmPlaceMedicine::Request&, ArmPlaceMedicine::Response&);
  bool executePlaceSequence(double arm_angle, int8_t box_id, bool reset_arm);
  bool initCanSocket();
  void closeCanSocket();
  bool sendArmAngleCommand(double angle_deg);
  bool sendServoTriggerCommand(uint8_t mask, int old_angle, int new_angle,
                               int return_angle, uint16_t hold_ms);
  bool sendCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended);
  bool sendRawFrame(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended);
  void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
  void emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg);
  void cancelCallback(const std_msgs::Empty::ConstPtr&);
  void cancelSequence();
  void stopOutputs();
  bool waitInterruptibly(double seconds);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber fine_tuning_done_sub_;
  ros::Subscriber estop_sub_, reset_sub_, cancel_sub_, actions_enabled_sub_;
  ros::Publisher medicine_release_done_pub_;
  ros::ServiceServer place_medicine_srv_, arm_place_medicine_srv_;

  std::string can_device_;
  uint32_t arm_can_id_;
  int socket_fd_;
  bool auto_start_on_fine_tuning_;
  bool default_reset_arm_;
  double default_arm_angle_, arm_move_timeout_s_;
  double servo_hold_duration_s_, old_servo_move_duration_s_, old_servo_return_duration_s_;
  double new_servo_move_duration_s_, new_servo_return_duration_s_;
  int old_servo_open_angle_, old_servo_close_angle_;
  int new_servo_open_angle_, new_servo_close_angle_;
  std::atomic<bool> sequence_running_;
  std::mutex seq_mutex_;
  FrameSender sender_;
  std::atomic<bool> estop_latched_{false};
  std::atomic<bool> actions_enabled_{true};
  std::atomic<uint64_t> cancel_generation_{0};
  uint64_t sequence_generation_ = 0;
  std::mutex command_mutex_, wait_mutex_;
  std::condition_variable cancelled_;
};
}
