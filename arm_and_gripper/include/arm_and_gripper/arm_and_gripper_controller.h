#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <arm_and_gripper/PlaceMedicine.h>
#include <arm_and_gripper/ArmPlaceMedicine.h>

namespace arm_and_gripper {
class ArmAndGripperController {
 public:
  ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
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
  bool sendServoTriggerCommand(uint8_t mask, int old_control, int new_control,
                               int return_control, uint16_t hold_ms);
  bool sendCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc, bool extended);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber fine_tuning_done_sub_;
  ros::Publisher medicine_release_done_pub_;
  ros::ServiceServer place_medicine_srv_, arm_place_medicine_srv_;

  std::string can_device_;
  uint32_t arm_can_id_;
  int socket_fd_;
  bool auto_start_on_fine_tuning_;
  bool default_reset_arm_;
  double default_arm_angle_, arm_move_timeout_s_;
  double servo_hold_duration_s_, old_servo_move_duration_s_, old_servo_return_duration_s_;
  double new_servo_open_duration_s_, new_servo_close_duration_s_, servo_settle_duration_s_;
  int old_servo_open_angle_, old_servo_close_angle_;
  int new_servo_open_control_, new_servo_close_control_, new_servo_stop_control_;
  std::atomic<bool> sequence_running_;
  std::mutex seq_mutex_;
};
}
