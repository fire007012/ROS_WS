#include "arm_and_gripper/arm_and_gripper_controller.h"
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
#include <chrono>
#include <poll.h>

namespace arm_and_gripper {
ArmAndGripperController::ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh, FrameSender sender, AckReceiver receiver)
    : nh_(nh), pnh_(pnh), can_device_("can0"), arm_can_id_(0x701),
      arm_place_angle_1_(15.0), arm_place_angle_2_(15.0), arm_home_angle_1_(0.0), arm_home_angle_2_(0.0),
      arm_ack_timeout_s_(0.5), socket_fd_(-1),
      auto_start_on_fine_tuning_(false), default_reset_arm_(true),
      arm_move_timeout_s_(3.0), servo_hold_duration_s_(1.0), old_servo_move_duration_s_(0.5),
      old_servo_return_duration_s_(1.0), new_servo_move_duration_s_(0.5),
      new_servo_return_duration_s_(1.0),
      old_servo_open_angle_(0), old_servo_close_angle_(180), new_servo_open_angle_(0),
      new_servo_close_angle_(180), sequence_running_(false), sender_(std::move(sender)), receiver_(std::move(receiver)) {}

ArmAndGripperController::~ArmAndGripperController(){ closeCanSocket(); }

bool ArmAndGripperController::init(){
  pnh_.param<std::string>("can_device",can_device_,"can0");
  int id=static_cast<int>(arm_can_id_); pnh_.param("arm_can_id",id,id); arm_can_id_=static_cast<uint32_t>(id);
  pnh_.param<bool>("auto_start_on_fine_tuning",auto_start_on_fine_tuning_,false);
  pnh_.param<bool>("default_reset_arm",default_reset_arm_,true);
  pnh_.param("arm_place_angle_1",arm_place_angle_1_,15.0);
  pnh_.param("arm_place_angle_2",arm_place_angle_2_,15.0);
  pnh_.param("arm_home_angle_1",arm_home_angle_1_,0.0);
  pnh_.param("arm_home_angle_2",arm_home_angle_2_,0.0);
  pnh_.param("arm_ack_timeout_s",arm_ack_timeout_s_,0.5);
  pnh_.param("arm_move_timeout_s",arm_move_timeout_s_,3.0);
  pnh_.param("servo_hold_duration_s",servo_hold_duration_s_,1.0);
  pnh_.param("old_servo_move_duration_s",old_servo_move_duration_s_,0.5);
  pnh_.param("old_servo_return_duration_s",old_servo_return_duration_s_,1.0);
  pnh_.param("new_servo_move_duration_s",new_servo_move_duration_s_,0.5);
  pnh_.param("new_servo_return_duration_s",new_servo_return_duration_s_,1.0);
  pnh_.param("old_servo_open_angle",old_servo_open_angle_,0);
  pnh_.param("old_servo_close_angle",old_servo_close_angle_,180);
  pnh_.param("new_servo_open_angle",new_servo_open_angle_,0);
  pnh_.param("new_servo_close_angle",new_servo_close_angle_,180);
  for (double value : {arm_place_angle_1_,arm_place_angle_2_,arm_home_angle_1_,arm_home_angle_2_}) {
    if (!std::isfinite(value) || std::fabs(value) > 80.0) {
      ROS_ERROR("[arm_and_gripper] angles must be finite and inside firmware default +/-80 deg"); return false;
    }
  }
  for (double value : {arm_move_timeout_s_,servo_hold_duration_s_,old_servo_move_duration_s_,
                       old_servo_return_duration_s_,new_servo_move_duration_s_,new_servo_return_duration_s_}) {
    if (!std::isfinite(value) || value < 0.0 || value > 30.0) return false;
  }
  for (int value : {old_servo_open_angle_,old_servo_close_angle_,new_servo_open_angle_,new_servo_close_angle_}) {
    if (value < 0 || value > 180) return false;
  }
  if (!default_reset_arm_ || arm_can_id_ != 0x701 || !std::isfinite(arm_ack_timeout_s_) ||
      arm_ack_timeout_s_ <= 0.0 || arm_ack_timeout_s_ > 5.0) {
    ROS_ERROR("[arm_and_gripper] STM32 PWM requires arm_can_id=0x701 and a valid ACK timeout"); return false;
  }
  fine_tuning_done_sub_=nh_.subscribe("/fine_tuning_done",1,&ArmAndGripperController::fineTuningDoneCallback,this);
  estop_sub_=nh_.subscribe("/emergency_stop",1,&ArmAndGripperController::emergencyStopCallback,this);
  reset_sub_=nh_.subscribe("/emergency_stop/reset",1,&ArmAndGripperController::emergencyResetCallback,this);
  cancel_sub_=nh_.subscribe("/mission/cancel",1,&ArmAndGripperController::cancelCallback,this);
  actions_enabled_sub_=nh_.subscribe<std_msgs::Bool>("/mission/actions_enabled",1,
      [this](const std_msgs::Bool::ConstPtr& msg){
        if(msg->data) actions_enabled_=true; else cancelSequence();
      });
  medicine_placed_pub_=nh_.advertise<std_msgs::Int8>("/medicine/placed",1,false);
  medicine_release_done_pub_=nh_.advertise<std_msgs::Bool>("/medicine_release_done",1,true);
  std_msgs::Bool initial; initial.data=false; medicine_release_done_pub_.publish(initial);
  place_medicine_srv_=nh_.advertiseService("/place_medicine",&ArmAndGripperController::placeMedicineCallback,this);
  arm_place_medicine_srv_=nh_.advertiseService("/arm_place_medicine",&ArmAndGripperController::armPlaceMedicineCallback,this);
  if(!sender_&&!initCanSocket()) ROS_WARN("[arm_and_gripper] CAN not ready; retry on command");
  ROS_INFO("[arm_and_gripper] STM32 CAN release ready; auto_start_on_fine_tuning=%s",auto_start_on_fine_tuning_?"true":"false");
  return true;
}
void ArmAndGripperController::fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg){
  if(!msg->data||!auto_start_on_fine_tuning_)return;
  executePlaceSequence(1,default_reset_arm_);
}
bool ArmAndGripperController::placeMedicineCallback(PlaceMedicine::Request& req,PlaceMedicine::Response& res){
  if(!req.reset_arm || !std::isfinite(req.arm_angle) || !std::isfinite(req.servo1_open_angle) ||
     !std::isfinite(req.servo2_open_angle) || req.arm_angle>=0 || req.servo1_open_angle>=0 || req.servo2_open_angle>=0) {
    res.success=false;res.message="Use /arm_place_medicine with bed_id/box_id; paired angles are configured in YAML. Legacy alias requires all angles=-1 and reset_arm=true.";return true;
  }
  if(sequence_running_.load()){res.success=false;res.message="busy";return true;}
  res.success=executePlaceSequence(1,default_reset_arm_);
  res.message=res.success?"commands acknowledged; timed release sequence complete (no position feedback)":"release failed or cancelled"; return true;
}
bool ArmAndGripperController::armPlaceMedicineCallback(ArmPlaceMedicine::Request& req,ArmPlaceMedicine::Response& res){
  if(sequence_running_.load()){res.success=false;res.message="busy";return true;}
  if(req.bed_id!=1&&req.bed_id!=3){res.success=false;res.message="invalid bed_id";return true;}
  if(req.box_id!=1&&req.box_id!=3){res.success=false;res.message="invalid box_id";return true;}
  res.success=executePlaceSequence(req.box_id,true,req.bed_id);
  res.message=res.success?"commands acknowledged; timed release sequence complete (no position feedback)":"release failed or cancelled"; return true;
}

bool ArmAndGripperController::initCanSocket(){
  if(socket_fd_>=0)return true;
  socket_fd_=socket(PF_CAN,SOCK_RAW,CAN_RAW);
  if(socket_fd_<0)return false;
  struct can_filter filters[2]={{0x702|CAN_EFF_FLAG,CAN_EFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG},
                                {0x703|CAN_EFF_FLAG,CAN_EFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG}};
  if(setsockopt(socket_fd_,SOL_CAN_RAW,CAN_RAW_FILTER,filters,sizeof(filters))<0) {closeCanSocket();return false;}
  struct ifreq ifr{}; std::strncpy(ifr.ifr_name,can_device_.c_str(),IFNAMSIZ-1);
  if(ioctl(socket_fd_,SIOCGIFINDEX,&ifr)<0){closeCanSocket();return false;}
  struct sockaddr_can addr{};addr.can_family=AF_CAN;addr.can_ifindex=ifr.ifr_ifindex;
  if(bind(socket_fd_,reinterpret_cast<struct sockaddr*>(&addr),sizeof(addr))<0){closeCanSocket();return false;} return true;
}
void ArmAndGripperController::closeCanSocket(){if(socket_fd_>=0){close(socket_fd_);socket_fd_=-1;}}
bool ArmAndGripperController::sendCanFrame(uint32_t id,const uint8_t* data,uint8_t dlc,bool ext){
  std::lock_guard<std::mutex> lock(command_mutex_);
  if(estop_latched_ || !actions_enabled_ || cancel_generation_.load()!=sequence_generation_) return false;
  return sendRawFrame(id,data,dlc,ext);
}
bool ArmAndGripperController::sendRawFrame(uint32_t id,const uint8_t* data,uint8_t dlc,bool ext){
  if(sender_) return sender_(id,data,dlc,ext);
  if(socket_fd_<0&&!initCanSocket())return false;
  struct can_frame f{};f.can_id=id|(ext?CAN_EFF_FLAG:0);f.can_dlc=dlc;std::memcpy(f.data,data,dlc);
  if(write(socket_fd_,&f,sizeof(f))!=static_cast<ssize_t>(sizeof(f))){ROS_ERROR("[arm_and_gripper] CAN write failed: %s",std::strerror(errno));return false;}return true;
}
namespace {
void checksum(uint8_t data[8]) {
  data[7]=0; for(int i=0;i<7;++i) data[7]=static_cast<uint8_t>(data[7]+data[i]);
}
}
bool ArmAndGripperController::waitForAck(uint32_t id,uint8_t code,uint8_t seq,uint8_t opcode,uint8_t mask) {
  if(receiver_) return receiver_(id,code,seq,opcode,mask,arm_ack_timeout_s_);
  // An injected transport alone is not evidence of firmware acceptance.
  if(sender_) return false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(arm_ack_timeout_s_);
  while(ros::ok()&&!estop_latched_&&actions_enabled_&&cancel_generation_.load()==sequence_generation_) {
    if(std::chrono::steady_clock::now()>=deadline) break;
    struct pollfd descriptor{socket_fd_,POLLIN,0};
    int ready=poll(&descriptor,1,10);
    if(ready<0) { if(errno==EINTR) continue; break; }
    if(ready==0) continue;
    struct can_frame frame{};
    if(recv(socket_fd_,&frame,sizeof(frame),MSG_DONTWAIT)!=static_cast<ssize_t>(sizeof(frame))) continue;
    if(!(frame.can_id&CAN_EFF_FLAG)||(frame.can_id&(CAN_RTR_FLAG|CAN_ERR_FLAG))||
       (frame.can_id&CAN_EFF_MASK)!=id||frame.can_dlc!=8) continue;
    uint8_t sum=0;for(int i=0;i<7;++i)sum=static_cast<uint8_t>(sum+frame.data[i]);
    if(sum!=frame.data[7]||frame.data[0]!=code||frame.data[1]!=seq||
       frame.data[3]!=opcode||frame.data[4]!=mask) continue;
    if(frame.data[2]!=0) ROS_ERROR("[arm_and_gripper] STM32 rejected command: result=%u",frame.data[2]);
    return frame.data[2]==0;
  }
  ROS_ERROR("[arm_and_gripper] missing STM32 ACK or sequence cancelled");return false;
}
bool ArmAndGripperController::sendAcknowledged(uint32_t id,uint32_t ack_id,uint8_t code,uint8_t data[8]) {
  data[6]=++command_seq_;checksum(data);
  return sendCanFrame(id,data,8,true)&&waitForAck(ack_id,code,data[6],data[0],data[1]);
}
bool ArmAndGripperController::sendArmAngleCommand(double first,double second){
  int16_t a=static_cast<int16_t>(std::lround(first*10.0));
  int16_t b=static_cast<int16_t>(std::lround(second*10.0));
  uint8_t d[8]={0x21,3,static_cast<uint8_t>(a),static_cast<uint8_t>(static_cast<uint16_t>(a)>>8),
      static_cast<uint8_t>(b),static_cast<uint8_t>(static_cast<uint16_t>(b)>>8),0,0};
  return sendAcknowledged(arm_can_id_,0x702,0xA1,d);
}
bool ArmAndGripperController::sendArmOwnershipCommand(uint8_t opcode) {
  uint8_t d[8]={opcode,0,0,0,0,0,0,0};return sendAcknowledged(arm_can_id_,0x702,0xA1,d);
}
bool ArmAndGripperController::sendServoTriggerCommand(uint8_t mask,int old_angle,int new_angle){
  uint8_t d[8]={0x22,mask,static_cast<uint8_t>(old_angle),static_cast<uint8_t>(new_angle),0,0,0,0};
  return sendAcknowledged(0x700,0x703,0xA2,d);
}
bool ArmAndGripperController::executePlaceSequence(int8_t box_id,bool reset_arm,int8_t bed_id){
  std::unique_lock<std::mutex> lock(seq_mutex_,std::try_to_lock);
  if(!lock.owns_lock() || estop_latched_ || !actions_enabled_) return false;
  sequence_generation_=cancel_generation_.load();
  sequence_running_.store(true);bool ok=true;
  std_msgs::Bool pending;pending.data=false;medicine_release_done_pub_.publish(pending);
  ROS_INFO("[arm_and_gripper] 1/4 arm place: PC6=%.1f PC7=%.1f",arm_place_angle_1_,arm_place_angle_2_);
  ok=sendArmAngleCommand(arm_place_angle_1_,arm_place_angle_2_)&&waitInterruptibly(arm_move_timeout_s_);
  if(ok&&box_id==1){
    ROS_INFO("[arm_and_gripper] 2/4 open box1 (PB0), then 3/4 close");
    ok=sendServoTriggerCommand(0x01,old_servo_open_angle_,0)&&waitInterruptibly(old_servo_move_duration_s_+servo_hold_duration_s_);
    // Timed release point, not physical medicine-arrival feedback. No latch.
    if(ok) {std_msgs::Int8 placed;placed.data=bed_id;medicine_placed_pub_.publish(placed);}
    if(ok) ok=sendServoTriggerCommand(0x01,old_servo_close_angle_,0)&&waitInterruptibly(old_servo_return_duration_s_);
  }else if(ok&&box_id==3){
    // PD14 now uses absolute angles, just like the PB0 positional servo.
    ROS_INFO("[arm_and_gripper] 2/4 open box3 (PD14), then 3/4 close");
    ok=sendServoTriggerCommand(0x02,0,new_servo_open_angle_)&&waitInterruptibly(new_servo_move_duration_s_+servo_hold_duration_s_);
    // Timed release point, not physical medicine-arrival feedback. No latch.
    if(ok) {std_msgs::Int8 placed;placed.data=bed_id;medicine_placed_pub_.publish(placed);}
    if(ok) ok=sendServoTriggerCommand(0x02,0,new_servo_close_angle_)&&waitInterruptibly(new_servo_return_duration_s_);
  }else ok=false;
  if(ok&&reset_arm) {
    ROS_INFO("[arm_and_gripper] 4/4 arm return: PC6=%.1f PC7=%.1f",arm_home_angle_1_,arm_home_angle_2_);
    ok=sendArmAngleCommand(arm_home_angle_1_,arm_home_angle_2_)&&waitInterruptibly(arm_move_timeout_s_);
  }
  if(ok) ok=sendArmOwnershipCommand(0x24);
  if(!ok) stopOutputs();
  sequence_running_.store(false);std_msgs::Bool done;done.data=ok;medicine_release_done_pub_.publish(done);return ok;
}
void ArmAndGripperController::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg){
  if(!msg->data || estop_latched_.exchange(true)) return;
  cancelSequence();
}
void ArmAndGripperController::emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data) estop_latched_=false; // Never restart the cancelled sequence.
}
void ArmAndGripperController::cancelCallback(const std_msgs::Empty::ConstPtr&){cancelSequence();}
void ArmAndGripperController::cancelSequence(){
  actions_enabled_=false;
  ++cancel_generation_;
  cancelled_.notify_all();
  if(sequence_running_) stopOutputs();
}
void ArmAndGripperController::stopOutputs(){
  std::lock_guard<std::mutex> lock(command_mutex_);
  // Keep arm PWM holding its last position; do not retract an unknown loaded pose.
  uint8_t stop[8]={0x23,0,0,0,0,0,0,0}; checksum(stop);
  sendRawFrame(arm_can_id_,stop,8,true);
  // Stop the release sequence and close both positional medicine-box servos.
  uint8_t close[8]={0x22,0x03,static_cast<uint8_t>(old_servo_close_angle_),
      static_cast<uint8_t>(new_servo_close_angle_),0,0,0,0};
  for(int i=0;i<7;++i) close[7]=static_cast<uint8_t>(close[7]+close[i]);
  sendRawFrame(0x700,close,8,true);
}
bool ArmAndGripperController::waitInterruptibly(double seconds){
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(std::max(0.0,seconds));
  std::unique_lock<std::mutex> lock(wait_mutex_);
  while(ros::ok()&&!estop_latched_&&cancel_generation_.load()==sequence_generation_){
    if(std::chrono::steady_clock::now()>=deadline) return true;
    cancelled_.wait_for(lock,std::chrono::milliseconds(20));
  }
  return false;
}
}
#ifndef ARM_AND_GRIPPER_NO_MAIN
int main(int argc,char**argv){
  ros::init(argc,argv,"arm_and_gripper_node");ros::NodeHandle nh,pnh("~");
  arm_and_gripper::ArmAndGripperController controller(nh,pnh);
  if(!controller.init()) return 1;
  ros::AsyncSpinner spinner(3);spinner.start();ros::waitForShutdown();return 0;
}
#endif
