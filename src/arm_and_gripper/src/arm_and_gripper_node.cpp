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

namespace arm_and_gripper {
ArmAndGripperController::ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), can_device_("can0"), arm_can_id_(0x205), socket_fd_(-1),
      auto_start_on_fine_tuning_(false), default_reset_arm_(true), default_arm_angle_(180.0),
      arm_move_timeout_s_(3.0), servo_hold_duration_s_(1.0), old_servo_move_duration_s_(0.5),
      old_servo_return_duration_s_(1.0), new_servo_open_duration_s_(0.235),
      new_servo_close_duration_s_(0.235), servo_settle_duration_s_(0.05),
      old_servo_open_angle_(180), old_servo_close_angle_(0), new_servo_open_control_(0),
      new_servo_close_control_(180), new_servo_stop_control_(88), sequence_running_(false) {}

ArmAndGripperController::~ArmAndGripperController(){ closeCanSocket(); }

bool ArmAndGripperController::init(){
  pnh_.param<std::string>("can_device",can_device_,"can0");
  int id=static_cast<int>(arm_can_id_); pnh_.param("arm_can_id",id,id); arm_can_id_=static_cast<uint32_t>(id);
  pnh_.param<bool>("auto_start_on_fine_tuning",auto_start_on_fine_tuning_,false);
  pnh_.param<bool>("default_reset_arm",default_reset_arm_,true);
  pnh_.param("default_arm_angle",default_arm_angle_,180.0);
  pnh_.param("arm_move_timeout_s",arm_move_timeout_s_,3.0);
  pnh_.param("servo_hold_duration_s",servo_hold_duration_s_,1.0);
  pnh_.param("old_servo_move_duration_s",old_servo_move_duration_s_,0.5);
  pnh_.param("old_servo_return_duration_s",old_servo_return_duration_s_,1.0);
  pnh_.param("new_servo_open_duration_s",new_servo_open_duration_s_,0.235);
  pnh_.param("new_servo_close_duration_s",new_servo_close_duration_s_,0.235);
  pnh_.param("servo_settle_duration_s",servo_settle_duration_s_,0.05);
  pnh_.param("old_servo_open_angle",old_servo_open_angle_,180);
  pnh_.param("old_servo_close_angle",old_servo_close_angle_,0);
  pnh_.param("new_servo_open_control",new_servo_open_control_,0);
  pnh_.param("new_servo_close_control",new_servo_close_control_,180);
  pnh_.param("new_servo_stop_control",new_servo_stop_control_,88);
  fine_tuning_done_sub_=nh_.subscribe("/fine_tuning_done",1,&ArmAndGripperController::fineTuningDoneCallback,this);
  medicine_release_done_pub_=nh_.advertise<std_msgs::Bool>("/medicine_release_done",1,true);
  std_msgs::Bool initial; initial.data=false; medicine_release_done_pub_.publish(initial);
  place_medicine_srv_=nh_.advertiseService("/place_medicine",&ArmAndGripperController::placeMedicineCallback,this);
  arm_place_medicine_srv_=nh_.advertiseService("/arm_place_medicine",&ArmAndGripperController::armPlaceMedicineCallback,this);
  if(!initCanSocket()) ROS_WARN("[arm_and_gripper] CAN not ready; retry on command");
  ROS_INFO("[arm_and_gripper] STM32 CAN release ready; auto_start_on_fine_tuning=%s",auto_start_on_fine_tuning_?"true":"false");
  return true;
}
void ArmAndGripperController::fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg){
  if(!msg->data||!auto_start_on_fine_tuning_)return;
  executePlaceSequence(default_arm_angle_,1,default_reset_arm_);
}
bool ArmAndGripperController::placeMedicineCallback(PlaceMedicine::Request&,PlaceMedicine::Response& res){
  if(sequence_running_.load()){res.success=false;res.message="busy";return true;}
  res.success=executePlaceSequence(default_arm_angle_,1,default_reset_arm_);
  res.message=res.success?"release complete":"release failed"; return true;
}
bool ArmAndGripperController::armPlaceMedicineCallback(ArmPlaceMedicine::Request& req,ArmPlaceMedicine::Response& res){
  if(sequence_running_.load()){res.success=false;res.message="busy";return true;}
  if(req.bed_id!=1&&req.bed_id!=3){res.success=false;res.message="invalid bed_id";return true;}
  if(req.box_id!=1&&req.box_id!=3){res.success=false;res.message="invalid box_id";return true;}
  res.success=executePlaceSequence(default_arm_angle_,req.box_id,true);
  res.message=res.success?"release complete":"release failed"; return true;
}

bool ArmAndGripperController::initCanSocket(){
  if(socket_fd_>=0)return true; socket_fd_=socket(PF_CAN,SOCK_RAW,CAN_RAW); if(socket_fd_<0)return false;
  struct ifreq ifr{}; std::strncpy(ifr.ifr_name,can_device_.c_str(),IFNAMSIZ-1);
  if(ioctl(socket_fd_,SIOCGIFINDEX,&ifr)<0){closeCanSocket();return false;}
  struct sockaddr_can addr{};addr.can_family=AF_CAN;addr.can_ifindex=ifr.ifr_ifindex;
  if(bind(socket_fd_,reinterpret_cast<struct sockaddr*>(&addr),sizeof(addr))<0){closeCanSocket();return false;} return true;
}
void ArmAndGripperController::closeCanSocket(){if(socket_fd_>=0){close(socket_fd_);socket_fd_=-1;}}
bool ArmAndGripperController::sendCanFrame(uint32_t id,const uint8_t* data,uint8_t dlc,bool ext){
  if(socket_fd_<0&&!initCanSocket())return false; struct can_frame f{};f.can_id=id|(ext?CAN_EFF_FLAG:0);f.can_dlc=dlc;std::memcpy(f.data,data,dlc);
  if(write(socket_fd_,&f,sizeof(f))!=static_cast<ssize_t>(sizeof(f))){ROS_ERROR("[arm_and_gripper] CAN write failed: %s",std::strerror(errno));return false;}return true;
}
bool ArmAndGripperController::sendArmAngleCommand(double angle){
  uint32_t addr=arm_can_id_&0xffu;uint32_t deg=static_cast<uint32_t>(std::lround(std::fabs(angle)*10.0));
  uint8_t a[8]={0xfb,static_cast<uint8_t>(angle>=0?1:0),0x01,0xf4,static_cast<uint8_t>(deg>>24),static_cast<uint8_t>(deg>>16),static_cast<uint8_t>(deg>>8),static_cast<uint8_t>(deg)};
  uint8_t b[4]={0xfb,0x02,0x00,0x6b};return sendCanFrame(addr<<8,a,8,true)&&sendCanFrame((addr<<8)|1,b,4,true);
}
bool ArmAndGripperController::sendServoTriggerCommand(uint8_t mask,int old_control,int new_control,int return_control,uint16_t hold_ms){
  uint8_t d[8]={0};d[0]=0x20;d[1]=mask;d[2]=static_cast<uint8_t>(std::max(0,std::min(180,old_control)));d[3]=static_cast<uint8_t>(std::max(0,std::min(180,new_control)));d[4]=static_cast<uint8_t>(std::max(0,std::min(180,return_control)));d[5]=hold_ms&0xff;d[6]=hold_ms>>8;for(int i=0;i<7;i++)d[7]=static_cast<uint8_t>(d[7]+d[i]);
  return sendCanFrame(0x700,d,8,true);
}
bool ArmAndGripperController::executePlaceSequence(double arm_angle,int8_t box_id,bool reset_arm){
  std::lock_guard<std::mutex> lock(seq_mutex_);sequence_running_.store(true);bool ok=true;
  std_msgs::Bool pending;pending.data=false;medicine_release_done_pub_.publish(pending);
  if(!sendArmAngleCommand(arm_angle))ok=false;ros::Duration(arm_move_timeout_s_).sleep();
  if(ok&&box_id==1){
    ok=sendServoTriggerCommand(0x01,old_servo_open_angle_,0,0,0);ros::Duration(old_servo_move_duration_s_+servo_hold_duration_s_).sleep();
    ok=sendServoTriggerCommand(0x01,old_servo_close_angle_,0,0,0)&&ok;ros::Duration(old_servo_return_duration_s_).sleep();
  }else if(ok&&box_id==3){
    ok=sendServoTriggerCommand(0x02,0,new_servo_open_control_,new_servo_stop_control_,static_cast<uint16_t>(new_servo_open_duration_s_*1000));ros::Duration(new_servo_open_duration_s_+servo_settle_duration_s_).sleep();
    ok=sendServoTriggerCommand(0x02,0,new_servo_close_control_,new_servo_stop_control_,static_cast<uint16_t>(new_servo_close_duration_s_*1000))&&ok;ros::Duration(new_servo_close_duration_s_+servo_settle_duration_s_).sleep();
  }else ok=false;
  if(ok&&reset_arm){ok=sendArmAngleCommand(-arm_angle);ros::Duration(arm_move_timeout_s_).sleep();}
  sequence_running_.store(false);std_msgs::Bool done;done.data=ok;medicine_release_done_pub_.publish(done);return ok;
}
}
int main(int argc,char**argv){ros::init(argc,argv,"arm_and_gripper_node");ros::NodeHandle nh,pnh("~");arm_and_gripper::ArmAndGripperController c(nh,pnh);return c.init()? (ros::spin(),0):1;}
