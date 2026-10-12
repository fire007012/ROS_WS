#include <arm_and_gripper/arm_and_gripper_controller.h>
#include <gtest/gtest.h>
#include <future>
#include <chrono>
#include <thread>
#include <vector>

TEST(ArmCancellation, InterruptsWaitAndNeverResumesAfterReset) {
  ros::NodeHandle nh, pnh("~");
  pnh.setParam("arm_move_timeout_s", 0.05);
  pnh.setParam("servo_hold_duration_s", 5.0);
  std::mutex mutex;
  std::vector<std::vector<uint8_t>> frames;
  arm_and_gripper::ArmAndGripperController controller(nh, pnh,
      [&](uint32_t, const uint8_t* data, uint8_t size, bool) {
        std::lock_guard<std::mutex> lock(mutex);
        frames.emplace_back(data, data+size);
        return true;
      }, [](uint32_t,uint8_t,uint8_t,uint8_t,uint8_t,double){return true;});
  ASSERT_TRUE(controller.init());
  ros::AsyncSpinner spinner(3);
  spinner.start();
  auto estop = nh.advertise<std_msgs::Bool>("/emergency_stop", 1);
  auto reset = nh.advertise<std_msgs::Bool>("/emergency_stop/reset", 1);
  auto cancel = nh.advertise<std_msgs::Empty>("/mission/cancel", 1);
  auto enabled = nh.advertise<std_msgs::Bool>("/mission/actions_enabled", 1);
  auto client = nh.serviceClient<arm_and_gripper::ArmPlaceMedicine>("/arm_place_medicine");
  const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while (estop.getNumSubscribers()==0 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_GT(estop.getNumSubscribers(), 0u);
  for (int mode=0; mode<2; ++mode) {
    std_msgs::Bool yes;yes.data=true;
    enabled.publish(yes);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
      std::lock_guard<std::mutex> lock(mutex);
      frames.clear();
    }
    auto call = std::async(std::launch::async, [&]() {
      arm_and_gripper::ArmPlaceMedicine service;
      service.request.bed_id=1;
      service.request.box_id=1;
      return client.call(service) && service.response.success;
    });
    bool started=false;
    const auto start_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!started && std::chrono::steady_clock::now()<start_deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        for(const auto& frame:frames)
          if(mode==0 ? frame[0]==0x21 : (frame[0]==0x22 && frame[1]==1 && frame[2]==0)) started=true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(started);
    if(mode==0) estop.publish(yes); else cancel.publish(std_msgs::Empty());
    ASSERT_EQ(call.wait_for(std::chrono::milliseconds(400)), std::future_status::ready);
    EXPECT_FALSE(call.get());
    {
      std::lock_guard<std::mutex> lock(mutex);
      bool stopped=false, closed=false;
      for(const auto& frame:frames) {
        if(frame[0]==0x23) stopped=true;
        if(stopped) { EXPECT_NE(frame[0], 0x21) << "motion after stop"; }
        if(frame[0]==0x22 && frame[1]==3 && frame[2]==180 && frame[3]==180) closed=true;
      }
      EXPECT_TRUE(stopped);
      EXPECT_TRUE(closed);
    }
    {
      arm_and_gripper::ArmPlaceMedicine service;
      service.request.bed_id=1;service.request.box_id=1;
      ASSERT_TRUE(client.call(service));
      EXPECT_FALSE(service.response.success);
    }
    size_t count;
    {std::lock_guard<std::mutex> lock(mutex);count=frames.size();}
    reset.publish(yes);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    {std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(frames.size(), count);}
  }
  spinner.stop();
}


TEST(ArmSequence, BothBoxesUseFourStagesAndIndependentAbsoluteHome) {
  ros::NodeHandle nh,pnh("~");
  pnh.setParam("arm_move_timeout_s",0.01);
  pnh.setParam("servo_hold_duration_s",0.01);
  pnh.setParam("old_servo_move_duration_s",0.01);pnh.setParam("old_servo_return_duration_s",0.01);
  pnh.setParam("new_servo_move_duration_s",0.01);pnh.setParam("new_servo_return_duration_s",0.01);
  pnh.setParam("arm_place_angle_1",15.0);pnh.setParam("arm_place_angle_2",-20.0);
  pnh.setParam("arm_home_angle_1",2.0);pnh.setParam("arm_home_angle_2",-3.0);
  struct Frame {uint32_t id;std::vector<uint8_t> data;};
  std::vector<Frame> frames;
  arm_and_gripper::ArmAndGripperController controller(nh,pnh,
      [&](uint32_t id,const uint8_t *data,uint8_t size,bool ext){
        EXPECT_TRUE(ext);EXPECT_EQ(size,8);frames.push_back({id,{data,data+size}});return true;
      },[](uint32_t,uint8_t,uint8_t,uint8_t,uint8_t,double){return true;});
  ASSERT_TRUE(controller.init());ros::AsyncSpinner spinner(3);spinner.start();
  auto client=nh.serviceClient<arm_and_gripper::ArmPlaceMedicine>("/arm_place_medicine");
  for(int box:{1,3}) {
    frames.clear();arm_and_gripper::ArmPlaceMedicine service;service.request.bed_id=1;service.request.box_id=box;
    ASSERT_TRUE(client.call(service));ASSERT_TRUE(service.response.success);ASSERT_EQ(frames.size(),5u);
    EXPECT_EQ(frames[0].id,0x701u);EXPECT_EQ(frames[0].data[0],0x21);
    EXPECT_EQ(frames[0].data[1],3);EXPECT_EQ(frames[0].data[2],150);
    EXPECT_EQ(frames[0].data[4],56);EXPECT_EQ(frames[0].data[5],255); // -200 deci-deg
    EXPECT_EQ(frames[1].id,0x700u);EXPECT_EQ(frames[1].data[0],0x22);
    EXPECT_EQ(frames[1].data[1],box==1?1:2);
    EXPECT_EQ(frames[1].data[box==1?2:3],0);
    EXPECT_EQ(frames[2].data[box==1?2:3],180);
    EXPECT_EQ(frames[3].id,0x701u);EXPECT_EQ(frames[3].data[0],0x21);
    EXPECT_EQ(frames[3].data[2],20);EXPECT_EQ(frames[3].data[3],0);
    EXPECT_EQ(frames[3].data[4],226);EXPECT_EQ(frames[3].data[5],255);
    EXPECT_EQ(frames[4].data[0],0x24);
    for(const auto &frame:frames){uint8_t sum=0;for(int i=0;i<7;++i)sum+=frame.data[i];EXPECT_EQ(sum,frame.data[7]);}
  }
  spinner.stop();
}

TEST(ArmSequence, MissingArmAckPreventsOpeningAndReportsFailure) {
  ros::NodeHandle nh,pnh("~");
  std::vector<std::vector<uint8_t>> frames;
  arm_and_gripper::ArmAndGripperController controller(nh,pnh,
      [&](uint32_t,const uint8_t *data,uint8_t size,bool){frames.emplace_back(data,data+size);return true;},
      [](uint32_t,uint8_t,uint8_t,uint8_t,uint8_t,double){return false;});
  ASSERT_TRUE(controller.init());ros::AsyncSpinner spinner(3);spinner.start();
  auto client=nh.serviceClient<arm_and_gripper::ArmPlaceMedicine>("/arm_place_medicine");
  arm_and_gripper::ArmPlaceMedicine service;service.request.bed_id=3;service.request.box_id=3;
  ASSERT_TRUE(client.call(service));EXPECT_FALSE(service.response.success);
  ASSERT_EQ(frames.size(),3u);EXPECT_EQ(frames[0][0],0x21);EXPECT_EQ(frames[1][0],0x23);
  EXPECT_EQ(frames[2][0],0x22);EXPECT_EQ(frames[2][2],180);EXPECT_EQ(frames[2][3],180);
  spinner.stop();
}


TEST(ArmSequence, FailedBoxCloseAckNeverRetractsArm) {
  ros::NodeHandle nh,pnh("~");
  unsigned box_acks=0;
  std::vector<std::vector<uint8_t>> frames;
  arm_and_gripper::ArmAndGripperController controller(nh,pnh,
      [&](uint32_t,const uint8_t *data,uint8_t size,bool){frames.emplace_back(data,data+size);return true;},
      [&](uint32_t,uint8_t,uint8_t,uint8_t opcode,uint8_t,double){return opcode!=0x22 || ++box_acks<2;});
  ASSERT_TRUE(controller.init());ros::AsyncSpinner spinner(3);spinner.start();
  auto client=nh.serviceClient<arm_and_gripper::ArmPlaceMedicine>("/arm_place_medicine");
  arm_and_gripper::ArmPlaceMedicine service;service.request.bed_id=1;service.request.box_id=1;
  ASSERT_TRUE(client.call(service));EXPECT_FALSE(service.response.success);
  ASSERT_EQ(frames.size(),5u);
  EXPECT_EQ(frames[0][0],0x21);EXPECT_EQ(frames[1][0],0x22);EXPECT_EQ(frames[2][0],0x22);
  EXPECT_EQ(frames[3][0],0x23);EXPECT_EQ(frames[4][0],0x22);
  spinner.stop();
}

TEST(ArmSequence, InvalidAnglesFailInitialization) {
  ros::NodeHandle nh,pnh("~");pnh.setParam("arm_place_angle_1",180.0);
  arm_and_gripper::ArmAndGripperController controller(nh,pnh,
      [](uint32_t,const uint8_t*,uint8_t,bool){return true;});
  EXPECT_FALSE(controller.init());pnh.setParam("arm_place_angle_1",15.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_arm_cancellation");
  return RUN_ALL_TESTS();
}
