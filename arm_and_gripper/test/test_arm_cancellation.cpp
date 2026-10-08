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
      });
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
          if(mode==0 ? frame[0]==0xfb : (frame[0]==0x20 && frame[2]>0)) started=true;
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
        if(frame==std::vector<uint8_t>({0xfe,0x98,0,0x6b})) stopped=true;
        if(stopped) { EXPECT_NE(frame[0], 0xfb) << "motion after stop"; }
        if(frame[0]==0x20 && frame[1]==3 && frame[2]==0 && frame[3]==0) closed=true;
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_arm_cancellation");
  return RUN_ALL_TESTS();
}
