// Include the real controller with an in-memory transport; no SocketCAN opens.
#define OPEN_MEDICINE_BOX_NO_MAIN
#include "../src/open_medicine_box.cpp"
#include <gtest/gtest.h>
#include <future>
#include <thread>
#include <vector>

TEST(BoxCancellation, ClosesActiveBoxBeforeInterruptingHold) {
  ros::NodeHandle nh, pnh("~");
  pnh.setParam("hold_duration_sec", 5.0);
  std::mutex mutex;
  std::vector<std::vector<uint8_t>> frames;
  robot_navigation::OpenMedicineBoxNode controller(nh,pnh,
      [&](uint32_t, const uint8_t* data, uint8_t size) {
        std::lock_guard<std::mutex> lock(mutex);
        frames.emplace_back(data,data+size);
        return true;
      });
  ASSERT_TRUE(controller.init());
  ros::AsyncSpinner spinner(3);spinner.start();
  auto cancel=nh.advertise<std_msgs::Empty>("/mission/cancel",1);
  auto client=nh.serviceClient<robot_navigation::OpenMedicineBox>("/open_medicine_box");
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(cancel.getNumSubscribers()==0 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_GT(cancel.getNumSubscribers(),0u);
  auto call=std::async(std::launch::async,[&]() {
    robot_navigation::OpenMedicineBox service;service.request.box_id=3;
    return client.call(service)&&service.response.success;
  });
  bool opened=false;
  while(!opened && std::chrono::steady_clock::now()<deadline) {
    {std::lock_guard<std::mutex> lock(mutex);opened=!frames.empty();}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(opened);
  cancel.publish(std_msgs::Empty());
  ASSERT_EQ(call.wait_for(std::chrono::milliseconds(400)),std::future_status::ready);
  EXPECT_FALSE(call.get());
  robot_navigation::OpenMedicineBox late;late.request.box_id=3;
  ASSERT_TRUE(client.call(late));
  EXPECT_FALSE(late.response.success);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(frames.size(),2u);
    EXPECT_EQ(frames[0][1],3);EXPECT_EQ(frames[0][2],90);
    EXPECT_EQ(frames[1][1],3);EXPECT_EQ(frames[1][2],0);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  {std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(frames.size(),2u);}
  spinner.stop();
}

TEST(BoxCancellation, AlsoClosesBetweenServiceCompletionAndArmStart) {
  ros::NodeHandle nh,pnh("~");
  pnh.setParam("hold_duration_sec",0.0);
  std::mutex mutex;
  std::vector<int> angles;
  robot_navigation::OpenMedicineBoxNode controller(nh,pnh,
      [&](uint32_t,const uint8_t* data,uint8_t) {
        std::lock_guard<std::mutex> lock(mutex);angles.push_back(data[2]);return true;
      });
  ASSERT_TRUE(controller.init());
  ros::AsyncSpinner spinner(3);spinner.start();
  auto cancel=nh.advertise<std_msgs::Empty>("/mission/cancel",1);
  auto client=nh.serviceClient<robot_navigation::OpenMedicineBox>("/open_medicine_box");
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(cancel.getNumSubscribers()==0 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_GT(cancel.getNumSubscribers(),0u);
  robot_navigation::OpenMedicineBox service;service.request.box_id=1;
  ASSERT_TRUE(client.call(service));ASSERT_TRUE(service.response.success);
  cancel.publish(std_msgs::Empty());
  bool closed=false;
  while(!closed && std::chrono::steady_clock::now()<deadline) {
    {std::lock_guard<std::mutex> lock(mutex);closed=angles.size()==2;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(closed);
  {std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(angles,std::vector<int>({90,0}));}
  spinner.stop();
}

int main(int argc,char**argv) {
  testing::InitGoogleTest(&argc,argv);ros::init(argc,argv,"test_box_cancellation");
  return RUN_ALL_TESTS();
}
