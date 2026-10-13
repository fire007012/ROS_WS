// Test-only transport adapter: inject complete CAN frames into the production
// decoder/bridge. No CAN interface, motor sender or physical chassis is started.
#include "robot_navigation/vl53l1x_can_bridge.h"
#include <std_msgs/UInt8MultiArray.h>
namespace robot_navigation {
class Vl53l1xBridgeTestPeer {
 public:
  static bool start(Vl53l1xCanBridge& b) {
    if (!b.configure()) return false;
    b.poll_timer_ = b.nh_.createSteadyTimer(ros::WallDuration(.005),
        &Vl53l1xCanBridge::pollTimerCallback, &b);
    b.diagnostics_timer_ = b.nh_.createSteadyTimer(ros::WallDuration(.1),
        &Vl53l1xCanBridge::diagnosticsTimerCallback, &b);
    return true;
  }
  static void receive(Vl53l1xCanBridge& b, const std_msgs::UInt8MultiArray::ConstPtr& m) {
    if (m->data.size() < 2 || m->data.size() > 10) return;
    can_frame f{};
    f.can_id = m->data[0] | (m->data[1] << 8);
    f.can_dlc = m->data.size() - 2;
    std::copy(m->data.begin() + 2, m->data.end(), f.data);
    b.handleFrame(f);
  }
};
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "upper_alignment_bridge_fixture");
  ros::NodeHandle nh, pnh("~");
  robot_navigation::Vl53l1xCanBridge bridge(nh, pnh);
  if (!robot_navigation::Vl53l1xBridgeTestPeer::start(bridge)) return 1;
  auto sub = nh.subscribe<std_msgs::UInt8MultiArray>("/test/range_can", 50,
      [&bridge](const std_msgs::UInt8MultiArray::ConstPtr& m) {
        robot_navigation::Vl53l1xBridgeTestPeer::receive(bridge, m);
      });
  ros::spin();
}
