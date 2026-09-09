#include <cmath>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <path_manager/PathPoint.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Trigger.h>

namespace path_manager {

class MoveBasePathAdapter {
 public:
  MoveBasePathAdapter()
      : pnh_("~"), action_client_("move_base", true), has_target_(false),
        waiting_for_next_point_(false), path_revision_(0), goal_timeout_sec_(60.0) {}

  bool init() {
    pnh_.param<std::string>("goal_frame", goal_frame_, std::string("map"));
    pnh_.param<double>("goal_timeout_sec", goal_timeout_sec_, 60.0);
    goal_timeout_sec_ = std::max(1.0, goal_timeout_sec_);

    path_points_sub_ = nh_.subscribe("/path_points", 10,
        &MoveBasePathAdapter::pathPointCallback, this);
    revision_sub_ = nh_.subscribe("/path_manager/path_revision", 1,
        &MoveBasePathAdapter::pathRevisionCallback, this);
    path_finished_pub_ = nh_.advertise<std_msgs::Bool>("/path_finished", 1, true);
    next_point_client_ = nh_.serviceClient<std_srvs::Trigger>("/next_point");
    timeout_timer_ = nh_.createTimer(ros::Duration(0.1),
        &MoveBasePathAdapter::timeoutCallback, this);

    std_msgs::Bool finished;
    finished.data = false;
    path_finished_pub_.publish(finished);

    ROS_INFO("[move_base_path_adapter] waiting for move_base action server");
    if (!action_client_.waitForServer(ros::Duration(10.0))) {
      ROS_ERROR("[move_base_path_adapter] move_base action server is unavailable");
      return false;
    }
    ROS_INFO("[move_base_path_adapter] ready; YAML path points will be sent in %s",
             goal_frame_.c_str());
    return true;
  }

 private:
  void pathRevisionCallback(const std_msgs::UInt32::ConstPtr& msg) {
    if (msg->data == path_revision_) return;
    path_revision_ = msg->data;
    action_client_.cancelAllGoals();
    has_target_ = false;
    waiting_for_next_point_ = false;

    std_msgs::Bool finished;
    finished.data = false;
    path_finished_pub_.publish(finished);
    ROS_INFO("[move_base_path_adapter] switched to path revision %u", path_revision_);
  }

  void pathPointCallback(const path_manager::PathPoint::ConstPtr& msg) {
    const bool target_changed = !has_target_ ||
        std::abs(target_.x - msg->x) >= 1e-6 ||
        std::abs(target_.y - msg->y) >= 1e-6 ||
        std::abs(target_.tolerance - msg->tolerance) >= 1e-6;
    if (!target_changed) return;

    target_ = *msg;
    has_target_ = true;
    waiting_for_next_point_ = false;
    goal_start_time_ = ros::Time::now();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = goal_start_time_;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.pose.position.x = target_.x;
    goal.target_pose.pose.position.y = target_.y;
    goal.target_pose.pose.orientation.w = 1.0;

    std_msgs::Bool finished;
    finished.data = false;
    path_finished_pub_.publish(finished);
    action_client_.sendGoal(goal,
        [this](const actionlib::SimpleClientGoalState& state,
               const move_base_msgs::MoveBaseResultConstPtr& result) {
          goalDoneCallback(state, result);
        });
    ROS_INFO("[move_base_path_adapter] sent waypoint (%.3f, %.3f), has_next=%s",
             target_.x, target_.y, target_.has_next ? "true" : "false");
  }

  void goalDoneCallback(const actionlib::SimpleClientGoalState& state,
                        const move_base_msgs::MoveBaseResultConstPtr&) {
    if (!has_target_) return;
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("[move_base_path_adapter] waypoint failed: %s; holding route",
                state.toString().c_str());
      has_target_ = false;
      waiting_for_next_point_ = true;
      return;
    }

    if (!target_.has_next) {
      has_target_ = false;
      std_msgs::Bool finished;
      finished.data = true;
      path_finished_pub_.publish(finished);
      ROS_INFO("[move_base_path_adapter] ====== path completed ======");
      return;
    }

    std_srvs::Trigger next_point;
    if (!next_point_client_.call(next_point) || !next_point.response.success) {
      ROS_ERROR("[move_base_path_adapter] cannot advance to next waypoint: %s",
                next_point.response.message.c_str());
      has_target_ = false;
      waiting_for_next_point_ = true;
      return;
    }
    has_target_ = false;
    waiting_for_next_point_ = true;
  }

  void timeoutCallback(const ros::TimerEvent&) {
    if (!has_target_ || goal_start_time_.isZero()) return;
    if ((ros::Time::now() - goal_start_time_).toSec() <= goal_timeout_sec_) return;

    ROS_ERROR("[move_base_path_adapter] waypoint timed out after %.1f s; cancelling goal",
              goal_timeout_sec_);
    action_client_.cancelGoal();
    has_target_ = false;
    waiting_for_next_point_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  ros::Subscriber path_points_sub_;
  ros::Subscriber revision_sub_;
  ros::Publisher path_finished_pub_;
  ros::ServiceClient next_point_client_;
  ros::Timer timeout_timer_;

  path_manager::PathPoint target_;
  bool has_target_;
  bool waiting_for_next_point_;
  uint32_t path_revision_;
  ros::Time goal_start_time_;
  double goal_timeout_sec_;
  std::string goal_frame_;
};

}  // namespace path_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_base_path_adapter_node");
  path_manager::MoveBasePathAdapter adapter;
  if (!adapter.init()) return 1;
  ros::spin();
  return 0;
}
