/**
 * @file mission_controller.cpp
 * @brief 任务状态机节点实现（事件驱动，非阻塞）
 *
 * 管理比赛全流程：
 *   IDLE → GOTO_NURSE → SCAN_QR → GOTO_BED_A → POSITION_IN_CIRCLE →
 *   SCAN_BARCODE → OPEN_BOX → PLACE_MEDICINE → VOICE →
 *   GOTO_BED_B → ... → RETURN_HOME → STOP
 *
 * 设计原则：
 *   - 10Hz 定时器驱动状态机
 *   - 入口动作仅执行一次（action_initiated_ 保护）
 *   - 状态迁移条件由话题回调设置原子标志
 *   - 无阻塞等待 — 所有 poll 循环已消除
 */

#include "robot_navigation/mission_controller.h"

#include <cmath>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
MissionController::MissionController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , current_state_(State::IDLE)
    , mission_started_(false)
    , estop_latched_(false)
    , mission_completed_(false)
    , action_initiated_(false)
    , qr_received_(false)
    , barcode_bed1_received_(false)
    , barcode_bed3_received_(false)
    , path_finished_received_(false)
    , fine_tuning_done_received_(false)
    , medicine_release_done_received_(false)
    , odom_received_(false)
    , home_arrived_(false)
    , skipBedsideScan_(true)
    , bedsideScanSkipWaitSec_(1.0)
    , stage_timeout_sec_(30.0)
    , mission_timeout_sec_(180.0)
    , state_machine_rate_hz_(10.0)
    , path_nurse_station_("nurse_station")
    , path_bed1_circle_("bed1_circle")
    , path_bed3_circle_("bed3_circle")
    , path_home_("HOME")
    , home_hold_sec_(5.0)
    , voice_timing_active_(false)
    , stage_skip_attempted_(false)
    , competition_mode_(true)
    , audio_video_enabled_(false)
    , audio_ready_(false)
    , audio_active_(false)
    , scan_session_id_(0)
    , start_auth_token_(0x5A17)
    , chassis_locked_(false)
    , bed_verified_(false)
    , enable_vl53_circle_check_(true)
    , require_all_vl53_ranges_(true)
    , vl53_circle_timeout_sec_(0.5)
    , vl53_min_clearance_m_(0.05)
    , base_projection_radius_m_(0.20)
    , full_projection_radius_m_(0.25)
    , circle_boundary_margin_m_(0.02)
    , front_range_topic_("/front/range")
    , left_range_topic_("/left/range")
    , right_range_topic_("/right/range")
{
  // ── 参数读取 ──
  // mission_params.yaml keeps task settings under "mission". Accept legacy
  // flat private parameters as well (standalone rosrun/test deployments).
  const ros::NodeHandle mission_nh = pnh_.hasParam("mission")
      ? ros::NodeHandle(pnh_, "mission") : pnh_;
  mission_nh.param<double>("stage_timeout_sec", stage_timeout_sec_, 30.0);
  stage_timeout_default_sec_ = stage_timeout_sec_;
  mission_nh.param<double>("navigation_stage_timeout_sec", navigation_stage_timeout_sec_, 75.0);
  mission_nh.param<double>("mission_timeout_sec", mission_timeout_sec_, 180.0);
  mission_nh.param<double>("state_machine_rate_hz", state_machine_rate_hz_, 10.0);
  mission_nh.param("odom_timeout_sec", odom_timeout_sec_, 0.5);
  pnh_.param<bool>("competition_mode", competition_mode_, true);
  pnh_.param<bool>("audio_video_enabled", audio_video_enabled_, false);
  int start_token = static_cast<int>(start_auth_token_); pnh_.param<int>("start_auth_token", start_token, start_token); start_auth_token_ = static_cast<uint32_t>(start_token);
  pnh_.param<bool>("enable_vl53_circle_check", enable_vl53_circle_check_, true);
  pnh_.param<bool>("require_all_vl53_ranges", require_all_vl53_ranges_, true);
  pnh_.param<double>("vl53_circle_timeout_sec", vl53_circle_timeout_sec_, 0.5);
  pnh_.param<double>("vl53_min_clearance_m", vl53_min_clearance_m_, 0.05);
  pnh_.param<double>("base_projection_radius_m", base_projection_radius_m_, 0.20);
  pnh_.param<double>("full_projection_radius_m", full_projection_radius_m_, 0.25);
  pnh_.param<double>("circle_boundary_margin_m", circle_boundary_margin_m_, 0.02);
  pnh_.param<std::string>("front_range_topic", front_range_topic_, "/front/range");
  pnh_.param<std::string>("left_range_topic", left_range_topic_, "/left/range");
  pnh_.param<std::string>("right_range_topic", right_range_topic_, "/right/range");
  vl53_circle_timeout_sec_ = std::max(0.01, vl53_circle_timeout_sec_);
  vl53_min_clearance_m_ = std::max(0.0, vl53_min_clearance_m_);
  base_projection_radius_m_ = std::max(0.0, base_projection_radius_m_);
  full_projection_radius_m_ = std::max(base_projection_radius_m_, full_projection_radius_m_);
  circle_boundary_margin_m_ = std::max(0.0, circle_boundary_margin_m_);

  mission_nh.param<std::string>("path_nurse_station", path_nurse_station_, "nurse_station");
  mission_nh.param<std::string>("path_bed1_circle", path_bed1_circle_, "bed1_circle");
  mission_nh.param<std::string>("path_bed3_circle", path_bed3_circle_, "bed3_circle");
  mission_nh.param<std::string>("path_bed1_to_bed3", path_bed1_to_bed3_, "bed1_to_bed3");
  mission_nh.param<std::string>("path_bed3_to_bed1", path_bed3_to_bed1_, "bed3_to_bed1");
  mission_nh.param<std::string>("path_bed1_to_home", path_bed1_to_home_, "bed1_to_home");
  mission_nh.param<std::string>("path_bed3_to_home", path_bed3_to_home_, "bed3_to_home");
  mission_nh.param<std::string>("path_home", path_home_, "HOME");

  // 圆圈参数
  mission_nh.param<double>("circle_bed1_x", circle_bed1_.center_x, 0.0);
  mission_nh.param<double>("circle_bed1_y", circle_bed1_.center_y, 0.0);
  mission_nh.param<double>("circle_bed1_radius", circle_bed1_.radius, 0.3);

  mission_nh.param<double>("circle_bed3_x", circle_bed3_.center_x, 0.0);
  mission_nh.param<double>("circle_bed3_y", circle_bed3_.center_y, 0.0);
  mission_nh.param<double>("circle_bed3_radius", circle_bed3_.radius, 0.3);

  mission_nh.param<double>("home_hold_sec", home_hold_sec_, 5.0);

  // 起始区多边形顶点（从参数加载）
  XmlRpc::XmlRpcValue home_vertices;
  if (mission_nh.getParam("home_zone_vertices", home_vertices) &&
      home_vertices.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < home_vertices.size(); ++i) {
      Point2D pt;
      pt.x = static_cast<double>(home_vertices[i]["x"]);
      pt.y = static_cast<double>(home_vertices[i]["y"]);
      home_zone_vertices_.push_back(pt);
    }
  }
}

bool MissionController::init() {
  pnh_.param<bool>("skip_bedside_scan", skipBedsideScan_, true);
  pnh_.param<double>("bedside_scan_skip_wait_s", bedsideScanSkipWaitSec_, 1.0);
  bedsideScanSkipWaitSec_ = std::max(0.0, bedsideScanSkipWaitSec_);
  // ── 订阅 ──
  qr_result_sub_     = nh_.subscribe("/qr_result", 1,
                                     &MissionController::qrResultCallback, this);
  barcode_bed1_sub_  = nh_.subscribe("/barcode_bed1", 1,
                                     &MissionController::barcodeBed1Callback, this);
  barcode_bed3_sub_  = nh_.subscribe("/barcode_bed3", 1,
                                     &MissionController::barcodeBed3Callback, this);
  start_signal_sub_  = nh_.subscribe("/start_signal/physical", 1,
                                     &MissionController::startSignalCallback, this);
  emergency_stop_sub_ = nh_.subscribe(
      "/emergency_stop", 10,
      &MissionController::emergencyStopCallback, this);
  emergency_reset_sub_ = nh_.subscribe(
      "/emergency_stop/reset", 2,
      &MissionController::emergencyResetCallback, this);
  odom_sub_          = nh_.subscribe("/odom", 10,
                                     &MissionController::odomCallback, this);
  front_range_sub_   = nh_.subscribe(front_range_topic_, 10,
                                     &MissionController::frontRangeCallback, this);
  left_range_sub_    = nh_.subscribe(left_range_topic_, 10,
                                     &MissionController::leftRangeCallback, this);
  right_range_sub_   = nh_.subscribe(right_range_topic_, 10,
                                     &MissionController::rightRangeCallback, this);
  path_finished_sub_  = nh_.subscribe("/path_finished", 10,
                                     &MissionController::pathFinishedCallback, this);

  fine_tuning_done_sub_ = nh_.subscribe("/fine_tuning_done", 1,
                                        &MissionController::fineTuningDoneCallback, this);
  fine_tuning_failed_sub_ = nh_.subscribe("/fine_tuning_failed", 1,
                                        &MissionController::fineTuningFailedCallback, this);

  medicine_release_done_sub_ = nh_.subscribe("/medicine_release_done", 1, &MissionController::medicineReleaseDoneCallback, this);
  audio_ready_sub_ = nh_.subscribe("/Ready", 1, &MissionController::audioReadyCallback, this);
  audio_active_sub_ = nh_.subscribe("/audio_chat/active", 1, &MissionController::audioActiveCallback, this);

  // ── 发布 ──
  mission_finished_pub_ = nh_.advertise<std_msgs::Bool>("/mission_finished", 1, true);
  mission_timeout_pub_  = nh_.advertise<std_msgs::Bool>("/mission_timeout", 1, true);
  stop_all_pub_         = nh_.advertise<std_msgs::Empty>("/stop_all", 1, true);
  cancel_pub_           = nh_.advertise<std_msgs::Empty>("/mission/cancel", 1);
  actions_enabled_pub_  = nh_.advertise<std_msgs::Bool>("/mission/actions_enabled", 1, true);
  display_text_pub_     = nh_.advertise<std_msgs::String>("/robot_display", 1, true);
  chassis_lock_pub_     = nh_.advertise<std_msgs::Bool>("/chassis_lock", 1, true);
  scan_target_bed_pub_ = nh_.advertise<std_msgs::Int8>("/barcode_scan/target_bed", 1, true);
  scan_session_pub_ = nh_.advertise<std_msgs::UInt32>("/barcode_scan/session", 1, true);

  // ── 服务客户端 ──
  ROS_INFO("[mission_controller] 等待服务就绪...");
  const ros::Duration service_timeout(10.0);

  if (!ros::service::waitForService("/select_path", service_timeout)) {
    ROS_WARN("[mission_controller] /select_path 未就绪");
  }
  path_select_client_ = nh_.serviceClient<path_manager::SelectPath>("/select_path");

  if (!ros::service::waitForService("/fine_tuning/start", service_timeout)) {
    ROS_WARN("[mission_controller] /fine_tuning/start 未就绪");
  }
  fine_tuning_client_ = nh_.serviceClient<std_srvs::Trigger>("/fine_tuning/start");

  if (!ros::service::waitForService("/open_medicine_box", service_timeout)) {
    ROS_WARN("[mission_controller] /open_medicine_box 未就绪");
  }
  open_box_client_ = nh_.serviceClient<robot_navigation::OpenMedicineBox>("/open_medicine_box");

  if (!ros::service::waitForService("/arm_place_medicine", service_timeout)) {
    ROS_WARN("[mission_controller] /arm_place_medicine 未就绪");
  }
  arm_place_client_ = nh_.serviceClient<arm_and_gripper::ArmPlaceMedicine>("/arm_place_medicine");

  if (!ros::service::waitForService("/speak", service_timeout)) {
    ROS_WARN("[mission_controller] /speak 未就绪");
  }
  speak_client_ = nh_.serviceClient<robot_navigation::Speak>("/speak");

  // ── 定时器 ──
  // 全局任务定时器（3分钟），先不启动，收到 start_signal 后再启动
  mission_timer_ = nh_.createTimer(ros::Duration(mission_timeout_sec_),
                                   &MissionController::missionTimerCallback,
                                   this, true, false);  // oneshot, 初始不启动

  // 状态机主循环
  const double period = 1.0 / std::max(1.0, state_machine_rate_hz_);
  state_machine_timer_ = nh_.createTimer(ros::Duration(period),
                                         &MissionController::stateMachineTimerCallback,
                                         this);

  // ── 初始显示 ──
  {
    std_msgs::String init_display;
    init_display.data = "等待启动信号...";
    display_text_pub_.publish(init_display);
  }

  ROS_INFO("[mission_controller] 初始化完成，当前状态: %s",
           stateToString(current_state_).c_str());
  ROS_INFO("[mission_controller]   阶段超时: %.0f s, 全局超时: %.0f s",
           stage_timeout_sec_, mission_timeout_sec_);
  ROS_INFO("[mission_controller]   投影校验: %s, 车体半径=%.3f m, 整机半径=%.3f m, VL53超时=%.2f s",
           enable_vl53_circle_check_ ? "enabled" : "disabled",
           base_projection_radius_m_, full_projection_radius_m_, vl53_circle_timeout_sec_);
  ROS_INFO("[mission_controller]   等待 /start_signal 信号...");

  return true;
}

// ── 回调: QR 结果 ──────────────────────────────────────────
void MissionController::qrResultCallback(const robot_navigation::QrResult::ConstPtr& msg) {
  if (current_state_ != State::SCAN_QR) return;
  if (qr_received_) return;

  qr_result_ = *msg;
  qr_received_ = true;

  ROS_INFO("[mission_controller] 收到 QR 结果: first_bed=%d, first_box=%d, "
           "second_bed=%d, second_box=%d",
           msg->first_bed, msg->first_box,
           msg->second_bed, msg->second_box);
}

void MissionController::frontRangeCallback(const sensor_msgs::Range::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(range_mutex_);
  front_range_ = *msg;
  front_range_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

void MissionController::leftRangeCallback(const sensor_msgs::Range::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(range_mutex_);
  left_range_ = *msg;
  left_range_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

void MissionController::rightRangeCallback(const sensor_msgs::Range::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(range_mutex_);
  right_range_ = *msg;
  right_range_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

// ── 回调: 条形码 bed1 ──────────────────────────────────────
void MissionController::barcodeBed1Callback(const std_msgs::String::ConstPtr& msg) {
  if (current_state_ != State::SCAN_BARCODE_A && current_state_ != State::SCAN_BARCODE_B) return;
  if (!isValidBarcode(msg->data)) { ROS_WARN("[mission_controller] 忽略空或非法的1床条码"); return; }
  barcode_bed1_value_ = msg->data;
  barcode_bed1_received_ = true;
  barcode_display_line1_ = "1床条码: " + msg->data;
  updatePersistentBarcodeDisplay();
}

// ── 回调: 条形码 bed3 ──────────────────────────────────────
void MissionController::barcodeBed3Callback(const std_msgs::String::ConstPtr& msg) {
  if (current_state_ != State::SCAN_BARCODE_A && current_state_ != State::SCAN_BARCODE_B) return;
  if (!isValidBarcode(msg->data)) { ROS_WARN("[mission_controller] 忽略空或非法的3床条码"); return; }
  barcode_bed3_value_ = msg->data;
  barcode_bed3_received_ = true;
  barcode_display_line2_ = "3床条码: " + msg->data;
  updatePersistentBarcodeDisplay();
}

// ── 回调: 启动信号 ─────────────────────────────────────────
void MissionController::startSignalCallback(const std_msgs::UInt32::ConstPtr& msg) {
  if (msg->data != start_auth_token_) { ROS_WARN("unauthenticated start signal"); return; }
  if (estop_latched_) { ROS_WARN("[mission_controller] start ignored while ESTOP is latched"); return; }
  if (mission_started_ && !mission_completed_) return;
  if (service_future_.valid()) {
    if (service_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      ROS_WARN("[mission_controller] start ignored until cancelled service has returned");
      return;
    }
    service_future_.get();  // Discard a result from the cancelled generation.
  }
  // 如果上次任务已完成/失败/超时，允许重新启动
  if (mission_completed_.load()) {
    ROS_INFO("[mission_controller] 上次任务已结束，准备重新启动");
    mission_started_.store(false);
  }

  if (mission_started_.load()) return;

  resetMissionState();
  ++mission_generation_;
  std_msgs::Bool enabled;
  enabled.data = true;
  actions_enabled_pub_.publish(enabled);
  unlockChassis();
  mission_started_.store(true);
  enterState(State::GOTO_NURSE);

  // 启动全局超时定时器
  mission_timer_.start();

  ROS_INFO("[mission_controller] ====== 收到启动信号！开始任务 ======");

  std_msgs::String display;
  display.data = "任务开始";
  display_text_pub_.publish(display);
}

void MissionController::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data || estop_latched_) return;
  ROS_ERROR("[mission_controller] physical/software ESTOP: cancelling mission and navigation");
  estop_latched_ = true;
  cancelMissionActions();
  mission_started_.store(false);
  mission_completed_.store(true);
  mission_timer_.stop();
  action_initiated_ = false;
  current_state_ = State::STOP;
  chassis_locked_ = true;
  std_msgs::Bool lock; lock.data = true; chassis_lock_pub_.publish(lock);
  std_msgs::String display; display.data = "急停: 任务已停止"; display_text_pub_.publish(display);
}

void MissionController::emergencyResetCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data || !estop_latched_) return;
  estop_latched_ = false;
  mission_started_.store(false);
  mission_completed_.store(false);
  current_state_ = State::IDLE;
  chassis_locked_ = true;
  std_msgs::Bool lock; lock.data = true; chassis_lock_pub_.publish(lock);
  std_msgs::String display; display.data = "急停已复位，等待启动"; display_text_pub_.publish(display);
  ROS_WARN("[mission_controller] ESTOP reset accepted by upper safety policy");
}

// ── 回调: 里程计 ───────────────────────────────────────────
void MissionController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  const double age = msg->header.stamp.isZero() ? 0.0 :
      (ros::Time::now() - msg->header.stamp).toSec();
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  const auto& v = msg->twist.twist;
  odom_received_ = age >= -0.05 && age <= odom_timeout_sec_ &&
      std::isfinite(p.x) && std::isfinite(p.y) &&
      std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
      (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) > 1e-9 &&
      std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z);
  if (!odom_received_) return;
  current_odom_ = *msg;
  last_odom_time_ = ros::SteadyTime::now();
}

// ── 回调: 路径完成信号 ─────────────────────────────────────
void MissionController::pathFinishedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    path_finished_received_.store(true);
  }
}

// ── 回调: 微调完成信号 ─────────────────────────────────────
void MissionController::fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && (current_state_ == State::POSITION_IN_CIRCLE_A ||
                    current_state_ == State::POSITION_IN_CIRCLE_B)) {
    fine_tuning_done_received_.store(true);
    ROS_INFO("[mission_controller] 收到微调完成信号");
  }
}

void MissionController::fineTuningFailedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data && (current_state_ == State::POSITION_IN_CIRCLE_A ||
                    current_state_ == State::POSITION_IN_CIRCLE_B)) {
    enterFailedState("床旁微调失败，保持停车");
  }
}

void MissionController::medicineReleaseDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) { medicine_release_done_received_.store(true); ROS_INFO("[mission_controller] 收到放药完成信号"); }
}

void MissionController::audioReadyCallback(const std_msgs::Int32::ConstPtr& msg) { audio_ready_ = msg->data == 1; }
void MissionController::audioActiveCallback(const std_msgs::Bool::ConstPtr& msg) { audio_active_ = msg->data; }

// ── 全局超时回调 ───────────────────────────────────────────
void MissionController::missionTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!mission_started_ || mission_completed_) return;
  ROS_ERROR("[mission_controller] ====== 全局任务超时！(%.0f s) ======",
            mission_timeout_sec_);

  // Bedside return routes are valid only from their specified origins.
  // An arbitrary mid-route timeout must cancel and stop in place.
  enterTimeoutState();
}

// ── 状态机主循环（10Hz，非阻塞） ─────────────────────────
void MissionController::stateMachineTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!mission_started_.load() || mission_completed_.load()) return;
  processState();
}

// ── 状态处理（事件驱动） ─────────────────────────────────
void MissionController::processState() {
  // 终态直接返回
  switch (current_state_) {
    case State::IDLE:
    case State::STOP:
    case State::TIMEOUT:
    case State::FAILED:
      return;
    default:
      break;
  }

  // ── 超时检查 ──
  if (isStageTimedOut()) {
    if (competition_mode_) {
    ROS_ERROR("[mission_controller] 比赛模式阶段超时，fail-closed: %s", stateToString(current_state_).c_str());
    enterFailedState("阶段超时: " + stateToString(current_state_));
    return;
  }

  if (!stage_skip_attempted_) {
      stage_skip_attempted_ = true;
      stage_start_time_ = ros::Time::now();
      switch (current_state_) {
        case State::SCAN_QR: enterState(State::GOTO_BED_A); return;
        case State::POSITION_IN_CIRCLE_A: enterState(State::SCAN_BARCODE_A); return;
        case State::SCAN_BARCODE_A: enterState(State::OPEN_BOX_A); return;
        case State::POSITION_IN_CIRCLE_B: enterState(State::SCAN_BARCODE_B); return;
        case State::SCAN_BARCODE_B: enterState(State::OPEN_BOX_B); return;
        case State::HOME_CHECK: enterState(State::STOP); return;
        default: break;
      }
  }

    ROS_ERROR("[mission_controller] 状态 %s 阶段超时 (%.0f s)，进入失败",
              stateToString(current_state_).c_str(), stage_timeout_sec_);
    enterFailedState("阶段超时: " + stateToString(current_state_));
    return;
  }

  if (pollServiceCall()) return;

  // ── 首次进入：执行入口动作 ──
  if (!action_initiated_) {
    action_initiated_ = true;

    switch (current_state_) {
      case State::GOTO_NURSE:         actionGotoNurse();          break;
      case State::SCAN_QR:            actionScanQr();             break;
      case State::GOTO_BED_A:         actionGotoBedA();           break;
      case State::POSITION_IN_CIRCLE_A: actionPositionInCircleA(); break;
      case State::SCAN_BARCODE_A:     actionScanBarcodeA();       break;
      case State::OPEN_BOX_A:         actionOpenBoxA();           break;
      case State::PLACE_MEDICINE_A:   actionPlaceMedicineA();     break;
      case State::VOICE_A:            actionVoiceA();             break;
      case State::GOTO_BED_B:         actionGotoBedB();           break;
      case State::POSITION_IN_CIRCLE_B: actionPositionInCircleB(); break;
      case State::SCAN_BARCODE_B:     actionScanBarcodeB();       break;
      case State::OPEN_BOX_B:         actionOpenBoxB();           break;
      case State::PLACE_MEDICINE_B:   actionPlaceMedicineB();     break;
      case State::VOICE_B:            actionVoiceB();             break;
      case State::RETURN_HOME:        actionReturnHome();         break;
      case State::HOME_CHECK:         actionHomeCheck();          break;
      default: break;
    }
    return;
  }

  // ── 检查完成条件，驱动状态迁移 ──
  switch (current_state_) {
    case State::GOTO_NURSE:
    case State::GOTO_BED_A:
    case State::GOTO_BED_B:
    case State::RETURN_HOME:
      if (checkGotoComplete()) return;  // 内部完成状态切换
      break;

    case State::SCAN_QR:
      if (checkScanQrComplete()) return;
      break;

    case State::POSITION_IN_CIRCLE_A:
    case State::POSITION_IN_CIRCLE_B:
      if (checkPositionInCircleComplete()) return;
      break;

    case State::SCAN_BARCODE_A:
    case State::SCAN_BARCODE_B:
      if (checkScanBarcodeComplete()) return;
      break;

    case State::HOME_CHECK:
      if (checkHomeCheckComplete()) return;
      break;

    // Service results are consumed by pollServiceCall without blocking callbacks.
    default:
      break;
  }
}

// ── 进入新状态 ─────────────────────────────────────────────
void MissionController::enterState(State new_state) {
  current_state_ = new_state;
  const bool navigating = new_state == State::GOTO_NURSE ||
      new_state == State::GOTO_BED_A || new_state == State::GOTO_BED_B ||
      new_state == State::RETURN_HOME;
  stage_timeout_sec_ = navigating ? navigation_stage_timeout_sec_ : stage_timeout_default_sec_;
  action_initiated_ = false;
  stage_start_time_ = ros::Time::now();
  stage_skip_attempted_ = false;
  ROS_INFO("[mission_controller] >>> 进入状态: %s", stateToString(new_state).c_str());
}

// ======================================================================
//  入口动作（每个动作只执行一次）
// ======================================================================

void MissionController::actionGotoNurse() {
  ROS_INFO("[mission_controller] 动作: 导航到护士台");
  path_finished_received_.store(false);
  callSelectPath(path_nurse_station_);
}

void MissionController::actionScanQr() {
  ++scan_session_id_; std_msgs::UInt32 session; session.data = scan_session_id_; scan_session_pub_.publish(session);
  ROS_INFO("[mission_controller] 动作: 等待二维码识别");
  qr_received_.store(false);
  // 无额外动作，等待 qrResultCallback 设置标志
}

void MissionController::actionGotoBedA() {
  int bed = qr_result_.first_bed;
  std::string path = (bed == 1) ? path_bed1_circle_ : path_bed3_circle_;
  ROS_INFO("[mission_controller] 动作: 导航到 %d 床 (%s)", bed, path.c_str());
  path_finished_received_.store(false);
  callSelectPath(path);
}

void MissionController::actionPositionInCircleA() {
  ROS_INFO("[mission_controller] 动作: 启动微调（A床）");
  fine_tuning_done_received_.store(false);
  medicine_release_done_received_.store(false);
  callFineTuningStart();
}

void MissionController::actionScanBarcodeA() {
  const int bed = qr_result_.first_bed;
  std_msgs::Int8 target; target.data = static_cast<int8_t>(bed); scan_target_bed_pub_.publish(target);
  if (skipBedsideScan_) {
    bedsideScanSkipStart_ = ros::Time::now();
    ROS_WARN("[mission_controller] 无摄像头：跳过 %d 床条形码扫描，等待 %.1f s；药箱仍按护士台 first_box=%d",
             bed, bedsideScanSkipWaitSec_, qr_result_.first_box);
    return;
  }
  ROS_INFO("[mission_controller] 动作: 等待 %d 床条形码", bed);
  if (bed == 1) barcode_bed1_received_.store(false);
  else barcode_bed3_received_.store(false);
}

void MissionController::actionOpenBoxA() {
  ROS_INFO("[mission_controller] 准备执行放药，药箱=%d（护士台二维码指定）", qr_result_.first_box);
  lockChassis();
  callOpenMedicineBox(qr_result_.first_box);
}

void MissionController::actionPlaceMedicineA() {
  int bed = qr_result_.first_bed;
  ROS_INFO("[mission_controller] 动作: 放置药品到 %d 床（底盘已锁死）", bed);
  callArmPlaceMedicine(bed, qr_result_.first_box);
}

void MissionController::actionVoiceA() {
  int bed = qr_result_.first_bed;

  // ── 5秒内语音播报检查（P2-1） ──
  if (voice_timing_active_) {
    double elapsed = (ros::Time::now() - medicine_placed_time_).toSec();
    if (elapsed > 5.0) {
      ROS_WARN("[mission_controller] ⚠ 语音播报超时 (%.1f s > 5s)，不得分但继续流程", elapsed);
    } else {
      ROS_INFO("[mission_controller] ✅ 语音播报在 %.1f 秒内（5秒内有效）", elapsed);
    }
    voice_timing_active_ = false;
  }

  if (audio_video_enabled_ && (!audio_ready_ || !audio_active_)) {
    enterFailedState("音视频链路未就绪"); return;
  }
  std::string text = std::to_string(bed) + "床病人请取药";
  ROS_INFO("[mission_controller] 动作: 播报 \"%s\"", text.c_str());
  callSpeak(text);
}

void MissionController::actionGotoBedB() {
  int bed = qr_result_.second_bed;
  std::string path = (bed == 1) ? path_bed3_to_bed1_ : path_bed1_to_bed3_;
  ROS_INFO("[mission_controller] 动作: 导航到 %d 床 (%s)", bed, path.c_str());
  path_finished_received_.store(false);
  callSelectPath(path);
}

void MissionController::actionPositionInCircleB() {
  ROS_INFO("[mission_controller] 动作: 启动微调（B床）");
  fine_tuning_done_received_.store(false);
  medicine_release_done_received_.store(false);
  callFineTuningStart();
}

void MissionController::actionScanBarcodeB() {
  const int bed = qr_result_.second_bed;
  std_msgs::Int8 target; target.data = static_cast<int8_t>(bed); scan_target_bed_pub_.publish(target);
  if (skipBedsideScan_) {
    bedsideScanSkipStart_ = ros::Time::now();
    ROS_WARN("[mission_controller] 无摄像头：跳过 %d 床条形码扫描，等待 %.1f s；药箱仍按护士台 second_box=%d",
             bed, bedsideScanSkipWaitSec_, qr_result_.second_box);
    return;
  }
  ROS_INFO("[mission_controller] 动作: 等待 %d 床条形码", bed);
  if (bed == 1) barcode_bed1_received_.store(false);
  else barcode_bed3_received_.store(false);
}

void MissionController::actionOpenBoxB() {
  ROS_INFO("[mission_controller] 准备执行放药，药箱=%d（护士台二维码指定）", qr_result_.second_box);
  lockChassis();
  callOpenMedicineBox(qr_result_.second_box);
}

void MissionController::actionPlaceMedicineB() {
  int bed = qr_result_.second_bed;
  ROS_INFO("[mission_controller] 动作: 放置药品到 %d 床（底盘已锁死）", bed);
  callArmPlaceMedicine(bed, qr_result_.second_box);
}

void MissionController::actionVoiceB() {
  int bed = qr_result_.second_bed;

  if (voice_timing_active_) {
    double elapsed = (ros::Time::now() - medicine_placed_time_).toSec();
    if (elapsed > 5.0) {
      ROS_WARN("[mission_controller] ⚠ 语音播报超时 (%.1f s > 5s)，不得分但继续流程", elapsed);
    } else {
      ROS_INFO("[mission_controller] ✅ 语音播报在 %.1f 秒内（5秒内有效）", elapsed);
    }
    voice_timing_active_ = false;
  }

  if (audio_video_enabled_ && (!audio_ready_ || !audio_active_)) {
    enterFailedState("音视频链路未就绪"); return;
  }
  std::string text = std::to_string(bed) + "床病人请取药";
  ROS_INFO("[mission_controller] 动作: 播报 \"%s\"", text.c_str());
  callSpeak(text);
}

void MissionController::actionReturnHome() {
  // 根据最后访问的病床选择专用返回路径（更高效）
  int last_bed = qr_result_.second_bed;
  std::string home_path = (last_bed == 1) ? path_bed1_to_home_ :
                          (last_bed == 3) ? path_bed3_to_home_ : path_home_;

  ROS_INFO("[mission_controller] 动作: 返回起始区 (使用路径: %s)", home_path.c_str());
  path_finished_received_.store(false);

  // HOME starts at P7; falling back to it from a bedside could cut through
  // the nurse station. A missing dedicated route must stop the mission.
  callSelectPath(home_path);
}

void MissionController::actionHomeCheck() {
  ROS_INFO("[mission_controller] 动作: 检测是否在起始区内");
  home_arrived_ = false;
}

// ======================================================================
//  完成条件检查
// ======================================================================

bool MissionController::checkGotoComplete() {
  if (path_finished_received_.load()) {
    // 根据当前状态决定下一状态
    switch (current_state_) {
      case State::GOTO_NURSE:  enterState(State::SCAN_QR);            break;
      case State::GOTO_BED_A:  enterState(State::POSITION_IN_CIRCLE_A); break;
      case State::GOTO_BED_B:  enterState(State::POSITION_IN_CIRCLE_B); break;
      case State::RETURN_HOME: enterState(State::HOME_CHECK);          break;
      default: break;
    }
    return true;
  }
  return false;
}

bool MissionController::checkScanQrComplete() {
  if (qr_received_.load()) {
    if ((qr_result_.first_bed != 1 && qr_result_.first_bed != 3) ||
        (qr_result_.first_box != 1 && qr_result_.first_box != 3) ||
        (qr_result_.second_bed != 1 && qr_result_.second_bed != 3) ||
        (qr_result_.second_box != 1 && qr_result_.second_box != 3) ||
        qr_result_.second_bed == qr_result_.first_bed ||
        qr_result_.second_box == qr_result_.first_box) {
      enterFailedState("QR 结果为空");
      return true;
    }
    enterState(State::GOTO_BED_A);
    return true;
  }
  return false;
}

bool MissionController::checkPositionInCircleComplete() {
  if (fine_tuning_done_received_.load()) {
    const bool is_first_visit = current_state_ == State::POSITION_IN_CIRCLE_A;
    const int bed = is_first_visit ? qr_result_.first_bed : qr_result_.second_bed;
    const CircleDef& circle = (bed == 1) ? circle_bed1_ : circle_bed3_;
    // The bed circle excludes the arm: only the complete chassis footprint is
    // required to be inside it. VL53 freshness/clearance is checked as well.
    if (!isProjectionInsideCircle(circle, false) || !areVl53RangesValid()) {
      return false;
    }
    if (is_first_visit) {
      enterState(State::SCAN_BARCODE_A);
    } else {
      enterState(State::SCAN_BARCODE_B);
    }
    return true;
  }
  return false;
}

bool MissionController::checkScanBarcodeComplete() {
  if (skipBedsideScan_) {
    if ((ros::Time::now() - bedsideScanSkipStart_).toSec() < bedsideScanSkipWaitSec_) return false;
    ROS_WARN("[mission_controller] 条形码扫描已跳过（无摄像头），继续执行护士台指定药箱");
    if (current_state_ == State::SCAN_BARCODE_A) enterState(State::OPEN_BOX_A);
    else enterState(State::OPEN_BOX_B);
    return true;
  }
  // 用 load() 获取原子值
  int bed = (current_state_ == State::SCAN_BARCODE_A)
      ? qr_result_.first_bed : qr_result_.second_bed;
  bool got_it = (bed == 1) ? barcode_bed1_received_.load()
                           : barcode_bed3_received_.load();

  if (got_it && isValidBarcode(bed == 1 ? barcode_bed1_value_ : barcode_bed3_value_)) {
    // ── 床号验证（P2-2）：确认在正确的病床 ──
    int expected_bed = (current_state_ == State::SCAN_BARCODE_A)
        ? qr_result_.first_bed : qr_result_.second_bed;
    if (!verifyBedNumber(expected_bed)) {
      ROS_ERROR("[mission_controller] 床号校验失败！可能走错了病床");
      enterFailedState("床号校验失败: 期望" + std::to_string(expected_bed) + "床");
      return true;
    }

    if (current_state_ == State::SCAN_BARCODE_A) {
      enterState(State::OPEN_BOX_A);
    } else {
      enterState(State::OPEN_BOX_B);
    }
    return true;
  }
  return false;
}

bool MissionController::checkHomeCheckComplete() {
  if (!odom_received_.load()) return false;
  if ((ros::SteadyTime::now() - last_odom_time_).toSec() > odom_timeout_sec_) {
    home_arrived_ = false;
    return false;
  }

  ros::Time now = ros::Time::now();

  if (!isRobotInHomeZone() || !areVl53RangesValid()) {
    home_arrived_ = false;
    return false;
  }

  // Count the five-second hold only while the entire vehicle is stationary.
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const auto& velocity = current_odom_.twist.twist;
    if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
        !std::isfinite(velocity.angular.z) ||
        std::hypot(velocity.linear.x, velocity.linear.y) > 0.01 ||
        std::abs(velocity.angular.z) > 0.02) {
      home_arrived_ = false;
      return false;
    }
  }

  if (!home_arrived_) {
    home_arrived_ = true;
    home_arrival_time_ = now;
    ROS_INFO("[mission_controller] 机器人已进入起始区");
    return false;
  }

  double hold = (now - home_arrival_time_).toSec();
  if (hold >= home_hold_sec_) {
    ROS_INFO("[mission_controller] 机器人在起始区内稳定 %.1f 秒", hold);

    // ── 全部完成! ──
    enterState(State::STOP);
    mission_completed_.store(true);

    mission_timer_.stop();
    lockChassis();

    std_msgs::Bool done;
    done.data = true;
    mission_finished_pub_.publish(done);

    std_msgs::String display;
    // 持久双行条码显示（P1-3：比赛结束裁判核对用）
    display.data = barcode_display_line1_ + "\n" + barcode_display_line2_ +
                   "\n--- 任务完成 ---";
    display_text_pub_.publish(display);

    ROS_INFO("[mission_controller] ====== 全部任务完成！======");
    return true;
  }
  return false;
}

// ======================================================================
//  辅助: 超时检查
// ======================================================================
bool MissionController::isStageTimedOut() const {
  return (ros::Time::now() - stage_start_time_).toSec() > stage_timeout_sec_;
}

// ======================================================================
//  服务调用辅助
// ======================================================================
void MissionController::callSelectPath(const std::string& path_name) {
  path_manager::SelectPath srv;
  srv.request.path_name = path_name;
  beginServiceCall(path_select_client_, srv);
}

void MissionController::callFineTuningStart() {
  std_srvs::Trigger srv;
  beginServiceCall(fine_tuning_client_, srv);
}

void MissionController::callOpenMedicineBox(int8_t box_id) {
  robot_navigation::OpenMedicineBox srv;
  srv.request.box_id = box_id;
  beginServiceCall(open_box_client_, srv);
}

void MissionController::callArmPlaceMedicine(int8_t bed_id, int8_t box_id) {
  arm_and_gripper::ArmPlaceMedicine srv;
  srv.request.bed_id = bed_id;
  srv.request.box_id = box_id;
  beginServiceCall(arm_place_client_, srv);
}

void MissionController::callSpeak(const std::string& text) {
  robot_navigation::Speak srv;
  srv.request.text = text;
  beginServiceCall(speak_client_, srv);
}

bool MissionController::pollServiceCall() {
  if (!service_future_.valid()) return false;
  if (service_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return true;
  const auto result = service_future_.get();
  if (service_generation_ != mission_generation_ || service_state_ != current_state_) return true;
  if (!result.success) {
    enterFailedState("服务失败 (" + stateToString(service_state_) + "): " + result.message);
    return true;
  }
  switch (service_state_) {
    case State::OPEN_BOX_A: enterState(State::PLACE_MEDICINE_A); break;
    case State::OPEN_BOX_B: enterState(State::PLACE_MEDICINE_B); break;
    case State::PLACE_MEDICINE_A:
    case State::PLACE_MEDICINE_B:
      medicine_placed_time_ = ros::Time::now();
      voice_timing_active_ = true;
      enterState(service_state_ == State::PLACE_MEDICINE_A ? State::VOICE_A : State::VOICE_B);
      break;
    case State::VOICE_A: unlockChassis(); enterState(State::GOTO_BED_B); break;
    case State::VOICE_B: unlockChassis(); enterState(State::RETURN_HOME); break;
    default: break;  // Navigation/fine tuning complete through their event topics.
  }
  return true;
}

void MissionController::cancelMissionActions() {
  ++mission_generation_;
  mission_timer_.stop();
  lockChassis();
  std_msgs::Bool enabled;
  enabled.data = false;
  actions_enabled_pub_.publish(enabled);
  std_msgs::Empty cancel;
  cancel_pub_.publish(cancel);
}

// ── 判断机器人是否在起始区内 ───────────────────────────────
bool MissionController::isRobotInHomeZone() const {
  if (!odom_received_ || home_zone_vertices_.empty()) {
    // 无里程计数据或无配置，回退到圆形判定
    return false;
  }

  std::lock_guard<std::mutex> lock(odom_mutex_);
  double rx = current_odom_.pose.pose.position.x;
  double ry = current_odom_.pose.pose.position.y;

  // 射线法判断点是否在多边形内
  int crossings = 0;
  const size_t n = home_zone_vertices_.size();
  for (size_t i = 0; i < n; ++i) {
    const Point2D& p1 = home_zone_vertices_[i];
    const Point2D& p2 = home_zone_vertices_[(i + 1) % n];

    if (((p1.y > ry) != (p2.y > ry)) &&
        (rx < (p2.x - p1.x) * (ry - p1.y) / (p2.y - p1.y) + p1.x)) {
      crossings++;
    }
  }

  if (crossings % 2 != 1) return false;

  // The rule checks the vertical projection of every part, not just base_link.
  // Approximate the measured vehicle+arm envelope by a configurable disk and
  // require its clearance from every home-zone edge. This is conservative for
  // rectangular zones and can be tuned with full_projection_radius_m.
  const double required_clearance = full_projection_radius_m_ + circle_boundary_margin_m_;
  for (size_t i = 0; i < n; ++i) {
    const Point2D& p1 = home_zone_vertices_[i];
    const Point2D& p2 = home_zone_vertices_[(i + 1) % n];
    const double ex = p2.x - p1.x;
    const double ey = p2.y - p1.y;
    const double length_sq = ex * ex + ey * ey;
    const double projection = length_sq > 1e-12
        ? clamp(((rx - p1.x) * ex + (ry - p1.y) * ey) / length_sq, 0.0, 1.0)
        : 0.0;
    const double nearest_x = p1.x + projection * ex;
    const double nearest_y = p1.y + projection * ey;
    if (std::hypot(rx - nearest_x, ry - nearest_y) < required_clearance) {
      return false;
    }
  }
  return true;
}

bool MissionController::isProjectionInsideCircle(const CircleDef& circle,
                                                  bool include_arm) const {
  if (!odom_received_.load()) return false;
  if ((ros::SteadyTime::now() - last_odom_time_).toSec() > odom_timeout_sec_) return false;

  nav_msgs::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = current_odom_;
  }

  const double x = odom.pose.pose.position.x;
  const double y = odom.pose.pose.position.y;
  const double envelope_radius = include_arm ? full_projection_radius_m_
                                             : base_projection_radius_m_;
  const double usable_radius = circle.radius - envelope_radius - circle_boundary_margin_m_;
  const double center_error = std::hypot(x - circle.center_x, y - circle.center_y);
  const bool inside = usable_radius > 0.0 && center_error <= usable_radius;
  if (!inside) {
    ROS_WARN_THROTTLE(1.0,
                      "[mission_controller] 投影校验未通过: center_error=%.3f m, usable_radius=%.3f m",
                      center_error, usable_radius);
  }
  return inside;
}

bool MissionController::areVl53RangesValid() const {
  if (!enable_vl53_circle_check_) return true;

  std::lock_guard<std::mutex> lock(range_mutex_);
  const ros::Time now = ros::Time::now();
  const sensor_msgs::Range* samples[] = {&front_range_, &left_range_, &right_range_};
  const ros::Time* stamps[] = {&front_range_time_, &left_range_time_, &right_range_time_};
  int valid_count = 0;
  for (size_t i = 0; i < 3; ++i) {
    const double age = (now - *stamps[i]).toSec();
    const double header_age = samples[i]->header.stamp.isZero() ? 0.0 :
        (now - samples[i]->header.stamp).toSec();
    const bool valid = !stamps[i]->isZero() && age >= 0.0 &&
                       age <= vl53_circle_timeout_sec_ &&
                       header_age >= -0.05 && header_age <= vl53_circle_timeout_sec_ &&
                       std::isfinite(samples[i]->range) &&
                       std::isfinite(samples[i]->min_range) && std::isfinite(samples[i]->max_range) &&
                       samples[i]->range >= vl53_min_clearance_m_ &&
                       samples[i]->range >= samples[i]->min_range &&
                       samples[i]->range <= samples[i]->max_range;
    if (valid) ++valid_count;
  }

  const bool ok = require_all_vl53_ranges_ ? valid_count == 3 : valid_count > 0;
  if (!ok) {
    ROS_WARN_THROTTLE(1.0,
                      "[mission_controller] VL53投影校验等待数据: valid=%d/3, require_all=%s",
                      valid_count, require_all_vl53_ranges_ ? "true" : "false");
  }
  return ok;
}

// ── 重置任务状态 ───────────────────────────────────────────
void MissionController::resetMissionState() {
  qr_received_.store(false);
  barcode_bed1_received_.store(false);
  barcode_bed3_received_.store(false);
  path_finished_received_.store(false);
  fine_tuning_done_received_.store(false);
  medicine_release_done_received_.store(false);
  home_arrived_ = false;
  action_initiated_ = false;
  mission_completed_.store(false);

  qr_result_ = QrResult();
  ++scan_session_id_;
  std_msgs::UInt32 session; session.data = scan_session_id_; scan_session_pub_.publish(session);
  std_msgs::Int8 target; target.data = 0; scan_target_bed_pub_.publish(target);
  barcode_bed1_value_.clear();
  barcode_bed3_value_.clear();
  // 注意: barcode_display_line1_/line2_ 不重置（持久保留用于比赛结束核对）

  chassis_locked_ = false;
  voice_timing_active_ = false;
  bed_verified_ = false;
  stage_skip_attempted_ = false;
  stage_timeout_sec_ = stage_timeout_default_sec_;

  ROS_INFO("[mission_controller] 任务状态已重置");
}

// ── 进入失败状态 ───────────────────────────────────────────
void MissionController::enterFailedState(const std::string& reason) {
  cancelMissionActions();
  ROS_ERROR("[mission_controller] ====== 任务失败: %s ======", reason.c_str());
  current_state_ = State::FAILED;
  mission_completed_.store(true);

  std_msgs::Bool fail_msg;
  fail_msg.data = false;
  mission_finished_pub_.publish(fail_msg);

  std_msgs::Empty stop;
  stop_all_pub_.publish(stop);

  std_msgs::String display;
  display.data = "任务失败: " + reason;
  display_text_pub_.publish(display);
}

// ── 进入超时状态 ───────────────────────────────────────────
void MissionController::enterTimeoutState() {
  cancelMissionActions();
  current_state_ = State::TIMEOUT;
  mission_completed_.store(true);

  std_msgs::Bool timeout_msg;
  timeout_msg.data = true;
  mission_timeout_pub_.publish(timeout_msg);

  std_msgs::Empty stop;
  stop_all_pub_.publish(stop);

  std_msgs::String display;
  display.data = "任务超时!";
  display_text_pub_.publish(display);
}

// ── 状态描述 ───────────────────────────────────────────────
std::string MissionController::stateToString(State s) const {
  switch (s) {
    case State::IDLE:               return "IDLE";
    case State::GOTO_NURSE:         return "GOTO_NURSE";
    case State::SCAN_QR:            return "SCAN_QR";
    case State::GOTO_BED_A:         return "GOTO_BED_A";
    case State::POSITION_IN_CIRCLE_A: return "POSITION_IN_CIRCLE_A";
    case State::SCAN_BARCODE_A:     return "SCAN_BARCODE_A";
    case State::OPEN_BOX_A:         return "OPEN_BOX_A";
    case State::PLACE_MEDICINE_A:   return "PLACE_MEDICINE_A";
    case State::VOICE_A:            return "VOICE_A";
    case State::GOTO_BED_B:         return "GOTO_BED_B";
    case State::POSITION_IN_CIRCLE_B: return "POSITION_IN_CIRCLE_B";
    case State::SCAN_BARCODE_B:     return "SCAN_BARCODE_B";
    case State::OPEN_BOX_B:         return "OPEN_BOX_B";
    case State::PLACE_MEDICINE_B:   return "PLACE_MEDICINE_B";
    case State::VOICE_B:            return "VOICE_B";
    case State::RETURN_HOME:        return "RETURN_HOME";
    case State::HOME_CHECK:         return "HOME_CHECK";
    case State::STOP:               return "STOP";
    case State::TIMEOUT:            return "TIMEOUT";
    case State::FAILED:             return "FAILED";
    default:                        return "UNKNOWN";
  }
}

// ── 底盘锁死 ───────────────────────────────────────────────
void MissionController::lockChassis() {
  chassis_locked_ = true;
  std_msgs::Bool lock_msg;
  lock_msg.data = true;
  chassis_lock_pub_.publish(lock_msg);

  // The mux stops immediately on /chassis_lock. /stop_all is reserved for
  // actual faults because health_monitor converts it to a latched ESTOP.
  ROS_INFO("[mission_controller] 🔒 底盘已锁死");
}

void MissionController::unlockChassis() {
  chassis_locked_ = false;
  std_msgs::Bool lock_msg;
  lock_msg.data = false;
  chassis_lock_pub_.publish(lock_msg);

  ROS_INFO("[mission_controller] 🔓 底盘已解锁");
}

// ── 持久条码显示（P1-3） ───────────────────────────────────
void MissionController::updatePersistentBarcodeDisplay() {
  std_msgs::String display;
  // 格式：第一行 = 1床条码，第二行 = 3床条码
  // 每行独立显示，不随任务阶段变化
  std::string line1 = barcode_display_line1_.empty()
      ? "1床条码: ---" : barcode_display_line1_;
  std::string line2 = barcode_display_line2_.empty()
      ? "3床条码: ---" : barcode_display_line2_;

  display.data = line1 + "\n" + line2;
  display_text_pub_.publish(display);

  ROS_INFO("[mission_controller] [DISPLAY] %s | %s", line1.c_str(), line2.c_str());
}

bool MissionController::isValidBarcode(const std::string& value) const {
  if (value.size() < 4 || value.size() > 32) return false;
  for (unsigned char c : value) {
    if (c <= 0x20 || c > 0x7e) return false;
  }
  return true;
}

// ── 床号视觉校验（P2-2） ───────────────────────────────────
bool MissionController::verifyBedNumber(int expected_bed) {
  // 基于当前已收到的条形码数据推断所在床位
  // 如果 QR 指定先到1床，则应该在1床收到条形码
  // 这里做简单的逻辑校验：预期的床号是否与被调用告知的一致

  if (expected_bed == 1 && barcode_bed1_received_ && isValidBarcode(barcode_bed1_value_)) {
    ROS_INFO("[mission_controller] ✅ 床号校验通过: 确认在1床");
    return true;
  }
  if (expected_bed == 3 && barcode_bed3_received_ && isValidBarcode(barcode_bed3_value_)) {
    ROS_INFO("[mission_controller] ✅ 床号校验通过: 确认在3床");
    return true;
  }

  // 如果还没收到条形码，暂时无法确认（不算失败）
  if (!barcode_bed1_received_ && !barcode_bed3_received_) {
    ROS_WARN("[mission_controller] ⚠ 暂未收到条形码，无法校验床号");
    return true;  // 暂不阻止流程
  }

  // 收到了错误的床号条形码
  ROS_ERROR("[mission_controller] ❌ 床号校验失败！期望 %d 床，但收到了另一床的条形码",
            expected_bed);
  return false;
}

// ── 阶段超时检查（增强版，含跳过尝试） ──────────────────────
// （此函数已在头文件中声明，在 processState() 中调用）
// 如果阶段超时，在 enterFailedState 前尝试跳到下一阶段

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::MissionController controller(nh, pnh);
  if (!controller.init()) {
    ROS_FATAL("[mission_controller] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
