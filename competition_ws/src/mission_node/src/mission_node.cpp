#include "mission_node/mission_fsm.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>

namespace mission {

// ==================== 构造函数 ====================

MissionFSM::MissionFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {

  // 订阅 Fast-Drone-250 定位源
  vins_odom_sub_ = nh_.subscribe("/vins_fusion/odometry", 10,
                                 &MissionFSM::vinsOdomCallback, this);

  // 订阅视觉检测结果
  color_sub_ = nh_.subscribe("/vision/detected_color", 10,
                             &MissionFSM::colorCallback, this);

  // 订阅抓取机构状态
  gripper_state_sub_ = nh_.subscribe("/gripper/state", 10,
    [this](const std_msgs::String::ConstPtr& msg) {
      if (msg->data == "grasped") ball_held_ = true;
      if (msg->data == "released") ball_held_ = false;
    });

  // 发布航点给 EGO-Planner (flight_type=1: 订阅 /move_base_simple/goal)
  waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // 发布抓取指令
  gripper_cmd_pub_ = nh_.advertise<std_msgs::String>("/gripper/command", 10);

  // 发布分数
  score_pub_ = nh_.advertise<std_msgs::Int32>("/mission/score", 10, true);

  // 读取参数
  pnh_.param("enable_grasp",       enable_grasp_,       false);
  pnh_.param("conservative_mode",  conservative_mode_,  true);
  pnh_.param("speed",              config_.speed,       0.5);
  pnh_.param("tolerance",          config_.tolerance,   0.3);
}

bool MissionFSM::init() {
  setState(TaskState::IDLE, "Initialized");
  loadMissionConfig();

  ROS_INFO("[Mission] FSM initialized. Fast-Drone-250 mode.");
  ROS_INFO("[Mission] Conservative: %s  Grasp: %s",
           conservative_mode_ ? "ON" : "OFF",
           enable_grasp_ ? "ON" : "OFF");

  // 等待 VINS 里程计数据
  ros::Rate rate(10);
  int wait_count = 0;
  while (ros::ok() && currentZ() < 1e-6 && wait_count < 100) {
    ros::spinOnce();
    rate.sleep();
    wait_count++;
  }

  if (wait_count >= 100) {
    ROS_WARN("[Mission] VINS 里程计超时, 但仍继续运行");
  } else {
    ROS_INFO("[Mission] VINS connected — pos(%.2f, %.2f, %.2f)",
             currentX(), currentY(), currentZ());
  }
  return true;
}

// ==================== 主运行循环 ====================

void MissionFSM::run() {
  ros::Rate rate(20);
  mission_start_time_ = ros::Time::now();

  while (ros::ok() && state_ != TaskState::FINISH && state_ != TaskState::FAILURE) {
    ros::spinOnce();

    // 时间预算检查
    double elapsed = (ros::Time::now() - mission_start_time_).toSec();
    if (elapsed > 720.0) {  // 12分钟硬限
      ROS_ERROR("[Mission] 全局超时 (%.0fs), 紧急降落!", elapsed);
      setState(TaskState::EMERGENCY_LAND, "Global timeout");
    }

    switch (state_) {
      case TaskState::IDLE:           handleIdle();           break;
      case TaskState::ARMING:         handleArming();         break;
      case TaskState::TAKEOFF:        handleTakeoff();        break;
      case TaskState::NAV_TO_A:       handleNavToA();         break;
      case TaskState::DETECT_COLOR:   handleDetectColor();    break;
      case TaskState::GRASP_BALL:     handleGraspBall();      break;
      case TaskState::PASS_GATE1:     handlePassGate1();      break;
      case TaskState::NAV_TO_B:       handleNavToB();         break;
      case TaskState::PASS_GATE2:     handlePassGate2();      break;
      case TaskState::DROP_AND_OUTPUT:handleDropAndOutput();   break;
      case TaskState::LANDING:        handleLanding();        break;
      case TaskState::EMERGENCY_LAND: handleEmergencyLand();  break;
      case TaskState::FINISH:
      case TaskState::FAILURE:                                break;
    }

    rate.sleep();
  }

  ROS_INFO("[Mission] === 任务结束 === 总分: %d", total_score_);
}

// ==================== 状态处理 ====================

void MissionFSM::handleIdle() {
  ROS_INFO("[Mission] IDLE — 等待任务启动...");
  ros::Duration(1.0).sleep();
  setState(TaskState::ARMING, "Starting mission");
}

void MissionFSM::handleArming() {
  ROS_INFO("[Mission] ARMING — 等待 px4ctrl + takeoff.sh 执行起飞...");
  ROS_INFO("[Mission] 请确保已运行: sh shfiles/takeoff.sh");
  ROS_INFO("[Mission] px4ctrl 将自动解锁并切换到 offboard 模式");

  // px4ctrl 起飞后会在当前XY位置爬升到目标高度
  // 等待无人机到达起飞悬停高度
  ros::Rate rate(10);
  ros::Time start = ros::Time::now();
  double target_z = std::max(config_.takeoff_z / 100.0, config_.min_takeoff_height);

  while (ros::ok()) {
    ros::spinOnce();

    if (currentZ() > target_z - 0.15) {
      ROS_INFO("[Mission] 起飞悬停高度达到: %.2f m", currentZ());
      break;
    }

    if ((ros::Time::now() - start).toSec() > 20.0) {
      ROS_ERROR("[Mission] 起飞超时!");
      setState(TaskState::FAILURE, "Takeoff timeout");
      return;
    }
    rate.sleep();
  }

  addScore(10, "Task1: takeoff & hover");
  setState(TaskState::TAKEOFF, "Takeoff complete");
}

void MissionFSM::handleTakeoff() {
  double hover_z = std::max(config_.takeoff_z / 100.0, config_.min_takeoff_height);
  ROS_INFO("[Mission] TAKEOFF — 悬停在 (0, 0, %.1f) VINS坐标", hover_z);

  ros::Duration(config_.hover_duration).sleep();

  ROS_INFO("[Mission] 基本功能测试完成! (任务1 +10分)");
  setState(TaskState::NAV_TO_A, "Heading to A");
}

void MissionFSM::handleNavToA() {
  double ax = mapToVinsX(config_.a_x);
  double ay = mapToVinsY(config_.a_y);
  double az = mapToVinsZ(config_.a_z);
  ROS_INFO("[Mission] NAV_TO_A — 目标 (%.2f, %.2f, %.2f) VINS坐标", ax, ay, az);

  // 构建完整航点序列: 当前点 → via点 → A目标
  std::vector<Waypoint> wps;
  for (const auto& v : config_.via_a) {
    wps.push_back({mapToVinsX(v.x), mapToVinsY(v.y), mapToVinsZ(v.z)});
  }
  wps.push_back({ax, ay, az});

  publishWaypointSequence(wps, config_.speed * 0.6);

  // 等待到达A区
  if (!waitArrived(ax, ay, az, config_.tolerance, 60.0)) {
    ROS_WARN("[Mission] 到达A区超时");
    setState(TaskState::EMERGENCY_LAND, "Nav to A timeout");
    return;
  }

  // 悬停满足规则要求
  ROS_INFO("[Mission] 到达 A 区, 悬停 %.0f 秒...", config_.hover_duration);
  ros::Duration(config_.hover_duration).sleep();

  addScore(10, "Task2: H->A nav");
  setState(TaskState::DETECT_COLOR, "Arrived at A");
}

void MissionFSM::handleDetectColor() {
  ROS_INFO("[Mission] DETECT_COLOR — 等待 vision_node 检测结果...");

  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    if (!latest_color_.empty()) {
      config_.detected_color = latest_color_;
      ROS_INFO("[Mission] 检测到颜色: %s", latest_color_.c_str());

      std::cout << "\n============================================" << std::endl;
      std::cout << "  A处识别颜色: " << latest_color_ << std::endl;
      std::cout << "============================================\n" << std::endl;

      addScore(15, "Task6: color detection");
      break;
    }

    if ((ros::Time::now() - start).toSec() > config_.detect_timeout) {
      ROS_WARN("[Mission] 颜色检测超时, 默认 G");
      config_.detected_color = "G";
      break;
    }

    ros::Duration(0.3).sleep();
  }

  // 下一步分叉:
  // 保守模式: 去竞速门1 → B → 降落 (跳过抓取投放)
  // 激进模式: 抓球 → 竞速门1 → B → 投放 → 降落
  if (conservative_mode_) {
    setState(TaskState::PASS_GATE1, "Conservative: skip grasp, go gate1");
    return;
  }

  if (enable_grasp_) {
    setState(TaskState::GRASP_BALL, "Attempt grasp");
  } else {
    setState(TaskState::PASS_GATE1, "Skip grasp, go gate1");
  }
}

void MissionFSM::handleGraspBall() {
  ROS_INFO("[Mission] GRASP_BALL — 抓取小球 (+15分)");

  double bx = mapToVinsX(config_.ball_x);
  double by = mapToVinsY(config_.ball_y);
  double above_z = mapToVinsZ(config_.ball_z) + 0.10;  // 球上方10cm

  // Step 1: 飞到球上方
  publishWaypoint(bx, by, above_z);
  if (!waitArrived(bx, by, above_z, 0.15, 15.0)) {
    ROS_WARN("[Mission] 无法到达球位置, 跳过抓取");
    setState(TaskState::PASS_GATE1, "Skip grasp");
    return;
  }

  // Step 2: 缓降至抓取高度
  double grasp_z = mapToVinsZ(config_.ball_z) - 0.02;  // 球心略下方
  publishWaypoint(bx, by, grasp_z);
  if (!waitArrived(bx, by, grasp_z, 0.08, 8.0)) {
    ROS_WARN("[Mission] 下降抓取失败");
    publishWaypoint(bx, by, above_z);  // 回升
    ros::Duration(1.0).sleep();
    setState(TaskState::PASS_GATE1, "Grasp descend failed");
    return;
  }

  // Step 3: 触发抓取
  std_msgs::String cmd;
  cmd.data = "grasp";
  gripper_cmd_pub_.publish(cmd);
  ROS_INFO("[Mission] 抓取指令已发送, 等待确认...");

  ros::Time grasp_start = ros::Time::now();
  while (ros::ok() && !ball_held_) {
    ros::spinOnce();
    if ((ros::Time::now() - grasp_start).toSec() > 3.0) break;
    ros::Duration(0.1).sleep();
  }

  // Step 4: 抬升
  publishWaypoint(bx, by, mapToVinsZ(config_.a_z) + 0.2);
  if (!waitArrived(bx, by, mapToVinsZ(config_.a_z) + 0.2, 0.2, 5.0)) {
    ROS_WARN("[Mission] 抬升超时");
  }

  if (ball_held_) {
    addScore(15, "Task3: ball grasp");
    ROS_INFO("[Mission] 抓取成功!");
  } else {
    ROS_WARN("[Mission] 抓取未确认, 继续任务 (-5分罚分)");
    addScore(-5, "Grasp failed penalty");
  }

  setState(TaskState::PASS_GATE1, "Grasp complete");
}

void MissionFSM::handlePassGate1() {
  ROS_INFO("[Mission] PASS_GATE1 — 穿越竞速门1 (+5分)");

  double gx = mapToVinsX(config_.gate1_x);
  double gy = mapToVinsY(config_.gate1_y);
  double gz = mapToVinsZ(config_.gate1_z);

  // 门前 → 门中 → 门后 三连航点给 EGO-Planner 自动规划
  // Gate1 朝向 Y+, 从下方穿过
  std::vector<Waypoint> gate_wps;
  gate_wps.push_back({gx, gy - 0.60, gz});  // 门前60cm
  gate_wps.push_back({gx, gy, gz});          // 门正中
  gate_wps.push_back({gx, gy + 0.50, gz});   // 门后50cm

  publishWaypointSequence(gate_wps, config_.speed * 0.4);

  if (waitArrived(gx, gy + 0.50, gz, config_.gate_tolerance, 20.0)) {
    gate1_passed_ = true;
    addScore(5, "Gate1 passed");
    ROS_INFO("[Mission] 竞速门1 穿越成功! +5分");
  } else {
    ROS_WARN("[Mission] 竞速门1 穿越超时, 继续");
    gate1_passed_ = true;
  }

  setState(TaskState::NAV_TO_B, "After Gate1");
}

void MissionFSM::handleNavToB() {
  double bx = mapToVinsX(config_.b_x);
  double by = mapToVinsY(config_.b_y);
  double bz = mapToVinsZ(config_.b_z);
  ROS_INFO("[Mission] NAV_TO_B — 目标 (%.2f, %.2f, %.2f) VINS坐标", bx, by, bz);

  // 构建航点序列: 当前点 → via点 → B目标
  std::vector<Waypoint> wps;
  for (const auto& v : config_.via_b) {
    wps.push_back({mapToVinsX(v.x), mapToVinsY(v.y), mapToVinsZ(v.z)});
  }
  wps.push_back({bx, by, bz});

  publishWaypointSequence(wps, config_.speed * 0.6);

  if (!waitArrived(bx, by, bz, config_.tolerance, 60.0)) {
    ROS_WARN("[Mission] 到达B区超时");
    setState(TaskState::EMERGENCY_LAND, "Nav to B timeout");
    return;
  }

  ROS_INFO("[Mission] 到达 B 区, 悬停 %.0f 秒...", config_.hover_duration);
  ros::Duration(config_.hover_duration).sleep();

  addScore(10, "Task4: A->B nav");
  setState(TaskState::PASS_GATE2, "Arrived at B, try Gate2");
}

void MissionFSM::handlePassGate2() {
  ROS_INFO("[Mission] PASS_GATE2 — 穿越竞速门2 (+5分)");

  double gx = mapToVinsX(config_.gate2_x);
  double gy = mapToVinsY(config_.gate2_y);
  double gz = mapToVinsZ(config_.gate2_z);

  // Gate2 朝向 X+, 从左侧穿过
  std::vector<Waypoint> gate_wps;
  gate_wps.push_back({gx - 0.50, gy, gz});  // 门前50cm
  gate_wps.push_back({gx, gy, gz});          // 门正中
  gate_wps.push_back({gx + 0.50, gy, gz});   // 门后50cm

  publishWaypointSequence(gate_wps, config_.speed * 0.4);

  if (waitArrived(gx + 0.50, gy, gz, config_.gate_tolerance, 20.0)) {
    gate2_passed_ = true;
    addScore(5, "Gate2 passed");
    ROS_INFO("[Mission] 竞速门2 穿越成功! +5分");
  } else {
    ROS_WARN("[Mission] 竞速门2 穿越超时, 继续");
    gate2_passed_ = true;
  }

  setState(TaskState::DROP_AND_OUTPUT, "After Gate2");
}

void MissionFSM::handleDropAndOutput() {
  std::string color = config_.detected_color;
  if (color.empty()) color = "G";

  double score_x, score_y;
  if (color == "R") {
    score_x = mapToVinsX(config_.score_r_x);
    score_y = mapToVinsY(config_.score_r_y);
  } else if (color == "B") {
    score_x = mapToVinsX(config_.score_b_x);
    score_y = mapToVinsY(config_.score_b_y);
  } else {
    score_x = mapToVinsX(config_.score_g_x);
    score_y = mapToVinsY(config_.score_g_y);
  }

  ROS_INFO("[Mission] DROP_AND_OUTPUT — 颜色=%s, 目标(%.2f, %.2f)",
           color.c_str(), score_x, score_y);

  // 保守模式: 跳过投放, 直接准备降落
  if (conservative_mode_ || !ball_held_) {
    std::cout << "\n============================================" << std::endl;
    std::cout << "  识别颜色: " << color << "  降落区: " << color << std::endl;
    std::cout << "  当前总分: " << total_score_ << std::endl;
    std::cout << "============================================\n" << std::endl;
    setState(TaskState::LANDING, "Go land (no drop)");
    return;
  }

  // 飞到得分框正上方40cm
  double drop_z = 0.40;
  publishWaypoint(score_x, score_y, drop_z);
  if (!waitArrived(score_x, score_y, drop_z, 0.15, 15.0)) {
    ROS_WARN("[Mission] 无法到达投放位置");
    setState(TaskState::LANDING, "Drop position unreachable");
    return;
  }

  // 释放球
  std_msgs::String cmd;
  cmd.data = "release";
  gripper_cmd_pub_.publish(cmd);
  ROS_INFO("[Mission] 释放指令已发送");
  ball_held_ = false;
  ros::Duration(1.0).sleep();

  addScore(15, "Task5: precision drop");

  // 抬升回巡航高度
  publishWaypoint(score_x, score_y, mapToVinsZ(config_.b_z));
  ros::Duration(2.0).sleep();

  std::cout << "\n============================================" << std::endl;
  std::cout << "  投放颜色: " << color << "  降落区: " << color << std::endl;
  std::cout << "  当前总分: " << total_score_ << std::endl;
  std::cout << "============================================\n" << std::endl;

  setState(TaskState::LANDING, "Drop complete, landing");
}

void MissionFSM::handleLanding() {
  std::string color = config_.detected_color;
  if (color.empty()) color = "G";

  double land_x, land_y;
  if (color == "R") {
    land_x = mapToVinsX(config_.land_r_x);
    land_y = mapToVinsY(config_.land_r_y);
  } else if (color == "B") {
    land_x = mapToVinsX(config_.land_b_x);
    land_y = mapToVinsY(config_.land_b_y);
  } else {
    land_x = mapToVinsX(config_.land_g_x);
    land_y = mapToVinsY(config_.land_g_y);
  }

  ROS_INFO("[Mission] LANDING — 降落在 %s 区 (%.2f, %.2f)",
           color.c_str(), land_x, land_y);

  // Step 1: 飞到降落板上方 50cm
  publishWaypoint(land_x, land_y, 0.5);
  if (!waitArrived(land_x, land_y, 0.5, 0.2, 15.0)) {
    ROS_WARN("[Mission] 无法精确到达降落板上方, 继续降落");
  }

  // Step 2: 缓慢下降至触地
  ROS_INFO("[Mission] 开始下降...");
  if (descendToTouchdown(land_x, land_y, 25.0)) {
    addScore(15, "Task7: landing");
    ROS_INFO("[Mission] 降落完成!");
  } else {
    ROS_WARN("[Mission] 降落超时, 可能未完全着地");
  }

  // 最终分数报告
  std::cout << "\n============================================" << std::endl;
  std::cout << "  任务完成!" << std::endl;
  std::cout << "  目标颜色: " << color << std::endl;
  std::cout << "  总分: " << total_score_ << " / 100" << std::endl;
  std::cout << "  Gate1: " << (gate1_passed_ ? "OK" : "NO");
  std::cout << "  Gate2: " << (gate2_passed_ ? "OK" : "NO") << std::endl;
  double elapsed = (ros::Time::now() - mission_start_time_).toSec();
  std::cout << "  用时: " << static_cast<int>(elapsed) << "s" << std::endl;
  std::cout << "============================================\n" << std::endl;

  setState(TaskState::FINISH, "Mission complete");
}

void MissionFSM::handleEmergencyLand() {
  ROS_WARN("[Mission] EMERGENCY_LAND — 就近降落!");

  double cx = currentX(), cy = currentY();

  for (int i = 0; i < 50; i++) {
    publishWaypoint(cx, cy, 0.0);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  // 等待触地
  ros::Time start = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    if (currentZ() < 0.05 && std::abs(current_odom_.twist.twist.linear.z) < 0.1) break;
    if ((ros::Time::now() - start).toSec() > 15.0) break;
    publishWaypoint(cx, cy, 0.0);
    ros::Duration(0.1).sleep();
  }

  setState(TaskState::FAILURE, "Emergency landing");
}

void MissionFSM::handleFinish() {
  ROS_INFO("[Mission] FINISH");
}

// ==================== 飞控接口 (Fast-Drone-250) ====================

void MissionFSM::publishWaypoint(double x_vins, double y_vins, double z_vins, double yaw) {
  if (z_vins > config_.max_altitude) {
    z_vins = config_.max_altitude;
  }
  if (z_vins < 0) z_vins = 0;

  geometry_msgs::PoseStamped wp;
  wp.header.stamp = ros::Time::now();
  wp.header.frame_id = "vins_world";
  wp.pose.position.x = x_vins;
  wp.pose.position.y = y_vins;
  wp.pose.position.z = z_vins;
  wp.pose.orientation.w = 1.0;
  waypoint_pub_.publish(wp);
}

void MissionFSM::publishWaypointSequence(const std::vector<Waypoint>& waypoints, double speed) {
  if (waypoints.empty()) return;

  ros::Rate rate(10);
  for (const auto& wp : waypoints) {
    publishWaypoint(wp.x, wp.y, wp.z, wp.yaw);
    ROS_INFO("[Mission] 航点 → EGO-Planner: (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.z);

    // 等待无人机接近当前航点, 然后发下一个
    ros::Time start = ros::Time::now();
    while (ros::ok()) {
      ros::spinOnce();
      double d = std::sqrt(std::pow(currentX() - wp.x, 2) +
                           std::pow(currentY() - wp.y, 2) +
                           std::pow(currentZ() - wp.z, 2));
      if (d < config_.tolerance) break;
      if ((ros::Time::now() - start).toSec() > 40.0) break;
      rate.sleep();
    }
  }
}

bool MissionFSM::waitArrived(double x, double y, double z, double tolerance, double timeout) {
  ros::Rate rate(10);
  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();
    double d = std::sqrt(std::pow(currentX() - x, 2) +
                         std::pow(currentY() - y, 2) +
                         std::pow(currentZ() - z, 2));

    if (d < tolerance) {
      ROS_INFO("[Mission] 到达目标 (%.2f, %.2f, %.2f) 误差=%.2fm", x, y, z, d);
      return true;
    }

    // 持续更新航点以防 EGO-Planner 悬停
    publishWaypoint(x, y, z);

    if ((ros::Time::now() - start).toSec() > timeout) {
      ROS_WARN("[Mission] waitArrived 超时, 误差: %.2fm", d);
      return false;
    }
    rate.sleep();
  }
  return false;
}

bool MissionFSM::descendToTouchdown(double target_x, double target_y, double timeout) {
  ros::Rate rate(20);
  ros::Time start = ros::Time::now();
  double desired_z = 0.5;

  while (ros::ok()) {
    ros::spinOnce();

    double cz = currentZ();
    double vz = current_odom_.twist.twist.linear.z;

    // 逐步降低目标高度
    desired_z = std::max(desired_z - config_.landing_descend_speed / 20.0, 0.0);
    publishWaypoint(target_x, target_y, desired_z);

    // 触地检测: 高度<5cm 且 垂直速度<5cm/s
    if (cz < config_.landing_touchdown_thresh && std::abs(vz) < 0.05) {
      ROS_INFO("[Mission] 触地检测: z=%.3fm vz=%.3fm/s", cz, vz);
      ros::Duration(1.0).sleep();
      return true;
    }

    if ((ros::Time::now() - start).toSec() > timeout) return false;
    rate.sleep();
  }
  return false;
}

// ==================== 回调函数 ====================

void MissionFSM::colorCallback(const std_msgs::String::ConstPtr& msg) {
  if (!msg->data.empty() && msg->data != "unknown") {
    latest_color_ = msg->data;
  }
}

void MissionFSM::vinsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_odom_ = *msg;
}

// ==================== 分数追踪 ====================

void MissionFSM::addScore(int points, const std::string& reason) {
  total_score_ += points;
  ROS_INFO("[Mission] %+d分 (%s) — 累计: %d", points, reason.c_str(), total_score_);

  std_msgs::Int32 score_msg;
  score_msg.data = total_score_;
  score_pub_.publish(score_msg);
}

// ==================== 配置加载 ====================

void MissionFSM::loadMissionConfig() {
  double default_speed     = conservative_mode_ ? 0.2 : 0.3;
  double default_tolerance = conservative_mode_ ? 0.4 : 0.2;
  double default_hover     = conservative_mode_ ? 8.0 : 6.0;
  double default_detect_to = conservative_mode_ ? 45.0 : 15.0;

  pnh_.param("speed",            config_.speed,            default_speed);
  pnh_.param("tolerance",        config_.tolerance,        default_tolerance);
  pnh_.param("hover_duration",   config_.hover_duration,   default_hover);
  pnh_.param("detect_timeout",   config_.detect_timeout,   default_detect_to);
  pnh_.param("max_altitude",     config_.max_altitude,     1.5);
  pnh_.param("min_takeoff_height", config_.min_takeoff_height, 0.8);

  if (conservative_mode_) {
    enable_grasp_ = false;
    ROS_INFO("[Mission] 保守模式: speed=%.1f tol=%.1f hover=%.0fs 不抓取",
             config_.speed, config_.tolerance, config_.hover_duration);
  }

  // ---- 从 rosparam 读取 map.yaml 航点 (cm) ----
  auto readWP = [&](const std::string& path, double& x, double& y, double& z) {
    pnh_.param(path + "/x", x, x);
    pnh_.param(path + "/y", y, y);
    pnh_.param(path + "/z", z, z);
  };

  readWP("waypoints/takeoff",    config_.takeoff_x, config_.takeoff_y, config_.takeoff_z);
  readWP("waypoints/nav_to_a/target", config_.a_x, config_.a_y, config_.a_z);
  readWP("waypoints/nav_to_b/target", config_.b_x, config_.b_y, config_.b_z);

  // 读取降落区
  readWP("waypoints/landing_R", config_.land_r_x, config_.land_r_y, config_.land_r_x);
  readWP("waypoints/landing_G", config_.land_g_x, config_.land_g_y, config_.land_g_y);
  readWP("waypoints/landing_B", config_.land_b_x, config_.land_b_y, config_.land_b_y);

  // ---- 读取 via 点 (支持多个) ----
  config_.via_a.clear();
  config_.via_b.clear();

  int via_idx = 0;
  while (true) {
    double vx, vy, vz = 100;
    std::string base = "waypoints/nav_to_a/via/" + std::to_string(via_idx);
    if (!pnh_.getParam(base + "/x", vx)) break;
    pnh_.getParam(base + "/y", vy);
    pnh_.getParam(base + "/z", vz);
    config_.via_a.push_back({vx, vy, vz});
    via_idx++;
  }
  ROS_INFO("[Mission] H->A via points: %zu", config_.via_a.size());

  via_idx = 0;
  while (true) {
    double vx, vy, vz = 100;
    std::string base = "waypoints/nav_to_b/via/" + std::to_string(via_idx);
    if (!pnh_.getParam(base + "/x", vx)) break;
    pnh_.getParam(base + "/y", vy);
    pnh_.getParam(base + "/z", vz);
    config_.via_b.push_back({vx, vy, vz});
    via_idx++;
  }
  ROS_INFO("[Mission] A->B via points: %zu", config_.via_b.size());

  // ---- 球坐标 ----
  readWP("ball", config_.ball_x, config_.ball_y, config_.ball_z);

  // ---- 竞速门坐标 ----
  readWP("gates/gate1/center", config_.gate1_x, config_.gate1_y, config_.gate1_z);
  config_.gate1_z = 100;
  readWP("gates/gate2/center", config_.gate2_x, config_.gate2_y, config_.gate2_z);
  config_.gate2_z = 100;

  // ---- 得分框坐标 ----
  readWP("color_zones/R/center", config_.score_r_x, config_.score_r_y, config_.score_r_x);
  readWP("color_zones/G/center", config_.score_g_x, config_.score_g_y, config_.score_g_y);
  readWP("color_zones/B/center", config_.score_b_x, config_.score_b_y, config_.score_b_y);

  ROS_INFO("[Mission] Config: takeoff(%.0f,%.0f,%.0f) A(%.0f,%.0f) B(%.0f,%.0f)",
           config_.takeoff_x, config_.takeoff_y, config_.takeoff_z,
           config_.a_x, config_.a_y, config_.b_x, config_.b_y);
}

// ==================== 状态转换 ====================

void MissionFSM::setState(TaskState new_state, const std::string& reason) {
  ROS_INFO("[Mission] %s → %s (%s)",
           stateToString(state_).c_str(),
           stateToString(new_state).c_str(),
           reason.c_str());
  state_ = new_state;
  state_start_time_ = ros::Time::now();
}

bool MissionFSM::checkTimeout(ros::Time start, double timeout) {
  return (ros::Time::now() - start).toSec() > timeout;
}

}  // namespace mission

// ==================== 主函数 ====================

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mission::MissionFSM fsm(nh, pnh);

  if (!fsm.init()) {
    ROS_ERROR("[Mission] 初始化失败!");
    return 1;
  }

  fsm.run();

  return 0;
}
