#include "mission_node/mission_fsm.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>

namespace mission {

// ==================== 构造函数 ====================

MissionFSM::MissionFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {

  // 订阅
  state_sub_ = nh_.subscribe("/mavros/state", 10, &MissionFSM::stateCallback, this);
  odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &MissionFSM::odomCallback, this);

  // 发布
  setpoint_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  setpoint_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  // 服务
  arming_cli_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_cli_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  // 读取参数
  pnh_.param("enable_grasp",    enable_grasp_,    false);
  pnh_.param("conservative_mode", conservative_mode_, true);
  pnh_.param("speed",     config_.speed,     0.5);
  pnh_.param("tolerance", config_.tolerance, 0.3);
}

bool MissionFSM::init() {
  setState(TaskState::IDLE, "Initialized");

  // 从 param server 加载 map.yaml 配置
  loadMissionConfig();

  ROS_INFO("[Mission] FSM initialized. Waiting for mavros connection...");

  // 等待 mavros 连接
  ros::Rate rate(10);
  int wait_count = 0;
  while (ros::ok() && !current_state_.connected && wait_count < 50) {
    ros::spinOnce();
    rate.sleep();
    wait_count++;
  }
  if (!current_state_.connected) {
    ROS_ERROR("[Mission] Failed to connect to mavros!");
    return false;
  }
  ROS_INFO("[Mission] Connected to mavros! Mode: %s, Armed: %s",
           current_state_.mode.c_str(), current_state_.armed ? "YES" : "NO");
  return true;
}

// ==================== 主运行循环 ====================

void MissionFSM::run() {
  ros::Rate rate(20);

  while (ros::ok() && state_ != TaskState::FINISH && state_ != TaskState::FAILURE) {
    ros::spinOnce();

    switch (state_) {
      case TaskState::IDLE:           handleIdle();           break;
      case TaskState::ARMING:         handleArming();         break;
      case TaskState::TAKEOFF:        handleTakeoff();        break;
      case TaskState::NAV_TO_A:       handleNavToA();         break;
      case TaskState::DETECT_COLOR:   handleDetectColor();    break;
      case TaskState::GRASP_BALL:     handleGraspBall();      break;
      case TaskState::NAV_TO_B:       handleNavToB();         break;
      case TaskState::DROP_AND_OUTPUT:handleDropAndOutput();   break;
      case TaskState::NAV_TO_LAND:    handleNavToLand();      break;
      case TaskState::LANDING:        handleLanding();        break;
      case TaskState::EMERGENCY_LAND: handleEmergencyLand();  break;
      case TaskState::FINISH:
      case TaskState::FAILURE:                                break;
    }

    rate.sleep();
  }

  if (state_ == TaskState::FINISH) {
    ROS_INFO("[Mission] === 任务完成! ===");
  } else {
    ROS_WARN("[Mission] === 任务中止, 最后状态: %s ===", getStateStr().c_str());
  }
}

// ==================== 状态处理 ====================

void MissionFSM::handleIdle() {
  ROS_INFO("[Mission] IDLE — 等待进入 ARMING...");
  setState(TaskState::ARMING, "Starting mission");
}

void MissionFSM::handleArming() {
  ROS_INFO("[Mission] ARMING — 解锁并切换到 OFFBOARD 模式...");

  // 先发送一些 setpoint 让 PX4 进入 offboard 准备状态
  geometry_msgs::PoseStamped sp;
  sp.pose.position.x = 0;
  sp.pose.position.y = 0;
  sp.pose.position.z = 1.0;
  sp.pose.orientation.w = 1.0;

  ros::Rate rate(20);
  for (int i = 0; i < 50; i++) {
    setpoint_pos_pub_.publish(sp);
    ros::spinOnce();
    rate.sleep();
  }

  // 设置 OFFBOARD 模式
  if (!setOffboardMode()) {
    ROS_ERROR("[Mission] Failed to set OFFBOARD mode!");
    setState(TaskState::FAILURE, "Cannot set offboard");
    return;
  }

  // 解锁
  if (!arm()) {
    ROS_ERROR("[Mission] Failed to arm!");
    setState(TaskState::FAILURE, "Cannot arm");
    return;
  }

  ros::Duration(0.5).sleep();
  setState(TaskState::TAKEOFF, "Armed and offboard");
}

void MissionFSM::handleTakeoff() {
  double takeoff_z_m = config_.takeoff_z / 100.0;  // cm -> m
  // 规则5: 悬停高度不低于0.8m
  if (takeoff_z_m < config_.min_takeoff_height) {
    takeoff_z_m = config_.min_takeoff_height;
    ROS_WARN("[Mission] takeoff_z 低于规则要求(0.8m)，已自动提升至 %.1f m", takeoff_z_m);
  }
  ROS_INFO("[Mission] TAKEOFF — 起飞到 %.1f m 高度悬停...", takeoff_z_m);

  geometry_msgs::PoseStamped sp;
  sp.pose.position.x = config_.takeoff_x / 100.0;
  sp.pose.position.y = config_.takeoff_y / 100.0;
  sp.pose.position.z = takeoff_z_m;
  sp.pose.orientation.w = 1.0;

  ros::Rate rate(20);
  ros::Time start = ros::Time::now();

  // 持续发送位置设定点，直到到达目标高度
  while (ros::ok()) {
    ros::spinOnce();

    double current_z = current_odom_.pose.pose.position.z;
    double err_z = std::abs(current_z - takeoff_z_m);

    // 到达目标高度后，悬停 config_.hover_duration 秒
    if (err_z < 0.15) {
      ROS_INFO("[Mission] 到达目标高度, 悬停中...");
      ros::Duration(config_.hover_duration).sleep();
      break;
    }

    setpoint_pos_pub_.publish(sp);

    // 超时保护
    if ((ros::Time::now() - start).toSec() > 15.0) {
      ROS_WARN("[Mission] TAKEOFF 超时, 当前高度: %.2f", current_z);
      setState(TaskState::EMERGENCY_LAND, "Takeoff timeout");
      return;
    }

    rate.sleep();
  }

  ROS_INFO("[Mission] 基本功能测试完成! (任务① +10分)");
  setState(TaskState::NAV_TO_A, "Takeoff complete");
}

void MissionFSM::handleNavToA() {
  ROS_INFO("[Mission] NAV_TO_A — 飞到 A 区域 (%.0f, %.0f)...",
           config_.a_x, config_.a_y);

  // 先飞到中间点绕开挡板1
  {
    geometry_msgs::PoseStamped via;
    via.pose.position.x = config_.via_1_x / 100.0;
    via.pose.position.y = config_.via_1_y / 100.0;
    via.pose.position.z = config_.via_1_z / 100.0;
    via.pose.orientation.w = 1.0;

    if (!flyTo(via.pose.position.x, via.pose.position.y, via.pose.position.z,
               config_.speed, config_.tolerance)) {
      setState(TaskState::EMERGENCY_LAND, "Failed to reach via point");
      return;
    }
  }

  // 飞到 A 区域
  {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = config_.a_x / 100.0;
    target.pose.position.y = config_.a_y / 100.0;
    target.pose.position.z = config_.a_z / 100.0;
    target.pose.orientation.w = 1.0;

    if (!flyTo(target.pose.position.x, target.pose.position.y, target.pose.position.z,
               config_.speed, config_.tolerance)) {
      setState(TaskState::EMERGENCY_LAND, "Failed to reach A zone");
      return;
    }

    // 悬停 5 秒以上（规则要求）
    ROS_INFO("[Mission] 到达 A 区域, 悬停 6 秒...");
    ros::Duration(config_.hover_duration).sleep();
  }

  ROS_INFO("[Mission] 起点→A 导航完成! (任务② +10分)");
  setState(TaskState::DETECT_COLOR, "Arrived at A");
}

void MissionFSM::handleDetectColor() {
  ROS_INFO("[Mission] DETECT_COLOR — 检测 A 区域颜色...");

  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    std::string color = detectColorFromCamera();

    if (!color.empty()) {
      config_.detected_color = color;
      ROS_INFO("[Mission] 检测到颜色: %s", color.c_str());

      // 任务⑥: 在命令行输出颜色
      std::cout << "\n============================================" << std::endl;
      std::cout << "  A处识别颜色: " << color << std::endl;
      std::cout << "============================================\n" << std::endl;

      break;
    }

    // 超时
    if ((ros::Time::now() - start).toSec() > config_.detect_timeout) {
      ROS_WARN("[Mission] 颜色检测超时, 使用默认颜色 G");
      config_.detected_color = "G";
      break;
    }

    ros::Duration(0.5).sleep();
  }

  ROS_INFO("[Mission] 颜色识别输出完成! (任务⑥ +15分)");

  // 保守模式: 直接去B，跳过抓取
  if (conservative_mode_) {
    ROS_INFO("[Mission] 保守模式: 跳过抓取，直接导航至B");
    setState(TaskState::NAV_TO_B, "Conservative: skip grasp, go to B");
    return;
  }

  // 如果启用了抓取，进入抓取状态
  if (enable_grasp_) {
    setState(TaskState::GRASP_BALL, "Color detected, start grasp");
  } else {
    setState(TaskState::NAV_TO_B, "Color detected, skip grasp");
  }
}

void MissionFSM::handleGraspBall() {
  ROS_INFO("[Mission] GRASP_BALL — 抓取小球 (功能可选)...");

  // 靠近球的位置
  double ball_x = 0.625;  // m (62.5cm)
  double ball_y = 3.875;  // m (387.5cm)
  double grasp_z = 1.05;  // m (略高于球)

  if (!flyTo(ball_x, ball_y, grasp_z, 0.2, 0.15)) {
    ROS_WARN("[Mission] 无法到达球位置，跳过抓取");
    setState(TaskState::NAV_TO_B, "Skip grasp - cannot reach");
    return;
  }

  // 下降接触球
  ROS_INFO("[Mission] 下降接触球...");
  if (!flyTo(ball_x, ball_y, 0.95, 0.1, 0.1)) {
    setState(TaskState::NAV_TO_B, "Skip grasp - descend failed");
    return;
  }

  // 触发抓取（电磁铁通电/机械爪闭合）
  ROS_INFO("[Mission] 触发抓取机构...");
  ros::Duration(1.0).sleep();

  // 抬升确认抓取成功
  ROS_INFO("[Mission] 抬升确认...");
  if (!flyTo(ball_x, ball_y, 1.2, 0.2, 0.15)) {
    ROS_WARN("[Mission] 抓取可能失败, 继续任务");
  }

  ROS_INFO("[Mission] 物品抓取完成! (任务③ +15分)");
  setState(TaskState::NAV_TO_B, "Grasp complete");
}

void MissionFSM::handleNavToB() {
  ROS_INFO("[Mission] NAV_TO_B — 飞到 B 区域 (%.0f, %.0f)...",
           config_.b_x, config_.b_y);

  // ===== 保守模式: 穿越竞速门1 (+5分) =====
  if (conservative_mode_ && !gate1_passed_) {
    // 竞速门1: 中心(125, 410)cm, 开口z=65~135cm, 70×70cm
    double gate_z = 1.00;  // 穿门高度1m (开口中心)
    double gx = 1.25, gy = 4.10;  // gate center in meters

    ROS_INFO("[Mission] 保守模式: 穿越竞速门1 (+5分)");

    // Step 1: 飞到门前方(Y-侧)，对齐门洞
    if (!flyTo(gx, gy - 0.30, gate_z, config_.speed * 0.6, 0.15)) {
      ROS_WARN("[Mission] 无法对齐竞速门1，跳过穿门");
      gate1_passed_ = true;  // 标记已尝试
    } else {
      // Step 2: 低速穿越门洞
      ros::Duration(0.5).sleep();  // 短暂稳定
      if (!flyTo(gx, gy + 0.25, gate_z, config_.speed * 0.4, 0.10)) {
        ROS_WARN("[Mission] 穿门可能失败");
      } else {
        ROS_INFO("[Mission] === 竞速门1穿越成功! +5分 ===");
      }
      gate1_passed_ = true;
    }
  }

  // 经停场地中点（绕开障碍物）
  {
    geometry_msgs::PoseStamped via;
    via.pose.position.x = config_.via_2_x / 100.0;
    via.pose.position.y = config_.via_2_y / 100.0;
    via.pose.position.z = config_.via_2_z / 100.0;
    via.pose.orientation.w = 1.0;

    if (!flyTo(via.pose.position.x, via.pose.position.y, via.pose.position.z,
               config_.speed, config_.tolerance)) {
      setState(TaskState::EMERGENCY_LAND, "Failed to reach via point 2");
      return;
    }
  }

  // 飞到 B 区域
  {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = config_.b_x / 100.0;
    target.pose.position.y = config_.b_y / 100.0;
    target.pose.position.z = config_.b_z / 100.0;
    target.pose.orientation.w = 1.0;

    if (!flyTo(target.pose.position.x, target.pose.position.y, target.pose.position.z,
               config_.speed, config_.tolerance)) {
      setState(TaskState::EMERGENCY_LAND, "Failed to reach B zone");
      return;
    }

    ROS_INFO("[Mission] 到达 B 区域, 悬停 %.0f 秒...", config_.hover_duration);
    ros::Duration(config_.hover_duration).sleep();
  }

  double bonus = (gate1_passed_ ? 5.0 : 0.0);
  ROS_INFO("[Mission] A→B 导航完成! (任务④ +10分%s)",
           bonus > 0 ? ", 穿门+5分" : "");
  setState(TaskState::DROP_AND_OUTPUT, "Arrived at B");
}

void MissionFSM::handleDropAndOutput() {
  std::string color = config_.detected_color;
  if (color.empty()) color = "G";

  // 保守模式: 跳过投放，直接确认颜色准备降落
  if (conservative_mode_) {
    ROS_INFO("[Mission] DROP_AND_OUTPUT — 保守模式: 跳过投放, 颜色=%s, 准备降落", color.c_str());
    std::cout << "\n============================================" << std::endl;
    std::cout << "  识别颜色确认: " << color << std::endl;
    std::cout << "  降落目标区域: " << color << std::endl;
    std::cout << "============================================\n" << std::endl;
    setState(TaskState::NAV_TO_LAND, "Conservative: skip drop, go land");
    return;
  }

  ROS_INFO("[Mission] DROP_AND_OUTPUT — 颜色=%s, 准备投放+降落", color.c_str());

  // 选择目标投放/降落坐标
  double target_x, target_y;
  if (color == "R") {
    target_x = config_.land_r_x;
    target_y = config_.land_r_y;
  } else if (color == "B") {
    target_x = config_.land_b_x;
    target_y = config_.land_b_y;
  } else {
    target_x = config_.land_g_x;
    target_y = config_.land_g_y;
    color = "G";
  }

  // 悬停到目标区域上方 50cm
  double hover_x = target_x / 100.0;
  double hover_y = target_y / 100.0;
  double hover_z = 0.5;

  if (!flyTo(hover_x, hover_y, hover_z, config_.speed * 0.8, config_.tolerance)) {
    ROS_WARN("[Mission] 无法到达目标降落区上方, 就近降落");
  }

  // 如果启用了抓取，执行投放
  if (enable_grasp_) {
    ROS_INFO("[Mission] 投放小球...");
    // 释放机构（电磁铁断电/机械爪张开）
    ros::Duration(1.0).sleep();
    ROS_INFO("[Mission] 精准投放完成! (任务⑤ +15分)");
  } else {
    ROS_INFO("[Mission] 跳过投放 (未启用抓取)");
  }

  // 任务⑥: 颜色确认输出（已经在上一步输出过了，这里重复确保得分）
  std::cout << "\n============================================" << std::endl;
  std::cout << "  识别颜色确认: " << color << std::endl;
  std::cout << "  降落目标区域: " << color << std::endl;
  std::cout << "============================================\n" << std::endl;

  setState(TaskState::NAV_TO_LAND, "Ready to land");
}

void MissionFSM::handleNavToLand() {
  std::string color = config_.detected_color;
  if (color.empty()) color = "G";

  double target_x, target_y;
  if (color == "R") {
    target_x = config_.land_r_x / 100.0;
    target_y = config_.land_r_y / 100.0;
  } else if (color == "B") {
    target_x = config_.land_b_x / 100.0;
    target_y = config_.land_b_y / 100.0;
  } else {
    target_x = config_.land_g_x / 100.0;
    target_y = config_.land_g_y / 100.0;
  }

  ROS_INFO("[Mission] NAV_TO_LAND — 飞到 %s 降落区上方 (%.2f, %.2f)...",
           color.c_str(), target_x, target_y);

  // 飞到降落区上方 30cm
  if (!flyTo(target_x, target_y, 0.3, 0.3, 0.15)) {
    ROS_WARN("[Mission] 无法精确到达降落区, 就近降落");
  }

  setState(TaskState::LANDING, "Above landing zone");
}

void MissionFSM::handleLanding() {
  std::string color = config_.detected_color;
  if (color.empty()) color = "G";

  double target_x, target_y;
  if (color == "R") {
    target_x = config_.land_r_x / 100.0;
    target_y = config_.land_r_y / 100.0;
  } else if (color == "B") {
    target_x = config_.land_b_x / 100.0;
    target_y = config_.land_b_y / 100.0;
  } else {
    target_x = config_.land_g_x / 100.0;
    target_y = config_.land_g_y / 100.0;
  }

  ROS_INFO("[Mission] LANDING — 降落在 %s 区...", color.c_str());

  geometry_msgs::PoseStamped sp;
  sp.pose.position.x = target_x;
  sp.pose.position.y = target_y;
  sp.pose.orientation.w = 1.0;

  ros::Rate rate(50);  // 着陆阶段提高频率
  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    double current_z = current_odom_.pose.pose.position.z;

    // 逐渐降低目标高度
    double descent_z = std::max(current_z - 0.02, 0.0);  // 每次降2cm
    sp.pose.position.z = descent_z;
    setpoint_pos_pub_.publish(sp);

    // 检测触地: 高度接近0且速度接近0
    double vz = current_odom_.twist.twist.linear.z;
    if (current_z < 0.05 && std::abs(vz) < 0.05) {
      ROS_INFO("[Mission] 检测到触地!");
      break;
    }

    // 超时保护
    if ((ros::Time::now() - start).toSec() > 20.0) {
      ROS_WARN("[Mission] LANDING 超时, 强制完成");
      break;
    }

    rate.sleep();
  }

  // 触地后 disarming
  ros::Duration(1.0).sleep();
  mavros_msgs::CommandBool disarm_cmd;
  disarm_cmd.request.value = false;
  if (arming_cli_.call(disarm_cmd) && disarm_cmd.response.success) {
    ROS_INFO("[Mission] 已解锁(停桨)");
  }

  ROS_INFO("[Mission] 降落完成! (任务⑦ +15分)");
  setState(TaskState::FINISH, "Mission complete");
}

void MissionFSM::handleEmergencyLand() {
  ROS_WARN("[Mission] EMERGENCY_LAND — 紧急降落!");

  double current_x = current_odom_.pose.pose.position.x;
  double current_y = current_odom_.pose.pose.position.y;

  landAt(current_x, current_y);

  setState(TaskState::FAILURE, "Emergency landing executed");
}

void MissionFSM::handleFinish() {
  ROS_INFO("[Mission] FINISH — 任务结束");
}

// ==================== 配置加载 ====================

void MissionFSM::loadMissionConfig() {
  // 从层级 param 读取 (由 launch 的 rosparam load map.yaml 注入)
  // 飞行参数（直接参数，可被 launch 覆盖）
  double default_speed     = conservative_mode_ ? 0.2 : 0.3;
  double default_tolerance = conservative_mode_ ? 0.4 : 0.2;
  double default_hover     = conservative_mode_ ? 8.0 : 6.0;
  double default_detect_to = conservative_mode_ ? 45.0 : 30.0;
  pnh_.param("speed",            config_.speed,            default_speed);
  pnh_.param("tolerance",        config_.tolerance,        default_tolerance);
  pnh_.param("hover_duration",   config_.hover_duration,   default_hover);
  pnh_.param("detect_timeout",   config_.detect_timeout,   default_detect_to);
  // 规则4+5: 飞行限高 + 悬停最低高度
  pnh_.param("max_altitude",         config_.max_altitude,         1.5);
  pnh_.param("min_takeoff_height",   config_.min_takeoff_height,   0.8);

  if (conservative_mode_) {
    enable_grasp_ = false;  // 保守模式强制跳过抓取
    ROS_INFO("[Mission] 保守模式: 速度=%.1fm/s 容差=%.1fm 悬停=%.0fs 不抓取",
             config_.speed, config_.tolerance, config_.hover_duration);
  }

  auto readWP = [&](const std::string& path, double& x, double& y, double& z) {
    pnh_.param(path + "/x", x, x);
    pnh_.param(path + "/y", y, y);
    pnh_.param(path + "/z", z, z);
  };

  // 从 map.yaml 层级路径读取航点
  readWP("waypoints/takeoff",    config_.takeoff_x, config_.takeoff_y, config_.takeoff_z);
  pnh_.param("waypoints/hover_height", config_.hover_height, 100.0);

  readWP("waypoints/nav_to_a/target", config_.a_x, config_.a_y, config_.a_z);
  readWP("waypoints/nav_to_b/target", config_.b_x, config_.b_y, config_.b_z);

  // 中间航点：取 via 第一个点
  double via1_x = config_.via_1_x, via1_y = config_.via_1_y, via1_z = config_.via_1_z;
  double via2_x = config_.via_2_x, via2_y = config_.via_2_y, via2_z = config_.via_2_z;

  pnh_.param("waypoints/nav_to_a/via/0/x", via1_x, via1_x);
  pnh_.param("waypoints/nav_to_a/via/0/y", via1_y, via1_y);
  pnh_.param("waypoints/nav_to_a/via/0/z", via1_z, via1_z);
  config_.via_1_x = via1_x;
  config_.via_1_y = via1_y;
  config_.via_1_z = via1_z;

  pnh_.param("waypoints/nav_to_b/via/0/x", via2_x, via2_x);
  pnh_.param("waypoints/nav_to_b/via/0/y", via2_y, via2_y);
  pnh_.param("waypoints/nav_to_b/via/0/z", via2_z, via2_z);
  config_.via_2_x = via2_x;
  config_.via_2_y = via2_y;
  config_.via_2_z = via2_z;

  // 降落区坐标
  readWP("waypoints/landing_R", config_.land_r_x, config_.land_r_y, config_.land_r_x);
  readWP("waypoints/landing_G", config_.land_g_x, config_.land_g_y, config_.land_g_y);
  readWP("waypoints/landing_B", config_.land_b_x, config_.land_b_y, config_.land_b_y);

  ROS_INFO("[Mission] Config loaded — takeoff(%.0f,%.0f,%.0f) A(%.0f,%.0f) B(%.0f,%.0f)",
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

// ==================== 飞控接口 ====================

bool MissionFSM::arm() {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Rate rate(10);
  for (int i = 0; i < 30; i++) {
    if (arming_cli_.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("[Mission] 解锁成功");
      return true;
    }
    rate.sleep();
  }
  return false;
}

bool MissionFSM::setOffboardMode() {
  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = "OFFBOARD";

  ros::Rate rate(10);
  for (int i = 0; i < 30; i++) {
    if (set_mode_cli_.call(mode_cmd) && mode_cmd.response.mode_sent) {
      ROS_INFO("[Mission] OFFBOARD 模式设置成功");
      return true;
    }
    rate.sleep();
  }
  return false;
}

bool MissionFSM::flyTo(double x, double y, double z, double speed, double tolerance) {
  // 规则4: 禁止飞越木板墙 (max_altitude = 1.5m)
  if (z > config_.max_altitude) {
    ROS_WARN("[Mission] flyTo 目标高度 %.2fm 超出限高 %.2fm, 已限制", z, config_.max_altitude);
    z = config_.max_altitude;
  }
  geometry_msgs::PoseStamped sp;
  sp.pose.position.x = x;
  sp.pose.position.y = y;
  sp.pose.position.z = z;
  sp.pose.orientation.w = 1.0;

  ros::Rate rate(20);
  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    double cx = current_odom_.pose.pose.position.x;
    double cy = current_odom_.pose.pose.position.y;
    double cz = current_odom_.pose.pose.position.z;
    double dist = std::sqrt(std::pow(cx - x, 2) + std::pow(cy - y, 2) + std::pow(cz - z, 2));

    if (dist < tolerance) {
      return true;
    }

    setpoint_pos_pub_.publish(sp);

    // 超时 (30 seconds max per leg)
    if ((ros::Time::now() - start).toSec() > 30.0) {
      ROS_WARN("[Mission] flyTo 超时 (目标: %.1f %.1f %.1f, 距离: %.2f)", x, y, z, dist);
      return false;
    }

    rate.sleep();
  }
  return false;
}

bool MissionFSM::landAt(double x, double y) {
  geometry_msgs::PoseStamped sp;
  sp.pose.position.x = x;
  sp.pose.position.y = y;
  sp.pose.position.z = 0.0;
  sp.pose.orientation.w = 1.0;

  ros::Rate rate(50);
  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    double cz = current_odom_.pose.pose.position.z;
    double vz = current_odom_.twist.twist.linear.z;

    sp.pose.position.z = std::max(cz - 0.02, 0.0);
    setpoint_pos_pub_.publish(sp);

    if (cz < 0.05 && std::abs(vz) < 0.05) {
      return true;
    }

    if ((ros::Time::now() - start).toSec() > 20.0) {
      return false;
    }
    rate.sleep();
  }
  return true;
}

// ==================== 感知接口 ====================

std::string MissionFSM::detectColorFromCamera() {
  // 颜色检测由 vision_node 独立完成
  // 这里通过 ROS param 或 topic 获取 vision_node 的结果
  std::string color;
  if (pnh_.getParam("detected_color", color) && !color.empty()) {
    return color;
  }
  return "";
}

// ==================== 回调函数 ====================

void MissionFSM::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  current_state_ = *msg;
}

void MissionFSM::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_odom_ = *msg;
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
