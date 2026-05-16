#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

namespace mission {

// ==================== 任务状态定义 ====================

enum class TaskState {
  IDLE,
  ARMING,
  TAKEOFF,
  NAV_TO_A,
  DETECT_COLOR,
  GRASP_BALL,
  PASS_GATE1,
  NAV_TO_B,
  PASS_GATE2,
  DROP_AND_OUTPUT,
  LANDING,
  FINISH,
  EMERGENCY_LAND,
  FAILURE
};

inline std::string stateToString(TaskState s) {
  switch (s) {
    case TaskState::IDLE:           return "IDLE";
    case TaskState::ARMING:         return "ARMING";
    case TaskState::TAKEOFF:        return "TAKEOFF";
    case TaskState::NAV_TO_A:       return "NAV_TO_A";
    case TaskState::DETECT_COLOR:   return "DETECT_COLOR";
    case TaskState::GRASP_BALL:     return "GRASP_BALL";
    case TaskState::PASS_GATE1:     return "PASS_GATE1";
    case TaskState::NAV_TO_B:       return "NAV_TO_B";
    case TaskState::PASS_GATE2:     return "PASS_GATE2";
    case TaskState::DROP_AND_OUTPUT:return "DROP_AND_OUTPUT";
    case TaskState::LANDING:        return "LANDING";
    case TaskState::FINISH:         return "FINISH";
    case TaskState::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case TaskState::FAILURE:        return "FAILURE";
  }
  return "UNKNOWN";
}

// ==================== 航点结构 ====================

struct Waypoint {
  double x, y, z;   // VINS坐标系 (m), 原点=起飞点
  double yaw = 0;
};

// ==================== 任务配置 ====================

struct MissionConfig {
  // 坐标 (cm, map.yaml 坐标系)
  double takeoff_x = 30, takeoff_y = 30, takeoff_z = 80;

  double a_x = 130, a_y = 400, a_z = 100;
  double b_x = 500, b_y = 130, b_z = 100;

  // 路径经由点 (cm, map.yaml 坐标系)
  std::vector<Waypoint> via_a;  // H→A 途经点
  std::vector<Waypoint> via_b;  // A→B 途经点

  // 球坐标 (cm)
  double ball_x = 62.5, ball_y = 387.5, ball_z = 100;

  // 降落区坐标 (cm)
  double land_r_x = 445, land_r_y = 100;
  double land_g_x = 500, land_g_y = 190;
  double land_b_x = 545, land_b_y = 100;

  // 竞速门坐标 (cm)
  double gate1_x = 125, gate1_y = 410, gate1_z = 100;
  double gate2_x = 400, gate2_y = 385, gate2_z = 100;

  // 得分框坐标 (cm, 颜色区中心)
  double score_r_x = 445, score_r_y = 100;
  double score_g_x = 500, score_g_y = 190;
  double score_b_x = 545, score_b_y = 100;

  // 飞行参数
  double speed = 0.5;
  double tolerance = 0.3;
  double hover_duration = 6.0;
  double detect_timeout = 15.0;
  double grasp_timeout = 10.0;
  double max_altitude = 1.5;
  double min_takeoff_height = 0.8;
  double gate_tolerance = 0.15;

  // 降落参数
  double landing_descend_speed = 0.15;
  double landing_touchdown_thresh = 0.05;

  // 颜色配置
  std::string detected_color = "";
};

// ==================== 任务状态机类 ====================

class MissionFSM {
public:
  MissionFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MissionFSM() = default;

  bool init();
  void run();

  TaskState getState() const { return state_; }
  std::string getStateStr() const { return stateToString(state_); }

private:
  // 状态处理函数
  void handleIdle();
  void handleArming();
  void handleTakeoff();
  void handleNavToA();
  void handleDetectColor();
  void handleGraspBall();
  void handlePassGate1();
  void handleNavToB();
  void handlePassGate2();
  void handleDropAndOutput();
  void handleLanding();
  void handleEmergencyLand();
  void handleFinish();

  // 状态转换
  void setState(TaskState new_state, const std::string& reason = "");
  bool checkTimeout(ros::Time start, double timeout);

  // ----- 坐标转换 -----
  // map.yaml cm → VINS m (减去起飞点偏移)
  inline double mapToVinsX(double cm) const { return (cm - config_.takeoff_x) / 100.0; }
  inline double mapToVinsY(double cm) const { return (cm - config_.takeoff_y) / 100.0; }
  inline double mapToVinsZ(double cm) const { return cm / 100.0; }

  // ----- 飞控接口 (Fast-Drone-250: EGO-Planner waypoint 模式) -----
  void publishWaypoint(double x_vins, double y_vins, double z_vins, double yaw = 0);
  void publishWaypointSequence(const std::vector<Waypoint>& waypoints, double speed = 0.5);
  bool waitArrived(double x, double y, double z, double tolerance, double timeout);
  bool descendToTouchdown(double target_x, double target_y, double timeout = 20.0);

  // ----- 感知接口 -----
  void colorCallback(const std_msgs::String::ConstPtr& msg);
  void vinsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  double currentX() const { return current_odom_.pose.pose.position.x; }
  double currentY() const { return current_odom_.pose.pose.position.y; }
  double currentZ() const { return current_odom_.pose.pose.position.z; }

  // 分数追踪
  void addScore(int points, const std::string& reason);

  // ----- 配置 -----
  void loadMissionConfig();

  // ROS 句柄
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 订阅
  ros::Subscriber vins_odom_sub_;   // Fast-Drone-250: VINS odometry
  ros::Subscriber color_sub_;       // /vision/detected_color
  ros::Subscriber gripper_state_sub_; // /gripper/state

  // 发布
  ros::Publisher waypoint_pub_;     // /move_base_simple/goal → EGO-Planner (flight_type=1)
  ros::Publisher gripper_cmd_pub_;  // /gripper/command
  ros::Publisher score_pub_;        // /mission/score (for logging)

  // 状态
  TaskState state_ = TaskState::IDLE;
  MissionConfig config_;
  nav_msgs::Odometry current_odom_;

  // 计时
  ros::Time state_start_time_;
  ros::Time mission_start_time_;

  // 最新颜色
  std::string latest_color_;

  // 模式开关
  bool enable_grasp_ = false;
  bool conservative_mode_ = true;
  bool gate1_passed_ = false;
  bool gate2_passed_ = false;
  bool ball_held_ = false;

  // 分数追踪
  int total_score_ = 0;
  int collision_count_ = 0;
};

}  // namespace mission

#endif  // MISSION_FSM_H
