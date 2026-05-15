#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>

namespace mission {

// ==================== 任务状态定义 ====================

enum class TaskState {
  IDLE,
  ARMING,
  TAKEOFF,
  NAV_TO_A,
  DETECT_COLOR,
  GRASP_BALL,         // 可选
  NAV_TO_B,
  DROP_AND_OUTPUT,    // 投放+颜色输出
  NAV_TO_LAND,
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
    case TaskState::NAV_TO_B:       return "NAV_TO_B";
    case TaskState::DROP_AND_OUTPUT:return "DROP_AND_OUTPUT";
    case TaskState::NAV_TO_LAND:    return "NAV_TO_LAND";
    case TaskState::LANDING:        return "LANDING";
    case TaskState::FINISH:         return "FINISH";
    case TaskState::EMERGENCY_LAND: return "EMERGENCY_LAND";
    case TaskState::FAILURE:        return "FAILURE";
  }
  return "UNKNOWN";
}

// ==================== 任务配置 ====================

struct MissionConfig {
  // 坐标 (cm)
  double takeoff_x = 30, takeoff_y = 30, takeoff_z = 80;
  double hover_height = 100;
  double a_x = 130, a_y = 400, a_z = 100;
  double b_x = 500, b_y = 130, b_z = 100;
  double via_1_x = 30, via_1_y = 200, via_1_z = 100;   // 绕开挡板1
  double via_2_x = 300, via_2_y = 250, via_2_z = 100;  // 经停中点
  double land_r_x = 445, land_r_y = 100;
  double land_g_x = 500, land_g_y = 190;
  double land_b_x = 545, land_b_y = 100;

  // 飞行参数
  double speed = 0.5;             // m/s
  double tolerance = 0.3;         // 到达判定距离 (m)
  double hover_duration = 6.0;    // 悬停秒数 (>5秒)
  double detect_timeout = 30.0;   // 颜色检测超时
  double grasp_timeout = 10.0;    // 抓取超时
  double emergency_timeout = 60.0;// 紧急情况处理时间

  // 颜色配置
  std::string detected_color = "";  // 检测到的颜色 R/G/B
};

// ==================== 任务状态机类 ====================

class MissionFSM {
public:
  MissionFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MissionFSM() = default;

  bool init();
  void run();  // 执行任务（阻塞，直到完成或失败）

  // 获取当前状态
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
  void handleNavToB();
  void handleDropAndOutput();
  void handleNavToLand();
  void handleLanding();
  void handleEmergencyLand();
  void handleFinish();

  // 状态转换
  void setState(TaskState new_state, const std::string& reason = "");
  bool checkTimeout(ros::Time start, double timeout);

  void loadMissionConfig();

  // ----- 飞控接口 -----
  bool arm();
  bool setOffboardMode();
  bool flyTo(double x, double y, double z, double speed, double tolerance);
  bool flyToVia(const std::vector<geometry_msgs::PoseStamped>& waypoints,
                double speed, double tolerance);
  bool landAt(double x, double y);

  // ----- 感知接口 -----
  std::string detectColorFromCamera();
  bool confirmPosition(double x, double y, double z, double tolerance);

  // ----- 回调函数 -----
  void stateCallback(const mavros_msgs::State::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  // ROS 句柄
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 订阅
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;

  // 发布
  ros::Publisher setpoint_pos_pub_;
  ros::Publisher setpoint_vel_pub_;

  // 服务客户端
  ros::ServiceClient arming_cli_;
  ros::ServiceClient set_mode_cli_;

  // 状态
  TaskState state_ = TaskState::IDLE;
  MissionConfig config_;
  mavros_msgs::State current_state_;
  nav_msgs::Odometry current_odom_;

  // 计时
  ros::Time state_start_time_;

  // 是否启用抓取（可选）
  bool enable_grasp_ = false;
};

}  // namespace mission

#endif  // MISSION_FSM_H
