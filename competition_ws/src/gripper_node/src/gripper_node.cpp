#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

/**
 * 抓取机构控制节点
 *
 * 订阅: /gripper/command   — "grasp" / "release"
 * 发布: /gripper/state     — "grasped" / "released"
 *
 * 硬件接口: 通过 GPIO 或串口控制电磁铁/舵机
 *   电磁铁: GPIO 高低电平
 *   舵机:   PWM 信号 (如 SG90)
 *
 * 当前实现: 虚拟 GPIO (std::cout 模拟)，实机需替换为实际硬件驱动
 */
class GripperNode {
public:
  GripperNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : grasped_(false) {

    cmd_sub_ = nh.subscribe("/gripper/command", 1, &GripperNode::cmdCallback, this);
    state_pub_ = nh.advertise<std_msgs::String>("/gripper/state", 1);

    // 读取硬件引脚配置
    pnh.param<int>("gpio_pin", gpio_pin_, 18);

    ROS_INFO("[Gripper] Gripper node initialized (GPIO %d)", gpio_pin_);
    ROS_WARN("[Gripper] Using virtual GPIO — replace with actual hardware driver");
  }

private:
  void cmdCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "grasp" && !grasped_) {
      grasp();
    } else if (msg->data == "release" && grasped_) {
      release();
    } else {
      ROS_WARN("[Gripper] Unknown command: %s (state: %s)",
               msg->data.c_str(), grasped_ ? "grasped" : "released");
    }
  }

  void grasp() {
    // TODO: 实机替换为 GPIO 写高电平 (电磁铁) 或 PWM 控制 (舵机)
    ROS_INFO("[Gripper] Grasping...");
    grasped_ = true;

    std_msgs::String state_msg;
    state_msg.data = "grasped";
    state_pub_.publish(state_msg);
  }

  void release() {
    // TODO: 实机替换为 GPIO 写低电平 (电磁铁) 或 PWM 控制 (舵机)
    ROS_INFO("[Gripper] Releasing...");
    grasped_ = false;

    std_msgs::String state_msg;
    state_msg.data = "released";
    state_pub_.publish(state_msg);
  }

  ros::Subscriber cmd_sub_;
  ros::Publisher state_pub_;
  bool grasped_;
  int gpio_pin_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GripperNode node(nh, pnh);
  ros::spin();

  return 0;
}
