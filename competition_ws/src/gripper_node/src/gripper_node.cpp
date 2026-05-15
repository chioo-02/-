#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/**
 * 抓取机构控制节点 — 实机版
 *
 * 支持两种硬件模式:
 *   electromagnet — GPIO 高低电平控制电磁铁 (默认)
 *   servo         — PCA9685 I2C PWM 控制舵机
 *
 * 订阅: /gripper/command  ("grasp" / "release")
 * 发布: /gripper/state    ("grasped" / "released" / "error")
 */

// ==================== GPIO 电磁铁 ====================

class GPIOMagnet {
public:
  GPIOMagnet(int pin) : pin_(pin), exported_(false) {}

  bool init() {
    // 导出 GPIO
    std::ofstream exp("/sys/class/gpio/export");
    if (!exp) { ROS_ERROR("[Gripper] Cannot open GPIO export"); return false; }
    exp << pin_;
    exp.close();

    // 等待 sysfs 创建
    usleep(100000);

    // 设为输出
    std::string dir_path = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/direction";
    std::ofstream dir(dir_path);
    if (!dir) {
      ROS_ERROR("[Gripper] Cannot set GPIO%d direction", pin_);
      return false;
    }
    dir << "out";
    dir.close();

    exported_ = true;
    ROS_INFO("[Gripper] GPIO%d exported as electromagnet", pin_);
    return true;
  }

  void set(bool high) {
    if (!exported_) return;
    std::ofstream val("/sys/class/gpio/gpio" + std::to_string(pin_) + "/value");
    if (val) val << (high ? "1" : "0");
  }

  ~GPIOMagnet() {
    if (exported_) {
      set(false);
      std::ofstream unexp("/sys/class/gpio/unexport");
      if (unexp) unexp << pin_;
    }
  }

private:
  int pin_;
  bool exported_;
};

// ==================== PCA9685 舵机 ====================

class PCA9685Servo {
public:
  PCA9685Servo(int channel, int min_pulse, int max_pulse)
    : channel_(channel), min_pulse_(min_pulse), max_pulse_(max_pulse), fd_(-1) {}

  bool init(uint8_t i2c_addr = 0x40) {
    fd_ = open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0) { ROS_ERROR("[Gripper] Cannot open /dev/i2c-1"); return false; }

    if (ioctl(fd_, I2C_SLAVE, i2c_addr) < 0) {
      ROS_ERROR("[Gripper] PCA9685 not found at 0x%02x", i2c_addr);
      close(fd_); fd_ = -1;
      return false;
    }

    // 初始化 PCA9685: 50Hz, 自动增量
    writeReg(0x00, 0x11);  // MODE1: sleep + auto-increment
    usleep(5000);
    writeReg(0xFE, 121);   // PRE_SCALE: ~50Hz (25MHz / 4096 / 50 ≈ 122)
    writeReg(0x00, 0x21);  // MODE1: auto-increment + restart

    ROS_INFO("[Gripper] PCA9685 initialized on ch%d", channel_);
    return true;
  }

  void setAngle(double angle_deg) {
    if (fd_ < 0) return;
    // 角度 → 脉宽 → 计数值 (4096 / 20ms * pulse_ms)
    double clamped = std::max(0.0, std::min(180.0, angle_deg));
    int pulse = min_pulse_ + (max_pulse_ - min_pulse_) * clamped / 180.0;
    int count = pulse * 4096 / 20000;

    uint8_t reg = 0x06 + 4 * channel_;
    writeReg(reg, 0);              // ON_L, ON_H
    writeReg(reg + 1, 0);
    writeReg(reg + 2, count & 0xFF);       // OFF_L
    writeReg(reg + 3, (count >> 8) & 0x0F); // OFF_H
  }

  ~PCA9685Servo() { if (fd_ >= 0) close(fd_); }

private:
  void writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(fd_, buf, 2) != 2) {
      ROS_WARN("[Gripper] I2C write failed reg=0x%02x", reg);
    }
  }

  int fd_, channel_, min_pulse_, max_pulse_;
};

// ==================== ROS 节点 ====================

class GripperNode {
public:
  GripperNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : grasped_(false), mode_(ELECTROMAGNET) {

    cmd_sub_  = nh.subscribe("/gripper/command", 1, &GripperNode::cmdCallback, this);
    state_pub_ = nh.advertise<std_msgs::String>("/gripper/state", 1, true);

    // 读取配置
    std::string type;
    pnh.param<std::string>("gripper_type", type, "electromagnet");
    pnh.param<int>("gpio_pin", gpio_pin_, 18);
    pnh.param<int>("servo_channel", servo_ch_, 0);

    // 初始化硬件
    bool ok = false;
    if (type == "servo") {
      mode_ = SERVO;
      servo_ = new PCA9685Servo(servo_ch_, 500, 2500);  // SG90: 500~2500us
      ok = servo_->init();
      if (ok) servo_->setAngle(0);  // 初始闭合
    } else {
      mode_ = ELECTROMAGNET;
      magnet_ = new GPIOMagnet(gpio_pin_);
      ok = magnet_->init();
    }

    if (ok) {
      ROS_INFO("[Gripper] Ready (%s mode)", type.c_str());
      publishState("released");
    } else {
      ROS_ERROR("[Gripper] Hardware init failed!");
      publishState("error");
    }
  }

  ~GripperNode() {
    delete magnet_;
    delete servo_;
  }

private:
  void cmdCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "grasp") {
      grasp();
    } else if (msg->data == "release") {
      release();
    } else {
      ROS_WARN("[Gripper] Unknown command: %s", msg->data.c_str());
    }
  }

  void grasp() {
    if (grasped_) return;
    ROS_INFO("[Gripper] Grasping...");

    if (mode_ == ELECTROMAGNET) {
      magnet_->set(true);  // GPIO 高电平, 电磁铁通电
    } else {
      servo_->setAngle(90);  // 舵机转到闭合角度
    }

    grasped_ = true;
    publishState("grasped");
  }

  void release() {
    if (!grasped_) return;
    ROS_INFO("[Gripper] Releasing...");

    if (mode_ == ELECTROMAGNET) {
      magnet_->set(false);  // GPIO 低电平, 电磁铁断电
    } else {
      servo_->setAngle(0);  // 舵机转到张开角度
    }

    grasped_ = false;
    publishState("released");
  }

  void publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  ros::Subscriber cmd_sub_;
  ros::Publisher  state_pub_;
  bool grasped_;

  enum Mode { ELECTROMAGNET, SERVO };
  Mode mode_;

  GPIOMagnet*   magnet_ = nullptr;
  PCA9685Servo* servo_  = nullptr;
  int gpio_pin_, servo_ch_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GripperNode node(nh, pnh);
  ros::spin();

  return 0;
}
