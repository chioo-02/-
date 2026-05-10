#include "vision_node/color_detector.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

/**
 * 颜色检测 ROS 节点
 *
 * 订阅: /camera/color/image_raw   — RGB 图像
 *       /camera/depth/image_rect_raw — 深度图
 *       /camera/color/camera_info  — 相机内参
 *
 * 发布: /vision/detected_color    — 检测到的颜色 (std_msgs/String)
 *       /vision/target_position    — 目标3D位置 (geometry_msgs/PointStamped)
 *       /vision/debug_image        — 调试图像
 */
class VisionNode {
public:
  VisionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : detector_(nh, pnh) {

    // 订阅
    rgb_sub_ = nh.subscribe(pnh.param<std::string>("camera_topic", "/camera/color/image_raw"),
                            1, &VisionNode::rgbCallback, this);
    depth_sub_ = nh.subscribe(pnh.param<std::string>("depth_topic", "/camera/depth/image_rect_raw"),
                              1, &VisionNode::depthCallback, this);
    cam_info_sub_ = nh.subscribe(pnh.param<std::string>("camera_info_topic", "/camera/color/camera_info"),
                                 1, &VisionNode::camInfoCallback, this);

    // 发布
    color_pub_ = nh.advertise<std_msgs::String>("/vision/detected_color", 1);
    position_pub_ = nh.advertise<geometry_msgs::PointStamped>("/vision/target_position", 1);
    debug_pub_ = nh.advertise<sensor_msgs::Image>("/vision/debug_image", 1);

    // 设置参数给 mission_node 获取
    pnh_ = ros::NodeHandle("~");  // 私有命名空间
    pnh_.setParam("detected_color", "");

    ROS_INFO("[Vision] VisionNode started");
  }

private:
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      latest_rgb_ = cv_ptr_->image;
      processFrame();
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge error: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
      depth_ptr_ = cv_bridge::toCvCopy(msg);
      latest_depth_ = depth_ptr_->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("depth cv_bridge error: %s", e.what());
    }
  }

  void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    latest_cam_info_ = *msg;
    has_cam_info_ = true;
  }

  void processFrame() {
    if (latest_rgb_.empty()) return;

    // 执行颜色检测
    auto result = detector_.detect(latest_rgb_);

    if (!result.color.empty()) {
      // 发布颜色结果
      std_msgs::String color_msg;
      color_msg.data = result.color;
      color_pub_.publish(color_msg);

      // 写入参数供 mission_node 读取
      pnh_.setParam("detected_color", result.color);

      ROS_DEBUG("[Vision] Detected: %s (conf: %.2f)", result.color.c_str(), result.confidence);

      // 如果有深度图，获取3D位置
      if (!latest_depth_.empty() && has_cam_info_) {
        cv::Point3d pos3d = detector_.get3DPosition(latest_depth_, result.center, latest_cam_info_);

        if (pos3d.z > 0) {
          geometry_msgs::PointStamped pos_msg;
          pos_msg.header.stamp = ros::Time::now();
          pos_msg.header.frame_id = "camera_color_optical_frame";
          pos_msg.point.x = pos3d.x;
          pos_msg.point.y = pos3d.y;
          pos_msg.point.z = pos3d.z;
          position_pub_.publish(pos_msg);
        }
      }

      // 发布调试图像（标注检测结果）
      if (debug_pub_.getNumSubscribers() > 0) {
        cv::Mat debug_img = latest_rgb_.clone();
        cv::circle(debug_img, result.center, 10, cv::Scalar(0, 255, 0), 2);
        cv::putText(debug_img, result.color, cv::Point(result.center.x - 20, result.center.y - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(
          std_msgs::Header(), "bgr8", debug_img).toImageMsg();
        debug_pub_.publish(debug_msg);
      }
    }
  }

  // ROS 通信
  ros::Subscriber rgb_sub_, depth_sub_, cam_info_sub_;
  ros::Publisher color_pub_, position_pub_, debug_pub_;
  ros::NodeHandle pnh_;

  // 图像数据
  cv_bridge::CvImagePtr cv_ptr_, depth_ptr_;
  cv::Mat latest_rgb_, latest_depth_;
  sensor_msgs::CameraInfo latest_cam_info_;
  bool has_cam_info_ = false;

  // 颜色检测器
  vision::ColorDetector detector_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  VisionNode node(nh, pnh);
  ros::spin();

  return 0;
}
