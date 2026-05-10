#ifndef COLOR_DETECTOR_H
#define COLOR_DETECTOR_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace vision {

/**
 * HSV 颜色检测结果
 */
struct ColorResult {
  std::string color;      // "R", "G", "B" 或 ""
  double confidence;      // 0.0 ~ 1.0
  cv::Point2d center;     // 图像中目标中心 (像素)
  cv::Point3d position;   // 世界坐标系中的3D位置 (m)
  bool has_depth;         // 是否有深度数据
};

/**
 * HSV 颜色检测器
 *
 * 检测 R/G/B 三色，找出图像中面积最大的纯色区域。
 * 可配合深度图获取目标3D位置。
 */
class ColorDetector {
public:
  ColorDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  /** 检测图像中的主要颜色 */
  ColorResult detect(const cv::Mat& rgb_image);

  /** 获取深度图中指定像素位置的3D坐标 */
  cv::Point3d get3DPosition(const cv::Mat& depth_image, cv::Point2d pixel,
                            const sensor_msgs::CameraInfo& cam_info);

  /** 从深度图像中获取指定像素的深度值 (mm) */
  float getDepthValue(const cv::Mat& depth_image, cv::Point2d pixel);

  /** 设置HSV阈值 */
  void setHSVThresholds(const std::string& color,
                        const cv::Scalar& lower, const cv::Scalar& upper);

  /** 调试: 显示二值化Mask */
  cv::Mat getDebugMask(const cv::Mat& rgb_image, const std::string& color);

private:
  /** 对单色做阈值分割 */
  cv::Mat thresholdColor(const cv::Mat& hsv_image, const std::string& color);

  /** 找最大连通域 */
  double findLargestContour(const cv::Mat& mask, cv::Point2d& center);

  /** 红色检测（特殊处理：红色在HSV两端） */
  cv::Mat thresholdRed(const cv::Mat& hsv_image);

  // HSV 阈值
  struct HSVRange {
    cv::Scalar lower;
    cv::Scalar upper;
    bool two_ranges;  // 红色需要两个范围
    cv::Scalar lower2;
    cv::Scalar upper2;
  };

  std::map<std::string, HSVRange> hsv_ranges_;

  // 参数
  int min_contour_area_;
  int max_contour_area_;
};

}  // namespace vision

#endif  // COLOR_DETECTOR_H
