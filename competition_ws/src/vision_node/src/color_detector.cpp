#include "vision_node/color_detector.h"
#include <ros/ros.h>

namespace vision {

ColorDetector::ColorDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  // 读取参数
  pnh.param("min_contour_area", min_contour_area_, 500);
  pnh.param("max_contour_area", max_contour_area_, 100000);

  // 从参数服务器加载HSV阈值
  auto loadHSV = [&](const std::string& color) -> HSVRange {
    HSVRange range;
    std::vector<int> l, u;

    if (color == "red") {
      pnh.getParam("red/lower_1", l);
      pnh.getParam("red/upper_1", u);
      if (l.size() == 3 && u.size() == 3) {
        range.lower = cv::Scalar(l[0], l[1], l[2]);
        range.upper = cv::Scalar(u[0], u[1], u[2]);
      }
      pnh.getParam("red/lower_2", l);
      pnh.getParam("red/upper_2", u);
      if (l.size() == 3 && u.size() == 3) {
        range.lower2 = cv::Scalar(l[0], l[1], l[2]);
        range.upper2 = cv::Scalar(u[0], u[1], u[2]);
      }
      range.two_ranges = true;
    } else {
      pnh.getParam(color + "/lower", l);
      pnh.getParam(color + "/upper", u);
      if (l.size() == 3 && u.size() == 3) {
        range.lower = cv::Scalar(l[0], l[1], l[2]);
        range.upper = cv::Scalar(u[0], u[1], u[2]);
      }
      range.two_ranges = false;
    }
    return range;
  };

  hsv_ranges_["red"]   = loadHSV("red");
  hsv_ranges_["green"] = loadHSV("green");
  hsv_ranges_["blue"]  = loadHSV("blue");

  ROS_INFO("[Vision] ColorDetector initialized");
}

ColorResult ColorDetector::detect(const cv::Mat& rgb_image) {
  ColorResult result;
  result.color = "";
  result.confidence = 0.0;

  if (rgb_image.empty()) return result;

  cv::Mat hsv;
  cv::cvtColor(rgb_image, hsv, cv::COLOR_BGR2HSV);

  // 检测三种颜色
  std::vector<std::pair<std::string, cv::Mat>> masks;
  masks.emplace_back("R", thresholdRed(hsv));
  masks.emplace_back("G", thresholdColor(hsv, "green"));
  masks.emplace_back("B", thresholdColor(hsv, "blue"));

  // 找出面积最大的颜色
  double max_area = 0;
  cv::Point2d best_center;

  for (auto& m : masks) {
    cv::Point2d center;
    double area = findLargestContour(m.second, center);

    if (area > max_area && area >= min_contour_area_ && area <= max_contour_area_) {
      max_area = area;
      result.color = m.first;
      result.center = center;
    }
  }

  if (!result.color.empty()) {
    result.confidence = std::min(max_area / 50000.0, 1.0);
  }

  return result;
}

cv::Mat ColorDetector::thresholdColor(const cv::Mat& hsv_image, const std::string& color) {
  auto it = hsv_ranges_.find(color);
  if (it == hsv_ranges_.end()) return cv::Mat();

  cv::Mat mask;
  cv::inRange(hsv_image, it->second.lower, it->second.upper, mask);

  // 可选: 形态学降噪
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

  return mask;
}

cv::Mat ColorDetector::thresholdRed(const cv::Mat& hsv_image) {
  auto& red = hsv_ranges_["red"];

  cv::Mat mask1, mask2;
  cv::inRange(hsv_image, red.lower, red.upper, mask1);
  cv::inRange(hsv_image, red.lower2, red.upper2, mask2);

  cv::Mat mask = mask1 | mask2;

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

  return mask;
}

double ColorDetector::findLargestContour(const cv::Mat& mask, cv::Point2d& center) {
  if (mask.empty()) return 0;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double max_area = 0;
  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area > max_area) {
      max_area = area;
      cv::Moments m = cv::moments(contour);
      if (m.m00 > 0) {
        center.x = m.m10 / m.m00;
        center.y = m.m01 / m.m00;
      }
    }
  }

  return max_area;
}

cv::Point3d ColorDetector::get3DPosition(const cv::Mat& depth_image, cv::Point2d pixel,
                                          const sensor_msgs::CameraInfo& cam_info) {
  cv::Point3d pos(0, 0, 0);

  if (depth_image.empty()) return pos;

  float depth_mm = getDepthValue(depth_image, pixel);
  if (depth_mm <= 0) return pos;

  double depth_m = depth_mm / 1000.0;

  // 根据相机内参反投影
  double fx = cam_info.P[0];
  double fy = cam_info.P[5];
  double cx = cam_info.P[2];
  double cy = cam_info.P[6];

  pos.x = (pixel.x - cx) * depth_m / fx;
  pos.y = (pixel.y - cy) * depth_m / fy;
  pos.z = depth_m;

  return pos;
}

float ColorDetector::getDepthValue(const cv::Mat& depth_image, cv::Point2d pixel) {
  int x = cvRound(pixel.x);
  int y = cvRound(pixel.y);

  if (x < 0 || x >= depth_image.cols || y < 0 || y >= depth_image.rows) {
    return -1;
  }

  // Realsense 深度图为 16UC1 (mm)
  if (depth_image.type() == CV_16UC1) {
    return depth_image.at<uint16_t>(y, x);
  }
  // 32FC1 (m)
  else if (depth_image.type() == CV_32FC1) {
    return depth_image.at<float>(y, x) * 1000.0f;
  }

  return -1;
}

void ColorDetector::setHSVThresholds(const std::string& color,
                                      const cv::Scalar& lower, const cv::Scalar& upper) {
  if (hsv_ranges_.count(color)) {
    hsv_ranges_[color].lower = lower;
    hsv_ranges_[color].upper = upper;
  }
}

cv::Mat ColorDetector::getDebugMask(const cv::Mat& rgb_image, const std::string& color) {
  if (rgb_image.empty()) return cv::Mat();

  cv::Mat hsv;
  cv::cvtColor(rgb_image, hsv, cv::COLOR_BGR2HSV);

  if (color == "R") return thresholdRed(hsv);
  return thresholdColor(hsv, color);
}

}  // namespace vision
