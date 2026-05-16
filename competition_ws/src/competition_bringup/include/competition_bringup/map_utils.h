#ifndef COMPETITION_MAP_UTILS_H
#define COMPETITION_MAP_UTILS_H

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

namespace competition {

// ==================== 数据结构 ====================

struct Point3D {
  double x, y, z;
  Point3D() : x(0), y(0), z(0) {}
  Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct Zone2D {
  std::string label;
  double cx, cy;       // 中心坐标 (cm)
  double sx, sy;       // 尺寸 (cm)
};

struct Waypoint {
  double x, y, z;      // 坐标 (cm)
  std::string desc;
};

struct ColorZone {
  std::string landing_name;
  double center_x, center_y;
  double score_x, score_y;
};

struct ObstacleInfo {
  std::string id;
  double x_min, x_max, y_min, y_max;
  double size_x, size_y, size_z;
  bool is_dynamic;
  std::string desc;
};

struct GateInfo {
  double cx, cy;         // 中心
  double z_min, z_max;   // 开口Z范围
  double opening_w, opening_h;
  std::string desc;
};

struct WallSegment {
  std::string along;     // "x" 或 "y"
  double x_min, x_max, y_min, y_max;
  double z_min, z_max;
};

// ==================== 地图管理类 ====================

class MapManager {
public:
  MapManager() {}

  bool load(const std::string& yaml_path) {
    try {
      YAML::Node config = YAML::LoadFile(yaml_path);
      loadArena(config["arena"]);
      loadZones(config["zones"]);
      loadWaypoints(config["waypoints"]);
      loadColorZones(config["color_zones"]);
      loadObstacles(config["obstacles"]);
      loadGates(config["gates"]);
      loadWalls(config["walls"]);
      loadSafety(config["safety"]);
      return true;
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to load map config: %s", e.what());
      return false;
    }
  }

  // ----- 坐标查询 -----
  Point3D getWaypoint(const std::string& name) const;
  Point3D getLandingPos(const std::string& color) const;
  ColorZone getColorZone(const std::string& color) const;
  GateInfo getGate(int id) const;  // id: 1 or 2

  // ----- 区域判断 -----
  bool isInZoneA(double x, double y) const;
  bool isInZoneB(double x, double y) const;
  bool isInTakeoffZone(double x, double y) const;
  bool isNearGate(double x, double y, double z, int gate_id, double threshold) const;

  // ----- 障碍物查询 -----
  std::vector<ObstacleInfo> getStaticObstacles() const;
  std::vector<ObstacleInfo> getDynamicObstacles() const;
  bool checkCollision(double x, double y, double z, double safety_margin) const;

  // ----- 路径辅助 -----
  std::vector<Point3D> getPathToA() const;   // 起飞→A 路径
  std::vector<Point3D> getPathToB() const;   // A→B 路径
  std::vector<Point3D> getPathToLanding(const std::string& color) const;

  // ----- 竞速门穿越辅助 -----
  Point3D getGateApproachPoint(int gate_id, double dist) const;
  Point3D getGateExitPoint(int gate_id, double dist) const;

  // 获取默认巡航高度
  double getCruiseHeight() const { return hover_height_; }

  // 安全限高 (规则4: 禁止飞越木板墙)
  double getMaxAltitude() const { return max_altitude_ / 100.0; }
  double getMinTakeoffHeight() const { return min_takeoff_height_ / 100.0; }

  // arena bounds in meters (for PX4/mavros, using meters)
  double getArenaLength() const { return length_x_ / 100.0; }
  double getArenaWidth() const { return width_y_ / 100.0; }
  double getArenaHeight() const { return height_z_ / 100.0; }

private:
  void loadArena(const YAML::Node& node);
  void loadZones(const YAML::Node& node);
  void loadWaypoints(const YAML::Node& node);
  void loadColorZones(const YAML::Node& node);
  void loadObstacles(const YAML::Node& node);
  void loadGates(const YAML::Node& node);
  void loadWalls(const YAML::Node& node);
  void loadSafety(const YAML::Node& node);

  // Arena
  double length_x_, width_y_, height_z_;
  double wall_height_, wall_bottom_;

  // Zones
  Zone2D takeoff_zone_, zone_a_, zone_b_;

  // Waypoints
  double hover_height_;
  Point3D takeoff_wp_, detect_hover_;
  std::vector<Point3D> path_a_, path_b_;
  Point3D landing_r_, landing_g_, landing_b_;

  // Color zones
  std::map<std::string, ColorZone> color_zones_;

  // Obstacles
  std::vector<ObstacleInfo> obstacles_;

  // Gates
  GateInfo gate1_, gate2_;

  // Walls
  std::vector<WallSegment> walls_;

  // Safety
  double obstacle_clearance_, gate_clearance_, wall_avoid_dist_;
  double max_altitude_, min_takeoff_height_;
};

// ==================== 内联实现 ====================

inline double dist2D(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2, dy = y1 - y2;
  return std::sqrt(dx*dx + dy*dy);
}

inline double dist3D(const Point3D& a, const Point3D& b) {
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// cm -> m (for PX4/mavros)
inline double cm2m(double cm) { return cm / 100.0; }
// m -> cm
inline double m2cm(double m) { return m * 100.0; }

}  // namespace competition

#endif  // COMPETITION_MAP_UTILS_H
