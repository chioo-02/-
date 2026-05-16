#include "competition_bringup/map_utils.h"
#include <algorithm>

namespace competition {

// ==================== YAML 加载函数 ====================

void MapManager::loadArena(const YAML::Node& node) {
  length_x_ = node["length_x"].as<double>(600);
  width_y_ = node["width_y"].as<double>(500);
  height_z_ = node["height_z"].as<double>(250);
  wall_height_ = node["wall_height"].as<double>(150);
  wall_bottom_ = node["wall_bottom"].as<double>(30);
}

void MapManager::loadZones(const YAML::Node& node) {
  if (node["takeoff"]) {
    takeoff_zone_.cx = node["takeoff"]["center"][0].as<double>();
    takeoff_zone_.cy = node["takeoff"]["center"][1].as<double>();
    takeoff_zone_.sx = node["takeoff"]["size"][0].as<double>();
    takeoff_zone_.sy = node["takeoff"]["size"][1].as<double>();
  }
  if (node["zone_A"]) {
    zone_a_.cx = node["zone_A"]["center"][0].as<double>();
    zone_a_.cy = node["zone_A"]["center"][1].as<double>();
  }
  if (node["zone_B"]) {
    zone_b_.cx = node["zone_B"]["center"][0].as<double>();
    zone_b_.cy = node["zone_B"]["center"][1].as<double>();
  }
}

void MapManager::loadWaypoints(const YAML::Node& node) {
  hover_height_ = node["hover_height"].as<double>(100);

  auto readPoint = [&](const YAML::Node& n) -> Point3D {
    return Point3D(n["x"].as<double>(), n["y"].as<double>(), n["z"].as<double>());
  };

  // Takeoff waypoint
  takeoff_wp_ = readPoint(node["takeoff"]);

  // Path to A (via points + target)
  path_a_.clear();
  if (node["nav_to_a"]["via"]) {
    for (const auto& pt : node["nav_to_a"]["via"]) {
      path_a_.push_back(readPoint(pt));
    }
  }
  path_a_.push_back(readPoint(node["nav_to_a"]["target"]));

  // Detect hover
  detect_hover_ = readPoint(node["detect_hover"]);

  // Path to B (via points + target)
  path_b_.clear();
  if (node["nav_to_b"]["via"]) {
    for (const auto& pt : node["nav_to_b"]["via"]) {
      path_b_.push_back(readPoint(pt));
    }
  }
  path_b_.push_back(readPoint(node["nav_to_b"]["target"]));

  // Landing points
  landing_r_ = readPoint(node["landing_R"]);
  landing_g_ = readPoint(node["landing_G"]);
  landing_b_ = readPoint(node["landing_B"]);
}

void MapManager::loadColorZones(const YAML::Node& node) {
  static const std::string colors[] = {"R", "G", "B"};
  for (const auto& c : colors) {
    if (!node[c]) continue;
    ColorZone cz;
    cz.landing_name = node[c]["landing"].as<std::string>();
    cz.center_x = node[c]["center"][0].as<double>();
    cz.center_y = node[c]["center"][1].as<double>();
    cz.score_x = node[c]["score_x"].as<double>();
    cz.score_y = node[c]["score_y"].as<double>();
    color_zones_[c] = cz;
  }
}

void MapManager::loadObstacles(const YAML::Node& node) {
  obstacles_.clear();
  const auto& positions = node["possible_positions"];
  for (const auto& kv : positions) {
    ObstacleInfo obs;
    obs.id = kv.first.as<std::string>();
    obs.x_min = kv.second["x_range"][0].as<double>();
    obs.x_max = kv.second["x_range"][1].as<double>();
    obs.y_min = kv.second["y_range"][0].as<double>();
    obs.y_max = kv.second["y_range"][1].as<double>();
    obs.is_dynamic = (kv.second["type"].as<std::string>() == "dynamic");
    obs.desc = kv.second["desc"].as<std::string>();
    obs.size_x = 30;
    obs.size_y = 30;
    obs.size_z = 180;
    obstacles_.push_back(obs);
  }
}

void MapManager::loadGates(const YAML::Node& node) {
  const auto loadGate = [&](const YAML::Node& n) -> GateInfo {
    GateInfo g;
    g.cx = n["center"][0].as<double>();
    g.cy = n["center"][1].as<double>();
    g.z_min = n["opening_z_min"].as<double>();
    g.z_max = n["opening_z_max"].as<double>();
    g.opening_w = n["opening_size"][0].as<double>();
    g.opening_h = n["opening_size"][1].as<double>();
    g.desc = n["desc"].as<std::string>();
    return g;
  };
  gate1_ = loadGate(node["gate1"]);
  gate2_ = loadGate(node["gate2"]);
}

void MapManager::loadWalls(const YAML::Node& node) {
  walls_.clear();
  for (const auto& kv : node) {
    const auto& w = kv.second();
    WallSegment ws;
    ws.along = w["along"].as<std::string>();
    ws.x_min = w["x_range"][0].as<double>();
    ws.x_max = w["x_range"][1].as<double>();
    // y could be a single value or a range
    ws.y_min = w["y_range"] ? w["y_range"][0].as<double>() : w["y"].as<double>();
    ws.y_max = w["y_range"] ? w["y_range"][1].as<double>() : w["y"].as<double>();
    ws.z_min = w["z_range"][0].as<double>();
    ws.z_max = w["z_range"][1].as<double>();
    walls_.push_back(ws);
  }
}

void MapManager::loadSafety(const YAML::Node& node) {
  obstacle_clearance_ = node["obstacle_clearance"].as<double>(30);
  gate_clearance_ = node["gate_clearance"].as<double>(15);
  wall_avoid_dist_ = node["wall_avoid_distance"].as<double>(40);
  max_altitude_ = node["max_altitude"].as<double>(150);
  min_takeoff_height_ = node["min_takeoff_height"].as<double>(80);
}

// ==================== 坐标查询 ====================

Point3D MapManager::getWaypoint(const std::string& name) const {
  if (name == "takeoff") return takeoff_wp_;
  if (name == "detect") return detect_hover_;
  if (name == "a_target" && !path_a_.empty()) return path_a_.back();
  if (name == "b_target" && !path_b_.empty()) return path_b_.back();
  if (name == "landing_R") return landing_r_;
  if (name == "landing_G") return landing_g_;
  if (name == "landing_B") return landing_b_;
  ROS_WARN("Unknown waypoint: %s", name.c_str());
  return Point3D(0, 0, hover_height_);
}

Point3D MapManager::getLandingPos(const std::string& color) const {
  if (color == "R") return landing_r_;
  if (color == "G") return landing_g_;
  if (color == "B") return landing_b_;
  ROS_WARN("Unknown color for landing: %s", color.c_str());
  return landing_g_;  // default to green
}

ColorZone MapManager::getColorZone(const std::string& color) const {
  auto it = color_zones_.find(color);
  if (it != color_zones_.end()) return it->second;
  ROS_WARN("Unknown color zone: %s", color.c_str());
  static ColorZone default_cz;
  return default_cz;
}

GateInfo MapManager::getGate(int id) const {
  if (id == 1) return gate1_;
  return gate2_;
}

// ==================== 区域判断 ====================

bool MapManager::isInZoneA(double x, double y) const {
  return dist2D(x, y, zone_a_.cx, zone_a_.cy) < 80.0;  // 半径80cm
}

bool MapManager::isInZoneB(double x, double y) const {
  return dist2D(x, y, zone_b_.cx, zone_b_.cy) < 80.0;
}

bool MapManager::isInTakeoffZone(double x, double y) const {
  return std::abs(x - takeoff_zone_.cx) < takeoff_zone_.sx / 2 &&
         std::abs(y - takeoff_zone_.cy) < takeoff_zone_.sy / 2;
}

bool MapManager::isNearGate(double x, double y, double z, int gate_id, double threshold) const {
  const auto& g = (gate_id == 1) ? gate1_ : gate2_;
  return dist2D(x, y, g.cx, g.cy) < threshold &&
         z > g.z_min && z < g.z_max;
}

// ==================== 障碍物查询 ====================

std::vector<ObstacleInfo> MapManager::getStaticObstacles() const {
  std::vector<ObstacleInfo> result;
  for (const auto& obs : obstacles_) {
    if (!obs.is_dynamic) result.push_back(obs);
  }
  return result;
}

std::vector<ObstacleInfo> MapManager::getDynamicObstacles() const {
  std::vector<ObstacleInfo> result;
  for (const auto& obs : obstacles_) {
    if (obs.is_dynamic) result.push_back(obs);
  }
  return result;
}

bool MapManager::checkCollision(double x, double y, double z, double margin) const {
  for (const auto& obs : obstacles_) {
    bool in_x = x >= (obs.x_min - margin) && x <= (obs.x_max + obs.size_x + margin);
    bool in_y = y >= (obs.y_min - margin) && y <= (obs.y_max + obs.size_y + margin);
    bool in_z = z >= 0 && z <= (obs.size_z + margin);
    if (in_x && in_y && in_z) return true;
  }
  // Check walls
  for (const auto& w : walls_) {
    bool in_x = x >= w.x_min && x <= w.x_max;
    bool in_y = y >= w.y_min && y <= w.y_max;
    bool in_z = z >= w.z_min && z <= w.z_max;
    if (in_x && in_y && in_z) return true;
  }
  return false;
}

// ==================== 路径辅助 ====================

std::vector<Point3D> MapManager::getPathToA() const {
  std::vector<Point3D> path;
  path.push_back(takeoff_wp_);
  path.insert(path.end(), path_a_.begin(), path_a_.end());
  return path;
}

std::vector<Point3D> MapManager::getPathToB() const {
  // 从识别悬停点出发 → 经由中转到B
  std::vector<Point3D> path;
  path.push_back(detect_hover_);
  path.insert(path.end(), path_b_.begin(), path_b_.end());
  return path;
}

std::vector<Point3D> MapManager::getPathToLanding(const std::string& color) const {
  std::vector<Point3D> path;
  auto land_pos = getLandingPos(color);
  // 先到降落区上方50cm，再下降
  path.push_back(Point3D(land_pos.x, land_pos.y, 50));
  path.push_back(land_pos);
  return path;
}

// ==================== 竞速门穿越辅助 ====================

Point3D MapManager::getGateApproachPoint(int gate_id, double dist) const {
  const auto& g = (gate_id == 1) ? gate1_ : gate2_;
  // 竞速门1朝向Y正方向，竞速门2朝向X正方向
  if (gate_id == 1)
    return Point3D(g.cx, g.cy - dist, 100);
  else
    return Point3D(g.cx - dist, g.cy, 100);
}

Point3D MapManager::getGateExitPoint(int gate_id, double dist) const {
  const auto& g = (gate_id == 1) ? gate1_ : gate2_;
  if (gate_id == 1)
    return Point3D(g.cx, g.cy + dist, 100);
  else
    return Point3D(g.cx + dist, g.cy, 100);
}

}  // namespace competition
