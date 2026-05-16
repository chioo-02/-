#!/usr/bin/env python3
"""
从 map.yaml 提取关键坐标点并打印。
单位: cm (1cm = 0.01m)，坐标系 X→右(0~600), Y→上(0~500), Z→上(0~250)
"""

import yaml
import os

MAP_PATH = os.path.join(os.path.dirname(__file__),
                        "src/competition_bringup/config/map.yaml")


def fmt_pt(x, y, z=None):
    """格式化坐标点"""
    if z is not None:
        return f"({x:6.1f}, {y:6.1f}, {z:6.1f}) cm"
    return f"({x:6.1f}, {y:6.1f}) cm"


def print_sep(title):
    print()
    print("=" * 65)
    print(f"  {title}")
    print("=" * 65)


def main():
    with open(MAP_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # ========== 场地基本信息 ==========
    arena = data["arena"]
    print_sep("场地基本信息")
    print(f"  场地尺寸:  {arena['length_x']} × {arena['width_y']} × {arena['height_z']} cm")
    print(f"            (即 {arena['length_x']/100:.1f} × {arena['width_y']/100:.1f} × {arena['height_z']/100:.1f} m)")
    print(f"  围墙高度:  {arena['wall_height']} cm (底部镂空 {arena['wall_bottom']} cm)")

    # ========== 关键区域 ==========
    zones = data["zones"]
    print_sep("关键区域")
    for zone_name, zone in zones.items():
        print(f"  {zone_name:12s}  中心 {fmt_pt(*zone['center']):30s}  {zone.get('desc', '')}")
        if "size" in zone:
            print(f"  {'':12s}  尺寸 {zone['size'][0]}×{zone['size'][1]} cm")

    # ========== 航点序列 ==========
    wp = data["waypoints"]
    hover_h = wp["hover_height"]
    print_sep(f"航点序列 (默认巡航高度: {hover_h} cm)")
    waypoint_list = []

    # 起飞
    waypoint_list.append(("起飞悬停", wp["takeoff"]["x"], wp["takeoff"]["y"], wp["takeoff"]["z"],
                          wp["takeoff"]["desc"]))

    # nav_to_a via
    for i, via in enumerate(wp["nav_to_a"]["via"]):
        waypoint_list.append((f"→A 绕行点{i+1}", via["x"], via["y"], via["z"], via["desc"]))

    # nav_to_a target
    t = wp["nav_to_a"]["target"]
    waypoint_list.append(("→A 目标点", t["x"], t["y"], t["z"], t["desc"]))

    # detect hover
    waypoint_list.append(("颜色识别悬停", wp["detect_hover"]["x"], wp["detect_hover"]["y"],
                          wp["detect_hover"]["z"], wp["detect_hover"]["desc"]))

    # nav_to_b via
    for i, via in enumerate(wp["nav_to_b"]["via"]):
        waypoint_list.append((f"→B 绕行点{i+1}", via["x"], via["y"], via["z"], via["desc"]))

    # nav_to_b target
    t = wp["nav_to_b"]["target"]
    waypoint_list.append(("→B 目标点", t["x"], t["y"], t["z"], t["desc"]))

    # landings
    for color in ["R", "G", "B"]:
        key = f"landing_{color}"
        w = wp[key]
        waypoint_list.append((f"降落区 {color}", w["x"], w["y"], w["z"], w["desc"]))

    for i, (name, x, y, z, desc) in enumerate(waypoint_list):
        marker = " ◆" if "悬停" in name or "目标" in name else (" ▼" if "降落" in name else "   ")
        print(f"  [{i+1:2d}]{marker} {name:16s}  {fmt_pt(x, y, z):38s}  {desc}")

    # ========== 颜色区 → 降落映射 ==========
    print_sep("颜色区 → 降落映射")
    cz = data["color_zones"]
    for color in ["R", "G", "B"]:
        c = cz[color]
        w = wp[c["landing"]]
        print(f"  颜色 {color}: 识别后降落到 {c['landing']}"
              f"  → 得分框中心 {fmt_pt(c['score_x'], c['score_y'])}"
              f"  → 降落点 {fmt_pt(w['x'], w['y'], w['z'])}")

    # ========== A区抓取球 ==========
    ball = data["ball"]
    print_sep("A区抓取球")
    print(f"  位置:    {fmt_pt(ball['x'], ball['y'], ball['z'])}")
    print(f"  直径:    {ball['diameter']} cm")
    print(f"  说明:    {ball['desc']}")
    print(f"  ↓ 米制:  ({ball['x']/100:.3f}, {ball['y']/100:.3f}, {ball['z']/100:.3f}) m")

    # ========== 障碍物候选位置 ==========
    obs = data["obstacles"]
    print_sep(f"障碍物候选位置 (默认尺寸 {obs['default_size'][0]}×{obs['default_size'][1]}×{obs['default_size'][2]} cm, 随机选最多5个)")
    pp = obs["possible_positions"]
    for oid in sorted(pp.keys()):
        o = pp[oid]
        type_tag = "◈ 动态" if o["type"] == "dynamic" else "○ 静态"
        print(f"  {oid:4s}  {type_tag:8s}  "
              f"X: [{o['x_range'][0]:5.1f} ~ {o['x_range'][1]:5.1f}] cm  "
              f"Y: [{o['y_range'][0]:5.1f} ~ {o['y_range'][1]:5.1f}] cm  "
              f"  中心≈({(o['x_range'][0]+o['x_range'][1])/2:.0f}, {(o['y_range'][0]+o['y_range'][1])/2:.0f}) cm"
              f"  {o['desc']}")

    # ========== 竞速门 ==========
    gates = data["gates"]
    print_sep("竞速门 (通过一个 +5分, 开口 70×70 cm)")
    for gid, g in gates.items():
        print(f"  {gid:6s}  中心 {fmt_pt(*g['center'])}"
              f"  Z开口: {g['opening_z_min']}~{g['opening_z_max']} cm"
              f"  ({g['opening_size'][0]}×{g['opening_size'][1]} cm)"
              f"  → {g['desc']}")

    # ========== 内部挡板 ==========
    walls = data["walls"]
    print_sep("内部挡板 (需规避)")
    for wid, w in walls.items():
        along = w["along"]
        if along == "x":
            desc = (f"沿X方向, X:[{w['x_range'][0]}, {w['x_range'][1]}] cm, "
                    f"Y={w['y']} cm, Z:[{w['z_range'][0]}, {w['z_range'][1]}] cm")
        else:
            desc = (f"沿Y方向, X={w['x']} cm, "
                    f"Y:[{w['y_range'][0]}, {w['y_range'][1]}] cm, "
                    f"Z:[{w['z_range'][0]}, {w['z_range'][1]}] cm")
        print(f"  {wid:6s}  {desc}  → {w['desc']}")

    # ========== 安全参数 ==========
    safety = data["safety"]
    print_sep("安全参数")
    print(f"  障碍物安全距离:    {safety['obstacle_clearance']} cm")
    print(f"  竞速门安全余量:    {safety['gate_clearance']} cm")
    print(f"  避墙距离:          {safety['wall_avoid_distance']} cm")
    print(f"  最长飞行时间:      {safety['max_flight_time']} 秒 ({safety['max_flight_time']/60:.0f} 分钟)")

    # ========== 任务流程快速速查 ==========
    print_sep("任务流程关键点速查")
    print("  ① 起飞      → H区上方 ({:5.1f}, {:5.1f}, {:5.1f}) cm".format(
        wp["takeoff"]["x"], wp["takeoff"]["y"], wp["takeoff"]["z"]))
    print("  ② →A        → 绕行 ({:5.1f}, {:5.1f}) 避开挡板1 → 到达 ({:5.1f}, {:5.1f}) cm".format(
        wp["nav_to_a"]["via"][0]["x"], wp["nav_to_a"]["via"][0]["y"],
        wp["nav_to_a"]["target"]["x"], wp["nav_to_a"]["target"]["y"]))
    print("  ③ 颜色识别  → 悬停 ({:5.1f}, {:5.1f}, {:5.1f}) cm".format(
        wp["detect_hover"]["x"], wp["detect_hover"]["y"], wp["detect_hover"]["z"]))
    print("  ④ 抓球      → ({:5.1f}, {:5.1f}, {:5.1f}) cm (离靶心50cm)".format(
        ball["x"], ball["y"], ball["z"]))
    print("  ⑤ 过门1     → ({:5.1f}, {:5.1f}) cm (可选加分+5)".format(
        gates["gate1"]["center"][0], gates["gate1"]["center"][1]))
    print("  ⑥ →B        → 经中转 ({:5.1f}, {:5.1f}) → 到达 ({:5.1f}, {:5.1f}) cm".format(
        wp["nav_to_b"]["via"][0]["x"], wp["nav_to_b"]["via"][0]["y"],
        wp["nav_to_b"]["target"]["x"], wp["nav_to_b"]["target"]["y"]))
    print("  ⑦ 投放+降落 → 根据颜色选择对应降落区 (R:{:5.1f},{:5.1f} / G:{:5.1f},{:5.1f} / B:{:5.1f},{:5.1f}) cm".format(
        wp["landing_R"]["x"], wp["landing_R"]["y"],
        wp["landing_G"]["x"], wp["landing_G"]["y"],
        wp["landing_B"]["x"], wp["landing_B"]["y"]))
    print()

if __name__ == "__main__":
    main()
