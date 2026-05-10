#!/usr/bin/env python3
"""
随机化障碍物位置，用于仿真测试。
从 map.yaml 读取候选位置，随机选取至多5个放置。

用法:
  python randomize_obstacles.py --world ../worlds/competition.world --map ../../competition_bringup/config/map.yaml
"""
import argparse
import random
import yaml
import re


def main():
    parser = argparse.ArgumentParser(description="随机化障碍物位置")
    parser.add_argument("--world", default="competition.world", help="Gazebo world 文件路径")
    parser.add_argument("--map", default="map.yaml", help="map.yaml 路径")
    parser.add_argument("--seed", type=int, default=None, help="随机种子（可复现）")
    parser.add_argument("--num-obstacles", type=int, default=5, help="障碍物数量 (默认5)")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    # 读取 map.yaml
    with open(args.map, "r") as f:
        config = yaml.safe_load(f)

    obstacles = config["obstacles"]["possible_positions"]
    keys = list(obstacles.keys())

    # 随机选取
    selected = random.sample(keys, min(args.num_obstacles, len(keys)))
    print(f"选中的障碍物: {', '.join(selected)}")

    # 生成替换用的 <include> 块
    includes = []
    for key in selected:
        obs = obstacles[key]
        x_range = obs["x_range"]
        y_range = obs["y_range"]
        x = random.randint(x_range[0], x_range[1]) / 100.0
        y = random.randint(y_range[0], y_range[1]) / 100.0
        includes.append(f"""    <include>
      <uri>model://obstacle_box</uri>
      <name>{key.lower()}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
    </include>""")

    # 读取 world 文件并替换障碍物块
    with open(args.world, "r") as f:
        world_content = f.read()

    # 找到障碍物区域的开始和结束
    start_marker = "<!-- ======== Obstacles ======== -->"
    end_marker = "<!-- ======== Gates ======== -->"

    start_idx = world_content.find(start_marker)
    end_idx = world_content.find(end_marker)

    if start_idx == -1 or end_idx == -1:
        print("错误: 无法在 world 文件中找到障碍物标记区域")
        return

    obstacles_block = world_content[start_idx:end_idx]
    new_obstacles_block = start_marker + "\n" + "\n".join(includes) + "\n\n"

    new_content = world_content.replace(obstacles_block, new_obstacles_block)

    with open(args.world, "w") as f:
        f.write(new_content)

    print(f"已更新 {args.world}，放置了 {len(selected)} 个障碍物")


if __name__ == "__main__":
    main()
