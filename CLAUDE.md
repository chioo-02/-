# 浙江省第十届大学生机器人竞赛 — 空中机器人邀请赛

## 项目概述

四旋翼无人机在 6m×5m×2.5m 场地内全自主完成：起飞→导航→颜色识别→抓取→导航→投放→降落。技术栈: ROS1 Noetic + PX4 + Mavros + OpenCV。规则版本: 20260515。

---

## 一、场地坐标系与空间结构 (cm, x→右0~600, y→上0~500, z→上0~250)

```
Y(500) ┌───────────────────────────────────────────────────┐
       │  A区(130,400)           Gate1(125,410)  Gate2     │
       │  [识别靶+球(62.5,387.5)]  [门朝Y, 70cm方] (400,385) │
       │                                                   │
       │  wall1: y=96, x=0~300, z=30~150                   │
       │                                                   │
       │  H(30,30)                        B区(500,130)     │
       │  [起飞50×50]                      [R/G/B三色降落板] │
     0 └───────────────────────────────────────────────────┘
     0                                                    600
```

### 关键坐标 (m，PX4/mavros单位)

| 名称 | x | y | z | 说明 |
|------|---|---|---|------|
| 起飞点 | 0.30 | 0.30 | 0.80 | H区上空 |
| A区悬停 | 1.30 | 4.00 | 1.00 | 颜色识别位置 |
| 抓取球 | 0.625 | 3.875 | 1.00 | 球心离地1m，直径6cm，内嵌磁铁 |
| B区悬停 | 5.00 | 1.30 | 1.00 | 投放前悬停 |
| 降落R | 4.45 | 1.00 | 0 | 红色降落板 50×50cm |
| 降落G | 5.00 | 1.90 | 0 | 绿色降落板 50×50cm |
| 降落B | 5.45 | 1.00 | 0 | 蓝色降落板 50×50cm |
| 竞速门1 | 1.25 | 4.10 | 开口0.65~1.35 | 朝向Y+，通过+5分 |
| 竞速门2 | 4.00 | 3.85 | 开口0.65~1.35 | 朝向X+，通过+5分 |
| 中间绕行点 | 0.30→0.30 | 0.30→2.00 | 0.80→1.00 | 避开挡板1 |
| A→B中转 | 3.00 | 2.50 | 1.00 | 场地中部 |

### 障碍物位置 (30×30×180cm，随机6个，O2/O6为动态≤0.5m/s)

| ID | x范围 | y范围 | 类型 |
|----|-------|-------|------|
| O1 | 60~100 | 110~150 | 静态 |
| O2 | 160~200 | 90~130 | **动态** |
| O3 | 220~260 | 330~370 | 静态 |
| O4 | 340~380 | 200~240 | 静态 |
| O5 | 420~460 | 350~390 | 静态 |
| O6 | 200~240 | 410~450 | **动态** |

### 约束
- 围墙1.5m，上接防护网至2.5m（规则4：**禁止飞越木板墙**）
- 围墙底部30cm镂空
- 内部挡板4块，竞速门开口70×70cm，棋盘格边框4cm

---

## 二、比赛任务与评分 (满分100)

| 任务 | 分值 | 通过条件 | 关键点 |
|------|------|---------|--------|
| ① 基本功能测试 | 10 | 起飞→悬停>5s且≥0.8m→降落 | **必须拿** |
| ② 起点→A导航 | 10 | 全部位进入A区停留>5s | **必须拿** |
| ③ 物品抓取 | 15 | 抓取白色小球保持>5s | 掉落-5分，仅扣一次 |
| ④ A→B导航 | 10 | 全部位进入B区停留>5s | 掉球-5分(不重复扣) |
| ⑤ 精准投放 | 15 | 得分框内/中/外→15/10/5分 | **颜色错×1/3**，致命 |
| ⑥ 识别任务 | 15 | 命令行输出A处颜色R/G/B | 不必等抓取，上去就做 |
| ⑦ 降落任务 | 15 | 全落15/半落10/一个脚5分 | **颜色错×1/3**，致命 |

**罚分**: 碰撞障碍物-1/次(上限4次)
**加分**: 通过竞速门+5/个(每门只加一次)
**同分**: 比较任务完成时间

---

## 三、获胜策略

### 策略优先级
1. **必保基础35分**: 任务①②⑥ = 起飞+去A+输出颜色。保守模式(`conservative_mode=true`)稳拿
2. **颜色正确性≫一切**: 投放/降落颜色错得分×1/3，即使投中满环也只有5分。**颜色确认与输出是最高优先级**
3. **竞速门加分**: 两门+10分可拉开差距。门1在A区右侧，A→B必经；门2在右上，可绕行获取
4. **抓取保稳**: 抓不到球跳过（-5分仅扣一次），不要为15分冒撞柱-1×4=-4+掉球-5的风险
5. **任务⑥即做**: 到A区后立即识别输出，不等抓取完成

### 推荐飞行路径

```
起飞(30,30) → 先爬升到Y=200绕开wall1(挡板y=96横墙) → A区(130,400)
  → 悬停识别颜色 → 命令行输出(任务⑥, +15)
  → [可选] 抓球(62.5, 387.5, z=100)
  → [可选] 穿越Gate1(125, 410) +5分
  → 经中转(300, 250) → B区(500, 130)
  → 根据识别颜色选对应得分框投放(任务⑤) → 同一颜色区降落(任务⑦)
  → [可选] 若未过Gate2，返程穿越 +5分(不推荐，耗时)
```

### 风险点
- **wall1(y=96)**: 从H直飞A会被挡，必须先沿Y方向爬升绕开
- **O2动态**: 中左区域往复运动，A→B经中转点(300,250)应与之保持>30cm安全距离
- **O6动态**: A区右上方，影响Gate1穿越路径
- **Gate1穿越**: 开口70×70cm在z=65~135cm，需精确高度控制
- **颜色混淆**: 红色HSV两端(0~10, 160~180)，实机光照下容易误检，**必须用calibrate_hsv.py标定**

---

## 四、代码结构

```
competition_ws/src/
├── competition_bringup/          # 启动配置 + 地图管理C++库
│   ├── config/map.yaml           # 地图所有坐标(20260515更新)
│   ├── launch/competition.launch  # 仿真启动(含Gazebo)
│   ├── launch/real_flight.launch  # 实机启动(无Gazebo)
│   ├── include/.../map_utils.h    # 地图管理类声明
│   └── src/map_utils.cpp          # 地图加载/查询/碰撞检测实现
│
├── competition_gazebo/           # Gazebo仿真环境
│   ├── models/                    # 障碍物/门/场地模型
│   ├── worlds/competition.world   # 场地定义(内联所有模型pose)
│   ├── launch/gazebo.launch
│   └── scripts/randomize_obstacles.py  # 随机化6个障碍物
│
├── vision_node/                  # 颜色检测(OpenCV HSV)
│   ├── include/.../color_detector.h   # ColorDetector类声明
│   ├── src/color_detector.cpp         # HSV阈值分割+轮廓检测
│   ├── src/vision_node.cpp            # ROS节点: 订阅图像,发布颜色/位置
│   ├── config/color_params.yaml       # HSV阈值(实机须重新标定!)
│   └── scripts/calibrate_hsv.py       # HSV滑动条实时标定工具
│
├── mission_node/                 # 任务状态机(核心调度)
│   ├── include/.../mission_fsm.h      # FSM状态定义+MissionConfig
│   └── src/mission_node.cpp           # FSM实现: 所有任务handle函数
│
└── gripper_node/                 # 抓取机构控制
    └── src/gripper_node.cpp           # GPIO电磁铁 / PCA9685舵机
```

### 节点间通信
```
vision_node → /vision/detected_color (std_msgs/String) → mission_node(通过param读取)
vision_node → /vision/target_position (PointStamped) → (备用于接近目标)
mission_node → mavros setpoint_position/setpoint_velocity → PX4
mission_node → /gripper/command (String "grasp"/"release") → gripper_node
gripper_node → /gripper/state (String) → mission_node
```

---

## 五、代码编写规范

### 通用
- C++17 (核心模块), Python3 (工具脚本)
- 命名: PascalCase类/结构体, camelCase函数, snake_case变量, trailing underscore成员
- 注释: 只在WHY非显而易见时写。不写"做什么"注释
- 不引入不必要的抽象/helper，三行重复代码比过早抽象好

### ROS/飞控
- offboard模式切换必须有超时保护和故障检测(已有30s超时)
- Emergency KILL独立于主控循环
- 仿真和实机参数分离(competition.launch vs real_flight.launch)
- 所有飞行高度必须 ≤ `config_.max_altitude = 1.5m` (规则4硬限制)

### OpenCV/视觉
- 红色检测始终用双范围(HSV两端) — `thresholdRed()`已实现
- 实机比赛前必须用`calibrate_hsv.py`现场标定HSV阈值
- 深度图类型: Realsense D435 → 16UC1(单位mm)，代码已兼容32FC1

### 状态机扩展
- 新状态添加到`TaskState` enum → 在`run()`的switch中添加case → 实现handleXxx()
- 状态转换用`setState()` (会打印日志和记录时间戳)
- 每个handle函数内的循环必须调用`ros::spinOnce()`保持mavros连接

### 坐标单位
- **map.yaml: cm** (与STL/3MF一致，便于人工理解)
- **PX4/mavros: m** (ROS标准，代码中 `/100.0` 转换)
- 新增坐标需在map.yaml声明 → mission_node的loadMissionConfig()读取

---

## 六、实机部署检查清单

- [ ] HSV阈值已用`calibrate_hsv.py`重新标定
- [ ] 相机topic已在`real_flight.launch`中remap到实际话题
- [ ] mavros连接正常(`/mavros/state` → `connected: True`)
- [ ] 抓取机构类型(`electromagnet`/`servo`)和引脚已配置
- [ ] 场地坐标原点与PX4 local_position原点对齐
- [ ] `max_altitude=1.5m` 已确认(规则4)
- [ ] `min_takeoff_height≥0.8m` 已确认(规则5)
- [ ] **全链路地面仿真验证通过后再实飞**
- [ ] 比赛时先跑保守模式(`conservative_mode=true`)基操确认，再考虑启用抓取

---

## 七、已知待改进

- `mission_node`当前通过ROS param获取颜色结果(轮询)，应改为订阅`/vision/detected_color` topic
- 竞速门穿越逻辑未实现(gate1/2坐标已有，需姿态对齐+高度控制)
- 动态障碍物O2/O6避障未实现(需实时感知或运动预测)
- `conservative_mode`和`enable_grasp`的逻辑耦合，应拆分为独立开关
- `flyTo()`使用纯位置设定点，未做速度规划(acceleration/deceleration)
- 颜色检测超时默认30s偏长，改为15s+重试逻辑更合理
