# 6天冲刺方案 — 目标60分（适配 Fast-Drone-250 架构）

## 你的实际硬件/软件栈

```
硬件: 风火轮X200 (5寸, 倒推) + PX4 v1.11 + NUC + Realsense D435 + AT9S遥控
定位: VINS-Fusion (视觉惯性里程计)
控制: px4ctrl (轨迹跟踪, /px4ctrl/* topics)
规划: EGO-Planner (支持 waypoint 模式 flight_type=2)
感知: competition_ws/vision_node (HSV颜色检测)
抓取: competition_ws/gripper_node (GPIO电磁铁)
```

## 与 competition_ws 原有架构的差异

| | competition_ws (旧) | Fast-Drone-250 (你的实际环境) |
|---|---|---|
| 定位 | mavros `/local_position/odom` | VINS `/vins_fusion/odometry` |
| 控制 | mavros `setpoint_position` | px4ctrl `*/cmd_vel` + `*/cmd_acc` |
| 起飞 | mission_node 发位置设定点 | `sh shfiles/takeoff.sh` + px4ctrl |
| 路径 | `flyTo()` 直线位置设定点 | EGO-Planner 轨迹规划 |
| 紧急安全 | 无独立机制 | RC 5ch(切自稳)/6ch(退轨迹)/7ch(急停桨) |
| 坐标系 | 假设PX4 local_position原点 | VINS初始化原点=起飞时飞机位置 |

**核心结论**: mission_node 不再直接控制 mavros，而是作为"任务指挥官"向 EGO-Planner 下发航点，由 px4ctrl 执行。

---

## 架构桥接方案

```
mission_node (任务FSM + 航点决策)
    │
    ├── /mission/waypoint (geometry_msgs/PoseStamped)  → EGO-Planner waypoint模式
    │        │
    │        ▼
    │   EGO-Planner (避障轨迹规划, flight_type=2)
    │        │
    │        ▼
    │   px4ctrl (轨迹跟踪控制)
    │        │
    │        ▼
    │   mavros → PX4 (飞控执行)
    │
    ├── /vision/detected_color ← vision_node (HSV检测)
    ├── /gripper/command        → gripper_node (抓取)
    ├── /gripper/state          ← gripper_node (确认)
    │
    └── 紧急安全: RC遥控器 (独立于程序, 5ch切自稳/7ch急停)
```

### 坐标系对齐（关键）
- VINS原点 = 起飞时无人机位置
- map.yaml 坐标原点 = 场地左下角 (0,0,0)
- **起飞点H**: 场地坐标(30,30)cm → 必须确保起飞时机体在场地内 (30,30) 位置
- 所有航点 = map.yaml坐标 - (30,30) = VINS坐标系下的目标
- 例如: A区(130,400) → VINS坐标(100,370)

---

## 60分构成（与你的平台对齐后）

```
① 起飞悬停         10  ← px4ctrl takeoff.sh 已有
② H→A 导航         10  ← EGO-Planner waypoint模式
⑥ 颜色识别输出     15  ← vision_node 已有
③ 抓取小球         15  ← 需完善（盲抓+推力确认）
④ A→B 导航         10  ← EGO-Planner waypoint模式
⑤ 精准投放         15  ← 需实现（坐标对准+释放）
⑦ 精确降落         15  ← 需实现（深度辅助降落）
Gate1 + Gate2      +10  ← EGO-Planner waypoint 穿门
────────────────────────
保守目标             60~70
```

---

## 砍掉的内容
- ~~A*路径规划~~ → 用 EGO-Planner waypoint 模式（你已有且可工作）
- ~~flyTo()位置设定点~~ → 用 px4ctrl 轨迹控制（更稳定）
- ~~AprilTag~~ → 用 VINS 定位 + 深度图对准（精度够用）
- ~~Gazebo动态障碍物~~ → 实机验证优先
- ~~visual servo全套~~ → 用 px4ctrl position hold + 深度对准简化版

---

## Day 1: Bug修复 + 架构桥接（地基）

### 1.1 修复 map_utils 崩溃
**文件**: `map_utils.cpp` `loadWalls()` (122行)
- `kv.second()` → `kv.second`
- x添加标量回退（wall3用`x: 129`而非`x_range`）
- 30分钟工作量

### 1.2 修复 map.yaml 航点为安全路径
**文件**: `map.yaml`
- H→A via 从 (30,200) 改为 (340,30)→(340,200)（绕wall1）
- A→B via 从 (300,250) 改为 (300,270)→(420,270)（绕O4上方）
- 同步更新 extract_map_points.py 供调试

### 1.3 mission_node 适配 Fast-Drone-250 架构
**这是最关键的一天工作** — 让 mission_node 融入你的现有架构

**文件**: `mission_node.cpp` — 重写控制接口层

```cpp
// 不再用 mavros setpoints，改为发航点给 EGO-Planner
class MissionFSM {
  // 新增: 发布航点到 EGO-Planner waypoint 模式
  ros::Publisher waypoint_pub_;   // /mission/waypoint
  
  // 新增: 订阅 VINS 里程计（替代 mavros odom）
  ros::Subscriber vins_odom_sub_;
  
  // 移除: 不再直接发 setpoint_position/velocity 到 mavros
  // 移除: 不再直接调用 mavros arming/set_mode 服务
  // (这些由 px4ctrl + takeoff.sh 处理)
};
```

**状态机改造**:
```
ARMING → 由 shfiles/takeoff.sh 替代
TAKEOFF → px4ctrl 自动起飞
NAV_TO_A → 发航点 (100, 370, 1.0) 到 EGO-Planner
NAV_TO_B → 发航点 (470, 100, 1.0) 到 EGO-Planner  
LANDING → 发航点 (target_x, target_y, 0) 到 EGO-Planner
```

**航点坐标系转换**: map.yaml坐标(cm) → VINS坐标(m，偏移-30cm)
```cpp
Point3D mapToVins(double map_x_cm, double map_y_cm, double map_z_cm) {
  return Point3D(
    (map_x_cm - 30) / 100.0,  // 减去起飞点偏移，cm→m
    (map_y_cm - 30) / 100.0,
    map_z_cm / 100.0
  );
}
```

### 1.4 颜色检测订阅修复
**文件**: `mission_fsm.h`, `mission_node.cpp`
- 订阅 `/vision/detected_color`，回调更新 `latest_color_`

### 1.5 px4ctrl 参数适配风火轮X200
**文件**: `Fast-Drone-250/src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml`
- `mass`: 实测风火轮X200 + 电池 + NUC + 抓取机构的总重量(kg)
- `hover_percent`: 通过PX4 log查看悬停油门百分比，或实测
- `gain/Kp`, `gain/Kv`: 先保持默认，实飞时根据响应调整（超调→减小，迟钝→增大）
- 倒推不影响这些参数

**Day 1验证**: 
- 启动 `sh shfiles/rspx4.sh`（realsense+mavros+VINS）正常
- `roslaunch px4ctrl run_ctrl.launch` 正常
- mission_node 能发布航点，EGO-Planner 能接收

---

## Day 2: 竞速门穿越 + 投放到降落（+25分潜力）

### 2.1 竞速门穿越（+10分）
不用自己写 passGate() — 直接发航点序列给 EGO-Planner

**Gate1** (场地坐标125,410 → VINS坐标95,380, z=1.0):
```
航点序列: A区(100,370,1.0) → 门前(95,350,1.0) → 门后(95,410,1.0) → 中转(270,240,1.0)
```
EGO-Planner 会自动规划穿越门的平滑轨迹

**Gate2** (场地坐标400,385 → VINS坐标370,355, z=1.0):
```
航点序列: 中转(270,240,1.0) → 门前(320,355,1.0) → 门后(420,355,1.0) → B区(470,100,1.0)
```

**文件**: `mission_node.cpp` — 新增 `PASS_GATE1` / `PASS_GATE2` 状态
每个状态发一个航点序列给 EGO-Planner，等待无人机到达后切换下一状态

### 2.2 精密投放（15分）
**简化方案**: 飞到得分框正上方→降高度→释放

利用 VINS 定位 + 已知得分框坐标:
```
① 飞到对应颜色得分框上方 (如 R: 415,70)
② 缓慢降至 z=0.4m (框上方)
③ 释放球（/gripper/command "release"）
④ 抬升至 z=1.0m
```

不需要深度对准 — VINS精度1-3cm在5cm框内径下够用。颜色选择来自 Step ⑥。

**Day 2验证**: Gate1+Gate2航点序列飞行无误，球落入得分框

---

## Day 3: 抓取完善 + 降落（+30分潜力）

### 3.1 抓取可靠性（15分）
**文件**: `mission_node.cpp` `handleGraspBall()`

基于已知坐标盲抓 + gripper状态确认:
```
① 飞到球上方 VINS坐标 (32.5, 357.5, 1.10)m
② 缓降至 z=0.92m
③ 发 /gripper/command "grasp"
④ 等 gripper_state "grasped" (3s超时)
⑤ 成功→抬升 z=1.2m
   失败→重试1次
   两次失败→放弃（-5分不致命）
```
球坐标从 map.yaml 读取，自动转VINS坐标

### 3.2 降落（15分）
**简化方案**: EGO-Planner航点降落 + 深度触地检测

```
① 飞到对应颜色降落板上方 VINS坐标
  R: (415,70,0.5) / G: (470,160,0.5) / B: (515,70,0.5)
② EGO-Planner发 z=0 航点，让无人机自动下降
③ mission_node监听 VINS odom: z<0.05 且 vz<0.05 → 触地
④ 调用 px4ctrl disarm 或切 land 模式
```

不需要 visual servo — VINS 定位在室内10m范围内漂移<0.3m，在50cm降落板上够用。如想更稳健，可在下降最后0.3m用深度图做微调。

**Day 3验证**: 抓取成功率>70% + 降落位置在板范围内

---

## Day 4: 完整状态机 + 时间预算 + 安全联调

### 4.1 完整FSM流程（适配后）

```
IDLE → ARMING(调用takeoff.sh) → TAKEOFF(px4ctrl自动)
  → NAV_TO_A(发文航点) → DETECT_COLOR(读vision_node结果)
  → [GRASP_BALL(抓取+确认)] 
  → [PASS_GATE1(发Gate1航点序列)]
  → NAV_TO_B(发B区航点) 
  → [PASS_GATE2(发Gate2航点序列)]
  → DROP_AND_OUTPUT(选择颜色→飞得分框→释放)
  → LANDING(飞降落板→下降→触地检测→disarm)
  → FINISH
```

### 4.2 EGO-Planner 配置适配比赛场地
**文件**: `ego_planner/single_run_in_exp.launch`
- `map_size`: 设为 `7.0`（场地6m+余量）
- `max_vel`: `0.5`（比赛求稳不求快）
- `max_acc`: `2.0`
- `flight_type`: `2`（waypoint模式）
- `fx/fy/cx/cy`: 你的 D435 内参（已标定）

**文件**: `ego_planner/advanced_param_exp.xml`
- `resolution`: `0.1` (10cm栅格)
- `obstacles_inflation`: `0.3` (30cm膨胀，配合30cm障碍物间距)

### 4.3 时间预算管理
```
0~5min:   全开
5~8min:   跳过抓取
8~10min:  跳过竞速门
10~12min: 就近降落（不区分颜色）
>12min:   紧急降落
```

### 4.4 罚分追踪 + 日志
记录碰撞(-1/次)、掉球(-5)、门通过(+5/个)，命令行实时显示累计分

**Day 4验证**: 全状态机走通，时间预算各档正确

---

## Day 5: 实机地面联调（不飞！）

### 5.1 全链路地面测试
```
① 启动: sh shfiles/rspx4.sh
② 启动: roslaunch px4ctrl run_ctrl.launch  
③ 启动: roslaunch competition_bringup real_flight.launch (mission+vision+gripper)
④ 启动: roslaunch ego_planner single_run_in_exp.launch flight_type:=2
```

不装螺旋桨，用遥控器确认各模式切换正常:
- 5ch切自稳 → mavros state 确认
- mission_node 发航点 → EGO-Planner 接收并规划轨迹
- vision_node 颜色检测正常
- gripper_node 机构响应

### 5.2 坐标系验证
拿飞机在场地内移动，对比 VINS odom 和实际位置:
- 飞机放H区 → VINS显示(0,0)
- 飞机放A区 → VINS显示约(1.0, 3.7)
- 飞机放B区 → VINS显示约(4.7, 1.0)
- 偏差>0.3m → 检查VINS外参标定

### 5.3 px4ctrl参数确认
- 测整机重量 → 填 `ctrl_param_fpv.yaml` mass
- 检查 `hover_percent`（参考PX4 log或QGC显示）

---

## Day 6: 实飞验证

### 上午: HSV现场标定
- 比赛场地光照下用 `calibrate_hsv.py` 标定R/G/B
- 确认相机图像正常
- 确认 VINS 在场地内初始化稳定（缓慢小范围晃动后回原点）

### 下午: 分层实飞

```
第1层: 基础功能 (35分)
  起飞→悬停→VINS确认→去A→输出颜色→去B→降落
  确认: VINS不飘、路径无碰撞、颜色输出正确

第2层: +竞速门 (45~50分)
  起飞→A→颜色→Gate1→B→降落
  确认: 门穿越不碰框

第3层: +抓取+投放 (65~70分)
  起飞→A→颜色→抓球→Gate1→B→投放→降落
  确认: 抓取成功、投放精准、降落准确

第4层: 全开 (75+分)
  起飞→A→颜色→抓球→Gate1→Gate2→B→投放→降落
```

**安全原则**: RC遥控器始终在手，任何异常→6ch退轨迹→5ch切自稳→7ch急停

---

## 文件修改总清单

| Day | 文件 | 改动 |
|-----|------|------|
| 1 | `map_utils.cpp` | 修复 loadWalls 崩溃 (kv.second + x_range回退) |
| 1 | `map.yaml` | 更新 via 为安全路径，坐标注释改为VINS偏移 |
| 1 | `mission_fsm.h` | 添加 waypoint_pub_/vins_odom_sub_/坐标转换/新状态 |
| 1 | `mission_node.cpp` | **重写控制层**: 航点→EGO-Planner，订阅VINS odom，移除mavros setpoints |
| 1 | `ctrl_param_fpv.yaml` | 适配风火轮X200: mass/hover_percent |
| 1 | `single_run_in_exp.launch` | map_size=7, flight_type=2, max_vel=0.5 |
| 1 | `color_sub_` | 订阅 /vision/detected_color 替代参数轮询 |
| 2 | `mission_node.cpp` | PASS_GATE1/2状态：发Gate航点序列 |
| 2 | `mission_node.cpp` | DROP_AND_OUTPUT改造：飞到得分框→降→放→升 |
| 3 | `mission_node.cpp` | GRASP_BALL改造：已知坐标盲抓+gripper确认+重试 |
| 3 | `mission_node.cpp` | LANDING改造：VINS航点降落+深度触地检测 |
| 4 | `mission_node.cpp` | 完整FSM串联+时间预算+罚分追踪+日志 |
| 5 | 地面联调 | 全链路地面测试+坐标系验证+参数确认 |
| 6 | 实飞 | HSV标定+分层实飞 |
