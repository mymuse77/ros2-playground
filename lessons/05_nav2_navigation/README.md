# 05 Nav2 导航与多点巡逻

> 有了 04 课的地图，小车终于可以"看着地图走路"了：AMCL 告诉它自己在地图上的哪里，规划器在地图上找一条最好的路，
> 控制器沿着路开过去，遇到地图上没有的障碍就绕开，卡住了还会自己想办法脱困。最后写一个巡逻节点，让它在几个点之间一直转下去。

## 学习目标

- 理解 Nav2 的整体结构：定位、代价地图、规划器、控制器、行为树、生命周期管理各管什么
- 理解 AMCL 粒子滤波定位，以及它和 04 课 SLAM 的区别
- 看懂代价地图：静态层、障碍层、膨胀层，`robot_radius` 和 `inflation_radius` 的作用
- 会读、会改 Nav2 的参数文件，知道"改哪里会发生什么"
- 对比三种规划器（NavFn / Smac 2D / Theta*）和三种控制器（DWB / RPP / MPPI）的行为差异
- 会用 `nav2_simple_commander` 写导航程序：发目标、看反馈、超时取消、统计结果

## 前置条件

- 已完成的课程：[03 到点导航](../03_go_to_goal/)（导航动作、Bug2 的局限）、[04 SLAM 建图](../04_slam_mapping/)（地图、坐标系、生命周期节点）
- 依赖本仓库的包：`pg_sim`（仿真和 04 课的地图）、`pg_obstacle_avoidance_py`（验收用的 `safety_monitor`）
- 需要额外安装的软件包：`navigation2`、`nav2-bringup`（一键安装脚本已经装好）
- 编译：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to pg_nav2_py
source install/setup.bash
```

## 知识点

### 1. 从 Bug2 到 Nav2

| | 03 课 Bug2 | 05 课 Nav2 |
|---|---|---|
| 知道自己在哪 | 只靠里程计，误差越走越大 | AMCL 用雷达和地图对比，误差不累积 |
| 知道路怎么走 | 不知道，朝目标直走，撞墙了沿墙绕 | 在整张地图上规划最短（或最安全）的路径 |
| 避开障碍 | 碰到才绕，绕的方向是固定的 | 规划时就离障碍远一点；地图上没有的障碍用雷达实时补上 |
| 卡住了 | 判定"无法到达"就放弃 | 行为树依次尝试：清代价地图、原地转、等待、后退，再重新规划 |

代价是复杂度：Nav2 一启动就是十几个节点、几百个参数。这一课的重点不是记住参数，而是知道**每一块在管什么**。

### 2. Nav2 的结构

```
/map ──→ map_server ──→ global_costmap ──→ planner_server（全局路径 /plan）
/scan ─→ amcl（定位：map → odom）                │
                                                  ▼
bt_navigator（行为树：规划 → 跟踪 → 失败时恢复）  ← navigate_to_pose 动作（RViz、巡逻节点）
                                                  │
/scan ─→ local_costmap ──→ controller_server（跟踪路径，算速度）
                                                  │ cmd_vel_nav
                        velocity_smoother（限速、限加速度）
                                                  │ cmd_vel_smoothed
                        collision_monitor（最后一道防线）──→ /cmd_vel ──→ 小车
```

所有这些节点都是 04 课讲过的**生命周期节点**。`lifecycle_manager` 按顺序把它们 configure → activate，
任何一个失败，整组都不会激活（本课开发时就遇到过：地图文件路径不对，map_server 配置失败，定位整组都没起来）。

```bash
ros2 lifecycle get /amcl                 # active [3]
ros2 node list                           # 本课的 launch 关掉了 composition，每个节点一个进程
```

### 3. AMCL：在已知地图上定位

04 课的 SLAM 是"边画地图边定位"；有了地图以后，只需要定位，用 AMCL（自适应蒙特卡洛定位）：

1. 撒一堆**粒子**，每个粒子是一个"猜测的位姿"（RViz 里蓝色的小箭头）
2. 车动了：每个粒子按里程计的运动挪一下，再加点随机噪声（里程计不完全可信）
3. 来了一帧雷达：对每个粒子，假设车就在那里，看雷达点是不是正好落在地图的墙上——对得上的粒子权重高
4. 按权重重新抽样：权重高的粒子复制多份，权重低的淘汰
5. 粒子的加权平均就是定位结果，发布 `map → odom`（和 SLAM 发布的是同一段 TF）

刚启动时粒子散成一大片；车走几步，粒子就聚成一小团。**原地不动时 AMCL 不更新**（`update_min_d: 0.25`、`update_min_a: 0.2`），
所以 `/amcl_pose` 在车停着时不会变。

AMCL 需要一个**初始位姿**：大概告诉它车在哪，它在附近找。本课的参数文件用 `set_initial_pose: true`
直接设为地图原点 (0, 0, 0)——04 课地图的原点就是建图开始时小车的位置，而仿真的默认出生点和那时一样。
换了出生点就要在 RViz 里用 **2D Pose Estimate** 重新指定。

### 4. 代价地图

规划器和控制器都不直接看 `/map`，而是看**代价地图**：每格一个 0–255 的代价。

| 代价 | 含义 |
|---|---|
| 254 致命 | 障碍物本身 |
| 253 内切 | 车**中心**到这里，车身就会碰到障碍（离障碍不到 `robot_radius`） |
| 1–252 膨胀 | 离障碍越近越"贵"，`inflation_radius` 以外为 0 |
| 0 空闲 | |

```
   障碍 │ 内切 │←──── 膨胀区（代价逐渐降低）────→│ 空闲
   254  │ 253  │ 252 ………………………………………… 1 │  0
        │←────→│
      robot_radius
        │←──────────── inflation_radius ────────→│
```

代价地图由**图层**叠加而成：

| 图层 | 全局代价地图 | 局部代价地图 | 作用 |
|---|---|---|---|
| static_layer | ✓ | | 04 课的地图 |
| obstacle_layer / voxel_layer | ✓ | ✓ | 雷达实时看到的障碍（地图上没有的东西） |
| inflation_layer | ✓ | ✓ | 在障碍周围加上膨胀代价 |

全局代价地图覆盖整张地图（`map` 坐标系），给规划器用；局部代价地图只有车周围 3 m × 3 m，跟着车走（`odom` 坐标系），给控制器用。

本课把官方的 `robot_radius: 0.15` 改成了 0.22：waffle 车身 0.281 × 0.306 m，0.15 m 的圆根本盖不住车身，
规划出来的路径会让车身蹭到柱子。

### 5. 规划器：找一条路

规划器在全局代价地图上找从当前位置到目标的路径。行为树每秒重新规划一次，所以地图上出现新障碍时路径会跟着变。

| 规划器 | 原理 | 路径的样子 |
|---|---|---|
| **NavFn**（`GridBased`，默认） | 波前法（Dijkstra）：从目标往外"扩散"代价，再从起点顺着梯度走下来 | 大方向合理，但顺着栅格走，有很多小锯齿 |
| **Smac 2D**（`Smac2D`） | 带代价的 A*，8 邻域搜索，最后平滑 | 平滑；`cost_travel_multiplier` 越大越躲着障碍 |
| **Theta\***（`ThetaStar`） | A* 的变体："看得见"就直接连线 | 少数几段直线，转折点少 |

三个规划器在参数文件里同时加载，用 `/planner_selector` 话题切换，不用重启。实验 4 用 `compare_planners` 工具定量比较。

### 6. 控制器：沿着路开

控制器每 0.1 s 算一次速度，让车沿着全局路径走，同时避开局部代价地图里的障碍。

| 控制器 | 原理 | 特点 |
|---|---|---|
| **DWB**（`FollowPath`，默认） | 动态窗口法：在当前速度附近采样 800 组 (v, ω)，每组模拟 1.5 s，用一组评价器打分选最好的 | 经典、可调的地方多；评价器权重不好调 |
| **RPP**（`RPP`） | 规则化纯追踪：在路径上前视一段距离找"胡萝卜点"，算出开过去的圆弧；弯道、靠近障碍时降速 | 参数少、行为好预测，基本完全照着全局路径走 |
| **MPPI**（`MPPI`） | 模型预测路径积分：随机生成 1000 条速度序列，模拟 2.8 s，按得分加权平均 | 轨迹最平滑，能"抄近路"；计算量最大 |

三个控制器同样同时加载，用 `/controller_selector` 话题切换。实验 5 让三个控制器各巡逻一圈比较。

### 7. 行为树与恢复行为

`bt_navigator` 收到目标后执行一棵**行为树**（`navigate_to_pose_w_replanning_and_recovery.xml`），大致是：

```
重试最多 6 次 {
    并行 {
        每秒一次：规划路径（失败 → 清全局代价地图再试一次）
        跟踪路径          （失败 → 清局部代价地图再试一次）
    }
    失败时 → 轮流尝试：清代价地图 → 原地转 90° → 等 5 s → 后退 0.3 m
}
```

行为树里的 `PlannerSelector`、`ControllerSelector` 节点订阅 `/planner_selector`、`/controller_selector`，
决定这次用哪个规划器和控制器。它们用 Transient Local QoS，发布时也要用 Transient Local：

```bash
ros2 topic pub --once --qos-durability transient_local /controller_selector std_msgs/msg/String "{data: RPP}"
```

### 8. 用 Python 指挥 Nav2：nav2_simple_commander

`nav2_simple_commander.BasicNavigator` 把 Nav2 的动作包装成普通函数：

```python
nav = BasicNavigator()
nav.waitUntilNav2Active()            # 等 amcl、bt_navigator 激活
nav.goToPose(pose)                   # 发 navigate_to_pose 目标
while not nav.isTaskComplete():      # 每次调用内部 spin 0.1 s
    feedback = nav.getFeedback()     # 剩余距离、恢复次数……
result = nav.getResult()             # SUCCEEDED / FAILED / CANCELED
```

巡逻节点 `patrol_node.py` 继承 `BasicNavigator`，加上航点文件、圈数、单点超时、Ctrl+C 时取消目标等功能。
巡逻的"下一个去哪、失败怎么办、统计结果"放在不依赖 ROS 的 `patrol.py` 里，用单元测试覆盖。

一个坑：`waitUntilNav2Active()` 在收到 `/amcl_pose` 之前会反复发布初始位姿（默认是地图原点）。
如果小车已经开到别处再单独启动巡逻节点，定位就被"拽回"原点了。所以巡逻节点自己写了只等待、不发布的
`wait_for_nav2()`。

为什么不用 Nav2 自带的 `follow_waypoints` 动作一次发完所有点？它只在最后告诉你"哪些点没到"，
拿不到每个点的用时，也没法对单个点设超时。逐个发 `navigate_to_pose` 更适合学习和统计。

### 9. 代码结构

```
waypoints.py              航点文件解析、朝向 ↔ 四元数             纯 Python → 单元测试
patrol.py                 巡逻计划（顺序、圈数、失败处理）、统计     纯 Python → 单元测试
path_metrics.py           路径长度、转弯量、离障碍的距离            纯 Python → 单元测试
patrol_node.py            巡逻节点（BasicNavigator）
compare_planners_node.py  规划器对比工具
launch/nav2_sim.launch.py 仿真 + Nav2 + RViz +（可选）巡逻
config/nav2.yaml          Nav2 的全部参数，逐段注释，改过的地方标【改】
config/waypoints.yaml     巡逻航点
```

## 一键运行

```bash
# 仿真 + Nav2 + RViz，在 RViz 里用 "2D Goal Pose" 点目标
ros2 launch pg_nav2_py nav2_sim.launch.py

# 同时启动巡逻（一直巡逻，Ctrl+C 停止并打印汇总）
ros2 launch pg_nav2_py nav2_sim.launch.py patrol:=true
```

预期效果：RViz 中小车在地图左侧，周围一团蓝色粒子；全局代价地图在柱子和围墙周围画出彩色的膨胀区。
点一个目标后出现绿色的全局路径，小车沿着它开过去。巡逻时黄色箭头标出 5 个航点。

## 实验步骤

### 实验 1：看 Nav2 怎么启动

```bash
ros2 launch pg_nav2_py nav2_sim.launch.py
```

启动后十几秒内，日志里依次出现 `lifecycle_manager_localization` 和 `lifecycle_manager_navigation`
的 `Configuring ...`、`Activating ...`，最后是 `Managed nodes are active`。另开一个终端：

```bash
ros2 node list                 # 十几个 Nav2 节点
ros2 lifecycle get /amcl       # active [3]
ros2 lifecycle get /bt_navigator
ros2 topic info /cmd_vel -v    # 谁在给小车发速度？（collision_monitor）
```

**思考**：`ros2 lifecycle set /map_server deactivate` 之后会发生什么？RViz 里的地图还在吗？
（地图是 Transient Local，已经收到的不会消失；但 lifecycle_manager 发现 map_server 不再 active，可能把整组都关掉。）

### 实验 2：发一个目标

在 RViz 里点 **2D Goal Pose**，在地图右侧空地上按住鼠标拖出方向。观察：

1. 绿色的全局路径（`/plan`）立刻出现，绕开柱子周围的彩色膨胀区
2. 车头前面的短线是 DWB 的局部轨迹（`/local_plan`），每 0.1 s 变一次
3. 蓝色粒子随着车移动越来越集中
4. 到达后车头转到你拖出的方向

也可以用命令行发目标（`--feedback` 能看到剩余距离、恢复次数）：

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.45, y: -0.05}, orientation: {w: 1.0}}}}"
```

再把目标点在**柱子上**（例如 `x: 2.0, y: 0.5`）：规划失败，行为树开始恢复——清代价地图、原地转、等待、后退，
重试几次后报告失败。这就是 03 课 Bug2 做不到的"知道自己去不了"。

### 实验 3：代价地图调参

代价地图的膨胀参数可以运行时修改，RViz 里马上能看到膨胀区变化：

```bash
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.35   # 原来 0.5
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 10.0  # 原来 5.0
```

再发实验 2 的目标，路径离柱子更近了吗？`cost_scaling_factor` 越大，代价随距离下降得越快，路径越"敢"贴近障碍。

`robot_radius` 不能运行时改。把 `config/nav2.yaml` 里两处 `robot_radius` 改回官方的 0.15，重新 launch，
观察柱子周围深色的"内切区"变窄了多少；再用 `acceptance:=true`（实验 6）跑一圈，看 `safety_monitor` 的最小净空。

### 实验 4：规划器对比

```bash
ros2 launch pg_nav2_py nav2_sim.launch.py          # 终端 1
ros2 run pg_nav2_py compare_planners --ros-args -p use_sim_time:=true   # 终端 2
```

`compare_planners` 不开车，只让 planner_server 对巡逻路线的每一段各算一次路径，输出 Markdown 表格。指标：

| 指标 | 含义 |
|---|---|
| 长度 | 路径点之间的距离之和 |
| 累计转弯 | 相邻两段方向变化的绝对值之和，越小路径越"直"；锯齿越多数值越大 |
| 最小距离 | 路径上的点离最近障碍格子的距离（车中心到障碍，不是车身） |
| 请求 ms | 从发出请求到收到路径的墙钟时间，含通信开销 |

想在 RViz 里直接看形状：`ros2 topic pub --once --qos-durability transient_local /planner_selector std_msgs/msg/String "{data: ThetaStar}"`，
再发同一个目标，对比绿线。

### 实验 5：控制器对比

三个控制器各巡逻一圈：

```bash
ros2 launch pg_nav2_py nav2_sim.launch.py acceptance:=true loops:=1 controller:=FollowPath
ros2 launch pg_nav2_py nav2_sim.launch.py acceptance:=true loops:=1 controller:=RPP
ros2 launch pg_nav2_py nav2_sim.launch.py acceptance:=true loops:=1 controller:=MPPI
```

记录每个航点的用时和 `safety_monitor` 的最小净空。运行时用 `top -p $(pgrep -f controller_server)`
看 `controller_server` 的 CPU 占用。（MPPI 每个周期模拟 1000 条轨迹，计算量最大；但仿真很慢时控制周期按仿真时间算，
现实中每秒只跑几次，三者的 CPU 占用可能差不多，见实验记录。）
在 RViz 里打开 RPP 的前视点（`/lookahead_point`），看它怎样沿着路径往前"牵着"小车走。

### 实验 6：巡逻验收

```bash
ros2 launch pg_nav2_py nav2_sim.launch.py acceptance:=true loops:=3 gui:=false rviz:=false
```

巡逻 3 圈后自动退出。看最后的汇总：每个航点的成功次数和平均用时，以及 `safety_monitor` 的
"验收通过/不通过：……最小净空 …… m，接触 0 次"。

在巡逻过程中试试 Ctrl+C：巡逻节点会先取消正在执行的目标（小车停下），再打印汇总。

### 实验 7：定位"绑架"

启动后在 RViz 里用 **2D Pose Estimate** 故意把小车放到错误的位置（例如地图右上角），然后发一个目标。
观察粒子：一开始全在错误位置附近，雷达和地图对不上；AMCL 能不能自己纠正过来？
（AMCL 只在初始位姿附近找，偏得太远通常纠正不了。这就是"机器人绑架问题"。
`ros2 service call /reinitialize_global_localization std_srvs/srv/Empty` 把粒子撒满整张地图，再开一会儿车，看它能不能重新找到自己。）

## 代码导读

| 文件 | 作用 |
|---|---|
| [`config/nav2.yaml`](pg_nav2_py/config/nav2.yaml) | Nav2 全部参数，逐段注释（本课核心） |
| [`launch/nav2_sim.launch.py`](pg_nav2_py/launch/nav2_sim.launch.py) | 仿真 + Nav2 + RViz + 巡逻 |
| [`waypoints.py`](pg_nav2_py/pg_nav2_py/waypoints.py) | 航点文件解析 |
| [`patrol.py`](pg_nav2_py/pg_nav2_py/patrol.py) | 巡逻计划和统计 |
| [`patrol_node.py`](pg_nav2_py/pg_nav2_py/patrol_node.py) | 巡逻节点 `patrol` |
| [`path_metrics.py`](pg_nav2_py/pg_nav2_py/path_metrics.py) | 路径评价指标 |
| [`compare_planners_node.py`](pg_nav2_py/pg_nav2_py/compare_planners_node.py) | 规划器对比工具 `compare_planners` |
| [`config/waypoints.yaml`](pg_nav2_py/config/waypoints.yaml) | 巡逻航点 |
| [`rviz/nav2.rviz`](pg_nav2_py/rviz/nav2.rviz) | 地图、代价地图、路径、粒子、航点 |
| [`test/`](pg_nav2_py/test/) | 单元测试 |

建议阅读顺序：`config/nav2.yaml`（对照知识点 2 的结构图）→ `launch/nav2_sim.launch.py` →
`patrol.py` → `test/test_patrol.py` → `patrol_node.py` → `path_metrics.py` → `compare_planners_node.py`。

按提交记录学习：`git log --oneline -- lessons/05_nav2_navigation`。

## 实验记录

环境：WSL2 + Gazebo Harmonic，无窗口模式，turtlebot3_world 场景，仿真约 0.3 倍速。表中"用时"都是仿真时间。

| 日期 | 实验 | 参数 | 现象 | 结论 |
|---|---|---|---|---|
| 2026-11-24 | 单元测试 | `colcon test --packages-select pg_nav2_py` | `test_waypoints.py`、`test_patrol.py`、`test_path_metrics.py` 全部通过（含 flake8、pep257） | 航点解析、巡逻计划、路径指标都可以离线验证 |
| 2026-11-24 | 启动 | 默认 | 所有 Nav2 节点 `active [3]`；`/amcl_pose` 在 (0, 0) | `set_initial_pose: true` 生效，启动即定位好 |
| 2026-11-24 | 单个目标 | `send_goal` 到 (1.45, -0.05) | 成功；Gazebo 真值换算到地图坐标为 (1.19, -0.07) | 位置误差在 0.25 m 到达容差内 |
| 2026-11-24 | 规划器对比 | `compare_planners`，5 段共约 11.4 m | 见下表 | Smac2D 和 Theta\* 的路径更直；离障碍的距离三者差不多 |
| 2026-11-24 | 控制器对比 | 三个控制器各巡逻 1 圈 | 见下表，全部 5/5 到达、接触 0 次 | 都能完成；MPPI 最快、离障碍最远 |
| 2026-11-24 | 巡逻验收 | `acceptance:=true loops:=3` | 15/15 到达，恢复 0 次；里程 33.1 m，最小净空 0.126 m，接触 0 次 | 验收通过 |

**规划器对比**（巡逻路线 5 段合计）：

| 规划器 | 总长 m | 累计转弯 ° | 离障碍最小距离 m | 平均请求 ms |
|---|---|---|---|---|
| GridBased（NavFn） | 11.59 | 1200 | 0.35 | 10.9 |
| Smac2D | 11.33 | 339 | 0.33 | 5.3 |
| ThetaStar | 11.38 | 352 | 0.36 | 5.0 |

- 总长差不多（差 2%），差别主要在**累计转弯**：NavFn 沿着栅格上的梯度走，路径上有很多小锯齿，
  转弯量是另外两个的 3.5 倍。控制器会把小锯齿"抹平"，所以开起来差别没有这么大。
- 离障碍的距离由膨胀层决定（代价越高越不愿意走），三个规划器都在 0.33–0.36 m。
- 规划都只要几毫秒到二十几毫秒，比 1 Hz 的重新规划周期小得多。数字随 CPU 负载波动，不要太当真。
- 同一段 east→south，Smac2D 和 Theta\* 的累计转弯是 0：起点终点之间没有障碍，直接一条直线。

**控制器对比**（巡逻 1 圈，每个航点的用时 s）：

| 控制器 | north | east | south | center | home | 合计 | 最小净空 m | controller_server CPU |
|---|---|---|---|---|---|---|---|---|
| FollowPath（DWB） | 12.9 | 7.6 | 12.8 | 7.1 | 12.4 | 52.8 | 0.130 | 平均 9%，最高 20% |
| RPP | 12.4 | 8.5 | 11.3 | 7.4 | 9.0 | 48.6 | 0.146 | 平均 7%，最高 20% |
| MPPI | 11.5 | 8.2 | 10.9 | 8.2 | 7.9 | 46.7 | 0.196 | 平均 9%，最高 20% |

- MPPI 合计最快，离障碍也最远：它同时优化整段轨迹，转弯时提前切入，不完全贴着全局路径走。
- DWB 在终点附近的原地转向最慢（home 12.4 s，MPPI 只要 7.9 s）。
- CPU 占用三者差不多：仿真只有约 0.3 倍速，10 Hz 的控制周期按仿真时间算，现实中每秒只跑 3 次左右，
  MPPI 的计算量没体现出来。在真实时间运行的机器上差别会明显得多。
- 只跑了一圈，单个航点相差一两秒属于正常波动；要下结论应多跑几圈取平均（`loops:=5`）。

**开发中发现的问题**（都已修正，详见提交记录和 [踩坑记录](../../docs/troubleshooting.md#nav205-课)）：

| 现象 | 原因 | 解决 |
|---|---|---|
| 巡逻到一半 Nav2 整组停掉，日志 `SERVER map_server IS DOWN after not receiving a heartbeat` | 生命周期管理的心跳按墙钟计时，CPU 占满时误判 | launch 里 `SetParameter('bond_timeout', 0.0)` 关掉心跳 |
| 到点后停在 0.05–0.25 m 处，反复 `Failed to make progress`，单点用时从 12 s 变成 30–65 s | DWB 的 `xy_goal_tolerance`（0.05）与 goal_checker（0.25）不一致 | DWB 改为 0.25；进度检查改用 `PoseProgressChecker` |
| 规划器对比的"计算用时"几乎都是 0 | `planning_time` 按仿真时钟计时 | 改为自己用墙钟量请求用时 |

## 课后练习

1. **自己的航点**：在 RViz 里用 **Publish Point** 工具点几个位置（`ros2 topic echo /clicked_point` 看坐标），
   写一个新的航点文件，用 `waypoints_file:=` 参数巡逻。航点离柱子太近会怎样？
2. **按控制器切换**：给 `waypoints.yaml` 的每个航点加一个可选的 `controller` 字段，巡逻节点到这个点时先切换控制器
   （窄处用 RPP，开阔处用 MPPI）。先改 `waypoints.py` 和单元测试，再改节点。
3. **路上的新障碍**：巡逻时在 Gazebo 里放一个方块挡在路上（窗口模式下用顶部工具栏的立方体），观察局部代价地图怎样把它画进去、
   全局路径怎样绕开。把方块放在航点上又会怎样？
4. **用 follow_waypoints**：用 `BasicNavigator.followWaypoints()` 一次发完所有航点，和逐个发 `navigate_to_pose` 比较：
   能拿到什么信息、拿不到什么信息？
5. **调 MPPI**：把 `config/nav2.yaml` 里 MPPI 的 `batch_size` 从 1000 改成 300，`time_steps` 从 28 改成 15，
   看 `controller_server` 的 CPU 占用和轨迹平滑程度怎样变化。

## 常见问题

**问题**：RViz 里没有地图，或者提示 `Frame [map] does not exist`
**解决**：定位还没激活。`ros2 lifecycle get /map_server`、`ros2 lifecycle get /amcl` 应为 `active [3]`。
如果是 `unconfigured`，在日志里找 `map_server` 的第一条 ERROR，多半是地图路径不对（见 [踩坑记录](../../docs/troubleshooting.md#nav205-课)）。

**问题**：点了 2D Goal Pose，小车不动，日志里有 `Goal was rejected`
**解决**：`bt_navigator` 还没激活。等日志出现 `Managed nodes are active` 再发目标。

**问题**：小车在原地转圈、后退，最后报告失败
**解决**：这是行为树在执行恢复行为，说明规划或跟踪失败了。目标是不是在障碍物或膨胀区里？
在 RViz 里看全局代价地图：目标点所在格子的颜色是不是很"热"。

**问题**：小车开到目标附近（剩余零点几米）就停住，日志反复出现 `Failed to make progress`，恢复几次后才到
**解决**：检查控制器自己的到达容差和 `goal_checker` 是否一致。官方参数里 DWB 的 `xy_goal_tolerance` 是 0.05 m，
`goal_checker` 是 0.25 m：goal_checker 认为位置到了、只等朝向，DWB 却还没开始原地转向，只输出很小的角速度。
本课把 DWB 的改成了 0.25 m（`config/nav2.yaml` 里有详细注释）。

**问题**：巡逻节点一直打印"等待 AMCL 的定位结果"
**解决**：AMCL 还没收到初始位姿。本课的参数文件会自动设置；如果改过参数文件，在 RViz 里用 2D Pose Estimate 设置一次。

**问题**：`ros2 topic pub /controller_selector ...` 没效果
**解决**：要加 `--qos-durability transient_local`，见知识点 7。

**问题**：日志里大量 `Control loop missed its desired rate of 10.0000 Hz`
**解决**：仿真太慢（WSL2 没有独立显卡时常见），控制器跟不上 10 Hz。一般不影响结果；关掉 Gazebo 窗口（`gui:=false`）能好一些。

## 参考资料

- [Nav2 官方文档](https://docs.nav2.org/)
- [Nav2 参数配置指南](https://docs.nav2.org/configuration/index.html)
- [Macenski et al., The Marathon 2: A Navigation System（IROS 2020）](https://arxiv.org/abs/2003.00368)
- [nav2_simple_commander API](https://docs.nav2.org/commander_api/index.html)
- [规划器选择指南（NavFn、Smac、Theta\*）](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
- [Thrun, Burgard, Fox, Probabilistic Robotics 第 8 章：蒙特卡洛定位](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)
- [行为树在 Nav2 中的使用](https://docs.nav2.org/behavior_trees/index.html)
- [TurtleBot3 e-Manual：Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)
