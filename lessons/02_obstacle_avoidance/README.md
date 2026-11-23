# 02 避障漫游：激光雷达 + 状态机

> 让 TurtleBot3 在仿真世界里自己到处走、不撞墙：读激光雷达，划分扇区，用一个两状态的状态机决定怎么走。

## 学习目标

- 读懂 `sensor_msgs/msg/LaserScan`：角度怎么算、哪些读数无效
- 会把"一圈距离"归纳成少量有意义的特征（扇区最近距离）
- 会设计一个简单的状态机，理解迟滞和方向锁定为什么能消除抖动
- 掌握机器人控制节点的安全写法：数据超时停车、退出时停车
- 会把算法写成不依赖 ROS 的纯 Python 模块，并用合成数据做单元测试
- 会用 launch 复用其他包的启动文件，用 RViz 标记把算法内部状态画出来

## 前置条件

- 已完成的课程：[01 ROS 2 基础](../01_ros2_basics/)
- 需要额外安装的软件包：无（安装脚本已装好 Gazebo 和 TurtleBot3）
- 编译：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to pg_obstacle_avoidance_py
source install/setup.bash
```

## 知识点

### 1. 激光雷达数据

TurtleBot3 waffle 的雷达每秒转 10 圈，每圈 360 个读数：

| 字段 | 值 | 含义 |
|---|---|---|
| `angle_min` | 0 | 第一个读数的方向：正前方 |
| `angle_increment` | 约 0.0175 rad（1°） | 相邻读数的角度间隔，**逆时针**为正 |
| `range_min` / `range_max` | 0.12 / 3.5 m | 有效量程 |
| `ranges[i]` | 距离，m | 第 i 个读数，方向 = `angle_min + i × angle_increment` |

所以 `ranges[0]` 是正前方，`ranges[90]` 是左边，`ranges[270]` 是右边（也就是 -90°）。

读数的三种特殊情况：

- `inf`：这个方向在量程内没有障碍，按 `range_max` 计
- `NaN` 或小于 `range_min`：测量失败，跳过
- 扇区里一个有效读数都没有：当作"被挡住"。宁可多转一下，也不要盲目往前冲

### 2. 扇区划分

只关心车前方的半圈，分成三块：

```
                前方 front（±30°）
            ╲       │       ╱
     左前     ╲     │     ╱     右前
  30° ~ 90°     ╲   │   ╱    -90° ~ -30°
   ──────────────[ 车 ]──────────────    ← 车后方不参与决策
```

每个扇区只取一个数：**最近距离**。360 个数变成 3 个数，后面的决策就简单了。

### 3. 状态机

```
                    前方 < stop_distance
                    （选择更开阔的一侧，锁定方向）
         ┌─────────┐ ───────────────────────▶ ┌─────────┐
         │ FORWARD │                           │ TURNING │ ──┐ 同方向转太久
         └─────────┘ ◀─────────────────────── └─────────┘ ◀─┘ → 反方向转
                    前方 > clear_distance
```

- **FORWARD**：前方越近速度越慢（`slow_distance` 到 `stop_distance` 线性减速）；
  侧方太近时，往另一侧偏转一点
- **TURNING**：原地转向，直到前方开阔

三个让它"不抖"的设计：

| 设计 | 没有它会怎样 |
|---|---|
| **迟滞**：小于 0.5 m 开始转，大于 0.7 m 才停 | 距离在 0.5 m 附近波动时，每帧都在"转 / 不转"之间切换 |
| **方向锁定**：开始转向时选定方向，转完前不改 | 两侧距离差不多时，每帧重新比较，左一下、右一下原地打摆 |
| **超时换向**：同一方向转了 6 s 还没脱困就反过来 | 在 U 形角落里可能一直朝一个方向转却找不到出口 |

### 4. 安全措施

控制真实机器人时，"出问题时让车停下"比"算法多聪明"更重要。

| 情况 | 处理 |
|---|---|
| 雷达数据中断（驱动崩溃、网线松动） | 超过 `scan_timeout`（0.5 s）没有新数据就发零速度 |
| 节点退出（Ctrl+C、launch 关闭） | 先发零速度并等对方确认收到，再退出 |
| 运行时把参数改成不合理的组合 | 整体校验，直接拒绝 |

退出停车有两个坑，本课代码里都处理了（详见 `avoider_node.py` 的 `main()` 注释）：

1. rclpy 默认的 Ctrl+C 处理会**先关闭 ROS**，之后就发不出消息了，所以要关掉默认处理，自己接管
2. `rclpy.spin()` 一直阻塞在 C 代码里，没有消息时 Python 的信号处理函数没机会运行。改用 `spin_once` 循环

看门狗还有一个细节：**超时判断用系统的稳定时钟，不用仿真时钟**。仿真卡住时仿真时钟也停了，
用它计时就永远发现不了超时。

### 5. 代码结构：算法与 ROS 分离

```
logic.py          扇区分析 + 状态机       纯 Python，不 import rclpy → 单元测试
markers.py        RViz 扇形标记           只用消息类型               → 单元测试
safety.py         净空、里程、卡住统计     纯 Python                  → 单元测试
avoider_node.py   避障节点                 只负责收发消息、参数、安全
safety_monitor_node.py  验收节点           独立旁观，不参与控制
```

状态机不自己读时钟，时间由调用者传进来。这样单元测试里可以直接说"现在是第 6.1 秒"，
不用真的等 6 秒。

## 一键运行

```bash
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py
```

预期效果：Gazebo 和 RViz 两个窗口打开，小车开始在 turtlebot3_world 里漫游。
RViz 中车前方有三个半透明扇形，绿色表示开阔、黄色表示减速区、红色表示要转向，扇形半径就是该扇区的最近距离。
车头上方显示当前状态（FORWARD / TURNING），终端里打印每次状态切换。

其他场景：`world:=house`（更复杂的室内）、`world:=empty`（空地）。
不要窗口：`gui:=false rviz:=false`。

## 实验步骤

### 实验 1：观察雷达数据

先单独启动仿真，不开避障：

```bash
ros2 launch pg_sim tb3_sim.launch.py rviz:=true
```

另开终端：

```bash
ros2 topic echo --once /scan --field ranges | head -c 600     # 前几十个读数
ros2 topic echo --once /scan --field angle_increment
ros2 topic hz /scan
```

**思考**：`ranges[0]`、`ranges[90]`、`ranges[180]` 分别是哪个方向？在 Gazebo 里拖动小车（或者往车前放个方块），验证你的判断。

### 实验 2：运行避障，观察状态机

```bash
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py
```

另开终端：

```bash
ros2 topic echo /avoider/state           # 状态
ros2 topic echo /cmd_vel --field twist   # 速度指令
```

观察：接近障碍时速度怎么变？转向时选的是哪一侧？RViz 里扇形颜色和状态切换是否对应？

### 实验 3：调参

运行中修改参数，立即生效：

```bash
ros2 param list /avoider
ros2 param describe /avoider stop_distance
ros2 param set /avoider max_linear_speed 0.26       # 更快
ros2 param set /avoider stop_distance 0.35          # 更大胆
ros2 param set /avoider stop_distance 0.9           # 大于 clear_distance，会被拒绝
ros2 param set /avoider front_half_angle_deg 15.0   # 前方扇区变窄
```

**思考**：
- 前方扇区变窄到 15° 后，小车在经过柱子时有什么变化？为什么可能擦到柱子侧面？
- 把 `clear_distance` 改成 0.51（几乎没有迟滞），观察转向结束时是否出现抖动。

### 实验 4：安全措施

1. 雷达断线：运行避障时，在另一个终端执行 `pkill -f parameter_bridge`，观察小车和日志
   （注意：这同时会断开 `/clock` 和 `/cmd_vel`，只能看到日志；更干净的做法见课后练习 3）
2. 退出停车：按 Ctrl+C 关掉 launch 之前，另开终端运行 `ros2 topic echo /cmd_vel --field twist.linear.x`，
   观察最后一条消息是不是 0

### 实验 5：自动验收

避障节点自己说"我没撞"不算数，用独立的 `safety_monitor` 来裁判：

```bash
# 终端 A：无窗口运行避障
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py gui:=false rviz:=false
# 终端 B：统计 300 秒（仿真时间），结束时打印报告；退出码 0 = 通过
ros2 run pg_obstacle_avoidance_py safety_monitor --ros-args -p use_sim_time:=true -p duration:=300.0
echo $?
```

`safety_monitor` 把车身近似成矩形，计算每个雷达点到**车身边缘**的距离（净空），
而不是到雷达中心的距离，这样才能真实反映"差多少会撞上"。

### 实验 6：换个场景

```bash
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py world:=house
```

house 场景里有门、走廊、家具，比 turtlebot3_world 难得多。观察小车会在哪里遇到困难，想想为什么。

## 代码导读

| 文件 | 作用 |
|---|---|
| [`logic.py`](pg_obstacle_avoidance_py/pg_obstacle_avoidance_py/logic.py) | 扇区分析、避障参数、状态机（本课核心） |
| [`avoider_node.py`](pg_obstacle_avoidance_py/pg_obstacle_avoidance_py/avoider_node.py) | 避障节点：订阅雷达、发布速度、参数管理、安全措施 |
| [`markers.py`](pg_obstacle_avoidance_py/pg_obstacle_avoidance_py/markers.py) | 把扇区画成 RViz 扇形标记 |
| [`safety.py`](pg_obstacle_avoidance_py/pg_obstacle_avoidance_py/safety.py) | 车身净空、里程、卡住统计 |
| [`safety_monitor_node.py`](pg_obstacle_avoidance_py/pg_obstacle_avoidance_py/safety_monitor_node.py) | 验收节点 |
| [`launch/avoid_sim.launch.py`](pg_obstacle_avoidance_py/launch/avoid_sim.launch.py) | 仿真 + 避障 + RViz |
| [`config/params.yaml`](pg_obstacle_avoidance_py/config/params.yaml) | 全部避障参数及说明 |
| [`test/`](pg_obstacle_avoidance_py/test/) | 单元测试：合成雷达数据测试扇区和状态机 |

建议阅读顺序：`logic.py` → `test/test_logic.py`（看测试就知道每个函数该怎么用）→ `avoider_node.py`
→ `markers.py` → `safety.py` → `safety_monitor_node.py` → `avoid_sim.launch.py`。

按提交记录学习：`git log --oneline -- lessons/02_obstacle_avoidance`。
"避障节点"那次提交的说明里记录了开发中踩过的两个坑。

## 实验记录

开发时在 WSL2（无独立显卡，Gazebo 无窗口模式，仿真约 0.3 倍速）上的实测记录，供对照：

| 日期 | 实验 | 参数 | 现象 | 结论 |
|---|---|---|---|---|
| 2026-11-24 | 单元测试 | `colcon test` | 算法、标记、安全统计共 105 项全部通过（含 flake8、pep257） | 算法在合成数据上行为正确 |
| 2026-11-24 | 雷达数据 | turtlebot3_world | `/scan` 360 个读数，`angle_min` 0，量程 0.12–3.5 m；`topic hz` 约 3.1–3.7 Hz | 仿真慢于真实时间，频率按墙钟统计所以偏低，不影响节点逻辑 |
| 2026-11-24 | RViz 标记 | 默认 | `/avoider/sectors` 每帧 4 个标记（3 个扇形 + 状态文字），坐标系 `base_scan` | 扇形与雷达读数一致 |
| 2026-11-24 | 调参 | `stop_distance 0.9` | 被拒绝：`要求 0 < stop_distance < clear_distance`，原参数保持不变 | 参数整体校验生效 |
| 2026-11-24 | 调参 | 运行时修改 `control_rate_hz` | 被拒绝：`Trying to set a read-only parameter` | 只读参数只能在启动时设置 |
| 2026-11-24 | 雷达断线 | 用转发节点切断 `/scan` | 0.5 s 后速度变为 0，日志提示雷达超时；恢复转发后继续漫游 | 看门狗生效。最初用仿真时钟计时，仿真卡住时看门狗也"卡住"，改为稳定时钟后正常 |
| 2026-11-24 | 退出停车 | Ctrl+C / `kill -TERM` | 两种方式下 `/cmd_vel` 最后一条都从 0.20 变为 0.0 | 退出前停车生效（后台启动时 SIGINT 被忽略，需显式安装处理函数） |
| 2026-11-24 | 自动验收 | turtlebot3_world，300 s | 里程 34.80 m，最小净空 0.087 m，接触 0 次，未卡住，状态切换 57 次；墙钟 953 s | **通过** |

## 课后练习

1. **记忆**：现在的算法只看当前一帧，转过去之后就"忘了"刚才的障碍。给控制器加一个简单的记忆
   （比如记录最近 N 次转向的方向），减少在同一个角落反复转的情况。先写单元测试。
2. **更平滑**：现在 FORWARD 和 TURNING 之间速度是跳变的。加一个速度斜坡（限制加速度），
   比较前后 `/cmd_vel` 的曲线（`rqt_plot /cmd_vel/twist/linear/x`）。
3. **更干净的断线测试**：写一个"雷达开关"节点，订阅 `/scan` 转发到 `/scan_gated`，提供一个
   `std_srvs/srv/SetBool` 服务控制是否转发；避障节点用 `-r scan:=/scan_gated` 订阅它。
4. **跟墙走**：新增一个 FOLLOW_WALL 状态，沿右侧墙保持 0.4 m 距离前进（03 课 Bug2 算法会用到）。
5. **验收更严格**：让 `safety_monitor` 额外统计"转向时间占比"，占比过高说明避障太保守。

## 常见问题

**问题**：小车不动，日志停在"等待雷达数据……"
**解决**：检查 `ros2 topic hz /scan` 是否有数据。仿真刚启动时需要十几秒，雷达桥接起来后会自动开始。

**问题**：小车原地转圈不停
**解决**：多半是前方扇区被近处障碍挡住，转到哪里前方都小于 `clear_distance`。检查 `clear_distance`
是否设得太大，或者把小车移到开阔处。`max_turn_time` 到时会自动换向。

**问题**：`ros2 topic echo /cmd_vel` 报类型错误或收不到
**解决**：TurtleBot3 Jazzy 版用的是 `geometry_msgs/msg/TwistStamped`（带时间戳），不是 `Twist`。

**问题**：仿真比现实慢，`ros2 topic hz /scan` 不到 10 Hz
**解决**：无窗口模式（`gui:=false`）能明显加快。节点使用仿真时间，行为不受影响，只是现实中看起来慢。

**问题**：修改 `config/params.yaml` 后没有生效
**解决**：确认编译时用了 `--symlink-install`，并且重新 launch 了。

## 参考资料

- [sensor_msgs/msg/LaserScan 定义](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [REP-103：坐标系与单位约定](https://www.ros.org/reps/rep-0103.html)
- [QoS 设置（为什么雷达要用 sensor_data）](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [RViz 标记：Marker 类型说明](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)
- [launch 文件中包含其他 launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [TurtleBot3 e-Manual（Jazzy 仿真）](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
