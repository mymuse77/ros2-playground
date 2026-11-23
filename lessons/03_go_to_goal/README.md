# 03 到点导航：P 控制 + Bug2 避障

> 告诉小车"去 (3, 0)"，它自己开过去；路上有墙就沿着墙绕过去。整个导航过程用一个**动作**提供：
> 发目标、看进度、随时取消。

## 学习目标

- 会从 `/odom` 取出小车位姿，处理四元数和角度跨越 ±180° 的问题
- 理解比例控制（P 控制）：误差越大修正越猛，以及"先转向再前进"等实用规则
- 理解 Bug2 算法：M-line、碰撞点、沿边走、离开条件、无法到达的判定
- 会写一个"真正干活"的动作服务端：控制循环、进度反馈、取消、抢占、超时
- 会写动作客户端：发送目标、限频打印反馈、Ctrl+C 时先取消目标再退出
- 会用 launch 参数在同一个 launch 文件里切换不同节点

## 前置条件

- 已完成的课程：[01 ROS 2 基础](../01_ros2_basics/)（动作）、[02 避障漫游](../02_obstacle_avoidance/)（雷达、状态机、安全措施）
- 依赖本仓库的包：`pg_interfaces`（动作定义）、`pg_sim`（仿真）、`pg_obstacle_avoidance_py`
  （复用 02 课的扇区距离函数，以及验收用的 `safety_monitor`）。下面的 `--packages-up-to` 会一起编译
- 需要额外安装的软件包：无
- 编译：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to pg_go_to_goal_py
source install/setup.bash
```

## 知识点

### 1. 位姿与角度

`/odom`（`nav_msgs/msg/Odometry`）里的朝向是四元数，平面机器人只需要绕 z 轴的转角 yaw：

```
yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
```

**所有角度相减之后都要归一化到 [-π, π)**。例如车头朝 170°，目标在 -170° 方向，
直接相减得 -340°（向右转一大圈），归一化后是 +20°（向左转一点）。这是新手最常见的 bug，
`geometry.py` 里的 `normalize_angle` 和 `heading_error` 就是为它准备的。

### 2. 到点控制：比例控制

```
朝向误差 e_θ = 目标方向 - 当前朝向（归一化）
角速度   ω   = k_angular × e_θ          （限幅 max_angular_speed）
线速度   v   = k_linear  × 距离 × cos(e_θ)（限幅 max_linear_speed）
```

两条实用规则：

| 规则 | 没有它会怎样 |
|---|---|
| 朝向误差 > 45° 时**原地转向**，不前进 | 目标在身后时，车会边走边转，画一个很大的弧 |
| 线速度乘以 `cos(e_θ)` | 方向偏得越多越不该往前冲，否则会冲过头再绕回来 |

距离小于 `goal_tolerance`（0.1 m）就算到达。容差不能设成 0：里程计有噪声，车永远停不到"正好"的位置。

### 3. Bug2 算法

不需要地图，只需要"我在哪、目标在哪、周围有没有障碍"：

```
            ★ 目标
            │
       ┌────┼────┐  ← 3. 被挡住：记下碰撞点 H，沿障碍物边缘走（障碍物在右手边）
       │ 障 │ 碍 │
       └────┼────┘  ← 4. 回到 M-line，比 H 离目标更近，朝目标方向畅通 → 离开障碍
            │
            │       ← 2. 沿 M-line 朝目标走
            ● 起点      1. 起点到目标连一条直线：M-line
```

| 状态 | 行为 | 切换条件 |
|---|---|---|
| GO_TO_GOAL | P 控制朝目标走 | 前方 < `hit_distance` 且车开到目标前会碰到它 → FOLLOW_WALL |
| FOLLOW_WALL | 右手贴墙走 | 满足离开条件 → GO_TO_GOAL；绕一圈回到 H → FAILED |
| REACHED | 停车 | 离目标 < 容差 |
| FAILED | 停车 | 目标被障碍物围住，无法到达 |

**离开条件为什么要"比碰撞点更近"**：否则车可能在 M-line 上的同一点反复"离开—碰撞—离开"，
永远在原地打转。这个条件保证每离开一次都离目标更近，所以 Bug2 一定会结束（到达或判定失败）。

**"会碰到它"怎么判断**：车是有大小的。车中心到达目标时，车头已经伸到"目标距离 + 车体半径"处，
所以判断条件是 `前方距离 < 目标距离 + robot_radius`，而不是 `前方距离 < 目标距离`。
开发时在仿真里遇到过：目标在方块中心，用后一种写法时，里程计误差加上雷达装在车中心后方，
使"障碍比目标近"不成立，车直接顶着方块打滑，里程计却报告"已到达"
（Gazebo 里的真实位置离目标还有 0.5 m）。单元测试 `test_closed_loop_goal_inside_box_fails` 记录了这个场景。

### 4. 沿边走：同时看距离和方向

在车的右半边（-180° ~ 0°）找最近的障碍点，控制两个量：

```
ω = k_wall × (wall_distance - 最近距离) + k_wall_angle × (最近点方向 + 90°)
       距离：离墙太近向左、太远向右           方向：让最近点保持在正右方
```

只看"正右方扇区的距离"是不够的。开发时在仿真里遇到过：绕过墙的外拐角后，墙角跑到了右后方，
不在正右方扇区里，算法以为墙丢了，一直原地右转打小圈。加上方向项后，墙角在右后方时
方向误差为负，车会绕着墙角转过去。单元测试 `test_outer_corner_turns_right` 记录了这个场景。

### 5. 动作服务端：一个真正干活的执行回调

01 课的倒计时动作只是 sleep，这里的执行回调是一个完整的控制循环：

```
每个控制周期（10 Hz）：
  检查：节点关闭？取消请求？被新目标抢占？超时？ → 停车，结束目标
  读传感器快照：数据过期 → 发零速度，等待（不结束目标）
  Bug2 计算速度 → 发布 /cmd_vel
  到达 / 失败 → 停车，返回结果
  发布反馈：当前位置、剩余距离、状态；发布 RViz 标记
```

几个设计选择：

| 选择 | 原因 |
|---|---|
| 新目标**抢占**旧目标 | 与 Nav2 一致。用户改主意时不用先取消 |
| 多线程执行器 + 回调组 | 执行回调一直在循环，传感器回调必须能在别的线程里同时运行 |
| 循环节拍用稳定时钟 | 与 02 课的看门狗同理：仿真卡住时也能发现数据超时 |
| 超时 `goal_timeout` 用 ROS 时间 | 衡量的是"机器人走了多久"，仿真慢时不应提前超时 |
| 每个目标开始时读一次参数 | 运行中 `ros2 param set` 对下一个目标生效，不会在半路改变行为 |

### 6. 动作客户端：Ctrl+C 时先取消，回应丢了要重试

如果客户端被 Ctrl+C 直接杀掉，服务端并不知道，小车会继续开向旧目标。所以客户端接管 SIGINT，
先发取消请求、等服务端确认，再退出。和 02 课一样，等待时不用 `spin_until_future_complete`
（阻塞在 C 层，Ctrl+C 不能及时生效），而是用 `spin_once` 循环。

动作在底层由 3 个服务 + 2 个话题组成：

| 组成 | 类型 | 作用 |
|---|---|---|
| `send_goal` | 服务 | 发目标，回应"接受 / 拒绝" |
| `cancel_goal` | 服务 | 取消 |
| `get_result` | 服务 | 取结果（目标结束后才回应） |
| `feedback` | 话题 | 进度 |
| `status` | 话题 | 所有目标的状态 |

开发时遇到过：客户端刚启动、DDS 还没完成发现时，服务端的"已接受"回应没送到
（服务端日志 `failed to send response (timeout)`），客户端收得到反馈（话题），却拿不到目标句柄，
永远等不到结果。所以客户端：

- 等"接受/拒绝"最多 5 秒，超时就重发目标（服务端的抢占规则会让那个没人认领的旧目标结束）
- 从 `status` 话题看到目标已经结束、结果却迟迟不到时，重新请求一次结果

### 7. 代码结构

```
geometry.py            位姿、角度、点到直线距离      纯 Python → 单元测试
go_to_goal.py          P 控制器                      纯 Python → 单元测试（含运动学闭环）
bug2.py                Bug2 状态机、沿边走           纯 Python → 单元测试（含射线模拟雷达的闭环）
ros_utils.py           消息转换、退出前停车的主循环   各节点共用
go_to_goal_node.py     到点节点（/goal_pose → /cmd_vel，不避障）
navigate_server_node.py  导航动作服务端（Bug2）
navigate_client_node.py  导航动作客户端
```

## 一键运行

```bash
# 终端 A：仿真 + 导航服务端 + RViz（默认场景：本课的 Bug2 练习场）
ros2 launch pg_go_to_goal_py navigate_sim.launch.py
# 终端 B：发目标
ros2 run pg_go_to_goal_py navigate_client 3.0 0.0
```

场景布置（俯视，小车出生在原点、车头朝 +x）：

```
      2 ┤     ■ 方块 (-1.5, 1.5)
      1 ┤             ┃
      0 ┤   ●────────>┃─────────★ 目标 (3.0, 0)
     -1 ┤  出生点      ┃ 墙 x=1.5，长 2 m
     -2 ┤      ◯ 圆柱 (0, -2)
        └──────────────────────────▶ x
           -1   0    1    2    3          四周有 8 m × 8 m 围墙
```

预期效果：小车朝 +x 开，在墙前约 0.45 m 处停下左转，沿墙向上走，绕过墙的上端后回到 x 轴
（M-line），转向目标开过去。终端 B 每秒打印一次进度，最后打印里程和用时。
RViz 中灰线是 M-line，绿色圆盘是目标，红点是碰撞点，蓝色箭头是走过的轨迹。

## 实验步骤

### 实验 1：P 控制（不避障）

```bash
ros2 launch pg_go_to_goal_py navigate_sim.launch.py mode:=p
```

在 RViz 工具栏点 **2D Goal Pose**，在空地上点一下（拖动的方向会被忽略）。观察：

- 目标在身后时，车是先原地转还是边走边转？
- 快到目标时速度怎么变？
- 把目标点在墙后面：车会直接撞上去——这个节点故意不避障

**思考**：`rotate_threshold_deg` 改成 180（即永远不原地转），目标在身后时轨迹会变成什么样？

```bash
ros2 param describe /go_to_goal rotate_threshold_deg   # 说明、单位，Read only: true
ros2 param set /go_to_goal k_angular 4.0               # 被拒绝：只读参数
```

到点节点只在启动时读一次参数，所以把参数声明成只读，运行中修改会被明确拒绝，
而不是"显示设置成功、行为却没变"。要改参数需要修改 `config/navigate.yaml` 后重新 launch。
对比 `navigate_server`：它有参数回调，可以运行中修改（见实验 5）。

### 实验 2：Bug2 绕墙

```bash
ros2 launch pg_go_to_goal_py navigate_sim.launch.py
ros2 run pg_go_to_goal_py navigate_client 3.0 0.0
```

也可以用命令行工具直接发目标，看原始的反馈消息：

```bash
ros2 action send_goal --feedback /navigate_to_point pg_interfaces/action/NavigateToPoint "{x: 3.0, y: 0.0}"
```

观察服务端日志中的状态切换，以及 RViz 中碰撞点的位置。

### 实验 3：取消与抢占

```bash
ros2 run pg_go_to_goal_py navigate_client 3.0 0.0 --cancel-after 15   # 15 秒后取消
```

- 运行中按 Ctrl+C：客户端会先取消目标再退出，小车停下
- 一个目标执行中，另开终端再发一个目标：旧目标以"被新目标取代"结束，车转向新目标

### 实验 4：无法到达的目标

```bash
ros2 run pg_go_to_goal_py navigate_client -1.5 1.5    # 目标在方块里面
```

小车会绕方块一圈，回到碰撞点附近时报告"目标无法到达"。

结束后对比里程计和 Gazebo 里的真实位置：

```bash
ros2 topic echo --once /odom --field pose.pose.position   # 里程计认为的位置
gz model -m waffle -p                                     # Gazebo 中的真实位姿
```

两者会差出几十厘米：里程计靠轮子转了多少圈推算位置，原地转向和贴墙绕行时轮子会打滑，
误差随路程不断累积，而且永远不会自己消失。这就是 04、05 课要用地图和定位（AMCL）修正位置的原因。

### 实验 5：调参

```bash
ros2 param set /navigate_server wall_distance 0.3   # 贴墙更近
ros2 param set /navigate_server hit_distance 0.6    # 更早开始绕
```

参数对**下一个目标**生效。比较不同参数下的里程和用时（客户端最后一行）。

**思考**：`hit_distance` 小于 `wall_distance` 会怎样？（提示：刚开始沿边走时前方就被挡住）

### 实验 6：换个场景

```bash
ros2 launch pg_go_to_goal_py navigate_sim.launch.py world:=world x_pose:=-2.0 y_pose:=-0.5
ros2 run pg_go_to_goal_py navigate_client 2.0 0.5
```

注意：目标是 **odom 坐标系**（以出生点为原点），不是 Gazebo 的世界坐标。从 (-2, -0.5) 出生时，
odom 的 (2.0, 0.5) 对应世界坐标 (0, 0)，正好是中间那根柱子，可以试试会发生什么。

## 代码导读

| 文件 | 作用 |
|---|---|
| [`geometry.py`](pg_go_to_goal_py/pg_go_to_goal_py/geometry.py) | 位姿、四元数、角度归一化、点到直线距离 |
| [`go_to_goal.py`](pg_go_to_goal_py/pg_go_to_goal_py/go_to_goal.py) | P 控制器 |
| [`bug2.py`](pg_go_to_goal_py/pg_go_to_goal_py/bug2.py) | Bug2 状态机、沿边走（本课核心） |
| [`ros_utils.py`](pg_go_to_goal_py/pg_go_to_goal_py/ros_utils.py) | 消息转换、退出前停车 |
| [`go_to_goal_node.py`](pg_go_to_goal_py/pg_go_to_goal_py/go_to_goal_node.py) | 到点节点 |
| [`navigate_server_node.py`](pg_go_to_goal_py/pg_go_to_goal_py/navigate_server_node.py) | 导航动作服务端 |
| [`navigate_client_node.py`](pg_go_to_goal_py/pg_go_to_goal_py/navigate_client_node.py) | 导航动作客户端 |
| [`launch/navigate_sim.launch.py`](pg_go_to_goal_py/launch/navigate_sim.launch.py) | 仿真 + 导航节点（`mode` 切换）+ RViz |
| [`config/navigate.yaml`](pg_go_to_goal_py/config/navigate.yaml) | 全部参数及说明 |
| [`worlds/bug2_course.sdf`](pg_go_to_goal_py/worlds/bug2_course.sdf) | Bug2 练习场景 |
| [`test/`](pg_go_to_goal_py/test/) | 单元测试，`test_bug2.py` 里有一个射线模拟雷达，可以离线跑闭环 |

建议阅读顺序：`geometry.py` → `go_to_goal.py` → `test/test_go_to_goal.py` → `bug2.py` →
`test/test_bug2.py` → `navigate_server_node.py` → `navigate_client_node.py` → launch。

按提交记录学习：`git log --oneline -- lessons/03_go_to_goal`。

## 实验记录

环境：WSL2 + Gazebo Harmonic，无窗口模式，仿真约 0.5 倍速。

| 日期 | 实验 | 参数 | 现象 | 结论 |
|---|---|---|---|---|
| 2026-11-24 | 单元测试 | `colcon test --packages-select pg_go_to_goal_py` | 152 项全部通过（含 flake8、pep257） | 算法可以离线验证 |
| 2026-11-24 | 验收：绕墙到 (3, 0) | 默认参数 | 在 (1.01, 0) 进入 FOLLOW_WALL，(1.97, 0.09) 回到 GO_TO_GOAL；到达，误差 0.098 m，里程 5.81 m，仿真用时 44.8 s（墙钟 87 s）；`safety_monitor`：最小净空 0.132 m，接触 0 次 | 通过 |
| 2026-11-24 | 沿边走第一版 | 只看正右方 ±30° 扇区的距离 | 绕过墙的外拐角后原地打小圈 | 必须同时看最近点的方向，见知识点 4 |
| 2026-11-24 | 取消 | `--cancel-after 5` | 5.1 s 时取消，已走 1.02 m，车停下 | 取消生效 |
| 2026-11-24 | Ctrl+C 客户端 | 执行中按 Ctrl+C | 服务端日志"目标已取消"，车停下 | 客户端退出前先取消 |
| 2026-11-24 | 非法目标 | `x: .nan` | 目标被拒绝 | 输入检查生效 |
| 2026-11-24 | 抢占 | 执行中发新目标 | 旧目标"被新目标取代"（1.40 m / 13.0 s）；新目标到达，误差 0.099 m | 与 Nav2 行为一致 |
| 2026-11-24 | 无法到达（第一版） | 目标 (-1.5, 1.5)，判断条件"障碍比目标近" | 没有进入 FOLLOW_WALL，顶着方块打滑后报告"已到达"；Gazebo 真实位置 (-1.05, 1.73) | 判断要计入车体半径，见知识点 3 |
| 2026-11-24 | 无法到达 | 目标 (-1.5, 1.5)，加入 `robot_radius` 0.22 | 在 (-1.17, 1.20) 进入 FOLLOW_WALL，绕一圈后报告无法到达，里程 7.06 m，用时 57.2 s；结束时里程计与真实位置相差约 0.5 m | 通过；里程计误差会累积 |
| 2026-11-24 | 关闭服务端 | 执行中 Ctrl+C 服务端 | `/cmd_vel` 从 0.22 变为 0.0，日志"节点正在关闭" | 退出前停车 |
| 2026-11-24 | 原地转向 135° | 角速度 1.0 / 0.3 rad/s | 里程计 136.1° / 135.1°，与真实朝向基本一致 | 单次转向误差很小，误差主要来自长距离累积和碰撞打滑 |

## 课后练习

1. **Bug1 对比**：实现 Bug1（绕障碍物完整一圈，记下离目标最近的点，再回到那里离开），
   用 `test_bug2.py` 里的射线模拟器比较 Bug1 和 Bug2 在不同障碍物形状下的路径长度。
2. **左手还是右手**：现在固定右手贴墙。改成根据碰撞时左右哪边更开阔来选择方向，
   在实验 2 的场景里能省多少路？
3. **PID**：给到点控制器的角速度加上微分项（D），观察转向时的超调是否减少。
4. **到达后对准朝向**：给 `NavigateToPoint` 加一个可选的目标朝向，到达位置后原地转到该朝向。
   需要修改 `pg_interfaces`，体会接口变更对客户端和服务端的影响。
5. **更安全**：在服务端里复用 02 课的思路——FOLLOW_WALL 时如果最近距离小于 0.2 m，先停车后退。

## 常见问题

**问题**：客户端提示"等不到服务端 navigate_to_point"
**解决**：确认 launch 时没有设置 `mode:=p`；`ros2 action list` 应能看到 `/navigate_to_point`。

**问题**：到点节点报"只接受 odom 坐标系的目标"
**解决**：RViz 的 Fixed Frame 要设为 `odom`（本课的 RViz 配置已经设好）。用 `map` 坐标系需要 tf 变换，05 课会讲。

**问题**：小车沿墙走时一直原地转小圈
**解决**：说明沿边走丢了墙。检查 `wall_distance` 是否远小于 `hit_distance`，
或者 `ros2 topic hz /scan` 是否太低。本课的沿边走算法已经处理了外拐角，见知识点 4。

**问题**：目标明明可达，却报告"绕障碍物一圈回到碰撞点"
**解决**：沿边走时离墙太远，可能"跨过"了 M-line 而没检测到。适当增大 `mline_tolerance`（代码参数），或减小 `wall_speed`。

**问题**：仿真里小车动作很慢
**解决**：无窗口模式（`gui:=false`）更快。WSL2 下仿真常常只有 0.3 倍速左右，节点用仿真时间，行为不受影响。

## 参考资料

- [Lumelsky & Stepanov, Path-planning strategies for a point mobile automaton（Bug 算法原始论文）](https://link.springer.com/article/10.1007/BF01840369)
- [ROS 2 动作概念](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Actions.html)
- [编写 Python 动作服务端与客户端](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [执行器与回调组](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html)
- [REP-105：机器人坐标系约定（odom、map）](https://www.ros.org/reps/rep-0105.html)
- [nav_msgs/msg/Odometry 定义](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)
