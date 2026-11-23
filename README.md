# ROS 2 Playground

一个循序渐进的 ROS 2 学习仓库：从最基础的话题通信开始，一步步做到激光雷达避障、到点导航、SLAM 建图和 Nav2 自主导航，全部在 Gazebo 仿真里完成。

每一课都是一个独立文件夹，有自己的 README（学习目标、实验步骤、代码导读、课后练习），可以单独编译和运行。提交记录按"一个关键步骤一次提交"组织，可以配合 `git log` 逐步回看每一步做了什么。

## 环境

| 项目 | 版本 |
|---|---|
| 系统 | Windows 11 + WSL2 + Ubuntu 24.04 |
| ROS 2 | Jazzy Jalisco |
| 仿真器 | Gazebo Harmonic |
| 机器人 | TurtleBot3 Waffle（仿真） |

环境搭建见 [setup/](setup/)（一键安装脚本）和 [docs/environment.md](docs/environment.md)。

## 快速开始

```bash
# 1. 仓库放在工作空间的 src 目录下（整个仓库就是工作空间的源码目录）
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/mymuse77/ros2_playground.git

# 2. 安装依赖并编译
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

# 3. 运行任意一课，例如第 02 课的避障
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py
```

运行测试（代码风格检查 + 单元测试）：

```bash
cd ~/ros2_ws && colcon test && colcon test-result --verbose
```

## 学习路线

| 阶段 | 课程 | 主题 | 语言 |
|---|---|---|---|
| 一、基础 | [01](lessons/01_ros2_basics/) | ROS 2 核心概念：话题、服务、参数、动作、launch、bag | Python |
| 二、移动机器人自主 | [02](lessons/02_obstacle_avoidance/) | 激光雷达避障漫游 | Python |
| | [03](lessons/03_go_to_goal/) | 里程计、坐标变换与到点控制（Bug2 算法） | Python |
| | [04](lessons/04_slam_mapping/) | SLAM 建图 | Python + 配置 |
| | [05](lessons/05_nav2_navigation/) | Nav2 导航与多点巡逻 | Python + 配置 |
| 三、感知 | 06 | 相机与 OpenCV 颜色跟随 | Python |
| 四、Rust 与工程化 | 07 | Rust 节点：安全守护 | Rust |
| 五、自建机器人与综合 | 08 | 自建差速机器人（URDF + ros2_control） | 配置为主 |
| | 09（可选） | MoveIt 2 机械臂 | Python |
| | 10（可选） | 综合项目 | 混合 |

三个移动机器人课程在同一个场景里做，可以直接对比：02 课只看眼前（反应式避障），03 课知道自己在哪、要去哪（Bug2），05 课在地图上走最优路径（Nav2）。

## TODO 清单

完成一项勾选一项。每一项对应一次或几次提交，可以用 `git log --oneline` 对照查看。

### 阶段 0：仓库准备
- [x] 仓库骨架：README、LICENSE、`.gitignore`、`docs/`
- [x] 一键安装脚本放入 `setup/`
- [x] 公共包 `pg_interfaces`：自定义消息、服务、动作
- [x] 公共包 `pg_sim`：仿真启动（支持无窗口模式）

### 阶段一：基础（01 课）
- [x] 话题：发布者与订阅者（使用自定义消息）
- [x] 服务：重置计数器
- [x] 参数：运行时修改参数并立即生效
- [ ] 动作：可取消、带进度反馈的倒计时
- [ ] launch 文件 + 参数文件，一条命令启动全部节点
- [ ] 01 课 README（含 bag 录制与回放实验）

### 阶段二：移动机器人自主（02–05 课）
- [ ] 02 雷达扇区分析与避障决策算法 + 单元测试
- [ ] 02 避障节点（状态机，带超时保护）
- [ ] 02 launch、参数文件、RViz 配置
- [ ] 02 仿真验收：自动漫游 5 分钟无碰撞
- [ ] 02 课 README
- [ ] 03 位姿与角度工具函数 + 单元测试
- [ ] 03 到点控制器（比例控制）
- [ ] 03 Bug2 避障导航算法 + 单元测试
- [ ] 03 导航动作服务端与客户端（可取消、有进度反馈）
- [ ] 03 仿真验收：绕过障碍到达目标点
- [ ] 03 课 README
- [ ] 04 SLAM 建图 launch（键盘遥控 / 自动漫游两种方式）
- [ ] 04 地图保存工具
- [ ] 04 生成 turtlebot3_world 地图并放入 `pg_sim`
- [ ] 04 课 README
- [ ] 05 Nav2 导航 launch（使用 04 课的地图）
- [ ] 05 多点巡逻节点（`nav2_simple_commander`）
- [ ] 05 仿真验收：自动循环巡逻
- [ ] 05 课 README（含规划器、控制器对比实验）

### 后续阶段（06–10 课）
- [ ] 06 相机与 OpenCV 颜色跟随
- [ ] 07 Rust 安全守护节点
- [ ] 08 自建差速机器人
- [ ] 09 MoveIt 2 机械臂（可选）
- [ ] 10 综合项目（可选）

## 语言约定

- **Python**：默认语言，所有节点、launch 文件、工具脚本优先使用 Python。
- **Rust**：用于对性能或可靠性敏感的独立节点（07 课开始）。
- **C++**：只在框架要求时使用，例如 `ros2_control` 硬件接口、Nav2 插件。

## 目录结构

```
ros2_playground/
├─ README.md            本文件：总览、学习路线、TODO 清单
├─ docs/                环境说明、规范约定、命令速查、踩坑记录、课程模板
├─ setup/               一键安装脚本（Windows + WSL2）
├─ common/              各课共享的功能包
│   ├─ pg_interfaces/   自定义消息、服务、动作
│   └─ pg_sim/          仿真启动、场景、地图
└─ lessons/             每课一个文件夹，内含 README 和功能包
```

命名、代码风格、提交信息的规范见 [docs/conventions.md](docs/conventions.md)。

## 文档

- [docs/environment.md](docs/environment.md)：环境说明
- [docs/conventions.md](docs/conventions.md)：规范约定
- [docs/cheatsheet.md](docs/cheatsheet.md)：常用命令速查
- [docs/troubleshooting.md](docs/troubleshooting.md)：踩坑记录
- [docs/lesson_template.md](docs/lesson_template.md)：每课 README 模板

## 许可证

[Apache License 2.0](LICENSE)
