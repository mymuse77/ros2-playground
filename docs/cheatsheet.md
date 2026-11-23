# 常用命令速查

## 工作空间

```bash
cd ~/ros2_ws
colcon build --symlink-install                        # 编译全部；--symlink-install 让 Python 修改后不用重新编译
colcon build --packages-select pg_basics_py           # 只编译某个包
colcon build --packages-up-to pg_obstacle_avoidance_py # 编译某个包及其依赖
source install/setup.bash                             # 编译后加载环境（新终端都要执行，已写进 ~/.bashrc）
colcon test --packages-select pg_basics_py && colcon test-result --verbose
rosdep install --from-paths src --ignore-src -y       # 安装 package.xml 里声明的依赖
rm -rf build install log                              # 清理编译产物
```

## 节点与话题

```bash
ros2 node list                         # 列出节点
ros2 node info /avoider                # 节点的订阅、发布、服务、参数
ros2 topic list -t                     # 列出话题及类型
ros2 topic echo /scan --once           # 打印一条消息
ros2 topic hz /scan                    # 测量发布频率
ros2 topic info /cmd_vel -v            # 发布者、订阅者及 QoS
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}}}"        # 以 10 Hz 发布消息
ros2 interface show pg_interfaces/msg/Greeting   # 查看消息定义
rqt_graph                              # 图形化查看节点连接
```

## 服务、参数、动作

```bash
ros2 service list -t
ros2 service call /reset_counter pg_interfaces/srv/ResetCounter "{start_value: 100}"
ros2 param list /greeter
ros2 param get /greeter publish_rate_hz
ros2 param set /greeter publish_rate_hz 5.0
ros2 param dump /greeter > params.yaml
ros2 param describe /avoider stop_distance     # 参数说明、类型、取值范围、是否只读
ros2 action list -t
ros2 action info /navigate_to_point            # 动作的服务端和客户端
ros2 action send_goal --feedback /countdown pg_interfaces/action/Countdown "{seconds: 5}"
ros2 action send_goal --feedback /navigate_to_point pg_interfaces/action/NavigateToPoint \
  "{x: 3.0, y: 0.0}"                           # 03 课：导航到 odom 坐标系的 (3, 0)
```

## launch

```bash
ros2 launch pg_basics_py basics.launch.py
ros2 launch pg_basics_py basics.launch.py --show-args     # 查看可传入的参数
ros2 launch pg_sim tb3_sim.launch.py world:=house gui:=false
ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py   # 02 课：避障漫游
ros2 launch pg_go_to_goal_py navigate_sim.launch.py        # 03 课：Bug2 导航（mode:=p 切换为 P 控制）
ros2 launch pg_slam_py slam_sim.launch.py                  # 04 课：SLAM 建图（explore:=auto 自动漫游）
```

## 地图与生命周期节点

```bash
ros2 run turtlebot3_teleop teleop_keyboard                 # TurtleBot3 键盘遥控（需要 TURTLEBOT3_MODEL）
ros2 run pg_slam_py save_map ~/maps/my_map                 # 04 课：保存地图（.pgm + .yaml）
ros2 run pg_slam_py save_map --stats-only                  # 只看地图统计：空闲面积、已知比例
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map    # Nav2 自带的地图保存工具
ros2 topic info /map -v                                    # 查看 QoS（地图是 TRANSIENT_LOCAL）
ros2 lifecycle get /slam_toolbox                           # 生命周期节点的当前状态
ros2 lifecycle set /slam_toolbox deactivate                # 切换状态：configure / activate / deactivate ...
```

## bag 录制与回放

```bash
ros2 bag record -o ~/bags/demo /scan /odom      # 录制指定话题
ros2 bag info ~/bags/demo                       # 查看录包信息
ros2 bag play ~/bags/demo --rate 2.0            # 两倍速回放
ros2 bag play ~/bags/demo --delay 2.0           # 等 2 秒再开始，避免订阅者还没连上就丢掉开头的消息
```

## 坐标变换（TF）

```bash
ros2 run tf2_tools view_frames                  # 生成 TF 树的 PDF
ros2 run tf2_ros tf2_echo odom base_footprint   # 实时打印两个坐标系之间的变换
ros2 run tf2_ros tf2_echo map odom              # SLAM / 定位对里程计的修正量（04、05 课）
```

## 仿真与调试

```bash
ros2 launch pg_sim tb3_sim.launch.py                       # 启动 TurtleBot3 仿真
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true                              # 键盘遥控（Jazzy 需要 stamped）
rviz2                                                      # 三维可视化
rqt                                                        # 图形化调试工具集
glxinfo -B | grep renderer                                 # 确认使用显卡渲染（应显示 D3D12）
pkill -f "gz sim"                                          # 仿真没退干净时强制结束
ros2 topic echo --once --field clock.sec /clock            # 当前仿真时间（秒）
ros2 run pg_obstacle_avoidance_py safety_monitor \
  --ros-args -p use_sim_time:=true -p duration:=300.0      # 02 课：独立统计净空和接触次数
```
