# 踩坑记录

学习过程中遇到的问题和解决办法，随学随记。

## 环境

**`packages.ros.org` 报 HTTPS 证书错误**
部分网络或代理会劫持这个域名。不要关闭证书校验，改用国内镜像（华为云、阿里云、清华、中科大都同步了 ROS 2 仓库）。一键安装脚本会自动测速选择。

**`glxinfo -B` 显示 `llvmpipe`（CPU 渲染）**
WSLg 默认可能没有选中 D3D12 驱动。在 `~/.bashrc` 中设置：
```bash
export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
```
仍不行就更新 Windows 显卡驱动，然后 `wsl --shutdown` 重启 WSL。

**通过 `wsl -- bash -c "..."` 执行命令时找不到 `ros2`**
非交互式 shell 会在 `~/.bashrc` 开头直接返回，后面的 `source /opt/ros/jazzy/setup.bash` 不会执行。脚本里要自己 `source`，或使用 `bash -i`。

**代码放在 `/mnt/c`、`/mnt/d` 下编译很慢**
WSL 访问 Windows 磁盘要经过 9P 协议，大量小文件读写很慢。代码放在 WSL 的 Linux 文件系统里（如 `~/ros2_ws`），Windows 侧用 `\\wsl$\Ubuntu-24.04\home\...` 访问。

## ROS 2 / TurtleBot3

**发布 `/cmd_vel` 后小车不动**
Jazzy 版 TurtleBot3 的 `/cmd_vel` 类型是 `geometry_msgs/msg/TwistStamped`，网上很多教程用的是 `Twist`。类型不匹配时不会报错，只是收不到。用 `ros2 topic info /cmd_vel -v` 检查类型。

**键盘遥控没反应**
`teleop_twist_keyboard` 默认发 `Twist`，需要加 `--ros-args -p stamped:=true`。

**`turtlebot3_world.launch.py` 的 `gui:=false` 不起作用**
官方 launch 文件没有声明 `gui` 参数，传进去会被静默忽略，总会打开 Gazebo 窗口。需要无窗口运行时使用本仓库的 `ros2 launch pg_sim tb3_sim.launch.py gui:=false`。

**订阅 `/scan` 收不到数据**
传感器话题通常使用"尽力而为"（best effort）的 QoS。订阅时使用 `rclpy.qos.qos_profile_sensor_data`，它既能收 best effort 也能收 reliable 的发布者。

**仿真时间**
使用 Gazebo 时，所有节点都应设置 `use_sim_time:=true`，否则节点用的是系统时间，和仿真时间对不上，TF 会报时间戳相关的错误。

**Gazebo 关闭后还有残留进程，再次启动报错**
执行 `pkill -f "gz sim"` 清理后再启动。

**WSL2 下仿真比现实慢，`ros2 topic hz /scan` 只有 3 Hz 左右**
没有独立显卡时 Gazebo 的传感器渲染很吃 CPU，仿真常常只有 0.3 倍速左右（`/clock` 走 1 秒，现实过了 3 秒）。
节点都用仿真时间，行为不受影响；`topic hz` 按现实时间统计，所以显示偏低。无窗口模式（`gui:=false`）能快一些。

## 节点编写

**按 Ctrl+C 后小车没有停，还在按最后的速度跑**（02 课）
两个原因叠加：
1. rclpy 默认的信号处理会**先关闭 ROS**，之后再 `publish()` 已经发不出去。
   解决：`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`，自己处理 Ctrl+C，退出前先发零速度
2. `rclpy.spin()` 阻塞在 C 代码里，没有消息到达时 Python 的信号处理函数没机会运行，Ctrl+C 要等很久才生效。
   解决：改用 `while rclpy.ok(): rclpy.spin_once(node, timeout_sec=0.1)` 循环

完整写法见 `lessons/02_obstacle_avoidance/.../avoider_node.py` 的 `main()`，03 课把它整理成了
`pg_go_to_goal_py/ros_utils.py` 里的 `init_with_safe_shutdown` 和 `spin_until_shutdown`。

**脚本里用 `&` 放到后台的节点，`kill -INT` 杀不掉**
非交互式 shell 启动的后台进程会继承"忽略 SIGINT"的设置。节点里显式安装处理函数即可：
`signal.signal(signal.SIGINT, signal.default_int_handler)`；或者用 `kill -TERM`。
不要在脚本里用 `set -m` 来绕过：WSL 的非交互式 shell 中它会让后台进程被挂起。

**看门狗在仿真卡住时不起作用**（02 课）
定时器默认使用节点时钟，`use_sim_time:=true` 时就是仿真时钟。仿真卡住时仿真时钟也停了，定时器不再触发，
永远发现不了"数据超时"。看门狗这类安全相关的计时要用稳定时钟：
`create_timer(..., clock=Clock(clock_type=ClockType.STEADY_TIME))`。

**对 `ros2 launch` 按 Ctrl+C 后，节点退出时打出一大段 `KeyboardInterrupt`**（05 课）
节点收到了两次 SIGINT：终端把 Ctrl+C 发给整个进程组一次，launch 再转发一次。第一次让节点进入清理，
第二次打断了 `destroy_node()`。清理开始时忽略 SIGINT：`signal.signal(signal.SIGINT, signal.SIG_IGN)`，
见 `safety_monitor_node.py` 和 `patrol_node.py` 的 `main()`。

**动作服务端退出时卡住**（03 课）
多线程执行器关闭时会等待正在运行的执行回调结束。如果执行回调是一个控制循环，它必须能感知到"节点要关了"。
03 课的做法：退出前设置一个 `threading.Event`，循环每个周期检查它。

**动作客户端一直等不到结果，服务端日志有 `failed to send response (timeout)`**（03 课）
服务的回应在 DDS 发现还没完成时可能送不到，CPU 很忙（比如跑着仿真）时更容易出现。
发送目标、请求结果都是服务调用，都可能遇到。客户端要给等待设超时并重试，见 `navigate_client_node.py`。

**`ros2 bag play` 回放时开头几条消息丢了**（01 课）
回放一开始就发布，订阅者可能还没完成发现（discovery）。加 `--delay 2.0` 等 2 秒再开始。

**测试一个节点"雷达断线"时，直接杀掉 `parameter_bridge` 不合适**（02 课）
桥接节点同时转发 `/clock` 和 `/cmd_vel`，杀掉后仿真时钟和速度指令也断了，测试结果没有意义。
更干净的做法：写一个转发节点把 `/scan` 转到 `/scan_gated`，被测节点用 `-r scan:=/scan_gated` 订阅它，需要断线时停掉转发节点。

## 算法

**沿墙走时绕过外拐角后原地打小圈**（03 课）
只看"正右方扇区的最近距离"时，墙角转到右后方后就不在扇区里了，算法以为墙丢了，一直最大角速度右转。
解决：在整个右半边找最近点，同时控制距离和方向（让最近点保持在正右方），见 `bug2.py` 的 `wall_follow_command`。
教训：单元测试里的墙太短没暴露这个问题，后来补了一个和仿真场景一模一样的闭环测试。

**目标在障碍物里面，却报告"已到达"**（03 课）
判断"被挡住"时只比较了"前方距离 < 目标距离"。目标在方块中心时，里程计误差加上雷达装在车中心后方，
这个条件不成立，车顶着方块打滑，里程计继续累加，最后"到达"了一个车根本去不了的位置。
解决：车中心到达目标时车头伸到"目标距离 + 车体半径"处，改为 `前方距离 < 目标距离 + robot_radius`。
排查方法：对比 `/odom` 和 Gazebo 的真实位姿 `gz model -m waffle -p`，两者差得多就说明打滑或撞上了。

## 地图（04 课）

**地图图片上下颠倒**
`OccupancyGrid` 的第 0 行在地图最下面（y 最小），图片的第 0 行在最上面，写图片时要上下翻转，见 `map_io.py` 的 `to_pgm`。

**同一张地图保存两次，结果不一样**
slam_toolbox 每 `map_update_interval` 秒重新生成一次地图，车还在动时两次保存之间地图已经变了。先停车，等几秒再保存。

**保存工具收不到地图**
订阅 `/map` 要用 Transient Local（与发布者一致），否则只能收到连上之后的新地图；slam_toolbox 没激活时也没有地图，用 `ros2 lifecycle get /slam_toolbox` 确认是 `active`。

## Nav2（05 课）

**map_server 配置失败：`Failed to load map yaml file: map.yaml`，定位整组都没激活**
参数文件里写了 `map_server.ros__parameters.yaml_filename`。按具体节点名设置的参数优先级高于 launch 通过
`map:=` 传入的路径，所以总是去加载那个（不存在的）文件。从参数文件里删掉这一项，地图只用 launch 参数指定。
排查：`ros2 lifecycle get /map_server` 是 `unconfigured`，日志里找 `map_server` 的第一条 ERROR。

**`bad file: .../install/pg_sim/share/pg_sim/maps/tb3_world.yaml`**
地图是后来放进 `pg_sim/maps` 的，安装目录里还没有。`colcon build --packages-select pg_sim` 重新编译一次。

**`ros2 launch ... controller:=` 报 `malformed launch argument`**
命令行上不能传空值。想用默认值就不传这个参数。launch 文件内部要把可能为空的参数传给节点时，
用 `ParameterValue(LaunchConfiguration('x'), value_type=str)` 指明类型，否则空字符串会被当成"没有值"。

**`ros2 topic pub /controller_selector ...` 发了没效果**
行为树里的选择器用 Transient Local 订阅。发布时也要 `--qos-durability transient_local`，
否则两边 QoS 不兼容，消息根本收不到（不会报错）。

**车停着时 `/amcl_pose` 不更新**
AMCL 只在车移动超过 `update_min_d`（0.25 m）或转动超过 `update_min_a`（0.2 rad）后才更新。这是正常现象，不是定位挂了。

**单独启动巡逻节点后，小车的定位"跳回"了地图原点**
`BasicNavigator.waitUntilNav2Active()` 在收到 `/amcl_pose` 之前会反复发布初始位姿（默认是原点）。
本课的巡逻节点改为只订阅、不发布，见 `patrol_node.py` 的 `wait_for_nav2()`。

**新节点看不到已有的服务，或日志出现 `open_and_lock_file failed`、`A message was lost`**
之前用 `kill -9` 强制结束的进程在 `/dev/shm` 留下了 FastDDS 共享内存文件，新进程的端点发现会出问题
（本课开发时巡逻节点一直打印 `amcl/get_state service not available`，命令行却能看到这个服务）。
关掉所有 ROS 进程后清理：
```bash
ros2 daemon stop
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

**`CRITICAL FAILURE: SERVER map_server IS DOWN after not receiving a heartbeat`，之后所有目标都失败**
`lifecycle_manager` 和每个节点之间有心跳（bond），按墙钟计时，4 s 收不到就认为节点挂了，把整组都关掉。
WSL2 上仿真把 CPU 占满、或者系统时钟跳变时，心跳会被误判为超时（本课开发时 3 次长时间巡逻里有 2 次中途出现）。
本课的 launch 用 `SetParameter('bond_timeout', 0.0)` 关掉了心跳，见 `nav2_sim.launch.py` 的注释。
注意 `lifecycle_manager` 不读 Nav2 参数文件，写在 `nav2.yaml` 里没有用。实车上应保留心跳。

## 代码风格检查（flake8）

| 报错 | 原因 | 解决 |
|---|---|---|
| A003 `next` / `set` is a python builtin | 方法名或变量名与内置函数重名 | 换个名字，如 `take()`、`with_reading()` |
| I100 import 顺序不对 | Google 风格按模块名字母排序，`from rcl_interfaces...` 要在 `import rclpy` 前面 | 按模块名排序，不看 `import` / `from` |
| D301 / W605 反斜杠 | 文档字符串里画图用了 `\` | 用制表符 `╲ ╱`，或改用原始字符串 |
| D400 / D415 摘要要以句点结尾 | 规则针对英文 | 本仓库在 `setup.cfg` 中忽略这两条 |
