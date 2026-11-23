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
