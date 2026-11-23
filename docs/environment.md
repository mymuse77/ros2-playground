# 环境说明

## 组成

| 层 | 内容 |
|---|---|
| Windows | Windows 11，WSL2（启用 WSLg，可以直接显示 Linux 图形窗口） |
| Linux | Ubuntu 24.04 LTS，用户 `ros`，开启 systemd |
| ROS 2 | Jazzy Jalisco（`ros-jazzy-desktop`），安装在 `/opt/ros/jazzy` |
| 仿真 | Gazebo Harmonic、`ros_gz`、`ros2_control`、`gz_ros2_control` |
| 应用包 | Nav2、SLAM Toolbox、Cartographer、TurtleBot3 全套、`teleop_twist_keyboard` |
| 开发工具 | `colcon`、`rosdep`、`vcstool`、`build-essential`、`git` |

一键安装方法见 [setup/README.md](../setup/README.md)。

## 显卡加速

WSLg 通过 Mesa 的 D3D12 驱动把 OpenGL 渲染转给 Windows 显卡（AMD、Intel、NVIDIA 都支持），Gazebo 和 RViz 因此能用显卡渲染。验证方法：

```bash
glxinfo -B | grep "OpenGL renderer"
# 期望：OpenGL renderer string: D3D12 (你的显卡型号)
# 如果是 llvmpipe，说明在用 CPU 渲染，见 troubleshooting.md
```

## 环境变量

安装脚本会生成 `~/.ros-wsl-env`，并在 `~/.bashrc` 里引入：

| 变量 | 值 | 作用 |
|---|---|---|
| — | `source /opt/ros/jazzy/setup.bash` | 加载 ROS 2 |
| — | `source ~/ros2_ws/install/setup.bash` | 加载本工作空间（编译过才有） |
| `ROS_DOMAIN_ID` | `0` | DDS 域号，同一局域网里多人使用时可以改成不同值互不干扰 |
| `TURTLEBOT3_MODEL` | `waffle` | TurtleBot3 型号（waffle 带相机） |
| `GALLIUM_DRIVER` 等 | `d3d12` | 让渲染走显卡 |

## 目录约定

| 位置 | 用途 |
|---|---|
| `~/ros2_ws/src/ros2_playground` | 本仓库 |
| `~/ros2_ws/build`、`install`、`log` | 编译产物（不提交） |
| `~/bags` | ros2 bag 录包数据（不提交） |
| `~/maps` | 临时保存的地图；确定使用的地图复制到 `common/pg_sim/maps` 后提交 |

## WSL 资源

`%USERPROFILE%\.wslconfig` 里可以限制 WSL 使用的内存和 CPU，例如：

```ini
[wsl2]
memory=16GB
processors=12
swap=8GB
```

修改后执行 `wsl --shutdown` 生效。

## 进入环境

```powershell
wsl -d Ubuntu-24.04        # 或者设为默认后直接 wsl
```

推荐用 VS Code + WSL 插件开发：在 WSL 里进入仓库目录执行 `code .`，编辑器运行在 Windows 上，文件、终端、git 都在 WSL 里。
