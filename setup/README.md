# ROS 2 Jazzy + Gazebo Harmonic 一键安装（Windows / WSL2）

在一台新的 Windows 电脑上，用一条命令装好：

- WSL2 + Ubuntu 24.04（官方镜像，SHA256 校验）
- ROS 2 Jazzy desktop、Gazebo Harmonic、`ros_gz`、`ros2_control`
- Nav2、SLAM Toolbox、TurtleBot3（可用 `-SkipRobots` 跳过）
- 显卡加速配置（AMD / Intel / NVIDIA 都通过 WSLg + Mesa D3D12 渲染）
- 国内镜像自动测速选择，ROS 签名密钥指纹校验

## 1. 前置条件

| 项目 | 要求 |
|---|---|
| 系统 | Windows 11（推荐），或 Windows 10 21H2+（build 19044+，否则没有 Linux 图形窗口） |
| 虚拟化 | BIOS 里已开启（Intel VT-x / AMD SVM），任务管理器 → 性能 → CPU 显示"虚拟化：已启用" |
| WSL | 已启用 WSL2。没有的话，用**管理员** PowerShell 运行 `wsl --install --no-distribution`，重启电脑 |
| 硬盘 | 目标盘至少 **40 GB** 空闲（实际占用约 20–25 GB） |
| 内存 | 建议 16 GB 以上（32 GB 更好） |
| 显卡驱动 | 更新到最新版（AMD Adrenalin / Intel Graphics / NVIDIA），否则会退回 CPU 渲染 |
| 网络 | 能访问 `releases.ubuntu.com`、`raw.githubusercontent.com` 和国内镜像。有系统代理时 WSL 会自动继承 |

## 2. 一键安装

1. 在新电脑上从 GitHub 下载本仓库（页面上 **Code → Download ZIP**，或在 Windows 上 `git clone`），解压到一个不含空格和中文的路径，例如 `D:\ros2-playground`
2. 打开 PowerShell（普通权限即可，不需要管理员），进入 `setup` 目录运行：

```powershell
cd D:\ros2-playground\setup
powershell -ExecutionPolicy Bypass -File .\install.ps1
```

安装完成后，在 WSL 里把仓库克隆到工作空间（Windows 上下载的那份只用于安装，可以删掉）：

```bash
cd ~/ros2_ws/src && git clone https://github.com/mymuse77/ros2-playground.git
```

默认会：安装到空闲空间最大的盘的 `X:\WSL\Ubuntu-24.04`，用户名 `ros`，自动选镜像。
全程约 **20–60 分钟**，主要取决于下载速度。过程输出同时记录在 `X:\WSL\logs\` 下。

### 常用示例

```powershell
# 指定安装位置，只装核心（不装 Nav2/SLAM/TurtleBot3），并设为默认发行版
.\install.ps1 -InstallDir E:\WSL\ros -SkipRobots -SetDefault

# 指定镜像（阿里云），装完自动导出备份
.\install.ps1 -Mirror aliyun -Backup

# 离线/已下载好镜像：不联网下载 rootfs
.\install.ps1 -RootfsPath D:\downloads\ubuntu-24.04.5-wsl-amd64.wsl
```

### 参数说明

| 参数 | 默认值 | 说明 |
|---|---|---|
| `-DistroName` | `Ubuntu-24.04` | WSL 里显示的发行版名称 |
| `-InstallDir` | 空闲最大的盘 `X:\WSL\<DistroName>` | 虚拟磁盘 `ext4.vhdx` 所在目录，必须为空 |
| `-UserName` | `ros` | Linux 用户名（sudo 免密） |
| `-Mirror` | `auto` | `auto` 自动测速；也可选 `huawei` `aliyun` `tuna` `ustc` `official` |
| `-RootfsPath` | 空（自动下载） | 使用本地 `.wsl` / `.tar.gz` 镜像 |
| `-MemoryGB` / `-Processors` / `-SwapGB` | 自动 / 自动 / 8 | 写入 `.wslconfig`。自动 = 内存一半（4–16 GB）、CPU 的 3/4 |
| `-SkipRobots` | 关 | 不装 Nav2 / SLAM Toolbox / TurtleBot3 |
| `-SkipWslConfig` | 关 | 不修改 `.wslconfig` |
| `-SkipVerify` | 关 | 装完不跑验证 |
| `-SetDefault` | 关 | 设为 `wsl` 的默认发行版 |
| `-Backup` / `-BackupDir` | 关 | 装完导出 `.tar` 备份（约 7 GB） |
| `-RosKeyFile` | 空 | 本地 `ros.key`，用于无法访问 GitHub 的网络 |
| `-SimulatePackages` | 关 | 只模拟装包，用来测试整个流程（不下载大包） |

## 3. 安装结束后

```powershell
wsl --shutdown                 # 让 .wslconfig 的内存/CPU 限制生效（会关闭所有 WSL 发行版）
wsl -d Ubuntu-24.04            # 进入环境
```

在 Linux 里：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py     # Gazebo 窗口 + TurtleBot3
bash /opt/ros-installer/verify.sh --tb3                      # 完整自检（无头仿真，约 1 分钟）
```

验证项：ROS 2 通信、Gazebo 版本、`/dev/dxg`、OpenGL 渲染器（应显示 `D3D12 (你的显卡)`）、TurtleBot3 雷达与运动。

## 4. 安装内容与位置

| 内容 | 位置 |
|---|---|
| 虚拟磁盘 | `<InstallDir>\ext4.vhdx` |
| ROS 2 | `/opt/ros/jazzy` |
| 工作空间 | `~/ros2_ws`（代码放 Linux 文件系统里，不要放 `/mnt/d`，编译会很慢） |
| 环境变量 | `~/.ros-wsl-env`（由 `~/.bashrc` 引入） |
| 安装脚本副本 | `/opt/ros-installer/`（可重复运行） |
| Windows 里访问 Linux 文件 | `\\wsl$\Ubuntu-24.04\home\ros\` |

## 5. 常见问题

**脚本提示 WSL is not ready**
用管理员 PowerShell 运行 `wsl --install --no-distribution`，重启，再运行脚本。若仍失败，检查 BIOS 虚拟化。

**OpenGL 渲染器显示 llvmpipe（CPU 渲染）**
更新 Windows 显卡驱动，执行 `wsl --shutdown`，重新进入。`verify.sh` 会提示。

**下载很慢 / 超时**
默认已自动测速。可手动指定 `-Mirror huawei`（或 aliyun / tuna / ustc）。装包在中途失败可以直接重跑 `setup.sh`（幂等，已装好的不会重装）：
```powershell
wsl -d Ubuntu-24.04 -u root --exec bash /opt/ros-installer/setup.sh --user ros
```

**`packages.ros.org` 证书错误**
部分网络/代理会劫持该域名的 HTTPS。脚本不会跳过证书校验，而是改用国内镜像；ROS 密钥来源不同但会校验指纹（Open Robotics 官方密钥，指纹见 `linux/setup.sh` 顶部的 `ROS_KEY_FPR`），指纹不符会直接中止。

**密钥下载失败**
手动下载 `https://raw.githubusercontent.com/ros/rosdistro/master/ros.key` 保存到本地，加参数 `-RosKeyFile C:\path\ros.key`。

**提示发行版已存在 / 目录非空**
脚本不会覆盖已有系统。换 `-DistroName` 和 `-InstallDir`，或自行确认数据不需要后再 `wsl --unregister <名称>`（**会删除该发行版全部数据**）。

**Gazebo 窗口不显示**
需要 Windows 11 或 Windows 10 21H2+ 的 WSLg；`wsl --update`（管理员）后再试。

**ssh / 防火墙环境下的代理**
`.wslconfig` 默认开启 `autoProxy=true`（Win11 22H2+ 同时启用 mirrored 网络），WSL 会继承 Windows 系统代理。

## 6. 备份、迁移、卸载

```powershell
# 备份
wsl --export Ubuntu-24.04 D:\backup\ubuntu2404-ros.tar
# 恢复 / 迁移到另一台机器
wsl --import Ubuntu-24.04 D:\WSL\Ubuntu-24.04 D:\backup\ubuntu2404-ros.tar --version 2
wsl -d Ubuntu-24.04 -u root --exec bash -c "printf '[user]\ndefault=ros\n' >> /etc/wsl.conf"   # 恢复后默认用户可能变成 root
# 卸载（删除全部数据，先备份！）
wsl --unregister Ubuntu-24.04
```

## 7. 目录结构

```
setup/
├─ install.ps1        Windows 入口：检查环境、下载并校验镜像、导入 WSL、写 .wslconfig、调用 setup.sh、验证
├─ README.md          本文档
└─ linux/
   ├─ setup.sh        Linux 端：建用户、配 wsl.conf、选镜像、ROS 密钥(指纹校验)、装 ROS/Gazebo、写环境变量
   └─ verify.sh       自检脚本（--tb3 追加无头仿真测试）
```

## 8. 测试说明

在一台 Windows 11、AMD Ryzen 7 8845HS（Radeon 780M 核显）的机器上，同一套步骤已手动完整装过并验证（TurtleBot3 雷达 10 Hz、运动控制正常、GPU 渲染为 D3D12）。
这套一键脚本整理自那次手动安装，已用临时发行版做过流程测试（`-SimulatePackages`）；**尚未在一台全新的、没装过 WSL 的机器上完整实跑过**。第一次在新机器上运行时如遇问题，日志在 `X:\WSL\logs\` 下。
