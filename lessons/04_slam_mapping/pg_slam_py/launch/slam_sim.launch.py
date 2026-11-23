"""
04 课 · 一键启动：仿真 + slam_toolbox 在线建图 + RViz，两种探索方式。

  explore:=teleop（默认）  只启动仿真和 SLAM，在另一个终端用键盘开车：
                            ros2 run turtlebot3_teleop teleop_keyboard
  explore:=auto            同时启动 02 课的避障漫游节点，小车自己到处走

slam_toolbox 是一个生命周期节点（lifecycle node），需要依次"配置 → 激活"才开始工作。
这里直接引入它自带的 online_async_launch.py，由那个 launch 自动完成这两步。

用法：
  ros2 launch pg_slam_py slam_sim.launch.py
  ros2 launch pg_slam_py slam_sim.launch.py explore:=auto
  ros2 launch pg_slam_py slam_sim.launch.py world:=house explore:=auto
  ros2 launch pg_slam_py slam_sim.launch.py --show-args
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述：仿真 + SLAM +（可选）自动漫游。"""
    pkg_share = get_package_share_directory('pg_slam_py')
    sim_launch = os.path.join(
        get_package_share_directory('pg_sim'), 'launch', 'tb3_sim.launch.py')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    avoid_params = os.path.join(
        get_package_share_directory('pg_obstacle_avoidance_py'), 'config', 'params.yaml')

    explore = LaunchConfiguration('explore')
    is_auto = IfCondition(EqualsSubstitution(explore, 'auto'))
    is_teleop = IfCondition(EqualsSubstitution(explore, 'teleop'))

    declare_args = [
        DeclareLaunchArgument(
            'explore', default_value='teleop', choices=['teleop', 'auto'],
            description='探索方式：teleop 键盘遥控（另开终端）/ auto 02 课的避障漫游'),
        DeclareLaunchArgument(
            'world', default_value='world',
            description='仿真场景：world | house | empty，或场景文件的完整路径'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='是否打开 Gazebo 窗口'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='是否打开 RViz'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=os.path.join(pkg_share, 'config', 'slam.yaml'),
            description='slam_toolbox 的参数文件'),
    ]

    # 仿真：RViz 的固定坐标系用 map（SLAM 启动后才有），看到的是"地图上的小车"
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg_share, 'rviz', 'slam.rviz'),
            'fixed_frame': 'map',
        }.items())

    # SLAM：订阅 /scan 和 odom → base_footprint 的 TF，发布 /map 和 map → odom 的 TF
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': 'true',
        }.items())

    # 自动探索：02 课的避障节点，参数沿用 02 课的配置
    avoider = Node(
        package='pg_obstacle_avoidance_py', executable='avoider', name='avoider',
        parameters=[avoid_params, {'use_sim_time': True}],
        output='screen', condition=is_auto)

    teleop_hint = LogInfo(
        msg='键盘遥控：另开一个终端运行 `ros2 run turtlebot3_teleop teleop_keyboard`',
        condition=is_teleop)

    return LaunchDescription(declare_args + [simulation, slam, avoider, teleop_hint])
