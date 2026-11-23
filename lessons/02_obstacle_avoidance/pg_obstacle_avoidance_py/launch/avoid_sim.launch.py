"""
02 课 · 一键启动：仿真 + 避障节点 + RViz。

复用公共包 pg_sim 的仿真 launch（IncludeLaunchDescription），只额外启动避障节点，
并把 RViz 配置换成本课的版本（多显示一个扇区标记）。

用法：
  ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py
  ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py world:=house
  ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py gui:=false rviz:=false   # 无窗口
  ros2 launch pg_obstacle_avoidance_py avoid_sim.launch.py --show-args
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述：仿真（含可选的 Gazebo 窗口和 RViz）+ 避障节点。"""
    pkg_share = get_package_share_directory('pg_obstacle_avoidance_py')
    sim_launch = os.path.join(
        get_package_share_directory('pg_sim'), 'launch', 'tb3_sim.launch.py')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value='world',
            description='仿真场景：world | house | empty，或 .world 文件的完整路径'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='是否打开 Gazebo 窗口'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='是否打开 RViz'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
            description='避障节点的参数文件'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg_share, 'rviz', 'avoid.rviz'),
        }.items())

    # use_sim_time：使用仿真时钟（/clock），否则节点的时间与仿真对不上
    avoider = Node(
        package='pg_obstacle_avoidance_py', executable='avoider', name='avoider',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        output='screen')

    return LaunchDescription(declare_args + [simulation, avoider])
