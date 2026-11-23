"""
05 课 · 一键启动：仿真 + Nav2（AMCL 定位 + 规划 + 控制）+ RViz，使用 04 课建好的地图。

启动后小车已经在地图原点定位好（参数文件里 amcl.set_initial_pose: true），
在 RViz 里用"2D Goal Pose"点一个目标，小车就会规划路径并开过去。
加上 patrol:=true 则同时启动巡逻节点，按 config/waypoints.yaml 循环巡逻。

Nav2 本身由 nav2_bringup 的 bringup_launch.py 启动，它再分成两部分：
  localization_launch.py   map_server + amcl                     （地图和定位）
  navigation_launch.py     planner / controller / bt_navigator … （规划和控制）
每部分各有一个 lifecycle_manager，按顺序把节点 configure → activate。

用法：
  ros2 launch pg_nav2_py nav2_sim.launch.py
  ros2 launch pg_nav2_py nav2_sim.launch.py patrol:=true
  ros2 launch pg_nav2_py nav2_sim.launch.py patrol:=true loops:=1 controller:=RPP
  ros2 launch pg_nav2_py nav2_sim.launch.py --show-args
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """生成启动描述：仿真 + Nav2 + RViz +（可选）巡逻节点。"""
    pkg_share = get_package_share_directory('pg_nav2_py')
    sim_launch = os.path.join(
        get_package_share_directory('pg_sim'), 'launch', 'tb3_sim.launch.py')
    bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    default_map = os.path.join(
        get_package_share_directory('pg_sim'), 'maps', 'tb3_world.yaml')

    declare_args = [
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='地图文件（.yaml）；默认是 04 课建好的 turtlebot3_world 地图'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2.yaml'),
            description='Nav2 参数文件'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='是否打开 Gazebo 窗口'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='是否打开 RViz'),
        DeclareLaunchArgument(
            'patrol', default_value='false', description='是否同时启动巡逻节点'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=os.path.join(pkg_share, 'config', 'waypoints.yaml'),
            description='巡逻航点文件'),
        DeclareLaunchArgument(
            'loops', default_value='0', description='巡逻圈数，0 表示一直巡逻'),
        DeclareLaunchArgument(
            'planner', default_value='',
            description='规划器：GridBased | Smac2D | ThetaStar，空表示默认（GridBased）'),
        DeclareLaunchArgument(
            'controller', default_value='',
            description='控制器：FollowPath（DWB）| RPP | MPPI，空表示默认（FollowPath）'),
    ]

    # 仿真：固定使用 turtlebot3_world 场景（地图是在这个场景里建的），出生点用默认值，
    # 正好是建图时的起点，也就是地图坐标系的原点
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': 'world',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg_share, 'rviz', 'nav2.rviz'),
            'fixed_frame': 'map',
        }.items())

    # Nav2：use_composition 设为 False，每个节点一个进程。
    # 默认的 True 会把所有节点装进同一个容器进程，省内存；这里为了学习时能用
    # ros2 node list / top 分别看到每个节点（例如对比控制器时看 controller_server 的 CPU 占用）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items())

    # 巡逻节点：自己会等 Nav2 就绪，所以和 Nav2 同时启动即可。
    # planner、controller 可能是空字符串，要用 ParameterValue 指明类型，
    # 否则 launch 会把空字符串当成"没有值"而报错
    patrol = Node(
        package='pg_nav2_py', executable='patrol', name='patroller', output='screen',
        parameters=[{
            'use_sim_time': True,
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
            'planner': ParameterValue(LaunchConfiguration('planner'), value_type=str),
            'controller': ParameterValue(LaunchConfiguration('controller'), value_type=str),
        }],
        condition=IfCondition(LaunchConfiguration('patrol')))

    return LaunchDescription(declare_args + [simulation, nav2, patrol])
