"""
03 课 · 一键启动：仿真（默认 Bug2 练习场景）+ 导航节点 + RViz。

两种控制模式（二选一，两个节点都会发 /cmd_vel，不能同时运行）：
  mode:=bug2  启动动作服务端 navigate_server，用 navigate_client 或 ros2 action 发目标（默认）
  mode:=p     启动到点控制节点 go_to_goal，在 RViz 里用 "2D Goal Pose" 点目标（不避障）

用法：
  ros2 launch pg_go_to_goal_py navigate_sim.launch.py
  ros2 launch pg_go_to_goal_py navigate_sim.launch.py mode:=p
  ros2 launch pg_go_to_goal_py navigate_sim.launch.py world:=world x_pose:=-2.0 y_pose:=-0.5
  ros2 launch pg_go_to_goal_py navigate_sim.launch.py gui:=false rviz:=false   # 无窗口
然后在另一个终端：
  ros2 run pg_go_to_goal_py navigate_client 3.0 0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述：仿真 + 按 mode 启动 navigate_server 或 go_to_goal。"""
    pkg_share = get_package_share_directory('pg_go_to_goal_py')
    sim_launch = os.path.join(
        get_package_share_directory('pg_sim'), 'launch', 'tb3_sim.launch.py')

    declare_args = [
        DeclareLaunchArgument(
            'mode', default_value='bug2', choices=['bug2', 'p'],
            description='bug2：Bug2 动作服务端；p：到点 P 控制（不避障）'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds', 'bug2_course.sdf'),
            description='仿真场景：默认本课的 Bug2 练习场景；也可用 world | house | empty'),
        # 本课场景以原点为中心布置，小车出生在原点；换 world 场景时建议 x_pose:=-2.0
        DeclareLaunchArgument('x_pose', default_value='0.0', description='出生点 x，m'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='出生点 y，m'),
        DeclareLaunchArgument('gui', default_value='true', description='是否打开 Gazebo 窗口'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否打开 RViz'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'navigate.yaml'),
            description='导航节点的参数文件'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': '0.0',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg_share, 'rviz', 'navigate.rviz'),
        }.items())

    def mode_is(name):
        """条件：mode 参数等于 name。"""
        return IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), name))

    common = [LaunchConfiguration('params_file'), {'use_sim_time': True}]
    navigate_server = Node(
        package='pg_go_to_goal_py', executable='navigate_server', name='navigate_server',
        parameters=common, output='screen', condition=mode_is('bug2'))
    go_to_goal = Node(
        package='pg_go_to_goal_py', executable='go_to_goal', name='go_to_goal',
        parameters=common, output='screen', condition=mode_is('p'))

    return LaunchDescription(declare_args + [simulation, navigate_server, go_to_goal])
