"""
06 课 · 相机场景：只启动仿真（带彩色目标的练习场景），用来检查相机画面。

复用公共包 pg_sim 的仿真 launch（IncludeLaunchDescription），只替换默认场景和机器人型号：
  - 场景用本课的 color_course.sdf（红球 + 三个干扰物体）
  - 机器人用 waffle：burger 型号没有相机

用法：
  ros2 launch pg_vision_py camera_sim.launch.py
  ros2 launch pg_vision_py camera_sim.launch.py gui:=false        # 无窗口
  ros2 launch pg_vision_py camera_sim.launch.py --show-args

看相机画面（另开终端）：
  ros2 run rqt_image_view rqt_image_view /camera/image_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述：本课场景下的 TurtleBot3 waffle 仿真。"""
    pkg_share = get_package_share_directory('pg_vision_py')
    sim_launch = os.path.join(
        get_package_share_directory('pg_sim'), 'launch', 'tb3_sim.launch.py')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds', 'color_course.sdf'),
            description='仿真场景：默认本课的彩色目标场景；也可用 world | house | empty'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='是否打开 Gazebo 窗口'),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='是否打开 RViz'),
        # 本课场景以原点为中心布置，小车出生在原点、车头朝向红球
        DeclareLaunchArgument('x_pose', default_value='0.0', description='出生点 x（米）'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='出生点 y（米）'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': 'waffle',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items())

    return LaunchDescription(declare_args + [simulation])
