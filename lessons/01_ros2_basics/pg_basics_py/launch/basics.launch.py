"""
01 课 · 一条命令启动全部节点：greeter、listener、countdown_server。

知识点：
  - DeclareLaunchArgument：声明启动参数，可在命令行用 名字:=值 覆盖
  - LaunchConfiguration：在启动时读取参数的值
  - Node(parameters=[YAML 文件路径])：从参数文件加载节点参数
  - GroupAction + PushRosNamespace：给一组节点统一加命名空间，
    同一套程序可以启动多份互不干扰（比如多个机器人）

用法：
  ros2 launch pg_basics_py basics.launch.py
  ros2 launch pg_basics_py basics.launch.py namespace:=robot1
  ros2 launch pg_basics_py basics.launch.py params_file:=/path/to/my.yaml
  ros2 launch pg_basics_py basics.launch.py --show-args     # 查看全部参数
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """生成启动描述：声明参数，启动三个节点。"""
    default_params = os.path.join(
        get_package_share_directory('pg_basics_py'), 'config', 'basics.yaml')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='节点的命名空间，为空表示不加命名空间'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='greeter 节点的参数文件（YAML）'),
    ]

    # output='screen'：节点日志直接打印到终端，方便观察
    nodes = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='pg_basics_py', executable='greeter', name='greeter',
            parameters=[params_file], output='screen'),
        Node(
            package='pg_basics_py', executable='listener', name='listener',
            output='screen'),
        Node(
            package='pg_basics_py', executable='countdown_server', name='countdown_server',
            output='screen'),
    ])

    return LaunchDescription(declare_args + [nodes])
