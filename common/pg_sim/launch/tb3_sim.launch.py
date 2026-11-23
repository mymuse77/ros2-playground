"""
启动 TurtleBot3 Gazebo 仿真（各课程共用）。

与 turtlebot3_gazebo 官方 launch 的区别：
  - 支持 gui:=false 无窗口运行（官方 launch 没有这个参数，总会打开 Gazebo 窗口），
    方便自动化测试，也能节省显卡资源；
  - 用一个 world 参数切换场景，不用记不同的 launch 文件名；
  - 可选同时打开 RViz（rviz:=true），并指定 RViz 的固定坐标系。

用法示例：
  ros2 launch pg_sim tb3_sim.launch.py
  ros2 launch pg_sim tb3_sim.launch.py world:=house x_pose:=-3.0 y_pose:=1.0
  ros2 launch pg_sim tb3_sim.launch.py gui:=false rviz:=true
  ros2 launch pg_sim tb3_sim.launch.py --show-args      # 查看全部参数

启动后可用的主要话题：
  /scan（激光雷达）、/odom（里程计）、/imu、/cmd_vel（速度指令，TwistStamped）、
  /camera/image_raw、/camera/camera_info、/tf、/clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 场景简称 -> turtlebot3_gazebo/worlds 下的文件名
WORLDS = {
    'world': 'turtlebot3_world.world',   # 3x3 排列的柱子，外围六边形墙
    'house': 'turtlebot3_house.world',   # 有多个房间的室内场景
    'empty': 'empty_world.world',        # 空场景，只有地面
}


def _launch_setup(context):
    """
    根据参数的实际取值生成要启动的动作。

    launch 参数在 generate_launch_description() 执行时还没有值，
    需要用到具体字符串（例如拼接文件路径）的部分放在 OpaqueFunction 里，
    等 launch 系统解析完参数后再执行。
    """
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_arg = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    # world 既可以是简称（world/house/empty），也可以是 .world/.sdf 文件的完整路径
    if world_arg in WORLDS:
        world_path = os.path.join(tb3_share, 'worlds', WORLDS[world_arg])
    elif os.path.isfile(world_arg):
        world_path = world_arg
    else:
        raise RuntimeError(
            f'未知场景 "{world_arg}"：可选 {", ".join(WORLDS)}，或者给出场景文件的完整路径')

    def gz_sim(gz_args):
        """引入 ros_gz_sim 的 gz_sim.launch.py（每次新建 launch 源，避免两次引入共用同一组动作）。"""
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    actions = []

    # 1. Gazebo 服务端：负责物理仿真和传感器。-r 立即开始仿真，-s 只启动服务端，-v2 日志级别
    actions.append(gz_sim(f'-r -s -v2 {world_path}'))

    # 2. Gazebo 图形界面（客户端）：只负责显示，关掉它不影响仿真
    if gui:
        actions.append(gz_sim('-g -v2'))

    # 3. robot_state_publisher：读取 URDF，发布 /robot_description 和机器人各部件之间的固定 TF
    urdf_path = os.path.join(tb3_share, 'urdf', f'turtlebot3_{model}.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]))

    # 4. 把机器人模型放进 Gazebo 场景
    actions.append(Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', model,
            '-file', os.path.join(tb3_share, 'models', f'turtlebot3_{model}', 'model.sdf'),
            '-x', LaunchConfiguration('x_pose').perform(context),
            '-y', LaunchConfiguration('y_pose').perform(context),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw').perform(context),
        ]))

    # 5. ros_gz_bridge：在 Gazebo 话题和 ROS 话题之间转换消息（/scan、/odom、/cmd_vel 等）。
    #    转换规则直接复用 turtlebot3_gazebo 提供的配置文件
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(tb3_share, 'params', f'turtlebot3_{model}_bridge.yaml'),
        }]))

    # 6. 相机图像桥接（burger 型号没有相机）
    if model != 'burger':
        actions.append(Node(
            package='ros_gz_image',
            executable='image_bridge',
            output='screen',
            arguments=['/camera/image_raw']))

    return actions


def generate_launch_description():
    """声明 launch 参数，并组装仿真启动流程。"""
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    pg_sim_share = get_package_share_directory('pg_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='world',
            description='场景：world（柱子场景）/ house（房屋）/ empty（空场景），或场景文件完整路径'),
        DeclareLaunchArgument(
            'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'waffle'),
            description='TurtleBot3 型号：burger / waffle / waffle_pi（默认取环境变量 TURTLEBOT3_MODEL）'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否打开 Gazebo 窗口；false 时只运行仿真，不显示画面'),
        DeclareLaunchArgument('x_pose', default_value='-2.0', description='出生点 x（米）'),
        DeclareLaunchArgument('y_pose', default_value='-0.5', description='出生点 y（米）'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='出生朝向（弧度，0 为 x 轴正方向）'),
        DeclareLaunchArgument('rviz', default_value='false', description='是否同时打开 RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pg_sim_share, 'rviz', 'tb3.rviz'),
            description='RViz 配置文件'),
        DeclareLaunchArgument(
            'fixed_frame', default_value='odom',
            description='RViz 固定坐标系：没有地图时用 odom，建图和导航时用 map'),

        # Gazebo 需要知道去哪里找 TurtleBot3 的模型文件
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_share, 'models')),

        OpaqueFunction(function=_launch_setup),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config'),
                       '-f', LaunchConfiguration('fixed_frame')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
