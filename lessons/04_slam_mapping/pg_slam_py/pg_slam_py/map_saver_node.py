"""
04 课 · 地图保存工具：收一张 /map，存成 .pgm + .yaml，打印地图统计后退出。

用法：
  ros2 run pg_slam_py save_map ~/maps/my_map
  ros2 run pg_slam_py save_map ~/maps/my_map --free-thresh 0.25 --occupied-thresh 0.65
  ros2 run pg_slam_py save_map --stats-only                      # 只打印统计，不保存

作用与 Nav2 自带的 `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map` 相同，
写成 Python 是为了看清楚地图文件里到底是什么（格式说明见 map_io.py）。

QoS：slam_toolbox 用 Transient Local（"保留最后一条"）发布 /map，地图很大、更新不频繁，
后启动的订阅者一连上就能收到最新的一张，不用等下一次更新。订阅端也必须用 Transient Local，
否则只能收到连上之后新发布的地图。
"""

import argparse
import sys

from nav_msgs.msg import OccupancyGrid
from pg_slam_py.map_io import (check_thresholds, DEFAULT_FREE_THRESH,
                               DEFAULT_OCCUPIED_THRESH, GridMap, map_stats, save_map,
                               yaw_from_quaternion)
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

SPIN_TIMEOUT_S = 0.1   # spin_once 每次最多等待的时间，s

# 与 slam_toolbox、map_server 发布 /map 时的 QoS 匹配
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def parse_args(argv):
    """
    解析命令行参数。

    :param argv: 参数列表，不含程序名和 --ros-args 之后的 ROS 参数
    :return: argparse.Namespace
    """
    parser = argparse.ArgumentParser(description='把 /map 保存成 .pgm + .yaml')
    parser.add_argument('path', nargs='?',
                        help='不带扩展名的保存路径，例如 ~/maps/tb3_world（--stats-only 时可省略）')
    parser.add_argument('--topic', default='map', help='地图话题（默认 map）')
    parser.add_argument('--free-thresh', type=float, default=DEFAULT_FREE_THRESH,
                        help=f'占用概率 <= 此值算空闲（默认 {DEFAULT_FREE_THRESH}）')
    parser.add_argument('--occupied-thresh', type=float, default=DEFAULT_OCCUPIED_THRESH,
                        help=f'占用概率 >= 此值算障碍（默认 {DEFAULT_OCCUPIED_THRESH}）')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='等待地图的最长时间（真实时间），s（默认 10）')
    parser.add_argument('--stats-only', action='store_true', help='只打印统计，不保存文件')
    options = parser.parse_args(argv)
    if options.path is None and not options.stats_only:
        parser.error('需要保存路径（或者加 --stats-only 只看统计）')
    try:
        check_thresholds(options.free_thresh, options.occupied_thresh)
    except ValueError as error:
        parser.error(str(error))
    return options


def grid_from_message(msg):
    """
    把 nav_msgs/msg/OccupancyGrid 转换成 GridMap。

    :param msg: OccupancyGrid
    :return: GridMap
    """
    info = msg.info
    q = info.origin.orientation
    return GridMap(width=info.width, height=info.height, resolution=info.resolution,
                   origin_x=info.origin.position.x, origin_y=info.origin.position.y,
                   origin_yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
                   data=tuple(msg.data))


class MapReceiver(Node):
    """订阅地图话题，收到第一张地图后保存下来。"""

    def __init__(self, topic):
        """创建订阅。"""
        super().__init__('map_saver')
        self.grid = None
        self.create_subscription(OccupancyGrid, topic, self._on_map, MAP_QOS)

    def _on_map(self, msg):
        """地图回调：只保留第一张。"""
        if self.grid is None:
            self.grid = grid_from_message(msg)

    def wait_for_map(self, timeout):
        """
        处理回调直到收到地图或超时。

        :param timeout: 超时时间（真实时间，仿真卡住时也能超时），s
        :return: GridMap；超时返回 None
        """
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        start = steady.now()
        while self.grid is None:
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)
            if (steady.now() - start).nanoseconds * 1e-9 > timeout:
                return None
        return self.grid


def report(logger, grid, stats):
    """打印地图尺寸和统计。"""
    logger.info(
        f'地图 {grid.width} × {grid.height} 格，分辨率 {grid.resolution:.3f} m/格'
        f'（{grid.width * grid.resolution:.2f} × {grid.height * grid.resolution:.2f} m），'
        f'原点 ({grid.origin_x:.3f}, {grid.origin_y:.3f})')
    logger.info(
        f'空闲 {stats.free_cells} 格（{stats.free_area:.2f} m²），障碍 {stats.occupied_cells} 格，'
        f'未知 {stats.unknown_cells} 格；已知比例 {stats.known_ratio:.1%}')


def main(args=None):
    """节点入口：成功返回 0，等不到地图返回 1。"""
    options = parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    rclpy.init(args=args)
    node = MapReceiver(options.topic)
    exit_code = 1
    try:
        grid = node.wait_for_map(options.timeout)
        if grid is None:
            node.get_logger().error(
                f'{options.timeout:.0f} s 内没有收到 {options.topic}，SLAM 或 map_server 在运行吗？')
        else:
            stats = map_stats(grid, options.free_thresh, options.occupied_thresh)
            report(node.get_logger(), grid, stats)
            if not options.stats_only:
                pgm, yaml = save_map(options.path, grid, options.free_thresh,
                                     options.occupied_thresh)
                node.get_logger().info(f'已保存：{pgm}、{yaml}')
            exit_code = 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
