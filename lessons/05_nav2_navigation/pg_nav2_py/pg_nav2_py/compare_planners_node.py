"""
05 课 · 规划器对比：同样的起点和终点，让每个规划器各算一条路径，比较长度、转弯量、离障碍的距离和计算用时。

"计算用时"是从发出请求到收到结果的墙钟时间（见 plan() 的注释）。

只算路径、不开车：调用 planner_server 的 compute_path_to_pose 动作，并指定 use_start=True
（起点用我们给的位姿，而不是小车当前的位置），所以几秒钟就能比完整条巡逻路线。

路线取自航点文件，相邻两个航点为一段，最后一个回到第一个：
    home → north → east → south → center → home

输出一张 Markdown 表格，可以直接贴进 README。

用法（先启动 nav2_sim.launch.py）：
  ros2 run pg_nav2_py compare_planners --ros-args -p use_sim_time:=true
  ros2 run pg_nav2_py compare_planners --ros-args -p use_sim_time:=true \
      -p planners:="['GridBased', 'ThetaStar']"
"""

import math
import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from pg_nav2_py.path_metrics import min_clearance, occupied_cells, path_length, total_turning
from pg_nav2_py.waypoints import load_route, yaw_to_quaternion
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_WAYPOINTS = os.path.join(
    get_package_share_directory('pg_nav2_py'), 'config', 'waypoints.yaml')

PARAMS = {
    'waypoints_file': (DEFAULT_WAYPOINTS, '航点文件（YAML），相邻两点为一段'),
    'planners': (['GridBased', 'Smac2D', 'ThetaStar'], '要对比的规划器（参数文件里的名字）'),
}

# /map 用 Transient Local 发布，订阅端也要一致（04 课知识点 6）
MAP_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class PlannerComparer(BasicNavigator):
    """规划器对比节点。"""

    def __init__(self):
        """声明参数，读取航点，订阅地图（用来算离障碍的距离）。"""
        super().__init__(node_name='planner_comparer')
        values = {
            name: self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True)).value
            for name, (default, description) in PARAMS.items()}
        self.route = load_route(values['waypoints_file'])
        self.planners = list(values['planners'])
        self.obstacles = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)

    def _on_map(self, msg):
        """收到地图：取出所有障碍格子的中心点。"""
        info = msg.info
        self.obstacles = occupied_cells(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, msg.data)

    def make_pose(self, wp):
        """航点 → PoseStamped。"""
        pose = PoseStamped()
        pose.header.frame_id = self.route.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quaternion(wp.yaw)
        return pose

    def plan(self, start, goal, planner):
        """
        让指定的规划器算一条路径。

        :return: (路径点 [(x, y), ...], 请求用时 s)；规划失败时为 (None, None)
        """
        # 用时自己用墙钟量，不用结果里的 planning_time：planner_server 用节点时钟计时，
        # use_sim_time 时那是仿真时间，一次规划期间 /clock 几乎不走，读出来基本是 0。
        # 墙钟量的是整个动作请求（发目标 → 收结果），包含几毫秒的通信开销
        t0 = time.perf_counter()
        # getPath() 只返回路径，_getPathImpl() 返回完整结果和状态。
        # 它是 BasicNavigator 的内部函数，但 Nav2 自己的 getPath 也是这么用的
        result = self._getPathImpl(
            self.make_pose(start), self.make_pose(goal), planner, use_start=True)
        seconds = time.perf_counter() - t0
        if result is None or self.status != GoalStatus.STATUS_SUCCEEDED:
            return None, None
        points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
        return points, seconds

    def run(self):
        """逐段、逐个规划器计算并打印表格。"""
        # 只需要 planner_server，不需要定位，所以不等 amcl
        self._waitForNodeToActivate('planner_server')
        while self.obstacles is None:
            self.info('等待 /map ……')
            rclpy.spin_once(self, timeout_sec=1.0)

        wps = self.route.waypoints
        legs = list(zip(wps, wps[1:] + wps[:1]))
        totals = {p: {'length': 0.0, 'turn': 0.0, 'clear': math.inf, 'time': 0.0, 'ok': 0}
                  for p in self.planners}

        lines = ['| 路段 | 规划器 | 长度 m | 累计转弯 ° | 最小距离 m | 请求 ms |',
                 '|---|---|---|---|---|---|']
        for start, goal in legs:
            for planner in self.planners:
                points, seconds = self.plan(start, goal, planner)
                leg = f'{start.name}→{goal.name}'
                if points is None:
                    lines.append(f'| {leg} | {planner} | 失败 | | | |')
                    continue
                length = path_length(points)
                turn = math.degrees(total_turning(points))
                clear = min_clearance(points, self.obstacles)
                t = totals[planner]
                t['length'] += length
                t['turn'] += turn
                t['clear'] = min(t['clear'], clear)
                t['time'] += seconds
                t['ok'] += 1
                lines.append(f'| {leg} | {planner} | {length:.2f} | {turn:.0f} | '
                             f'{clear:.2f} | {seconds * 1000:.1f} |')
        for planner, t in totals.items():
            mean_ms = t['time'] / t['ok'] * 1000 if t['ok'] else math.nan
            lines.append(f'| **合计（{t["ok"]}/{len(legs)} 段）** | **{planner}** | '
                         f'{t["length"]:.2f} | {t["turn"]:.0f} | {t["clear"]:.2f} | '
                         f'平均 {mean_ms:.1f} |')

        # 一次性打印，避免和 BasicNavigator 的日志交错
        print('\n'.join(lines), flush=True)


def main(args=None):
    """节点入口：算完就退出。"""
    rclpy.init(args=args)
    node = PlannerComparer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
