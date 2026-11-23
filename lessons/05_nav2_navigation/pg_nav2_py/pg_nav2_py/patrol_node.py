"""
05 课 · 多点巡逻节点：按航点文件循环巡逻，记录每个点的结果和用时。

用 Nav2 自带的 Python 接口 nav2_simple_commander.BasicNavigator：
它把 navigate_to_pose 等动作包装成普通的函数调用（goToPose、isTaskComplete、getResult ……），
不用自己写动作客户端。巡逻节点直接继承它，再加上自己的参数。

    ┌──────── 巡逻节点 ────────┐   navigate_to_pose 动作    ┌─── Nav2 ───┐
    │ PatrolSchedule：下一个点 │ ──────── 目标 ───────────→ │ bt_navigator│
    │ PatrolStats：结果和用时  │ ←── 反馈（剩余距离）/ 结果 ─ │   ……       │
    └──────────────────────────┘                             └────────────┘

为什么不用 Nav2 的 follow_waypoints 动作（BasicNavigator.followWaypoints）一次发完？
它只在最后告诉你"哪些点没到"，拿不到每个点的用时，也不能对单个点设超时。
逐个发 navigate_to_pose 更适合学习和统计。

用法：
  ros2 launch pg_nav2_py nav2_sim.launch.py patrol:=true             # 和 Nav2 一起启动
  ros2 run pg_nav2_py patrol --ros-args -p use_sim_time:=true -p loops:=1
  ros2 run pg_nav2_py patrol --ros-args -p use_sim_time:=true -p controller:=RPP
"""

import os
import signal

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from pg_nav2_py.patrol import GoalOutcome, PatrolSchedule, PatrolStats
from pg_nav2_py.waypoints import load_route, yaw_to_quaternion
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_WAYPOINTS = os.path.join(
    get_package_share_directory('pg_nav2_py'), 'config', 'waypoints.yaml')

# 参数名 → (默认值, 说明)。只在启动时读取，声明为只读
PARAMS = {
    'waypoints_file': (DEFAULT_WAYPOINTS, '航点文件（YAML），格式见 config/waypoints.yaml'),
    'loops': (0, '巡逻圈数，0 表示一直巡逻'),
    'goal_timeout': (180.0, '单个航点的超时，超时后取消并跳到下一个点，s（仿真时间）'),
    'dwell_time': (2.0, '到达航点后停留多久再去下一个，s（仿真时间）'),
    'max_consecutive_failures': (3, '连续失败多少个航点后停止巡逻'),
    'planner': ('', '规划器：GridBased | Smac2D | ThetaStar，空字符串表示用行为树默认（GridBased）'),
    'controller': ('', '控制器：FollowPath（DWB）| RPP | MPPI，空字符串表示用行为树默认（FollowPath）'),
    'feedback_period': (10.0, '打印导航进度的间隔，s（仿真时间）'),
}

# 规划器 / 控制器选择话题的 QoS：行为树里的 PlannerSelector、ControllerSelector 用
# Transient Local 订阅，发布者也要用 Transient Local，后启动的一方才能收到之前选好的值
SELECTOR_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

# BasicNavigator 的结果 → 巡逻的结果
OUTCOMES = {
    TaskResult.SUCCEEDED: GoalOutcome.SUCCEEDED,
    TaskResult.FAILED: GoalOutcome.FAILED,
    TaskResult.CANCELED: GoalOutcome.CANCELED,
    TaskResult.UNKNOWN: GoalOutcome.FAILED,
}


class Patroller(BasicNavigator):
    """巡逻节点：BasicNavigator + 航点、圈数、超时等参数。"""

    def __init__(self):
        """声明参数，读取航点文件，创建选择器和航点标记的发布者。"""
        super().__init__(node_name='patroller')
        values = {
            name: self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True)).value
            for name, (default, description) in PARAMS.items()}
        self._goal_timeout = Duration(seconds=values['goal_timeout'])
        self._dwell_time = Duration(seconds=values['dwell_time'])
        self._feedback_period = Duration(seconds=values['feedback_period'])
        self._planner = values['planner']
        self._controller = values['controller']

        self.route = load_route(values['waypoints_file'])
        names = [w.name for w in self.route.waypoints]
        self.schedule = PatrolSchedule(
            self.route.waypoints, values['loops'], values['max_consecutive_failures'])
        self.stats = PatrolStats(names)
        self._task_active = False   # 是否有一个导航目标正在执行（Ctrl+C 时要取消它）

        self._planner_pub = self.create_publisher(String, 'planner_selector', SELECTOR_QOS)
        self._controller_pub = self.create_publisher(String, 'controller_selector', SELECTOR_QOS)
        self._marker_pub = self.create_publisher(MarkerArray, 'patrol_waypoints', SELECTOR_QOS)

        loops = values['loops'] or '无限'
        self.info(f'航点 {len(names)} 个（{" → ".join(names)}），圈数 {loops}，'
                  f'单点超时 {values["goal_timeout"]:.0f} s')

    # ───────────────────────────── 准备 ─────────────────────────────

    def select_algorithms(self):
        """按参数选择规划器和控制器（发布到行为树的选择器话题）；参数为空时不发布。"""
        for name, publisher, what in [(self._planner, self._planner_pub, '规划器'),
                                      (self._controller, self._controller_pub, '控制器')]:
            if name:
                publisher.publish(String(data=name))
                self.info(f'{what}：{name}')

    def publish_markers(self):
        """在 RViz 里显示航点：箭头表示位置和朝向，文字是名字。"""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, wp in enumerate(self.route.waypoints):
            for kind in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header.frame_id = self.route.frame_id
                m.header.stamp = stamp
                m.ns = 'arrow' if kind == Marker.ARROW else 'name'
                m.id = i
                m.type = kind
                m.pose = self._make_pose(wp).pose
                if kind == Marker.ARROW:
                    m.scale.x, m.scale.y, m.scale.z = 0.3, 0.06, 0.06
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 1.0
                else:
                    m.pose.position.z = 0.3
                    m.scale.z = 0.15
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                    m.text = wp.name
                markers.markers.append(m)
        self._marker_pub.publish(markers)

    # ───────────────────────────── 巡逻 ─────────────────────────────

    def wait_for_nav2(self):
        """
        等 amcl 激活并给出定位结果、bt_navigator 激活。只等，不发布初始位姿。

        BasicNavigator.waitUntilNav2Active() 在收到 /amcl_pose 之前会反复发布初始位姿
        （默认是地图原点）。巡逻节点和 Nav2 一起启动时没关系，但如果小车已经开到别处、
        再单独启动巡逻节点，就会把 AMCL 的定位"拽回"原点。这里改为只订阅、不发布：
        初始位姿由参数文件（amcl.set_initial_pose）或 RViz 的 2D Pose Estimate 设置。
        """
        self._waitForNodeToActivate('amcl')
        # initial_pose_received 由 BasicNavigator 订阅 /amcl_pose 的回调设置；
        # /amcl_pose 是 Transient Local，AMCL 之前发布过的最后一条也能收到
        while not self.initial_pose_received:
            self.info('等待 AMCL 的定位结果；如果一直等不到，在 RViz 里用 2D Pose Estimate 设置初始位姿')
            rclpy.spin_once(self, timeout_sec=2.0)
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 已就绪')

    def run(self):
        """等 Nav2 就绪，然后按计划逐个去航点，直到巡逻结束。"""
        self.wait_for_nav2()
        self.select_algorithms()
        self.publish_markers()

        while (leg := self.schedule.current) is not None:
            outcome, duration = self.visit(leg)
            self.stats.record(leg.waypoint.name, outcome, duration)
            self.schedule.report(outcome)
            self.info(f'[第 {leg.loop} 圈] {leg.waypoint.name}：{outcome.value}，'
                      f'用时 {duration:.1f} s')
            if outcome is GoalOutcome.SUCCEEDED and not self.schedule.finished:
                self.sleep_sim(self._dwell_time)

    def visit(self, leg):
        """
        去一个航点：发目标、等结果，期间定时打印进度，超时就取消。

        :param leg: patrol.Leg
        :return: (GoalOutcome, 用时 s)
        """
        wp = leg.waypoint
        self.info(f'[第 {leg.loop} 圈] 前往 {wp.name} ({wp.x:.2f}, {wp.y:.2f})')
        start = self.get_clock().now()
        if not self.goToPose(self._make_pose(wp)):
            return GoalOutcome.FAILED, 0.0
        self._task_active = True

        last_report = start
        # isTaskComplete() 内部会 spin 最多 0.1 s，所以这个循环不会空转
        while not self.isTaskComplete():
            now = self.get_clock().now()
            if now - start > self._goal_timeout:
                self.warn(f'{wp.name} 超时，取消')
                self.cancel_current()
                return GoalOutcome.TIMEOUT, self._seconds_since(start)
            feedback = self.getFeedback()
            if feedback is not None and now - last_report >= self._feedback_period:
                last_report = now
                self.info(f'  剩余 {feedback.distance_remaining:.2f} m，'
                          f'恢复行为 {feedback.number_of_recoveries} 次')
        self._task_active = False

        outcome = OUTCOMES[self.getResult()]
        if outcome is GoalOutcome.FAILED:
            result = self.result_future.result().result
            self.warn(f'{wp.name} 失败，错误码 {result.error_code}')
        return outcome, self._seconds_since(start)

    def cancel_current(self):
        """取消正在执行的导航目标，并等到 Nav2 确认结束。"""
        if not self._task_active:
            return
        self.cancelTask()
        while not self.isTaskComplete():
            pass
        self._task_active = False

    def sleep_sim(self, duration):
        """按仿真时间等待（期间继续处理消息，/clock 才会更新）。"""
        end = self.get_clock().now() + duration
        while self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def log_summary(self):
        """打印巡逻汇总。"""
        self.info(f'巡逻结束：{self.schedule.stop_reason}，'
                  f'完成 {self.schedule.completed_loops} 圈')
        for line in self.stats.summary_lines():
            self.info(line)

    # ───────────────────────────── 工具 ─────────────────────────────

    def _make_pose(self, wp):
        """航点 → PoseStamped。"""
        pose = PoseStamped()
        pose.header.frame_id = self.route.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        q = yaw_to_quaternion(wp.yaw)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        return pose

    def _seconds_since(self, start):
        """从 start 到现在的秒数（节点时钟，use_sim_time 时是仿真时间）。"""
        return (self.get_clock().now() - start).nanoseconds / 1e9


def main(args=None):
    """
    节点入口：巡逻到结束或 Ctrl+C，退出前取消正在执行的目标并打印汇总。

    rclpy 默认的信号处理会在 Ctrl+C 时先关闭 ROS，之后就没法再发"取消目标"的请求了，
    小车会继续开向最后一个航点。所以这里自己处理 Ctrl+C（原理见 02 课 avoider_node.py）。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = Patroller()
    exit_code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        node.schedule.stop('Ctrl+C')
        node.cancel_current()
    finally:
        node.schedule.stop('异常退出')    # 正常结束或 Ctrl+C 时已有原因，这里不会覆盖
        node.log_summary()
        # 有任何一个航点没到达，就用非零退出码告诉调用者（例如验收脚本）
        if node.stats.count(GoalOutcome.SUCCEEDED) < node.stats.attempts:
            exit_code = 1
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
