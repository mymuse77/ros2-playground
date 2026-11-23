"""
03 课 · 导航动作服务端：用 Bug2 算法开到目标点，过程中持续反馈，可以取消。

动作接口 pg_interfaces/action/NavigateToPoint：
  目标   x, y（odom 坐标系，m），tolerance（到达容差，<= 0 表示使用参数 goal_tolerance）
  反馈   当前位置、剩余距离、当前状态（GO_TO_GOAL / FOLLOW_WALL）
  结果   是否成功、说明、行驶里程、用时

这是 01 课"倒计时动作"的实战版：执行回调里跑一个控制循环，每个周期读传感器、
算速度、发反馈、检查取消。

几个设计要点：
  - 新目标会"抢占"旧目标（与 Nav2 的行为一致）：旧目标以 ABORTED 结束
  - 控制循环用稳定时钟定节拍，雷达/里程计超时则停车等待（与 02 课相同的看门狗思路）
  - 超过 goal_timeout（仿真时间）仍未到达则中止，防止无限绕行
  - 每个目标开始时读取一次参数，修改参数对下一个目标生效；修改时整体校验，不合法则拒绝
  - 在 RViz 中发布 M-line、目标点、碰撞点标记（/navigate_server/markers）

运行（仿真已启动）：
  ros2 run pg_go_to_goal_py navigate_server --ros-args -p use_sim_time:=true
发目标：
  ros2 action send_goal --feedback /navigate_to_point pg_interfaces/action/NavigateToPoint
      "{x: 3.0, y: 0.0}"
  或 ros2 run pg_go_to_goal_py navigate_client 3.0 0.0
"""

import threading

from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry
from pg_go_to_goal_py.bug2 import Bug2Params, Bug2Planner, Bug2State
from pg_go_to_goal_py.geometry import distance
from pg_go_to_goal_py.go_to_goal import GoToGoalParams, STOP
from pg_go_to_goal_py.ros_utils import (init_with_safe_shutdown, make_twist_stamped,
                                        pose_from_odometry, publish_stop, spin_until_shutdown)
from pg_interfaces.action import NavigateToPoint
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

ODOM_FRAME = 'odom'

# 参数名 → (默认值, 说明)。每个目标开始时读取一次
PARAMS = {
    'goal_tolerance': (GoToGoalParams().goal_tolerance, '默认到达容差，m'),
    'max_linear_speed': (GoToGoalParams().max_linear_speed, '朝目标前进的最大线速度，m/s'),
    'max_angular_speed': (GoToGoalParams().max_angular_speed, '朝目标前进的最大角速度，rad/s'),
    'k_linear': (GoToGoalParams().k_linear, '线速度增益，(m/s)/m'),
    'k_angular': (GoToGoalParams().k_angular, '角速度增益，(rad/s)/rad'),
    'hit_distance': (Bug2Params().hit_distance, '前方小于此距离视为被挡住，m'),
    'wall_distance': (Bug2Params().wall_distance, '沿边走时与障碍保持的距离，m'),
    'wall_speed': (Bug2Params().wall_speed, '沿边走的线速度，m/s'),
    'robot_radius': (Bug2Params().robot_radius, '车体半径：目标离障碍物比这近时车无法停在目标上，m'),
    'goal_timeout': (300.0, '单个目标的最长执行时间（ROS 时间），s'),
    'data_timeout': (0.5, '雷达或里程计超过此时间未更新则停车等待，s'),
    'control_rate_hz': (10.0, '控制频率，Hz'),
}


class NavigateServer(Node):
    """NavigateToPoint 动作服务端（Bug2）。"""

    def __init__(self):
        """声明参数，创建订阅、发布和动作服务端。"""
        super().__init__('navigate_server')
        for name, (default, description) in PARAMS.items():
            # 描述会显示在 `ros2 param describe` 里；控制频率只在启动时生效，设为只读
            self.declare_parameter(name, default, ParameterDescriptor(
                description=description, read_only=(name == 'control_rate_hz')))
        self._validate(self._current_params())   # 参数文件里的值不合法时直接报错退出
        # 运行中修改参数时先整体校验，不合法就拒绝（否则要到下一个目标开始时才出错）
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._lock = threading.Lock()
        self._pose = None            # 最新位姿 Pose2D
        self._pose_time = None       # 收到位姿的稳定时钟时间，s
        self._scan = None            # 最新雷达数据
        self._scan_time = None       # 收到雷达的稳定时钟时间，s
        self._active_goal = None     # 当前正在执行的目标句柄
        # 节点关闭标志：执行循环看到后结束目标，否则执行器关闭时会一直等待执行线程
        self._closing = threading.Event()

        # 传感器回调放在互斥组里，与动作执行回调互不阻塞
        sensors = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=sensors)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data,
                                 callback_group=sensors)
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)

        self._server = ActionServer(
            self, NavigateToPoint, 'navigate_to_point',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('导航动作服务端已就绪：navigate_to_point（目标使用 odom 坐标系）')

    # ------------------------------------------------------------------ 参数
    def _current_params(self):
        """读取全部参数，返回 {名字: 值}。"""
        return {name: self.get_parameter(name).value for name in PARAMS}

    def _validate(self, values):
        """检查一组参数是否合法，不合法时抛出 ValueError。"""
        self._bug2_params(values)   # GoToGoalParams、Bug2Params 构造时会检查各自的取值
        for name in ('goal_timeout', 'data_timeout', 'control_rate_hz'):
            if values[name] <= 0.0:
                raise ValueError(f'{name} 必须大于 0')

    def _on_set_parameters(self, parameters):
        """参数修改前的校验回调：把新值代入当前参数整体检查，不合法则拒绝。"""
        values = self._current_params()
        values.update({p.name: p.value for p in parameters if p.name in values})
        try:
            self._validate(values)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ 传感器
    def _on_odom(self, msg):
        """保存最新位姿。"""
        with self._lock:
            self._pose = pose_from_odometry(msg)
            self._pose_time = self._now_steady()

    def _on_scan(self, msg):
        """保存最新雷达数据。"""
        with self._lock:
            self._scan = msg
            self._scan_time = self._now_steady()

    def _sensor_snapshot(self, timeout):
        """取一份传感器数据快照；任一数据缺失或过期时返回 (None, None)。"""
        now = self._now_steady()
        with self._lock:
            fresh = (self._pose_time is not None and self._scan_time is not None
                     and now - self._pose_time <= timeout and now - self._scan_time <= timeout)
            return (self._pose, self._scan) if fresh else (None, None)

    # ------------------------------------------------------------------ 动作
    def _on_goal(self, goal_request):
        """收到新目标：坐标必须是有限数值。"""
        values = (goal_request.x, goal_request.y, goal_request.tolerance)
        if not all(abs(v) < float('inf') for v in values):   # 同时排除 NaN 和 inf
            self.get_logger().warning('拒绝目标：坐标或容差不是有限数值')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        """目标被接受后：登记为当前目标（旧目标会在自己的循环里发现被抢占），然后开始执行。"""
        with self._lock:
            self._active_goal = goal_handle
        goal_handle.execute()

    def _execute(self, goal_handle):
        """执行一个导航目标：控制循环直到到达、失败、取消、超时或被抢占。"""
        goal = goal_handle.request
        values = self._current_params()   # 参数已在修改时校验过
        planner = Bug2Planner(self._bug2_params(values))
        tolerance = goal.tolerance if goal.tolerance > 0.0 else values['goal_tolerance']
        period = Duration(seconds=1.0 / values['control_rate_hz'])
        log = self.get_logger()
        log.info(f'新目标：({goal.x:.2f}, {goal.y:.2f})，容差 {tolerance:.2f} m')

        start_time = self._now_ros()
        traveled, last_xy, started, last_state = 0.0, None, False, None
        feedback = NavigateToPoint.Feedback()

        def finish(status, success, message):
            """统一的结束处理：停车、设置最终状态、返回结果。"""
            publish_stop(self, self._cmd_pub)
            status()
            elapsed = self._now_ros() - start_time
            log.info(f'{message}（里程 {traveled:.2f} m，用时 {elapsed:.1f} s）')
            return NavigateToPoint.Result(
                success=success, message=message, distance_traveled=traveled,
                elapsed_time=elapsed)

        while True:
            if self._closing.is_set():
                return finish(goal_handle.abort, False, '节点正在关闭')
            if goal_handle.is_cancel_requested:
                return finish(goal_handle.canceled, False, '目标已取消')
            if self._active_goal is not goal_handle:
                return finish(goal_handle.abort, False, '被新目标取代')
            if self._now_ros() - start_time > values['goal_timeout']:
                return finish(goal_handle.abort, False,
                              f'超过 {values["goal_timeout"]:.0f} s 仍未到达，已中止')

            pose, scan = self._sensor_snapshot(values['data_timeout'])
            if pose is None:
                # 数据缺失或过期：停车等待，不结束任务
                self._cmd_pub.publish(make_twist_stamped(
                    STOP, self.get_clock().now().to_msg()))
            else:
                if not started:
                    planner.start(pose, goal.x, goal.y, tolerance)
                    started = True
                command = planner.update(pose, scan)
                if planner.state is Bug2State.REACHED:
                    return finish(goal_handle.succeed, True,
                                  f'已到达目标，误差 {planner.distance_to_goal(pose):.3f} m')
                if planner.state is Bug2State.FAILED:
                    return finish(goal_handle.abort, False, planner.failure_reason)
                self._cmd_pub.publish(make_twist_stamped(
                    command, self.get_clock().now().to_msg()))

                if last_xy is not None:
                    traveled += distance(*last_xy, pose.x, pose.y)
                last_xy = (pose.x, pose.y)
                if planner.state is not last_state:
                    log.info(f'状态 → {planner.state.value}（位置 {pose.x:.2f}, {pose.y:.2f}）')
                    last_state = planner.state

                feedback.current_x, feedback.current_y = pose.x, pose.y
                feedback.distance_remaining = planner.distance_to_goal(pose)
                feedback.state = planner.state.value
                goal_handle.publish_feedback(feedback)
                self._publish_markers(planner, goal.x, goal.y)

            if not self._steady.sleep_for(period):
                return finish(goal_handle.abort, False, '节点正在关闭')

    # ------------------------------------------------------------------ 工具
    @staticmethod
    def _bug2_params(values):
        """把 ROS 参数组装成 Bug2Params。"""
        go = GoToGoalParams(
            k_linear=values['k_linear'], k_angular=values['k_angular'],
            max_linear_speed=values['max_linear_speed'],
            max_angular_speed=values['max_angular_speed'],
            goal_tolerance=values['goal_tolerance'])
        return Bug2Params(go_to_goal=go, hit_distance=values['hit_distance'],
                          wall_distance=values['wall_distance'],
                          wall_speed=values['wall_speed'],
                          robot_radius=values['robot_radius'])

    def _publish_markers(self, planner, goal_x, goal_y):
        """在 RViz 中画出 M-line、目标点和碰撞点。"""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        mline = _marker(stamp, 0, 'm_line', Marker.LINE_STRIP, (0.9, 0.9, 0.9, 0.8))
        mline.scale.x = 0.02
        mline.points = [Point(x=planner.start_point[0], y=planner.start_point[1]),
                        Point(x=goal_x, y=goal_y)]
        markers.markers.append(mline)

        goal = _marker(stamp, 1, 'goal', Marker.CYLINDER, (0.2, 0.9, 0.2, 0.8))
        goal.pose.position.x, goal.pose.position.y = goal_x, goal_y
        goal.scale.x = goal.scale.y = 2 * planner.tolerance
        goal.scale.z = 0.05
        markers.markers.append(goal)

        if planner.hit_point is not None:
            hit = _marker(stamp, 2, 'hit_point', Marker.SPHERE, (0.95, 0.3, 0.2, 0.9))
            hit.pose.position.x, hit.pose.position.y = planner.hit_point
            hit.scale.x = hit.scale.y = hit.scale.z = 0.1
            markers.markers.append(hit)
        self._marker_pub.publish(markers)

    def close(self):
        """节点退出前调用：通知执行循环结束当前目标，并发布零速度。"""
        self._closing.set()
        publish_stop(self, self._cmd_pub)

    def _now_steady(self):
        """稳定时钟的当前时间，s。"""
        return self._steady.now().nanoseconds * 1e-9

    def _now_ros(self):
        """ROS 时钟（仿真时为仿真时钟）的当前时间，s。"""
        return self.get_clock().now().nanoseconds * 1e-9


def _marker(stamp, marker_id, namespace, marker_type, rgba):
    """创建一个 odom 坐标系下的 Marker。"""
    marker = Marker()
    marker.header.frame_id = ODOM_FRAME
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return marker


def main(args=None):
    """节点入口：多线程执行器运行，退出前停车。"""
    init_with_safe_shutdown(args)
    node = NavigateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_until_shutdown(node, executor=executor, on_exit=node.close)


if __name__ == '__main__':
    main()
