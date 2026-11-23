"""
03 课 · 到点控制节点：在 RViz 里点一个目标，小车用 P 控制开过去（不避障）。

数据流：
  /goal_pose (PoseStamped，RViz 工具栏的 "2D Goal Pose") ──┐
  /odom (Odometry，当前位姿) ──────────────────────────────┴─▶ P 控制 ─▶ /cmd_vel

这个节点故意不做避障，用来单独观察比例控制的效果：
调大 k_angular 会怎样？朝向误差阈值 rotate_threshold 有什么用？
需要绕开障碍时，用本课的 navigate_server（Bug2）。

运行（仿真已启动）：
  ros2 run pg_go_to_goal_py go_to_goal --ros-args -p use_sim_time:=true
发目标（odom 坐标系）：
  RViz 中点 "2D Goal Pose"，或
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped
      "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5}}}"
"""

import math

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from pg_go_to_goal_py.go_to_goal import compute_go_to_goal, GoToGoalParams
from pg_go_to_goal_py.ros_utils import (init_with_safe_shutdown, make_twist_stamped,
                                        pose_from_odometry, publish_stop, spin_until_shutdown)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

ODOM_FRAME = 'odom'


class GoToGoalNode(Node):
    """接收目标点，用 P 控制开过去。"""

    def __init__(self):
        """声明参数，创建订阅、发布和控制定时器。"""
        super().__init__('go_to_goal')
        # 参数只在启动时读取一次（与 navigate_server 不同，这里没有参数回调），
        # 所以全部声明为只读：运行中 `ros2 param set` 会被明确拒绝，而不是"设置成功但没效果"
        d = GoToGoalParams()
        self._params = GoToGoalParams(
            k_linear=self._declare('k_linear', d.k_linear, '线速度增益，(m/s)/m'),
            k_angular=self._declare('k_angular', d.k_angular, '角速度增益，(rad/s)/rad'),
            max_linear_speed=self._declare(
                'max_linear_speed', d.max_linear_speed, '最大线速度，m/s'),
            max_angular_speed=self._declare(
                'max_angular_speed', d.max_angular_speed, '最大角速度，rad/s'),
            goal_tolerance=self._declare(
                'goal_tolerance', d.goal_tolerance, '到达容差，m'),
            rotate_threshold=math.radians(self._declare(
                'rotate_threshold_deg', math.degrees(d.rotate_threshold),
                '朝向误差大于此角度时先原地转向，度')))
        rate_hz = self._declare('control_rate_hz', 10.0, '控制频率，Hz')

        self._pose = None   # 最新位姿 Pose2D
        self._goal = None   # 当前目标 (x, y)；None 表示没有任务

        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_timer(1.0 / rate_hz, self._on_control)
        self.get_logger().info('到点控制已启动，等待目标（/goal_pose，odom 坐标系）')

    def _declare(self, name, default, description):
        """声明一个只读参数并返回它的值（参数文件里的值优先）。"""
        descriptor = ParameterDescriptor(description=description, read_only=True)
        return self.declare_parameter(name, default, descriptor).value

    def _on_odom(self, msg):
        """里程计回调：保存当前位姿。"""
        self._pose = pose_from_odometry(msg)

    def _on_goal(self, msg):
        """目标回调：检查坐标系，设置新目标（会覆盖正在执行的目标）。"""
        if msg.header.frame_id != ODOM_FRAME:
            # 本课还没有引入 tf 坐标变换（05 课会用到），所以只接受 odom 坐标系的目标
            self.get_logger().error(
                f'只接受 {ODOM_FRAME} 坐标系的目标，收到的是 "{msg.header.frame_id}"。'
                f'请把 RViz 的 Fixed Frame 设为 {ODOM_FRAME}')
            return
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'新目标：({self._goal[0]:.2f}, {self._goal[1]:.2f})')

    def _on_control(self):
        """控制定时器回调：有目标且有位姿时，计算并发布速度。"""
        if self._goal is None or self._pose is None:
            return
        command, reached = compute_go_to_goal(self._pose, *self._goal, self._params)
        self._cmd_pub.publish(make_twist_stamped(command, self.get_clock().now().to_msg()))
        if reached:
            self.get_logger().info(
                f'已到达目标，当前位置 ({self._pose.x:.2f}, {self._pose.y:.2f})')
            self._goal = None

    def stop(self):
        """发布零速度。"""
        publish_stop(self, self._cmd_pub)


def main(args=None):
    """节点入口：运行到 Ctrl+C，退出前停车。"""
    init_with_safe_shutdown(args)
    node = GoToGoalNode()
    spin_until_shutdown(node, on_exit=node.stop)


if __name__ == '__main__':
    main()
