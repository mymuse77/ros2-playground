"""
03 课 · 导航动作客户端：发一个目标点，打印进度，可以中途取消。

用法：
  ros2 run pg_go_to_goal_py navigate_client 3.0 0.0
  ros2 run pg_go_to_goal_py navigate_client 3.0 0.0 --tolerance 0.15
  ros2 run pg_go_to_goal_py navigate_client 3.0 0.0 --cancel-after 10   # 10 秒后取消
  （仿真中加 --ros-args -p use_sim_time:=true，让 --cancel-after 按仿真时间计）

运行中按 Ctrl+C 也会先取消目标再退出，小车不会带着旧目标继续跑。

流程与 01 课的倒计时客户端相同：
  等待服务端 → 发送目标 → 收到"接受/拒绝" → 持续收反馈 → 等待结果
"""

import argparse
import sys

from action_msgs.msg import GoalStatus
from pg_go_to_goal_py.ros_utils import init_with_safe_shutdown, SPIN_TIMEOUT_S
from pg_interfaces.action import NavigateToPoint
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

SERVER_WAIT_S = 10.0            # 启动时等待服务端上线的时间，s
FEEDBACK_PRINT_PERIOD_S = 1.0   # 反馈约 10 Hz，每秒打印一次即可，s（ROS 时间）
# 以下等待都按真实时间（稳定时钟）计，仿真卡住时也能超时
CANCEL_WAIT_S = 5.0             # Ctrl+C 后等待服务端确认取消的时间，s
GOAL_RESPONSE_WAIT_S = 5.0      # 等待"接受/拒绝"回应的时间，超时则重发目标，s
GOAL_SEND_ATTEMPTS = 3          # 最多发送几次目标
RESULT_RETRY_S = 3.0            # 目标已结束但结果迟迟不到时，隔多久重新请求结果
SERVER_LOST_S = 5.0             # 服务端连续这么久不在线，就认为它已退出，不再等结果
TERMINAL_STATUSES = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                     GoalStatus.STATUS_ABORTED)


def parse_args(argv):
    """
    解析命令行参数。

    :param argv: 参数列表，不含程序名和 --ros-args 之后的 ROS 参数
    :return: argparse.Namespace，字段 x、y、tolerance、cancel_after
    """
    parser = argparse.ArgumentParser(description='发送 NavigateToPoint 目标（odom 坐标系）')
    parser.add_argument('x', type=float, help='目标 x，m')
    parser.add_argument('y', type=float, help='目标 y，m')
    parser.add_argument('--tolerance', type=float, default=0.0,
                        help='到达容差，m；<= 0 表示使用服务端参数（默认）')
    parser.add_argument('--cancel-after', type=float, default=None, metavar='SECONDS',
                        help='发送后多少秒取消目标（演示取消功能）')
    return parser.parse_args(argv)


class NavigateClient(Node):
    """NavigateToPoint 动作客户端。"""

    def __init__(self):
        """创建动作客户端。"""
        super().__init__('navigate_client')
        self._client = ActionClient(self, NavigateToPoint, 'navigate_to_point')
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_print = None   # 上次打印反馈的时间，s

    def send(self, x, y, tolerance):
        """
        等待服务端并发送目标。

        发送目标本质上是一次服务调用。客户端刚启动、DDS 还没完成发现时，服务端的
        "已接受"回应可能送不到（服务端日志会出现 failed to send response），
        客户端就会一直等下去。所以等待回应设了超时，超时后重发：
        服务端的抢占规则保证旧的那个"没人认领"的目标会被新目标取代。

        :return: 目标句柄 ClientGoalHandle；服务端不在线、拒绝或一直无回应时返回 None
        """
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            self.get_logger().error('等不到服务端 navigate_to_point，navigate_server 启动了吗？')
            return None
        goal = NavigateToPoint.Goal(x=x, y=y, tolerance=tolerance)
        for attempt in range(1, GOAL_SEND_ATTEMPTS + 1):
            future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            if self._spin_until(future, GOAL_RESPONSE_WAIT_S):
                break
            self.get_logger().warning(
                f'{GOAL_RESPONSE_WAIT_S:.0f} s 内没有收到服务端回应（第 {attempt} 次），重发目标')
        else:
            self.get_logger().error('多次发送都没有回应，放弃')
            return None
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('目标被拒绝')
            return None
        self.get_logger().info(f'目标已接受：({x:.2f}, {y:.2f})')
        return handle

    def _on_feedback(self, message):
        """反馈回调：限频打印当前位置、剩余距离和状态。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_print is not None and now - self._last_print < FEEDBACK_PRINT_PERIOD_S:
            return
        self._last_print = now
        f = message.feedback
        self.get_logger().info(
            f'[{f.state}] 位置 ({f.current_x:.2f}, {f.current_y:.2f})，'
            f'剩余 {f.distance_remaining:.2f} m')

    def wait_result(self, handle, cancel_after=None):
        """
        等待结果；cancel_after 秒后（ROS 时间）还没结束就发取消请求。

        请求结果也是一次服务调用，回应同样可能丢失。目标状态（走话题，见
        handle.status）已经是终态、结果却迟迟不到时，重新请求一次结果；
        服务端会把结果保留一段时间（默认 15 分钟），可以重复获取。

        服务端中途退出（崩溃、被关掉）时结果永远不会来，所以还要检查服务端是否在线。

        :return: 结果 NavigateToPoint.Result；服务端已退出时返回 None
        """
        result_future = handle.get_result_async()
        start = self.get_clock().now()
        cancel_sent = False
        terminal_since = None   # 发现目标已结束的时间（稳定时钟），s
        online_at = self._now_steady()   # 最近一次确认服务端在线的时间（稳定时钟），s
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)
            if self._client.server_is_ready():
                online_at = self._now_steady()
            elif self._now_steady() - online_at > SERVER_LOST_S:
                self.get_logger().error('服务端已离线，不再等待结果')
                return None
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if cancel_after is not None and not cancel_sent and elapsed >= cancel_after:
                self.get_logger().info(f'已过 {cancel_after:.1f} s，发送取消请求')
                handle.cancel_goal_async()
                cancel_sent = True
            if handle.status in TERMINAL_STATUSES:
                now = self._now_steady()
                if terminal_since is None:
                    terminal_since = now
                elif now - terminal_since > RESULT_RETRY_S:
                    self.get_logger().warning('目标已结束但没有收到结果，重新请求')
                    result_future = handle.get_result_async()
                    terminal_since = now
        return result_future.result().result

    def cancel(self, handle):
        """取消目标并等待服务端确认（用于 Ctrl+C 退出）。"""
        future = handle.cancel_goal_async()
        if self._spin_until(future, CANCEL_WAIT_S):
            self.get_logger().info('目标已取消')
        else:
            self.get_logger().warning('取消请求没有得到服务端确认')

    def _spin_until(self, future, timeout=None):
        """
        处理回调直到 future 完成；返回是否在超时前完成。

        :param timeout: 超时时间（真实时间），s；None 表示一直等

        不用 rclpy.spin_until_future_complete()：它在 C 层阻塞等待，
        期间 Python 的信号处理函数执行不了，Ctrl+C 要等到下一条消息到达才生效。
        """
        start = self._now_steady()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)
            if timeout is not None and self._now_steady() - start > timeout:
                return False
        return True

    def _now_steady(self):
        """稳定时钟的当前时间，s。"""
        return self._steady.now().nanoseconds * 1e-9


def main(args=None):
    """节点入口：发送目标，打印结果；成功返回 0，否则返回 1。"""
    options = parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    # 接管 SIGINT：Ctrl+C 时 ROS 上下文仍然有效，才能发出取消请求
    init_with_safe_shutdown(args)
    node = NavigateClient()
    handle = None
    exit_code = 1
    try:
        handle = node.send(options.x, options.y, options.tolerance)
        if handle is not None:
            result = node.wait_result(handle, options.cancel_after)
            handle = None   # 已结束（或服务端已不在），退出时不需要再取消
            if result is not None:
                outcome = '成功' if result.success else '未成功'
                node.get_logger().info(
                    f'{outcome}：{result.message}（里程 {result.distance_traveled:.2f} m，'
                    f'用时 {result.elapsed_time:.1f} s）')
                exit_code = 0 if result.success else 1
    except KeyboardInterrupt:
        if handle is not None and rclpy.ok():
            node.cancel(handle)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
