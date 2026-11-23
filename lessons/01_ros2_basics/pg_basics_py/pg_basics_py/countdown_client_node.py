"""
01 课 · 动作客户端：发送倒计时目标，打印反馈，可在指定时间后取消。

知识点：
  - ActionClient(节点, 动作类型, 动作名)
  - send_goal_async(目标, feedback_callback)：发送目标，返回的 Future 完成后得到目标句柄，
    句柄的 accepted 表示服务端是否接受
  - goal_handle.get_result_async()：等待最终结果
  - goal_handle.cancel_goal_async()：请求取消
  - 结果里的 status 表示目标的最终状态：SUCCEEDED / CANCELED / ABORTED

运行（先启动 countdown_server）：
  ros2 run pg_basics_py countdown_client 5                   # 倒计时 5 秒
  ros2 run pg_basics_py countdown_client 10 --cancel-after 3 # 3 秒后取消
"""

import argparse
import sys

from action_msgs.msg import GoalStatus
from pg_interfaces.action import Countdown
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

SERVER_WAIT_TIMEOUT_S = 5.0

# 目标最终状态码 → 可读名称
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: '成功',
    GoalStatus.STATUS_CANCELED: '已取消',
    GoalStatus.STATUS_ABORTED: '已中止',
}


class CountdownClient(Node):
    """倒计时动作客户端。"""

    def __init__(self):
        """创建动作客户端。"""
        super().__init__('countdown_client')
        self._client = ActionClient(self, Countdown, 'countdown')
        self._goal_handle = None
        self._cancel_timer = None

    def run(self, seconds, cancel_after_s=None):
        """
        发送目标并等待结束。

        :param seconds: 倒计时秒数
        :param cancel_after_s: 发送后多少秒请求取消；None 表示不取消
        :return: 退出码，0 表示目标成功或按要求取消成功
        """
        log = self.get_logger()
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
            log.error(f'{SERVER_WAIT_TIMEOUT_S} 秒内没有等到动作服务端，请先启动 countdown_server')
            return 1

        # 第一步：发送目标，等待服务端决定接受还是拒绝
        goal = Countdown.Goal(seconds=seconds)
        log.info(f'发送目标：倒计时 {seconds} 秒')
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            log.error('目标被服务端拒绝')
            return 1
        log.info('目标已被接受')

        # 需要取消时，创建定时器，到时间后发送取消请求
        if cancel_after_s is not None:
            self._cancel_timer = self.create_timer(cancel_after_s, self._on_cancel_timer)

        # 第二步：等待最终结果
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        status_name = STATUS_NAMES.get(wrapped.status, f'未知状态 {wrapped.status}')
        log.info(f'最终状态：{status_name}，completed={wrapped.result.completed}，'
                 f'remaining={wrapped.result.remaining}')

        expected = (GoalStatus.STATUS_SUCCEEDED if cancel_after_s is None
                    else GoalStatus.STATUS_CANCELED)
        return 0 if wrapped.status == expected else 1

    def _on_feedback(self, feedback_msg):
        """反馈回调：打印剩余秒数。"""
        self.get_logger().info(f'反馈：剩余 {feedback_msg.feedback.remaining} 秒')

    def _on_cancel_timer(self):
        """取消定时器回调：只触发一次，发送取消请求。"""
        self._cancel_timer.cancel()
        self.get_logger().info('请求取消目标')
        self._goal_handle.cancel_goal_async()


def parse_args(argv):
    """
    解析命令行参数（不含 ROS 参数）。

    :param argv: 命令行参数列表，不含程序名
    :return: argparse.Namespace，包含 seconds、cancel_after
    """
    parser = argparse.ArgumentParser(prog='countdown_client', description='发送倒计时目标')
    parser.add_argument('seconds', type=int, help='倒计时秒数')
    parser.add_argument('--cancel-after', type=float, default=None, metavar='秒',
                        help='发送目标后多少秒请求取消')
    args = parser.parse_args(argv)
    if args.cancel_after is not None and args.cancel_after <= 0:
        parser.error('--cancel-after 必须大于 0')
    return args


def main(args=None):
    """节点入口：解析参数，执行一次倒计时后退出。"""
    options = parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=args)
    node = CountdownClient()
    exit_code = 1
    try:
        exit_code = node.run(options.seconds, options.cancel_after)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
