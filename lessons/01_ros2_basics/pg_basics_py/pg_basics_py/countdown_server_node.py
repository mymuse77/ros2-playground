"""
01 课 · 动作服务端：倒计时，每秒反馈剩余秒数，可以中途取消。

动作（action）适合"耗时较长、需要看到进度、可能中途取消"的任务，
比如导航到某个点、机械臂抓取。它由三部分组成：
  目标（goal）     客户端发来的任务，这里是倒计时的秒数
  反馈（feedback） 执行过程中持续发回的进度，这里是剩余秒数
  结果（result）   任务结束时返回一次，这里是否完成、还剩几秒

知识点：
  - ActionServer(节点, 动作类型, 动作名, execute_callback, goal_callback, cancel_callback)
  - goal_callback：决定接受还是拒绝新目标（这里拒绝非法秒数，以及在已有任务时拒绝新任务）
  - cancel_callback：决定是否接受取消请求
  - execute_callback：真正执行任务；结束时必须调用 succeed()/canceled()/abort() 之一
  - MultiThreadedExecutor + ReentrantCallbackGroup：执行回调运行期间，
    其他线程仍能处理新目标和取消请求。用单线程执行器的话，倒计时期间节点会"卡住"

运行：
  ros2 run pg_basics_py countdown_server
试一试：
  ros2 action send_goal --feedback /countdown pg_interfaces/action/Countdown "{seconds: 5}"
  ros2 run pg_basics_py countdown_client 10 --cancel-after 3
"""

import threading

from pg_interfaces.action import Countdown
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

# 倒计时秒数的合法范围（与 Countdown.action 中的注释一致）
MIN_SECONDS = 1
MAX_SECONDS = 3600
TICK_PERIOD_S = 1.0


def is_valid_seconds(seconds):
    """判断倒计时秒数是否在 MIN_SECONDS ~ MAX_SECONDS 之间。"""
    return MIN_SECONDS <= seconds <= MAX_SECONDS


class CountdownServer(Node):
    """倒计时动作服务端，同一时间只执行一个目标。"""

    def __init__(self):
        """创建动作服务端。"""
        super().__init__('countdown_server')

        # 是否有正在执行的目标。goal_callback 和 execute_callback 可能在不同线程，
        # 所以读写这个标志时要加锁
        self._busy = False
        self._lock = threading.Lock()

        # 可重入回调组：允许这个组里的回调在多个线程中并行执行
        self._action_server = ActionServer(
            self, Countdown, 'countdown',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('倒计时动作服务端已就绪：countdown')

    def _on_goal(self, goal_request):
        """
        收到新目标时调用：决定接受还是拒绝。

        :param goal_request: Countdown.Goal，包含 seconds
        :return: GoalResponse.ACCEPT 或 GoalResponse.REJECT
        """
        seconds = goal_request.seconds
        if not is_valid_seconds(seconds):
            self.get_logger().warning(
                f'拒绝目标：秒数 {seconds} 不在 {MIN_SECONDS} ~ {MAX_SECONDS} 之间')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warning('拒绝目标：已有倒计时在执行')
                return GoalResponse.REJECT
            # 接受时立即占用，避免两个目标同时通过检查
            self._busy = True
        self.get_logger().info(f'接受目标：倒计时 {seconds} 秒')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """收到取消请求时调用：这里总是同意取消。"""
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """
        执行倒计时：每秒发布一次反馈，期间检查是否被取消。

        :param goal_handle: 目标句柄，用来读取目标、发布反馈、报告最终状态
        :return: Countdown.Result
        """
        try:
            remaining = goal_handle.request.seconds
            feedback = Countdown.Feedback()

            while remaining > 0:
                # 取消请求被接受后，is_cancel_requested 变为 True。
                # 检查间隔是 1 秒，所以取消最多延迟 1 秒生效
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info(f'倒计时已取消，剩余 {remaining} 秒')
                    return Countdown.Result(completed=False, remaining=remaining)

                feedback.remaining = remaining
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f'剩余 {remaining} 秒')

                # 用节点时钟睡眠：使用仿真时间时会跟随仿真时钟。
                # 返回 False 表示被提前唤醒（例如节点正在关闭）
                if not self.get_clock().sleep_for(Duration(seconds=TICK_PERIOD_S)):
                    goal_handle.abort()
                    return Countdown.Result(completed=False, remaining=remaining)
                remaining -= 1

            goal_handle.succeed()
            self.get_logger().info('倒计时完成')
            return Countdown.Result(completed=True, remaining=0)
        finally:
            # 无论成功、取消还是出错，都要释放占用，否则之后的目标会一直被拒绝
            with self._lock:
                self._busy = False


def main(args=None):
    """节点入口：用多线程执行器运行节点，直到按 Ctrl+C。"""
    rclpy.init(args=args)
    node = CountdownServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
