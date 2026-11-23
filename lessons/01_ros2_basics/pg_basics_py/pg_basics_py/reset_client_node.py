"""
01 课 · 服务客户端：调用 greeter 的 reset_counter 服务，然后退出。

知识点：
  - 客户端：create_client(服务类型, 服务名)
  - wait_for_service()：等待服务端上线，避免请求发给"空气"
  - call_async() + spin_until_future_complete()：异步发请求，等待响应期间继续处理 ROS 事件。
    不要在回调里用同步的 call()，那样会卡住执行器、造成死锁
  - rclpy.utilities.remove_ros_args()：从命令行参数里去掉 --ros-args 部分，只留下程序自己的参数

运行（先启动 greeter）：
  ros2 run pg_basics_py reset_client          # 重置为 0
  ros2 run pg_basics_py reset_client 100      # 重置为 100
"""

import sys

from pg_basics_py.counter import UINT32_MAX
from pg_interfaces.srv import ResetCounter
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

SERVICE_WAIT_TIMEOUT_S = 5.0
RESPONSE_TIMEOUT_S = 5.0


class ResetClient(Node):
    """reset_counter 服务的客户端。"""

    def __init__(self):
        """创建客户端。"""
        super().__init__('reset_client')
        self._client = self.create_client(ResetCounter, 'reset_counter')

    def reset(self, start_value):
        """
        发送重置请求并等待响应。

        :param start_value: 新的计数器起始值
        :return: ResetCounter.Response；服务不在线或超时则返回 None
        """
        if not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f'{SERVICE_WAIT_TIMEOUT_S} 秒内没有等到服务 {self._client.srv_name}，'
                '请先启动 greeter')
            return None

        request = ResetCounter.Request()
        request.start_value = start_value

        # call_async 立即返回一个 Future；spin_until_future_complete 一边处理事件一边等它完成
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=RESPONSE_TIMEOUT_S)
        if not future.done():
            self.get_logger().error(f'{RESPONSE_TIMEOUT_S} 秒内没有收到响应')
            return None
        return future.result()


def parse_start_value(argv):
    """
    从命令行参数里解析起始值。

    :param argv: 已去掉 ROS 参数的命令行参数列表，argv[0] 是程序名
    :return: 起始值；没有提供则为 0
    :raises ValueError: 参数不是 0 ~ UINT32_MAX 之间的整数
    """
    if len(argv) < 2:
        return 0
    value = int(argv[1])
    if not 0 <= value <= UINT32_MAX:
        raise ValueError(f'起始值必须在 0 ~ {UINT32_MAX} 之间：{value}')
    return value


def main(args=None):
    """节点入口：解析参数，调用一次服务，打印结果后退出。"""
    try:
        start_value = parse_start_value(remove_ros_args(args=sys.argv))
    except ValueError as error:
        print(f'参数错误：{error}\n用法：ros2 run pg_basics_py reset_client [起始值]')
        sys.exit(2)

    rclpy.init(args=args)
    node = ResetClient()
    exit_code = 1
    try:
        response = node.reset(start_value)
        if response is not None:
            log = node.get_logger()
            (log.info if response.success else log.error)(response.message)
            exit_code = 0 if response.success else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
