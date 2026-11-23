"""
01 课 · 话题订阅者：接收问候消息，检查是否丢消息，并计算传输延迟。

知识点：
  - 订阅者：create_subscription(消息类型, 话题名, 回调函数, QoS 队列深度)
  - 回调函数在 rclpy.spin() 里被调用，每收到一条消息调用一次
  - 时间计算：rclpy.time.Time.from_msg() 把消息里的时间戳转换成可以相减的 Time 对象

运行（先启动 greeter）：
  ros2 run pg_basics_py listener
观察节点连接关系：
  rqt_graph
"""

from pg_basics_py.counter import check_sequence, SEQ_GAP, SEQ_RESET
from pg_interfaces.msg import Greeting
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time


class Listener(Node):
    """订阅 Greeting 消息并打印的节点。"""

    def __init__(self):
        """创建订阅者。"""
        super().__init__('listener')

        # 上一条消息的序号，用来判断消息是否连续；None 表示还没收到过
        self._last_count = None

        # 话题名、消息类型都要和发布者一致，否则收不到。QoS 队列深度 10
        self._subscription = self.create_subscription(
            Greeting, 'greeting', self._on_greeting, 10)

        self.get_logger().info(f'开始订阅话题 {self._subscription.topic_name}')

    def _on_greeting(self, msg):
        """订阅回调：打印消息内容、序号检查结果和传输延迟。"""
        # 延迟 = 现在的时间 - 消息里的时间戳。两者都是 ROS 时间，可以直接相减
        delay_ms = (self.get_clock().now() - Time.from_msg(msg.stamp)).nanoseconds / 1e6

        status, missed = check_sequence(self._last_count, msg.count)
        self._last_count = msg.count

        log = self.get_logger()
        if status == SEQ_GAP:
            log.warning(f'丢失了 {missed} 条消息（序号跳到 #{msg.count}）')
        elif status == SEQ_RESET:
            log.warning(f'发布端计数器被重置，从 #{msg.count} 重新开始')
        log.info(f'收到 #{msg.count}: {msg.text}（延迟 {delay_ms:.2f} ms）')


def main(args=None):
    """节点入口：初始化 ROS，运行节点直到按 Ctrl+C。"""
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
