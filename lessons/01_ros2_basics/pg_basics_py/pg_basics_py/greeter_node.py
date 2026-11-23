"""
01 课 · greeter 节点：周期性发布问候消息，并提供"重置计数器"服务。

知识点：
  - 节点：继承 rclpy.node.Node，节点名在 ROS 网络里唯一标识这个程序
  - 发布者：create_publisher(消息类型, 话题名, QoS 队列深度)
  - 定时器：create_timer(周期秒数, 回调函数)，回调在 rclpy.spin() 里被调用
  - 服务端：create_service(服务类型, 服务名, 回调函数)，回调收到请求、填写并返回响应
  - 自定义接口：pg_interfaces/msg/Greeting、pg_interfaces/srv/ResetCounter

运行：
  ros2 run pg_basics_py greeter
观察：
  ros2 topic echo /greeting
  ros2 topic hz /greeting
  ros2 service call /reset_counter pg_interfaces/srv/ResetCounter "{start_value: 100}"
"""

from pg_basics_py.counter import Counter
from pg_interfaces.msg import Greeting
from pg_interfaces.srv import ResetCounter
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

GREETING_TEXT = 'Hello ROS 2'
PUBLISH_PERIOD_S = 1.0


class Greeter(Node):
    """周期性发布 Greeting 消息、并可通过服务重置计数器的节点。"""

    def __init__(self):
        """创建发布者、定时器和服务端。"""
        super().__init__('greeter')

        self._counter = Counter(start=0)

        # 话题名用相对名称 'greeting'（不以 / 开头），这样以后可以通过命名空间或重映射改名。
        # 第三个参数 10 是 QoS 的队列深度：发得比收得快时，最多缓存 10 条未处理的消息
        self._publisher = self.create_publisher(Greeting, 'greeting', 10)

        # 定时器：每隔 PUBLISH_PERIOD_S 秒调用一次 _on_timer
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self._on_timer)

        # 服务端：收到 reset_counter 请求时调用 _on_reset_counter。
        # 和话题不同，服务是"一问一答"：客户端发请求，服务端处理完返回一个响应
        self._reset_service = self.create_service(
            ResetCounter, 'reset_counter', self._on_reset_counter)

        self.get_logger().info(
            f'开始在话题 {self._publisher.topic_name} 上发布，周期 {PUBLISH_PERIOD_S} 秒；'
            f'服务 {self._reset_service.srv_name} 已就绪')

    def _on_timer(self):
        """定时器回调：组装一条消息并发布。"""
        msg = Greeting()
        # 时间戳用节点的时钟（使用仿真时间时会自动换成仿真时钟）
        msg.stamp = self.get_clock().now().to_msg()
        msg.text = GREETING_TEXT
        msg.count = self._counter.take()

        self._publisher.publish(msg)
        self.get_logger().info(f'发布 #{msg.count}: {msg.text}')

    def _on_reset_counter(self, request, response):
        """
        服务回调：把计数器重置为请求中的 start_value。

        :param request: ResetCounter.Request，包含 start_value
        :param response: ResetCounter.Response，由 rclpy 预先创建，填写后返回即可
        :return: 填好的 response
        """
        try:
            response.previous_value = self._counter.reset(request.start_value)
        except ValueError as error:
            # uint32 字段本身就限制了范围，这里通常不会出错；
            # 保留这个分支是为了演示：服务端出错时应当返回失败原因，而不是让节点崩溃
            response.success = False
            response.message = str(error)
            self.get_logger().error(f'重置失败：{error}')
            return response

        response.success = True
        response.message = (
            f'计数器已从 {response.previous_value} 重置为 {request.start_value}')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    """节点入口：初始化 ROS，运行节点直到按 Ctrl+C。"""
    rclpy.init(args=args)
    node = Greeter()
    try:
        # spin：进入事件循环，处理定时器、订阅、服务等回调，直到节点被关闭
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 或 launch 关闭节点时会走到这里，属于正常退出
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
