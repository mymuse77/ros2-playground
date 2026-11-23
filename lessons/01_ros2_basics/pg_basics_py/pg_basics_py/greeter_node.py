"""
01 课 · greeter 节点：周期性发布问候消息，提供"重置计数器"服务，参数可在运行时修改。

知识点：
  - 节点：继承 rclpy.node.Node，节点名在 ROS 网络里唯一标识这个程序
  - 发布者：create_publisher(消息类型, 话题名, QoS 队列深度)
  - 定时器：create_timer(周期秒数, 回调函数)，回调在 rclpy.spin() 里被调用
  - 服务端：create_service(服务类型, 服务名, 回调函数)，回调收到请求、填写并返回响应
  - 参数：declare_parameter(名字, 默认值, 描述符)。描述符可以写说明、限制取值范围
  - 参数回调（两种）：
      add_on_set_parameters_callback       修改"之前"调用，可以拒绝这次修改
      add_post_set_parameters_callback     修改"成功之后"调用，用来让新值立即生效
  - 自定义接口：pg_interfaces/msg/Greeting、pg_interfaces/srv/ResetCounter

运行：
  ros2 run pg_basics_py greeter
  ros2 run pg_basics_py greeter --ros-args -p publish_rate_hz:=5.0 -p greeting_text:=Hi
观察：
  ros2 topic echo /greeting
  ros2 topic hz /greeting
  ros2 service call /reset_counter pg_interfaces/srv/ResetCounter "{start_value: 100}"
  ros2 param describe /greeter publish_rate_hz
  ros2 param set /greeter publish_rate_hz 5.0      # 立即生效，用 topic hz 观察
  ros2 param set /greeter publish_rate_hz 100.0    # 超出范围，被拒绝
  ros2 param set /greeter greeting_text "''"       # 空字符串，被拒绝
"""

from pg_basics_py.counter import Counter
from pg_interfaces.msg import Greeting
from pg_interfaces.srv import ResetCounter
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# 参数默认值与取值范围
DEFAULT_PUBLISH_RATE_HZ = 1.0
MIN_PUBLISH_RATE_HZ = 0.1
MAX_PUBLISH_RATE_HZ = 50.0
DEFAULT_GREETING_TEXT = 'Hello ROS 2'


class Greeter(Node):
    """周期性发布 Greeting 消息的节点，支持服务重置计数器、运行时修改参数。"""

    def __init__(self):
        """声明参数，创建发布者、定时器和服务端。"""
        super().__init__('greeter')

        self._counter = Counter(start=0)

        # ---------- 参数 ----------
        # 参数必须先声明才能使用。描述符里的 floating_point_range 会被 ROS 自动检查：
        # 超出范围的修改直接被拒绝，不会进入我们的回调
        self.declare_parameter(
            'publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ,
            ParameterDescriptor(
                description='问候消息的发布频率，单位 Hz',
                floating_point_range=[FloatingPointRange(
                    from_value=MIN_PUBLISH_RATE_HZ, to_value=MAX_PUBLISH_RATE_HZ)]))
        self.declare_parameter(
            'greeting_text', DEFAULT_GREETING_TEXT,
            ParameterDescriptor(description='问候消息的文字内容，不能为空'))

        # 把参数值缓存到成员变量，定时器回调里直接用，不必每次都查询参数
        rate_hz = self.get_parameter('publish_rate_hz').value
        self._greeting_text = self.get_parameter('greeting_text').value

        # 修改前检查（可以拒绝）、修改后生效，分成两个回调，职责清晰
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)

        # ---------- 话题 ----------
        # 话题名用相对名称 'greeting'（不以 / 开头），这样以后可以通过命名空间或重映射改名。
        # 第三个参数 10 是 QoS 的队列深度：发得比收得快时，最多缓存 10 条未处理的消息
        self._publisher = self.create_publisher(Greeting, 'greeting', 10)

        # 定时器：每隔 1/rate_hz 秒调用一次 _on_timer
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        # ---------- 服务 ----------
        # 收到 reset_counter 请求时调用 _on_reset_counter。
        # 和话题不同，服务是"一问一答"：客户端发请求，服务端处理完返回一个响应
        self._reset_service = self.create_service(
            ResetCounter, 'reset_counter', self._on_reset_counter)

        self.get_logger().info(
            f'开始在话题 {self._publisher.topic_name} 上发布，频率 {rate_hz} Hz；'
            f'服务 {self._reset_service.srv_name} 已就绪')

    # ------------------------------------------------------------------ 话题
    def _on_timer(self):
        """定时器回调：组装一条消息并发布。"""
        msg = Greeting()
        # 时间戳用节点的时钟（使用仿真时间时会自动换成仿真时钟）
        msg.stamp = self.get_clock().now().to_msg()
        msg.text = self._greeting_text
        msg.count = self._counter.take()

        self._publisher.publish(msg)
        self.get_logger().info(f'发布 #{msg.count}: {msg.text}')

    # ------------------------------------------------------------------ 服务
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

    # ------------------------------------------------------------------ 参数
    def _validate_parameters(self, parameters):
        """
        修改前检查：只做判断，不改变节点状态。

        范围检查已由描述符完成，这里补充描述符表达不了的规则。

        :param parameters: 本次要修改的参数列表（rclpy.parameter.Parameter）
        :return: SetParametersResult，successful=False 时整批修改都会被拒绝
        """
        for param in parameters:
            if param.name == 'greeting_text' and not param.value.strip():
                return SetParametersResult(successful=False, reason='greeting_text 不能为空')
        return SetParametersResult(successful=True)

    def _apply_parameters(self, parameters):
        """
        修改成功后调用：让新值立即生效。

        :param parameters: 已经修改成功的参数列表
        """
        for param in parameters:
            if param.name == 'publish_rate_hz':
                # 定时器的周期创建后不能直接改，所以销毁旧定时器、按新频率重建
                self.destroy_timer(self._timer)
                self._timer = self.create_timer(1.0 / param.value, self._on_timer)
                self.get_logger().info(f'发布频率已改为 {param.value} Hz')
            elif param.name == 'greeting_text':
                self._greeting_text = param.value
                self.get_logger().info(f'问候语已改为 "{param.value}"')


def main(args=None):
    """节点入口：初始化 ROS，运行节点直到按 Ctrl+C。"""
    rclpy.init(args=args)
    node = Greeter()
    try:
        # spin：进入事件循环，处理定时器、订阅、服务、参数等回调，直到节点被关闭
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 或 launch 关闭节点时会走到这里，属于正常退出
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
