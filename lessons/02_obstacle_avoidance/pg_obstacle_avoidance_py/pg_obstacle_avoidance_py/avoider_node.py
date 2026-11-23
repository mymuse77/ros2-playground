"""
02 课 · 避障节点：订阅激光雷达，按固定频率计算并发布速度指令。

数据流：
  /scan (LaserScan) ──▶ 扇区分析 ──▶ 状态机 ──▶ /cmd_vel (TwistStamped)
                                          └──▶ /avoider/sectors (MarkerArray，给 RViz 看)
                                          └──▶ /avoider/state   (String，当前状态)

节点只负责 ROS 的收发和参数管理，算法全部在 logic.py 里。

安全措施：
  - 雷达数据超时（scan_timeout）：立即停车，等数据恢复后再继续
  - 节点退出（Ctrl+C 或 launch 关闭）：先发零速度并等待对方确认收到，再退出，
    否则小车会保持最后的速度继续走
  - 参数修改先整体校验，不合法的组合直接拒绝，控制器始终使用合法参数

两种时钟：
  - 稳定时钟（STEADY_TIME，系统单调时钟）：驱动控制定时器、判断雷达超时。
    它是"看门狗"，必须一直走：即使仿真卡死、/clock 停止，也要能发现超时并停车
  - ROS 时钟（use_sim_time=true 时是仿真时钟）：给状态机计时（转向超时），
    让小车的行为跟随仿真速度，仿真慢放时转向时间也同比例变长

知识点：
  - 传感器 QoS：qos_profile_sensor_data（best effort），与仿真/真机雷达驱动匹配
  - 控制与感知解耦：收到雷达只保存结果，由定时器按固定频率输出控制指令
  - 私有话题名 '~/sectors'：自动展开为 /<节点名>/sectors
  - 只读参数：control_rate_hz 只能在启动时设置

运行（需要先启动仿真）：
  ros2 run pg_obstacle_avoidance_py avoider --ros-args -p use_sim_time:=true
"""

import signal

from geometry_msgs.msg import TwistStamped
from pg_obstacle_avoidance_py.logic import (AvoidanceController, AvoidanceParams,
                                            from_ros_params, is_stale, STOP, summarize_scan,
                                            to_ros_params)
from pg_obstacle_avoidance_py.markers import build_markers
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

# 避障参数的说明（ROS 参数名 → 说明），与 logic.AvoidanceParams 的字段一一对应
PARAM_DESCRIPTIONS = {
    'max_linear_speed': '前方开阔时的前进速度，m/s',
    'min_linear_speed': '接近障碍时的最低前进速度，m/s',
    'turn_speed': '原地转向的角速度，rad/s；也是前进时转向角速度的上限',
    'steer_gain': '前进时躲避侧方障碍的转向增益，(rad/s)/m',
    'stop_distance': '前方小于此距离时停止前进、原地转向，m',
    'slow_distance': '前方小于此距离时开始减速，m',
    'clear_distance': '转向中前方大于此距离时恢复前进，m（需大于 stop_distance）',
    'side_distance': '侧方小于此距离时前进中向另一侧偏转，m',
    'max_turn_time': '同一方向转向超过此时间仍未脱困则换向，s',
    'front_half_angle_deg': '前方扇区的半角，度',
    'side_angle_deg': '侧方扇区的外边界，度',
}

DEFAULT_CONTROL_RATE_HZ = 10.0
DEFAULT_SCAN_TIMEOUT = 0.5
STOP_ACK_TIMEOUT = Duration(seconds=1.0)   # 退出时等待停车指令被确认的最长时间
SPIN_TIMEOUT_S = 0.1                       # 主循环每次等待事件的最长时间


class Avoider(Node):
    """基于激光雷达的避障漫游节点。"""

    def __init__(self):
        """声明参数，创建订阅、发布和控制定时器。"""
        super().__init__('avoider')

        # ---------- 参数 ----------
        # 避障参数：名字和默认值都从 AvoidanceParams 生成，保证与算法一致
        for name, default in to_ros_params(AvoidanceParams()).items():
            self.declare_parameter(
                name, default, ParameterDescriptor(description=PARAM_DESCRIPTIONS[name]))
        self.declare_parameter(
            'scan_timeout', DEFAULT_SCAN_TIMEOUT,
            ParameterDescriptor(description='雷达数据超过此时间未更新则停车，s'))
        self.declare_parameter(
            'control_rate_hz', DEFAULT_CONTROL_RATE_HZ,
            ParameterDescriptor(description='控制频率，Hz（只能在启动时设置）', read_only=True))

        self._controller = AvoidanceController(from_ros_params(self._avoidance_param_values()))
        self._scan_timeout = self.get_parameter('scan_timeout').value
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)

        # ---------- 感知状态（由雷达回调更新，由控制定时器读取） ----------
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._summary = None          # 最新一帧的扇区距离
        self._scan_frame = ''         # 雷达坐标系，标记画在这个坐标系里
        self._last_scan_time = None   # 最近一次收到雷达的时间（稳定时钟），s
        self._stale_reported = False  # 是否已经报告过"雷达超时"，避免刷屏
        self._last_state = None       # 上一个控制周期的状态，用于打印状态切换

        # ---------- 话题 ----------
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/sectors', 10)
        self._state_pub = self.create_publisher(String, '~/state', 10)
        # 雷达驱动通常用 best effort 发布，订阅端用 sensor_data 配置才能匹配
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        # 控制定时器用稳定时钟驱动：仿真卡住时它照样运行，才能发现雷达超时
        rate_hz = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate_hz, self._on_control, clock=self._steady_clock)
        self.get_logger().info(f'避障节点已启动：控制频率 {rate_hz} Hz，等待雷达数据……')

    # ------------------------------------------------------------------ 回调
    def _on_scan(self, scan):
        """雷达回调：只做扇区分析并保存结果，速度指令由控制定时器统一发出。"""
        params = self._controller.params
        self._summary = summarize_scan(scan, params.front_half_angle, params.side_angle)
        self._scan_frame = scan.header.frame_id
        self._last_scan_time = _seconds(self._steady_clock)

    def _on_control(self):
        """控制定时器回调：检查数据是否新鲜，运行状态机，发布速度和可视化。"""
        if is_stale(self._last_scan_time, _seconds(self._steady_clock), self._scan_timeout):
            if not self._stale_reported and self._last_scan_time is not None:
                self.get_logger().warning('雷达数据超时，停车等待')
            self._stale_reported = True
            self._controller.reset()
            self._last_state = None
            self._publish_command(STOP)
            return
        if self._stale_reported:
            self.get_logger().info('收到雷达数据，开始避障漫游')
            self._stale_reported = False

        # 状态机用 ROS 时钟计时，跟随仿真速度
        command = self._controller.update(self._summary, _seconds(self.get_clock()))
        self._publish_command(command)
        self._report_state()

    # ------------------------------------------------------------------ 参数
    def _validate_parameters(self, parameters):
        """修改前检查：把新值与其余当前值合在一起，整体构造一次 AvoidanceParams 验证约束。"""
        values = self._avoidance_param_values()
        for param in parameters:
            if param.name in values:
                values[param.name] = param.value
            elif param.name == 'scan_timeout' and param.value <= 0.0:
                return SetParametersResult(successful=False, reason='scan_timeout 必须大于 0')
        try:
            from_ros_params(values)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _apply_parameters(self, parameters):
        """修改成功后：用最新参数替换控制器的参数，下一个控制周期立即生效。"""
        self._controller.params = from_ros_params(self._avoidance_param_values())
        self._scan_timeout = self.get_parameter('scan_timeout').value
        changed = ', '.join(f'{p.name}={p.value}' for p in parameters)
        self.get_logger().info(f'参数已更新：{changed}')

    def _avoidance_param_values(self):
        """读取全部避障参数的当前值，返回 dict（ROS 参数名 → 数值）。"""
        return {name: self.get_parameter(name).value for name in PARAM_DESCRIPTIONS}

    # ------------------------------------------------------------------ 输出
    def _publish_command(self, command):
        """把 logic.Command 转换成 TwistStamped 并发布。"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = command.linear
        msg.twist.angular.z = command.angular
        self._cmd_pub.publish(msg)

    def _report_state(self):
        """发布状态和扇区标记；状态切换时打印日志。"""
        state = self._controller.state
        s = self._summary
        detail = f'前 {s.front:.2f} m，左前 {s.front_left:.2f} m，右前 {s.front_right:.2f} m'
        if state is not self._last_state:
            direction = {1: '，向左转', -1: '，向右转'}.get(self._controller.turn_direction, '')
            self.get_logger().info(f'状态 → {state.value}{direction}（{detail}）')
            self._last_state = state

        self._state_pub.publish(String(data=state.value))
        self._marker_pub.publish(build_markers(
            s, self._controller.params, state.value, self._scan_frame,
            self.get_clock().now().to_msg()))

    def stop(self):
        """
        发布零速度，并等待订阅者确认收到（最多 STOP_ACK_TIMEOUT）。节点退出前调用。

        :return: 是否在超时前得到全部订阅者的确认
        """
        self._publish_command(STOP)
        return self._cmd_pub.wait_for_all_acked(STOP_ACK_TIMEOUT)


def _seconds(clock):
    """读取时钟的当前时间，单位 s。"""
    return clock.now().nanoseconds * 1e-9


def _raise_keyboard_interrupt(signum, frame):
    """把 SIGTERM 转换成 KeyboardInterrupt，走与 Ctrl+C 相同的退出流程。"""
    raise KeyboardInterrupt


def main(args=None):
    """
    节点入口。

    与 01 课有两点不同：
      1. 关闭 rclpy 默认的信号处理。默认处理在收到 Ctrl+C 时会先关闭 ROS 上下文，
         那之后就发不出"停车"指令了。改为由 Python 抛出 KeyboardInterrupt，
         按"发零速度 → 销毁节点 → 关闭 ROS"的顺序退出
      2. 用 spin_once 循环代替 spin。Python 的信号处理函数只能在执行 Python 代码时运行，
         spin() 会一直阻塞在 C 代码里，没有事件到来时就收不到 Ctrl+C；
         spin_once 每 SPIN_TIMEOUT_S 秒至少返回一次，保证能及时响应
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # 显式安装两个信号的处理函数：
    #   SIGINT（Ctrl+C）：从后台启动的进程可能继承了"忽略 SIGINT"，这时 Python 不会
    #                     自动安装默认处理，所以这里明确装上
    #   SIGTERM（kill 命令、launch 超时强制结束）：也走同样的退出流程
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = Avoider()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            acked = node.stop()
            node.get_logger().info(
                '已发送停车指令' + ('，节点退出' if acked else '（未收到确认），节点退出'))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
