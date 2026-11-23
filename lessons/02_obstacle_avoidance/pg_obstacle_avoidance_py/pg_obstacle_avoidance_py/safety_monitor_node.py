"""
02 课 · 安全监视节点：统计漫游过程中的最小净空、接触次数、里程，用于仿真验收。

它和避障节点完全独立，只"旁观"：订阅 /scan 和 /odom，不发布任何控制指令。
自己写的算法，最好用另一套独立的检查来验证，而不是让算法自己报告"我没撞"。

  - 净空：障碍物到车身矩形（而不是雷达中心）的最短距离，见 safety.py
  - 接触：净空小于 contact_distance
  - 卡住：stuck_time 秒内移动不到 stuck_distance 米

设置 duration > 0 时，到时间后打印最终报告并退出，退出码 0 = 通过，1 = 不通过，
便于脚本自动验收。duration 按 ROS 时间计算（仿真中即仿真时间）。

运行（仿真和避障节点已启动）：
  ros2 run pg_obstacle_avoidance_py safety_monitor --ros-args -p use_sim_time:=true
验收 5 分钟（仿真时间）：
  ros2 run pg_obstacle_avoidance_py safety_monitor --ros-args -p use_sim_time:=true
      -p duration:=300.0       （与上一行写在同一行）
"""

import signal
import sys

from nav_msgs.msg import Odometry
from pg_obstacle_avoidance_py.safety import footprint_clearance, TripMonitor
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# TurtleBot3 waffle 的车身矩形（以雷达为中心），单位 m：
#   前后：车身约 0.266 m 长，半长 0.133，再留 7 mm 余量 → 0.14（车身 0.280 m）
#   左右：两侧轮子比车身略微突出，半宽取 0.155（车身 0.310 m）
# 余量让"净空"偏保守：报告的净空比真实的略小，不会把擦边算成没碰到
DEFAULT_HALF_LENGTH = 0.14
DEFAULT_HALF_WIDTH = 0.155

# 参数名 → (默认值, 说明)。只在启动时读取，声明为只读
PARAMS = {
    'half_length': (DEFAULT_HALF_LENGTH, '车身矩形半长（雷达到车头/车尾），m'),
    'half_width': (DEFAULT_HALF_WIDTH, '车身矩形半宽（雷达到左右两侧），m'),
    'contact_distance': (0.03, '净空小于此值记为一次接触，m'),
    'stuck_time': (30.0, '在这么长时间内移动不到 stuck_distance 记为卡住，s'),
    'stuck_distance': (0.10, '判断卡住的移动距离，m'),
    'report_period': (30.0, '多久打印一次阶段报告（ROS 时间），s'),
    'duration': (0.0, '验收时长（ROS 时间），s；0 表示一直运行'),
}


class SafetyMonitor(Node):
    """旁观式安全统计节点。"""

    def __init__(self):
        """声明参数，创建订阅和报告定时器。"""
        super().__init__('safety_monitor')
        values = {
            name: self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True)).value
            for name, (default, description) in PARAMS.items()}
        self._half_length = values['half_length']
        self._half_width = values['half_width']
        contact = values['contact_distance']
        stuck_time = values['stuck_time']
        stuck_distance = values['stuck_distance']
        report_period = values['report_period']
        self._duration = values['duration']

        self._monitor = TripMonitor(contact, stuck_time, stuck_distance)
        self.passed = None   # 到达 duration 后填入验收结果

        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(report_period, self._log_report)
        self.get_logger().info(
            f'安全监视已启动：车身 {2 * self._half_length:.3f} × {2 * self._half_width:.3f} m，'
            f'接触阈值 {contact} m' + (f'，验收时长 {self._duration} s' if self._duration else ''))

    def _on_scan(self, scan):
        """雷达回调：计算净空，记录最小值和接触。"""
        clearance = footprint_clearance(scan, self._half_length, self._half_width)
        self._monitor.update_clearance(clearance)

    def _on_odom(self, msg):
        """里程计回调：累加里程；到达验收时长时结束。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        self._monitor.update_position(now, p.x, p.y)
        if self._duration > 0.0 and self._monitor.report().elapsed >= self._duration:
            report = self._log_report(final=True)
            self.passed = report.passed
            raise SystemExit   # 跳出 spin，由 main() 处理退出码

    def _log_report(self, final=False):
        """打印当前统计，返回 TripReport。"""
        r = self._monitor.report()
        text = (f'用时 {r.elapsed:.0f} s，里程 {r.distance:.2f} m，最小净空 {r.min_clearance:.3f} m，'
                f'接触 {r.contacts} 次' + ('，已卡住！' if r.stuck else ''))
        if final:
            verdict = '通过' if r.passed else '不通过'
            self.get_logger().info(f'===== 验收{verdict}：{text} =====')
        else:
            self.get_logger().info(text)
        return r


def main(args=None):
    """节点入口：运行到 Ctrl+C，或到达 duration 后按验收结果设置退出码。"""
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except (KeyboardInterrupt, ExternalShutdownException):
        node._log_report(final=True)
    finally:
        # 在终端里对 ros2 launch 按 Ctrl+C 时，节点会收到两次 SIGINT：终端发给整个进程组一次，
        # launch 再转发一次。第二次如果落在下面的清理过程中，会打断 destroy_node()、
        # 打出一大段 KeyboardInterrupt。最终报告已经打印了，清理期间忽略 SIGINT
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        passed = node.passed
        node.destroy_node()
        rclpy.try_shutdown()
    if passed is not None:
        sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
