"""
03 课 · 节点公用的 ROS 工具：消息转换、"退出前停车"的主循环。

02 课的避障节点里已经讲过退出停车的原理，这里把它整理成可复用的函数，
本课的几个控制节点都用它。
"""

import signal

from geometry_msgs.msg import TwistStamped
from pg_go_to_goal_py.geometry import Pose2D, yaw_from_quaternion
from pg_go_to_goal_py.go_to_goal import STOP
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

SPIN_TIMEOUT_S = 0.1
STOP_ACK_TIMEOUT = Duration(seconds=1.0)


def pose_from_odometry(msg):
    """把 nav_msgs/msg/Odometry 转换成 Pose2D（odom 坐标系）。"""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


def make_twist_stamped(command, stamp, frame_id='base_footprint'):
    """
    把速度指令转换成 TwistStamped（TurtleBot3 Jazzy 版 /cmd_vel 的类型）。

    :param command: 有 linear、angular 属性的对象（m/s、rad/s）
    :param stamp: 时间戳 builtin_interfaces/Time
    """
    msg = TwistStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.linear.x = float(command.linear)
    msg.twist.angular.z = float(command.angular)
    return msg


def publish_stop(node, publisher):
    """发布零速度并等待订阅者确认，返回是否得到确认。"""
    publisher.publish(make_twist_stamped(STOP, node.get_clock().now().to_msg()))
    return publisher.wait_for_all_acked(STOP_ACK_TIMEOUT)


def _raise_keyboard_interrupt(signum, frame):
    """把 SIGTERM 转换成 KeyboardInterrupt。"""
    raise KeyboardInterrupt


def init_with_safe_shutdown(args=None):
    """
    初始化 rclpy，并接管 SIGINT/SIGTERM，使节点退出前还能发布消息。

    原理见 02 课 avoider_node.py 的 main()。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)


def spin_until_shutdown(node, executor=None, on_exit=None):
    """
    运行节点直到 Ctrl+C / SIGTERM，然后调用 on_exit（例如停车），再销毁节点、关闭 ROS。

    :param node: 节点
    :param executor: 执行器；None 表示单线程执行
    :param on_exit: 退出前调用的函数，此时 ROS 仍可用
    """
    try:
        while rclpy.ok():
            if executor is None:
                rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
            else:
                executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if on_exit is not None and rclpy.ok():
            on_exit()
        if executor is not None:
            executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
