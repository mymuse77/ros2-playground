"""
06 课 · 颜色跟随节点：订阅相机图像，找到指定颜色的目标，控制小车追着它走。

数据流：
  /camera/image_raw (Image) ──▶ 缩小 ──▶ 颜色检测 ──▶ 方位角、距离 ──┐
  /camera/camera_info ──▶ 焦距 ───────────────────────────────────────┤
                                                                      ▼
                          /cmd_vel (TwistStamped) ◀── 跟随控制器（定时器驱动）
                          /color_follower/debug_image (Image，画了框的画面)
                          /color_follower/state       (String，当前状态)

节点只负责 ROS 的收发和参数管理，算法在 color_detector.py、camera_model.py、follower.py。

与 02 课（避障）相同的做法：
  - 图像回调只保存检测结果，由定时器按固定频率输出控制指令
  - 看门狗用稳定时钟：图像超时（相机卡住、仿真暂停）立即停车
  - 退出（Ctrl+C 或 launch 关闭）时先发零速度并等待确认，再退出
  - 参数修改先整体校验，不合法的组合直接拒绝
  - 传感器 QoS：qos_profile_sensor_data（best effort）

本课新增的知识点：
  - cv_bridge 与图像编码：Gazebo 相机发的是 rgb8（红绿蓝），OpenCV 用 bgr8（蓝绿红），
    不转换的话红色会被当成蓝色。先缩小再转换通道，只处理 1/36 的像素
  - 先缩小再检测：1920×1080 的图像每帧 6 MB，检测只需要目标的大致位置，缩到 320 宽足够
  - 焦距来自 camera_info：不把相机参数写死在代码里，换相机也能用
  - 调试图像：只有 rqt_image_view 等订阅者在看的时候才画和发布，没人看就不花这个计算量

运行（需要先启动仿真）：
  ros2 run pg_vision_py color_follower --ros-args -p use_sim_time:=true
"""

import signal
import threading

from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from pg_vision_py import follower, settings
from pg_vision_py.camera_model import observe, PinholeCamera
from pg_vision_py.color_detector import (convert_to_bgr, detect, draw_detection,
                                         resize_to_width)
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# 控制参数的说明（ROS 参数名 → 说明），与 follower.FollowerParams 的字段一一对应
FOLLOWER_DESCRIPTIONS = {
    'target_distance': '希望与目标保持的距离，m',
    'distance_tolerance': '距离死区，误差小于它就不动，m',
    'angle_tolerance_deg': '方位死区，偏差小于它就不转，度',
    'max_linear_speed': '最大前进速度，m/s',
    'max_angular_speed': '最大转向角速度，rad/s',
    'linear_gain': '距离误差 → 前进速度的比例系数，(m/s)/m',
    'angular_gain': '方位误差 → 转向角速度的比例系数，(rad/s)/rad',
    'turn_only_angle_deg': '方位偏差超过此角度时只转向不前进，度',
    'max_linear_accel': '线速度的最大增长率，m/s²',
    'lost_timeout': '看不到目标超过此时间后开始原地搜索，s',
    'search_speed': '搜索时的转圈角速度，rad/s；0 表示不搜索',
}

# 视觉设置的说明（ROS 参数名 → 说明），与 settings.VisionSettings 的字段一一对应
VISION_DESCRIPTIONS = {
    'color': '要跟随的颜色：red | orange | green | blue',
    'target_width': '目标的真实宽度，m；用来由像素宽度估算距离',
    'image_width': '检测前把图像缩小到这个宽度，像素',
    'min_area_ratio': '目标面积占整幅图像的最小比例，更小的当噪声',
    'image_timeout': '相机图像超过此时间没更新就停车，s',
    'publish_debug_image': '是否发布带标注的调试图像（仅在有订阅者时才处理）',
}

DEFAULT_CONTROL_RATE_HZ = 10.0
STOP_ACK_TIMEOUT = Duration(seconds=1.0)   # 退出时等待停车指令被确认的最长时间
SPIN_TIMEOUT_S = 0.1                       # 主循环每次等待事件的最长时间


class ColorFollower(Node):
    """颜色跟随节点。"""

    def __init__(self):
        """声明参数，创建订阅、发布和控制定时器。"""
        super().__init__('color_follower')

        # ---------- 参数 ----------
        # 名字和默认值都从两个数据类生成，保证与算法一致
        for name, default in follower.to_ros_params(follower.FollowerParams()).items():
            self.declare_parameter(
                name, default, ParameterDescriptor(description=FOLLOWER_DESCRIPTIONS[name]))
        for name, default in settings.to_ros_params(settings.VisionSettings()).items():
            self.declare_parameter(
                name, default, ParameterDescriptor(description=VISION_DESCRIPTIONS[name]))
        self.declare_parameter(
            'control_rate_hz', DEFAULT_CONTROL_RATE_HZ,
            ParameterDescriptor(description='控制频率，Hz（只能在启动时设置）', read_only=True))

        self._controller = follower.FollowController(
            follower.from_ros_params(self._values(FOLLOWER_DESCRIPTIONS)))
        self._settings = settings.from_ros_params(self._values(VISION_DESCRIPTIONS))
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)

        # ---------- 感知状态（由图像回调更新，由控制定时器读取） ----------
        self._bridge = CvBridge()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._camera = None            # 原始分辨率下的相机模型，收到 camera_info 后才有
        self._observation = None       # 最新一帧的观测；None 表示这一帧没看到目标
        self._last_image_time = None   # 最近一次收到图像的时间（稳定时钟），s
        self._stale_reported = False   # 是否已报告过"图像超时"，避免刷屏
        self._last_state = None        # 上一个控制周期的状态，用于打印状态切换

        # ---------- 话题 ----------
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        # 相机驱动通常用 best effort 发布，订阅端用 sensor_data 配置才能匹配
        self.create_subscription(
            CameraInfo, 'camera/camera_info', self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, 'camera/image_raw', self._on_image, qos_profile_sensor_data)

        # 控制定时器用稳定时钟驱动：仿真卡住时它照样运行，才能发现图像超时
        rate_hz = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate_hz, self._on_control, clock=self._steady_clock)
        self.get_logger().info(
            f'颜色跟随节点已启动：跟随{self._settings.color}色，控制频率 {rate_hz} Hz，等待相机数据……')

    # ------------------------------------------------------------------ 回调
    def _on_camera_info(self, msg):
        """相机内参回调：取焦距 fx（K 矩阵的第一个元素）建立相机模型。只在第一次生效。"""
        if self._camera is not None or msg.k[0] <= 0.0:
            return
        self._camera = PinholeCamera(fx=msg.k[0], image_width=msg.width)
        self.get_logger().info(f'相机内参：{msg.width}×{msg.height}，焦距 fx={msg.k[0]:.1f} 像素')

    def _on_image(self, msg):
        """图像回调：缩小、检测、换算成方位角和距离并保存，速度指令由控制定时器统一发出。"""
        if self._camera is None:
            return   # 没有焦距就没法估距离，等 camera_info
        cfg = self._settings
        # passthrough：不让 cv_bridge 转换，先缩小再转通道顺序（见模块说明）
        frame = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
        try:
            small = convert_to_bgr(resize_to_width(frame, cfg.image_width), msg.encoding)
        except ValueError as error:
            # 编码不支持：跳过这一帧。图像时间不更新，看门狗会让小车停下，不会带着旧结果乱走
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        detection = detect(small, cfg.hsv_range, cfg.min_area_ratio)

        self._observation = observe(detection, self._camera, cfg.target_width)
        self._last_image_time = _seconds(self._steady_clock)

        # 没人订阅调试图像就不画：画框和编码都有成本
        if cfg.publish_debug_image and self._debug_pub.get_subscription_count() > 0:
            text = 'no target' if detection is None else (
                f'bearing {self._observation.bearing:+.2f} rad  dist '
                + ('?' if self._observation.distance is None
                   else f'{self._observation.distance:.2f} m'))
            debug = self._bridge.cv2_to_imgmsg(draw_detection(small, detection, text), 'bgr8')
            debug.header = msg.header
            self._debug_pub.publish(debug)

    def _on_control(self):
        """控制定时器回调：检查图像是否新鲜，运行控制器，发布速度和状态。"""
        if follower.is_stale(self._last_image_time, _seconds(self._steady_clock),
                             self._settings.image_timeout):
            if not self._stale_reported and self._last_image_time is not None:
                self.get_logger().warning('相机图像超时，停车等待')
            self._stale_reported = True
            self._controller.reset()
            self._last_state = None
            self._publish_command(follower.STOP)
            return
        if self._stale_reported:
            self.get_logger().info('收到相机图像，开始跟随')
            self._stale_reported = False

        # 控制器用 ROS 时钟计时，跟随仿真速度
        command = self._controller.update(self._observation, _seconds(self.get_clock()))
        self._publish_command(command)
        self._report_state()

    # ------------------------------------------------------------------ 参数
    def _validate_parameters(self, parameters):
        """修改前检查：把新值与其余当前值合在一起，整体构造一次数据类来验证约束。"""
        follower_values = self._values(FOLLOWER_DESCRIPTIONS)
        vision_values = self._values(VISION_DESCRIPTIONS)
        for param in parameters:
            if param.name in follower_values:
                follower_values[param.name] = param.value
            elif param.name in vision_values:
                vision_values[param.name] = param.value
        try:
            follower.from_ros_params(follower_values)
            settings.from_ros_params(vision_values)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _apply_parameters(self, parameters):
        """修改成功后：用最新参数替换控制器和视觉设置，下一个周期立即生效。"""
        self._controller.params = follower.from_ros_params(self._values(FOLLOWER_DESCRIPTIONS))
        self._settings = settings.from_ros_params(self._values(VISION_DESCRIPTIONS))
        changed = ', '.join(f'{p.name}={p.value}' for p in parameters)
        self.get_logger().info(f'参数已更新：{changed}')

    def _values(self, descriptions):
        """读取 descriptions 里列出的全部参数的当前值，返回 dict（ROS 参数名 → 值）。"""
        return {name: self.get_parameter(name).value for name in descriptions}

    # ------------------------------------------------------------------ 输出
    def _publish_command(self, command):
        """把 follower.Command 转换成 TwistStamped 并发布。"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = command.linear
        msg.twist.angular.z = command.angular
        self._cmd_pub.publish(msg)

    def _report_state(self):
        """发布状态；状态切换时打印日志。"""
        state = self._controller.state
        if state is not self._last_state:
            obs = self._observation
            detail = '' if obs is None else (
                f'（方位 {obs.bearing:+.2f} rad，距离 '
                + ('未知' if obs.distance is None else f'{obs.distance:.2f} m') + '）')
            self.get_logger().info(f'状态 → {state.value}{detail}')
            self._last_state = state
        self._state_pub.publish(String(data=state.value))

    def stop(self):
        """
        发布零速度，并等待订阅者确认收到（最多 STOP_ACK_TIMEOUT）。节点退出前调用。

        :return: 是否在超时前得到全部订阅者的确认
        """
        self._publish_command(follower.STOP)
        return self._cmd_pub.wait_for_all_acked(STOP_ACK_TIMEOUT)


def _seconds(clock):
    """读取时钟的当前时间，单位 s。"""
    return clock.now().nanoseconds * 1e-9


_stop_requested = threading.Event()   # 收到退出信号后置位，主循环检查它


def _request_stop(signum, frame):
    """SIGINT（Ctrl+C）和 SIGTERM 的处理函数：只置标志位，由主循环自己退出。"""
    _stop_requested.set()


def main(args=None):
    """
    节点入口。退出流程与 02 课的避障节点相似，但用"标志位"代替"抛异常"：

      1. 关闭 rclpy 默认的信号处理。默认处理在收到 Ctrl+C 时会先关闭 ROS 上下文，
         之后就发不出停车指令了。改成"发零速度 → 销毁节点 → 关闭 ROS"的顺序
      2. 信号处理函数只置标志位，不抛异常。02 课的做法是抛 KeyboardInterrupt，
         但本节点每帧要把 6 MB 的图像消息转成 Python 对象，如果信号恰好在转换途中到来，
         异常会被底层库包装成 RuntimeError，绕过 except，节点异常退出、停车指令也发不出。
         置标志位在任何时刻都是安全的
      3. 用 spin_once 循环代替 spin，每 SPIN_TIMEOUT_S 秒检查一次标志位，退出延迟很短
      4. 显式安装 SIGINT 和 SIGTERM 的处理函数：后台启动的进程可能继承了"忽略 SIGINT"，
         这时 Python 不会自动安装默认处理。Ctrl+C 会被发两次（终端发给整个进程组，
         launch 再转发一次），处理函数只是置位，重复调用没有副作用
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    node = ColorFollower()
    try:
        while rclpy.ok() and not _stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    except ExternalShutdownException:
        pass   # ROS 上下文被外部关闭（例如 launch 结束），下面不再发停车指令
    finally:
        if rclpy.ok():
            acked = node.stop()
            node.get_logger().info(
                '已发送停车指令' + ('，节点退出' if acked else '（未收到确认），节点退出'))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
