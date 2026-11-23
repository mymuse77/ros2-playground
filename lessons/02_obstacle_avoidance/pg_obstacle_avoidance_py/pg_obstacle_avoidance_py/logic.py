"""
02 课 · 避障算法（纯 Python，不依赖 ROS）。

分两部分：
  1. 雷达扇区分析：把一圈距离读数归纳成"前方、左前、右前"三个扇区的最近距离
  2. 避障决策：根据这三个距离，用一个两状态的状态机计算速度指令

坐标约定（ROS REP-103）：x 向前、y 向左，角度逆时针为正。
所以"左"对应正角度，向左转对应正角速度。

              前方 front
          ╲     │     ╱
    左前    ╲   │   ╱    右前
  front_left  ╲ │ ╱  front_right
   ──────────[ 车 ]──────────     ← 车后方的读数不参与决策
"""

from dataclasses import dataclass, fields
from enum import Enum
import math


# ============================================================ 扇区分析

def normalize_angle(angle):
    """把角度归一化到 [-pi, pi)，单位 rad。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def sector_min_range(scan, sector_start, sector_end):
    """
    求一个扇区内的最近距离。

    读数的处理规则：
      - NaN 或小于 range_min：无效（测量失败或太近测不准），跳过
      - 大于 range_max（包括 inf）：这个方向在量程内没有障碍，按 range_max 计
    扇区内没有任何有效读数时返回 0.0，即"当作被挡住"——宁可多转一下，也不盲目前进。

    :param scan: 雷达数据。只要有 ranges、angle_min、angle_increment、range_min、range_max
                 这几个属性就行，sensor_msgs/msg/LaserScan 可以直接传入
    :param sector_start: 扇区起始角度，rad
    :param sector_end: 扇区结束角度，rad。要求 -pi <= start < end <= pi（扇区不能跨越正后方）
    :return: 扇区内的最近距离，m
    """
    nearest = None
    for i, distance in enumerate(scan.ranges):
        angle = normalize_angle(scan.angle_min + i * scan.angle_increment)
        if not sector_start <= angle <= sector_end:
            continue
        if math.isnan(distance) or distance < scan.range_min:
            continue
        distance = min(distance, scan.range_max)
        if nearest is None or distance < nearest:
            nearest = distance
    return 0.0 if nearest is None else nearest


@dataclass(frozen=True)
class ScanSummary:
    """三个扇区各自的最近距离，单位 m。"""

    front: float
    front_left: float
    front_right: float


def summarize_scan(scan, front_half_angle, side_angle):
    """
    把一帧雷达数据归纳为三个扇区的最近距离。

    扇区划分（角度相对车头，左正右负）：
      前方 [-front_half_angle, +front_half_angle]
      左前 [+front_half_angle, +side_angle]
      右前 [-side_angle, -front_half_angle]

    :param scan: 雷达数据，要求同 sector_min_range()
    :param front_half_angle: 前方扇区的半角，rad
    :param side_angle: 侧方扇区的外边界，rad
    :return: ScanSummary
    """
    return ScanSummary(
        front=sector_min_range(scan, -front_half_angle, front_half_angle),
        front_left=sector_min_range(scan, front_half_angle, side_angle),
        front_right=sector_min_range(scan, -side_angle, -front_half_angle),
    )


def is_stale(last_time, now, timeout):
    """
    判断数据是否过期。

    :param last_time: 最近一次收到数据的时间，s；从没收到过为 None
    :param now: 当前时间，s
    :param timeout: 超时时间，s
    :return: 从没收到过，或距离上次收到已超过 timeout 时返回 True
    """
    return last_time is None or now - last_time > timeout


# ============================================================ 避障决策

@dataclass(frozen=True)
class AvoidanceParams:
    """
    避障参数，全部使用国际单位（m、s、rad、m/s、rad/s）。

    几个距离阈值的关系：stop_distance < clear_distance，stop_distance < slow_distance。
    stop 和 clear 拉开差距形成"迟滞"：小于 stop 开始转向，大于 clear 才恢复前进，
    避免距离在阈值附近波动时反复切换状态。
    """

    max_linear_speed: float = 0.20               # 前方开阔时的前进速度
    min_linear_speed: float = 0.05               # 接近障碍时的最低前进速度
    turn_speed: float = 1.0                      # 原地转向的角速度；也是前进时转向角速度的上限
    steer_gain: float = 2.0                      # 前进时躲避侧方障碍的转向增益，(rad/s)/m
    stop_distance: float = 0.50                  # 前方小于此距离：停止前进，原地转向
    slow_distance: float = 1.00                  # 前方小于此距离：开始减速
    clear_distance: float = 0.70                 # 转向中前方大于此距离：恢复前进
    side_distance: float = 0.35                  # 侧方小于此距离：前进时向另一侧偏转
    max_turn_time: float = 6.0                   # 同一方向转向超过此时间仍未脱困：换个方向转
    front_half_angle: float = math.radians(30.0)  # 前方扇区半角
    side_angle: float = math.radians(90.0)       # 侧方扇区外边界

    def __post_init__(self):
        """检查参数之间的约束，不满足时抛出 ValueError。"""
        checks = [
            (0.0 <= self.min_linear_speed <= self.max_linear_speed,
             '要求 0 <= min_linear_speed <= max_linear_speed'),
            (self.turn_speed > 0.0, '要求 turn_speed > 0'),
            (self.steer_gain >= 0.0, '要求 steer_gain >= 0'),
            (0.0 < self.stop_distance < self.clear_distance,
             '要求 0 < stop_distance < clear_distance'),
            (self.stop_distance < self.slow_distance, '要求 stop_distance < slow_distance'),
            (self.side_distance > 0.0, '要求 side_distance > 0'),
            (self.max_turn_time > 0.0, '要求 max_turn_time > 0'),
            (0.0 < self.front_half_angle < self.side_angle <= math.pi,
             '要求 0 < front_half_angle < side_angle <= 180°'),
        ]
        for ok, message in checks:
            if not ok:
                raise ValueError(message)


# AvoidanceParams 中以弧度存储、但对外（ROS 参数）以"度"表示的字段
ANGLE_FIELDS = ('front_half_angle', 'side_angle')
DEG_SUFFIX = '_deg'


def to_ros_params(params):
    """
    把 AvoidanceParams 转换成 ROS 参数字典。

    角度字段换算成度，名字加 _deg 后缀（如 front_half_angle → front_half_angle_deg），
    其他字段名字和数值不变。节点用它声明参数及默认值。

    :param params: AvoidanceParams
    :return: dict，ROS 参数名 → 数值
    """
    result = {}
    for f in fields(params):
        value = getattr(params, f.name)
        if f.name in ANGLE_FIELDS:
            # 四舍五入去掉浮点误差，避免显示成 29.999999999999996
            result[f.name + DEG_SUFFIX] = round(math.degrees(value), 6)
        else:
            result[f.name] = value
    return result


def from_ros_params(values):
    """
    把 ROS 参数字典转换回 AvoidanceParams，是 to_ros_params() 的逆操作。

    :param values: dict，ROS 参数名 → 数值；可以只包含部分参数，缺少的取默认值
    :return: AvoidanceParams
    :raises ValueError: 参数之间的约束不满足
    :raises TypeError: 包含未知的参数名
    """
    kwargs = {}
    for name, value in values.items():
        if name.endswith(DEG_SUFFIX) and name[:-len(DEG_SUFFIX)] in ANGLE_FIELDS:
            kwargs[name[:-len(DEG_SUFFIX)]] = math.radians(value)
        else:
            kwargs[name] = float(value)
    return AvoidanceParams(**kwargs)


class State(Enum):
    """避障状态机的状态。"""

    FORWARD = 'FORWARD'   # 前进：根据前方距离调速，根据侧方距离微调方向
    TURNING = 'TURNING'   # 转向：原地旋转，直到前方开阔


@dataclass(frozen=True)
class Command:
    """速度指令：linear 前进速度 m/s，angular 角速度 rad/s（正值向左转）。"""

    linear: float
    angular: float


STOP = Command(0.0, 0.0)


def _clamp(value, low, high):
    """把 value 限制在 [low, high] 区间内。"""
    return max(low, min(high, value))


class AvoidanceController:
    """
    避障状态机。

    状态转移：
      FORWARD --(前方 < stop_distance)--> TURNING（选择更开阔的一侧，并锁定方向）
      TURNING --(前方 > clear_distance)--> FORWARD
      TURNING --(同方向转了 max_turn_time 还没脱困)--> TURNING（反方向）

    锁定转向方向是为了避免"左一下、右一下"的抖动：两侧距离差不多时，
    如果每帧都重新比较左右，转向方向会来回切换，车就原地打摆。

    控制器本身不读时钟，时间由调用者传入，所以可以在单元测试里精确控制时间。
    """

    def __init__(self, params):
        """
        创建控制器。

        :param params: AvoidanceParams；运行中可以直接替换 controller.params
        """
        self.params = params
        self.reset()

    def reset(self):
        """回到初始状态（FORWARD）。"""
        self._state = State.FORWARD
        self._turn_direction = 0     # +1 向左转，-1 向右转，0 未在转向
        self._turn_start = None      # 本次朝当前方向转向的开始时间，s

    @property
    def state(self):
        """当前状态（State）。"""
        return self._state

    @property
    def turn_direction(self):
        """当前转向方向：+1 向左、-1 向右、0 未在转向。"""
        return self._turn_direction

    def update(self, summary, now):
        """
        根据最新的扇区距离，更新状态并计算速度指令。

        :param summary: ScanSummary
        :param now: 当前时间，s（只用于转向超时判断，任意起点均可）
        :return: Command
        """
        p = self.params
        if self._state is State.FORWARD:
            if summary.front < p.stop_distance:
                self._start_turning(summary, now)
                return self._turn_command()
            return self._forward_command(summary)

        # ---- TURNING ----
        if summary.front > p.clear_distance:
            self.reset()
            return self._forward_command(summary)
        if now - self._turn_start > p.max_turn_time:
            # 同一方向转了太久还没找到出路（比如卡在角落），换个方向试试
            self._turn_direction = -self._turn_direction
            self._turn_start = now
        return self._turn_command()

    def _start_turning(self, summary, now):
        """进入转向状态：朝更开阔的一侧转，距离相同时默认向左。"""
        self._state = State.TURNING
        self._turn_direction = 1 if summary.front_left >= summary.front_right else -1
        self._turn_start = now

    def _turn_command(self):
        """原地转向的速度指令。"""
        return Command(0.0, self._turn_direction * self.params.turn_speed)

    def _forward_command(self, summary):
        """
        前进状态的速度指令。

        前进速度：前方距离从 slow_distance 降到 stop_distance 的过程中，
                  速度从 max_linear_speed 线性降到 min_linear_speed
        角速度：  侧方距离小于 side_distance 时，按"侵入的深度 × steer_gain"向另一侧偏转
        """
        p = self.params
        ratio = _clamp((summary.front - p.stop_distance) / (p.slow_distance - p.stop_distance),
                       0.0, 1.0)
        linear = p.min_linear_speed + (p.max_linear_speed - p.min_linear_speed) * ratio

        left_intrusion = max(0.0, p.side_distance - summary.front_left)
        right_intrusion = max(0.0, p.side_distance - summary.front_right)
        # 左边太近 → 向右转（负角速度）；右边太近 → 向左转（正角速度）
        angular = p.steer_gain * (right_intrusion - left_intrusion)
        angular = _clamp(angular, -p.turn_speed, p.turn_speed)
        return Command(linear, angular)
