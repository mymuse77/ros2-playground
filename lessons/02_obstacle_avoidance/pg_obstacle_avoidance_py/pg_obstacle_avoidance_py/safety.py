"""
02 课 · 安全验收统计（纯 Python，不依赖 ROS）。

雷达测的是"障碍物到雷达中心"的距离，但会不会撞上，取决于障碍物到"车身外轮廓"的距离。
这里把车身近似为以雷达为中心的矩形，计算每个雷达点到矩形边缘的最短距离（净空）。

    y ▲
      │  ┌───────────┐
      │  │           │ ← 车身矩形：长 2 × half_length，宽 2 × half_width
      │  │     ●     │   ● 雷达（TurtleBot3 waffle 的雷达正好在车身中心）
      │  │           │
      │  └───────────┘ · · · · ✕ 障碍点：净空 = 点到矩形的最短距离
      └──────────────────▶ x（车头方向）
"""

from dataclasses import dataclass
import math


def point_to_box_distance(x, y, half_length, half_width):
    """
    计算点到"以原点为中心、与坐标轴对齐的矩形"的最短距离。

    :param x: 点的 x 坐标，m
    :param y: 点的 y 坐标，m
    :param half_length: 矩形 x 方向的半长，m
    :param half_width: 矩形 y 方向的半宽，m
    :return: 距离，m；点在矩形内部时为 0
    """
    dx = max(abs(x) - half_length, 0.0)
    dy = max(abs(y) - half_width, 0.0)
    return math.hypot(dx, dy)


def footprint_clearance(scan, half_length, half_width):
    """
    计算一帧雷达数据中，障碍物到车身矩形的最小净空。

    无效读数（NaN、inf、超出 [range_min, range_max]）跳过。

    :param scan: 雷达数据，需要 ranges、angle_min、angle_increment、range_min、range_max
    :param half_length: 车身半长，m
    :param half_width: 车身半宽，m
    :return: 最小净空，m；没有任何有效读数时返回 inf
    """
    clearance = math.inf
    for i, distance in enumerate(scan.ranges):
        if not scan.range_min <= distance <= scan.range_max:   # NaN 与任何数比较都是 False
            continue
        angle = scan.angle_min + i * scan.angle_increment
        d = point_to_box_distance(distance * math.cos(angle), distance * math.sin(angle),
                                  half_length, half_width)
        clearance = min(clearance, d)
    return clearance


@dataclass
class TripReport:
    """验收报告。"""

    elapsed: float            # 统计时长，s
    distance: float           # 行驶里程，m
    min_clearance: float      # 最小净空，m
    contacts: int             # 接触次数（净空从安全变为小于 contact_distance 的次数）
    stuck: bool               # 最近 stuck_time 内移动距离是否小于 stuck_distance

    @property
    def passed(self):
        """没有接触、也没有卡住，即为通过。"""
        return self.contacts == 0 and not self.stuck


class TripMonitor:
    """
    累计行驶统计：里程、最小净空、接触次数、是否卡住。

    时间、位置、净空都由调用者传入，便于单元测试。
    """

    def __init__(self, contact_distance, stuck_time, stuck_distance):
        """
        创建统计器。

        :param contact_distance: 净空小于此值视为接触（碰撞），m
        :param stuck_time: 卡住判定的时间窗口，s
        :param stuck_distance: 时间窗口内移动距离小于此值视为卡住，m
        """
        self._contact_distance = contact_distance
        self._stuck_time = stuck_time
        self._stuck_distance = stuck_distance
        self._start_time = None
        self._last_time = None
        self._last_position = None
        self._distance = 0.0
        self._min_clearance = math.inf
        self._contacts = 0
        self._in_contact = False
        self._anchor = None          # (时间, 位置)：卡住判定的参考点

    def update_position(self, now, x, y):
        """
        记录一次位置，累加里程并更新卡住判定的参考点。

        :param now: 当前时间，s
        :param x: 位置 x，m
        :param y: 位置 y，m
        """
        if self._start_time is None:
            self._start_time = now
        self._last_time = now
        if self._last_position is not None:
            self._distance += math.dist(self._last_position, (x, y))
        self._last_position = (x, y)
        # 离开参考点足够远，说明在正常移动，把参考点挪到当前位置
        if self._anchor is None or math.dist(self._anchor[1], (x, y)) >= self._stuck_distance:
            self._anchor = (now, (x, y))

    def update_clearance(self, clearance):
        """
        记录一次净空，更新最小净空和接触次数。

        接触按"进入"次数计：连续多帧都小于阈值只算一次。
        """
        self._min_clearance = min(self._min_clearance, clearance)
        touching = clearance < self._contact_distance
        if touching and not self._in_contact:
            self._contacts += 1
        self._in_contact = touching

    def report(self):
        """生成当前的 TripReport。"""
        elapsed = 0.0 if self._start_time is None else self._last_time - self._start_time
        stuck = (self._anchor is not None
                 and self._last_time - self._anchor[0] > self._stuck_time)
        return TripReport(elapsed=elapsed, distance=self._distance,
                          min_clearance=self._min_clearance, contacts=self._contacts,
                          stuck=stuck)
