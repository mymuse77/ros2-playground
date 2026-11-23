"""
03 课 · Bug2 避障导航算法（纯 Python，不依赖 ROS）。

Bug 系列算法是最简单的"带避障的到点导航"：不需要地图，只需要知道自己在哪、目标在哪，
加上一个能测距的传感器。Bug2 的规则：

  1. 起点和目标点连成一条直线，叫 M-line（Main line）
  2. 沿 M-line 朝目标前进（GO_TO_GOAL）
  3. 前方被挡住时，记下这个"碰撞点"，开始沿障碍物边缘走（FOLLOW_WALL），
     本实现让障碍物始终在车的右侧
  4. 沿边走的过程中，再次回到 M-line、并且比碰撞点离目标更近、并且朝目标的方向没有障碍时，
     离开障碍物，回到第 2 步
  5. 绕了一整圈又回到碰撞点，说明目标被围住了，无法到达（FAILED）

            目标 ★
                │
           ┌────┼────┐
           │  障│碍物 │ ← 沿右手边绕行
           └────┼────┘
                │ ← M-line
            起点 ●

扇区距离复用 02 课的 sector_min_range（跨包导入 Python 模块）。
"""

from dataclasses import dataclass, field
from enum import Enum
import math

from pg_go_to_goal_py.geometry import (distance, distance_to_line, heading_error,
                                       normalize_angle)
from pg_go_to_goal_py.go_to_goal import Command, compute_go_to_goal, GoToGoalParams, STOP
from pg_obstacle_avoidance_py.logic import sector_min_range


def range_toward(scan, direction, half_width):
    """
    求以 direction 为中心、半宽 half_width 的扇区内的最近距离。

    与 sector_min_range 不同，这里的扇区可以跨越正后方（±pi），会自动拆成两段。

    :param scan: 雷达数据（LaserScan 或兼容对象）
    :param direction: 扇区中心方向（相对车头），rad
    :param half_width: 扇区半宽，rad，要求小于 pi
    :return: 最近距离，m
    """
    center = normalize_angle(direction)
    start, end = center - half_width, center + half_width
    if start < -math.pi:
        return min(sector_min_range(scan, start + 2 * math.pi, math.pi),
                   sector_min_range(scan, -math.pi, end))
    if end > math.pi:
        return min(sector_min_range(scan, start, math.pi),
                   sector_min_range(scan, -math.pi, end - 2 * math.pi))
    return sector_min_range(scan, start, end)


def nearest_obstacle(scan, sector_start, sector_end):
    """
    求扇区内最近的障碍点：返回 (距离, 方向)。

    与 sector_min_range 的区别是同时给出方向。inf（量程内无障碍）、NaN 和小于
    range_min 的读数都不算障碍点。

    :param scan: 雷达数据（LaserScan 或兼容对象）
    :param sector_start: 扇区起始角（相对车头，逆时针为正），rad
    :param sector_end: 扇区结束角，rad
    :return: (距离 m, 方向 rad)；扇区内没有障碍点时返回 (inf, 0.0)
    """
    best_range, best_angle = math.inf, 0.0
    for i, r in enumerate(scan.ranges):
        if not scan.range_min <= r <= scan.range_max:   # 同时排除 NaN、inf、过近读数
            continue
        angle = normalize_angle(scan.angle_min + i * scan.angle_increment)
        if sector_start <= angle <= sector_end and r < best_range:
            best_range, best_angle = r, angle
    return best_range, best_angle


@dataclass(frozen=True)
class Bug2Params:
    """Bug2 参数，单位 m、rad、m/s、rad/s。"""

    go_to_goal: GoToGoalParams = field(default_factory=GoToGoalParams)
    hit_distance: float = 0.45          # 前方小于此距离视为"被挡住"
    wall_distance: float = 0.40         # 沿边走时与右侧障碍保持的距离
    k_wall: float = 2.0                 # 距离误差的转向增益，(rad/s)/m
    k_wall_angle: float = 1.0           # 方向误差的转向增益，(rad/s)/rad
    wall_speed: float = 0.12            # 沿边走的前进速度
    wall_turn_speed: float = 0.8        # 沿边走的角速度上限
    mline_tolerance: float = 0.10       # 离 M-line 小于此距离视为"在 M-line 上"
    leave_margin: float = 0.20          # 离开障碍时，要比碰撞点离目标近至少这么多
    min_follow_distance: float = 0.50   # 沿边走过这么远之后，才开始判断离开或绕圈
    loop_tolerance: float = 0.25        # 回到碰撞点这么近，视为绕了一圈
    front_half_angle: float = math.radians(25.0)   # 前方扇区半角

    def __post_init__(self):
        """检查参数取值，不满足时抛出 ValueError。"""
        positives = [self.hit_distance, self.wall_distance, self.k_wall, self.k_wall_angle,
                     self.wall_speed, self.wall_turn_speed, self.mline_tolerance,
                     self.min_follow_distance, self.loop_tolerance, self.front_half_angle]
        if min(positives) <= 0.0:
            raise ValueError('Bug2 的距离、速度、增益、角度参数都必须大于 0')
        if self.leave_margin < 0.0:
            raise ValueError('leave_margin 不能为负数')
        if self.loop_tolerance >= self.min_follow_distance:
            # 否则刚开始沿边走就会被误判为"绕了一圈"
            raise ValueError('要求 loop_tolerance < min_follow_distance')


class Bug2State(Enum):
    """Bug2 的状态。"""

    GO_TO_GOAL = 'GO_TO_GOAL'
    FOLLOW_WALL = 'FOLLOW_WALL'
    REACHED = 'REACHED'
    FAILED = 'FAILED'


def wall_follow_command(scan, params):
    """
    右手沿边走的速度指令。

    在车的右半边（-180° ~ 0°）找最近的障碍点，同时控制两个量：
      距离：保持 wall_distance       → k_wall × (wall_distance - 距离)
      方向：让最近点保持在正右方 -90° → k_wall_angle × (方向 + 90°)

        最近点在右前方（-45°，快撞上了） → 方向误差为正 → 向左转
        最近点在右后方（-135°，刚过外拐角）→ 方向误差为负 → 向右绕着拐角转

    只看距离不看方向是不够的：在外拐角处，墙角会跑到右后方，"正右方扇区"里看不到它，
    车会以为丢了墙而原地打转（开发时在仿真中遇到过）。

    特殊情况：
      - 前方被挡住：原地左转，把障碍物转到右侧
      - 右半边没有 2 倍 wall_distance 以内的障碍（丢了墙）：以 wall_distance 为半径向右画弧找墙

    :return: Command
    """
    p = params
    front = range_toward(scan, 0.0, p.front_half_angle)
    if front < p.hit_distance:
        return Command(0.0, p.wall_turn_speed)
    dist, bearing = nearest_obstacle(scan, -math.pi, 0.0)
    if dist > 2.0 * p.wall_distance:
        return Command(p.wall_speed, -p.wall_speed / p.wall_distance)
    angular = (p.k_wall * (p.wall_distance - dist)
               + p.k_wall_angle * normalize_angle(bearing + math.pi / 2))
    return Command(p.wall_speed, max(-p.wall_turn_speed, min(p.wall_turn_speed, angular)))


class Bug2Planner:
    """
    Bug2 状态机。用法：

        planner = Bug2Planner(params)
        planner.start(当前位姿, 目标x, 目标y)
        每个控制周期：command = planner.update(当前位姿, 雷达数据)
        planner.state 为 REACHED 或 FAILED 时结束
    """

    def __init__(self, params):
        """创建规划器，start() 之前处于 FAILED 状态（没有任务）。"""
        self.params = params
        self._state = Bug2State.FAILED
        self.failure_reason = '尚未设置目标'

    @property
    def state(self):
        """当前状态 Bug2State。"""
        return self._state

    @property
    def start_point(self):
        """M-line 起点 (x, y)，即 start() 时的位置。"""
        return self._start

    @property
    def goal(self):
        """目标点 (x, y)。"""
        return self._goal

    @property
    def tolerance(self):
        """本次导航使用的到达容差，m。"""
        return self._tolerance

    @property
    def hit_point(self):
        """最近一次进入沿边走时的碰撞点 (x, y)；还没碰到障碍时为 None。"""
        return self._hit_point

    def start(self, pose, goal_x, goal_y, tolerance=None):
        """
        开始一次导航：以当前位置为起点建立 M-line。

        :param pose: 当前位姿 Pose2D
        :param goal_x: 目标 x，m
        :param goal_y: 目标 y，m
        :param tolerance: 到达容差，m；None 表示用 go_to_goal.goal_tolerance
        """
        self._start = (pose.x, pose.y)
        self._goal = (goal_x, goal_y)
        self._tolerance = (self.params.go_to_goal.goal_tolerance
                           if tolerance is None else tolerance)
        self._state = Bug2State.GO_TO_GOAL
        self.failure_reason = ''
        self._hit_point = None
        self._hit_goal_distance = None
        self._follow_length = 0.0
        self._last_position = None

    def distance_to_goal(self, pose):
        """当前位置到目标点的距离，m。"""
        return distance(pose.x, pose.y, *self._goal)

    def update(self, pose, scan):
        """
        计算当前控制周期的速度指令。

        :param pose: 当前位姿 Pose2D
        :param scan: 雷达数据（LaserScan 或兼容对象）
        :return: Command
        """
        if self._state in (Bug2State.REACHED, Bug2State.FAILED):
            return STOP
        p = self.params
        goal_distance = self.distance_to_goal(pose)
        if goal_distance < self._tolerance:
            self._state = Bug2State.REACHED
            return STOP

        if self._state is Bug2State.GO_TO_GOAL:
            front = range_toward(scan, 0.0, p.front_half_angle)
            # 被挡住，而且障碍物比目标还近（目标就在障碍物前面时不用绕）
            if front < p.hit_distance and front < goal_distance:
                self._enter_follow_wall(pose, goal_distance)
            else:
                return self._go_to_goal(pose)

        # ---- FOLLOW_WALL ----
        self._follow_length += distance(pose.x, pose.y, *self._last_position)
        self._last_position = (pose.x, pose.y)
        if self._follow_length > p.min_follow_distance:
            if self._can_leave(pose, scan, goal_distance):
                self._state = Bug2State.GO_TO_GOAL
                return self._go_to_goal(pose)
            if distance(pose.x, pose.y, *self._hit_point) < p.loop_tolerance:
                self._state = Bug2State.FAILED
                self.failure_reason = '绕障碍物一圈回到碰撞点，目标无法到达'
                return STOP
        return wall_follow_command(scan, p)

    def _enter_follow_wall(self, pose, goal_distance):
        """记录碰撞点，进入沿边走状态。"""
        self._state = Bug2State.FOLLOW_WALL
        self._hit_point = (pose.x, pose.y)
        self._hit_goal_distance = goal_distance
        self._follow_length = 0.0
        self._last_position = (pose.x, pose.y)

    def _can_leave(self, pose, scan, goal_distance):
        """Bug2 的离开条件：在 M-line 上、比碰撞点更近、朝目标方向畅通。"""
        p = self.params
        on_mline = abs(distance_to_line(pose.x, pose.y, *self._start, *self._goal)) \
            < p.mline_tolerance
        closer = goal_distance < self._hit_goal_distance - p.leave_margin
        if not (on_mline and closer):
            return False
        toward_goal = range_toward(scan, heading_error(pose, *self._goal), p.front_half_angle)
        return toward_goal > p.hit_distance

    def _go_to_goal(self, pose):
        """朝目标点的 P 控制指令。"""
        command, _ = compute_go_to_goal(pose, *self._goal, self.params.go_to_goal,
                                        self._tolerance)
        return command
