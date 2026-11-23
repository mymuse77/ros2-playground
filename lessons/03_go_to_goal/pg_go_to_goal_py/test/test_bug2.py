"""
Bug2 算法的单元测试。

除了单步测试外，还用一个极简的"二维雷达模拟器"做闭环测试：
在小车和目标之间放一堵墙，检查 Bug2 能否绕过去到达目标。
"""

from dataclasses import dataclass, field
import math

from pg_go_to_goal_py.bug2 import (Bug2Params, Bug2Planner, Bug2State, nearest_obstacle,
                                   range_toward, wall_follow_command)
from pg_go_to_goal_py.geometry import normalize_angle, Pose2D
import pytest

RANGE_MAX = 3.5


# ============================================================ 测试工具：雷达数据

@dataclass
class FakeScan:
    """360 线雷达（1° 一个读数，0° 为正前方），默认全部为 inf。"""

    ranges: list = field(default_factory=lambda: [math.inf] * 360)
    angle_min: float = 0.0
    angle_increment: float = math.radians(1.0)
    range_min: float = 0.12
    range_max: float = RANGE_MAX

    def with_obstacle(self, from_deg, to_deg, distance):
        """把 [from_deg, to_deg]（度，左正右负）范围内的读数设为 distance。"""
        for deg in range(from_deg, to_deg + 1):
            self.ranges[deg % 360] = distance
        return self


@dataclass(frozen=True)
class Segment:
    """墙段：从 (x1, y1) 到 (x2, y2)。"""

    x1: float
    y1: float
    x2: float
    y2: float


def ray_segment_distance(ox, oy, angle, seg):
    """从 (ox, oy) 沿 angle 方向的射线与墙段的交点距离；不相交返回 inf。"""
    dx, dy = math.cos(angle), math.sin(angle)
    ex, ey = seg.x2 - seg.x1, seg.y2 - seg.y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return math.inf
    t = ((seg.x1 - ox) * ey - (seg.y1 - oy) * ex) / denom     # 射线参数
    u = ((seg.x1 - ox) * dy - (seg.y1 - oy) * dx) / denom     # 墙段参数
    return t if t > 0.0 and 0.0 <= u <= 1.0 else math.inf


def simulate_scan(pose, walls):
    """生成一帧模拟雷达数据：从 pose 向 360 个方向发射射线，取与墙段的最近交点。"""
    scan = FakeScan()
    for i in range(360):
        angle = pose.yaw + math.radians(i)
        d = min(ray_segment_distance(pose.x, pose.y, angle, w) for w in walls)
        scan.ranges[i] = d if d <= RANGE_MAX else math.inf
    return scan


def box(cx, cy, half_x, half_y):
    """以 (cx, cy) 为中心的矩形障碍物，返回 4 条墙段。"""
    x1, x2, y1, y2 = cx - half_x, cx + half_x, cy - half_y, cy + half_y
    return [Segment(x1, y1, x2, y1), Segment(x2, y1, x2, y2),
            Segment(x2, y2, x1, y2), Segment(x1, y2, x1, y1)]


def right_wall_scan(distance, yaw_deg=0.0):
    """车在原点，右侧 distance 处有一堵与 x 轴平行的长墙；车头朝向 yaw_deg。"""
    wall = Segment(-10.0, -distance, 10.0, -distance)
    return simulate_scan(Pose2D(0.0, 0.0, math.radians(yaw_deg)), [wall])


# ============================================================ 单步测试

class TestRangeToward:
    """可跨越正后方的扇区。"""

    def test_front(self):
        scan = FakeScan().with_obstacle(-5, 5, 1.0)
        assert range_toward(scan, 0.0, math.radians(20)) == pytest.approx(1.0)

    def test_across_rear(self):
        # 中心在 175°，扇区 155° ~ 195°（即 -165°），障碍在 -170°
        scan = FakeScan().with_obstacle(-170, -170, 0.8)
        assert range_toward(scan, math.radians(175), math.radians(20)) == pytest.approx(0.8)

    def test_across_rear_negative_side(self):
        scan = FakeScan().with_obstacle(170, 170, 0.6)
        assert range_toward(scan, math.radians(-175), math.radians(20)) == pytest.approx(0.6)


class TestNearestObstacle:
    """扇区内最近障碍点（距离 + 方向）。"""

    def test_distance_and_bearing(self):
        scan = FakeScan().with_obstacle(-100, -80, 1.0).with_obstacle(-30, -30, 0.5)
        dist, bearing = nearest_obstacle(scan, -math.pi, 0.0)
        assert dist == pytest.approx(0.5)
        assert bearing == pytest.approx(math.radians(-30))

    def test_outside_sector_ignored(self):
        scan = FakeScan().with_obstacle(45, 45, 0.3)      # 在左边
        assert nearest_obstacle(scan, -math.pi, 0.0) == (math.inf, 0.0)

    def test_invalid_readings_ignored(self):
        scan = FakeScan().with_obstacle(-90, -90, float('nan')).with_obstacle(-60, -60, 0.05)
        assert nearest_obstacle(scan, -math.pi, 0.0)[0] == math.inf


class TestWallFollow:
    """右手沿边走。"""

    def setup_method(self):
        self.params = Bug2Params()

    def test_front_blocked_turns_left(self):
        scan = FakeScan().with_obstacle(-10, 10, 0.3)
        command = wall_follow_command(scan, self.params)
        assert command.linear == 0.0 and command.angular > 0.0

    def test_keeps_distance(self):
        too_close = wall_follow_command(right_wall_scan(0.25), self.params)
        too_far = wall_follow_command(right_wall_scan(0.6), self.params)
        exact = wall_follow_command(right_wall_scan(0.4), self.params)
        assert too_close.angular > 0.0      # 太近：向左偏
        assert too_far.angular < 0.0        # 太远：向右偏
        assert exact.angular == pytest.approx(0.0, abs=0.05)
        assert exact.linear == pytest.approx(self.params.wall_speed)

    def test_heading_toward_wall_turns_left(self):
        # 距离正好，但车头向右偏了 20°，正朝墙开过去：最近点在右前方，应向左修正
        command = wall_follow_command(right_wall_scan(0.4, yaw_deg=-20.0), self.params)
        assert command.angular > 0.0

    def test_outer_corner_turns_right(self):
        # 刚经过外拐角：唯一的障碍点在右后方 -135°，应向右绕着拐角转
        scan = FakeScan().with_obstacle(-135, -135, 0.45)
        command = wall_follow_command(scan, self.params)
        assert command.linear > 0.0 and command.angular < 0.0

    def test_lost_wall_arcs_right(self):
        # 右边什么都没有：以 wall_distance 为半径向右画弧
        command = wall_follow_command(FakeScan(), self.params)
        assert command.linear == pytest.approx(self.params.wall_speed)
        assert command.angular == pytest.approx(
            -self.params.wall_speed / self.params.wall_distance)


class TestBug2Steps:
    """状态切换。"""

    def setup_method(self):
        self.params = Bug2Params()
        self.planner = Bug2Planner(self.params)

    def test_idle_before_start(self):
        assert self.planner.state is Bug2State.FAILED
        assert self.planner.update(Pose2D(0, 0, 0), FakeScan()).linear == 0.0

    def test_clear_path_goes_to_goal(self):
        self.planner.start(Pose2D(0, 0, 0), 3.0, 0.0)
        command = self.planner.update(Pose2D(0, 0, 0), FakeScan())
        assert self.planner.state is Bug2State.GO_TO_GOAL
        assert command.linear > 0.0

    def test_obstacle_ahead_starts_wall_following(self):
        self.planner.start(Pose2D(0, 0, 0), 3.0, 0.0)
        self.planner.update(Pose2D(0, 0, 0), FakeScan().with_obstacle(-20, 20, 0.4))
        assert self.planner.state is Bug2State.FOLLOW_WALL
        assert self.planner.hit_point == (0, 0)

    def test_obstacle_beyond_goal_is_ignored(self):
        # 目标在 0.3 m 处，障碍物在 0.4 m 处：直接去目标，不绕
        self.planner.start(Pose2D(0, 0, 0), 0.3, 0.0)
        self.planner.update(Pose2D(0, 0, 0), FakeScan().with_obstacle(-20, 20, 0.4))
        assert self.planner.state is Bug2State.GO_TO_GOAL

    def test_reached(self):
        self.planner.start(Pose2D(0, 0, 0), 0.05, 0.0)
        assert self.planner.update(Pose2D(0, 0, 0), FakeScan()).linear == 0.0
        assert self.planner.state is Bug2State.REACHED

    def test_start_properties(self):
        self.planner.start(Pose2D(1.0, 2.0, 0.0), 3.0, 4.0, tolerance=0.3)
        assert self.planner.start_point == (1.0, 2.0)
        assert self.planner.goal == (3.0, 4.0)
        assert self.planner.tolerance == 0.3
        assert self.planner.hit_point is None

    def test_invalid_params(self):
        with pytest.raises(ValueError):
            Bug2Params(hit_distance=0.0)
        with pytest.raises(ValueError):
            Bug2Params(loop_tolerance=0.6, min_follow_distance=0.5)


# ============================================================ 闭环仿真

def run_bug2(walls, goal, max_steps=6000, dt=0.05):
    """闭环运行 Bug2，返回 (最终状态, 经历过的状态集合, 离墙最近距离)。"""
    planner = Bug2Planner(Bug2Params())
    pose = Pose2D(0.0, 0.0, 0.0)
    planner.start(pose, *goal)
    seen, nearest = set(), math.inf
    for _ in range(max_steps):
        scan = simulate_scan(pose, walls)
        nearest = min(nearest, min(scan.ranges))
        command = planner.update(pose, scan)
        seen.add(planner.state)
        if planner.state in (Bug2State.REACHED, Bug2State.FAILED):
            break
        yaw = normalize_angle(pose.yaw + command.angular * dt)
        pose = Pose2D(pose.x + command.linear * math.cos(pose.yaw) * dt,
                      pose.y + command.linear * math.sin(pose.yaw) * dt, yaw)
    return planner.state, seen, nearest


def test_closed_loop_goes_around_wall():
    """起点 (0,0)，目标 (4,0)，中间 x=2 处有一堵 1.6 m 宽的墙。"""
    state, seen, nearest = run_bug2(box(2.0, 0.0, 0.1, 0.8), goal=(4.0, 0.0))
    assert state is Bug2State.REACHED
    assert Bug2State.FOLLOW_WALL in seen      # 确实绕了墙
    assert nearest > 0.2                      # 全程没有贴到墙上


@pytest.mark.parametrize('dt', [0.05, 0.3])
def test_closed_loop_bug2_course(dt):
    """
    与仿真场景 worlds/bug2_course.sdf 相同：x=1.5 处 0.2 × 2.0 m 的墙，目标 (3,0)。

    dt=0.3 模拟控制周期较长的情况（仿真慢于真实时间时，按仿真时间算的控制周期会变长）。
    """
    state, seen, nearest = run_bug2(box(1.5, 0.0, 0.1, 1.0), goal=(3.0, 0.0),
                                    max_steps=int(300 / dt), dt=dt)
    assert state is Bug2State.REACHED
    assert Bug2State.FOLLOW_WALL in seen
    assert nearest > 0.2


def test_closed_loop_unreachable_goal_fails():
    """目标被一个封闭的方框围住，应在绕一圈后报告失败。"""
    state, _, _ = run_bug2(box(3.0, 0.0, 0.8, 0.8), goal=(3.0, 0.0))
    assert state is Bug2State.FAILED
