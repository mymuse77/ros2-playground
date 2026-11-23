"""避障算法的单元测试：用合成的雷达数据验证扇区分析和状态机（不需要启动 ROS）。"""

from dataclasses import dataclass, field
import math

from pg_obstacle_avoidance_py.logic import (AvoidanceController, AvoidanceParams, Command,
                                            is_stale, normalize_angle, ScanSummary,
                                            sector_min_range, State, summarize_scan)
import pytest


@dataclass
class FakeScan:
    """
    模拟 TurtleBot3 的雷达：360 个读数，从正前方（0 rad）开始逆时针，每度一个。

    只包含算法用到的字段，鸭子类型与 sensor_msgs/msg/LaserScan 兼容。
    """

    ranges: list = field(default_factory=lambda: [math.inf] * 360)
    angle_min: float = 0.0
    angle_increment: float = math.radians(1.0)
    range_min: float = 0.12
    range_max: float = 3.5

    def with_reading(self, degree, distance):
        """设置某个角度（度，左正右负）的读数，返回自身便于链式调用。"""
        self.ranges[degree % 360] = distance
        return self


def summary(front=3.5, left=3.5, right=3.5):
    """快速构造 ScanSummary。"""
    return ScanSummary(front=front, front_left=left, front_right=right)


# ============================================================ 扇区分析

class TestNormalizeAngle:
    """角度归一化。"""

    @pytest.mark.parametrize('angle, expected', [
        (0.0, 0.0),
        (math.radians(350), math.radians(-10)),
        (math.radians(-190), math.radians(170)),
        (2.5 * math.pi, 0.5 * math.pi),
    ])
    def test_values(self, angle, expected):
        """超出 ±π 的角度（如 350°、-190°、2.5π）：归一化为 -π 到 π 之间的等价角。"""
        assert normalize_angle(angle) == pytest.approx(expected)


class TestSectorMinRange:
    """单个扇区的最近距离。"""

    def test_open_space_returns_range_max(self):
        """扇区内全是 inf（量程内没有障碍）：按 range_max 计，返回 3.5。"""
        assert sector_min_range(FakeScan(), -0.5, 0.5) == pytest.approx(3.5)

    def test_finds_nearest_across_zero_degree(self):
        """前方扇区跨越 0°：右侧 -10°（即第 350 个读数）也要被统计到，返回最近的 0.8。"""
        scan = FakeScan().with_reading(-10, 0.8).with_reading(10, 1.2)
        assert sector_min_range(scan, math.radians(-30), math.radians(30)) == pytest.approx(0.8)

    def test_ignores_readings_outside_sector(self):
        """障碍只在扇区外（90°）：不计入，返回 range_max。"""
        scan = FakeScan().with_reading(90, 0.3)
        assert sector_min_range(scan, math.radians(-30), math.radians(30)) == pytest.approx(3.5)

    def test_ignores_nan_and_below_range_min(self):
        """扇区内有 nan 和小于 range_min 的读数：忽略它们，返回有效最小值 1.0。"""
        scan = FakeScan().with_reading(0, math.nan).with_reading(5, 0.05).with_reading(-5, 1.0)
        assert sector_min_range(scan, math.radians(-30), math.radians(30)) == pytest.approx(1.0)

    def test_no_valid_reading_is_treated_as_blocked(self):
        """扇区内没有任何有效读数（全是 nan）：视为被挡住，返回 0.0。"""
        scan = FakeScan(ranges=[math.nan] * 360)
        assert sector_min_range(scan, -0.5, 0.5) == 0.0


class TestSummarizeScan:
    """三个扇区的划分。"""

    def test_sectors_left_positive_right_negative(self):
        """在 0°、+60°、-60° 放障碍：分别计入前方、左前方（正角）、右前方（负角）。"""
        scan = FakeScan().with_reading(0, 2.0).with_reading(60, 0.6).with_reading(-60, 0.9)
        result = summarize_scan(scan, math.radians(30), math.radians(90))
        assert result == ScanSummary(front=2.0, front_left=0.6, front_right=0.9)

    def test_rear_is_ignored(self):
        """障碍只在后方（180°、±120°）：三个扇区都按 range_max 计。"""
        scan = FakeScan().with_reading(180, 0.2).with_reading(120, 0.2).with_reading(-120, 0.2)
        result = summarize_scan(scan, math.radians(30), math.radians(90))
        assert result == ScanSummary(front=3.5, front_left=3.5, front_right=3.5)


class TestIsStale:
    """数据超时判断。"""

    def test_never_received(self):
        """从未收到数据（时间戳为 None）：判定为超时。"""
        assert is_stale(None, 10.0, 0.5)

    def test_fresh(self):
        """数据在 0.2 s 前收到（超时阈值 0.5 s）：判定为未超时。"""
        assert not is_stale(9.8, 10.0, 0.5)

    def test_timed_out(self):
        """数据在 1.0 s 前收到（超时阈值 0.5 s）：判定为超时。"""
        assert is_stale(9.0, 10.0, 0.5)


# ============================================================ 参数校验

class TestAvoidanceParams:
    """参数约束。"""

    def test_defaults_are_valid(self):
        """使用全部默认值构造 AvoidanceParams：不抛出异常。"""
        AvoidanceParams()

    @pytest.mark.parametrize('overrides', [
        {'min_linear_speed': 0.3, 'max_linear_speed': 0.2},
        {'turn_speed': 0.0},
        {'steer_gain': -1.0},
        {'stop_distance': 0.8, 'clear_distance': 0.7},
        {'stop_distance': 0.5, 'slow_distance': 0.5},
        {'side_distance': 0.0},
        {'max_turn_time': 0.0},
        {'front_half_angle': math.radians(100), 'side_angle': math.radians(90)},
    ])
    def test_invalid_combinations(self, overrides):
        """速度上下限颠倒、速度/增益/时间等非正、距离阈值顺序错、扇区角重叠：抛出 ValueError。"""
        with pytest.raises(ValueError):
            AvoidanceParams(**overrides)


# ============================================================ 状态机

class TestForward:
    """前进状态的速度计算。"""

    def setup_method(self):
        """每个测试前用默认参数新建控制器。"""
        self.params = AvoidanceParams()
        self.ctrl = AvoidanceController(self.params)

    def test_open_space_full_speed_straight(self):
        """三个扇区都开阔：以 max_linear_speed 直行，状态保持 FORWARD。"""
        cmd = self.ctrl.update(summary(), now=0.0)
        assert cmd == Command(self.params.max_linear_speed, 0.0)
        assert self.ctrl.state is State.FORWARD

    def test_slows_down_linearly(self):
        """前方 0.75 m，正好在 stop(0.5) 和 slow(1.0) 中间：速度取 min 和 max 的中间值。"""
        cmd = self.ctrl.update(summary(front=0.75), now=0.0)
        expected = (self.params.min_linear_speed + self.params.max_linear_speed) / 2
        assert cmd.linear == pytest.approx(expected)

    def test_steers_right_when_left_is_close(self):
        """左前方 0.25 m 过近：向右修正，角速度为 -steer_gain * 0.10。"""
        cmd = self.ctrl.update(summary(left=0.25), now=0.0)
        assert cmd.angular < 0.0
        assert cmd.angular == pytest.approx(-self.params.steer_gain * 0.10)

    def test_steers_left_when_right_is_close(self):
        """右前方 0.25 m 过近：向左修正，角速度为正。"""
        assert self.ctrl.update(summary(right=0.25), now=0.0).angular > 0.0

    def test_steering_is_clamped(self):
        """steer_gain 极大且左前方很近：修正角速度被限幅为 -turn_speed。"""
        params = AvoidanceParams(steer_gain=100.0)
        cmd = AvoidanceController(params).update(summary(left=0.13), now=0.0)
        assert cmd.angular == pytest.approx(-params.turn_speed)


class TestTurning:
    """转向状态：选方向、锁方向、迟滞、超时换向。"""

    def setup_method(self):
        """每个测试前用默认参数新建控制器。"""
        self.params = AvoidanceParams()
        self.ctrl = AvoidanceController(self.params)

    def test_turns_toward_more_open_side(self):
        """前方受阻：进入 TURNING，左侧更开阔就原地左转，右侧更开阔就原地右转。"""
        cmd = self.ctrl.update(summary(front=0.4, left=1.0, right=0.6), now=0.0)
        assert self.ctrl.state is State.TURNING
        assert cmd == Command(0.0, self.params.turn_speed)          # 向左

        ctrl = AvoidanceController(self.params)
        cmd = ctrl.update(summary(front=0.4, left=0.6, right=1.0), now=0.0)
        assert cmd == Command(0.0, -self.params.turn_speed)         # 向右

    def test_direction_is_locked_while_turning(self):
        """左转途中右侧变得更开阔：仍保持左转。"""
        self.ctrl.update(summary(front=0.4, left=1.0, right=0.6), now=0.0)
        # 转了一会儿，右边变得更开阔，但方向不变，避免来回打摆
        cmd = self.ctrl.update(summary(front=0.45, left=0.5, right=2.0), now=0.5)
        assert cmd.angular > 0.0

    def test_hysteresis_keeps_turning_between_stop_and_clear(self):
        """转向中前方距离回升到 stop 与 clear 之间：仍保持 TURNING（迟滞）。"""
        self.ctrl.update(summary(front=0.4), now=0.0)
        # 0.6 m：已大于 stop(0.5)，但还没到 clear(0.7) → 继续转
        self.ctrl.update(summary(front=0.6), now=0.1)
        assert self.ctrl.state is State.TURNING

    def test_resumes_forward_when_clear(self):
        """转向中前方距离超过 clear：回到 FORWARD，转向方向清零并恢复前进。"""
        self.ctrl.update(summary(front=0.4), now=0.0)
        cmd = self.ctrl.update(summary(front=0.8), now=0.5)
        assert self.ctrl.state is State.FORWARD
        assert self.ctrl.turn_direction == 0
        assert cmd.linear > 0.0

    def test_flips_direction_after_timeout(self):
        """左转后前方一直受阻：max_turn_time 之前保持左转，超时后改为右转。"""
        self.ctrl.update(summary(front=0.4, left=1.0, right=0.6), now=0.0)
        assert self.ctrl.turn_direction == 1
        timeout = self.params.max_turn_time
        self.ctrl.update(summary(front=0.4), now=timeout - 0.1)
        assert self.ctrl.turn_direction == 1
        cmd = self.ctrl.update(summary(front=0.4), now=timeout + 0.1)
        assert self.ctrl.turn_direction == -1
        assert cmd.angular < 0.0

    def test_params_can_be_replaced_at_runtime(self):
        """运行时替换 params（max_linear_speed=0.1）：下一次输出的线速度即为 0.1。"""
        self.ctrl.params = AvoidanceParams(max_linear_speed=0.1)
        assert self.ctrl.update(summary(), now=0.0).linear == pytest.approx(0.1)

    def test_reset_returns_to_forward(self):
        """转向中调用 reset()：回到 FORWARD，转向方向清零。"""
        self.ctrl.update(summary(front=0.4), now=0.0)
        self.ctrl.reset()
        assert self.ctrl.state is State.FORWARD
        assert self.ctrl.turn_direction == 0
