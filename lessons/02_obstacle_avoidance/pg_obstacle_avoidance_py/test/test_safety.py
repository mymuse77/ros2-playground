"""安全验收统计的单元测试。"""

from dataclasses import dataclass, field
import math

from pg_obstacle_avoidance_py.safety import (footprint_clearance, point_to_box_distance,
                                             TripMonitor)
import pytest


@dataclass
class FakeScan:
    """360 线雷达，默认全部为 inf（量程内无障碍）。"""

    ranges: list = field(default_factory=lambda: [math.inf] * 360)
    angle_min: float = 0.0
    angle_increment: float = math.radians(1.0)
    range_min: float = 0.12
    range_max: float = 3.5


class TestPointToBox:
    """点到矩形的距离。"""

    @pytest.mark.parametrize('x, y, expected', [
        (0.5, 0.0, 0.4),          # 正前方：只差 x 方向
        (0.0, -0.3, 0.1),         # 正右方：只差 y 方向
        (0.4, 0.5, math.hypot(0.3, 0.3)),   # 斜前方：到角点的距离
        (0.05, 0.05, 0.0),        # 在矩形内部
    ])
    def test_values(self, x, y, expected):
        assert point_to_box_distance(x, y, 0.1, 0.2) == pytest.approx(expected)


class TestFootprintClearance:
    """车身净空。"""

    def test_no_obstacle(self):
        assert footprint_clearance(FakeScan(), 0.14, 0.155) == math.inf

    def test_front_obstacle(self):
        scan = FakeScan()
        scan.ranges[0] = 0.5
        assert footprint_clearance(scan, 0.14, 0.155) == pytest.approx(0.36)

    def test_side_obstacle_uses_width(self):
        scan = FakeScan()
        scan.ranges[90] = 0.5      # 正左方
        assert footprint_clearance(scan, 0.14, 0.155) == pytest.approx(0.345)

    def test_invalid_readings_ignored(self):
        scan = FakeScan()
        scan.ranges[0] = math.nan
        scan.ranges[1] = 0.05      # 小于 range_min
        assert footprint_clearance(scan, 0.14, 0.155) == math.inf


class TestTripMonitor:
    """里程、接触、卡住统计。"""

    def setup_method(self):
        self.monitor = TripMonitor(contact_distance=0.03, stuck_time=10.0, stuck_distance=0.1)

    def test_distance_accumulates(self):
        for t, x in enumerate([0.0, 1.0, 1.0, 3.0]):
            self.monitor.update_position(float(t), x, 0.0)
        report = self.monitor.report()
        assert report.distance == pytest.approx(3.0)
        assert report.elapsed == pytest.approx(3.0)

    def test_contact_counted_once_per_entry(self):
        for clearance in [0.5, 0.02, 0.01, 0.2, 0.02]:
            self.monitor.update_clearance(clearance)
        report = self.monitor.report()
        assert report.contacts == 2
        assert report.min_clearance == pytest.approx(0.01)

    def test_moving_robot_is_not_stuck(self):
        for t in range(30):
            self.monitor.update_position(float(t), 0.05 * t, 0.0)
        assert not self.monitor.report().stuck
        assert self.monitor.report().passed

    def test_stationary_robot_is_stuck(self):
        for t in range(30):
            self.monitor.update_position(float(t), 0.001 * t, 0.0)
        report = self.monitor.report()
        assert report.stuck
        assert not report.passed

    def test_empty_report(self):
        report = self.monitor.report()
        assert report.elapsed == 0.0
        assert report.passed
