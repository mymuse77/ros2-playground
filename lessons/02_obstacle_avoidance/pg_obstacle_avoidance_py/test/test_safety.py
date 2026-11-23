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
        """点在矩形正前方、正右方、斜前方或内部：返回到矩形的最近距离，内部为 0。"""
        assert point_to_box_distance(x, y, 0.1, 0.2) == pytest.approx(expected)


class TestFootprintClearance:
    """车身净空。"""

    def test_no_obstacle(self):
        """雷达读数全为 inf：车身净空为无穷大。"""
        assert footprint_clearance(FakeScan(), 0.14, 0.155) == math.inf

    def test_front_obstacle(self):
        """正前方 0.5 m 有障碍：减去车身前向半长 0.14，净空为 0.36。"""
        scan = FakeScan()
        scan.ranges[0] = 0.5
        assert footprint_clearance(scan, 0.14, 0.155) == pytest.approx(0.36)

    def test_side_obstacle_uses_width(self):
        """正左方 0.5 m 有障碍：减去车身半宽 0.155，净空为 0.345。"""
        scan = FakeScan()
        scan.ranges[90] = 0.5      # 正左方
        assert footprint_clearance(scan, 0.14, 0.155) == pytest.approx(0.345)

    def test_invalid_readings_ignored(self):
        """只有 nan 和小于 range_min 的读数：全部忽略，净空为无穷大。"""
        scan = FakeScan()
        scan.ranges[0] = math.nan
        scan.ranges[1] = 0.05      # 小于 range_min
        assert footprint_clearance(scan, 0.14, 0.155) == math.inf


class TestTripMonitor:
    """里程、接触、卡住统计。"""

    def setup_method(self):
        """每个测试前新建一个 TripMonitor。"""
        self.monitor = TripMonitor(contact_distance=0.03, stuck_time=10.0, stuck_distance=0.1)

    def test_distance_accumulates(self):
        """每秒沿 x 轴移到 0、1、1、3：累计里程 3.0，用时 3.0 s。"""
        for t, x in enumerate([0.0, 1.0, 1.0, 3.0]):
            self.monitor.update_position(float(t), x, 0.0)
        report = self.monitor.report()
        assert report.distance == pytest.approx(3.0)
        assert report.elapsed == pytest.approx(3.0)

    def test_contact_counted_once_per_entry(self):
        """净空两次进入接触距离以内：连续接触只算一次，共 2 次，最小净空 0.01。"""
        for clearance in [0.5, 0.02, 0.01, 0.2, 0.02]:
            self.monitor.update_clearance(clearance)
        report = self.monitor.report()
        assert report.contacts == 2
        assert report.min_clearance == pytest.approx(0.01)

    def test_moving_robot_is_not_stuck(self):
        """30 s 内持续移动：不判为卡住，验收通过。"""
        for t in range(30):
            self.monitor.update_position(float(t), 0.05 * t, 0.0)
        assert not self.monitor.report().stuck
        assert self.monitor.report().passed

    def test_stationary_robot_is_stuck(self):
        """30 s 内几乎不动：判为卡住，验收不通过。"""
        for t in range(30):
            self.monitor.update_position(float(t), 0.001 * t, 0.0)
        report = self.monitor.report()
        assert report.stuck
        assert not report.passed

    def test_empty_report(self):
        """没有任何数据就生成报告：用时为 0，验收通过。"""
        report = self.monitor.report()
        assert report.elapsed == 0.0
        assert report.passed
