"""path_metrics 的单元测试：长度、转弯量、最小距离、占用格子。"""

import math

from pg_nav2_py.path_metrics import min_clearance, occupied_cells, path_length, total_turning
import pytest


class TestPathLength:
    """路径长度。"""

    def test_polyline(self):
        """3-4-5 三角形的两条直角边。"""
        assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)

    @pytest.mark.parametrize('points', [[], [(1, 2)]])
    def test_short(self, points):
        """少于两个点时长度为 0。"""
        assert path_length(points) == 0.0


class TestTotalTurning:
    """累计转弯角度。"""

    def test_straight(self):
        """直线不转弯。"""
        assert total_turning([(0, 0), (1, 0), (2, 0), (3, 0)]) == pytest.approx(0.0)

    def test_right_angle(self):
        """一个直角弯：π/2。"""
        assert total_turning([(0, 0), (1, 0), (1, 1)]) == pytest.approx(math.pi / 2)

    def test_zigzag_adds_up(self):
        """左转再右转，绝对值相加，不会互相抵消。"""
        assert total_turning([(0, 0), (1, 0), (1, 1), (2, 1)]) == pytest.approx(math.pi)

    def test_wraparound(self):
        """方向从 170° 变到 -170°，实际只转了 20°，不是 340°。"""
        a = math.radians(170)
        b = math.radians(-170)
        points = [(0, 0), (math.cos(a), math.sin(a)),
                  (math.cos(a) + math.cos(b), math.sin(a) + math.sin(b))]
        assert total_turning(points) == pytest.approx(math.radians(20))

    def test_duplicate_points_ignored(self):
        """重复点没有方向，跳过，不影响结果。"""
        assert total_turning([(0, 0), (1, 0), (1, 0), (2, 0)]) == pytest.approx(0.0)


class TestMinClearance:
    """离障碍物的最小距离。"""

    def test_basic(self):
        """取所有路径点中离障碍最近的那个。"""
        path = [(0, 0), (1, 0), (2, 0)]
        assert min_clearance(path, [(1, 0.5), (5, 5)]) == pytest.approx(0.5)

    def test_empty(self):
        """没有障碍时为无穷大。"""
        assert min_clearance([(0, 0)], []) == math.inf


class TestOccupiedCells:
    """从占用栅格取出障碍格子的中心。"""

    def test_centers_and_row_order(self):
        """第 0 行在最下面；未知（-1）和低于阈值的不算。"""
        # 2 × 2 地图，分辨率 1 m，原点 (10, 20)
        #   第 1 行（上）:  -1  100
        #   第 0 行（下）: 100   50
        data = [100, 50, -1, 100]
        cells = occupied_cells(2, 2, 1.0, 10.0, 20.0, data)
        assert cells == [(10.5, 20.5), (11.5, 21.5)]

    def test_threshold(self):
        """阈值可调。"""
        assert len(occupied_cells(2, 1, 1.0, 0.0, 0.0, [50, 100], threshold=50)) == 2

    def test_length_mismatch(self):
        """数据长度必须等于宽 × 高。"""
        with pytest.raises(ValueError):
            occupied_cells(2, 2, 1.0, 0.0, 0.0, [0, 0, 0])
