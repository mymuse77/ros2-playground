"""ROS 参数换算与 RViz 标记几何计算的单元测试。"""

import math

from builtin_interfaces.msg import Time
from pg_obstacle_avoidance_py.logic import (AvoidanceParams, from_ros_params, ScanSummary,
                                            to_ros_params)
from pg_obstacle_avoidance_py.markers import (arc_points, build_markers, distance_color,
                                              fan_triangles, GREEN, RED, YELLOW)
import pytest
from visualization_msgs.msg import Marker


class TestRosParams:
    """AvoidanceParams 与 ROS 参数字典的互相转换。"""

    def test_angles_exposed_in_degrees(self):
        values = to_ros_params(AvoidanceParams())
        assert values['front_half_angle_deg'] == pytest.approx(30.0)
        assert values['side_angle_deg'] == pytest.approx(90.0)
        assert 'front_half_angle' not in values

    def test_round_trip(self):
        params = AvoidanceParams(max_linear_speed=0.15, front_half_angle=math.radians(25))
        assert from_ros_params(to_ros_params(params)) == params

    def test_partial_values_use_defaults(self):
        params = from_ros_params({'stop_distance': 0.4})
        assert params.stop_distance == pytest.approx(0.4)
        assert params.clear_distance == AvoidanceParams().clear_distance

    def test_integer_values_are_accepted(self):
        assert from_ros_params({'turn_speed': 1}).turn_speed == 1.0

    def test_invalid_combination_raises(self):
        with pytest.raises(ValueError):
            from_ros_params({'stop_distance': 0.9})   # 大于默认的 clear_distance

    def test_unknown_name_raises(self):
        with pytest.raises(TypeError):
            from_ros_params({'no_such_param': 1.0})


class TestGeometry:
    """扇形几何计算。"""

    def test_arc_endpoints_on_sector_edges(self):
        points = arc_points(0.0, math.pi / 2, 2.0)
        assert points[0] == pytest.approx((2.0, 0.0))
        assert points[-1][0] == pytest.approx(0.0, abs=1e-9)
        assert points[-1][1] == pytest.approx(2.0)

    def test_arc_points_on_circle(self):
        for x, y in arc_points(-0.5, 0.5, 1.5):
            assert math.hypot(x, y) == pytest.approx(1.5)

    def test_fan_is_triangles_from_origin(self):
        triangles = fan_triangles(0.0, math.radians(60), 1.0)
        assert len(triangles) % 3 == 0
        assert all(p == (0.0, 0.0) for p in triangles[::3])

    @pytest.mark.parametrize('distance, expected', [(0.3, RED), (0.7, YELLOW), (2.0, GREEN)])
    def test_distance_color(self, distance, expected):
        assert distance_color(distance, 0.5, 1.0) == expected


def test_build_markers():
    params = AvoidanceParams()
    summary = ScanSummary(front=0.4, front_left=1.5, front_right=0.8)
    markers = build_markers(summary, params, 'TURNING', 'base_scan', Time()).markers
    assert [m.ns for m in markers] == ['front', 'front_left', 'front_right', 'state']
    assert all(m.header.frame_id == 'base_scan' for m in markers)
    assert markers[0].type == Marker.TRIANGLE_LIST
    assert (markers[0].color.r, markers[0].color.g) == pytest.approx(RED[:2])
    assert markers[3].text == 'TURNING'
