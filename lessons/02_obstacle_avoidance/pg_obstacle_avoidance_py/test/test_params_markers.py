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
        """默认参数转为 ROS 参数：角度以 *_deg 名称按度暴露，不含弧度字段。"""
        values = to_ros_params(AvoidanceParams())
        assert values['front_half_angle_deg'] == pytest.approx(30.0)
        assert values['side_angle_deg'] == pytest.approx(90.0)
        assert 'front_half_angle' not in values

    def test_round_trip(self):
        """自定义参数转为 ROS 参数再转回：得到与原来相等的 AvoidanceParams。"""
        params = AvoidanceParams(max_linear_speed=0.15, front_half_angle=math.radians(25))
        assert from_ros_params(to_ros_params(params)) == params

    def test_partial_values_use_defaults(self):
        """只提供 stop_distance：该值生效，其余参数取默认值。"""
        params = from_ros_params({'stop_distance': 0.4})
        assert params.stop_distance == pytest.approx(0.4)
        assert params.clear_distance == AvoidanceParams().clear_distance

    def test_integer_values_are_accepted(self):
        """参数值为整数（turn_speed=1）：被接受，值等于 1.0。"""
        assert from_ros_params({'turn_speed': 1}).turn_speed == 1.0

    def test_invalid_combination_raises(self):
        """stop_distance 大于默认 clear_distance：抛出 ValueError。"""
        with pytest.raises(ValueError):
            from_ros_params({'stop_distance': 0.9})   # 大于默认的 clear_distance

    def test_unknown_name_raises(self):
        """包含未知参数名：抛出 TypeError。"""
        with pytest.raises(TypeError):
            from_ros_params({'no_such_param': 1.0})


class TestGeometry:
    """扇形几何计算。"""

    def test_arc_endpoints_on_sector_edges(self):
        """0 到 π/2、半径 2 的圆弧：首点为 (2, 0)，末点为 (0, 2)。"""
        points = arc_points(0.0, math.pi / 2, 2.0)
        assert points[0] == pytest.approx((2.0, 0.0))
        assert points[-1][0] == pytest.approx(0.0, abs=1e-9)
        assert points[-1][1] == pytest.approx(2.0)

    def test_arc_points_on_circle(self):
        """半径 1.5 的圆弧：每个点到原点的距离都等于 1.5。"""
        for x, y in arc_points(-0.5, 0.5, 1.5):
            assert math.hypot(x, y) == pytest.approx(1.5)

    def test_fan_is_triangles_from_origin(self):
        """生成扇形三角形：点数是 3 的倍数，每个三角形的第一个点都是原点。"""
        triangles = fan_triangles(0.0, math.radians(60), 1.0)
        assert len(triangles) % 3 == 0
        assert all(p == (0.0, 0.0) for p in triangles[::3])

    @pytest.mark.parametrize('distance, expected', [(0.3, RED), (0.7, YELLOW), (2.0, GREEN)])
    def test_distance_color(self, distance, expected):
        """阈值为 0.5 和 1.0 时，距离 0.3、0.7、2.0：分别显示红、黄、绿色。"""
        assert distance_color(distance, 0.5, 1.0) == expected


def test_build_markers():
    """前方 0.4 m、状态 TURNING：生成三个扇区和状态文字共四个标记，前方扇区为红色三角形列表。"""
    params = AvoidanceParams()
    summary = ScanSummary(front=0.4, front_left=1.5, front_right=0.8)
    markers = build_markers(summary, params, 'TURNING', 'base_scan', Time()).markers
    assert [m.ns for m in markers] == ['front', 'front_left', 'front_right', 'state']
    assert all(m.header.frame_id == 'base_scan' for m in markers)
    assert markers[0].type == Marker.TRIANGLE_LIST
    assert (markers[0].color.r, markers[0].color.g) == pytest.approx(RED[:2])
    assert markers[3].text == 'TURNING'
