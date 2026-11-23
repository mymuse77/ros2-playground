"""几何工具函数的单元测试。"""

import math

from pg_go_to_goal_py.geometry import (distance, distance_to_line, heading_error,
                                       normalize_angle, Pose2D, yaw_from_quaternion)
import pytest


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0), (1.5 * math.pi, -0.5 * math.pi), (-1.5 * math.pi, 0.5 * math.pi)])
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


@pytest.mark.parametrize('yaw', [0.0, 0.5, -2.0, 3.0])
def test_yaw_from_quaternion(yaw):
    # 绕 z 轴旋转 yaw 的四元数：(0, 0, sin(yaw/2), cos(yaw/2))
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert yaw_from_quaternion(*q) == pytest.approx(yaw)


def test_distance():
    assert distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)


def test_heading_error_left_positive():
    pose = Pose2D(0.0, 0.0, 0.0)
    assert heading_error(pose, 0.0, 1.0) == pytest.approx(math.pi / 2)
    assert heading_error(pose, 0.0, -1.0) == pytest.approx(-math.pi / 2)


def test_heading_error_wraps():
    # 朝向 170°，目标在 -170° 方向 → 只需再向左转 20°
    pose = Pose2D(0.0, 0.0, math.radians(170))
    target = (math.cos(math.radians(-170)), math.sin(math.radians(-170)))
    assert heading_error(pose, *target) == pytest.approx(math.radians(20))


def test_distance_to_line_sign():
    # 直线从 (0,0) 指向 (1,0)：上方（左侧）为正，下方为负
    assert distance_to_line(0.5, 0.3, 0.0, 0.0, 1.0, 0.0) == pytest.approx(0.3)
    assert distance_to_line(0.5, -0.2, 0.0, 0.0, 1.0, 0.0) == pytest.approx(-0.2)


def test_distance_to_degenerate_line():
    # 起点终点重合时退化为点到点距离
    assert distance_to_line(3.0, 4.0, 0.0, 0.0, 0.0, 0.0) == pytest.approx(5.0)
