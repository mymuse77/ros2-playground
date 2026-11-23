"""到点控制器（比例控制）的单元测试。"""

import math

from pg_go_to_goal_py.geometry import distance, normalize_angle, Pose2D
from pg_go_to_goal_py.go_to_goal import compute_go_to_goal, GoToGoalParams, STOP
import pytest

PARAMS = GoToGoalParams()
ORIGIN = Pose2D(0.0, 0.0, 0.0)


def test_reached_within_tolerance():
    """目标在 0.05 m 处（默认容差以内）：判定到达并输出 STOP。"""
    command, reached = compute_go_to_goal(ORIGIN, 0.05, 0.0, PARAMS)
    assert reached and command == STOP


def test_custom_tolerance():
    """目标在 0.3 m 处、自定义容差 0.5 m：判定到达。"""
    _, reached = compute_go_to_goal(ORIGIN, 0.3, 0.0, PARAMS, 0.5)
    assert reached


def test_straight_ahead_far_is_capped():
    """目标在正前方 5 m：未到达，线速度被限幅为 max_linear_speed，角速度为 0。"""
    command, reached = compute_go_to_goal(ORIGIN, 5.0, 0.0, PARAMS)
    assert not reached
    assert command.linear == pytest.approx(PARAMS.max_linear_speed)
    assert command.angular == pytest.approx(0.0)


def test_slows_down_near_goal():
    """目标在正前方 0.2 m：线速度按比例减为 k_linear * 0.2。"""
    command, _ = compute_go_to_goal(ORIGIN, 0.2, 0.0, PARAMS)
    assert command.linear == pytest.approx(PARAMS.k_linear * 0.2)


def test_rotates_in_place_when_goal_behind():
    """目标在身后偏左：原地转向，线速度为 0，角速度为 max_angular_speed。"""
    command, _ = compute_go_to_goal(ORIGIN, -2.0, 0.1, PARAMS)
    assert command.linear == 0.0
    assert command.angular == pytest.approx(PARAMS.max_angular_speed)


def test_small_heading_error_moves_and_turns():
    """目标在左前方 30°：边走边向左转，线速度乘以 cos(30°)。"""
    goal = (math.cos(math.radians(30)) * 2, math.sin(math.radians(30)) * 2)
    command, _ = compute_go_to_goal(ORIGIN, *goal, PARAMS)
    assert command.linear == pytest.approx(PARAMS.max_linear_speed * math.cos(math.radians(30)))
    assert command.angular > 0.0


def test_invalid_params():
    """k_linear 为 0 或 rotate_threshold 过大（4.0）：抛出 ValueError。"""
    with pytest.raises(ValueError):
        GoToGoalParams(k_linear=0.0)
    with pytest.raises(ValueError):
        GoToGoalParams(rotate_threshold=4.0)


def test_closed_loop_reaches_goal():
    """简单运动学仿真：反复执行控制器，小车应能到达身后的目标点。"""
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    for _ in range(2000):
        command, reached = compute_go_to_goal(Pose2D(x, y, yaw), -1.0, 1.5, PARAMS)
        if reached:
            break
        x += command.linear * math.cos(yaw) * dt
        y += command.linear * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + command.angular * dt)
    assert reached
    assert distance(x, y, -1.0, 1.5) < PARAMS.goal_tolerance
