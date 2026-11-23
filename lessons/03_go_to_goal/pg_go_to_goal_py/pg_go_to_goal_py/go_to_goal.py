"""
03 课 · 到点控制器：比例控制（P 控制），纯 Python，不依赖 ROS。

思路很简单：误差越大，修正得越猛。
  角速度 = k_angular × 朝向误差
  线速度 = k_linear  × 距离误差

再加两条规则，让它在真实车上更好用：
  1. 朝向误差太大（超过 rotate_threshold）时先原地转向，不往前走。
     否则目标在身后时会画一个很大的弧线
  2. 线速度乘以 cos(朝向误差)：方向越偏，前进越慢，避免"冲过头"
"""

from dataclasses import dataclass
import math

from pg_go_to_goal_py.geometry import distance, heading_error


@dataclass(frozen=True)
class GoToGoalParams:
    """到点控制参数，单位 m、rad、m/s、rad/s。"""

    k_linear: float = 0.5                         # 线速度增益，(m/s)/m
    k_angular: float = 1.5                        # 角速度增益，(rad/s)/rad
    max_linear_speed: float = 0.22                # 线速度上限
    max_angular_speed: float = 1.0                # 角速度上限
    goal_tolerance: float = 0.10                  # 距离小于此值即视为到达
    rotate_threshold: float = math.radians(45.0)  # 朝向误差超过此值时原地转向

    def __post_init__(self):
        """检查参数取值，不满足时抛出 ValueError。"""
        if min(self.k_linear, self.k_angular, self.max_linear_speed,
               self.max_angular_speed, self.goal_tolerance) <= 0.0:
            raise ValueError('增益、速度上限、到达容差都必须大于 0')
        if not 0.0 < self.rotate_threshold <= math.pi:
            raise ValueError('rotate_threshold 必须在 (0, pi] 之间')


@dataclass(frozen=True)
class Command:
    """速度指令：linear 前进速度 m/s，angular 角速度 rad/s（正值向左转）。"""

    linear: float
    angular: float


STOP = Command(0.0, 0.0)


def _clamp(value, limit):
    """把 value 限制在 [-limit, limit] 区间内。"""
    return max(-limit, min(limit, value))


def compute_go_to_goal(pose, goal_x, goal_y, params, tolerance=None):
    """
    计算朝目标点前进的速度指令。

    :param pose: 当前位姿 Pose2D
    :param goal_x: 目标 x，m
    :param goal_y: 目标 y，m
    :param params: GoToGoalParams
    :param tolerance: 到达容差，m；None 表示使用 params.goal_tolerance
    :return: (Command, 是否已到达)
    """
    tolerance = params.goal_tolerance if tolerance is None else tolerance
    dist = distance(pose.x, pose.y, goal_x, goal_y)
    if dist < tolerance:
        return STOP, True

    error = heading_error(pose, goal_x, goal_y)
    angular = _clamp(params.k_angular * error, params.max_angular_speed)
    if abs(error) > params.rotate_threshold:
        return Command(0.0, angular), False

    linear = min(params.k_linear * dist, params.max_linear_speed) * math.cos(error)
    return Command(linear, angular), False
