"""
03 课 · 平面几何工具（纯 Python，不依赖 ROS）。

坐标约定（ROS REP-103）：x 向前、y 向左，角度逆时针为正，单位 m 和 rad。
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """平面位姿：位置 (x, y)，朝向 yaw（与 x 轴的夹角，逆时针为正）。"""

    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    """把角度归一化到 [-pi, pi)，单位 rad。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x, y, z, w):
    """
    从四元数中取出绕 z 轴的转角 yaw。

    ROS 消息里的朝向（geometry_msgs/Quaternion）用四元数表示，
    平面移动机器人只关心 yaw。公式来自四元数转欧拉角（ZYX 顺序）。

    :return: yaw，rad，范围 [-pi, pi]
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distance(x1, y1, x2, y2):
    """两点之间的直线距离，m。"""
    return math.hypot(x2 - x1, y2 - y1)


def heading_error(pose, goal_x, goal_y):
    """
    朝向误差：目标方向与当前朝向的夹角。

    :param pose: 当前位姿 Pose2D
    :return: rad，范围 [-pi, pi)；正值表示目标在左边，应向左转
    """
    bearing = math.atan2(goal_y - pose.y, goal_x - pose.x)
    return normalize_angle(bearing - pose.yaw)


def distance_to_line(px, py, ax, ay, bx, by):
    """
    点 P 到直线 AB 的有符号距离。

    :return: m；P 在 A→B 方向的左侧为正，右侧为负。A、B 重合时返回 P 到 A 的距离
    """
    length = distance(ax, ay, bx, by)
    if length == 0.0:
        return distance(px, py, ax, ay)
    # 二维叉积 (B-A) × (P-A) 除以 |AB|
    return ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) / length
