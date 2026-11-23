"""
05 课 · 巡逻航点：从 YAML 文件读取、检查，以及朝向角和四元数的转换。

航点文件的格式（见 config/waypoints.yaml）：

    frame_id: map
    waypoints:
      - {name: north, x: 1.45, y: 2.40, yaw_deg: 0}
      - {name: home,  x: 0.00, y: 0.00, yaw_deg: 180}

ROS 2 参数不支持"字典列表"，所以航点放在单独的 YAML 文件里，节点只接收文件路径。
朝向在文件里用角度（yaw_deg）写，读进来后统一换成弧度。

本模块不依赖 ROS，可以直接用 pytest 测试。
"""

from dataclasses import dataclass
import math

import yaml


@dataclass(frozen=True)
class Waypoint:
    """
    一个巡逻航点。

    :ivar name: 名字，只用于日志和统计，必须唯一
    :ivar x: 位置 x，m（frame_id 坐标系，通常是 map）
    :ivar y: 位置 y，m
    :ivar yaw: 到达时车头的朝向，rad，0 表示 x 轴正方向，逆时针为正
    """

    name: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Route:
    """
    一条巡逻路线：坐标系 + 按顺序排列的航点。

    :ivar frame_id: 航点所在的坐标系
    :ivar waypoints: 航点，按巡逻顺序排列，至少一个
    """

    frame_id: str
    waypoints: tuple


def _number(item, key, index):
    """取出航点的数字字段，不是数字时报出是第几个航点的哪个字段。"""
    value = item.get(key)
    # bool 是 int 的子类，要单独排除，否则 x: true 会被当成 1
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'第 {index + 1} 个航点的 {key} 必须是数字，实际是 {value!r}')
    if not math.isfinite(value):
        raise ValueError(f'第 {index + 1} 个航点的 {key} 不是有限的数：{value!r}')
    return float(value)


def parse_route(data):
    """
    把 YAML 读出来的字典转换成 Route，并检查格式。

    :param data: yaml.safe_load 的结果
    :return: Route
    :raises ValueError: 格式不对（缺字段、类型错、名字重复、航点为空）
    """
    if not isinstance(data, dict):
        raise ValueError('航点文件的顶层必须是字典（包含 frame_id 和 waypoints）')
    frame_id = data.get('frame_id', 'map')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError(f'frame_id 必须是非空字符串，实际是 {frame_id!r}')
    items = data.get('waypoints')
    if not isinstance(items, list) or not items:
        raise ValueError('waypoints 必须是非空列表')

    waypoints = []
    seen = set()
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f'第 {index + 1} 个航点必须是字典，实际是 {item!r}')
        name = item.get('name', f'wp{index + 1}')
        if not isinstance(name, str) or not name:
            raise ValueError(f'第 {index + 1} 个航点的 name 必须是非空字符串')
        if name in seen:
            raise ValueError(f'航点名字重复：{name}')
        seen.add(name)
        yaw_deg = _number(item, 'yaw_deg', index) if 'yaw_deg' in item else 0.0
        waypoints.append(Waypoint(
            name, _number(item, 'x', index), _number(item, 'y', index), math.radians(yaw_deg)))
    return Route(frame_id, tuple(waypoints))


def load_route(path):
    """
    从 YAML 文件读取巡逻路线。

    :param path: 文件路径
    :return: Route
    :raises OSError: 文件打不开
    :raises ValueError: 格式不对
    """
    with open(path, 'r', encoding='utf-8') as f:
        return parse_route(yaml.safe_load(f))


def yaw_to_quaternion(yaw):
    """
    平面朝向角 → 四元数 (x, y, z, w)。

    绕 z 轴转 yaw 的四元数是 (0, 0, sin(yaw/2), cos(yaw/2))；
    geometry_msgs/msg/Pose 的 orientation 就要这四个数。
    """
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
