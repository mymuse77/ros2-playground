"""
02 课 · 把扇区分析结果画成 RViz 标记（MarkerArray）。

每个扇区画成一个扇形：半径 = 该扇区的最近距离，颜色表示危险程度
（绿：开阔，黄：减速区，红：停止区）。车头上方再显示一行状态文字。
在 RViz 里添加 MarkerArray 显示，话题选 /avoider/sectors 即可看到。

只依赖消息类型，不依赖 rclpy，几何计算部分可以直接单元测试。
"""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

ARC_STEP = math.radians(5.0)   # 画圆弧时相邻两点的角度间隔
FILL_ALPHA = 0.35              # 扇形填充的不透明度
TEXT_HEIGHT = 0.12             # 状态文字高度，m
TEXT_Z = 0.4                   # 状态文字离地高度（相对雷达），m

GREEN = (0.2, 0.8, 0.2)
YELLOW = (0.95, 0.8, 0.1)
RED = (0.9, 0.15, 0.15)


def arc_points(start, end, radius, step=ARC_STEP):
    """
    计算扇形边界圆弧上的点（不含圆心）。

    :param start: 起始角度，rad
    :param end: 结束角度，rad，要求 end >= start
    :param radius: 半径，m
    :param step: 相邻两点的最大角度间隔，rad
    :return: [(x, y), ...]，从 start 到 end，首尾两点正好落在扇形的两条边上
    """
    count = max(1, math.ceil((end - start) / step))
    return [(radius * math.cos(start + (end - start) * i / count),
             radius * math.sin(start + (end - start) * i / count))
            for i in range(count + 1)]


def fan_triangles(start, end, radius):
    """
    把扇形拆成以圆心为公共顶点的三角形，供 TRIANGLE_LIST 使用。

    :return: [(x, y), ...]，每 3 个点构成一个三角形
    """
    points = arc_points(start, end, radius)
    triangles = []
    for a, b in zip(points, points[1:]):
        triangles.extend([(0.0, 0.0), a, b])
    return triangles


def distance_color(distance, stop_distance, slow_distance):
    """
    根据距离选择颜色。

    :return: (r, g, b)，每个分量 0 ~ 1
    """
    if distance < stop_distance:
        return RED
    if distance < slow_distance:
        return YELLOW
    return GREEN


def build_markers(summary, params, state_text, frame_id, stamp):
    """
    生成扇区标记。

    :param summary: logic.ScanSummary，三个扇区的最近距离
    :param params: logic.AvoidanceParams，用到扇区角度和距离阈值
    :param state_text: 显示在车头上方的状态文字
    :param frame_id: 标记所在坐标系，应与雷达坐标系一致（扇形以雷达为圆心）
    :param stamp: 时间戳（builtin_interfaces/Time）
    :return: visualization_msgs/msg/MarkerArray
    """
    half, side = params.front_half_angle, params.side_angle
    sectors = [
        ('front', -half, half, summary.front),
        ('front_left', half, side, summary.front_left),
        ('front_right', -side, -half, summary.front_right),
    ]

    markers = MarkerArray()
    for marker_id, (name, start, end, distance) in enumerate(sectors):
        marker = _new_marker(frame_id, stamp, marker_id, name, Marker.TRIANGLE_LIST)
        # TRIANGLE_LIST 的 scale 是缩放系数，必须设为 1，否则什么都看不到
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        r, g, b = distance_color(distance, params.stop_distance, params.slow_distance)
        marker.color = ColorRGBA(r=r, g=g, b=b, a=FILL_ALPHA)
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in fan_triangles(start, end, distance)]
        markers.markers.append(marker)

    text = _new_marker(frame_id, stamp, len(sectors), 'state', Marker.TEXT_VIEW_FACING)
    text.pose.position.z = TEXT_Z
    text.scale.z = TEXT_HEIGHT
    text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    text.text = state_text
    markers.markers.append(text)
    return markers


def _new_marker(frame_id, stamp, marker_id, namespace, marker_type):
    """创建一个公共字段已填好的 Marker。同一 ns + id 的标记会被新消息覆盖。"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker
