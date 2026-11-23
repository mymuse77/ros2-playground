"""
05 课 · 路径评价指标：长度、转弯量、离障碍物的最小距离。

用于规划器对比实验（compare_planners）：同样的起点和终点，不同规划器给出的路径
有多长、拐了多少弯、离柱子有多近。

路径用 (x, y) 点的序列表示；障碍物用"被占用格子的中心点"表示。
本模块不依赖 ROS，可以直接用 pytest 测试。
"""

import math


def path_length(points):
    """
    路径总长度：相邻两点距离之和，m。

    :param points: [(x, y), ...]，少于两个点时长度为 0
    """
    return math.fsum(math.dist(a, b) for a, b in zip(points, points[1:]))


def total_turning(points, min_step=1e-6):
    """
    路径的累计转弯角度：每一段相对上一段的方向变化（取绝对值）之和，rad。

    直线为 0；绕过一根柱子大约是 π/2 ~ π。数值越小，路径越"直"。
    长度小于 min_step 的段（重复点）没有方向，跳过。

    :param points: [(x, y), ...]
    :param min_step: 小于此长度的段视为重复点，m
    """
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(points, points[1:]) if math.dist(a, b) > min_step]
    total = 0.0
    for h0, h1 in zip(headings, headings[1:]):
        # 方向差归一化到 (-π, π]，避免 179° → -179° 被算成 358°
        diff = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        total += abs(diff)
    return total


def min_clearance(points, obstacles):
    """
    路径上各点到最近障碍物的最小距离，m（以车中心计；减去车体半径才是车身的净空）。

    暴力计算：每个路径点和每个障碍点都比一次。路径几百个点、障碍几百个格子，足够快。

    :param points: 路径点 [(x, y), ...]
    :param obstacles: 障碍点 [(x, y), ...]
    :return: 最小距离；路径或障碍为空时为 math.inf
    """
    best = math.inf
    for p in points:
        for o in obstacles:
            d = math.dist(p, o)
            if d < best:
                best = d
    return best


def occupied_cells(width, height, resolution, origin_x, origin_y, data, threshold=65):
    """
    从占用栅格中取出所有"被占用"格子的中心点（map 坐标系）。

    :param width: 地图宽（格）
    :param height: 地图高（格）
    :param resolution: 每格边长，m
    :param origin_x: 左下角格子的角在 map 坐标系中的 x，m（不考虑旋转）
    :param origin_y: 左下角格子的角在 map 坐标系中的 y，m
    :param data: 按行存储的格子值，第 0 行在最下面（与 nav_msgs/OccupancyGrid 相同）
    :param threshold: 值 >= threshold 算占用；-1（未知）不算
    :return: [(x, y), ...]
    """
    if len(data) != width * height:
        raise ValueError(f'数据长度 {len(data)} 不等于 {width} × {height}')
    cells = []
    for index, value in enumerate(data):
        if value >= threshold:
            row, col = divmod(index, width)
            cells.append((origin_x + (col + 0.5) * resolution,
                          origin_y + (row + 0.5) * resolution))
    return cells
