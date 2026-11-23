"""
04 课 · 栅格地图的保存与统计（纯 Python，不依赖 ROS）。

SLAM 发布的地图是 nav_msgs/msg/OccupancyGrid：一个 width × height 的格子数组，
每格的值是"被占用的概率"×100：

    -1      未知（雷达从没扫到过）
    0       空闲（确定没有障碍）
    100     占用（确定有障碍）
    1 ~ 99  不确定（slam_toolbox 只输出 -1、0、100，其他 SLAM 算法可能输出中间值）

保存成文件时用 ROS 通用的"图片 + YAML"格式（Nav2 的 map_server 读的就是它）：

    xxx.pgm   灰度图，每格一个像素：白 254 = 空闲，黑 0 = 占用，灰 205 = 未知
    xxx.yaml  元数据：图片文件名、分辨率、原点、阈值

两个容易搞错的地方：
  1. 行的顺序：OccupancyGrid 的第 0 行在地图**最下面**（y 最小），
     图片的第 0 行在**最上面**，所以写图片时要上下翻转
  2. 阈值：占用概率 <= free_thresh 算空闲，>= occupied_thresh 算占用，其余算未知。
     默认值与 Jazzy 版 nav2 map_saver 相同，同样的地图两者写出的 .pgm 逐字节一致
"""

from dataclasses import dataclass
import math
import os

FREE_PIXEL = 254
OCCUPIED_PIXEL = 0
UNKNOWN_PIXEL = 205

DEFAULT_FREE_THRESH = 0.196      # 与 Jazzy 版 nav2 map_saver 的默认值相同
DEFAULT_OCCUPIED_THRESH = 0.65


@dataclass(frozen=True)
class GridMap:
    """
    栅格地图，字段与 OccupancyGrid 一一对应。

    data 按行存放，从地图左下角开始：格子 (col, row) 的值是 data[row * width + col]。
    原点 (origin_x, origin_y, origin_yaw) 是左下角格子的**角**在 map 坐标系中的位姿。
    """

    width: int                 # 格
    height: int                # 格
    resolution: float          # m/格
    origin_x: float            # m
    origin_y: float            # m
    origin_yaw: float          # rad，slam_toolbox 总是 0
    data: tuple                # int，-1 或 0 ~ 100

    def __post_init__(self):
        """检查尺寸与数据长度是否一致，不一致时抛出 ValueError。"""
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            raise ValueError('地图的宽、高、分辨率都必须大于 0')
        if len(self.data) != self.width * self.height:
            raise ValueError(
                f'数据长度 {len(self.data)} 与尺寸 {self.width} × {self.height} 不符')


@dataclass(frozen=True)
class MapStats:
    """地图统计：各类格子的数量和面积。"""

    free_cells: int
    occupied_cells: int
    unknown_cells: int
    cell_area: float           # m²/格

    @property
    def free_area(self):
        """空闲区域面积，m²。"""
        return self.free_cells * self.cell_area

    @property
    def known_ratio(self):
        """已知格子（空闲 + 占用）占全部格子的比例，0 ~ 1。"""
        total = self.free_cells + self.occupied_cells + self.unknown_cells
        return (self.free_cells + self.occupied_cells) / total


def check_thresholds(free_thresh, occupied_thresh):
    """检查阈值：要求 0 <= free_thresh < occupied_thresh <= 1，否则抛出 ValueError。"""
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError(
            f'要求 0 <= free_thresh < occupied_thresh <= 1，'
            f'实际为 {free_thresh}、{occupied_thresh}')


def cell_to_pixel(value, free_thresh=DEFAULT_FREE_THRESH,
                  occupied_thresh=DEFAULT_OCCUPIED_THRESH):
    """
    把一个格子的占用值转换成灰度（三值模式，与 nav2 map_saver 的 trinary 模式相同）。

    阈值先乘 100 再四舍五入成整数，和格子的值（整数）比较。

    :param value: 格子的值，-1 或 0 ~ 100
    :param free_thresh: 空闲阈值，0 ~ 1
    :param occupied_thresh: 占用阈值，0 ~ 1
    :return: 灰度 FREE_PIXEL / OCCUPIED_PIXEL / UNKNOWN_PIXEL
    """
    if value < 0 or value > 100:
        return UNKNOWN_PIXEL
    if value <= round(free_thresh * 100.0):
        return FREE_PIXEL
    if value >= round(occupied_thresh * 100.0):
        return OCCUPIED_PIXEL
    return UNKNOWN_PIXEL


def to_pgm(grid, free_thresh=DEFAULT_FREE_THRESH, occupied_thresh=DEFAULT_OCCUPIED_THRESH):
    """
    把地图编码成二进制 PGM（P5）图片。

    :param grid: GridMap
    :param free_thresh: 空闲阈值，0 ~ 1
    :param occupied_thresh: 占用阈值，0 ~ 1
    :return: bytes，可以直接写入 .pgm 文件
    """
    check_thresholds(free_thresh, occupied_thresh)
    header = f'P5\n{grid.width} {grid.height}\n255\n'.encode('ascii')
    pixels = bytearray(grid.width * grid.height)
    for image_row in range(grid.height):
        grid_row = grid.height - 1 - image_row          # 上下翻转：图片第 0 行是地图最上面一行
        src = grid_row * grid.width
        dst = image_row * grid.width
        for col in range(grid.width):
            pixels[dst + col] = cell_to_pixel(grid.data[src + col], free_thresh, occupied_thresh)
    return header + bytes(pixels)


def read_pgm(content):
    """
    解析二进制 PGM（P5，最大灰度 255）图片，支持头部的 # 注释。

    :param content: 文件内容 bytes
    :return: (width, height, pixels)，pixels 是从图片左上角开始逐行排列的 bytes
    """
    tokens, pos = [], 0
    while len(tokens) < 4:                              # 头部：P5、宽、高、最大灰度
        while content[pos:pos + 1].isspace():
            pos += 1
        if content[pos:pos + 1] == b'#':                # 注释一直到行尾
            pos = content.index(b'\n', pos) + 1
            continue
        end = pos
        while end < len(content) and not content[end:end + 1].isspace():
            end += 1
        tokens.append(content[pos:end])
        pos = end
    magic, width, height, max_value = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or max_value != 255:
        raise ValueError(f'只支持 P5、最大灰度 255 的 PGM，实际为 {magic!r}、{max_value}')
    pixels = content[pos + 1:pos + 1 + width * height]  # 最大灰度后面恰好一个空白字符
    if len(pixels) != width * height:
        raise ValueError('PGM 像素数据不完整')
    return width, height, pixels


def to_yaml(image_name, grid, free_thresh=DEFAULT_FREE_THRESH,
            occupied_thresh=DEFAULT_OCCUPIED_THRESH):
    """
    生成地图的 YAML 元数据（map_server 的格式）。

    :param image_name: 图片文件名。写相对路径（只写文件名），地图文件夹整体搬走也能用
    :param grid: GridMap
    :param free_thresh: 空闲阈值，0 ~ 1
    :param occupied_thresh: 占用阈值，0 ~ 1
    :return: str
    """
    return (f'image: {image_name}\n'
            f'mode: trinary\n'
            f'resolution: {grid.resolution:.3f}\n'
            f'origin: [{grid.origin_x:.3f}, {grid.origin_y:.3f}, {grid.origin_yaw:.3f}]\n'
            f'negate: 0\n'
            f'occupied_thresh: {occupied_thresh}\n'
            f'free_thresh: {free_thresh}\n')


def save_map(path_base, grid, free_thresh=DEFAULT_FREE_THRESH,
             occupied_thresh=DEFAULT_OCCUPIED_THRESH):
    """
    把地图保存成 <path_base>.pgm 和 <path_base>.yaml，目录不存在时自动创建。

    :param path_base: 不带扩展名的路径，例如 ~/maps/tb3_world
    :param grid: GridMap
    :param free_thresh: 空闲阈值，0 ~ 1
    :param occupied_thresh: 占用阈值，0 ~ 1
    :return: (pgm 路径, yaml 路径)
    """
    path_base = os.path.expanduser(path_base)
    directory = os.path.dirname(path_base)
    if directory:
        os.makedirs(directory, exist_ok=True)
    pgm_path, yaml_path = path_base + '.pgm', path_base + '.yaml'
    with open(pgm_path, 'wb') as f:
        f.write(to_pgm(grid, free_thresh, occupied_thresh))
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(to_yaml(os.path.basename(pgm_path), grid, free_thresh, occupied_thresh))
    return pgm_path, yaml_path


def map_stats(grid, free_thresh=DEFAULT_FREE_THRESH, occupied_thresh=DEFAULT_OCCUPIED_THRESH):
    """
    统计地图中空闲、占用、未知格子的数量（分类规则与 cell_to_pixel 相同）。

    :param grid: GridMap
    :param free_thresh: 空闲阈值，0 ~ 1
    :param occupied_thresh: 占用阈值，0 ~ 1
    :return: MapStats
    """
    counts = {FREE_PIXEL: 0, OCCUPIED_PIXEL: 0, UNKNOWN_PIXEL: 0}
    for value in grid.data:
        counts[cell_to_pixel(value, free_thresh, occupied_thresh)] += 1
    return MapStats(counts[FREE_PIXEL], counts[OCCUPIED_PIXEL], counts[UNKNOWN_PIXEL],
                    grid.resolution ** 2)


def yaw_from_quaternion(x, y, z, w):
    """四元数 → 绕 z 轴的转角 yaw，rad（推导见 03 课 geometry.py）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
