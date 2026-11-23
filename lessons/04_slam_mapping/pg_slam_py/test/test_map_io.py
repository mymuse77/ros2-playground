"""map_io 的单元测试：灰度转换、上下翻转、PGM 读写、YAML、统计。"""

import math

from pg_slam_py.map_io import (cell_to_pixel, check_thresholds, FREE_PIXEL, GridMap,
                               map_stats, OCCUPIED_PIXEL, read_pgm, save_map, to_pgm, to_yaml,
                               UNKNOWN_PIXEL, yaw_from_quaternion)
import pytest


def make_grid(rows, resolution=0.05, origin=(0.0, 0.0, 0.0)):
    """
    按"图片的样子"构造地图：rows[0] 是最上面一行，方便在测试里直观地画地图。

    OccupancyGrid 的第 0 行在最下面，所以这里要反过来存。
    """
    height, width = len(rows), len(rows[0])
    data = tuple(value for row in reversed(rows) for value in row)
    return GridMap(width, height, resolution, *origin, data)


class TestCellToPixel:
    """单个格子的三值转换（默认阈值：空闲 <= 20，占用 >= 65）。"""

    @pytest.mark.parametrize('value, pixel', [
        (-1, UNKNOWN_PIXEL), (0, FREE_PIXEL), (20, FREE_PIXEL), (21, UNKNOWN_PIXEL),
        (64, UNKNOWN_PIXEL), (65, OCCUPIED_PIXEL), (100, OCCUPIED_PIXEL),
        (101, UNKNOWN_PIXEL)])
    def test_default_thresholds(self, value, pixel):
        """free_thresh 0.196 × 100 四舍五入为 20，与 nav2 map_saver 一致。"""
        assert cell_to_pixel(value) == pixel

    def test_custom_thresholds(self):
        """调高空闲阈值后，原来"不确定"的格子变成空闲。"""
        assert cell_to_pixel(40) == UNKNOWN_PIXEL
        assert cell_to_pixel(40, free_thresh=0.5, occupied_thresh=0.9) == FREE_PIXEL

    def test_invalid_thresholds(self):
        """空闲阈值必须小于占用阈值，且都在 0 ~ 1 之间。"""
        for free, occupied in [(0.7, 0.6), (0.5, 0.5), (-0.1, 0.5), (0.2, 1.5)]:
            with pytest.raises(ValueError):
                check_thresholds(free, occupied)


class TestGridMap:
    """GridMap 的参数检查。"""

    def test_data_length_must_match(self):
        """数据长度必须等于宽 × 高。"""
        with pytest.raises(ValueError):
            GridMap(3, 2, 0.05, 0.0, 0.0, 0.0, (0,) * 5)

    def test_size_must_be_positive(self):
        """宽、高、分辨率必须大于 0。"""
        with pytest.raises(ValueError):
            GridMap(0, 2, 0.05, 0.0, 0.0, 0.0, ())


class TestPgm:
    """PGM 编码与解析。"""

    def test_header(self):
        """头部格式与 nav2 map_saver 相同：P5、宽 高、255，各占一行。"""
        content = to_pgm(make_grid([[0, 0, 0], [0, 0, 0]]))
        assert content.startswith(b'P5\n3 2\n255\n')

    def test_rows_are_flipped(self):
        """地图最上面一行（y 最大）应该是图片的第 0 行。"""
        grid = make_grid([[100, -1],     # 上：障碍、未知
                          [0, 0]])       # 下：空闲
        assert grid.data == (0, 0, 100, -1)            # OccupancyGrid：从下往上存
        width, height, pixels = read_pgm(to_pgm(grid))
        assert (width, height) == (2, 2)
        assert pixels == bytes([OCCUPIED_PIXEL, UNKNOWN_PIXEL, FREE_PIXEL, FREE_PIXEL])

    def test_read_pgm_with_comment(self):
        """其他工具（如 GIMP）写出的 PGM 头部可能带注释。"""
        content = b'P5\n# created by test\n2 1\n255\n' + bytes([0, 254])
        assert read_pgm(content) == (2, 1, bytes([0, 254]))

    def test_read_pgm_rejects_other_formats(self):
        """只支持二进制灰度图（P5），文本格式（P2）要报错。"""
        with pytest.raises(ValueError):
            read_pgm(b'P2\n1 1\n255\n0\n')

    def test_read_pgm_truncated(self):
        """像素数据不完整时报错。"""
        with pytest.raises(ValueError):
            read_pgm(b'P5\n2 2\n255\n' + bytes([0, 0, 0]))


class TestYaml:
    """YAML 元数据。"""

    def test_fields(self):
        """原点、分辨率、阈值都要写进去，图片只写文件名。"""
        grid = make_grid([[0]], resolution=0.05, origin=(-0.953, -2.075, 0.0))
        text = to_yaml('tb3_world.pgm', grid)
        assert 'image: tb3_world.pgm\n' in text
        assert 'resolution: 0.050\n' in text
        assert 'origin: [-0.953, -2.075, 0.000]\n' in text
        assert 'occupied_thresh: 0.65\n' in text
        assert 'free_thresh: 0.196\n' in text
        assert 'mode: trinary\n' in text


class TestSaveAndStats:
    """保存文件与统计。"""

    def test_save_map(self, tmp_path):
        """保存后两个文件都存在，图片内容与 to_pgm 相同，目录不存在时自动创建。"""
        grid = make_grid([[0, 100], [-1, 0]])
        pgm, yaml = save_map(str(tmp_path / 'sub' / 'demo'), grid)
        with open(pgm, 'rb') as f:
            assert f.read() == to_pgm(grid)
        with open(yaml, encoding='utf-8') as f:
            assert f.readline() == 'image: demo.pgm\n'

    def test_stats(self):
        """2 空闲、1 障碍、1 未知；每格 0.1 m × 0.1 m。"""
        stats = map_stats(make_grid([[0, 100], [-1, 0]], resolution=0.1))
        assert (stats.free_cells, stats.occupied_cells, stats.unknown_cells) == (2, 1, 1)
        assert stats.free_area == pytest.approx(0.02)
        assert stats.known_ratio == pytest.approx(0.75)


@pytest.mark.parametrize('yaw', [0.0, 0.5, -2.0, math.pi / 2])
def test_yaw_from_quaternion(yaw):
    """绕 z 轴转 yaw 的四元数 (0, 0, sin(yaw/2), cos(yaw/2)) 应还原出 yaw。"""
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == \
        pytest.approx(yaw)
