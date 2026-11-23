"""waypoints 的单元测试：航点文件解析、格式检查、四元数。"""

import math
import os

from pg_nav2_py.waypoints import load_route, parse_route, Waypoint, yaw_to_quaternion
import pytest

# 包里自带的航点文件（config/ 与 test/ 同级）
PACKAGE_WAYPOINTS = os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')


class TestParseRoute:
    """把 YAML 字典转换成 Route。"""

    def test_basic(self):
        """角度换成弧度，坐标换成 float。"""
        route = parse_route({'frame_id': 'map', 'waypoints': [
            {'name': 'a', 'x': 1, 'y': 2.5, 'yaw_deg': 90},
            {'name': 'b', 'x': -1.0, 'y': 0}]})
        assert route.frame_id == 'map'
        assert route.waypoints[0] == Waypoint('a', 1.0, 2.5, math.pi / 2)
        assert route.waypoints[1] == Waypoint('b', -1.0, 0.0, 0.0)
        assert isinstance(route.waypoints[0].x, float)

    def test_defaults(self):
        """frame_id 默认 map，name 默认 wp1、wp2 ……，yaw_deg 默认 0。"""
        route = parse_route({'waypoints': [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}]})
        assert route.frame_id == 'map'
        assert [w.name for w in route.waypoints] == ['wp1', 'wp2']
        assert route.waypoints[0].yaw == 0.0

    @pytest.mark.parametrize('data', [
        None,                                              # 空文件
        [],                                                # 顶层不是字典
        {'frame_id': 'map'},                               # 没有 waypoints
        {'waypoints': []},                                 # 航点为空
        {'waypoints': 'a'},                                # waypoints 不是列表
        {'frame_id': '', 'waypoints': [{'x': 0, 'y': 0}]},  # frame_id 为空
        {'waypoints': [[0, 0]]},                           # 航点不是字典
        {'waypoints': [{'x': 0}]},                         # 缺 y
        {'waypoints': [{'x': '1', 'y': 0}]},               # 字符串不是数字
        {'waypoints': [{'x': True, 'y': 0}]},              # bool 不当成数字
        {'waypoints': [{'x': float('nan'), 'y': 0}]},      # NaN
        {'waypoints': [{'x': 0, 'y': 0, 'yaw_deg': None}]},  # 写了 yaw_deg 但没有值
        {'waypoints': [{'name': 'a', 'x': 0, 'y': 0},
                       {'name': 'a', 'x': 1, 'y': 1}]},     # 名字重复
    ])
    def test_invalid(self, data):
        """格式不对时报 ValueError，而不是等到导航时才出错。"""
        with pytest.raises(ValueError):
            parse_route(data)

    def test_error_message_names_the_waypoint(self):
        """错误信息指出是第几个航点的哪个字段，方便改文件。"""
        with pytest.raises(ValueError, match='第 2 个航点的 y'):
            parse_route({'waypoints': [{'x': 0, 'y': 0}, {'x': 1, 'y': 'abc'}]})


class TestLoadRoute:
    """从文件读取。"""

    def test_package_file(self):
        """包里自带的航点文件能正确读取，且最后回到原点（巡逻一圈回到出发点）。"""
        route = load_route(PACKAGE_WAYPOINTS)
        assert route.frame_id == 'map'
        assert len(route.waypoints) >= 2
        last = route.waypoints[-1]
        assert (last.x, last.y) == (0.0, 0.0)

    def test_roundtrip(self, tmp_path):
        """写一个临时文件再读回来。"""
        path = tmp_path / 'wp.yaml'
        path.write_text('waypoints:\n  - {name: p, x: 1.5, y: -2, yaw_deg: 180}\n',
                        encoding='utf-8')
        assert load_route(str(path)).waypoints == (Waypoint('p', 1.5, -2.0, math.pi),)

    def test_missing_file(self, tmp_path):
        """文件不存在时报 OSError。"""
        with pytest.raises(OSError):
            load_route(str(tmp_path / 'nope.yaml'))


class TestYawToQuaternion:
    """朝向角 → 四元数。"""

    @pytest.mark.parametrize('yaw, z, w', [
        (0.0, 0.0, 1.0),
        (math.pi / 2, math.sqrt(0.5), math.sqrt(0.5)),
        (math.pi, 1.0, 0.0),
        (-math.pi / 2, -math.sqrt(0.5), math.sqrt(0.5)),
    ])
    def test_values(self, yaw, z, w):
        """绕 z 轴转 yaw：(0, 0, sin(yaw/2), cos(yaw/2))。"""
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        assert (qx, qy) == (0.0, 0.0)
        assert qz == pytest.approx(z, abs=1e-12)
        assert qw == pytest.approx(w, abs=1e-12)

    def test_unit_length(self):
        """四元数的模长为 1。"""
        for yaw in [0.3, 1.7, -2.9, 6.0]:
            q = yaw_to_quaternion(yaw)
            assert math.fsum(c * c for c in q) == pytest.approx(1.0)
