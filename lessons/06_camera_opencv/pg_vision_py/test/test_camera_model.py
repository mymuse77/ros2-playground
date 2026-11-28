"""针孔相机模型的单元测试：方位角、距离估算、缩放换算（不需要启动 ROS）。"""

import math

from pg_vision_py.camera_model import Observation, observe, PinholeCamera
from pg_vision_py.color_detector import Detection
import pytest

# TurtleBot3 waffle 仿真相机：1920×1080，水平视场角 1.02974 rad
HFOV = 1.02974
CAMERA = PinholeCamera.from_fov(HFOV, 1920)


def detection(cx, w, image_width=320, touches_border=False):
    """构造一个检测结果；touches_border 为 True 时让外接矩形顶到左边缘。"""
    x = 0 if touches_border else int(cx - w / 2)
    return Detection(cx=cx, cy=90, area=w * w, x=x, y=60, w=w, h=w,
                     image_width=image_width, image_height=image_width * 9 // 16)


class TestPinholeCamera:
    """构造与换算。"""

    def test_focal_length_from_fov(self):
        """仿真相机的焦距约 1697 像素：(1920 / 2) / tan(1.02974 / 2)，可与 camera_info 的 K 矩阵对照。"""
        assert CAMERA.fx == pytest.approx(1697, abs=1)

    def test_matches_measured_ball_width(self):
        """
        与仿真截图实测对照：红球中心在车体前 2.0 m，相机在车体前 0.069 m，所以距相机 1.931 m。

        模型预测球宽 0.3 × fx / 1.931 ≈ 263.7 像素，截图里实测 264 像素。
        """
        assert 0.3 * CAMERA.fx / 1.931 == pytest.approx(264, abs=1)

    def test_fov_is_recovered_at_image_edge(self):
        """画面最右边（x=1920）的方位角恰好是视场角的一半。"""
        assert CAMERA.bearing(1920) == pytest.approx(HFOV / 2)

    @pytest.mark.parametrize('fov', [0.0, -1.0, math.pi, 4.0])
    def test_rejects_invalid_fov(self, fov):
        """视场角不在 (0, π) 内：抛 ValueError。"""
        with pytest.raises(ValueError):
            PinholeCamera.from_fov(fov, 640)

    @pytest.mark.parametrize('fx, width', [(0.0, 640), (-5.0, 640), (500.0, 0)])
    def test_rejects_invalid_parameters(self, fx, width):
        """焦距或图像宽度不是正数：抛 ValueError。"""
        with pytest.raises(ValueError):
            PinholeCamera(fx, width)

    def test_scaled_camera_keeps_the_same_view(self):
        """图像缩小到 1/6 后焦距也缩小到 1/6：同一个物体的方位角不变。"""
        small = CAMERA.scaled_to(320)
        assert small.fx == pytest.approx(CAMERA.fx / 6)
        assert small.bearing(240) == pytest.approx(CAMERA.bearing(240 * 6))


class TestBearing:
    """方位角。"""

    def test_center_is_zero(self):
        """目标在画面正中：方位角为 0。"""
        assert CAMERA.bearing(960) == 0.0

    def test_right_is_positive_left_is_negative(self):
        """右侧为正、左侧为负，且左右对称。"""
        assert CAMERA.bearing(1500) > 0
        assert CAMERA.bearing(1500) == pytest.approx(-CAMERA.bearing(420))

    def test_matches_geometry(self):
        """偏离中心 fx 个像素：方位角恰好是 45°（tan 45° = 1）。"""
        assert CAMERA.bearing(960 + CAMERA.fx) == pytest.approx(math.radians(45))


class TestDistance:
    """距离估算。"""

    def test_ball_two_meters_away(self):
        """场景里的红球（宽 0.3 m）在相机前 2 m 处，原图中宽约 255 像素。"""
        assert CAMERA.distance(255, 0.3) == pytest.approx(2.0, rel=0.01)

    def test_inverse_proportion(self):
        """像素宽度翻倍，距离减半。"""
        assert CAMERA.distance(200, 0.3) == pytest.approx(2 * CAMERA.distance(400, 0.3))

    @pytest.mark.parametrize('pixel_width, real_width', [(0, 0.3), (-1, 0.3), (50, 0)])
    def test_rejects_non_positive_widths(self, pixel_width, real_width):
        """像素宽度或真实宽度不是正数：抛 ValueError，避免除以 0。"""
        with pytest.raises(ValueError):
            CAMERA.distance(pixel_width, real_width)


class TestObserve:
    """把检测结果换算成观测量。"""

    def test_no_detection_gives_none(self):
        """没有检测到目标：返回 None。"""
        assert observe(None, CAMERA, 0.3) is None

    def test_uses_scaled_focal_length(self):
        """检测在 320 宽的缩小图上做：宽 42.5 像素的球，距离约 2 m，正对中心。"""
        result = observe(detection(cx=160, w=42.5), CAMERA, 0.3)
        assert result.bearing == pytest.approx(0.0)
        assert result.distance == pytest.approx(2.0, rel=0.01)

    def test_bearing_sign_follows_image_side(self):
        """目标在画面右半边：方位角为正。"""
        assert observe(detection(cx=250, w=40), CAMERA, 0.3).bearing > 0
        assert observe(detection(cx=70, w=40), CAMERA, 0.3).bearing < 0

    def test_distance_unknown_when_touching_border(self):
        """目标贴着画面边缘（可能被截断）：方位角照给，距离为 None。"""
        result = observe(detection(cx=20, w=40, touches_border=True), CAMERA, 0.3)
        assert result.distance is None
        assert result.bearing < 0

    def test_observation_is_plain_data(self):
        """Observation 是不可变的普通数据，可以直接比较。"""
        assert Observation(0.1, 2.0) == Observation(0.1, 2.0)
