"""颜色检测的单元测试：用 numpy 画出合成图片，验证分割、定位和干扰排除（不需要启动 ROS）。"""

import cv2
import numpy as np
from pg_vision_py.color_detector import (COLOR_PRESETS, convert_to_bgr, detect, Detection,
                                         draw_detection, find_target, HsvRange,
                                         resize_to_width, segment)
import pytest

WIDTH, HEIGHT = 320, 180
RED, ORANGE = COLOR_PRESETS['red'], COLOR_PRESETS['orange']


def hsv_to_bgr(h, s=200, v=200):
    """把一个 HSV 颜色换成 BGR 三元组（用来在合成图上画指定色相的物体）。"""
    pixel = cv2.cvtColor(np.array([[[h, s, v]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)
    return tuple(int(c) for c in pixel[0, 0])


def blank_image():
    """浅灰色背景（S=0，不属于任何颜色），尺寸 320×180。"""
    return np.full((HEIGHT, WIDTH, 3), 200, dtype=np.uint8)


def with_circle(image, center, radius, hue, s=200, v=200):
    """在图像上画一个实心圆，返回同一张图便于链式调用。"""
    cv2.circle(image, center, radius, hsv_to_bgr(hue, s, v), thickness=-1)
    return image


# ============================================================ HsvRange

class TestHsvRange:
    """颜色范围的构造检查和绕回处理。"""

    def test_wraps_only_when_min_hue_exceeds_max(self):
        """h_min > h_max 才算绕回：红色绕回，绿色不绕回。"""
        assert RED.wraps
        assert not COLOR_PRESETS['green'].wraps

    @pytest.mark.parametrize('kwargs', [
        {'h_min': -1, 'h_max': 10},
        {'h_min': 0, 'h_max': 180},
        {'h_min': 0, 'h_max': 10, 's_min': 256},
        {'h_min': 0, 'h_max': 10, 'v_max': -1},
    ])
    def test_rejects_out_of_range_values(self, kwargs):
        """色相超出 0–179，或 S、V 超出 0–255：抛 ValueError。"""
        with pytest.raises(ValueError):
            HsvRange(**kwargs)

    def test_rejects_inverted_saturation_or_value(self):
        """S 或 V 的下限大于上限：抛 ValueError（H 可以反过来，S、V 不行）。"""
        with pytest.raises(ValueError):
            HsvRange(0, 10, s_min=200, s_max=100)
        with pytest.raises(ValueError):
            HsvRange(0, 10, v_min=200, v_max=100)

    def test_wrapping_mask_covers_both_ends_of_hue(self):
        """红色范围同时命中色相 0 和 175（两头），但不命中色相 90（青色）。"""
        hsv = np.array([[[0, 200, 200], [175, 200, 200], [90, 200, 200]]], dtype=np.uint8)
        assert RED.mask(hsv).tolist() == [[255, 255, 0]]

    def test_mask_excludes_gray(self):
        """灰色（饱和度 0）不属于任何颜色，即使色相值刚好在范围内。"""
        hsv = np.array([[[0, 0, 200], [0, 200, 200]]], dtype=np.uint8)
        assert RED.mask(hsv).tolist() == [[0, 255]]


# ============================================================ 分割与定位

class TestDetect:
    """从合成图片中找出目标。"""

    def test_blank_image_has_no_target(self):
        """纯灰色背景：没有目标，返回 None。"""
        assert detect(blank_image(), RED) is None

    def test_finds_position_and_size(self):
        """在 (100, 80) 画半径 20 的红圆：质心误差在 1 像素内，外接矩形宽高约 41。"""
        image = with_circle(blank_image(), (100, 80), 20, hue=0)
        found = detect(image, RED)
        assert found is not None
        assert found.cx == pytest.approx(100, abs=1)
        assert found.cy == pytest.approx(80, abs=1)
        assert found.w == pytest.approx(41, abs=2)
        assert found.h == pytest.approx(41, abs=2)
        assert (found.image_width, found.image_height) == (WIDTH, HEIGHT)

    @pytest.mark.parametrize('hue', [0, 5, 175, 179])
    def test_finds_red_on_both_sides_of_zero_hue(self, hue):
        """色相在 0 的两侧（0、5 和 175、179）的红色都能找到。"""
        image = with_circle(blank_image(), (160, 90), 20, hue=hue)
        assert detect(image, RED) is not None

    def test_dark_part_of_object_is_still_found(self):
        """明度较低（V=95，球背光的一面）的红色仍然在范围内，不会漏检。"""
        image = with_circle(blank_image(), (160, 90), 20, hue=0, s=150, v=95)
        assert detect(image, RED) is not None

    def test_orange_is_not_mistaken_for_red(self):
        """橙色物体（色相约 15）：红色范围不命中，橙色范围命中。"""
        image = with_circle(blank_image(), (160, 90), 20, hue=15)
        assert detect(image, RED) is None
        assert detect(image, ORANGE) is not None

    def test_picks_largest_blob(self):
        """画面里有大小两个红圆：返回大的那个。"""
        image = with_circle(blank_image(), (60, 90), 10, hue=0)
        with_circle(image, (240, 90), 25, hue=0)
        found = detect(image, RED)
        assert found.cx == pytest.approx(240, abs=1)

    def test_small_speck_is_ignored(self):
        """单个像素大小的红点是噪声：形态学开运算去掉它，检测结果为 None。"""
        image = blank_image()
        image[50, 50] = hsv_to_bgr(0)
        assert detect(image, RED) is None

    def test_min_area_ratio_filters_small_targets(self):
        """目标比 min_area_ratio 规定的还小：忽略；调低门槛后能找到。"""
        image = with_circle(blank_image(), (160, 90), 8, hue=0)
        assert detect(image, RED, min_area_ratio=0.05) is None
        assert detect(image, RED, min_area_ratio=0.001) is not None

    def test_hole_inside_object_is_filled(self):
        """红圆中间有个小洞（反光）：闭运算补上，目标仍是一整块，质心不偏。"""
        image = with_circle(blank_image(), (160, 90), 25, hue=0)
        cv2.circle(image, (160, 90), 2, (255, 255, 255), thickness=-1)
        found = detect(image, RED)
        assert found.cx == pytest.approx(160, abs=1)
        assert found.cy == pytest.approx(90, abs=1)


class TestSegmentAndFindTarget:
    """分步使用 segment 和 find_target。"""

    def test_segment_returns_binary_mask_of_same_size(self):
        """掩码与图像同尺寸，只含 0 和 255，红圆内部为 255。"""
        image = with_circle(blank_image(), (100, 80), 20, hue=0)
        mask = segment(image, RED)
        assert mask.shape == (HEIGHT, WIDTH)
        assert set(np.unique(mask)) == {0, 255}
        assert mask[80, 100] == 255 and mask[10, 10] == 0

    def test_find_target_on_empty_mask(self):
        """全黑掩码：没有轮廓，返回 None。"""
        assert find_target(np.zeros((HEIGHT, WIDTH), dtype=np.uint8)) is None

    def test_disabling_blur_and_morphology(self):
        """blur_ksize=0、morph_ksize=0：跳过平滑和去噪，单个像素的红点也会被找到。"""
        image = blank_image()
        image[50:53, 50:53] = hsv_to_bgr(0)
        assert detect(image, RED, min_area_ratio=0.0, blur_ksize=0, morph_ksize=0) is not None
        assert detect(image, RED, min_area_ratio=0.0) is None


# ============================================================ Detection 的派生量

class TestDetectionProperties:
    """水平偏移和贴边判断。"""

    @staticmethod
    def detection(x, y, w, h):
        """构造一个在 320×180 画面里、外接矩形为 (x, y, w, h) 的检测结果。"""
        return Detection(cx=x + w / 2, cy=y + h / 2, area=w * h, x=x, y=y, w=w, h=h,
                         image_width=WIDTH, image_height=HEIGHT)

    @pytest.mark.parametrize('x, expected', [
        (140, 0.0),        # 矩形 140–180，中心 160：正对画面中心
        (0, -0.875),       # 中心 20：偏左，(20-160)/160
        (280, 0.875),      # 中心 300：偏右
    ])
    def test_offset_x(self, x, expected):
        """水平偏移：左边为负、右边为正、中心为 0。"""
        assert self.detection(x, 80, 40, 40).offset_x == pytest.approx(expected)

    def test_offset_from_real_detection(self):
        """从真实检测得到的偏移：圆画在画面右侧，偏移为正。"""
        found = detect(with_circle(blank_image(), (240, 90), 20, hue=0), RED)
        assert found.offset_x == pytest.approx(0.5, abs=0.02)

    @pytest.mark.parametrize('x, y, w, h, expected', [
        (100, 50, 40, 40, False),     # 完全在画面里面
        (0, 50, 40, 40, True),        # 贴左边
        (280, 50, 40, 40, True),      # 右边缘刚好到 320
        (100, 0, 40, 40, True),       # 贴上边
        (100, 140, 40, 40, True),     # 下边缘刚好到 180
    ])
    def test_touches_border(self, x, y, w, h, expected):
        """外接矩形碰到画面四条边中的任何一条：视为贴边。"""
        assert self.detection(x, y, w, h).touches_border is expected

    def test_clipped_circle_touches_border(self):
        """圆有一半画到了画面外：检测结果是贴边的。"""
        found = detect(with_circle(blank_image(), (0, 90), 25, hue=0), RED)
        assert found.touches_border


# ============================================================ 图像缩放与调试绘制

class TestResizeToWidth:
    """降采样。"""

    def test_keeps_aspect_ratio(self):
        """1920×1080 缩到宽 320：高度按比例变成 180。"""
        big = np.zeros((1080, 1920, 3), dtype=np.uint8)
        assert resize_to_width(big, 320).shape == (180, 320, 3)

    def test_small_image_is_returned_unchanged(self):
        """图像已经不比目标宽：原样返回（不放大）。"""
        image = blank_image()
        assert resize_to_width(image, 640) is image


class TestConvertToBgr:
    """ROS 图像编码到 BGR 的转换。"""

    def test_rgb8_swaps_channels(self):
        """rgb8（红绿蓝顺序）：红色像素 (255, 0, 0) 转成 BGR 后是 (0, 0, 255)。"""
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        rgb[..., 0] = 255
        assert convert_to_bgr(rgb, 'rgb8')[0, 0].tolist() == [0, 0, 255]

    def test_bgr8_is_returned_unchanged(self):
        """bgr8：已经是 OpenCV 的顺序，原样返回。"""
        image = blank_image()
        assert convert_to_bgr(image, 'bgr8') is image

    def test_rgb8_red_is_detected_as_red(self):
        """端到端：rgb8 编码的红色圆，转换后能被红色范围找到；不转换则找不到。"""
        bgr = with_circle(blank_image(), (160, 90), 20, hue=0)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        assert detect(convert_to_bgr(rgb, 'rgb8'), RED) is not None
        assert detect(rgb, RED) is None

    def test_rejects_unsupported_encoding(self):
        """不支持的编码（如 mono8）：抛 ValueError，而不是悄悄返回错误的颜色。"""
        with pytest.raises(ValueError):
            convert_to_bgr(blank_image(), 'mono8')


class TestDrawDetection:
    """调试图像绘制。"""

    def test_returns_annotated_copy(self):
        """返回的是带标记的副本：与原图同尺寸、内容不同，原图不被修改。"""
        image = with_circle(blank_image(), (100, 80), 20, hue=0)
        original = image.copy()
        canvas = draw_detection(image, detect(image, RED), 'red')
        assert canvas.shape == image.shape
        assert not np.array_equal(canvas, image)
        assert np.array_equal(image, original)

    def test_works_without_detection(self):
        """没有目标时也能画（只画中心线和文字），不报错。"""
        canvas = draw_detection(blank_image(), None, 'no target')
        assert canvas.shape == (HEIGHT, WIDTH, 3)
