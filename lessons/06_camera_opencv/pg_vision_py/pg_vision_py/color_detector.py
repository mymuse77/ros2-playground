"""
06 课 · 颜色检测：用 HSV 阈值把图像里指定颜色的物体找出来。

这个模块只依赖 OpenCV 和 numpy，不依赖 ROS，所以可以直接用合成图片做单元测试。

处理流程：

    BGR 图像 ──▶ 高斯模糊 ──▶ 转 HSV ──▶ 阈值分割 ──▶ 形态学去噪 ──▶ 找最大轮廓 ──▶ Detection
                 (压低噪点)   (分离颜色)   (得到掩码)    (去斑点、补空洞)   (取面积最大的块)

知识点：
  - 为什么用 HSV 而不是 RGB：同一个红球，亮处和暗处的 RGB 值差别很大，但色相 H 几乎不变。
    H（色相）决定"是什么颜色"，S（饱和度）区分"彩色"和"灰色"，V（明度）是亮度。
    阈值只要卡住 H，再用 S 排除灰色的地面和墙，对光照变化就不敏感
  - OpenCV 里 H 的范围是 0–179（不是 0–360，因为要塞进 8 位），S、V 是 0–255
  - 红色的色相在 0 附近，会"绕回"：偏橙一侧是 0–10，偏紫一侧是 170–179。
    所以红色的范围是 170→10 这样"回头"的区间，HsvRange 用 h_min > h_max 表示
  - 形态学开运算（先腐蚀后膨胀）去掉零星的小斑点，闭运算（先膨胀后腐蚀）补上物体内部的小空洞
  - 图像坐标：原点在左上角，x 向右，y 向下
"""

from dataclasses import dataclass

import cv2
import numpy as np

H_MAX = 179   # OpenCV 中色相的最大值
SV_MAX = 255  # 饱和度、明度的最大值


@dataclass(frozen=True)
class HsvRange:
    """
    HSV 颜色范围（闭区间）。

    h_min > h_max 表示色相绕回：例如红色 h_min=170、h_max=10，
    覆盖 170–179 和 0–10 两段。
    """

    h_min: int
    h_max: int
    s_min: int = 70
    s_max: int = SV_MAX
    v_min: int = 50
    v_max: int = SV_MAX

    def __post_init__(self):
        """检查各分量都在合法范围内，S、V 的下限不能大于上限。"""
        for name in ('h_min', 'h_max'):
            if not 0 <= getattr(self, name) <= H_MAX:
                raise ValueError(f'{name} 必须在 0 到 {H_MAX} 之间，收到 {getattr(self, name)}')
        for name in ('s_min', 's_max', 'v_min', 'v_max'):
            if not 0 <= getattr(self, name) <= SV_MAX:
                raise ValueError(f'{name} 必须在 0 到 {SV_MAX} 之间，收到 {getattr(self, name)}')
        if self.s_min > self.s_max:
            raise ValueError(f's_min（{self.s_min}）不能大于 s_max（{self.s_max}）')
        if self.v_min > self.v_max:
            raise ValueError(f'v_min（{self.v_min}）不能大于 v_max（{self.v_max}）')

    @property
    def wraps(self):
        """色相范围是否绕回（跨过 0）。"""
        return self.h_min > self.h_max

    def mask(self, hsv):
        """
        在 HSV 图像里找出落在范围内的像素。

        :param hsv: HSV 图像，uint8，形状 (高, 宽, 3)
        :return: 掩码，uint8，形状 (高, 宽)，范围内的像素为 255，其余为 0
        """
        def in_range(h_low, h_high):
            return cv2.inRange(
                hsv,
                np.array([h_low, self.s_min, self.v_min], dtype=np.uint8),
                np.array([h_high, self.s_max, self.v_max], dtype=np.uint8))

        if not self.wraps:
            return in_range(self.h_min, self.h_max)
        # 绕回：分成 [0, h_max] 和 [h_min, 179] 两段分别分割，再合并
        return cv2.bitwise_or(in_range(0, self.h_max), in_range(self.h_min, H_MAX))


# 常用颜色的预设，数值按 06 课场景中的物体实测（Gazebo 渲染，无阴影）
#   红球：H=0，S 101–201，V 95–192（球的下半部分较暗）
COLOR_PRESETS = {
    'red': HsvRange(170, 10),
    'orange': HsvRange(11, 25),
    'green': HsvRange(40, 85),
    'blue': HsvRange(100, 130),
}


@dataclass(frozen=True)
class Detection:
    """
    一次检测的结果：目标在图像中的位置和大小。所有坐标和长度的单位都是像素。

    cx、cy 是轮廓的质心；x、y、w、h 是外接矩形（左上角和宽高）；
    image_width、image_height 是检测时的图像尺寸，用来换算相对位置。
    """

    cx: float
    cy: float
    area: float
    x: int
    y: int
    w: int
    h: int
    image_width: int
    image_height: int

    @property
    def offset_x(self):
        """
        目标相对画面中心的水平偏移，归一化到 [-1, 1]。

        0 表示正对中心，-1 是画面最左边，+1 是画面最右边。
        注意方向：偏移为正表示目标在右边，而 ROS 里的角速度为正是"向左转"，
        所以控制器里要取反。
        """
        half = self.image_width / 2.0
        return (self.cx - half) / half

    @property
    def touches_border(self):
        """
        目标是否贴着画面边缘。

        贴边说明物体可能只露出一部分（被画面截断），这时外接矩形的宽度比真实宽度小，
        用它估算的距离会偏远，不能直接信。
        """
        return (self.x <= 0 or self.y <= 0
                or self.x + self.w >= self.image_width
                or self.y + self.h >= self.image_height)


def resize_to_width(image, width):
    """
    把图像等比缩小到指定宽度；本来就不比它宽的图像原样返回。

    仿真相机输出 1920×1080，每帧 6 MB，直接处理太慢。检测只需要知道目标的大致位置，
    缩到 320 像素宽足够，计算量是原来的 1/36。缩小用 INTER_AREA（区域平均），
    比默认的双线性插值更适合降采样，不会产生锯齿。
    """
    height, current = image.shape[:2]
    if current <= width:
        return image
    new_height = max(1, round(height * width / current))
    return cv2.resize(image, (width, new_height), interpolation=cv2.INTER_AREA)


def convert_to_bgr(image, encoding):
    """
    把 ROS 图像（sensor_msgs/Image 转成的数组）统一成 OpenCV 习惯的 BGR 通道顺序。

    ROS 的 rgb8 是红绿蓝顺序，OpenCV 默认是蓝绿红，不转换的话红色会被当成蓝色，
    所有颜色阈值都会错。

    :param image: uint8 数组，形状 (高, 宽, 3)
    :param encoding: ROS 图像编码，支持 'rgb8' 和 'bgr8'
    :raises ValueError: 不支持的编码
    """
    if encoding == 'bgr8':
        return image
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    raise ValueError(f'不支持的图像编码 "{encoding}"，只支持 rgb8 和 bgr8')


def segment(image_bgr, color, blur_ksize=5, morph_ksize=5):
    """
    阈值分割：返回图像中指定颜色区域的掩码。

    :param image_bgr: BGR 图像，uint8，形状 (高, 宽, 3)
    :param color: HsvRange，要找的颜色范围
    :param blur_ksize: 高斯模糊的核大小（奇数）；0 表示不模糊
    :param morph_ksize: 形态学去噪的核大小；0 表示不处理
    :return: 掩码，uint8，形状 (高, 宽)，目标颜色的像素为 255
    """
    if blur_ksize > 0:
        image_bgr = cv2.GaussianBlur(image_bgr, (blur_ksize, blur_ksize), 0)
    mask = color.mask(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV))
    if morph_ksize > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_ksize, morph_ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # 去掉小斑点
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # 补上小空洞
    return mask


def find_target(mask, min_area_ratio=0.0005):
    """
    在掩码里找面积最大的一块，作为目标。

    :param mask: segment() 返回的掩码
    :param min_area_ratio: 目标面积占整幅图像的最小比例，更小的块当作噪声忽略
    :return: Detection；没有够大的块时返回 None
    """
    height, width = mask.shape[:2]
    # RETR_EXTERNAL：只要最外层轮廓，物体内部的洞不算；
    # CHAIN_APPROX_SIMPLE：只保留轮廓的拐点，省内存
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area <= 0 or area < min_area_ratio * width * height:
        return None
    # 图像矩：m00 是面积，m10/m00、m01/m00 是质心
    moments = cv2.moments(largest)
    x, y, w, h = cv2.boundingRect(largest)
    return Detection(
        cx=moments['m10'] / moments['m00'], cy=moments['m01'] / moments['m00'],
        area=area, x=x, y=y, w=w, h=h, image_width=width, image_height=height)


def detect(image_bgr, color, min_area_ratio=0.0005, blur_ksize=5, morph_ksize=5):
    """分割 + 找目标，一步完成。参数含义见 segment() 和 find_target()。"""
    mask = segment(image_bgr, color, blur_ksize, morph_ksize)
    return find_target(mask, min_area_ratio)


def draw_detection(image_bgr, detection, text=''):
    """
    在图像副本上画出检测结果，用于调试：外接矩形、质心十字、画面中心竖线和文字。

    没有目标时只画中心线和文字。原图不会被修改。
    OpenCV 自带的字体不支持中文，所以文字只能用英文和数字。
    """
    canvas = image_bgr.copy()
    height, width = canvas.shape[:2]
    cv2.line(canvas, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
    if detection is not None:
        # 贴边（可能被截断）用黄色框，正常用绿色框
        color = (0, 255, 255) if detection.touches_border else (0, 255, 0)
        cv2.rectangle(canvas, (detection.x, detection.y),
                      (detection.x + detection.w, detection.y + detection.h), color, 2)
        center = (int(round(detection.cx)), int(round(detection.cy)))
        cv2.drawMarker(canvas, center, color, cv2.MARKER_CROSS, 12, 2)
    if text:
        cv2.putText(canvas, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(canvas, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return canvas
