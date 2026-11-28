"""
06 课 · 针孔相机模型：由目标在图像中的位置和宽度，算出它的方位角和距离。

这个模块不依赖 ROS 和 OpenCV。

针孔模型（相机坐标系：z 向前，x 向右）：

         真实物体（宽 W，距离 Z）
              ┃◀─ W ─▶┃
              ┃       ┃              成像平面（像素）
              ┃       ┃            ┃◀─ w ─▶┃
        ──────┸───────┸─────────── ┸───────┸──
                      ╲           ╱
                       ╲  光心   ╱          w / f = W / Z
                        ╲ ● ───╱             ⇒  Z = W · f / w
                              f（焦距，单位：像素）

  - 方位角：目标中心离画面中心 dx 个像素，方位角 = atan(dx / f)。
    比"偏移像素数 × 常数"更准：偏得越远，每像素对应的角度越小
  - 距离：已知物体真实宽度 W，量出它在图像里的像素宽度 w，距离 Z = W·f / w
  - 焦距 f 可以由水平视场角算出：f = (图像宽度 / 2) / tan(视场角 / 2)；
    真机上一般从相机标定得到（camera_info 话题里的 K 矩阵）
  - 图像缩小后，焦距同比例缩小，所以要用 scaled_to() 换算

精度说明：
  - 距离按"物体在光轴方向上的距离"算，目标偏离画面中心越远，与直线距离的差越大
  - 球的轮廓宽度略大于 2r·f/Z，距离 0.8 m、半径 0.15 m 时偏差约 2%，这里忽略
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class PinholeCamera:
    """针孔相机：焦距（像素）和对应的图像宽度（像素）。"""

    fx: float
    image_width: int

    def __post_init__(self):
        """焦距和图像宽度必须为正。"""
        if self.fx <= 0:
            raise ValueError(f'焦距必须大于 0，收到 {self.fx}')
        if self.image_width <= 0:
            raise ValueError(f'图像宽度必须大于 0，收到 {self.image_width}')

    @classmethod
    def from_fov(cls, horizontal_fov, image_width):
        """
        由水平视场角（弧度）和图像宽度（像素）构造。

        视场角必须在 0 到 π 之间（不含两端）。
        """
        if not 0.0 < horizontal_fov < math.pi:
            raise ValueError(f'水平视场角必须在 0 到 π 之间，收到 {horizontal_fov}')
        return cls(fx=(image_width / 2.0) / math.tan(horizontal_fov / 2.0),
                   image_width=image_width)

    def scaled_to(self, image_width):
        """图像缩放到另一个宽度后的相机：焦距按相同比例缩放。"""
        return PinholeCamera(self.fx * image_width / self.image_width, image_width)

    def bearing(self, cx):
        """
        像素列 cx 对应的水平方位角，弧度。

        0 是正前方，目标在画面右侧为正、左侧为负（与图像 x 方向一致）。
        ROS 里 z 轴角速度为正是向左转，所以控制器需要取反。
        """
        return math.atan2(cx - self.image_width / 2.0, self.fx)

    def distance(self, pixel_width, real_width):
        """
        由物体的像素宽度和真实宽度（米）估算距离，米。

        :raises ValueError: pixel_width 或 real_width 不是正数
        """
        if pixel_width <= 0 or real_width <= 0:
            raise ValueError('像素宽度和真实宽度都必须大于 0')
        return real_width * self.fx / pixel_width


@dataclass(frozen=True)
class Observation:
    """
    控制器的输入：目标相对机器人的位置。

    bearing：方位角，弧度，目标在右为正；
    distance：距离，米；None 表示这一帧无法可靠估距（目标被画面边缘截断）。
    """

    bearing: float
    distance: Optional[float]


def observe(detection, camera, target_width):
    """
    把图像检测结果换算成控制器要用的观测量。

    :param detection: color_detector.Detection，没有目标时为 None
    :param camera: 原始分辨率下的 PinholeCamera；检测用的是缩小后的图像，
                   这里按 detection.image_width 自动换算焦距
    :param target_width: 目标的真实宽度，米
    :return: Observation；detection 为 None 时返回 None
    """
    if detection is None:
        return None
    scaled = camera.scaled_to(detection.image_width)
    # 目标贴着画面边缘时可能只露出一部分，像素宽度偏小，估出的距离会偏远，
    # 控制器据此前进会撞上去，所以宁可不估
    distance = None if detection.touches_border else scaled.distance(detection.w, target_width)
    return Observation(bearing=scaled.bearing(detection.cx), distance=distance)
