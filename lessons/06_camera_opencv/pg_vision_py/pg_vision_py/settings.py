"""
06 课 · 视觉部分的设置：跟什么颜色、目标多大、图像怎么处理。

与 follower.FollowerParams（控制参数）分开：这里的参数决定"看到什么"，
控制参数决定"怎么走"。字段名就是 ROS 参数名，和参数文件 config/follower.yaml 一一对应。
不依赖 ROS，构造时检查合法性，节点修改参数前可以先拿新值试构造一次，不合法就拒绝。
"""

from dataclasses import asdict, dataclass, fields

from pg_vision_py.color_detector import COLOR_PRESETS

MIN_IMAGE_WIDTH = 32   # 再小的图像里，物体只剩几个像素，检测没有意义


@dataclass(frozen=True)
class VisionSettings:
    """视觉设置。"""

    color: str = 'red'                  # 要跟随的颜色，COLOR_PRESETS 里的名字
    target_width: float = 0.3           # 目标的真实宽度，m（红球直径）
    image_width: int = 320              # 检测前把图像缩小到这个宽度，像素
    min_area_ratio: float = 0.0005      # 目标面积占整幅图像的最小比例，更小的当噪声
    image_timeout: float = 1.0          # 相机图像超过此时间没更新就停车，s
    publish_debug_image: bool = True    # 是否发布带标注的调试图像

    def __post_init__(self):
        """检查各项设置是否合法。"""
        if self.color not in COLOR_PRESETS:
            raise ValueError(f'未知颜色 "{self.color}"，可选：{", ".join(COLOR_PRESETS)}')
        if self.target_width <= 0:
            raise ValueError(f'target_width 必须大于 0，收到 {self.target_width}')
        if self.image_width < MIN_IMAGE_WIDTH:
            raise ValueError(f'image_width 不能小于 {MIN_IMAGE_WIDTH}，收到 {self.image_width}')
        if not 0.0 < self.min_area_ratio < 1.0:
            raise ValueError(f'min_area_ratio 必须在 0 到 1 之间，收到 {self.min_area_ratio}')
        if self.image_timeout <= 0:
            raise ValueError(f'image_timeout 必须大于 0，收到 {self.image_timeout}')

    @property
    def hsv_range(self):
        """当前颜色对应的 HSV 范围。"""
        return COLOR_PRESETS[self.color]


def to_ros_params(settings):
    """把设置转成 {ROS 参数名: 值}，用来声明 ROS 参数。"""
    return asdict(settings)


def from_ros_params(values):
    """
    把 {ROS 参数名: 值} 转成 VisionSettings。

    缺少的字段或不认识的字段都会报错（防止参数名写错被悄悄忽略）；
    会触发 VisionSettings 的合法性检查。
    """
    names = {f.name for f in fields(VisionSettings)}
    if set(values) != names:
        raise ValueError(f'参数名不匹配：多余 {sorted(set(values) - names)}，'
                         f'缺少 {sorted(names - set(values))}')
    return VisionSettings(
        color=str(values['color']),
        target_width=float(values['target_width']),
        image_width=int(values['image_width']),
        min_area_ratio=float(values['min_area_ratio']),
        image_timeout=float(values['image_timeout']),
        publish_debug_image=bool(values['publish_debug_image']))
