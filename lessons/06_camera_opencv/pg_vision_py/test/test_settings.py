"""视觉设置的单元测试：合法性检查与 ROS 参数转换（不需要启动 ROS）。"""

from pg_vision_py.color_detector import COLOR_PRESETS
from pg_vision_py.settings import from_ros_params, to_ros_params, VisionSettings
import pytest


class TestVisionSettings:
    """构造检查。"""

    def test_defaults_are_valid(self):
        """默认设置本身必须合法，默认颜色是红色。"""
        assert VisionSettings().color == 'red'

    @pytest.mark.parametrize('color', sorted(COLOR_PRESETS))
    def test_every_preset_color_is_accepted(self, color):
        """所有预设颜色都能用，并能取到对应的 HSV 范围。"""
        assert VisionSettings(color=color).hsv_range is COLOR_PRESETS[color]

    @pytest.mark.parametrize('overrides', [
        {'color': 'purple'},
        {'color': ''},
        {'target_width': 0.0},
        {'target_width': -0.3},
        {'image_width': 31},
        {'min_area_ratio': 0.0},
        {'min_area_ratio': 1.0},
        {'image_timeout': 0.0},
    ])
    def test_rejects_invalid_values(self, overrides):
        """未知颜色、非正的宽度和超时、过小的图像、不在 (0,1) 内的面积比例：抛 ValueError。"""
        with pytest.raises(ValueError):
            VisionSettings(**overrides)

    def test_unknown_color_message_lists_choices(self):
        """错误信息里列出可选颜色，用户一看就知道怎么改。"""
        with pytest.raises(ValueError, match='red'):
            VisionSettings(color='purple')


class TestRosParams:
    """与 ROS 参数字典互转。"""

    def test_round_trip(self):
        """to_ros_params 再 from_ros_params：得到相等的设置。"""
        settings = VisionSettings(color='blue', image_width=640, publish_debug_image=False)
        assert from_ros_params(to_ros_params(settings)) == settings

    def test_converts_types(self):
        """ROS 传来的整数（如 target_width: 1）会转成 float，浮点的图像宽度转成 int。"""
        values = to_ros_params(VisionSettings())
        values['target_width'] = 1
        values['image_width'] = 640.0
        result = from_ros_params(values)
        assert isinstance(result.target_width, float)
        assert isinstance(result.image_width, int)

    def test_rejects_unknown_or_missing_names(self):
        """参数名多了或少了：抛 ValueError，错别字不会被悄悄忽略。"""
        values = to_ros_params(VisionSettings())
        with pytest.raises(ValueError):
            from_ros_params({**values, 'colour': 'red'})
        del values['color']
        with pytest.raises(ValueError):
            from_ros_params(values)

    def test_invalid_value_is_rejected_on_conversion(self):
        """值不合法时，转换阶段就拒绝：节点修改参数前靠它做整体校验。"""
        values = to_ros_params(VisionSettings())
        values['color'] = 'purple'
        with pytest.raises(ValueError):
            from_ros_params(values)
