"""跟随控制器的单元测试：用手工构造的观测验证控制律和状态机（不需要启动 ROS）。"""

import math

from pg_vision_py.camera_model import Observation
from pg_vision_py.follower import (Command, FollowController, FollowerParams, from_ros_params,
                                   is_stale, State, STOP, to_ros_params)
import pytest

DT = 0.1                      # 控制周期，s
TARGET = FollowerParams().target_distance


def make_controller(**overrides):
    """按默认参数（可覆盖个别字段）创建控制器。"""
    return FollowController(FollowerParams(**overrides))


def run(controller, observation, steps=1, start=0.0):
    """
    以 DT 为周期喂入同一个观测 steps 次，返回最后一次的指令。

    start 是第一次调用的时间；同一个控制器多次调用 run 时要接着上次的时间。
    """
    command = None
    for i in range(steps):
        command = controller.update(observation, start + i * DT)
    return command


def settled(observation, **overrides):
    """让速度充分加速后（20 个周期）取稳定的指令，排除加速度限制的影响。"""
    return run(make_controller(**overrides), observation, steps=20)


# ============================================================ 参数

class TestParams:
    """参数检查与转换。"""

    def test_defaults_are_valid(self):
        """默认参数本身必须合法。"""
        FollowerParams()

    @pytest.mark.parametrize('overrides', [
        {'target_distance': 0.0},
        {'max_linear_speed': -0.1},
        {'angular_gain': 0.0},
        {'max_linear_accel': 0.0},
        {'lost_timeout': -1.0},
        {'search_speed': -0.5},
        {'distance_tolerance': 0.8},       # 不小于目标距离
        {'angle_tolerance_deg': 30.0},     # 不小于 turn_only_angle_deg
        {'turn_only_angle_deg': 90.0},     # 目标在侧面时也前进，不合理
    ])
    def test_rejects_invalid_values(self, overrides):
        """非正的增益和速度、负的容差和超时、互相矛盾的阈值：抛 ValueError。"""
        with pytest.raises(ValueError):
            FollowerParams(**overrides)

    def test_angle_properties_are_radians(self):
        """度数属性换算成弧度：2° 和 30°。"""
        p = FollowerParams()
        assert p.angle_tolerance == pytest.approx(math.radians(2.0))
        assert p.turn_only_angle == pytest.approx(math.radians(30.0))

    def test_ros_params_round_trip(self):
        """to_ros_params 再 from_ros_params：得到相等的参数。"""
        p = FollowerParams(target_distance=1.0, search_speed=0.0)
        assert from_ros_params(to_ros_params(p)) == p

    def test_from_ros_params_converts_ints_to_float(self):
        """YAML 里写成整数的值（如 1）也能接受：统一转成 float。"""
        values = to_ros_params(FollowerParams())
        values['target_distance'] = 1
        assert isinstance(from_ros_params(values).target_distance, float)

    def test_from_ros_params_rejects_unknown_or_missing(self):
        """参数名多了或少了：抛 ValueError，错别字不会被悄悄忽略。"""
        values = to_ros_params(FollowerParams())
        with pytest.raises(ValueError):
            from_ros_params({**values, 'typo': 1.0})
        del values['linear_gain']
        with pytest.raises(ValueError):
            from_ros_params(values)

    def test_is_stale(self):
        """从没收到过数据或间隔超过 timeout：过期；刚好等于 timeout 不算过期。"""
        assert is_stale(None, 10.0, 0.5)
        assert is_stale(9.0, 10.0, 0.5)
        assert not is_stale(9.5, 10.0, 0.5)
        assert not is_stale(9.8, 10.0, 0.5)


# ============================================================ 转向

class TestSteering:
    """转向：比例控制、死区、限幅、方向。"""

    def test_target_on_the_right_turns_right(self):
        """目标在右边（方位角为正）：向右转（角速度为负）。"""
        assert settled(Observation(0.2, TARGET)).angular < 0

    def test_target_on_the_left_turns_left(self):
        """目标在左边（方位角为负）：向左转（角速度为正）。"""
        assert settled(Observation(-0.2, TARGET)).angular > 0

    def test_proportional_to_bearing(self):
        """角速度 = -angular_gain × 方位角：增益 1.5、方位 0.2 rad → -0.3 rad/s。"""
        assert settled(Observation(0.2, TARGET)).angular == pytest.approx(-0.3)

    def test_deadband_stops_turning(self):
        """方位偏差在容差（2° ≈ 0.035 rad）内：不转，避免在目标附近抖动。"""
        assert settled(Observation(0.03, TARGET)).angular == 0.0
        assert settled(Observation(0.04, TARGET)).angular != 0.0

    def test_angular_speed_is_limited(self):
        """目标偏得很远（1.0 rad，比例输出 1.5）：角速度限制在上限 1.0。"""
        assert settled(Observation(1.0, TARGET)).angular == pytest.approx(-1.0)
        assert settled(Observation(-1.0, TARGET)).angular == pytest.approx(1.0)


# ============================================================ 前进

class TestDriving:
    """前进：距离控制、不倒车、对准后再走、加速度限制。"""

    def test_far_target_drives_forward(self):
        """目标比期望距离远 0.2 m，正对：前进速度 = 0.5 × 0.2 = 0.1 m/s，不转向。"""
        command = settled(Observation(0.0, TARGET + 0.2))
        assert command.linear == pytest.approx(0.1)
        assert command.angular == 0.0

    def test_speed_is_limited(self):
        """目标很远（比例输出 > 上限）：前进速度限制在 0.15 m/s。"""
        assert settled(Observation(0.0, 3.0)).linear == pytest.approx(0.15)

    def test_too_close_never_reverses(self):
        """目标太近：停下来，但不倒车（后方没有传感器）。"""
        command = settled(Observation(0.0, TARGET - 0.3))
        assert command.linear == 0.0

    def test_distance_deadband(self):
        """距离误差在容差（0.05 m）内：不前进。"""
        assert settled(Observation(0.0, TARGET + 0.04)).linear == 0.0
        assert settled(Observation(0.0, TARGET + 0.06)).linear > 0.0

    def test_misaligned_target_slows_the_drive(self):
        """方位偏差为 15°（turn_only_angle 30° 的一半）：前进速度减半。"""
        aligned = settled(Observation(0.0, 3.0)).linear
        half = settled(Observation(math.radians(15), 3.0)).linear
        assert half == pytest.approx(aligned / 2)

    def test_turn_only_when_far_off_axis(self):
        """方位偏差达到或超过 30°：只转向不前进。"""
        command = settled(Observation(math.radians(40), 3.0))
        assert command.linear == 0.0
        assert command.angular < 0

    def test_unknown_distance_turns_but_does_not_advance(self):
        """距离未知（目标被画面截断）：照常转向对准，但不前进。"""
        command = settled(Observation(0.3, None))
        assert command.linear == 0.0
        assert command.angular < 0

    def test_acceleration_is_limited_from_rest(self):
        """从静止起步：第一个周期不动，之后每周期最多增加 0.3 × 0.1 = 0.03 m/s。"""
        controller = make_controller()
        first = controller.update(Observation(0.0, 3.0), 0.0)
        second = controller.update(Observation(0.0, 3.0), DT)
        third = controller.update(Observation(0.0, 3.0), 2 * DT)
        assert first.linear == 0.0
        assert second.linear == pytest.approx(0.03)
        assert third.linear == pytest.approx(0.06)

    def test_braking_is_immediate(self):
        """全速前进时目标突然到位：线速度立即降为 0，不受加速度限制。"""
        controller = make_controller()
        run(controller, Observation(0.0, 3.0), steps=20)
        assert controller.update(Observation(0.0, TARGET), 2.0).linear == 0.0

    def test_long_gap_does_not_unlock_large_acceleration(self):
        """两次更新间隔很久（程序卡了 10 s）：按 0.5 s 算加速度，速度只涨 0.3 × 0.5 = 0.15。"""
        controller = make_controller(max_linear_speed=0.5)    # 上限放高，才看得出加速度限制
        controller.update(Observation(0.0, 3.0), 0.0)
        command = controller.update(Observation(0.0, 3.0), 10.0)
        assert command.linear == pytest.approx(0.15)          # 不是 0.5，更不是 0.3 × 10


# ============================================================ 状态机

class TestStates:
    """跟随、到位、等待、搜索。"""

    def test_initial_state_is_holding(self):
        """刚创建、还没看到目标：等待。"""
        assert make_controller().state is State.HOLDING

    def test_tracking_while_approaching(self):
        """目标还远：跟随状态。"""
        controller = make_controller()
        run(controller, Observation(0.0, 2.0), steps=3)
        assert controller.state is State.TRACKING

    def test_arrived_when_in_position(self):
        """距离和方位都在容差内：到位，指令全为 0。"""
        controller = make_controller()
        command = run(controller, Observation(0.01, TARGET + 0.02), steps=3)
        assert controller.state is State.ARRIVED
        assert command == STOP

    def test_not_arrived_when_only_distance_is_right(self):
        """距离对了但方向没对准：还要继续转，不算到位。"""
        controller = make_controller()
        run(controller, Observation(0.3, TARGET), steps=3)
        assert controller.state is State.TRACKING

    def test_not_arrived_when_distance_unknown(self):
        """距离未知：无法确认到位，仍是跟随状态。"""
        controller = make_controller()
        run(controller, Observation(0.0, None), steps=3)
        assert controller.state is State.TRACKING

    def test_holds_still_right_after_losing_target(self):
        """刚丢目标：立即停车并等待，不马上乱转。"""
        controller = make_controller()
        run(controller, Observation(0.0, 3.0), steps=10)
        command = controller.update(None, 1.0)
        assert command == STOP
        assert controller.state is State.HOLDING

    def test_starts_searching_after_timeout(self):
        """丢失超过 lost_timeout（1 s）：开始原地转圈搜索，速度 0.5 rad/s。"""
        controller = make_controller()
        controller.update(Observation(-0.1, 3.0), 0.0)
        assert controller.update(None, 0.1) == STOP                 # 从 0.1 s 开始计丢失时间
        assert controller.update(None, 1.0) == STOP                 # 才过 0.9 s，还在等
        command = controller.update(None, 1.2)                      # 过了 1.1 s，开始搜索
        assert controller.state is State.SEARCHING
        assert command == Command(0.0, 0.5)

    def test_searches_toward_where_target_vanished(self):
        """目标最后出现在右侧：向右转着找（角速度为负）；在左侧：向左。"""
        for bearing, sign in ((0.4, -1), (-0.4, +1)):
            controller = make_controller(lost_timeout=0.0)
            controller.update(Observation(bearing, 3.0), 0.0)
            assert controller.update(None, 0.1).angular == pytest.approx(sign * 0.5)

    def test_searches_left_if_target_never_seen(self):
        """从没见过目标：向左转着找（默认方向）。"""
        controller = make_controller(lost_timeout=0.0)
        assert controller.update(None, 0.0).angular == pytest.approx(0.5)

    def test_search_can_be_disabled(self):
        """search_speed=0：永远只等待，不转圈。"""
        controller = make_controller(search_speed=0.0)
        controller.update(Observation(0.0, 3.0), 0.0)
        for i in range(1, 100):
            assert controller.update(None, i * DT) == STOP
        assert controller.state is State.HOLDING

    def test_target_reappearing_ends_search(self):
        """搜索中重新看到目标：立即回到跟随，并按新目标转向。"""
        controller = make_controller(lost_timeout=0.0)
        controller.update(None, 0.0)
        assert controller.state is State.SEARCHING
        command = controller.update(Observation(0.2, 3.0), 0.1)
        assert controller.state is State.TRACKING
        assert command.angular == pytest.approx(-0.3)

    def test_loss_timer_restarts_after_each_sighting(self):
        """短暂丢失、又看到、再丢失：超时从第二次丢失重新计时，不会累计。"""
        controller = make_controller()
        controller.update(Observation(0.0, 3.0), 0.0)
        controller.update(None, 0.5)                 # 第一次丢失，等待 0.5 s
        controller.update(Observation(0.0, 3.0), 0.9)   # 看到
        controller.update(None, 1.0)                 # 第二次丢失，重新计时
        controller.update(None, 1.8)                 # 才 0.8 s，不应开始搜索
        assert controller.state is State.HOLDING

    def test_reset_clears_everything(self):
        """reset：回到等待状态，忘掉目标方向和当前速度（下次起步重新加速）。"""
        controller = make_controller(lost_timeout=0.0)
        run(controller, Observation(0.4, 3.0), steps=20)
        controller.reset()
        assert controller.state is State.HOLDING
        first = controller.update(Observation(0.0, 3.0), 10.0)
        assert first.linear == 0.0                   # 从静止重新起步
        controller.reset()
        assert controller.update(None, 11.0).angular == pytest.approx(0.5)   # 默认向左找
