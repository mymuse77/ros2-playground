"""
06 课 · 目标跟随控制器：根据目标的方位角和距离，输出前进速度和转向角速度。

这个模块不依赖 ROS，输入输出都是普通数字，可以直接单元测试。

控制律（两个独立的比例控制，再按对准程度协调）：

    转向：  angular = -angular_gain · bearing           （目标在右 → 向右转）
    前进：  linear  = linear_gain · (distance - target_distance)   （太远才前进，不倒车）
    协调：  linear *= 1 - |bearing| / turn_only_angle   （没对准时先转，对准了再走）

状态机：

    TRACKING ──▶ 看到目标：边转边走
       │  └────▶ ARRIVED：距离和方向都在容差内，不动
       ▼ 看不到目标
    HOLDING ──── 停车等待 lost_timeout 秒（目标可能只是被晃掉了一两帧）
       ▼ 超时
    SEARCHING ── 朝目标最后消失的方向原地转圈找；search_speed=0 则一直等待

知识点：
  - 死区：误差小于容差时输出 0，避免在目标附近来回抖动
  - 限幅：速度和角速度都有上限；线速度还限制加速度，起步更平顺（刹车不限制，要停就立刻停）
  - 不倒车：小车后面没有传感器，看不见后方，所以目标太近时只是停下来
  - 距离未知（目标被截断）时只转向不前进：宁可不动，也不要凭错误的距离撞上去
"""

from dataclasses import asdict, dataclass, fields
import enum
import math

# 两次更新的间隔超过这个值（例如程序卡住后恢复）时，按这个值计算加速度限制，
# 避免一次性放开太大的速度变化
MAX_DT = 0.5


class State(enum.Enum):
    """控制器状态；值是给人看的中文名称。"""

    TRACKING = '跟随'
    ARRIVED = '到位'
    HOLDING = '目标丢失，等待'
    SEARCHING = '搜索'


@dataclass(frozen=True)
class Command:
    """速度指令：linear 为前进速度（m/s），angular 为向左转的角速度（rad/s）。"""

    linear: float = 0.0
    angular: float = 0.0


STOP = Command()


@dataclass(frozen=True)
class FollowerParams:
    """
    跟随参数。字段名就是 ROS 参数名，数值与参数文件 config/follower.yaml 一致。

    角度用"度"表示，方便人读写；需要弧度时用 angle_tolerance、turn_only_angle 属性。
    """

    target_distance: float = 0.8       # 希望保持的距离，m
    distance_tolerance: float = 0.05   # 距离死区，m
    angle_tolerance_deg: float = 2.0   # 方位死区，度
    max_linear_speed: float = 0.15     # 最大前进速度，m/s
    max_angular_speed: float = 1.0     # 最大转向角速度，rad/s
    linear_gain: float = 0.5           # 距离误差 → 前进速度，(m/s)/m
    angular_gain: float = 1.5          # 方位误差 → 角速度，(rad/s)/rad
    turn_only_angle_deg: float = 30.0  # 方位偏差超过此角度时只转向不前进，度
    max_linear_accel: float = 0.3      # 线速度的最大变化率，m/s²
    lost_timeout: float = 1.0          # 看不到目标多久后开始搜索，s
    search_speed: float = 0.5          # 搜索时的转圈角速度，rad/s；0 表示不搜索

    def __post_init__(self):
        """检查参数组合是否合法。"""
        positive = ('target_distance', 'max_linear_speed', 'max_angular_speed',
                    'linear_gain', 'angular_gain', 'turn_only_angle_deg', 'max_linear_accel')
        for name in positive:
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} 必须大于 0，收到 {getattr(self, name)}')
        for name in ('distance_tolerance', 'angle_tolerance_deg', 'lost_timeout',
                     'search_speed'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} 不能为负，收到 {getattr(self, name)}')
        if self.distance_tolerance >= self.target_distance:
            raise ValueError('distance_tolerance 必须小于 target_distance')
        if self.angle_tolerance_deg >= self.turn_only_angle_deg:
            raise ValueError('angle_tolerance_deg 必须小于 turn_only_angle_deg')
        if self.turn_only_angle_deg >= 90.0:
            raise ValueError('turn_only_angle_deg 必须小于 90（目标在侧面时不应该前进）')

    @property
    def angle_tolerance(self):
        """方位死区，弧度。"""
        return math.radians(self.angle_tolerance_deg)

    @property
    def turn_only_angle(self):
        """只转向不前进的方位偏差，弧度。"""
        return math.radians(self.turn_only_angle_deg)


def to_ros_params(params):
    """把参数转成 {ROS 参数名: 值}，用来声明 ROS 参数。"""
    return asdict(params)


def from_ros_params(values):
    """
    把 {ROS 参数名: 值} 转成 FollowerParams。

    会触发 FollowerParams 的合法性检查；缺少的字段或不认识的字段都会报错。
    """
    names = {f.name for f in fields(FollowerParams)}
    if set(values) != names:
        raise ValueError(f'参数名不匹配：多余 {sorted(set(values) - names)}，'
                         f'缺少 {sorted(names - set(values))}')
    return FollowerParams(**{name: float(value) for name, value in values.items()})


def is_stale(last_time, now, timeout):
    """
    数据是否过期。

    :param last_time: 最近一次收到数据的时间；从没收到过为 None（视为过期）
    :param now: 当前时间，与 last_time 使用同一个时钟
    :param timeout: 允许的最大间隔
    """
    return last_time is None or now - last_time > timeout


def _clamp(value, low, high):
    """把 value 限制在 [low, high] 内。"""
    return max(low, min(high, value))


class FollowController:
    """
    目标跟随控制器。

    用法：每个控制周期调用一次 update()，传入最新观测（没看到目标传 None）和当前时间。
    """

    def __init__(self, params=None):
        """创建控制器；params 为 None 时使用默认参数。"""
        self.params = params if params is not None else FollowerParams()
        self.reset()

    def reset(self):
        """回到初始状态：忘掉目标、速度归零。数据中断（相机超时）后调用。"""
        self.state = State.HOLDING
        self._last_time = None      # 上一次 update 的时间
        self._lost_since = None     # 开始看不到目标的时间
        self._last_bearing = None   # 最后一次看到目标时的方位角，决定搜索方向
        self._linear = 0.0          # 上一次输出的线速度，用来限制加速度

    def update(self, observation, now):
        """
        计算本周期的速度指令。

        :param observation: camera_model.Observation；没看到目标为 None
        :param now: 当前时间，s（单调递增即可）
        :return: Command
        """
        dt = 0.0 if self._last_time is None else _clamp(now - self._last_time, 0.0, MAX_DT)
        self._last_time = now

        if observation is not None:
            self._lost_since = None
            self._last_bearing = observation.bearing
            command, self.state = self._track(observation)
        else:
            if self._lost_since is None:
                self._lost_since = now
            command, self.state = self._lost(now)

        command = self._limit_acceleration(command, dt)
        self._linear = command.linear
        return command

    def _track(self, observation):
        """看到目标：返回（指令，状态）。"""
        p = self.params
        bearing = observation.bearing

        # 转向：死区内不动，否则按比例转，方向取反（目标在右 → 向右转 = 角速度为负）
        if abs(bearing) <= p.angle_tolerance:
            angular = 0.0
        else:
            angular = _clamp(-p.angular_gain * bearing, -p.max_angular_speed, p.max_angular_speed)

        # 前进：距离未知时不动；太远才前进，且不倒车
        linear = 0.0
        error = None
        if observation.distance is not None:
            error = observation.distance - p.target_distance
            if error > p.distance_tolerance:
                linear = _clamp(p.linear_gain * error, 0.0, p.max_linear_speed)
        # 没对准时降低前进速度：偏差为 0 时全速，达到 turn_only_angle 时为 0
        linear *= _clamp(1.0 - abs(bearing) / p.turn_only_angle, 0.0, 1.0)

        arrived = (error is not None and error <= p.distance_tolerance
                   and abs(bearing) <= p.angle_tolerance)
        return Command(linear, angular), State.ARRIVED if arrived else State.TRACKING

    def _lost(self, now):
        """看不到目标：先停车等待，超时后朝最后消失的方向转圈搜索。"""
        p = self.params
        searching = p.search_speed > 0 and now - self._lost_since >= p.lost_timeout
        if not searching:
            return STOP, State.HOLDING
        # 目标最后在右边（方位角为正）就向右转（角速度为负）；从没见过目标则向左转
        direction = -1.0 if (self._last_bearing or 0.0) > 0 else 1.0
        return Command(0.0, direction * p.search_speed), State.SEARCHING

    def _limit_acceleration(self, command, dt):
        """
        限制线速度的增长率，起步更平顺；减速不限制，目标丢失或到位时立即刹车。

        转向角速度不限制。
        """
        linear = min(command.linear, self._linear + self.params.max_linear_accel * dt)
        return Command(linear, command.angular)
