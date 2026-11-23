"""
计数器与消息序号检查（纯 Python，不依赖 ROS）。

这个模块里的代码不导入 rclpy，可以脱离 ROS 直接用 pytest 测试。
节点只负责收发消息，具体逻辑都放在这里——这是本仓库统一的写法。
"""

# 自定义消息里 count 字段的类型是 uint32，取值范围 0 ~ 2^32-1
UINT32_MAX = 2 ** 32 - 1

# check_sequence() 返回的状态
SEQ_FIRST = 'first'   # 收到的第一条消息
SEQ_OK = 'ok'         # 序号连续
SEQ_GAP = 'gap'       # 序号跳跃：中间有消息丢失
SEQ_RESET = 'reset'   # 序号变小或不变：发布端的计数器被重置了


class Counter:
    """
    单调递增的计数器，可以重置到任意起始值。

    value 属性表示"下一次调用 take() 将返回的值"。
    超过 uint32 上限后回绕到 0，与消息字段的取值范围保持一致。
    """

    def __init__(self, start=0):
        """
        创建计数器。

        :param start: 起始值，范围 0 ~ UINT32_MAX
        :raises ValueError: 起始值超出范围或不是整数
        """
        self._value = self._validate(start)

    @property
    def value(self):
        """下一次调用 take() 将返回的值。"""
        return self._value

    def take(self):
        """返回当前值，然后加一（超过上限回绕到 0）。"""
        current = self._value
        self._value = 0 if current == UINT32_MAX else current + 1
        return current

    def reset(self, start=0):
        """
        把计数器重置为 start。

        :param start: 新的起始值，范围 0 ~ UINT32_MAX
        :return: 重置前的 value
        :raises ValueError: 起始值超出范围或不是整数
        """
        previous = self._value
        self._value = self._validate(start)
        return previous

    @staticmethod
    def _validate(value):
        """检查取值是否为 uint32 范围内的整数（bool 虽然是 int 的子类，也不接受）。"""
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f'计数器的值必须是整数，收到 {value!r}')
        if not 0 <= value <= UINT32_MAX:
            raise ValueError(f'计数器的值必须在 0 ~ {UINT32_MAX} 之间，收到 {value}')
        return value


def check_sequence(previous, current):
    """
    根据前后两条消息的序号，判断消息流是否连续。

    :param previous: 上一条消息的序号；还没收到过消息时为 None
    :param current: 当前消息的序号
    :return: (状态, 丢失条数)。状态为 SEQ_FIRST / SEQ_OK / SEQ_GAP / SEQ_RESET 之一，
             丢失条数只在 SEQ_GAP 时大于 0
    """
    if previous is None:
        return SEQ_FIRST, 0
    # 计数器从上限回绕到 0 也算连续
    expected = 0 if previous == UINT32_MAX else previous + 1
    if current == expected:
        return SEQ_OK, 0
    if current > expected:
        return SEQ_GAP, current - expected
    return SEQ_RESET, 0
