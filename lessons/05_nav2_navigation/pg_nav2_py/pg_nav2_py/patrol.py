"""
05 课 · 巡逻逻辑：按顺序循环走航点、失败怎么办、记录每个点的结果和用时。

    PatrolSchedule   下一个去哪：第几圈、第几个航点；失败的点跳过，连续失败太多次就停
    PatrolStats      每个航点去了几次、成功几次、平均用时，结束时打印汇总

和 Nav2 打交道的部分（发目标、等结果）在 patrol_node.py 里，这里只有纯逻辑，
可以不启动仿真直接用 pytest 测试。
"""

from dataclasses import dataclass
from enum import Enum


class GoalOutcome(Enum):
    """去一个航点的结果。"""

    SUCCEEDED = '成功'
    FAILED = '失败'        # Nav2 放弃了：规划不出路径，或恢复行为都试过了还是走不动
    TIMEOUT = '超时'       # 超过 goal_timeout 还没到，巡逻节点主动取消
    CANCELED = '取消'      # 被别人取消（例如在 RViz 里发了新目标）


@dataclass(frozen=True)
class Leg:
    """
    巡逻中的一段：第几圈的第几个航点。

    :ivar loop: 圈数，从 1 开始
    :ivar index: 航点序号，从 0 开始
    :ivar waypoint: 航点
    """

    loop: int
    index: int
    waypoint: object


class PatrolSchedule:
    """
    决定下一个去哪个航点。

    规则：
      - 按顺序走完所有航点算一圈，loops 圈后结束；loops = 0 表示一直巡逻
      - 某个点失败（任何非成功结果）就跳过它去下一个，巡逻不中断
      - 连续失败 max_consecutive_failures 次说明系统出了问题（定位丢了、被困住了），停止巡逻
    """

    def __init__(self, waypoints, loops=0, max_consecutive_failures=3):
        """
        创建巡逻计划，从第 1 圈的第一个航点开始。

        :param waypoints: 航点序列，至少一个
        :param loops: 巡逻圈数，0 表示无限
        :param max_consecutive_failures: 连续失败多少次后停止，至少 1
        """
        if not waypoints:
            raise ValueError('至少需要一个航点')
        if loops < 0:
            raise ValueError(f'loops 不能为负数：{loops}')
        if max_consecutive_failures < 1:
            raise ValueError(f'max_consecutive_failures 至少为 1：{max_consecutive_failures}')
        self._waypoints = tuple(waypoints)
        self._loops = loops
        self._max_failures = max_consecutive_failures
        self._loop = 1
        self._index = 0
        self._consecutive_failures = 0
        self._stop_reason = None

    @property
    def current(self):
        """当前要去的 Leg；巡逻已结束时为 None。"""
        if self._stop_reason is not None:
            return None
        return Leg(self._loop, self._index, self._waypoints[self._index])

    @property
    def finished(self):
        """巡逻是否已结束。"""
        return self._stop_reason is not None

    @property
    def stop_reason(self):
        """结束原因（字符串）；还没结束时为 None。"""
        return self._stop_reason

    @property
    def completed_loops(self):
        """已经走完的圈数。"""
        return self._loop - 1

    def report(self, outcome):
        """
        报告当前航点的结果，并前进到下一个航点。

        :param outcome: GoalOutcome
        :raises RuntimeError: 巡逻已经结束
        """
        if self.finished:
            raise RuntimeError('巡逻已经结束，不能再报告结果')

        if outcome is GoalOutcome.SUCCEEDED:
            self._consecutive_failures = 0
        else:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self._max_failures:
                self._stop_reason = f'连续 {self._consecutive_failures} 个航点失败'
                return

        self._index += 1
        if self._index == len(self._waypoints):
            self._index = 0
            self._loop += 1
            if self._loops and self._loop > self._loops:
                # 圈数由 completed_loops 给出，原因里不再重复
                self._stop_reason = '跑完设定的圈数'

    def stop(self, reason):
        """从外部结束巡逻（例如按了 Ctrl+C）。已经结束时保留原来的原因。"""
        if self._stop_reason is None:
            self._stop_reason = reason


@dataclass
class WaypointRecord:
    """
    一个航点的累计统计。

    :ivar attempts: 去了几次
    :ivar successes: 成功几次
    :ivar success_time: 成功的那几次一共用了多少秒（仿真时间）
    """

    attempts: int = 0
    successes: int = 0
    success_time: float = 0.0

    @property
    def mean_time(self):
        """成功时的平均用时，s；从没成功过时为 None。"""
        return self.success_time / self.successes if self.successes else None


class PatrolStats:
    """记录每次去航点的结果和用时，生成汇总。"""

    def __init__(self, names):
        """
        为每个航点建一条空记录。

        :param names: 航点名字，按巡逻顺序；汇总时也按这个顺序列出
        """
        self._records = {name: WaypointRecord() for name in names}
        self._outcomes = {outcome: 0 for outcome in GoalOutcome}

    def record(self, name, outcome, duration):
        """
        记录一次结果。

        :param name: 航点名字
        :param outcome: GoalOutcome
        :param duration: 用时，s（失败、超时时也记录，但不计入平均用时）
        :raises KeyError: 未知的航点名字
        """
        rec = self._records[name]
        rec.attempts += 1
        self._outcomes[outcome] += 1
        if outcome is GoalOutcome.SUCCEEDED:
            rec.successes += 1
            rec.success_time += duration

    def get(self, name):
        """某个航点的统计 WaypointRecord。"""
        return self._records[name]

    @property
    def attempts(self):
        """一共去了多少次航点。"""
        return sum(self._outcomes.values())

    def count(self, outcome):
        """某种结果出现了几次。"""
        return self._outcomes[outcome]

    @property
    def success_rate(self):
        """成功率 0 ~ 1；还没去过任何航点时为 None。"""
        total = self.attempts
        return self._outcomes[GoalOutcome.SUCCEEDED] / total if total else None

    def summary_lines(self):
        """
        汇总，每行一个字符串，便于逐行打日志。

        例如：
            航点 1 次，成功率 100%（成功 1、失败 0、超时 0、取消 0）
              north    成功 1/1  平均 23.4 s
        """
        rate = self.success_rate
        rate_text = '—' if rate is None else f'{rate:.0%}'
        counts = '、'.join(f'{o.value} {n}' for o, n in self._outcomes.items())
        lines = [f'航点 {self.attempts} 次，成功率 {rate_text}（{counts}）']
        width = max(len(name) for name in self._records)
        for name, rec in self._records.items():
            mean = '—' if rec.mean_time is None else f'{rec.mean_time:.1f} s'
            lines.append(f'  {name:<{width}}  成功 {rec.successes}/{rec.attempts}  平均 {mean}')
        return lines
