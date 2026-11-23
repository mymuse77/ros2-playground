"""patrol 的单元测试：巡逻顺序、圈数、失败跳过与停止、统计汇总。"""

from pg_nav2_py.patrol import GoalOutcome, PatrolSchedule, PatrolStats
import pytest

OK = GoalOutcome.SUCCEEDED
FAIL = GoalOutcome.FAILED
TIMEOUT = GoalOutcome.TIMEOUT


def visit_order(schedule, outcomes):
    """按给定结果依次报告，返回去过的 (圈, 航点) 列表。"""
    visited = []
    for outcome in outcomes:
        leg = schedule.current
        if leg is None:
            break
        visited.append((leg.loop, leg.waypoint))
        schedule.report(outcome)
    return visited


class TestPatrolSchedule:
    """下一个去哪。"""

    def test_order_and_loops(self):
        """按顺序走，走完一圈从头开始，loops 圈后结束。"""
        schedule = PatrolSchedule('abc', loops=2)
        visited = visit_order(schedule, [OK] * 10)
        assert visited == [(1, 'a'), (1, 'b'), (1, 'c'), (2, 'a'), (2, 'b'), (2, 'c')]
        assert schedule.finished
        assert schedule.current is None
        assert schedule.completed_loops == 2
        assert schedule.stop_reason == '跑完设定的圈数'

    def test_infinite(self):
        """圈数为 0 时一直巡逻。"""
        schedule = PatrolSchedule('ab', loops=0)
        visit_order(schedule, [OK] * 101)
        assert not schedule.finished
        assert schedule.completed_loops == 50
        assert schedule.current.waypoint == 'b'

    def test_failure_is_skipped(self):
        """某个点失败后跳过它，继续去下一个点。"""
        schedule = PatrolSchedule('abc', loops=1, max_consecutive_failures=3)
        visited = visit_order(schedule, [OK, FAIL, OK])
        assert visited == [(1, 'a'), (1, 'b'), (1, 'c')]
        assert schedule.stop_reason == '跑完设定的圈数'
        assert schedule.completed_loops == 1

    def test_consecutive_failures_stop(self):
        """连续失败达到上限时停止；成功一次计数清零。"""
        schedule = PatrolSchedule('abcd', loops=0, max_consecutive_failures=2)
        visit_order(schedule, [FAIL, OK, TIMEOUT])     # 失败、成功（清零）、超时：只连续 1 次
        assert not schedule.finished
        schedule.report(FAIL)                         # 连续第 2 次
        assert schedule.finished
        assert schedule.stop_reason == '连续 2 个航点失败'

    def test_single_waypoint(self):
        """只有一个航点时，每到一次算一圈。"""
        schedule = PatrolSchedule(['home'], loops=3)
        assert len(visit_order(schedule, [OK] * 5)) == 3

    def test_report_after_finish(self):
        """结束后不能再报告结果。"""
        schedule = PatrolSchedule('a', loops=1)
        schedule.report(OK)
        with pytest.raises(RuntimeError):
            schedule.report(OK)

    def test_external_stop(self):
        """外部停止（Ctrl+C）；已经结束时保留原来的原因。"""
        schedule = PatrolSchedule('ab')
        schedule.stop('Ctrl+C')
        assert schedule.current is None
        assert schedule.stop_reason == 'Ctrl+C'
        schedule.stop('别的原因')
        assert schedule.stop_reason == 'Ctrl+C'

    @pytest.mark.parametrize('kwargs', [
        {'waypoints': []},
        {'waypoints': 'a', 'loops': -1},
        {'waypoints': 'a', 'max_consecutive_failures': 0},
    ])
    def test_invalid_arguments(self, kwargs):
        """参数检查。"""
        with pytest.raises(ValueError):
            PatrolSchedule(**kwargs)


class TestPatrolStats:
    """结果统计。"""

    def test_counts_and_mean(self):
        """平均用时只算成功的那几次。"""
        stats = PatrolStats(['a', 'b'])
        stats.record('a', OK, 10.0)
        stats.record('a', OK, 20.0)
        stats.record('a', TIMEOUT, 99.0)
        stats.record('b', FAIL, 5.0)
        a = stats.get('a')
        assert (a.attempts, a.successes) == (3, 2)
        assert a.mean_time == pytest.approx(15.0)
        assert stats.get('b').mean_time is None
        assert stats.attempts == 4
        assert stats.count(OK) == 2
        assert stats.count(GoalOutcome.CANCELED) == 0
        assert stats.success_rate == pytest.approx(0.5)

    def test_empty(self):
        """还没去过任何航点时成功率为 None，汇总也能正常生成。"""
        stats = PatrolStats(['a'])
        assert stats.success_rate is None
        assert '成功率 —' in stats.summary_lines()[0]

    def test_unknown_name(self):
        """未知的航点名字报 KeyError。"""
        with pytest.raises(KeyError):
            PatrolStats(['a']).record('x', OK, 1.0)

    def test_summary(self):
        """汇总：第一行是总体，之后每个航点一行，按巡逻顺序。"""
        stats = PatrolStats(['north', 'home'])
        stats.record('north', OK, 23.44)
        stats.record('home', FAIL, 3.0)
        lines = stats.summary_lines()
        assert lines[0] == '航点 2 次，成功率 50%（成功 1、失败 1、超时 0、取消 0）'
        assert lines[1] == '  north  成功 1/1  平均 23.4 s'
        assert lines[2] == '  home   成功 0/1  平均 —'
