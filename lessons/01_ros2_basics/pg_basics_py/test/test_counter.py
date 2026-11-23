"""counter 模块的单元测试（不需要启动 ROS）。"""

from pg_basics_py.counter import (check_sequence, Counter, SEQ_FIRST, SEQ_GAP, SEQ_OK,
                                  SEQ_RESET, UINT32_MAX)
import pytest


class TestCounter:
    """Counter 类的行为。"""

    def test_take_returns_current_then_increments(self):
        counter = Counter(start=5)
        assert counter.take() == 5
        assert counter.take() == 6
        assert counter.value == 7

    def test_default_start_is_zero(self):
        assert Counter().take() == 0

    def test_reset_returns_previous_value(self):
        counter = Counter(start=0)
        counter.take()
        counter.take()
        assert counter.reset(100) == 2
        assert counter.take() == 100

    def test_wraps_around_at_uint32_max(self):
        counter = Counter(start=UINT32_MAX)
        assert counter.take() == UINT32_MAX
        assert counter.take() == 0

    @pytest.mark.parametrize('bad', [-1, UINT32_MAX + 1, 1.5, '3', True, None])
    def test_rejects_invalid_values(self, bad):
        with pytest.raises(ValueError):
            Counter(start=bad)
        with pytest.raises(ValueError):
            Counter().reset(bad)


class TestCheckSequence:
    """check_sequence() 的各种情况。"""

    def test_first_message(self):
        assert check_sequence(None, 42) == (SEQ_FIRST, 0)

    def test_consecutive(self):
        assert check_sequence(3, 4) == (SEQ_OK, 0)

    def test_gap_reports_missing_count(self):
        assert check_sequence(3, 7) == (SEQ_GAP, 3)

    @pytest.mark.parametrize('current', [0, 3, 2])
    def test_reset_when_not_increasing(self, current):
        assert check_sequence(3, current) == (SEQ_RESET, 0)

    def test_wrap_around_is_consecutive(self):
        assert check_sequence(UINT32_MAX, 0) == (SEQ_OK, 0)
