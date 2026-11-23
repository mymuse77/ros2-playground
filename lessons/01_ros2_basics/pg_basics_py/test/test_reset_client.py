"""reset_client 命令行参数解析的单元测试。"""

from pg_basics_py.counter import UINT32_MAX
from pg_basics_py.reset_client_node import parse_start_value
import pytest


def test_default_is_zero():
    assert parse_start_value(['reset_client']) == 0


def test_parses_value():
    assert parse_start_value(['reset_client', '100']) == 100


def test_accepts_upper_bound():
    assert parse_start_value(['reset_client', str(UINT32_MAX)]) == UINT32_MAX


@pytest.mark.parametrize('bad', ['-1', str(UINT32_MAX + 1), 'abc', '1.5'])
def test_rejects_invalid(bad):
    with pytest.raises(ValueError):
        parse_start_value(['reset_client', bad])
