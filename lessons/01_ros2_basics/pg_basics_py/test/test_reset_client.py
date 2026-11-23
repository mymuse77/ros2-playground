"""reset_client 命令行参数解析的单元测试。"""

from pg_basics_py.counter import UINT32_MAX
from pg_basics_py.reset_client_node import parse_start_value
import pytest


def test_default_is_zero():
    """命令行不带参数：起始值默认为 0。"""
    assert parse_start_value(['reset_client']) == 0


def test_parses_value():
    """命令行参数为 '100'：解析为整数 100。"""
    assert parse_start_value(['reset_client', '100']) == 100


def test_accepts_upper_bound():
    """参数恰好为 UINT32_MAX：接受并原样返回。"""
    assert parse_start_value(['reset_client', str(UINT32_MAX)]) == UINT32_MAX


@pytest.mark.parametrize('bad', ['-1', str(UINT32_MAX + 1), 'abc', '1.5'])
def test_rejects_invalid(bad):
    """参数为负数、超出 uint32、非数字或小数：抛出 ValueError。"""
    with pytest.raises(ValueError):
        parse_start_value(['reset_client', bad])
