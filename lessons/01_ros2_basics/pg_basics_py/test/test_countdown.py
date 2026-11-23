"""倒计时动作：目标校验和客户端参数解析的单元测试。"""

from pg_basics_py.countdown_client_node import parse_args
from pg_basics_py.countdown_server_node import is_valid_seconds, MAX_SECONDS, MIN_SECONDS
import pytest


@pytest.mark.parametrize('seconds', [MIN_SECONDS, 10, MAX_SECONDS])
def test_valid_seconds(seconds):
    """秒数为最小值、10 或最大值：is_valid_seconds() 判定为有效。"""
    assert is_valid_seconds(seconds)


@pytest.mark.parametrize('seconds', [0, MAX_SECONDS + 1])
def test_invalid_seconds(seconds):
    """秒数为 0 或超过最大值：is_valid_seconds() 判定为无效。"""
    assert not is_valid_seconds(seconds)


def test_parse_args_without_cancel():
    """只给秒数 '5'：seconds 为 5，cancel_after 为 None。"""
    args = parse_args(['5'])
    assert args.seconds == 5
    assert args.cancel_after is None


def test_parse_args_with_cancel():
    """给出秒数 10 和 --cancel-after 2.5：两个值都被正确解析。"""
    args = parse_args(['10', '--cancel-after', '2.5'])
    assert args.seconds == 10
    assert args.cancel_after == 2.5


@pytest.mark.parametrize('argv', [[], ['abc'], ['5', '--cancel-after', '0']])
def test_parse_args_rejects_invalid(argv):
    """缺少秒数、秒数非数字或 --cancel-after 为 0：parse_args() 退出（SystemExit）。"""
    # argparse 遇到错误时调用 sys.exit(2)
    with pytest.raises(SystemExit):
        parse_args(argv)
