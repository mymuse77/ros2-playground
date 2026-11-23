"""文档字符串检查：ament 约定 + 忽略 D400/D415（允许中文摘要）。"""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """检查包内所有 Python 文件的文档字符串格式。"""
    rc = main(argv=['--add-ignore', 'D400', 'D415'])
    assert rc == 0, '发现文档字符串格式问题'
