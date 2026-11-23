"""代码风格检查：使用包内 setup.cfg 的 [flake8] 配置运行 ament_flake8。"""

import os

from ament_flake8.main import main_with_errors
import pytest

# setup.cfg 与 test/ 目录同级
CONFIG = os.path.join(os.path.dirname(__file__), '..', 'setup.cfg')


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """检查包内所有 Python 文件的代码风格。"""
    rc, errors = main_with_errors(argv=['--config', CONFIG])
    assert rc == 0, '发现 %d 处代码风格问题：\n' % len(errors) + '\n'.join(errors)
