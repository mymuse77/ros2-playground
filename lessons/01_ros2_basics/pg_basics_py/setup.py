"""pg_basics_py 包的安装配置：注册节点入口，安装 launch 和参数文件。"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pg_basics_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引标记文件：让 `ros2 pkg`、`ros2 run` 能找到这个包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richie',
    maintainer_email='mymuse@foxmail.com',
    description='01 课：ROS 2 核心概念的 Python 示例',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    # 节点入口：`ros2 run pg_basics_py <名字>` 中的名字 = 模块:函数
    entry_points={
        'console_scripts': [
            'greeter = pg_basics_py.greeter_node:main',
            'listener = pg_basics_py.listener_node:main',
            'reset_client = pg_basics_py.reset_client_node:main',
            'countdown_server = pg_basics_py.countdown_server_node:main',
            'countdown_client = pg_basics_py.countdown_client_node:main',
        ],
    },
)
