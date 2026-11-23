"""pg_obstacle_avoidance_py 包的安装配置：注册节点入口，安装 launch、参数和 RViz 文件。"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pg_obstacle_avoidance_py'

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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richie',
    maintainer_email='mymuse@foxmail.com',
    description='02 课：基于激光雷达的避障漫游',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    # 节点入口：`ros2 run pg_obstacle_avoidance_py <名字>` 中的名字 = 模块:函数
    entry_points={
        'console_scripts': [
        ],
    },
)
