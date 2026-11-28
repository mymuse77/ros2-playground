"""pg_vision_py 包的安装配置：注册节点入口，安装 launch、参数、场景和 RViz 文件。"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pg_vision_py'

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
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richie',
    maintainer_email='mymuse@foxmail.com',
    description='06 课：相机图像处理与颜色跟随',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    # 节点入口：`ros2 run pg_vision_py <名字>` 中的名字 = 模块:函数
    entry_points={
        'console_scripts': [
            'color_follower = pg_vision_py.color_follower_node:main',
        ],
    },
)
