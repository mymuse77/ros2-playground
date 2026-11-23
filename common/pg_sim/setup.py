"""pg_sim 包的安装配置：本包只有数据文件（launch、RViz 配置、地图），没有节点。"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'pg_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引标记文件：让 `ros2 pkg` 等工具能找到这个包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装到 share/pg_sim/ 下，其他包通过 get_package_share_directory('pg_sim') 引用
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richie',
    maintainer_email='mymuse@foxmail.com',
    description='ROS 2 Playground 共享仿真资源',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
