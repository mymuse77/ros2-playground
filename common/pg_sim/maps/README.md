# 地图

这里存放 04 课 SLAM 建图生成、并在 05 课 Nav2 导航中使用的地图。

| 地图 | 场景 | 生成方式 |
|---|---|---|
| `tb3_world` | turtlebot3_world（`world:=world`），小车从默认出生点 (-2.0, -0.5) 出发 | 04 课自动漫游建图，见下文 |

每张地图由两个文件组成：

| 文件 | 内容 |
|---|---|
| `<名字>.pgm` | 灰度图：白色 = 空闲，黑色 = 障碍，灰色 = 未知 |
| `<名字>.yaml` | 元数据：图片文件名、分辨率（米/像素）、原点坐标、占用阈值 |

格式的详细说明见 [04 课 README](../../../lessons/04_slam_mapping/README.md) 的知识点 4。

**地图坐标系的原点就是建图开始时小车的位置**（Gazebo 世界坐标的出生点 (-2.0, -0.5)，车头朝 +x）。
05 课导航时小车也从同一个出生点出发，所以初始位姿是地图上的 (0, 0, 0)。

## 重新生成

```bash
ros2 launch pg_slam_py slam_sim.launch.py explore:=auto      # 或者用键盘遥控
# 地图完整后（save_map --stats-only 看空闲面积不再增长），先停车再保存：
ros2 run pg_slam_py save_map ~/ros2_ws/src/ros2-playground/common/pg_sim/maps/tb3_world
```

## 在 launch 文件里引用

```python
os.path.join(get_package_share_directory('pg_sim'), 'maps', 'tb3_world.yaml')
```

新地图放进这个目录后需要重新 `colcon build` 才会安装到 `install/` 下
（`--symlink-install` 时是符号链接，修改已有地图不用重新编译）。
