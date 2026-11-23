# 地图

这里存放 04 课 SLAM 建图生成、并在 05 课 Nav2 导航中使用的地图。

每张地图由两个文件组成：

| 文件 | 内容 |
|---|---|
| `<名字>.pgm` | 灰度图：白色 = 空闲，黑色 = 障碍，灰色 = 未知 |
| `<名字>.yaml` | 元数据：图片文件名、分辨率（米/像素）、原点坐标、占用阈值 |

在 launch 文件里通过下面的方式引用：

```python
os.path.join(get_package_share_directory('pg_sim'), 'maps', 'tb3_world.yaml')
```

新地图放进这个目录后需要重新 `colcon build` 才会安装到 `install/` 下。
