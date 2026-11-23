# 规范约定

## 命名

| 对象 | 规则 | 示例 |
|---|---|---|
| 课程文件夹 | `两位编号_主题` | `02_obstacle_avoidance` |
| 功能包 | `pg_<主题>_<语言后缀>`，共享包不加后缀。包名必须以字母开头，在工作空间里唯一 | `pg_obstacle_avoidance_py`、`pg_safety_guard_rs`、`pg_interfaces` |
| 节点名 | 小写 + 下划线 | `avoider`、`navigate_server` |
| 可执行文件（入口） | 与节点名相同；工具类用动词开头 | `avoider`、`save_map` |
| 话题、服务、动作 | 小写 + 下划线，使用相对名称（不以 `/` 开头），方便重映射和加命名空间 | `scan`、`cmd_vel`、`reset_counter` |
| 参数 | 小写 + 下划线，带单位的写进名字 | `forward_speed`、`safe_distance` |
| 自定义接口 | 大驼峰 | `Greeting.msg`、`ResetCounter.srv`、`NavigateToPoint.action` |

语言后缀：`_py`（Python）、`_rs`（Rust）、`_cpp`（C++）。

## 包的内部结构（Python）

```
pg_xxx_py/
├─ package.xml            包信息与依赖
├─ setup.py               安装配置：节点入口、launch/config 文件安装
├─ setup.cfg              安装路径 + flake8 配置
├─ resource/pg_xxx_py     ament 索引标记文件（空文件）
├─ pg_xxx_py/             Python 模块
│   ├─ __init__.py
│   ├─ <算法>.py          算法：纯 Python，不依赖 ROS，可直接单元测试
│   └─ <名字>_node.py     ROS 节点：只负责收发消息、读参数、调用算法
├─ launch/                launch 文件（*.launch.py）
├─ config/                参数文件（YAML）
├─ rviz/                  RViz 配置（可选）
├─ worlds/                本课专用的 Gazebo 场景（可选）
└─ test/                  代码风格检查 + 单元测试
```

**算法与节点分离**：算法模块以内容命名，不导入 rclpy，输入输出都是普通的 Python 数据；
节点模块以 `_node.py` 结尾，只做"消息 → 普通数据 → 调用算法 → 普通数据 → 消息"的转换。
这样算法可以脱离仿真直接用 pytest 测试，以后移植到 Rust 也更容易。各课的例子：

| 课程 | 算法模块（纯 Python） | 节点模块 |
|---|---|---|
| 02 | `logic.py`（扇区、避障决策）、`safety.py`（碰撞检测） | `avoider_node.py`、`safety_monitor_node.py` |
| 03 | `geometry.py`、`go_to_goal.py`、`bug2.py` | `go_to_goal_node.py`、`navigate_server_node.py`、`navigate_client_node.py` |
| 04 | `map_io.py`（地图文件读写） | `map_saver_node.py` |

多个节点共用、但依赖 ROS 的辅助代码（消息转换、退出停车等）放在单独的模块里，例如 03 课的 `ros_utils.py`。

## 代码风格

- 遵循 ROS 2 官方的 Python 风格（ament_flake8 + ament_pep257）：
  - 单引号字符串；每行最多 99 个字符；import 按 Google 风格分组排序
  - 多行文档字符串的摘要写在第二行（`"""` 后换行）
- 每个模块、类、公开函数都写文档字符串，说明用途、参数、返回值、单位
- 注释使用中文，标识符使用英文
- 物理量统一使用 SI 单位：米、秒、弧度、米/秒、弧度/秒；参数里出现角度时用"度"并在名字里标明（如 `front_half_angle_deg`），代码内部立即转换成弧度
- 中文文档字符串的摘要不强制以英文句点结尾（各包的 `setup.cfg` 里关闭了 D400/D415 两条规则），其余规则与 ROS 2 默认一致

## 测试

每个 Python 包都带三类测试，用 `colcon test` 运行：

| 测试 | 作用 |
|---|---|
| `test_flake8.py` | 代码风格检查 |
| `test_pep257.py` | 文档字符串检查 |
| `test_<算法模块>.py` | 算法的单元测试，例如 `test_logic.py`、`test_bug2.py` |

- 每个测试函数写一行中文文档字符串，说明"测的是什么场景、期望什么结果"，测试失败时一眼能看懂
- 和仿真相关的算法尽量加一个闭环测试（用简单的运动学模型代替仿真），例如 03 课的射线模拟雷达
- 仿真里发现的 bug，修复时补一个能复现它的单元测试

提交前必须保证 `colcon build` 和 `colcon test` 都通过。

## 提交信息

```
<范围>: <做了什么>

<可选：为什么这样做、要注意什么>
```

范围使用 `lesson01` … `lesson10`、`common`、`docs`、`setup`、`chore`，例如：

```
lesson02: 添加雷达扇区分析算法与单元测试
```

- 一次提交只做一件事，提交后代码能编译、测试能通过
- 每完成一课，打一个 tag：`lesson-01`、`lesson-02` …
- 完成 README 里 TODO 清单的某一项时，在同一次提交里把它勾上

## 每课 README

统一使用 [lesson_template.md](lesson_template.md) 的结构。
