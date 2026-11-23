# 规范约定

## 命名

| 对象 | 规则 | 示例 |
|---|---|---|
| 课程文件夹 | `两位编号_主题` | `02_obstacle_avoidance` |
| 功能包 | `pg_<主题>_<语言后缀>`，共享包不加后缀。包名必须以字母开头，在工作空间里唯一 | `pg_obstacle_avoidance_py`、`pg_safety_guard_rs`、`pg_interfaces` |
| 节点名 | 小写 + 下划线 | `obstacle_avoider` |
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
│   ├─ xxx_logic.py       算法：纯 Python，不依赖 ROS，可直接单元测试
│   └─ xxx_node.py        ROS 节点：只负责收发消息、读参数、调用算法
├─ launch/                launch 文件
├─ config/                参数文件（YAML）
└─ test/                  代码风格检查 + 单元测试
```

**算法与节点分离**：算法写在 `*_logic.py` 里，不导入 rclpy，输入输出都是普通的 Python 数据。节点只做"消息 → 普通数据 → 调用算法 → 普通数据 → 消息"的转换。这样算法可以脱离仿真直接用 pytest 测试，以后移植到 Rust 也更容易。

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
| `test_*_logic.py` | 算法的单元测试 |

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
