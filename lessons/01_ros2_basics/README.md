# 01 ROS 2 基础：话题、服务、参数、动作、launch

> 用一个"问候 + 倒计时"的小程序，把 ROS 2 的五个核心概念各写一遍，后面所有课程都建立在这些概念上。

## 学习目标

- 能说清话题、服务、动作三种通信方式的区别，知道什么场景该用哪一种
- 会写发布者、订阅者、服务端/客户端、动作服务端/客户端
- 会声明带取值范围的参数，并让运行时修改立即生效
- 会写 launch 文件和参数文件，一条命令启动多个节点，并能加命名空间
- 会用 `ros2` 命令行工具观察和调试：`topic`、`service`、`param`、`action`、`bag`

## 前置条件

- 已完成的课程：无（第一课）
- 需要额外安装的软件包：无（安装脚本已经装好 ROS 2 Jazzy）
- 已编译工作空间：

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to pg_basics_py
source install/setup.bash
```

## 知识点

### 三种通信方式

| | 话题 topic | 服务 service | 动作 action |
|---|---|---|---|
| 模式 | 发布 / 订阅，一对多 | 请求 / 响应，一问一答 | 目标 / 反馈 / 结果 |
| 是否等待 | 发完就走，不等对方 | 客户端等待响应 | 客户端可以边等边看进度 |
| 能否取消 | — | 不能 | 能 |
| 典型用途 | 传感器数据、速度指令 | 重置、查询、开关 | 导航到某点、机械臂抓取 |
| 本课示例 | `/greeting` | `/reset_counter` | `/countdown` |

选择的经验：**持续的数据流用话题；很快就能完成的操作用服务；耗时长、要看进度或可能中途取消的任务用动作。**

### 本课的节点关系

```
                    /greeting (Greeting, 话题)
   ┌──────────┐  ─────────────────────────────▶  ┌──────────┐
   │ greeter  │                                   │ listener │
   └──────────┘                                   └──────────┘
        ▲  /reset_counter (ResetCounter, 服务)
        │
   reset_client / ros2 service call

   ┌──────────────────┐   /countdown (Countdown, 动作)
   │ countdown_server │ ◀───────────────────────────  countdown_client / ros2 action send_goal
   └──────────────────┘
```

### 自定义接口

接口定义在公共包 [`pg_interfaces`](../../common/pg_interfaces/)，本课用到三个：

| 接口 | 内容 |
|---|---|
| `msg/Greeting` | `stamp` 时间戳、`text` 文字、`count` 序号 |
| `srv/ResetCounter` | 请求：`start_value`；响应：`success`、`previous_value`、`message` |
| `action/Countdown` | 目标：`seconds`；反馈：`remaining`；结果：`completed`、`remaining` |

查看接口定义：`ros2 interface show pg_interfaces/action/Countdown`

### 参数与参数回调

参数是节点的"配置项"，可以在启动时指定（命令行 `-p` 或 YAML 文件），也可以运行时修改。

- **描述符**（`ParameterDescriptor`）：写说明、限制取值范围。超出范围的修改由 ROS 自动拒绝
- **修改前回调**（`add_on_set_parameters_callback`）：检查描述符表达不了的规则，可以拒绝修改。只做判断，不要在这里改变节点状态
- **修改后回调**（`add_post_set_parameters_callback`）：修改已经成功，在这里让新值生效（比如按新频率重建定时器）

### 执行器与回调

`rclpy.spin()` 是一个事件循环：定时器到期、收到消息、收到请求时，调用对应的回调函数。
默认的单线程执行器同一时间只运行一个回调。如果某个回调耗时很长（比如倒计时的执行回调），
其他回调就要排队等它结束。所以动作服务端用了 **多线程执行器 + 可重入回调组**，
倒计时进行中也能处理取消请求。

## 一键运行

```bash
ros2 launch pg_basics_py basics.launch.py
```

预期效果：终端里 greeter 以 2 Hz 发布 `Hello from launch`，listener 同步打印收到的消息和延迟（约 0.5 ms）。
countdown_server 显示"倒计时动作服务端已就绪"。

另开一个终端试试：

```bash
ros2 service call /reset_counter pg_interfaces/srv/ResetCounter "{start_value: 100}"
ros2 action send_goal --feedback /countdown pg_interfaces/action/Countdown "{seconds: 5}"
```

## 实验步骤

> 每个实验都需要新开终端。新终端里先执行 `source ~/ros2_ws/install/setup.bash`。

### 实验 1：话题——发布与订阅

1. 终端 A：`ros2 run pg_basics_py greeter`
2. 终端 B：`ros2 run pg_basics_py listener`
3. 终端 C：依次执行下面的命令，观察输出

   ```bash
   ros2 node list                    # 有哪些节点
   ros2 topic list -t                # 有哪些话题及其类型
   ros2 topic info /greeting -v      # 谁在发布、谁在订阅、QoS 是什么
   ros2 topic hz /greeting           # 实际频率
   ros2 topic echo /greeting         # 消息内容
   rqt_graph                         # 图形化查看节点和话题的连接关系
   ```

4. 观察 listener 打印的第一条消息的延迟。
5. **思考**：为什么第一条消息的延迟明显比后面的大？（提示：节点之间需要先互相"发现"，再建立连接）
6. 关掉终端 A 的 greeter 再重新启动，listener 会打印什么？为什么？

### 实验 2：服务——重置计数器

1. 保持 greeter 和 listener 在运行
2. 用命令行调用服务：

   ```bash
   ros2 service list -t
   ros2 service call /reset_counter pg_interfaces/srv/ResetCounter "{start_value: 100}"
   ```

3. 用代码调用服务：`ros2 run pg_basics_py reset_client 7`
4. 观察 listener 的输出：它会提示"发布端计数器被重置"。
5. 试试非法参数：`ros2 run pg_basics_py reset_client -- -5`，观察退出码 `echo $?`
6. **思考**：关掉 greeter 后再运行 `reset_client`，会发生什么？代码里是怎么处理的？

### 实验 3：参数——运行时修改

1. 保持 greeter 在运行，另开一个终端运行 `ros2 topic hz /greeting`
2. 查看和修改参数：

   ```bash
   ros2 param list /greeter
   ros2 param describe /greeter publish_rate_hz        # 能看到取值范围 0.1 ~ 50
   ros2 param set /greeter publish_rate_hz 10.0        # 观察 topic hz 的输出变化
   ros2 param set /greeter publish_rate_hz 100.0       # 超出范围，被拒绝
   ros2 param set /greeter greeting_text "Hi ROS"
   ros2 param set /greeter greeting_text "''"          # 空字符串，被拒绝
   ros2 param dump /greeter                            # 导出当前参数（YAML 格式）
   ```

3. 启动时指定参数：`ros2 run pg_basics_py greeter --ros-args -p publish_rate_hz:=5.0`
4. **思考**：100.0 和空字符串都被拒绝了，但拒绝它们的分别是谁？（描述符 / 修改前回调）

### 实验 4：动作——可取消的倒计时

1. 终端 A：`ros2 run pg_basics_py countdown_server`
2. 终端 B：依次尝试

   ```bash
   ros2 action list -t
   ros2 action info /countdown
   ros2 action send_goal --feedback /countdown pg_interfaces/action/Countdown "{seconds: 5}"
   ros2 run pg_basics_py countdown_client 10 --cancel-after 3   # 3 秒后取消
   ros2 run pg_basics_py countdown_client 0                     # 非法目标，被拒绝
   ```

3. 并发测试：在终端 B 运行 `countdown_client 10`，同时在终端 C 运行 `countdown_client 2`。后发的目标会被拒绝。
4. **思考**：如果把 `countdown_server_node.py` 的 `main()` 改成 `rclpy.spin(node)`（单线程执行器），取消还能及时生效吗？动手改一下验证。

### 实验 5：launch 与命名空间

```bash
ros2 launch pg_basics_py basics.launch.py --show-args
ros2 launch pg_basics_py basics.launch.py                     # 终端 A
ros2 launch pg_basics_py basics.launch.py namespace:=robot1   # 终端 B
ros2 node list                                                # 终端 C：能看到两套节点
ros2 topic echo /robot1/greeting
```

**思考**：两套节点同时运行，为什么 `/listener` 只收到 `/greeting`，不会收到 `/robot1/greeting`？
（提示：代码里的话题名是相对名称 `greeting`，命名空间会加在前面）

修改 [`config/basics.yaml`](pg_basics_py/config/basics.yaml) 里的频率后重新 launch，不用重新编译就能生效
（前提是用了 `--symlink-install` 编译）。

### 实验 6：bag 录制与回放

bag 把话题数据录下来，之后可以反复回放。调试真实机器人时非常有用：采集一次数据，离线反复调算法。

```bash
mkdir -p ~/bags && cd ~/bags

# 1. 录制：greeter 以 5 Hz 发布，录 5 秒左右后按 Ctrl+C
ros2 run pg_basics_py greeter --ros-args -p publish_rate_hz:=5.0     # 终端 A
ros2 bag record -o greeting_bag --topics /greeting                    # 终端 B

# 2. 查看 bag 信息：时长、消息数、话题类型
ros2 bag info greeting_bag

# 3. 关掉 greeter，只开 listener，然后回放
ros2 run pg_basics_py listener                                        # 终端 A
ros2 bag play greeting_bag                                            # 终端 B

# 4. 其他回放方式
ros2 bag play greeting_bag --delay 2.0          # 等 2 秒再开始，避免开头丢消息
ros2 bag play greeting_bag --rate 2.0 --loop    # 2 倍速循环回放
```

观察并思考：

1. 回放时 listener 打印的"延迟"有好几秒甚至更大，为什么？
   （提示：回放的消息保留了录制时的 `stamp`，延迟算的其实是"现在 - 录制时刻"）
2. 不加 `--delay` 时，listener 收到的条数可能比 bag 里的少，为什么？
3. 循环回放时，listener 会提示"计数器被重置"，为什么？

## 代码导读

| 文件 | 作用 |
|---|---|
| [`counter.py`](pg_basics_py/pg_basics_py/counter.py) | 计数器和序号检查，纯 Python，不依赖 ROS |
| [`greeter_node.py`](pg_basics_py/pg_basics_py/greeter_node.py) | 发布者 + 服务端 + 参数（本课的核心文件） |
| [`listener_node.py`](pg_basics_py/pg_basics_py/listener_node.py) | 订阅者：检查是否丢消息、计算延迟 |
| [`reset_client_node.py`](pg_basics_py/pg_basics_py/reset_client_node.py) | 服务客户端：异步调用、超时处理 |
| [`countdown_server_node.py`](pg_basics_py/pg_basics_py/countdown_server_node.py) | 动作服务端：接受/拒绝目标、反馈、取消、多线程执行器 |
| [`countdown_client_node.py`](pg_basics_py/pg_basics_py/countdown_client_node.py) | 动作客户端：发送目标、接收反馈、取消 |
| [`launch/basics.launch.py`](pg_basics_py/launch/basics.launch.py) | 一键启动 + 命名空间 |
| [`config/basics.yaml`](pg_basics_py/config/basics.yaml) | greeter 的参数文件 |
| [`test/`](pg_basics_py/test/) | 代码风格检查 + 单元测试 |

建议阅读顺序：`counter.py` → `greeter_node.py` → `listener_node.py` → `reset_client_node.py`
→ `countdown_server_node.py` → `countdown_client_node.py` → `basics.launch.py`。

也可以按提交记录一步步看代码是怎么长出来的：

```bash
git log --oneline -- lessons/01_ros2_basics
git show <提交号>
```

运行测试：

```bash
colcon test --packages-select pg_basics_py && colcon test-result --verbose
```

## 实验记录

> 下面是开发时的验证记录，供参考。你自己做实验时，可以在表格后面继续追加。

| 日期 | 实验 | 参数 | 现象 | 结论 |
|---|---|---|---|---|
| 2026-11-24 | 1 话题 | 默认 1 Hz | 第一条消息延迟约 800 ms，之后约 0.5 ms | 第一条消息要等节点互相发现、建立连接 |
| 2026-11-24 | 3 参数 | 1 → 10 Hz | `topic hz` 从 1.000 变为 10.000；设 100.0 和空字符串都被拒绝 | 修改后回调重建定时器，新值立即生效 |
| 2026-11-24 | 4 动作 | 10 秒，3 秒后取消 | 状态 CANCELED，剩余 7 秒；并发目标被拒绝 | 多线程执行器保证取消能及时处理 |
| 2026-11-24 | 5 launch | `namespace:=robot1` | 两套节点互不干扰，参数文件通配匹配成功 | 相对话题名 + 命名空间 = 可多开 |
| 2026-11-24 | 6 bag | 5 Hz 录 4 秒 | 共 21 条；回放"延迟"约 8 秒；不加 delay 收到 14 条，`--delay 2.0` 收到 21 条 | stamp 是数据产生的时间；回放开头也要等发现 |

## 课后练习

1. 给 greeter 加一个参数 `max_count`：计数达到这个值后自动归零。用修改前回调保证它大于 0。
2. 写一个新节点 `stats`：订阅 `/greeting`，每 5 秒打印一次收到的消息数、丢失数、平均延迟。逻辑部分写成纯 Python 函数并加单元测试。
3. 修改动作服务端：新目标到来时不再拒绝，而是中止旧目标、执行新目标。（提示：`handle_accepted_callback`）
4. 在 launch 文件里加一个启动参数 `use_listener`（默认 true），为 false 时不启动 listener。（提示：`IfCondition`）
5. 把 listener 的 QoS 改成 `qos_profile_sensor_data`（best effort），再用 `ros2 topic info -v` 查看，并比较与 greeter 的兼容性。

## 常见问题

**问题**：`ros2 run pg_basics_py greeter` 提示 `Package 'pg_basics_py' not found`
**解决**：没编译或没 source。执行 `cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash`。

**问题**：`ImportError: cannot import name 'Greeting' from 'pg_interfaces.msg'`
**解决**：`pg_interfaces` 没编译。用 `colcon build --packages-up-to pg_basics_py` 会自动先编译依赖。

**问题**：`ros2 param set ... greeting_text ""` 报错或设置成了别的值
**解决**：空字符串在命令行要写成 `"''"`，否则会被解析成空参数。

**问题**：listener 一直收不到消息
**解决**：用 `ros2 topic info /greeting -v` 检查发布者、订阅者是否都在，话题名和类型是否一致；
如果加了命名空间，两者要在同一个命名空间下。

**问题**：在脚本里用 `&` 后台启动 launch，按 `pkill -INT` 关闭后节点仍然残留
**解决**：非交互 shell 中后台作业会忽略 SIGINT。交互终端里按 Ctrl+C 没有这个问题；
脚本里可以改用 `pkill -TERM` 直接结束节点进程。

## 参考资料

- [ROS 2 Jazzy 官方教程：理解节点、话题、服务、参数、动作](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [编写 Python 发布者和订阅者](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [编写 Python 动作服务端和客户端](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [在类中使用参数（Python）](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [launch 文件入门](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [录制和回放数据（bag）](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [执行器与回调组](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html)
