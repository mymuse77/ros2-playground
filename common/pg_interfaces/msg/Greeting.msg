# 01 课：问候消息
#
# 演示自定义消息：一条消息里可以组合基本类型（string、uint32）
# 和其他包里定义的类型（builtin_interfaces/Time）。

# 消息生成的时间（发布者填写）
builtin_interfaces/Time stamp

# 问候内容，例如 "Hello ROS 2"
string text

# 发布序号，从计数器起始值开始递增；调用 reset_counter 服务可以重置
uint32 count
