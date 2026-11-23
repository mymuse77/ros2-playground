cmake_minimum_required(VERSION 3.8)
project(pg_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 为下列接口定义生成 Python / C++ 等语言的代码。
# 新增接口时，把文件加到这个列表里即可。
rosidl_generate_interfaces(${PROJECT_NAME}
  # 01 课：ROS 2 基础
  "msg/Greeting.msg"
  "srv/ResetCounter.srv"
  "action/Countdown.action"
  # 03 课：到点导航
  "action/NavigateToPoint.action"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  # 只做 CMake 和 XML 格式检查（本包没有 C++ / Python 源码）
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
