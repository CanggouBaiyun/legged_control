cmake_minimum_required(VERSION 3.16)
project(go2_mpc_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#OCS2导出的链接接口依赖Boost::system
find_package(Boost REQUIRED COMPONENTS 
    system
    filesystem
    log_setup
    log)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(Threads REQUIRED)

find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EvaluatePolicy.srv"
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(
  mrt_interface_check
  src/mrt_interface_check.cpp
)

ament_target_dependencies(
    mrt_interface_check
    rclcpp
    ocs2_ros_interfaces
    ocs2_msgs
)

# 原来的程序：比较 MRT 求值和手动插值。
add_executable(
  mrt_policy_check
  src/mrt_policy_check.cpp
)

ament_target_dependencies(
  mrt_policy_check
  rclcpp
  ocs2_ros_interfaces
  ocs2_msgs
)

# 新程序：提供策略查询服务。
add_executable(
  mrt_policy_server
  src/mrt_policy_server.cpp
)

ament_target_dependencies(
  mrt_policy_server
  rclcpp
  ocs2_ros_interfaces
  ocs2_msgs
)

#链接当前包生成的EvaluatePolivy服务接口
rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
)

target_link_libraries(
    mrt_policy_server
    "${cpp_typesupport_target}"
)

# 安装后， ros2 run才能找到这写程序
install(
  TARGETS mrt_interface_check mrt_policy_check mrt_policy_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()