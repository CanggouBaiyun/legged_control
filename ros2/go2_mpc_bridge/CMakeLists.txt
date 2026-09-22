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

add_executable(
  mrt_interface_check
  src/mrt_interface_check.cpp
)

ament_target_dependencies(
    mrt_interface_check
    rclcpp
    ocs2_ros_interfaces
)

install(
  TARGETS mrt_interface_check
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()