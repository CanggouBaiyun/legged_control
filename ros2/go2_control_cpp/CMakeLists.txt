cmake_minimum_required(VERSION 3.16)
project(go2_control_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
message(STATUS "Pinocchio version: ${pinocchio_VERSION}")
message(STATUS "Pinocchio config: ${pinocchio_DIR}")

add_library(go2_model STATIC src/model.cpp src/dynamics.cpp)
target_compile_features(go2_model PUBLIC cxx_std_17)
target_include_directories(go2_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(go2_model PUBLIC pinocchio::pinocchio)
add_library(go2_estimation STATIC src/state_estimation.cpp src/estimator_preprocessor.cpp)
target_link_libraries(go2_estimation PUBLIC go2_model)

set(GO2_OSQP_ROOT "" CACHE PATH "Prefix containing matching OSQP 1.x C headers and library")
if(GO2_OSQP_ROOT)
  set(GO2_OSQP_INCLUDE_DIR "${GO2_OSQP_ROOT}/include/osqp")
  set(GO2_OSQP_LIBRARY "${GO2_OSQP_ROOT}/lib/libosqp.so")
  if(NOT EXISTS "${GO2_OSQP_INCLUDE_DIR}/osqp.h" OR NOT EXISTS "${GO2_OSQP_LIBRARY}")
    message(FATAL_ERROR "GO2_OSQP_ROOT must contain matching OSQP 1.x C headers and lib/libosqp.so")
  endif()
  message(STATUS "OSQP C headers: ${GO2_OSQP_INCLUDE_DIR}")
  message(STATUS "OSQP C runtime: ${GO2_OSQP_LIBRARY}")
  add_library(go2_wbc STATIC src/wbc.cpp src/osqp_solver.cpp)
  target_include_directories(go2_wbc PRIVATE "${GO2_OSQP_INCLUDE_DIR}")
  target_link_libraries(go2_wbc PUBLIC go2_model PRIVATE ${CMAKE_DL_LIBS})
endif()

set(GO2_MUJOCO_ROOT "" CACHE PATH "MuJoCo package directory containing include/ and libmujoco.so.*")
if(GO2_MUJOCO_ROOT)
  if(NOT TARGET go2_wbc)
    message(FATAL_ERROR "Native control requires GO2_OSQP_ROOT as well as GO2_MUJOCO_ROOT")
  endif()
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(ocs2_ros_interfaces REQUIRED)
  find_package(ocs2_msgs REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
  find_package(OpenMP REQUIRED COMPONENTS CXX)
  find_package(Threads REQUIRED)
  find_package(glfw3 REQUIRED)
  find_package(OpenGL REQUIRED)
  find_library(GO2_MUJOCO_LIBRARY NAMES mujoco libmujoco.so.3.4.0
    PATHS "${GO2_MUJOCO_ROOT}" NO_DEFAULT_PATH REQUIRED)
  # Official Simulate 3.4.0 and its pinned PNG dependency, vendored unchanged.
  get_filename_component(GO2_CONTROL_SOURCE_REAL "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
  set(GO2_SIMULATE_SOURCE "${GO2_CONTROL_SOURCE_REAL}/../../third_party/mujoco_simulate")
  add_library(go2_mujoco_simulate STATIC
    "${GO2_SIMULATE_SOURCE}/simulate/simulate.cc"
    "${GO2_SIMULATE_SOURCE}/simulate/glfw_adapter.cc"
    "${GO2_SIMULATE_SOURCE}/simulate/glfw_dispatch.cc"
    "${GO2_SIMULATE_SOURCE}/simulate/platform_ui_adapter.cc"
    "${GO2_SIMULATE_SOURCE}/lodepng/lodepng.cpp")
  target_compile_features(go2_mujoco_simulate PUBLIC cxx_std_17)
  target_include_directories(go2_mujoco_simulate PUBLIC
    "${GO2_SIMULATE_SOURCE}/simulate" "${GO2_MUJOCO_ROOT}/include"
    PRIVATE "${GO2_SIMULATE_SOURCE}/lodepng")
  target_link_libraries(go2_mujoco_simulate PUBLIC "${GO2_MUJOCO_LIBRARY}" glfw OpenGL::GL Threads::Threads)
  add_library(go2_reference STATIC src/reference.cpp)
  target_link_libraries(go2_reference PUBLIC go2_wbc)
  add_executable(online_command src/online_command.cpp)
  target_compile_features(online_command PRIVATE cxx_std_17)
  ament_target_dependencies(
    online_command
    rclcpp geometry_msgs ocs2_ros_interfaces ocs2_msgs
  )
  install(TARGETS online_command DESTINATION lib/${PROJECT_NAME})
  add_executable(control_simulation src/control_simulation.cpp src/local_mrt.cpp src/simulation.cpp)
  target_include_directories(control_simulation PRIVATE "${GO2_MUJOCO_ROOT}/include")
  ament_target_dependencies(control_simulation rclcpp ocs2_ros_interfaces ocs2_msgs)
  target_link_libraries(
  control_simulation
  go2_reference
  go2_estimation
  go2_mujoco_simulate
  )
  target_compile_definitions(control_simulation PRIVATE GO2_OSQP_LIBRARY="${GO2_OSQP_LIBRARY}")
  set_target_properties(control_simulation PROPERTIES INSTALL_RPATH "${GO2_MUJOCO_ROOT}")
  install(TARGETS control_simulation DESTINATION lib/${PROJECT_NAME})
  install(TARGETS go2_reference ARCHIVE DESTINATION lib)
  install(DIRECTORY "${GO2_SIMULATE_SOURCE}/" DESTINATION share/${PROJECT_NAME}/third_party/mujoco_simulate
    FILES_MATCHING PATTERN LICENSE PATTERN UPSTREAM.md PATTERN SHA256SUMS PATTERN LICENSE.lodepng)
endif()

add_executable(model_check src/model_check.cpp)
target_compile_features(model_check PRIVATE cxx_std_17)
target_link_libraries(model_check PRIVATE go2_model)
add_executable(dynamics_check src/dynamics_check.cpp)
target_link_libraries(dynamics_check PRIVATE go2_model)
add_executable(estimator_check src/estimator_check.cpp)
target_link_libraries(estimator_check PRIVATE go2_estimation)
install(TARGETS estimator_check DESTINATION lib/${PROJECT_NAME})
if(TARGET go2_wbc)
  add_executable(wbc_replay src/wbc_replay.cpp)
  target_link_libraries(wbc_replay PRIVATE go2_wbc)
  target_compile_definitions(wbc_replay PRIVATE GO2_OSQP_LIBRARY="${GO2_OSQP_LIBRARY}")
  install(TARGETS wbc_replay DESTINATION lib/${PROJECT_NAME})
  install(TARGETS go2_wbc ARCHIVE DESTINATION lib)
endif()

install(TARGETS model_check dynamics_check DESTINATION lib/${PROJECT_NAME})
install(TARGETS go2_model ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS go2_estimation ARCHIVE DESTINATION lib)
ament_package()
