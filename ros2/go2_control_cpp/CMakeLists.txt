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

add_executable(model_check src/model_check.cpp)
target_compile_features(model_check PRIVATE cxx_std_17)
target_link_libraries(model_check PRIVATE go2_model)
add_executable(dynamics_check src/dynamics_check.cpp)
target_link_libraries(dynamics_check PRIVATE go2_model)

install(TARGETS model_check dynamics_check DESTINATION lib/${PROJECT_NAME})
install(TARGETS go2_model ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
