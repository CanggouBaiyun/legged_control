cmake_minimum_required(VERSION 3.16)
project(go2_sdk2_readonly LANGUAGES CXX)

find_package(unitree_sdk2 REQUIRED CONFIG)
add_executable(go2_read_lowstate read_lowstate.cpp)
target_compile_features(go2_read_lowstate PRIVATE cxx_std_17)
target_link_libraries(go2_read_lowstate PRIVATE unitree_sdk2)
target_compile_options(go2_read_lowstate PRIVATE -Wall -Wextra -Wpedantic)
