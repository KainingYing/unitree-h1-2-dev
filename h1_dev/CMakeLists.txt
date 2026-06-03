cmake_minimum_required(VERSION 3.10)
project(h1_dev)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SDK 装在非默认路径 /opt/unitree_robotics，需加入搜索路径
list(APPEND CMAKE_PREFIX_PATH /opt/unitree_robotics)
find_package(unitree_sdk2 REQUIRED)

add_executable(read_state read_state.cpp)
target_link_libraries(read_state unitree_sdk2)

add_executable(probe_arm_sdk probe_arm_sdk.cpp)
target_link_libraries(probe_arm_sdk unitree_sdk2)
