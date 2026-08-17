cmake_minimum_required(VERSION 3.8)
project(fastlio2_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fexceptions")

# ---- FAST-LIO 核心（不修改，直接引用其源码与头文件）----
set(FASTLIO_CORE "${CMAKE_CURRENT_SOURCE_DIR}/../../fast_lio_core")
if(NOT EXISTS "${FASTLIO_CORE}/src/fast_lio.hpp")
  message(FATAL_ERROR "fast_lio_core not found at ${FASTLIO_CORE} — 先 clone/编译 T3.1")
endif()

# 与核心 CMake 一致的多线程宏（16 核 -> MP_PROC_NUM=3）
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=3)

# ---- ROS2 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_interfaces REQUIRED)

# ---- 系统库（与核心一致）----
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${FASTLIO_CORE}/include
  ${FASTLIO_CORE}/src
)

add_executable(ros2_localizer
  src/ros2_localizer.cpp
  ${FASTLIO_CORE}/include/ikd-Tree/ikd_Tree.cpp
  ${FASTLIO_CORE}/src/preprocess.cpp
)

ament_target_dependencies(ros2_localizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  livox_interfaces
)

target_link_libraries(ros2_localizer
  ${PCL_LIBRARIES}
  yaml-cpp
  nlohmann_json::nlohmann_json
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(ros2_localizer OpenMP::OpenMP_CXX)
endif()

install(TARGETS ros2_localizer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
