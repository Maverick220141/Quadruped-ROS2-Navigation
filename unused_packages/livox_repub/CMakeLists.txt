cmake_minimum_required(VERSION 3.8)
project(livox_repub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED) # 必须确保你已经安装或编译过 livox 驱动

add_executable(livox_repub_node src/livox_repub_node.cpp)
ament_target_dependencies(livox_repub_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS
  livox_repub_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()