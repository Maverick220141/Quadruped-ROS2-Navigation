cmake_minimum_required(VERSION 3.8)
project(my_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(my_nav src/my_nav.cpp)
target_include_directories(my_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_nav PUBLIC c_std_99 cxx_std_17)

# === BT plugin: RotateToGoalHeading ===
# StatefulActionNode：自闭环 P 控制器，把 robot yaw 旋转到 goal yaw
# 解决 nav2 humble Spin 节点的 spin_dist 在构造时读 blackboard 为 0 的 bug
add_library(rotate_to_goal_heading_bt_node SHARED
  src/rotate_to_goal_heading_node.cpp
)
target_include_directories(rotate_to_goal_heading_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rotate_to_goal_heading_bt_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(rotate_to_goal_heading_bt_node
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_compile_definitions(rotate_to_goal_heading_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(virtual_obstacle_layer SHARED
  src/virtual_obstacle_layer.cpp
)
target_include_directories(virtual_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(virtual_obstacle_layer PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(virtual_obstacle_layer
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d my_nav_costmap_plugins.xml)

add_executable(blocked_path_monitor_node
  src/blocked_path_monitor_node.cpp
)
target_include_directories(blocked_path_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(blocked_path_monitor_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(blocked_path_monitor_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(scan_virtual_obstacle_node
  src/scan_virtual_obstacle_node.cpp
)
target_include_directories(scan_virtual_obstacle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(scan_virtual_obstacle_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(scan_virtual_obstacle_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS my_nav
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS rotate_to_goal_heading_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS virtual_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS blocked_path_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS scan_virtual_obstacle_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch maps behavior_trees docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(rotate_to_goal_heading_bt_node)
ament_export_libraries(virtual_obstacle_layer)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
