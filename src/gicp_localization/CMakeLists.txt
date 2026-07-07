cmake_minimum_required(VERSION 3.8)
project(gicp_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# 强制开启 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

# 寻找系统依赖库
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# 寻找刚才 sudo make install 装好的 small_gicp
find_package(small_gicp REQUIRED)

# 包含头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(gicp_localization_node src/gicp_localization_node.cpp)

# 链接 OpenMP 加速
if(OpenMP_CXX_FOUND)
    target_link_libraries(gicp_localization_node OpenMP::OpenMP_CXX)
endif()

# 链接 PCL 与 small_gicp
target_link_libraries(gicp_localization_node 
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

# 链接 ROS 2 依赖
ament_target_dependencies(gicp_localization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

install(TARGETS gicp_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
