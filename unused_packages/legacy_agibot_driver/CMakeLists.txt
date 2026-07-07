cmake_minimum_required(VERSION 3.8)
project(legacy_agibot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(ARCH "unknown")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ARCH "x86_64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCH "aarch64")
endif()

if(ARCH STREQUAL "unknown")
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

set(AGIBOT_SDK_ROOT "" CACHE PATH "Optional external AgiBot/ZsiBot SDK root")
set(POSSIBLE_SDK_PATHS)
if(AGIBOT_SDK_ROOT)
  list(APPEND POSSIBLE_SDK_PATHS "${AGIBOT_SDK_ROOT}")
endif()
list(APPEND POSSIBLE_SDK_PATHS
  "${CMAKE_CURRENT_SOURCE_DIR}/../../src/agibot_motion_service/third_party/zsibot_sdk-0.2.6"
  "${CMAKE_CURRENT_SOURCE_DIR}/zsibot_sdk-0.2.6"
)

set(AGIBOT_SDK_FOUND_ROOT "")
foreach(SDK_PATH ${POSSIBLE_SDK_PATHS})
  if(EXISTS "${SDK_PATH}/include")
    set(AGIBOT_SDK_FOUND_ROOT "${SDK_PATH}")
    break()
  endif()
endforeach()

if(AGIBOT_SDK_FOUND_ROOT STREQUAL "")
  message(FATAL_ERROR "AgiBot SDK not found. Keep agibot_motion_service/third_party/zsibot_sdk-0.2.6 available or pass -DAGIBOT_SDK_ROOT=/path/to/zsibot_sdk.")
endif()

set(AGIBOT_SDK_INCLUDE_DIRS "${AGIBOT_SDK_FOUND_ROOT}/include")
set(SDK_LIB_DIR "${AGIBOT_SDK_FOUND_ROOT}/lib/zsl-1/${ARCH}")
find_library(AGIBOT_SDK_LIB
  NAMES "mc_sdk_zsl_1_${ARCH}"
  PATHS "${SDK_LIB_DIR}"
  NO_DEFAULT_PATH
  REQUIRED
)

add_executable(agibot_bridge_node src/agibot_bridge_node.cpp)
ament_target_dependencies(agibot_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)
target_include_directories(agibot_bridge_node PRIVATE "${AGIBOT_SDK_INCLUDE_DIRS}")
target_link_libraries(agibot_bridge_node "${AGIBOT_SDK_LIB}" pthread)

add_executable(footprint_publisher_node src/footprint_publisher_node.cpp)
ament_target_dependencies(footprint_publisher_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  agibot_bridge_node
  footprint_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
