cmake_minimum_required(VERSION 3.8)
project(agibot_motion_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MotionControl.srv"
  "srv/GetState.srv"
)

set(THIRD_PARTY_AGIBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zsibot_sdk-0.2.6")
set(AGIBOT_SDK_ROOT "" CACHE PATH "Optional external AgiBot/ZsiBot SDK root")

set(POSSIBLE_SDK_PATHS)
if(AGIBOT_SDK_ROOT)
  list(APPEND POSSIBLE_SDK_PATHS "${AGIBOT_SDK_ROOT}")
endif()
list(APPEND POSSIBLE_SDK_PATHS
  "${THIRD_PARTY_AGIBOT_SDK_ROOT}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../AgibotSDK"
)

set(AGIBOT_SDK_FOUND_ROOT "")
foreach(SDK_PATH ${POSSIBLE_SDK_PATHS})
  if(EXISTS "${SDK_PATH}/include")
    set(AGIBOT_SDK_FOUND_ROOT "${SDK_PATH}")
    break()
  endif()
endforeach()

if(AGIBOT_SDK_FOUND_ROOT STREQUAL "")
  message(FATAL_ERROR "AgiBot SDK not found. Keep third_party/zsibot_sdk-0.2.6 available or pass -DAGIBOT_SDK_ROOT=/path/to/zsibot_sdk.")
endif()

set(AGIBOT_SDK_INCLUDE_DIRS "${AGIBOT_SDK_FOUND_ROOT}/include")

if(WIN32)
  set(SDK_FULL_LIB_PATH "${AGIBOT_SDK_FOUND_ROOT}/lib/zsl-1/win64/libmc_sdk_zsl_1_win64.dll")
else()
  set(SDK_LIB_DIR "${AGIBOT_SDK_FOUND_ROOT}/lib/zsl-1/aarch64")
  set(SDK_FULL_LIB_PATH "${SDK_LIB_DIR}/libmc_sdk_zsl_1_aarch64.so")
  link_directories(${SDK_LIB_DIR})
endif()

add_executable(motion_server_node
  src/motion_server_node.cpp
)

ament_target_dependencies(motion_server_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

target_include_directories(motion_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${AGIBOT_SDK_INCLUDE_DIRS}
)

if(EXISTS ${SDK_FULL_LIB_PATH})
  target_link_libraries(motion_server_node
    "${SDK_FULL_LIB_PATH}"
    -Wl,--no-as-needed
  )
else()
  target_link_libraries(motion_server_node
    -lmc_sdk_zsl_1_aarch64
    -Wl,--no-as-needed
  )
  message(WARNING "AgiBot SDK library not found at ${SDK_FULL_LIB_PATH}; falling back to linker search path.")
endif()

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(motion_server_node
  ${cpp_typesupport_target}
)

install(TARGETS
  motion_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
