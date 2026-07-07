cmake_minimum_required(VERSION 3.8)
project(yesense_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tid.msg"
  "msg/ThreeAxis.msg"  
  "msg/SensorTemp.msg"
  "msg/Pressure.msg"
  "msg/EulerAngle.msg"  
  "msg/Quat.msg"  
  "msg/GnssPos.msg"  
  "msg/NavStatus.msg"  
  "msg/Vel.msg"  
  "msg/Utc.msg" 
  "msg/SampleTimestamp.msg"
  "msg/ImuData.msg"
  "msg/ImuDataTenAxis.msg"
  "msg/EulerOnly.msg"
  "msg/RobotLord.msg"  
  "msg/AttitudeMinVru.msg"    
  "msg/AttitudeMinAhrs.msg"
  "msg/AttitudeAllData.msg"  
  "msg/PosOnly.msg"    
  "msg/NavMin.msg"    
  "msg/NavMinUtc.msg"      
  "msg/NavAll.msg"                  
)

#ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
