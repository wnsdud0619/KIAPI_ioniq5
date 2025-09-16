cmake_minimum_required(VERSION 3.5)
project(kiapi_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED components common)
find_package(pcl_conversions REQUIRED)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# # Include Cpp "include" directory
# include_directories(include)

# # Create Cpp executable
# add_executable(cpp_executable src/node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# # Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )
include_directories(
  include
)

# pcl_byapss_component library
add_library(pcl_bypass_component SHARED
  src/pointcloud_bypass.cpp
)

target_compile_definitions(pcl_bypass_component 
PRIVATE "KIAPI_SENSOR_KIT_LAUNCH_BUILDING_DLL")


ament_target_dependencies(pcl_bypass_component 
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

rclcpp_components_register_nodes(pcl_bypass_component 
  "kiapi_sensor::PCLBypassNode")

  install(TARGETS
  pcl_bypass_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# pcl_sync_component library
add_library(pcl_sync_component SHARED
  src/pointcloud_sync.cpp
)

target_compile_definitions(pcl_sync_component 
PRIVATE "KIAPI_SENSOR_KIT_LAUNCH_BUILDING_DLL")


ament_target_dependencies(pcl_sync_component 
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
)

rclcpp_components_register_nodes(pcl_sync_component 
  "kiapi_sensor::PCLSyncNode")

  install(TARGETS
  pcl_sync_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(node_plugins "$(node_plugins)kiapi_sensor::PCLSyncNode;$<TARGET_FILE:pcl_sync_component>\n")
set(node_plugins "$(node_plugins)kiapi_sensor::PCLBypassNode;$<TARGET_FILE:pcl_bypass_component>\n")


add_executable(gnss_to_link 
  src/gnss_to_link.cpp)


ament_target_dependencies(gnss_to_link
  sensor_msgs  
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
install(TARGETS 
  gnss_to_link 
  DESTINATION lib/${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/vehicle_info_from_sensor.py
  scripts/gnss_to_link.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
  config
)



