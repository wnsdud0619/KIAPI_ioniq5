cmake_minimum_required(VERSION 3.8)
project(qt_ros2_interface)

# Enable compiler warnings for better debugging
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(my_custom_msgs REQUIRED)

# Enable automatic processing of Qt-related files
set(CMAKE_AUTOMOC ON)   # Automatically process moc files
set(CMAKE_AUTORCC ON)   # Automatically process qrc files (if needed)
set(CMAKE_AUTOUIC ON)   # Automatically process ui files (if needed)

# Add your executable
add_executable(qt_ros2_node src/qt_ros2_node.cpp)

# Specify ROS 2 dependencies
ament_target_dependencies(qt_ros2_node rclcpp std_msgs my_custom_msgs)

# Link Qt libraries
target_link_libraries(qt_ros2_node Qt5::Widgets)

# Include directories for my_custom_msgs
target_include_directories(qt_ros2_node
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${my_custom_msgs_INCLUDE_DIRS}  # my_custom_msgs 헤더 경로 추가
)

# Install executables
install(TARGETS qt_ros2_node
  DESTINATION lib/${PROJECT_NAME})

# Enable testing if necessary
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
