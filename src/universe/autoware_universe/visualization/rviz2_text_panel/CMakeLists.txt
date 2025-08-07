cmake_minimum_required(VERSION 3.8)
project(rviz2_text_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(novatel_oem7_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create shared library
add_library(rviz2_text_panel SHARED
  src/text_panel.cpp
)

target_include_directories(rviz2_text_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(rviz2_text_panel
  rclcpp
  rviz_common
  pluginlib
  rviz_rendering
  rviz_ogre_vendor
  Qt5Widgets
  novatel_oem7_msgs
)

target_link_libraries(rviz2_text_panel
  Qt5::Widgets
)

# Export plugin description to pluginlib
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install library
install(TARGETS
  rviz2_text_panel
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Testing support (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

