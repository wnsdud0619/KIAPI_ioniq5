cmake_minimum_required(VERSION 3.8)
project(kiapi_waypoints_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # ✅ 추가
find_package(builtin_interfaces REQUIRED)  # ✅ 명시적으로 추가

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KiapiWaypoints.msg"
  DEPENDENCIES builtin_interfaces std_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

