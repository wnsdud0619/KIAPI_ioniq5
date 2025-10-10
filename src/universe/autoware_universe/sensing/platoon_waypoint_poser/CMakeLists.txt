cmake_minimum_required(VERSION 3.8)
project(platoon_waypoint_poser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 표준 ament_cmake 방식 (component 미사용)
find_package(ament_cmake REQUIRED)

# ROS 의존성
find_package(rclcpp REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_geography_utils REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(kiapi_waypoints_msgs REQUIRED)

# (선택) GeographicLib — 환경에 따라 필요할 수 있음
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

# 포함 디렉토리 (헤더가 있다면 활성화)
# include_directories(include)

# 실행 파일 생성 (component 아님)
add_executable(platoon_waypoint_poser
  src/platoon_waypoint_poser.cpp
)

ament_target_dependencies(platoon_waypoint_poser
  rclcpp
  autoware_map_msgs
  autoware_geography_utils
  visualization_msgs
  geometry_msgs
  geographic_msgs
  kiapi_waypoints_msgs
)

# (선택) GeographicLib 연결
if(GeographicLib_LIBRARIES)
  target_link_libraries(platoon_waypoint_poser
    ${GeographicLib_LIBRARIES}
  )
endif()

# 설치
install(TARGETS platoon_waypoint_poser
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 런치/설정 파일 설치 (있을 때만)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
