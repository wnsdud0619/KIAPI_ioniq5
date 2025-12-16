cmake_minimum_required(VERSION 3.5)
project(vehicle_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)             # 추가
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Header.msg
  msg/Velocity.msg
  msg/Position.msg
  msg/Orientation.msg
  msg/Acceleration.msg
  msg/RotationRate.msg
  msg/Motion.msg
  msg/Steer.msg
  msg/Pedal.msg
  msg/InnerParts.msg
  msg/VehicleStatus.msg
  msg/Size.msg
  msg/LinearVel.msg
  msg/Object.msg
  msg/ObjectArray.msg
  msg/VehicleInfo.msg
  msg/DeviceD2XVehicleInfoTransmit.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()


