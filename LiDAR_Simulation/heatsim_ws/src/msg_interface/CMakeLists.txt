cmake_minimum_required(VERSION 3.8)
project(msg_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GPSAndIMU.msg"
  "msg/Gps.msg"
)

ament_package()