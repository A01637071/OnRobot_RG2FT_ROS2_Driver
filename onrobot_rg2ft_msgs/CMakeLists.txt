cmake_minimum_required(VERSION 3.5)
project(onrobot_rg2ft_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OnRobotRG2FTData.msg"
  "msg/RG2FTCommand.msg"
  "srv/SetProximityOffsets.srv"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

