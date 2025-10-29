cmake_minimum_required(VERSION 3.8)
project(onrobot_rg2ft_action_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(onrobot_rg2ft_msgs REQUIRED)

include_directories(include)

add_executable(onrobot_rg2ft_action_server_node
  src/onrobot_rg2ft_action_server_node.cpp
  src/onrobot_rg2ft_action_server.cpp
)

ament_target_dependencies(onrobot_rg2ft_action_server_node
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  onrobot_rg2ft_msgs
)

install(TARGETS
  onrobot_rg2ft_action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
