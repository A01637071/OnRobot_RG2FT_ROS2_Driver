cmake_minimum_required(VERSION 3.5)
project(onrobot_rg2ft_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(onrobot_rg2ft_msgs REQUIRED)

# Instalar el paquete Python
ament_python_install_package(${PROJECT_NAME})

# Instalar los nodos Python directamente como ejecutables
install(
  PROGRAMS
    onrobot_rg2ft_control/OnRobotRG2FTDriver.py
    onrobot_rg2ft_control/control_gripper_terminal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

