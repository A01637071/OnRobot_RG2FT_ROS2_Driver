cmake_minimum_required(VERSION 3.5)
project(onrobot_rg2ft_moveit_config)

# ament package
find_package(ament_cmake REQUIRED)

# Instalación de directorios launch y config
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
        PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/)

# Configuración final de ament
ament_package()

