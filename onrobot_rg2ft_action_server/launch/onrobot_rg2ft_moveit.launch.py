from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ip = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')
    gripper_name = LaunchConfiguration('gripper_name')

    rg2ft_launch = os.path.join(
        get_package_share_directory('onrobot_rg2ft_action_server'),
        'launch',
        'onrobot_rg2ft_action_server.launch.py'
    )

    moveit_launch = os.path.join(
        get_package_share_directory('onrobot_rg2ft_moveit_config'),
        'launch',
        'demo.launch.py'  # asegúrate que este exista y sea para ROS2
    )

    return LaunchDescription([
        DeclareLaunchArgument('ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument('gripper_name', default_value='onrobot_rg2ft'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rg2ft_launch),
            launch_arguments={
                'ip': ip,
                'port': port,
                'gripper_name': gripper_name
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'load_robot_description': 'true',
                'moveit_controller_manager': 'simple',
                'use_rviz': 'true'
            }.items(),
        ),
    ])

