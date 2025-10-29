import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ip = LaunchConfiguration('ip', default='192.168.1.1')
    port = LaunchConfiguration('port', default='502')
    gripper_name = LaunchConfiguration('gripper_name', default='onrobot_rg2ft')

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('onrobot_rg2ft_moveit_config'),
            'launch', 'gripper.launch.py')),
        launch_arguments={'ip': ip, 'port': port, 'gripper_name': gripper_name}.items()
    )

    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('onrobot_rg2ft_moveit_config'),
            'launch', 'warehouse.launch.py')),
        launch_arguments={}.items()
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('onrobot_rg2ft_moveit_config'),
            'launch', 'demo.launch.py')),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        gripper_launch,
        warehouse_launch,
        demo_launch
    ])

