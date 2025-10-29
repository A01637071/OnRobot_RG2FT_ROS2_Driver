import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    load_robot_description = LaunchConfiguration('load_robot_description', default='true')
    moveit_controller_manager = LaunchConfiguration('moveit_controller_manager', default='simple')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_robot_description', default_value='true'),
        DeclareLaunchArgument('moveit_controller_manager', default_value='simple'),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{
                'load_robot_description': load_robot_description,
                'moveit_controller_manager': moveit_controller_manager
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(
                get_package_share_directory('onrobot_rg2ft_moveit_config'),
                'config', 'moveit.rviz')]
        )
    ])

