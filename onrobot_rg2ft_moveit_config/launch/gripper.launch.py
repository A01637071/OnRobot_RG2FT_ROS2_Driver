import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ip = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')
    gripper_name = LaunchConfiguration('gripper_name')

    return LaunchDescription([
        DeclareLaunchArgument('ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument('gripper_name', default_value='onrobot_rg2ft'),

        Node(
            package='onrobot_rg2ft_control',
            executable='OnRobotRG2FTDriver.py',
            name='onrobot_rg2ft_driver',
            namespace=gripper_name,
            output='screen',
            parameters=[{
                'ip': ip,
                'port': port
            }]
        ),

        Node(
            package='onrobot_rg2ft_action_server',
            executable='onrobot_rg2ft_action_server',
            name='onrobot_rg2ft_action_server',
            output='screen',
            parameters=[{
                'min_angle': 0.0,
                'max_angle': 1.18,
                'min_effort': 3.0,
                'max_effort': 40.0,
                'default_effort': 20.0,
                'action_server_name': 'gripper_controller/gripper_cmd',
                'control_topic': f'{gripper_name}/command',
                'state_topic': f'{gripper_name}/state',
                'joint_states_topic': 'joint_states',
                'joint_name': 'finger_joint'
            }]
        )
    ])

