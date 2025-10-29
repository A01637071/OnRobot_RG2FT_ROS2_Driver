import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    reset = LaunchConfiguration('reset')
    warehouse_db_path = LaunchConfiguration('moveit_warehouse_database_path')

    return LaunchDescription([
        DeclareLaunchArgument('reset', default_value='false'),
        DeclareLaunchArgument('moveit_warehouse_database_path',
                              default_value=os.path.join(
                                  get_package_share_directory('onrobot_rg2ft_moveit_config'),
                                  'default_warehouse_mongo_db')),

        Node(
            package='moveit_ros_warehouse',
            executable='moveit_init_demo_warehouse',
            name='moveit_default_db_reset',
            output='screen',
            respawn=False,
            condition=IfCondition(reset),
            parameters=[{'moveit_warehouse_database_path': warehouse_db_path}]
        )
    ])

