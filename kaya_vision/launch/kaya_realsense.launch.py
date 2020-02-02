import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace="/",
            output='screen',
            parameters=[get_package_share_directory('kaya_vision')+'/config/d435.yaml']
            ),
    ])
