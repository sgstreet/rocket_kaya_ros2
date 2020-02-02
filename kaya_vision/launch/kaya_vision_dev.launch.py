import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    kaya_vision_config = os.path.join(get_package_share_directory('kaya_vision'), 'config', 'kaya_vision_dev.yaml')
    return LaunchDescription([
        Node(
            package='kaya_vision',
            node_executable='kaya_vision_node',
            output='screen',
            parameters=[kaya_vision_config]
            ),
    ])
