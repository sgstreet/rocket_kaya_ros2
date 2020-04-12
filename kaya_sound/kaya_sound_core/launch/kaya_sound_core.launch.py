import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='kaya_text_to_speech', node_executable='kaya_text_to_speech', output='screen'),
        Node(package='kaya_sound_core', node_executable='kaya_sound_core_node', output='screen'),
    ])
