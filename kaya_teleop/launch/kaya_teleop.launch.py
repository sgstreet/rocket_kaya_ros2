import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    kaya_teleop_config = os.path.join(get_package_share_directory('kaya_teleop'), 'config', 'kaya_teleop.yaml')
    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='kaya_teleop', node_executable='kaya_teleop_node', output='screen', parameters=[kaya_teleop_config]),
    ])
