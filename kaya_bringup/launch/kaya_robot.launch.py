import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kaya_base_config = os.path.join(get_package_share_directory('kaya_base'), 'config', 'kaya_base.yaml')
    kaya_urdf = os.path.join(get_package_share_directory('kaya_description'), 'urdf', 'kaya.urdf')
    kaya_state_config = os.path.join(get_package_share_directory('kaya_state'), 'config', 'kaya_state.yaml')
    kaya_teleop_config = os.path.join(get_package_share_directory('kaya_teleop'), 'config', 'kaya_teleop.yaml')
    kaya_vision_config = os.path.join(get_package_share_directory('kaya_vision'), 'config', 'kaya_vision.yaml')
    return LaunchDescription([
        Node(package='joy', node_executable='joy_node', output='screen'),
        Node(package='kaya_state', node_executable='kaya_state_node', output='screen', parameters=[kaya_state_config, {'urdf': kaya_urdf}]),
        Node(package='kaya_vision', node_executable='kaya_vision_node', output='screen', parameters=[kaya_vision_config]),
        Node(package='kaya_base', node_executable='kaya_base_node', output='screen', parameters=[kaya_base_config]),
        Node(package='kaya_teleop', node_executable='kaya_teleop_node', output='screen', parameters=[kaya_teleop_config]),
    ])
