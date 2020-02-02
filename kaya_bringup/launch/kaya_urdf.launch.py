import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    kaya_urdf = os.path.join(get_package_share_directory('kaya_description'), 'urdf', 'kaya.urdf')
    kaya_state_config = os.path.join(get_package_share_directory('kaya_state'), 'config', 'kaya_state.yaml')
    rviz_config = os.path.join(get_package_share_directory('kaya_description'), 'rviz', 'kaya.rviz')
    return LaunchDescription([
        Node(package='kaya_state', node_executable='kaya_joint_state_node', output='screen', parameters=[{'urdf': kaya_urdf}]),
        Node(package='kaya_state', node_executable='kaya_state_node', output='screen', parameters=[kaya_state_config, {'urdf': kaya_urdf}]),
        Node(package='rviz2', node_executable='rviz2', output='screen', arguments=['-d', rviz_config]),
    ])
