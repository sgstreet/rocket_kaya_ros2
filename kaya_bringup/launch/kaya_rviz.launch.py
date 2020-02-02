import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    rviz = os.path.join(get_package_share_directory('kaya_description'), 'rviz', 'kaya.rviz')
    return LaunchDescription([
        Node(package='rviz2', node_executable='rviz2', output='screen', arguments=['-d', rviz]),
    ])
