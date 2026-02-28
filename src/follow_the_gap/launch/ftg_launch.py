import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('follow_the_gap')
    config = os.path.join(pkg_dir, 'config', 'ftg_config.yaml')

    ftg_node = Node(
        package='follow_the_gap',
        executable='follow_the_gap_node',
        name='follow_the_gap_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([ftg_node])
