from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_directory = os.path.join(
        get_package_share_directory('nuri_localization'),
        'config'
    )
    config_file = os.path.join(config_directory, 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
