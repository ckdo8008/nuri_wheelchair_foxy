from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nuri_odom',
            executable='odom_node',
            name='diff_drive_controller',
            output='screen',
            emulate_tty=True,
        )
    ])