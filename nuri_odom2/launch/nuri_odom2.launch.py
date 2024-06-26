import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    description_dir = LaunchConfiguration(
        'description_dir',
        default=os.path.join(
            get_package_share_directory('nuri_description'),
            'launch'
        )
    )    
    return LaunchDescription([
        Node(
            package='nuri_odom2',
            executable='odometry',
            name='diff_drive_controller',
            output='screen',
            emulate_tty=True,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/nuri_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])