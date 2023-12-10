#!/usr/bin/env python3

# Author: Changu Do

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    description_dir = LaunchConfiguration(
        'description_dir',
        default=os.path.join(
            get_package_share_directory('nuri_description'),
            'launch'
        )
    )
    
    # compass_dir = LaunchConfiguration(
    #     'compass_dir',
    #     default=os.path.join(
    #         get_package_share_directory('hmc5883ldriver'),
    #         'launch'
    #     )
    # )
        
    return LaunchDescription([
        Node(
            package='nuri_odom',
            executable='odom_node',
            name='diff_drive_controller',
            output='screen',
            emulate_tty=True,
            parameters=None,
            namespace='',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/nuri_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([compass_dir, '/hmc5883ldriver_launch.py']),
        # )
    ])