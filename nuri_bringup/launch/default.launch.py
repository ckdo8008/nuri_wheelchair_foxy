#!/usr/bin/env python3

# Author: Changu Do

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    imu_parameter = LaunchConfiguration(
        'imu_parameter',
        default=os.path.join(
            get_package_share_directory('nuri_bringup'),
            'param/mpu6050.yaml'
        )
    )
    lidar_parameter = LaunchConfiguration(
        'lidar_parameter',
        default=os.path.join(
            get_package_share_directory('nuri_bringup'),
            'param/lsx10.yaml'
        )
    )
    imu_dir = LaunchConfiguration(
        'imu_dir',
        default=os.path.join(
            get_package_share_directory('mpu6050driver'),
            'launch'
        )
    )
    nurirobot_dir = LaunchConfiguration(
        'nurirobot_dir',
        default=os.path.join(
            get_package_share_directory('ros2-nurirobot-driver'),
            'launch'
        )
    )
    description_dir = LaunchConfiguration(
        'description_dir',
        default=os.path.join(
            get_package_share_directory('nuri_description'),
            'launch'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_parameter',
            default_value=lidar_parameter
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_dir, '/mpu6050driver_launch.py']),
            launch_arguments={'params_file': imu_parameter}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nurirobot_dir, '/nurirobot.launch.py']
            ),
        ),
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 0}  
            ],
            namespace='',
        ),        
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 1}  
            ],
            namespace='',
        ),        
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 2}  
            ],
            namespace='',
        ),        
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 3}  
            ],
            namespace='',
        ),        
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 4}  
            ],
            namespace='',
        ),        
        Node(
            package='ros2_sr04m_sensor',
            executable='AJ_SR04M_Node',
            name='AJ_SR04M_Node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'sensor_index': 5}  
            ],
            namespace='',
        ),        
        # Node(
        #     package='nuri_bringup',
        #     executable='imupublish_node',
        #     name='imupublish_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=None,
        #     namespace='',
        # ),
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_parameter],
            namespace='/',
        )          
    ])