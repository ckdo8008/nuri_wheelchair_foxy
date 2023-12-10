#!/usr/bin/env python3

# Author: Changu Do

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
  teleop_joy_parameter = LaunchConfiguration(
    'teleop_joy_parameter',
    default=os.path.join(
      get_package_share_directory('nuri_teleop'),
      'param/teleop_joy.param.yaml'
    )
  )
  return LaunchDescription([
    DeclareLaunchArgument('teleop_joy_parameter', default_value=teleop_joy_parameter),
    Node(
      package='nuri_teleop', executable='teleop_joy',
      name='teleop_joy_node',
      output='screen',
      emulate_tty=True,
      parameters=[teleop_joy_parameter],
    ),

  ])