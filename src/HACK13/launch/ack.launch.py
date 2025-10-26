import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

import xacro


def generate_launch_description():
  package_name = "hack13"

  cmd_interpreter = Node(
    package=package_name,
    executable="cmd_interpreter",
    parameters=[{"use_sim_time": True}],
  )

  ack_calc = Node(
    package=package_name,
    executable="ack_calc.py",
    parameters=[{"use_sim_time": True}],
  )

  transform = Node(
    package=package_name,
    executable="transform",
    parameters=[{"use_sim_time": True}],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      cmd_interpreter,
      ack_calc,
      transform,
      # robot_state_publisher,
    ]
  )
