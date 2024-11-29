#!/usr/bin/env python3
"""
Launch file for the Extended Kalman Filter (EKF) node in Gazebo simulation.

This script starts the robot_localization package's EKF node which combines (fuses)
data from wheel odometry and IMU sensors to better estimate the robot's position
and orientation.

:author: Addison Sears-Collins
:date: November 29, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the EKF node.

    This function creates and returns a LaunchDescription object that will start
    the EKF node from the robot_localization package. The node is configured
    using parameters from a YAML file.

    Returns:
        LaunchDescription: A complete launch description for the EKF node
    """
    # Constants for paths to different files and folders
    package_name = 'yahboom_rosmaster_localization'

    # Config file paths
    ekf_config_file_path = 'config/ekf.yaml'

    # Set the path to different packages
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Set the path to config files
    default_ekf_config_path = os.path.join(pkg_share, ekf_config_file_path)

    # Launch configuration variables
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF configuration YAML file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Specify the actions
    start_ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declarations
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions
    ld.add_action(start_ekf_node_cmd)

    return ld
