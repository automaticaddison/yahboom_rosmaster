#!/usr/bin/env python3
"""
Launch file for mecanum_parameters_node.

This launch file launches the mecanum_parameters_node with configurable parameters.

:author: Addison Sears-Collins
:date: November 24, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the mecanum_parameters_node.

    :return: A LaunchDescription object containing the actions to execute.
    :rtype: LaunchDescription
    """

    # Constants for paths to different files and folders
    package_name_system_tests = 'yahboom_rosmaster_system_tests'

    mecanum_params_path = 'config/mecanum_parameters.yaml'

    # Set the path to different files and folders
    pkg_share_system_tests = FindPackageShare(
        package=package_name_system_tests).find(package_name_system_tests)

    mecanum_params_path = os.path.join(pkg_share_system_tests, mecanum_params_path)

    # Launch configuration variables
    mecanum_params = LaunchConfiguration('mecanum_params')

    # Declare the launch arguments
    declare_mecanum_params_cmd = DeclareLaunchArgument(
        name='mecanum_params',
        default_value=mecanum_params_path,
        description='Full path to parameters for the parameters file.')

    # Launch the mecanum_parameters_node
    start_mecanum_parameters_node_cmd = Node(
        package=package_name_system_tests,
        executable='mecanum_parameters_node',
        parameters=[mecanum_params],
        output='screen')

    # Create the launch description and populate it
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_mecanum_params_cmd)

    # Add actions to the launch description
    ld.add_action(start_mecanum_parameters_node_cmd)

    return ld
