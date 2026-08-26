#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the mecanum wheel robot.

This script creates a launch description that starts the necessary controllers
for operating the mecanum wheel robot in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Mecanum Drive Controller: Controls the robot's mecanum drive movements via ~/cmd_vel

:author: Addison Sears-Collins
:date: November 20, 2024
"""

from launch import LaunchDescription
import os
from pathlib import Path

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    The function creates a launch sequence that ensures controllers are started
    in the correct order.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """

    # The controllers read their settings from this file. We hand it to each
    # spawner directly, because the controllers do not pick it up on their own.
    controllers_file = os.path.join(
        str(Path.home()),
        'ros2_ws/install/yahboom_rosmaster_description/share',
        'yahboom_rosmaster_description/config/rosmaster_x3/ros2_controllers.yaml')
    # Start mecanum drive controller
    start_mecanum_drive_controller_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--param-file', controllers_file],
        output='screen'
    )

    # Start joint state broadcaster
    start_joint_state_broadcaster_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_file],
        output='screen'
    )

    # Add delay to joint state broadcaster (if necessary)
    delayed_start = TimerAction(
        period=25.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # Register event handler for sequencing
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_mecanum_drive_controller_cmd]))

    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)

    return ld
