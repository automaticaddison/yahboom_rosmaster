#!/usr/bin/env python3
"""
Launch Nav2 for the Yahboom ROSMASTER X3 robot in Gazebo.

This launch file sets up a complete ROS 2 navigation environment.

:author: Addison Sears-Collins
:date: November 29, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description.

    Returns:
        LaunchDescription: A complete launch description for the robot.
    """
    # Constants for paths to different packages
    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    package_name_localization = 'yahboom_rosmaster_localization'

    # Launch and config file paths
    gazebo_launch_file_path = 'launch/yahboom_rosmaster.gazebo.launch.py'
    ekf_launch_file_path = 'launch/ekf_gazebo.launch.py'
    ekf_config_file_path = 'config/ekf.yaml'
    rviz_config_file_path = 'rviz/yahboom_rosmaster_gazebo_sim.rviz'

    # Set the path to different packages
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_localization = FindPackageShare(
        package=package_name_localization).find(package_name_localization)

    # Set default paths
    default_gazebo_launch_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)
    default_ekf_launch_path = os.path.join(pkg_share_localization, ekf_launch_file_path)
    default_ekf_config_path = os.path.join(pkg_share_localization, ekf_config_file_path)
    default_rviz_config_path = os.path.join(pkg_share_gazebo, rviz_config_file_path)

    # Launch configuration variables
    # Config and launch files
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    ekf_launch_file = LaunchConfiguration('ekf_launch_file')
    gazebo_launch_file = LaunchConfiguration('gazebo_launch_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Robot configuration
    robot_name = LaunchConfiguration('robot_name')
    world_file = LaunchConfiguration('world_file')

    # Position and orientation
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Feature flags
    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare all launch arguments
    # Config and launch files
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF configuration YAML file')

    declare_ekf_launch_file_cmd = DeclareLaunchArgument(
        name='ekf_launch_file',
        default_value=default_ekf_launch_path,
        description='Full path to the EKF launch file to use')

    declare_enable_odom_tf_cmd = DeclareLaunchArgument(
        name='enable_odom_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to enable odometry transform broadcasting via ROS 2 Control')

    declare_gazebo_launch_file_cmd = DeclareLaunchArgument(
        name='gazebo_launch_file',
        default_value=default_gazebo_launch_path,
        description='Full path to the Gazebo launch file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    # Robot configuration
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='rosmaster_x3',
        description='The name for the robot')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value='empty.world',
        description='World file name (e.g., empty.world, house.world)')

    # Position arguments
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.05',
        description='z component of initial position, meters')

    # Orientation arguments
    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # Feature flags
    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient (visualization)')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Specify the actions
    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'enable_odom_tf': enable_odom_tf,
            'headless': headless,
            'jsp_gui': jsp_gui,
            'load_controllers': load_controllers,
            'robot_name': robot_name,
            'rviz_config_file': rviz_config_file,
            'use_rviz': use_rviz,
            'use_gazebo': use_gazebo,
            'use_robot_state_pub': use_robot_state_pub,
            'use_sim_time': use_sim_time,
            'world_file': world_file,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }.items()
    )

    # Start EKF
    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ekf_launch_file]),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add all launch arguments
    # Config and launch files
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_ekf_launch_file_cmd)
    ld.add_action(declare_gazebo_launch_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Robot configuration
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_world_cmd)

    # Position declarations
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)

    # Orientation declarations
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Feature flags
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_ekf_cmd)
    ld.add_action(start_gazebo_cmd)

    return ld
