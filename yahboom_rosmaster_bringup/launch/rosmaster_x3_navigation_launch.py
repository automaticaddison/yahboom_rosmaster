#!/usr/bin/env python3
"""
Launch Nav2 for the Yahboom ROSMASTER X3 robot in Gazebo.

This launch file sets up a complete ROS 2 navigation environment.

:author: Addison Sears-Collins
:date: December 10, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description.

    Returns:
        LaunchDescription: A complete launch description for the robot.
    """
    # Constants for paths to different packages
    package_name_docking = 'yahboom_rosmaster_docking'
    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    package_name_localization = 'yahboom_rosmaster_localization'
    package_name_navigation = 'yahboom_rosmaster_navigation'

    # Launch and config file paths
    apriltag_launch_file_path = 'launch/apriltag_dock_pose_publisher.launch.py'
    gazebo_launch_file_path = 'launch/yahboom_rosmaster.gazebo.launch.py'
    ekf_launch_file_path = 'launch/ekf_gazebo.launch.py'
    ekf_config_file_path = 'config/ekf.yaml'
    map_file_path = 'maps/cafe_world_map.yaml'
    nav2_params_path = 'config/rosmaster_x3_nav2_default_params.yaml'
    rviz_config_file_path = 'rviz/nav2_default_view.rviz'

    # Set the path to different packages
    pkg_share_docking = FindPackageShare(
        package=package_name_docking).find(package_name_docking)
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_localization = FindPackageShare(
        package=package_name_localization).find(package_name_localization)
    pkg_share_navigation = FindPackageShare(
        package=package_name_navigation).find(package_name_navigation)

    # Set default paths
    default_apriltag_launch_path = PathJoinSubstitution(
        [pkg_share_docking, apriltag_launch_file_path])
    default_gazebo_launch_path = PathJoinSubstitution([pkg_share_gazebo, gazebo_launch_file_path])
    default_ekf_launch_path = PathJoinSubstitution([pkg_share_localization, ekf_launch_file_path])
    default_ekf_config_path = PathJoinSubstitution([pkg_share_localization, ekf_config_file_path])
    default_rviz_config_path = PathJoinSubstitution([pkg_share_navigation, rviz_config_file_path])

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = PathJoinSubstitution([nav2_dir, 'launch'])
    nav2_params_file_path = PathJoinSubstitution([pkg_share_navigation, nav2_params_path])
    static_map_path = PathJoinSubstitution([pkg_share_navigation, map_file_path])

    # Launch configuration variables
    # Config and launch files
    autostart = LaunchConfiguration('autostart')
    camera_namespace = LaunchConfiguration('camera_namespace')
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    ekf_launch_file = LaunchConfiguration('ekf_launch_file')
    gazebo_launch_file = LaunchConfiguration('gazebo_launch_file')
    map_yaml_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_composition = LaunchConfiguration('use_composition')
    use_namespace = LaunchConfiguration('use_namespace')
    use_respawn = LaunchConfiguration('use_respawn')

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

    # Feature flags related to Gazebo and the robot description
    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare all launch arguments
    # Config and launch files
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack')

    declare_camera_namespace_cmd = DeclareLaunchArgument(
        name='camera_namespace',
        default_value='cam_1',
        description='Namespace for the camera')

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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        name='nav2_params_file',
        default_value=nav2_params_file_path,
        description='Full path to the ROS2 parameters file to use for navigation nodes')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    # Robot configuration
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='rosmaster_x3',
        description='The name for the robot')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value='cafe.world',
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
        default_value='0.20',
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

    declare_use_composition_cmd = DeclareLaunchArgument(
        name='use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        name='use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

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
    start_apriltag_dock_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([default_apriltag_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_namespace': camera_namespace
        }.items()
    )

    start_assisted_teleop_cmd = Node(
        package='yahboom_rosmaster_navigation',
        executable='assisted_teleoperation.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Start the node that relays /cmd_vel to /mecanum_drive_controller/cmd_vel
    start_cmd_vel_relay_cmd = Node(
        package='yahboom_rosmaster_navigation',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Start Extended Kalman Filter node from the robot_localization ROS 2 package
    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ekf_launch_file]),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

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

    # Start the node that receives goal poses and sends the robot there
    start_nav_to_pose_cmd = Node(
        package='yahboom_rosmaster_navigation',
        executable='nav_to_pose.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([nav2_launch_dir, 'bringup_launch.py'])]
                                      ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add all launch arguments
    # Config and launch files
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_camera_namespace_cmd)
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_ekf_launch_file_cmd)
    ld.add_action(declare_gazebo_launch_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_respawn_cmd)

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
    ld.add_action(start_apriltag_dock_cmd)
    ld.add_action(start_assisted_teleop_cmd)
    ld.add_action(start_cmd_vel_relay_cmd)
    ld.add_action(start_ekf_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_nav_to_pose_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    return ld
