#!/usr/bin/env python3
"""
Launch file for publishing the pose of the AprilTag on the detected_dock_pose topic as a
geometry_msgs/PoseStamped message.

:author: Automatic Addison
:date: December 9, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description.

    Returns:
        LaunchDescription: A complete launch description
    """
    # Constants for paths to different files and folders
    package_name_docking = 'yahboom_rosmaster_docking'

    apriltag_ros_config_filename = 'apriltags_36h11.yaml'

    # Set the path to different files and folders
    pkg_share_docking = FindPackageShare(package=package_name_docking).find(package_name_docking)

    default_apriltag_ros_config_file_path = PathJoinSubstitution(
        [pkg_share_docking, 'config', apriltag_ros_config_filename])

    # Launch configuration variables
    apriltag_config_file = LaunchConfiguration('apriltag_config_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_apriltag_config_file_cmd = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=default_apriltag_ros_config_file_path,
        description='Full path to the AprilTag config file to use')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='cam_1',
        description='Namespace for the nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create the rectify node
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
        namespace=namespace,
        remappings=[
            ('image', 'color/image_raw'),
            ('image_rect', 'color/image_rect'),
        ],
        parameters=[{
            'queue_size': 5,
            'interpolation': 1,
            'use_sim_time': use_sim_time,
            'image_transport': 'raw'
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create the AprilTag node
    apriltag_ros_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag_node',
        namespace=namespace,
        remappings=[
            ('image_rect', 'color/image_rect'),
            ('camera_info', 'color/camera_info'),
        ],
        parameters=[
            apriltag_config_file,
            {'use_sim_time': use_sim_time}
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create the container
    start_apriltag_dock_pose_publisher = ComposableNodeContainer(
        name='apriltag_dock_pose_publisher',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            apriltag_ros_node
        ],
        output='screen'
    )

    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the arguments
    ld.add_action(declare_apriltag_config_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the container
    ld.add_action(start_apriltag_dock_pose_publisher)

    return ld
