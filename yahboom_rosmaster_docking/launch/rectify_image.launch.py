#!/usr/bin/env python3
"""
Launch file for image rectification processing.

This launch file sets up the image_proc rectification node that converts raw camera
images into rectified (undistorted) images for better computer vision processing.
It is particularly important for AprilTag detection accuracy.

Subscription Topics:
    /cam_1/color/image_raw (sensor_msgs/msg/Image): Raw camera image input
    /cam_1/depth/camera_info (sensor_msgs/msg/CameraInfo): Camera calibration information

Publishing Topics:
    /cam_1/color/image_rect (sensor_msgs/msg/Image): Rectified (undistorted) output image

Parameters:
    use_sim_time (bool, default: true): Use simulation clock when true
    queue_size (int, default: 5): Queue size for subscribers
    interpolation (int, default: 1): Interpolation method for image rectification (1 = Linear)
    namespace (string, default: cam_1): Namespace for the container node
    image_transport (string, default: raw): Transport hint for image topics

:author: Automatic Addison
:date: December 9, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate a launch description for image rectification.

    This function sets up all necessary parameters and nodes required to perform
    image rectification. It handles:
    1. Setting up configurable topic names through launch arguments
    2. Creating a composable node container for better performance
    3. Configuring the image_proc RectifyNode with appropriate parameters

    Returns:
        LaunchDescription: A complete launch description for image rectification
    """
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='cam_1',
        description='Namespace for the container node'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create a relay node to republish camera info from depth to color namespace
    camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='camera_info_relay',
        namespace=namespace,
        arguments=[
            'depth/camera_info',  # Input topic
            'color/camera_info'   # Output topic
        ]
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
    )

    # Create the container
    start_image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[rectify_node],
        output='screen'
    )

    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the relay node first to ensure camera_info is available
    ld.add_action(camera_info_relay)

    # Add the container
    ld.add_action(start_image_proc_container)

    return ld
