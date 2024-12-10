#!/usr/bin/env python3
"""
Launch file for publishing the pose of the AprilTag on the detected_dock_pose topic as a
geometry_msgs/PoseStamped message.

:author: Automatic Addison
:date: December 9, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate a launch description.

    Returns:
        LaunchDescription: A complete launch description
    """
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
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

    # Create the container
    start_apriltag_dock_pose_publisher = ComposableNodeContainer(
        name='apriltag_dock_pose_publisher',
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

    # Add the container
    ld.add_action(start_apriltag_dock_pose_publisher)

    return ld
