#!/usr/bin/env python3
"""
Create a PoseStamped message for ROS 2 navigation.

This script creates a geometry_msgs/PoseStamped message which is commonly used
for robot navigation in ROS 2. It shows how to properly construct a PoseStamped
message with header information and pose data.

:author: Addison Sears-Collins
:date: December 5, 2024
"""

from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


class PoseStampedGenerator(Node):
    """A ROS 2 node that generates PoseStamped messages."""

    def __init__(self, node_name='pose_stamped_generator'):
        """
        Initialize the ROS 2 node.

        :param node_name: Name of the node
        :type node_name: str
        """
        super().__init__(node_name)

    def create_pose_stamped(self, x=0.0, y=0.0, z=0.0,
                            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                            frame_id='map'):
        """
        Create a PoseStamped message with the specified parameters.

        :param x: x position coordinate
        :type x: float
        :param y: y position coordinate
        :type y: float
        :param z: z position coordinate
        :type z: float
        :param qx: x quaternion component
        :type qx: float
        :param qy: y quaternion component
        :type qy: float
        :param qz: z quaternion component
        :type qz: float
        :param qw: w quaternion component
        :type qw: float
        :param frame_id: reference frame
        :type frame_id: str
        :return: pose stamped message
        :rtype: PoseStamped
        """
        # Create the PoseStamped message object
        pose_stamped = PoseStamped()

        # Create and fill the header with frame_id and timestamp
        header = Header()
        header.frame_id = frame_id

        # Get current ROS time and convert to Time message
        now = self.get_clock().now()
        header.stamp = Time(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1]
        )

        # Create and fill the position message with x, y, z coordinates
        position = Point()
        position.x = float(x)
        position.y = float(y)
        position.z = float(z)

        # Create and fill the orientation message with quaternion values
        orientation = Quaternion()
        orientation.x = float(qx)
        orientation.y = float(qy)
        orientation.z = float(qz)
        orientation.w = float(qw)

        # Create Pose message and combine position and orientation
        pose = Pose()
        pose.position = position
        pose.orientation = orientation

        # Combine header and pose into the final PoseStamped message
        pose_stamped.header = header
        pose_stamped.pose = pose

        return pose_stamped
