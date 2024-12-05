#!/usr/bin/env python3

"""
ROS 2 node for publishing a single goal pose.
This script creates a ROS 2 node that publishes a single goal pose to the
/goal_pose/goal topic.

Publishing Topics:
    /goal_pose/goal (geometry_msgs/PoseStamped): The desired goal pose

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


class GoalPublisher(Node):
    """
    ROS 2 node for publishing a single goal pose.
    This node publishes a predefined goal pose once and then shuts down.
    """

    def __init__(self):
        """
        Initialize the GoalPublisher node.
        Sets up the publisher for the goal pose.
        """
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose/goal', 10)
        # Create a timer that will trigger the publishing of the goal pose
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.get_logger().info('Goal Publisher node has been initialized')

    def publish_goal(self):
        """
        Publish the goal pose and shut down the node.
        This method creates a PoseStamped message with a predefined goal,
        publishes it, and then shuts down the node.
        """
        msg = PoseStamped()

        header = Header()
        header.frame_id = 'map'

        # Get current ROS time and convert to Time message
        now = self.get_clock().now()
        header.stamp = Time(
            sec=now.seconds_nanoseconds()[0],
            nanosec=now.seconds_nanoseconds()[1]
        )

        # Create and fill the position message with x, y, z coordinates
        position = Point()
        position.x = 2.0
        position.y = 2.0
        position.z = 0.0

        # Create and fill the orientation message with quaternion values
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 1.0

        # Create Pose message and combine position and orientation
        pose = Pose()
        pose.position = position
        pose.orientation = orientation

        # Combine header and pose into the final PoseStamped message
        msg.header = header
        msg.pose = pose

        self.publisher.publish(msg)
        self.get_logger().info('Goal pose published')

        # Cancel the timer and indicate shutdown, but don't call rclpy.shutdown()
        self.timer.cancel()
        self.get_logger().info('Goal Publisher node is shutting down')


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.

    :param args: Command-line arguments
    :type args: list
    """
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
