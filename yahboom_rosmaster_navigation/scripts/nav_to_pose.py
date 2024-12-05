#!/usr/bin/env python3
"""
ROS 2 node for navigating to a goal pose and publishing status updates.

This script creates a ROS 2 node that:
    - Subscribes to a goal pose
    - Navigates the robot to the goal pose
    - Publishes the estimated time of arrival
    - Publishes the goal status
    - Cancels the goal if a stop signal is received

Subscription Topics:
    /goal_pose/goal (geometry_msgs/PoseStamped): The desired goal pose
    /stop/navigation/go_to_goal_pose (std_msgs/Bool): Signal to stop navigation
    /cmd_vel (geometry_msgs/Twist): Velocity command

Publishing Topics:
    /goal_pose/eta (std_msgs/String): Estimated time of arrival in seconds
    /goal_pose/status (std_msgs/String): Goal pose status

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import time
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped

# Global flags for navigation state
STOP_NAVIGATION_NOW = False
NAV_IN_PROGRESS = False
MOVING_FORWARD = True

COSTMAP_CLEARING_PERIOD = 0.5


class GoToGoalPose(Node):
    """This class subscribes to the goal pose and publishes the ETA
       and goal status information to ROS 2.
    """

    def __init__(self):
        """Constructor."""
        # Initialize the class using the constructor
        super().__init__('go_to_goal_pose')

        # Create the ROS 2 publishers
        self.publisher_eta = self.create_publisher(String, '/goal_pose/eta', 10)
        self.publisher_status = self.create_publisher(String, '/goal_pose/status', 10)

        # Create a subscriber
        # This node subscribes to messages of type geometry_msgs/PoseStamped
        self.subscription_go_to_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose/goal',
            self.go_to_goal_pose,
            10)

        # Keep track of time for clearing the costmaps
        self.current_time = self.get_clock().now().nanoseconds
        self.last_time = self.current_time
        self.dt = (self.current_time - self.last_time) * 1e-9

        # Clear the costmap every X.X seconds when the robot is not making forward progress
        self.costmap_clearing_period = COSTMAP_CLEARING_PERIOD

        # Launch the ROS 2 Navigation Stack
        self.navigator = BasicNavigator()

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

    def go_to_goal_pose(self, msg):
        """Go to goal pose."""
        global NAV_IN_PROGRESS, STOP_NAVIGATION_NOW  # pylint: disable=global-statement

        # Clear all costmaps before sending to a goal
        self.navigator.clearAllCostmaps()

        # Go to the goal pose
        self.navigator.goToPose(msg)

        # As long as the robot is moving to the goal pose
        while rclpy.ok() and not self.navigator.isTaskComplete():
            # Get the feedback message
            feedback = self.navigator.getFeedback()

            if feedback:
                # Publish the estimated time of arrival in seconds
                estimated_time_of_arrival = f"{Duration.from_msg(
                    feedback.estimated_time_remaining).nanoseconds / 1e9:.0f}"
                msg_eta = String()
                msg_eta.data = str(estimated_time_of_arrival)
                self.publisher_eta.publish(msg_eta)

                # Publish the goal status
                msg_status = String()
                msg_status.data = "IN_PROGRESS"
                self.publisher_status.publish(msg_status)
                NAV_IN_PROGRESS = True

                # Clear the costmap at the desired frequency
                # Get the current time
                self.current_time = self.get_clock().now().nanoseconds

                # How much time has passed in seconds since the last costmap clearing
                self.dt = (self.current_time - self.last_time) * 1e-9

                # If we are making no forward progress after X seconds, clear all costmaps
                if not MOVING_FORWARD and self.dt > self.costmap_clearing_period:
                    self.navigator.clearAllCostmaps()
                    self.last_time = self.current_time

            # Stop the robot if necessary
            if STOP_NAVIGATION_NOW:
                self.navigator.cancelTask()
                self.get_logger().info('Navigation cancellation request fulfilled...')

            time.sleep(0.1)

        # Reset the variable values
        STOP_NAVIGATION_NOW = False
        NAV_IN_PROGRESS = False

        # Publish the final goal status
        result = self.navigator.getResult()
        msg_status = String()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully reached the goal!')
            msg_status.data = "SUCCEEDED"
            self.publisher_status.publish(msg_status)

        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            msg_status.data = "CANCELED"
            self.publisher_status.publish(msg_status)

        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            msg_status.data = "FAILED"
            self.publisher_status.publish(msg_status)

        else:
            self.get_logger().info('Goal has an invalid return status!')
            msg_status.data = "INVALID"
            self.publisher_status.publish(msg_status)


class GetStopNavigationSignal(Node):
    """This class subscribes to a Boolean flag that tells the
       robot to stop navigation.
    """

    def __init__(self):
        """Constructor."""
        # Initialize the class using the constructor
        super().__init__('get_stop_navigation_signal')

        # Create a subscriber
        # This node subscribes to messages of type std_msgs/Bool
        self.subscription_stop_navigation = self.create_subscription(
            Bool,
            '/stop/navigation/go_to_goal_pose',
            self.set_stop_navigation,
            10)

    def set_stop_navigation(self, msg):
        """Determine if the robot needs to stop, and adjust the variable value accordingly."""
        global STOP_NAVIGATION_NOW  # pylint: disable=global-statement

        if NAV_IN_PROGRESS and msg.data:
            STOP_NAVIGATION_NOW = msg.data
            self.get_logger().info('Navigation cancellation request received by ROS 2...')


class GetCurrentVelocity(Node):
    """This class subscribes to the current velocity and determines if the robot
       is making forward progress.
    """

    def __init__(self):
        """Constructor."""
        # Initialize the class using the constructor
        super().__init__('get_current_velocity')

        # Create a subscriber
        # This node subscribes to messages of type geometry_msgs/Twist
        self.subscription_current_velocity = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_current_velocity,
            1)

    def get_current_velocity(self, msg):
        """Get the current velocity, and determine if the robot is making forward progress."""
        global MOVING_FORWARD  # pylint: disable=global-statement

        linear_x_value = msg.linear.x

        if linear_x_value <= 0.0:
            MOVING_FORWARD = False
        else:
            MOVING_FORWARD = True


def main(args=None):
    """Main function to initialize and run the ROS 2 nodes."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    try:
        # Create the nodes
        go_to_goal_pose = GoToGoalPose()
        get_stop_navigation_signal = GetStopNavigationSignal()
        get_current_velocity = GetCurrentVelocity()

        # Set up multithreading
        executor = MultiThreadedExecutor()
        executor.add_node(go_to_goal_pose)
        executor.add_node(get_stop_navigation_signal)
        executor.add_node(get_current_velocity)

        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            go_to_goal_pose.destroy_node()
            get_stop_navigation_signal.destroy_node()
            get_current_velocity.destroy_node()

    finally:
        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == '__main__':
    main()
