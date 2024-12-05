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


# Constants
COSTMAP_CLEARING_PERIOD = 0.5  # Time period for clearing costmaps (in seconds)
NAVIGATION_LOOP_DELAY = 0.1  # Delay in navigation loop to prevent CPU overuse (in seconds)


class NavigationState:
    """
    Encapsulates the global state of navigation.

    :param stop_requested: Flag to indicate if navigation stop is requested
    :param in_progress: Flag to indicate if navigation is currently in progress
    :param moving_forward: Flag to indicate if the robot is moving forward
    """

    def __init__(self):
        self.stop_requested = False
        self.in_progress = False
        self.moving_forward = True


# Global navigation state
nav_state = NavigationState()


class GoToGoalPose(Node):
    """
    ROS 2 node for navigating to a goal pose and publishing status updates.

    This node handles the main navigation logic, including processing
    feedback, publishing status updates, and managing the navigation lifecycle.
    """

    def __init__(self):
        """
        Initialize the GoToGoalPose node.

        Sets up publishers, subscribers, and initializes the navigation system.
        """
        super().__init__('go_to_goal_pose')

        # Set up publishers for ETA and status updates
        self.publisher_eta = self.create_publisher(String, '/goal_pose/eta', 10)
        self.publisher_status = self.create_publisher(String, '/goal_pose/status', 10)

        # Set up subscriber for receiving goal poses
        self.subscription_go_to_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose/goal',
            self.go_to_goal_pose,
            10)

        # Initialize time tracking for costmap clearing
        self.last_costmap_clear_time = self.get_clock().now()

        # Initialize the navigation system
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def go_to_goal_pose(self, msg):
        """
        Navigate to the specified goal pose.

        :param msg: The goal pose message
        :type msg: geometry_msgs/PoseStamped
        """
        # Clear costmaps before starting navigation
        self.navigator.clearAllCostmaps()

        # Start navigation to the goal pose
        self.navigator.goToPose(msg)

        # Main navigation loop
        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self._process_feedback(feedback)
                self._check_costmap_clearing()

            # Check if navigation should be stopped
            if nav_state.stop_requested:
                self.navigator.cancelTask()
                self.get_logger().info('Navigation cancellation request fulfilled.')
                break

            time.sleep(NAVIGATION_LOOP_DELAY)

        # Finalize navigation process
        self._finalize_navigation()

    def _process_feedback(self, feedback):
        """
        Process and publish navigation feedback.

        :param feedback: Navigation feedback from the robot
        :type feedback: nav2_msgs.action.NavigateToPose.Feedback
        """
        eta = self._calculate_eta(feedback)
        self._publish_eta(eta)
        self._publish_status("IN_PROGRESS")
        nav_state.in_progress = True

    def _calculate_eta(self, feedback):
        """
        Calculate estimated time of arrival.

        :param feedback: Navigation feedback from the robot
        :type feedback: nav2_msgs.action.NavigateToPose.Feedback
        :return: Estimated time of arrival in seconds
        :rtype: str
        """
        time_remaining_seconds = Duration.from_msg(
            feedback.estimated_time_remaining).nanoseconds / 1e9
        return f"{time_remaining_seconds:.0f}"

    def _publish_eta(self, eta):
        """
        Publish estimated time of arrival.

        :param eta: Estimated time of arrival in seconds
        :type eta: str
        """
        msg_eta = String()
        msg_eta.data = str(eta)
        self.publisher_eta.publish(msg_eta)

    def _publish_status(self, status):
        """
        Publish navigation status.

        :param status: Current navigation status
        :type status: str
        """
        msg_status = String()
        msg_status.data = status
        self.publisher_status.publish(msg_status)

    def _check_costmap_clearing(self):
        """
        Check and clear costmaps if necessary.

        Clears costmaps if the robot is not moving forward and a certain time has passed.
        This scenario could occur when a person walks in front of the robot, causing the
        robot to stop. We want to make sure the costmap is cleared so the robot can continue
        on its way.
        """
        current_time = self.get_clock().now()
        time_difference = (current_time - self.last_costmap_clear_time).nanoseconds / 1e9
        if (not nav_state.moving_forward and
                time_difference > COSTMAP_CLEARING_PERIOD):
            self.navigator.clearAllCostmaps()
            self.last_costmap_clear_time = current_time

    def _finalize_navigation(self):
        """
        Finalize navigation and publish result.

        Resets navigation state flags and publishes the final navigation status.
        """
        nav_state.stop_requested = False
        nav_state.in_progress = False

        result = self.navigator.getResult()
        status = self._get_status_string(result)
        self._publish_status(status)
        self.get_logger().info(f'Navigation result: {status}')

    def _get_status_string(self, result):
        """
        Get string representation of navigation result.

        :param result: The TaskResult enum value
        :type result: TaskResult
        :return: String representation of the result
        :rtype: str
        """
        if result == TaskResult.SUCCEEDED:
            return "SUCCEEDED"
        elif result == TaskResult.CANCELED:
            return "CANCELED"
        elif result == TaskResult.FAILED:
            return "FAILED"
        else:
            return "INVALID"


class GetStopNavigationSignal(Node):
    """
    ROS 2 node for receiving stop navigation signals.

    This node listens for signals to stop the current navigation task.
    """

    def __init__(self):
        """
        Initialize the GetStopNavigationSignal node.

        Sets up a subscriber for stop navigation signals.
        """
        super().__init__('get_stop_navigation_signal')

        self.subscription_stop_navigation = self.create_subscription(
            Bool,
            '/stop/navigation/go_to_goal_pose',
            self.set_stop_navigation,
            10)

    def set_stop_navigation(self, msg):
        """
        Set the stop navigation flag based on received message.

        :param msg: The stop navigation message
        :type msg: std_msgs/Bool
        """
        if nav_state.in_progress and msg.data:
            nav_state.stop_requested = True
            self.get_logger().info('Navigation cancellation request received.')


class GetCurrentVelocity(Node):
    """
    ROS 2 node for monitoring current velocity.

    This node listens to the robot's current velocity and updates the navigation state.
    """

    def __init__(self):
        """
        Initialize the GetCurrentVelocity node.

        Sets up a subscriber for current velocity messages.
        """
        super().__init__('get_current_velocity')

        self.subscription_current_velocity = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_current_velocity,
            1)

    def get_current_velocity(self, msg):
        """
        Determine if the robot is making forward progress based on current velocity.

        :param msg: The current velocity message
        :type msg: geometry_msgs/Twist
        """
        nav_state.moving_forward = msg.linear.x > 0.0


def main(args=None):
    """
    Main function to initialize and run the ROS 2 nodes.

    :param args: Command-line arguments
    :type args: list
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    try:
        # Create instances of all three nodes
        nodes = [
            GoToGoalPose(),
            GetStopNavigationSignal(),
            GetCurrentVelocity()
        ]

        # Set up multithreaded executor
        executor = MultiThreadedExecutor()
        for node in nodes:
            executor.add_node(node)

        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown and destroy all nodes
            executor.shutdown()
            for node in nodes:
                node.destroy_node()

    finally:
        # Shutdown the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
