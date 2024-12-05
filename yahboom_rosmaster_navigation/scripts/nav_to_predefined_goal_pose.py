#!/usr/bin/env python3
"""
ROS 2 node for navigating to predefined table locations.

This script creates a ROS 2 node that:
   - Allows user to select a predefined table location
   - Navigates the robot to the selected table
   - Publishes the estimated time of arrival
   - Publishes the goal status
   - Cancels the goal if a stop signal is received

Subscription Topics:
   /stop/navigation/go_to_predefined_goal_pose (std_msgs/Bool): Signal to stop navigation
   /cmd_vel (geometry_msgs/Twist): Velocity command

Publishing Topics:
   /goal_pose/eta (std_msgs/String): Estimated time of arrival in seconds
   /goal_pose/status (std_msgs/String): Goal pose status

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler

# Import our custom PoseStamped generator
from yahboom_rosmaster_navigation.posestamped_msg_generator import PoseStampedGenerator

# Global flags for navigation state
STOP_NAVIGATION_NOW = False
NAV_IN_PROGRESS = False
MOVING_FORWARD = True


class GoToPredefinedGoalPose(Node):
    """ROS 2 node for navigating to predefined table locations."""

    def __init__(self):
        """Initialize the navigation node."""
        super().__init__('go_to_predefined_goal_pose')

        # Initialize the PoseStamped generator
        self.pose_generator = PoseStampedGenerator('pose_generator')

        # Publishers
        self.publisher_eta = self.create_publisher(String, '/goal_pose/eta', 10)
        self.publisher_status = self.create_publisher(String, '/goal_pose/status', 10)

        # Keep track of time for clearing the costmaps
        self.current_time = self.get_clock().now().nanoseconds
        self.last_time = self.current_time
        self.dt = (self.current_time - self.last_time) * 1e-9

        # Clear the costmap every 0.5 seconds when the robot is not making forward progress
        self.costmap_clearing_period = 0.5

        # Initialize navigation
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Define table locations [x, y, yaw_angle]
        self.locations = {
            'table_1': [-0.96, -0.92, 0.0],
            'table_2': [1.16, -4.23, 0.0],
            'table_3': [0.792, -8.27, 0.0],
            'table_4': [-3.12, -7.495, 0.0],
            'table_5': [-2.45, -3.55, 0.0]
        }

    def create_goal_pose(self, x, y, yaw):
        """Create a PoseStamped message for a goal."""
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return self.pose_generator.create_pose_stamped(
            x=x, y=y, z=0.0,
            qx=q[0], qy=q[1], qz=q[2], qw=q[3],
            frame_id='map'
        )

    def go_to_goal_pose(self, table_name):
        """Navigate to a specific table location."""
        global NAV_IN_PROGRESS, STOP_NAVIGATION_NOW

        # Get table coordinates and create goal pose
        loc = self.locations[table_name]
        goal_pose = self.create_goal_pose(loc[0], loc[1], loc[2])

        # Clear all costmaps before sending to a goal
        self.navigator.clearAllCostmaps()
        self.navigator.goToPose(goal_pose)

        # As long as the robot is moving to the goal pose
        while rclpy.ok() and not self.navigator.isTaskComplete():
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
                self.current_time = self.get_clock().now().nanoseconds
                self.dt = (self.current_time - self.last_time) * 1e-9

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

        # Publish the final status
        result = self.navigator.getResult()
        msg_status = String()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully reached the goal!')
            msg_status.data = "SUCCEEDED"
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            msg_status.data = "CANCELED"
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            msg_status.data = "FAILED"
        else:
            self.get_logger().info('Goal has an invalid return status!')
            msg_status.data = "INVALID"

        self.publisher_status.publish(msg_status)


class GetStopNavigationSignal(Node):
    """This class subscribes to a Boolean flag that tells the robot to stop navigation."""

    def __init__(self):
        """Constructor."""
        super().__init__('get_stop_navigation_signal')

        self.subscription_stop_navigation = self.create_subscription(
            Bool,
            '/stop/navigation/go_to_predefined_goal_pose',
            self.set_stop_navigation,
            10)

    def set_stop_navigation(self, msg):
        """Determine if the robot needs to stop."""
        global STOP_NAVIGATION_NOW

        if NAV_IN_PROGRESS and msg.data:
            STOP_NAVIGATION_NOW = msg.data
            self.get_logger().info('Navigation cancellation request received by ROS 2...')


class GetCurrentVelocity(Node):
    """This class subscribes to the current velocity."""

    def __init__(self):
        """Constructor."""
        super().__init__('get_current_velocity')

        self.subscription_current_velocity = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_current_velocity,
            1)

    def get_current_velocity(self, msg):
        """Get the current velocity."""
        global MOVING_FORWARD

        MOVING_FORWARD = msg.linear.x > 0.0


def main(args=None):
    """Main function to initialize and run the ROS 2 nodes."""
    rclpy.init(args=args)

    try:
        # Create the nodes
        go_to_predefined_goal_pose = GoToPredefinedGoalPose()
        get_stop_navigation_signal = GetStopNavigationSignal()
        get_current_velocity = GetCurrentVelocity()

        # Set up multithreading
        executor = MultiThreadedExecutor()
        executor.add_node(go_to_predefined_goal_pose)
        executor.add_node(get_stop_navigation_signal)
        executor.add_node(get_current_velocity)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        try:
            while rclpy.ok():
                print("\nAVAILABLE TABLES:")
                print(*go_to_predefined_goal_pose.locations.keys(), sep="\n")

                user_input = input('\nEnter table number (1-5) or "exit": ').lower().strip()

                if user_input == 'exit':
                    break

                if user_input.isdigit() and 1 <= int(user_input) <= 5:
                    table_name = f'table_{user_input}'
                    go_to_predefined_goal_pose.go_to_goal_pose(table_name)

                    if STOP_NAVIGATION_NOW:
                        user_input = input(
                            "Navigation cancelled. Press 'c' to continue or any key to exit: ")
                        if user_input.lower() != 'c':
                            break
                        STOP_NAVIGATION_NOW = False
                else:
                    print("Invalid input. Please enter a number between 1 and 5.")

        finally:
            # Shutdown the executor and nodes
            executor.shutdown()
            go_to_predefined_goal_pose.destroy_node()
            get_stop_navigation_signal.destroy_node()
            get_current_velocity.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
