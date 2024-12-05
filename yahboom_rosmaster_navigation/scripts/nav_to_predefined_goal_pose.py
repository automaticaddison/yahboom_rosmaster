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

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler


# Import our custom PoseStamped generator
from yahboom_rosmaster_navigation.posestamped_msg_generator import PoseStampedGenerator

# Constants
COSTMAP_CLEARING_PERIOD = 0.5
NAVIGATION_LOOP_DELAY = 0.1


class NavigationState:
    """Encapsulates the global state of navigation."""

    def __init__(self):
        self.stop_requested = False
        self.in_progress = False
        self.moving_forward = True


# Global navigation state
nav_state = NavigationState()


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

        # Subscribers
        self.subscription_stop_navigation = self.create_subscription(
            Bool,
            '/stop/navigation/go_to_predefined_goal_pose',
            self.set_stop_navigation,
            10)

        self.subscription_current_velocity = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_current_velocity,
            1)

        # Initialize navigation
        self.last_costmap_clear_time = self.get_clock().now()
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

    def set_stop_navigation(self, msg):
        """Handle stop navigation requests."""
        if nav_state.in_progress and msg.data:
            nav_state.stop_requested = True
            self.get_logger().info('Navigation cancellation request received.')

    def get_current_velocity(self, msg):
        """Monitor robot's forward progress."""
        nav_state.moving_forward = msg.linear.x > 0.0

    def go_to_goal_pose(self, table_name):
        """Navigate to a specific table location."""

        # Get table coordinates and create goal pose
        loc = self.locations[table_name]
        goal_pose = self.create_goal_pose(loc[0], loc[1], loc[2])

        # Start navigation
        self.navigator.clearAllCostmaps()
        self.navigator.goToPose(goal_pose)

        # Navigation loop
        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Calculate and publish ETA
                eta = f"{
                    Duration.from_msg(
                        feedback.estimated_time_remaining).nanoseconds /
                    1e9:.0f}"
                msg_eta = String()
                msg_eta.data = eta
                self.publisher_eta.publish(msg_eta)

                # Publish status
                msg_status = String()
                msg_status.data = "IN_PROGRESS"
                self.publisher_status.publish(msg_status)
                nav_state.in_progress = True

                # Clear costmaps if needed
                current_time = self.get_clock().now()
                time_difference = (current_time - self.last_costmap_clear_time).nanoseconds / 1e9
                if (not nav_state.moving_forward and
                        time_difference > COSTMAP_CLEARING_PERIOD):
                    self.navigator.clearAllCostmaps()
                    self.last_costmap_clear_time = current_time

            # Check for stop request
            if nav_state.stop_requested:
                self.navigator.cancelTask()
                self.get_logger().info('Navigation cancelled.')
                break

            time.sleep(NAVIGATION_LOOP_DELAY)

        # Finalize navigation
        nav_state.stop_requested = False
        nav_state.in_progress = False

        # Get and publish final status
        result = self.navigator.getResult()
        status = "SUCCEEDED" if result == TaskResult.SUCCEEDED else \
            "CANCELED" if result == TaskResult.CANCELED else \
            "FAILED" if result == TaskResult.FAILED else "UNKNOWN"

        msg_status = String()
        msg_status.data = status
        self.publisher_status.publish(msg_status)
        self.get_logger().info(f'Navigation result: {status}')

    def run(self):
        """Main run loop for table navigation."""
        while rclpy.ok():
            print("\nAVAILABLE TABLES:")
            print(*self.locations.keys(), sep="\n")

            user_input = input('\nEnter table number (1-5) or "exit": ').lower().strip()

            if user_input == 'exit':
                break

            if user_input.isdigit() and 1 <= int(user_input) <= 5:
                table_name = f'table_{user_input}'
                self.go_to_goal_pose(table_name)

                if nav_state.stop_requested:
                    user_input = input(
                        "Navigation cancelled. Press 'c' to continue or any key to exit: ")
                    if user_input.lower() != 'c':
                        break
                    nav_state.stop_requested = False
            else:
                print("Invalid input. Please enter a number between 1 and 5.")

        self.navigator.lifecycleShutdown()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = GoToPredefinedGoalPose()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
