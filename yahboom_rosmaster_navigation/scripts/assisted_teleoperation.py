#!/usr/bin/env python3
"""
Assisted Teleop Node for ROS 2 Navigation

This script implements an Assisted Teleop node that interfaces with the Nav2 stack.
It manages the lifecycle of the AssistedTeleop action, handles cancellation requests,
and periodically clears costmaps to remove temporary obstacles.

Subscription Topics:
   /cmd_vel_teleop (geometry_msgs/Twist): Velocity commands for assisted teleop
   /cancel_assisted_teleop (std_msgs/Bool): Cancellation requests for assisted teleop

Parameters:
   ~/costmap_clear_frequency (double): Frequency in Hz for costmap clearing. Default: 2.0

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_msgs.action import AssistedTeleop
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import ParameterDescriptor


class AssistedTeleopNode(Node):
    """
    A ROS 2 node for managing Assisted Teleop functionality.
    """

    def __init__(self):
        """Initialize the AssistedTeleopNode."""
        super().__init__('assisted_teleop_node')

        # Declare and get parameters
        self.declare_parameter(
            'costmap_clear_frequency',
            2.0,
            ParameterDescriptor(description='Frequency in Hz for costmap clearing')
        )
        clear_frequency = self.get_parameter('costmap_clear_frequency').value

        # Create action client for assisted teleop
        self._action_client = ActionClient(self, AssistedTeleop, 'assisted_teleop')

        # Create service clients for clearing costmaps
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')

        # Wait for costmap services to become available
        while not self._clear_global_costmap.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Global costmap clearing service not available, waiting...')
        while not self._clear_local_costmap.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Local costmap clearing service not available, waiting...')

        # Create subscribers for velocity commands and cancellation requests
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_vel_callback, 10)
        self.cancel_sub = self.create_subscription(
            Bool, '/cancel_assisted_teleop', self.cancel_callback, 10)

        # Initialize state variables
        self.assisted_teleop_active = False
        self.cancellation_requested = False
        self._current_goal_handle = None

        # Create a timer for periodic costmap clearing with configurable frequency
        period = 1.0 / clear_frequency
        self.clear_costmaps_timer = self.create_timer(period, self.clear_costmaps_callback)

        self.get_logger().info(
            f'Assisted Teleop Node initialized with costmap clearing frequency: {
                clear_frequency} Hz')

    def cmd_vel_callback(self, twist_msg: Twist) -> None:
        """Process incoming velocity commands and activate assisted teleop if needed."""
        if not self.assisted_teleop_active and not self.cancellation_requested:
            if (abs(twist_msg.linear.x) > 0.0 or
                abs(twist_msg.linear.y) > 0.0 or
                    abs(twist_msg.angular.z) > 0.0):
                self.start_assisted_teleop()

    def start_assisted_teleop(self) -> None:
        """Start the Assisted Teleop action with indefinite duration."""
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Assisted Teleop action server not available')
            return

        # Create and send goal
        goal_msg = AssistedTeleop.Goal()

        self.assisted_teleop_active = True
        self.cancellation_requested = False

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('AssistedTeleop goal sent')

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('AssistedTeleop goal rejected')
            self.assisted_teleop_active = False
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('AssistedTeleop goal accepted')

        # Get the result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the action result."""
        result = future.result()
        if result:
            status = result.status
            self.get_logger().info(f'AssistedTeleop completed with status: {status}')
        else:
            self.get_logger().warn('Failed to get AssistedTeleop result')
        self.assisted_teleop_active = False
        self.cancellation_requested = False  # Reset cancellation flag when complete

    def feedback_callback(self, feedback_msg):
        """Process action feedback."""
        if feedback_msg:
            feedback = feedback_msg.feedback
            self.get_logger().debug(f'Received feedback: {feedback}')

    def cancel_callback(self, msg: Bool) -> None:
        """Handle cancellation requests for assisted teleop."""
        if msg.data and self.assisted_teleop_active and not self.cancellation_requested:
            self.cancel_assisted_teleop()

    def cancel_assisted_teleop(self) -> None:
        """Cancel the currently running Assisted Teleop action."""
        if self.assisted_teleop_active and self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.assisted_teleop_active = False
            self.cancellation_requested = True
            self.get_logger().info('AssistedTeleop cancellation requested')

    def cancel_done_callback(self, future):
        """Handle the cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('AssistedTeleop goal successfully cancelled')
        else:
            self.get_logger().warn('Failed to cancel AssistedTeleop goal')

    def clear_costmaps_callback(self) -> None:
        """Periodically clear all costmaps to remove temporary obstacles."""
        if not self.assisted_teleop_active:
            return

        # Clear both global and local costmaps
        try:
            self._clear_global_costmap.call_async(ClearEntireCostmap.Request())
            self._clear_local_costmap.call_async(ClearEntireCostmap.Request())
            self.get_logger().debug('Costmaps cleared successfully')
        except rclpy.exceptions.ROSInterruptException:
            self.get_logger().error('ROSInterrupt while clearing costmaps')


def main(args=None):
    """Initialize and run the AssistedTeleopNode."""
    rclpy.init(args=args)
    node = None

    try:
        node = AssistedTeleopNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Node shutting down due to keyboard interrupt')
    except ROSInterruptException:
        if node:
            node.get_logger().info('Node shutting down due to ROS interrupt')
    finally:
        if node:
            node.cancel_assisted_teleop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
