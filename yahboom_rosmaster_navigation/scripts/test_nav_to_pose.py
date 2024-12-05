#!/usr/bin/env python3
"""
ROS 2 node for sending a robot to different tables based on user input.

This script creates a ROS 2 node that continuously prompts the user to select a table
number (1-5) in the cafe world and publishes the corresponding goal pose for the
robot to navigate to. The script will keep running until the user enters 'q' or presses Ctrl+C.

Publishing Topics:
    /goal_pose/goal (geometry_msgs/PoseStamped): The desired goal pose for the robot

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yahboom_rosmaster_navigation.posestamped_msg_generator import PoseStampedGenerator


class GoalPublisher(Node):
    """
    ROS 2 node for publishing goal poses to different tables.

    This node allows users to input table numbers and publishes the corresponding
    goal poses for robot navigation.

    Attributes:
        publisher: Publisher object for goal poses
        locations (dict): Dictionary containing x,y coordinates for each table
        pose_generator: Generator for PoseStamped messages
    """

    def __init__(self):
        """
        Initialize the GoalPublisher node.

        Sets up the publisher for goal poses and defines the table locations.
        """
        super().__init__('goal_publisher')

        # Create a publisher that will send goal poses to the robot
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose/goal', 10)
        self.get_logger().info('Goal Publisher node has been initialized')

        # Initialize the PoseStamped message generator
        self.pose_generator = PoseStampedGenerator('pose_generator')

        # Dictionary storing the x,y coordinates for each table
        self.locations = {
            '1': {'x': -0.96, 'y': -0.92},   # Table 1 location
            '2': {'x': 1.16, 'y': -4.23},    # Table 2 location
            '3': {'x': 0.792, 'y': -8.27},   # Table 3 location
            '4': {'x': -3.12, 'y': -7.495},  # Table 4 location
            '5': {'x': -2.45, 'y': -3.55}    # Table 5 location
        }

    def publish_goal(self, table_number):
        """
        Publish a goal pose for the selected table.

        Creates and publishes a PoseStamped message with the coordinates
        corresponding to the selected table number.

        Args:
            table_number (str): The selected table number ('1' through '5')
        """
        # Get the coordinates for the selected table
        x = self.locations[table_number]['x']
        y = self.locations[table_number]['y']

        # Create the PoseStamped message using the generator
        # Using default orientation (facing forward) and z=0.0
        pose_msg = self.pose_generator.create_pose_stamped(
            x=x,
            y=y,
            z=0.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            frame_id='map'
        )

        # Publish the goal pose
        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Goal pose published for table {table_number}')

    def run_interface(self):
        """
        Run the user interface for table selection.

        Continuously prompts the user to select a table number and publishes
        the corresponding goal pose. Exits when user enters 'q' or Ctrl+C.
        """
        while True:
            # Display available tables
            print("\nAVAILABLE TABLES:")
            for table in self.locations:
                print(f"Table {table}")

            # Get user input
            user_input = input('\nEnter table number (1-5) or "q" to quit: ').strip()

            # Check if user wants to quit
            if user_input.lower() == 'q':
                break

            # Validate and process user input
            if user_input in self.locations:
                self.publish_goal(user_input)
            else:
                print("Invalid table number! Please enter a number between 1 and 5.")


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.

    Args:
        args: Command-line arguments (default: None)
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and run the node
    goal_publisher = GoalPublisher()

    try:
        goal_publisher.run_interface()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
