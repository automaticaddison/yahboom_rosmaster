/**
 * @file square_mecanum_controller.cpp
 * @brief Controls a mecanum wheeled robot to move in a square pattern
 *
 * This program creates a ROS 2 node that publishes velocity commands to make a
 * mecanum wheeled robot move in a square pattern. It takes advantage of the
 * omnidirectional capabilities of mecanum wheels to move in straight lines
 * along both X and Y axes.
 *
 * Publishing Topics:
 *     /mecanum_drive_controller/cmd_vel (geometry_msgs/TwistStamped):
 *         Velocity commands for the robot's motion
 *
 * @author Addison Sears-Collins
 * @date November 22, 2024
 */

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class SquareController : public rclcpp::Node
{
public:
    SquareController() : Node("square_controller")
    {
        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mecanum_drive_controller/cmd_vel", 10);

        // Create timer that calls our control function every 200ms
        timer_ = this->create_wall_timer(
            200ms, std::bind(&SquareController::timer_callback, this));

        // Initialize variables
        current_side_ = 0;  // Which side of square we're on (0-3)
        elapsed_time_ = 0.0;  // Time spent on current side
        robot_speed_ = 0.2;  // Speed in meters/second
        side_length_ = 2.0;  // Length of each side in meters
        time_per_side_ = side_length_ / robot_speed_;  // Time to complete one side
    }

    // Send stop command when node is shut down
    ~SquareController()
    {
        stop_robot();
    }

    // Function to stop the robot
    void stop_robot()
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        publisher_->publish(msg);  // All velocities default to 0
    }

private:
    void timer_callback()
    {
        // Create velocity command message
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();

        // Set velocity based on which side of the square we're on
        switch (current_side_) {
            case 0:  // Moving forward (positive X)
                msg.twist.linear.x = robot_speed_;
                break;
            case 1:  // Moving left (positive Y)
                msg.twist.linear.y = robot_speed_;
                break;
            case 2:  // Moving backward (negative X)
                msg.twist.linear.x = -robot_speed_;
                break;
            case 3:  // Moving right (negative Y)
                msg.twist.linear.y = -robot_speed_;
                break;
        }

        // Publish the velocity command
        publisher_->publish(msg);

        // Update time tracking
        elapsed_time_ += 0.2;  // 200ms in seconds

        // Check if we've completed the current side
        if (elapsed_time_ >= time_per_side_) {
            elapsed_time_ = 0.0;
            current_side_ = (current_side_ + 1) % 4;  // Move to next side (0-3)
        }
    }

    // Class member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    int current_side_;
    double elapsed_time_;
    double side_length_;
    double robot_speed_;
    double time_per_side_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
