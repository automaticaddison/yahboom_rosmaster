/**
 * @file timed_rotation_action_server.cpp
 * @brief This ROS 2 node implements an action server for the TimedRotation action.
 *
 * This program implements a ROS 2 action server that executes timed rotation actions for the Yahboom RosMaster robot.
 * It receives goals from action clients, publishes timestamped velocity commands to rotate the robot,
 * and provides feedback on the rotation progress.
 *
 * Subscription Topics:
 *   None
 *
 * Publishing Topics:
 *   /mecanum_drive_controller/cmd_vel (geometry_msgs/TwistStamped):
 *     Timestamped velocity command for the robot:
 *     - header.stamp: Current ROS time when command is published
 *     - header.frame_id: "base_link" (robot's base frame)
 *     - twist.linear.x: Always 0 (no forward/backward motion)
 *     - twist.linear.y: Always 0 (no left/right motion)
 *     - twist.angular.z: Rotation speed in radians/second (positive=counterclockwise, negative=clockwise)
 *
 * Action Server:
 *   timed_rotation (yahboom_rosmaster_msgs/action/TimedRotation):
 *     Executes timed rotation actions with the following:
 *     - Goal: angular_velocity (rad/s) and duration (seconds)
 *     - Feedback: elapsed_time and status updates
 *     - Result: success flag and actual_duration
 *
 * @author Addison Sears-Collins
 * @date November 25, 2024
 */

#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yahboom_rosmaster_msgs/action/timed_rotation.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using TimedRotation = yahboom_rosmaster_msgs::action::TimedRotation;
using GoalHandleTimedRotation = rclcpp_action::ServerGoalHandle<TimedRotation>;

/**
 * @class TimedRotationActionServer
 * @brief ROS 2 action server for the TimedRotation action.
 *
 * This class implements the action server functionality for the TimedRotation action.
 * It receives goals from an action client, executes the timed rotation, provides feedback,
 * and handles goal cancellation requests. The server maintains rotation by continuously
 * publishing timestamped velocity commands until the specified duration is reached or the goal is canceled.
 */
class TimedRotationActionServer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the TimedRotationActionServer class.
   *
   * Initializes the action server and the timestamped velocity command publisher.
   * Sets up callback bindings for handling goals, cancellation requests,
   * and accepted goals.
   */
  TimedRotationActionServer()
  : Node("timed_rotation_action_server_cpp")
  {
    using namespace std::placeholders;

    // Create the action server with callbacks for goal handling, cancellation, and execution
    action_server_ = rclcpp_action::create_server<TimedRotation>(
      this,
      "timed_rotation",
      std::bind(&TimedRotationActionServer::handle_goal, this, _1, _2),
      std::bind(&TimedRotationActionServer::handle_cancel, this, _1),
      std::bind(&TimedRotationActionServer::handle_accepted, this, _1));

    // Create publisher for timestamped velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mecanum_drive_controller/cmd_vel",
      10);  // Queue size of 10 messages
  }

private:
  // Class member variables
  rclcpp_action::Server<TimedRotation>::SharedPtr action_server_;  ///< The action server instance
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;  ///< Publisher for timestamped velocity commands

  /**
   * @brief Callback function for handling new goal requests.
   *
   * This function is called when a new goal is received from an action client.
   * Currently accepts all goals without any validation.
   *
   * @param uuid Unique identifier for the goal request.
   * @param goal The goal request message containing angular_velocity and duration.
   * @return rclcpp_action::GoalResponse indicating if the goal is accepted or rejected.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    [[maybe_unused]] std::shared_ptr<const TimedRotation::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request for a timed rotation action");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Callback function for handling cancel requests.
   *
   * This function is called when a client requests to cancel an ongoing goal.
   * Currently accepts all cancellation requests.
   *
   * @param goal_handle The goal handle associated with the goal to be canceled.
   * @return rclcpp_action::CancelResponse indicating if the cancellation is accepted.
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTimedRotation> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for timed rotation action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Callback function for handling accepted goals.
   *
   * This function is called when a goal has been accepted and needs to be executed.
   * Spawns a new thread to execute the goal to avoid blocking the main thread.
   *
   * @param goal_handle The goal handle associated with the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTimedRotation> goal_handle)
  {
    // Create a new thread using proper bind syntax
    std::thread{
      [this, goal_handle]() {
        this->execute(goal_handle);
      }
    }.detach();
  }

  /**
   * @brief Executes the timed rotation action.
   *
   * This function runs in a separate thread and performs the following:
   * 1. Publishes initial feedback with GOAL_RECEIVED status
   * 2. Creates and sends timestamped velocity commands based on goal parameters
   * 3. Continuously publishes feedback during rotation
   * 4. Handles cancellation requests during execution
   * 5. Stops rotation and publishes final result
   *
   * @param goal_handle The goal handle associated with the goal to be executed.
   */
  void execute(const std::shared_ptr<GoalHandleTimedRotation> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal for timed rotation action...");

    // Get the goal message
    const auto goal = goal_handle->get_goal();

    // Create result and feedback messages
    auto result = std::make_shared<TimedRotation::Result>();
    auto feedback = std::make_shared<TimedRotation::Feedback>();

    // Send initial feedback
    feedback->status = "GOAL_RECEIVED";
    goal_handle->publish_feedback(feedback);

    // Create timestamped velocity command message
    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    twist_stamped_msg.twist.angular.z = goal->angular_velocity;  // Set rotation speed
    twist_stamped_msg.header.frame_id = "base_link";  // Set the reference frame
    // linear.x and linear.y are automatically initialized to 0

    // Record start time for duration tracking
    auto start_time = this->now();

    // Main execution loop
    while (rclcpp::ok())
    {
      // Check for cancellation requests
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        feedback->status = "GOAL_CANCELED";
        goal_handle->publish_feedback(feedback);

        // Stop the robot with timestamped command
        twist_stamped_msg.twist.angular.z = 0.0;
        twist_stamped_msg.header.stamp = this->now();
        cmd_vel_publisher_->publish(twist_stamped_msg);
        return;
      }

      // Update timestamp and publish velocity command
      twist_stamped_msg.header.stamp = this->now();
      cmd_vel_publisher_->publish(twist_stamped_msg);

      // Calculate elapsed time
      auto elapsed_time = (this->now() - start_time).seconds();

      // Publish feedback
      feedback->elapsed_time = elapsed_time;
      feedback->status = "ROTATING";
      goal_handle->publish_feedback(feedback);

      // Check if we've reached the desired duration
      if (elapsed_time >= goal->duration)
      {
        break;
      }

      // Sleep to control update rate (5Hz)
      std::this_thread::sleep_for(200ms);
    }

    // Stop the robot with final timestamped command
    twist_stamped_msg.twist.angular.z = 0.0;
    twist_stamped_msg.header.stamp = this->now();
    cmd_vel_publisher_->publish(twist_stamped_msg);

    // Set and send result
    result->success = true;
    result->actual_duration = (this->now() - start_time).seconds();

    // Mark the goal as succeeded
    goal_handle->succeed(result);

    // Send final feedback
    feedback->status = "GOAL_SUCCEEDED";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

/**
 * @brief Main function for the timed rotation action server node.
 *
 * Initializes ROS 2, creates an instance of the TimedRotationActionServer,
 * and spins the node to process callbacks.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<TimedRotationActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}