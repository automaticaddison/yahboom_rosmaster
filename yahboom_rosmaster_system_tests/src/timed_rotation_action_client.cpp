/**
 * @file timed_rotation_action_client.cpp
 * @brief ROS 2 action client node for sending timed rotation goals.
 *
 * This program implements a ROS 2 action client node that sends timed rotation goals
 * to a corresponding action server. It subscribes to the /timed_rotation_mode topic
 * to determine when to send or cancel goals.
 *
 * Subscription Topics:
 *   /timed_rotation_mode (std_msgs/Bool): Topic indicating whether timed rotation mode is active
 *
 * Publishing Topics:
 *   None
 *
 * Action Client:
 *   timed_rotation (yahboom_rosmaster_msgs/action/TimedRotation):
 *     Sends goals for timed rotation and receives feedback and results:
 *     - Goal: angular_velocity (rad/s) and duration (seconds)
 *     - Feedback: elapsed_time (seconds) and status string
 *     - Result: success flag and actual_duration (seconds)
 *
 * @author Addison Sears-Collins
 * @date November 25, 2024
 */

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yahboom_rosmaster_msgs/action/timed_rotation.hpp"
#include "std_msgs/msg/bool.hpp"

using TimedRotation = yahboom_rosmaster_msgs::action::TimedRotation;
using GoalHandleTimedRotation = rclcpp_action::ClientGoalHandle<TimedRotation>;

/**
 * @class TimedRotationActionClient
 * @brief ROS 2 action client node for sending timed rotation goals.
 *
 * This class implements a ROS 2 action client that sends timed rotation goals and
 * processes feedback and results from the action server. It can be controlled via
 * a boolean topic to start and stop rotations.
 */
class TimedRotationActionClient : public rclcpp::Node
{
public:
  using TimedRotation = yahboom_rosmaster_msgs::action::TimedRotation;
  using GoalHandleTimedRotation = rclcpp_action::ClientGoalHandle<TimedRotation>;

  /**
   * @brief Constructor for the TimedRotationActionClient class.
   * @param options ROS 2 node options.
   */
  explicit TimedRotationActionClient(const rclcpp::NodeOptions& options)
  : Node("timed_rotation_action_client_cpp", options)
  {
    // Create the action client
    action_client_ = rclcpp_action::create_client<TimedRotation>(
      this,
      "timed_rotation"
    );

    // Create subscription to control rotation mode
    timed_rotation_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/timed_rotation_mode",
      10,
      std::bind(&TimedRotationActionClient::timed_rotation_mode_callback, this, std::placeholders::_1)
    );

    // Declare parameters with default values
    this->declare_parameter("angular_velocity", 0.5);  // rad/s
    this->declare_parameter("duration", 10.0);         // seconds
  }

private:
  /**
   * @brief Callback function for the /timed_rotation_mode topic.
   *
   * Controls the action client based on the received boolean value:
   * - True: Starts a new rotation if none is active
   * - False: Cancels the current rotation if one is active
   *
   * @param msg The received boolean message.
   */
  void timed_rotation_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!timed_rotation_mode_ && msg->data)
    {
      // Transition from False to True - Start rotation
      send_goal();
    }
    else if (timed_rotation_mode_ && !msg->data)
    {
      // Transition from True to False - Cancel if active
      if (goal_handle_ && goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED)
      {
        cancel_goal();
      }
    }
    timed_rotation_mode_ = msg->data;
  }

  /**
   * @brief Callback function for goal response.
   * @param goal_handle The goal handle returned by the action server.
   */
  void goal_response_callback(const GoalHandleTimedRotation::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by server");
  }

  /**
   * @brief Callback function for feedback.
   * @param goal_handle The goal handle associated with the feedback.
   * @param feedback The feedback message received from the action server.
   */
  void feedback_callback(
    const GoalHandleTimedRotation::SharedPtr& goal_handle,
    const std::shared_ptr<const TimedRotation::Feedback>& feedback)
  {
    (void)goal_handle;  // Unused parameter
    RCLCPP_INFO(get_logger(), "Feedback received - Elapsed time: %.2f seconds, Status: %s",
                feedback->elapsed_time,
                feedback->status.c_str());
  }

  /**
   * @brief Callback function for getting the result.
   * @param result The result message received from the action server.
   */
  void get_result_callback(const GoalHandleTimedRotation::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded! Actual duration: %.2f seconds",
                    result.result->actual_duration);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }

  /**
   * @brief Cancel the current goal.
   *
   * Attempts to cancel the currently executing goal. Checks for a valid goal handle
   * and waits for the cancellation response from the server.
   */
  void cancel_goal()
  {
    RCLCPP_INFO(get_logger(), "Requesting to cancel goal");
    if (!goal_handle_)
    {
        RCLCPP_ERROR(get_logger(), "Goal handle is null");
        return;
    }

    auto future = action_client_->async_cancel_goal(goal_handle_);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
        return;
    }
    RCLCPP_INFO(get_logger(), "Goal cancellation request sent");
  }

  /**
   * @brief Send a goal to the action server.
   *
   * Sends a new rotation goal using parameters from the parameter server.
   * If the server isn't available, it will wait for it to become available.
   */
  void send_goal()
  {
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(get_logger(), "Action server not available after 5 seconds");
      return;
    }

    // Get parameters for the goal
    auto angular_velocity = this->get_parameter("angular_velocity").as_double();
    auto duration = this->get_parameter("duration").as_double();

    auto goal_msg = TimedRotation::Goal();
    goal_msg.angular_velocity = angular_velocity;
    goal_msg.duration = duration;

    RCLCPP_INFO(get_logger(),
                "Sending goal: angular_velocity=%.2f rad/s, duration=%.2f seconds",
                angular_velocity, duration);

    auto send_goal_options = rclcpp_action::Client<TimedRotation>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TimedRotationActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&TimedRotationActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&TimedRotationActionClient::get_result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Class member variables
  rclcpp_action::Client<TimedRotation>::SharedPtr action_client_;        ///< Action client for timed rotation
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr timed_rotation_mode_sub_;  ///< Subscription to timed rotation mode
  bool timed_rotation_mode_{false};                                      ///< Current state of timed rotation mode
  GoalHandleTimedRotation::SharedPtr goal_handle_;                      ///< Handle for the current goal
};

/**
 * @brief Main function for the timed rotation action client node.
 *
 * Initializes ROS 2, creates an instance of the TimedRotationActionClient,
 * and spins the node to process callbacks.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<TimedRotationActionClient>(rclcpp::NodeOptions());
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}