/**
 * @file set_cleaning_state_service.cpp
 * @brief ROS 2 node that implements a service to set the cleaning state of a robot
 *
 * This program implements a ROS 2 node that provides a service to set the cleaning state
 * of a robot. It demonstrates the use of service servers in ROS 2.
 *
 * Subscription Topics:
 *   None
 *
 * Publishing Topics:
 *   None
 *
 * Services:
 *   /set_cleaning_state (yahboom_rosmaster_msgs/srv/SetCleaningState):
 *     Sets the cleaning state of the robot
 *
 * @author Addison Sears-Collins
 * @date November 26, 2024
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "yahboom_rosmaster_msgs/srv/set_cleaning_state.hpp"

/**
 * @class SetCleaningStateService
 * @brief A ROS 2 node that provides a service to set the cleaning state of a robot
 */
class SetCleaningStateService : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the SetCleaningStateService class.
   *
   * Initializes the node with the name "set_cleaning_state_service"
   * and creates the service server for the "/set_cleaning_state" service.
   */
  SetCleaningStateService()
  : Node("set_cleaning_state_service")
  {
    service_ = this->create_service<yahboom_rosmaster_msgs::srv::SetCleaningState>(
      "/set_cleaning_state",
      std::bind(&SetCleaningStateService::set_cleaning_state_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Cleaning Service Server is ready.");
  }

private:
  /**
   * @brief Callback function for the set_cleaning_state service.
   *
   * This function is called when a request is received for the "/set_cleaning_state" service.
   * It checks the desired cleaning state in the request and sets the appropriate response.
   *
   * @param request The request shared pointer for the set_cleaning_state service.
   * @param response The response shared pointer for the set_cleaning_state service.
   */
  void set_cleaning_state_callback(
    const std::shared_ptr<yahboom_rosmaster_msgs::srv::SetCleaningState::Request> request,
    std::shared_ptr<yahboom_rosmaster_msgs::srv::SetCleaningState::Response> response)
  {
    if (request->desired_cleaning_state) {
      RCLCPP_INFO(this->get_logger(), "Starting cleaning...");
      response->success = true;
      response->message = "Robot started cleaning";
    } else {
      RCLCPP_INFO(this->get_logger(), "Stopping cleaning...");
      response->success = true;
      response->message = "Robot stopped cleaning";
    }

    RCLCPP_INFO(this->get_logger(), "Response sent: %s", response->message.c_str());
  }

  /// Service server for setting cleaning state
  rclcpp::Service<yahboom_rosmaster_msgs::srv::SetCleaningState>::SharedPtr service_;
};

/**
 * @brief Main function to run the SetCleaningStateService node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetCleaningStateService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}