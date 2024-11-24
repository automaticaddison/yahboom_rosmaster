/**
 * @file mecanum_parameters_node.cpp
 * @brief This ROS 2 node demonstrates how to declare parameters and publish velocity messages.
 *
 * This program implements a ROS 2 node that declares parameters for a mecanum wheel robot 
 * and publishes velocity commands. It demonstrates parameter declaration, callback
 * registration for parameter changes, and publishing to the velocity topic.
 *
 * Subscription Topics:
 *   None
 *
 * Publishing Topics:
 *   /mecanum_drive_controller/cmd_vel (geometry_msgs/TwistStamped): Velocity command for the robot
 *   (linear.x, linear.y, and angular.z)
 *
 * Parameters:
 *   robot_name (string): The name of the robot
 *   robot_id (int): The unique ID number of the robot
 *   target_linear_velocity_x (double): Target linear velocity in x direction (m/s)
 *   target_linear_velocity_y (double): Target linear velocity in y direction (m/s)
 *   target_angular_velocity (double): Target angular velocity in radians per second
 *
 * @author Addison Sears-Collins
 * @date November 24, 2024
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * @class MecanumParameters
 * @brief A ROS 2 node that demonstrates parameter handling and velocity publishing.
 */
class MecanumParameters : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the MecanumParameters node.
   *
   * Initializes the node, declares parameters, sets up parameter change callback,
   * and creates a publisher and timer for velocity messages.
   */
  MecanumParameters() : Node("mecanum_parameters_node")
  {
    // Describe parameters
    rcl_interfaces::msg::ParameterDescriptor robot_name_descriptor;
    robot_name_descriptor.description = "The name of the robot";
    rcl_interfaces::msg::ParameterDescriptor robot_id_descriptor;
    robot_id_descriptor.description = "The unique ID number of the robot";
    rcl_interfaces::msg::ParameterDescriptor target_linear_velocity_x_descriptor;
    target_linear_velocity_x_descriptor.description = "Target linear velocity in x direction (meters per second)";
    rcl_interfaces::msg::ParameterDescriptor target_linear_velocity_y_descriptor;
    target_linear_velocity_y_descriptor.description = "Target linear velocity in y direction (meters per second)";
    rcl_interfaces::msg::ParameterDescriptor target_angular_velocity_descriptor;
    target_angular_velocity_descriptor.description = "Target angular velocity in radians per second";

    // Declare parameters using declare_parameter()
    this->declare_parameter("robot_name", "Automatic Addison Bot", robot_name_descriptor);
    this->declare_parameter("robot_id", 1, robot_id_descriptor);
    this->declare_parameter("target_linear_velocity_x", 0.0, target_linear_velocity_x_descriptor);
    this->declare_parameter("target_linear_velocity_y", 0.0, target_linear_velocity_y_descriptor);
    this->declare_parameter("target_angular_velocity", 0.0, target_angular_velocity_descriptor);

    // Register a callback function for parameter changes using the newer method
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MecanumParameters::parameter_change_callback, this, std::placeholders::_1));

    // Create a publisher for the /mecanum_drive_controller/cmd_vel topic
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mecanum_drive_controller/cmd_vel", 10);

    // Create a timer that will call the publish_velocity function every 0.5 seconds (2 Hz)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MecanumParameters::publish_velocity, this));
  }

private:
  /**
   * @brief Callback function for parameter changes.
   *
   * @param params A vector of rclcpp::Parameter objects representing the parameters that are
   *               being attempted to change.
   * @return rcl_interfaces::msg::SetParametersResult Object indicating whether the change was successful.
   */
  rcl_interfaces::msg::SetParametersResult parameter_change_callback(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params)
    {
      if (param.get_name() == "robot_name" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter robot_name has changed. The new value is: %s", param.as_string().c_str());
      }
      else if (param.get_name() == "robot_id" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter robot_id has changed. The new value is: %ld", param.as_int());  // Changed %d to %ld
      }
      else if (param.get_name() == "target_linear_velocity_x" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter target_linear_velocity_x has changed. The new value is: %f", param.as_double());
      }
      else if (param.get_name() == "target_linear_velocity_y" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter target_linear_velocity_y has changed. The new value is: %f", param.as_double());
      }
      else if (param.get_name() == "target_angular_velocity" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter target_angular_velocity has changed. The new value is: %f", param.as_double());
      }
      else
      {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
        break;
      }
    }
    return result;
  }

  /**
   * @brief Publishes velocity messages to the /mecanum_drive_controller/cmd_vel topic.
   *
   * This function is called periodically by the timer to publish the current
   * target linear and angular velocities.
   */
  void publish_velocity()
  {
    double target_linear_velocity_x = this->get_parameter("target_linear_velocity_x").as_double();
    double target_linear_velocity_y = this->get_parameter("target_linear_velocity_y").as_double();
    double target_angular_velocity = this->get_parameter("target_angular_velocity").as_double();

    geometry_msgs::msg::TwistStamped velocity_msg;
    velocity_msg.header.stamp = this->get_clock()->now();
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.twist.linear.x = target_linear_velocity_x;
    velocity_msg.twist.linear.y = target_linear_velocity_y;
    velocity_msg.twist.angular.z = target_angular_velocity;

    velocity_publisher_->publish(velocity_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;  ///< Publisher for velocity commands
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic velocity publishing
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  ///< Handle for the parameter callback
};

/**
 * @brief Main function to start the ROS 2 node.
 *
 * Initializes ROS 2, creates an instance of the MecanumParameters node,
 * and keeps it running to process callbacks and publish messages.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Return code (0 for normal exit).
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto mecanum_parameters_node = std::make_shared<MecanumParameters>();
  rclcpp::spin(mecanum_parameters_node);
  rclcpp::shutdown();
  return 0;
}
