/**
 * @file mecanum_drive_controller.hpp
 * @brief ROS 2 controller for mecanum drive mobile robots
 *
 * This controller handles the kinematics of a mecanum drive robot with four mecanum wheels.
 * It subscribes to velocity commands and converts them into individual wheel velocities,
 * taking into account the unique properties of mecanum wheels that allow omnidirectional movement.
 *
 * @author Addison Sears-Collins
 * @date May 15, 2025
 */

#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <vector>
#include <array>

// ROS 2 Controller interface
#include "controller_interface/controller_interface.hpp"
// Custom headers for mecanum drive functionality
#include "mecanum_drive_controller/odometry.hpp"
#include "mecanum_drive_controller/speed_limiter.hpp"
#include "mecanum_drive_controller/visibility_control.h"
// ROS 2 message types
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
// ROS 2 lifecycle and utilities
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// Auto-generated parameters
#include <mecanum_drive_controller/mecanum_drive_controller_parameters.hpp>

namespace mecanum_drive_controller
{

// Wheel position indices
enum WheelIndex : size_t
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  BACK_LEFT = 2,
  BACK_RIGHT = 3
};

/**
 * @brief Controller class for mecanum drive robots
 *
 * This class implements a controller for robots with mecanum wheels, allowing for
 * omnidirectional movement including lateral motion. It handles the conversion
 * between velocity commands and individual wheel velocities.
 */
class MecanumDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  /**
   * @brief Constructor for the mecanum drive controller
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  MecanumDriveController();

  /**
   * @brief Get the command interface configuration
   * @return Interface configuration for the controller's command interfaces
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Get the state interface configuration
   * @return Interface configuration for the controller's state interfaces
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Update function called periodically
   * @param time Current time
   * @param period Time since the last update
   * @return Success/failure of update
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Initialization function for the controller
   * @return Success/failure of initialization
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configuration function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of configuration
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activation function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of activation
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivation function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of deactivation
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of cleanup
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Error handling function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of error handling
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Shutdown function for the controller
   * @param previous_state Previous state of the controller
   * @return Success/failure of shutdown
   */
  MECANUM_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
   * @brief Structure to hold wheel feedback and command interfaces
   */
  struct WheelHandle
  {
    std::optional<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> feedback{};
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity{};

    WheelHandle() = default;

    WheelHandle(
      const hardware_interface::LoanedStateInterface & feedback_handle,
      hardware_interface::LoanedCommandInterface & velocity_handle)
      : feedback(std::cref(feedback_handle)),
      velocity(std::ref(velocity_handle))
   {
   }
};

  /**
   * @brief Get the feedback type for the wheels
   * @return String describing the feedback type
   */
  const char * feedback_type() const;

  /**
   * @brief Configure a single wheel
   * @param wheel_name Name of the wheel to configure
   * @param wheel_index Index of the wheel in the handles array
   * @return Success/failure of wheel configuration
   */
  controller_interface::CallbackReturn configure_wheel(
    const std::string & wheel_name,
    size_t wheel_index);

  // Fixed array of 4 wheel handles for the mecanum drive
  std::array<WheelHandle, 4> wheel_handles_;  // Handles for each wheel

  // ROS 2 parameter handling
  std::shared_ptr<ParamListener> param_listener_;  // Listens for parameter updates
  Params params_;                                  // Stores current parameters

  Odometry odometry_;  // Handles odometry calculations

  // Command timeout handling
  std::chrono::milliseconds cmd_vel_timeout_{500};  // Timeout for velocity commands

  // Publishers for odometry data
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  // Publishers for transform data
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Subscriber for velocity commands
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

  // Storage for received velocity commands
  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
  std::queue<Twist> previous_commands_;  // Stores last two commands

  // Speed limiters for each motion direction
  SpeedLimiter limiter_linear_x_;    // Limits forward/backward speed
  SpeedLimiter limiter_linear_y_;    // Limits left/right speed
  SpeedLimiter limiter_angular_;     // Limits rotational speed

  // Limited velocity command publishing
  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  // Timing management
  rclcpp::Time previous_update_timestamp_{0};
  double publish_rate_ = 50.0;  // Publishing rate in Hz
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  // Controller state
  bool is_halted = false;  // Tracks if controller is halted

  /**
   * @brief Reset the controller to its initial state
   * @return Success/failure of reset
   */
  bool reset();

  /**
   * @brief Halt the controller's operation
   */
  void halt();
};
}  // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
