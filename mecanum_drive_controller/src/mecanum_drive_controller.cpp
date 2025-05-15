/**
 * @file mecanum_drive_controller.cpp
 * @brief Implementation of the ROS 2 controller for mecanum drive mobile robots
 *
 * This controller implements the kinematics of a mecanum drive robot, converting velocity
 * commands into individual wheel velocities while handling different wheel radii and
 * providing odometry information.
 *
 * Inverse Kinematics Equations (all angular velocities in radians/second):
 * ω1 = 1/R1 * (vx - vy - ωz(L + W))  // front left
 * ω2 = 1/R2 * (vx + vy + ωz(L + W))  // front right
 * ω3 = 1/R3 * (vx + vy - ωz(L + W))  // back left
 * ω4 = 1/R4 * (vx - vy + ωz(L + W))  // back right
 *
 * Where:
 * R1,R2,R3,R4 = individual wheel radii (meters)
 * L = distance from robot center to front/back wheels (wheel_base/2) (meters)
 * W = distance from robot center to left/right wheels (wheel_separation/2) (meters)
 * vx = linear x velocity (m/s)
 * vy = linear y velocity (m/s)
 * ωz = angular z velocity (rad/s)
 *
 * Subscription Topics:
 *     ~/cmd_vel (geometry_msgs/msg/TwistStamped): Velocity commands for the robot
 *
 * Publishing Topics:
 *     ~/odom (nav_msgs/msg/Odometry): Odometry information from wheel encoders
 *     /tf (tf2_msgs/msg/TFMessage): Transform between odom and base_link frames
 *     ~/cmd_vel_out (geometry_msgs/msg/TwistStamped): Limited velocity commands (if enabled)
 *
 * @author Addison Sears-Collins
 * @date May 15, 2025
 */

#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace mecanum_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MecanumDriveController::MecanumDriveController()
: controller_interface::ControllerInterface()
{
}

const char * MecanumDriveController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.reserve(4);  // Reserve space for all four wheels

  // Add velocity command interfaces for each wheel
  conf_names.push_back(params_.front_left_joint_name + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.front_right_joint_name + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.back_left_joint_name + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.back_right_joint_name + "/" + HW_IF_VELOCITY);

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration MecanumDriveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.reserve(4);  // Reserve space for all four wheels

  // Add state interfaces for each wheel
  conf_names.push_back(params_.front_left_joint_name + "/" + feedback_type());
  conf_names.push_back(params_.front_right_joint_name + "/" + feedback_type());
  conf_names.push_back(params_.back_left_joint_name + "/" + feedback_type());
  conf_names.push_back(params_.back_right_joint_name + "/" + feedback_type());

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type MecanumDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(
    [&](const std::shared_ptr<Twist> & msg) {
    last_command_msg = msg;
  });

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // Command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_command_x = command.twist.linear.x;
  double & linear_command_y = command.twist.linear.y;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheel_base = params_.wheel_base;
  const double front_left_radius =
    params_.front_left_wheel_radius_multiplier * params_.wheel_radius;
  const double front_right_radius =
    params_.front_right_wheel_radius_multiplier * params_.wheel_radius;
  const double back_left_radius =
    params_.back_left_wheel_radius_multiplier * params_.wheel_radius;
  const double back_right_radius =
    params_.back_right_wheel_radius_multiplier * params_.wheel_radius;

  // Calculate distances from center to wheels
  const double L = wheel_base / 2.0;      // Distance to front/back wheels
  const double W = wheel_separation / 2.0; // Distance to left/right wheels

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command_x, linear_command_y, angular_command, time);
  }
  else
  {
    if (params_.position_feedback)
    {
      if (wheel_handles_[FRONT_LEFT].feedback && wheel_handles_[FRONT_RIGHT].feedback &&
        wheel_handles_[BACK_LEFT].feedback && wheel_handles_[BACK_RIGHT].feedback)
      {
        odometry_.update(
          wheel_handles_[FRONT_LEFT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[FRONT_RIGHT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[BACK_LEFT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[BACK_RIGHT].feedback->get().get_optional().value_or(0.0),
          time);
      }
      else
      {
        RCLCPP_ERROR(logger, "Wheel feedback handles are not available");
        return controller_interface::return_type::ERROR;
      }
    }
    else
    {
      if (wheel_handles_[FRONT_LEFT].feedback && wheel_handles_[FRONT_RIGHT].feedback &&
          wheel_handles_[BACK_LEFT].feedback && wheel_handles_[BACK_RIGHT].feedback)
      {
        odometry_.updateFromVelocity(
          wheel_handles_[FRONT_LEFT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[FRONT_RIGHT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[BACK_LEFT].feedback->get().get_optional().value_or(0.0),
          wheel_handles_[BACK_RIGHT].feedback->get().get_optional().value_or(0.0),
          time);
      }
      else
      {
        RCLCPP_ERROR(logger, "Wheel feedback handles are not available");
        return controller_interface::return_type::ERROR;
      }
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  // Apply speed limiters
  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;

  limiter_linear_x_.limit(
    linear_command_x, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_linear_y_.limit(
    linear_command_y, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  // Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheel velocities using inverse kinematics
  // ω1 = 1/R1 * (vx - vy - ωz(L + W))  // front left
  const double front_left_velocity = (
    linear_command_x - linear_command_y - angular_command * (L + W)) / front_left_radius;

  // ω2 = 1/R2 * (vx + vy + ωz(L + W))  // front right
  const double front_right_velocity = (
    linear_command_x + linear_command_y + angular_command * (L + W)) / front_right_radius;

  // ω3 = 1/R3 * (vx + vy - ωz(L + W))  // back left
  const double back_left_velocity = (
    linear_command_x + linear_command_y - angular_command * (L + W)) / back_left_radius;

  // ω4 = 1/R4 * (vx - vy + ωz(L + W))  // back right
  const double back_right_velocity = (
    linear_command_x - linear_command_y + angular_command * (L + W)) / back_right_radius;

  // Set wheel velocities using wheel indices
  if (wheel_handles_[FRONT_LEFT].velocity) {
    if (!wheel_handles_[FRONT_LEFT].velocity.value().get().set_value(front_left_velocity)) {
      RCLCPP_ERROR(logger, "Failed to set front left wheel velocity");
    }
  } else {
    RCLCPP_ERROR(logger, "Front left velocity handle is not available");
  }

  if (wheel_handles_[FRONT_RIGHT].velocity) {
    if (!wheel_handles_[FRONT_RIGHT].velocity.value().get().set_value(front_right_velocity)) {
      RCLCPP_ERROR(logger, "Failed to set front right wheel velocity");
    }
  } else {
    RCLCPP_ERROR(logger, "Front right velocity handle is not available");
  }

  if (wheel_handles_[BACK_LEFT].velocity) {
    if (!wheel_handles_[BACK_LEFT].velocity.value().get().set_value(back_left_velocity)) {
      RCLCPP_ERROR(logger, "Failed to set back left wheel velocity");
    }
  } else {
    RCLCPP_ERROR(logger, "Back left velocity handle is not available");
  }

  if (wheel_handles_[BACK_RIGHT].velocity) {
    if (!wheel_handles_[BACK_RIGHT].velocity.value().get().set_value(back_right_velocity)) {
      RCLCPP_ERROR(logger, "Failed to set back right wheel velocity");
    }
  } else {
    RCLCPP_ERROR(logger, "Back right velocity handle is not available");
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // Update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  // Check wheel radius parameter
  if (params_.wheel_radius <= 0.0)
  {
    RCLCPP_ERROR(logger, "Wheel radius must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double wheel_base = params_.wheel_base;
  const double front_left_radius =
    params_.front_left_wheel_radius_multiplier * params_.wheel_radius;
  const double front_right_radius =
    params_.front_right_wheel_radius_multiplier * params_.wheel_radius;
  const double back_left_radius =
    params_.back_left_wheel_radius_multiplier * params_.wheel_radius;
  const double back_right_radius =
    params_.back_right_wheel_radius_multiplier * params_.wheel_radius;

  odometry_.setWheelParams(
    wheel_separation,
    wheel_base,
    front_left_radius,
    front_right_radius,
    back_left_radius,
    back_right_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;

  // Setup speed limiters
  limiter_linear_x_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_linear_y_ = SpeedLimiter(
    params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
    params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.min_acceleration, params_.linear.y.max_acceleration, params_.linear.y.min_jerk,
    params_.linear.y.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(
    [&](std::shared_ptr<Twist> & msg) {
      msg = std::make_shared<Twist>();
    });

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // Initialize command subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
    DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Twist> msg) -> void {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }
      received_velocity_msg_ptr_.set(
        [&](std::shared_ptr<Twist> & stored_msg) {
        stored_msg = std::move(msg);
        });

    });

  // Initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // Limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // Initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // Initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // Keep track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // Configure each wheel using the WheelIndex enum
  if (configure_wheel(params_.front_left_joint_name, FRONT_LEFT) ==
    controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (configure_wheel(params_.front_right_joint_name, FRONT_RIGHT) ==
    controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (configure_wheel(params_.back_left_joint_name, BACK_LEFT) ==
    controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (configure_wheel(params_.back_right_joint_name, BACK_RIGHT) ==
    controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(logger, "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }

  // Reset wheel handles using WheelIndex
  for (size_t i = 0; i < wheel_handles_.size(); ++i)
  {
    wheel_handles_[i] = WheelHandle();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set([](std::shared_ptr<Twist> & stored_msg) {
    stored_msg = std::make_shared<Twist>();
  });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumDriveController::halt()
{
  for (auto & wheel_handle : wheel_handles_)
  {
    if (wheel_handle.velocity) {
      if (!wheel_handle.velocity->get().set_value(0.0)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to halt wheel velocity");
      }
    }
  }
}

bool MecanumDriveController::reset()
{
  odometry_.resetOdometry();

  // Release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  // Reset all wheel handles
  wheel_handles_ = std::array<WheelHandle, 4>();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn MecanumDriveController::configure_wheel(
  const std::string & wheel_name,
  size_t wheel_index)
{
  auto logger = get_node()->get_logger();

  // Get wheel state handle
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&wheel_name, this](const auto & interface)
    {
      return interface.get_prefix_name() == wheel_name &&
             interface.get_interface_name() == feedback_type();
    });

  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain state handle for %s", wheel_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get wheel command handle
  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&wheel_name](const auto & interface)
    {
      return interface.get_prefix_name() == wheel_name &&
             interface.get_interface_name() == HW_IF_VELOCITY;
    });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain command handle for %s", wheel_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Store the wheel handle at the specified index
  wheel_handles_[wheel_index] = WheelHandle(*state_handle, *command_handle);
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace mecanum_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  mecanum_drive_controller::MecanumDriveController,
  controller_interface::ControllerInterface)
