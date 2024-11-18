/**
 * @file odometry.hpp
 * @brief Odometry calculations for mecanum drive robots
 *
 * This class handles odometry calculations for a mecanum drive robot,
 * taking into account the unique kinematics of mecanum wheels that allow
 * for omnidirectional movement.
 *
 * @author Addison Sears-Collins
 * @date November 18, 2024
 */

#ifndef MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>
#include "rclcpp/time.hpp"

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace mecanum_drive_controller
{
class Odometry
{
public:
  /**
   * @brief Constructor for mecanum drive odometry
   * @param velocity_rolling_window_size Window size for velocity measurements
   */
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /**
   * @brief Initialize odometry at the specified time
   * @param time Current time
   */
  void init(const rclcpp::Time & time);

  /**
   * @brief Update odometry from wheel positions
   * @param front_left_pos Front left wheel position
   * @param front_right_pos Front right wheel position
   * @param back_left_pos Back left wheel position
   * @param back_right_pos Back right wheel position
   * @param time Current time
   * @return True if update was successful
   */
  bool update(
    double front_left_pos, double front_right_pos,
    double back_left_pos, double back_right_pos,
    const rclcpp::Time & time);

  /**
   * @brief Update odometry from wheel velocities
   * @param front_left_vel Front left wheel velocity
   * @param front_right_vel Front right wheel velocity
   * @param back_left_vel Back left wheel velocity
   * @param back_right_vel Back right wheel velocity
   * @param time Current time
   * @return True if update was successful
   */
  bool updateFromVelocity(
    double front_left_vel, double front_right_vel,
    double back_left_vel, double back_right_vel,
    const rclcpp::Time & time);

  /**
   * @brief Update odometry using velocity commands (open loop)
   * @param linear_x Linear velocity in x direction
   * @param linear_y Linear velocity in y direction
   * @param angular Angular velocity
   * @param time Current time
   */
  void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time);

  /**
   * @brief Reset odometry to initial state
   */
  void resetOdometry();

  // Getters for odometry values
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  /**
   * @brief Set wheel parameters for odometry calculations
   * @param wheel_separation Distance between left and right wheels
   * @param wheel_base Distance between front and back wheels
   * @param front_left_wheel_radius Radius of the front left wheel
   * @param front_right_wheel_radius Radius of the front right wheel
   * @param back_left_wheel_radius Radius of the back left wheel
   * @param back_right_wheel_radius Radius of the back right wheel
   */
  void setWheelParams(
    double wheel_separation,
    double wheel_base,
    double front_left_wheel_radius,
    double front_right_wheel_radius,
    double back_left_wheel_radius,
    double back_right_wheel_radius);

  /**
   * @brief Set size of velocity rolling window
   * @param velocity_rolling_window_size Window size for velocity measurements
   */
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  // \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  /**
   * @brief Integrate motion using Runge-Kutta 2nd order method
   * @param linear_x Linear velocity in x direction
   * @param linear_y Linear velocity in y direction
   * @param angular Angular velocity
   */
  void integrateRungeKutta2(double linear_x, double linear_y, double angular);

  /**
   * @brief Integrate motion using exact method
   * @param linear_x Linear velocity in x direction
   * @param linear_y Linear velocity in y direction
   * @param angular Angular velocity
   */
  void integrateExact(double linear_x, double linear_y, double angular);

  /**
   * @brief Reset velocity accumulators
   */
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_x_;  //   [m/s]
  double linear_y_;  //   [m/s]
  double angular_;   // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;  // Distance between left and right wheels
  double wheel_base_;       // Distance between front and back wheels
  double front_left_wheel_radius_;    // Radius of front left wheel
  double front_right_wheel_radius_;   // Radius of front right wheel
  double back_left_wheel_radius_;     // Radius of back left wheel
  double back_right_wheel_radius_;    // Radius of back right wheel

  // Previous wheel position/state [rad]:
  double front_left_wheel_old_pos_;
  double front_right_wheel_old_pos_;
  double back_left_wheel_old_pos_;
  double back_right_wheel_old_pos_;

  // Rolling mean accumulators for the velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_