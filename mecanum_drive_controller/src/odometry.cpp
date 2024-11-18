/**
 * @file odometry.cpp
 * @brief Implementation of odometry calculations for mecanum drive robots
 *
 * This implementation handles the unique kinematics of mecanum wheels to track
 * the robot's position and velocity in 2D space, including holonomic movement.
 *
 * Forward Kinematics Equations (all angular velocities in radians/second):
 * linear_x (m/s) = 1/4 * (R1*w1 + R2*w2 + R3*w3 + R4*w4)
 * linear_y (m/s) = 1/4 * (-R1*w1 + R2*w2 + R3*w3 - R4*w4)
 * angular (rad/s) = 1/(4*(L+W)) * (-R1*w1 + R2*w2 - R3*w3 + R4*w4)
 *
 * Where:
 * R1,R2,R3,R4 = individual wheel radii (meters)
 * L = distance from robot center to front/back wheels (meters)
 * W = distance from robot center to left/right wheels (meters)
 * w1,w2,w3,w4 = angular velocities of wheels (radians/second)
 *
 * @author Addison Sears-Collins
 * @date November 18, 2024
 */

#include "mecanum_drive_controller/odometry.hpp"

namespace mecanum_drive_controller
{

/**
 * @brief Constructor for Odometry class
 * @param velocity_rolling_window_size Size of window for velocity averaging
 */
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),           // Robot x position in meters
  y_(0.0),           // Robot y position in meters
  heading_(0.0),     // Robot heading in radians
  linear_x_(0.0),    // Forward/backward velocity in m/s
  linear_y_(0.0),    // Left/right velocity in m/s
  angular_(0.0),     // Angular velocity in rad/s
  wheel_separation_(0.0),  // Distance between left and right wheels (meters)
  wheel_base_(0.0),       // Distance between front and back wheels (meters)
  // Individual wheel radii in meters
  front_left_wheel_radius_(0.0),
  front_right_wheel_radius_(0.0),
  back_left_wheel_radius_(0.0),
  back_right_wheel_radius_(0.0),
  // Previous wheel positions in radians
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  back_left_wheel_old_pos_(0.0),
  back_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  // Initialize rolling average accumulators for each velocity component
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

/**
 * @brief Initialize odometry with starting timestamp
 * @param time Current ROS time
 */
void Odometry::init(const rclcpp::Time & time)
{
  // Reset all velocity accumulators and set initial timestamp
  resetAccumulators();
  timestamp_ = time;
}

/**
 * @brief Update odometry using wheel position measurements
 * @param front_left_pos Front left wheel position (radians)
 * @param front_right_pos Front right wheel position (radians)
 * @param back_left_pos Back left wheel position (radians)
 * @param back_right_pos Back right wheel position (radians)
 * @param time Current ROS time
 * @return true if update successful, false if time interval too small
 */
bool Odometry::update(
  double front_left_pos, double front_right_pos,
  double back_left_pos, double back_right_pos,
  const rclcpp::Time & time)
{
  // Calculate time difference since last update
  const double dt = time.seconds() - timestamp_.seconds();

  // Need reasonable time interval for velocity calculation
  if (dt < 0.0001)
  {
    return false;
  }

  // Store current wheel positions (in radians)
  const double front_left_wheel_cur_pos = front_left_pos;
  const double front_right_wheel_cur_pos = front_right_pos;
  const double back_left_wheel_cur_pos = back_left_pos;
  const double back_right_wheel_cur_pos = back_right_pos;

  // Calculate wheel velocities (rad/s) using position difference
  const double front_left_wheel_est_vel =
    (front_left_wheel_cur_pos - front_left_wheel_old_pos_) / dt;
  const double front_right_wheel_est_vel =
    (front_right_wheel_cur_pos - front_right_wheel_old_pos_) / dt;
  const double back_left_wheel_est_vel =
    (back_left_wheel_cur_pos - back_left_wheel_old_pos_) / dt;
  const double back_right_wheel_est_vel =
    (back_right_wheel_cur_pos - back_right_wheel_old_pos_) / dt;

  // Save current positions for next update
  front_left_wheel_old_pos_ = front_left_wheel_cur_pos;
  front_right_wheel_old_pos_ = front_right_wheel_cur_pos;
  back_left_wheel_old_pos_ = back_left_wheel_cur_pos;
  back_right_wheel_old_pos_ = back_right_wheel_cur_pos;

  // Update odometry using calculated velocities
  updateFromVelocity(
    front_left_wheel_est_vel, front_right_wheel_est_vel,
    back_left_wheel_est_vel, back_right_wheel_est_vel,
    time);

  return true;
}

/**
 * @brief Update odometry using wheel velocity measurements
 * @param front_left_vel Front left wheel velocity (rad/s)
 * @param front_right_vel Front right wheel velocity (rad/s)
 * @param back_left_vel Back left wheel velocity (rad/s)
 * @param back_right_vel Back right wheel velocity (rad/s)
 * @param time Current ROS time
 * @return true if update successful
 */
bool Odometry::updateFromVelocity(
  double front_left_vel, double front_right_vel,
  double back_left_vel, double back_right_vel,
  const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Calculate distances from robot center to wheels (meters)
  const double L = wheel_base_ / 2.0;      // Distance to front/back wheels
  const double W = wheel_separation_ / 2.0; // Distance to left/right wheels

  // Calculate robot velocity components using mecanum wheel kinematics
  // Each wheel contributes to motion in all directions based on its orientation

  // Forward/backward velocity (m/s)
  const double linear_x = (front_left_wheel_radius_ * front_left_vel +
                          front_right_wheel_radius_ * front_right_vel +
                          back_left_wheel_radius_ * back_left_vel +
                          back_right_wheel_radius_ * back_right_vel) * 0.25;

  // Left/right velocity (m/s)
  const double linear_y = (-front_left_wheel_radius_ * front_left_vel +
                          front_right_wheel_radius_ * front_right_vel +
                          back_left_wheel_radius_ * back_left_vel -
                          back_right_wheel_radius_ * back_right_vel) * 0.25;

  // Angular velocity (rad/s)
  const double angular = (-front_left_wheel_radius_ * front_left_vel +
                         front_right_wheel_radius_ * front_right_vel -
                         back_left_wheel_radius_ * back_left_vel +
                         back_right_wheel_radius_ * back_right_vel) /
                        (4.0 * (L + W));

  // Update robot's position and orientation
  integrateExact(linear_x * dt, linear_y * dt, angular * dt);

  timestamp_ = time;

  // Update velocity estimates using rolling mean for smoothing
  linear_x_accumulator_.accumulate(linear_x);
  linear_y_accumulator_.accumulate(linear_y);
  angular_accumulator_.accumulate(angular);

  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

/**
 * @brief Update odometry using direct velocity commands (open loop)
 * @param linear_x Forward/backward velocity (m/s)
 * @param linear_y Left/right velocity (m/s)
 * @param angular Angular velocity (rad/s)
 * @param time Current ROS time
 */
void Odometry::updateOpenLoop(
  double linear_x, double linear_y, double angular,
  const rclcpp::Time & time)
{
  // Store current velocities
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  // Calculate position change over time interval
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;

  // Update position and orientation
  integrateExact(linear_x * dt, linear_y * dt, angular * dt);
}

/**
 * @brief Reset odometry to initial state (0,0,0)
 */
void Odometry::resetOdometry()
{
  x_ = 0.0;        // meters
  y_ = 0.0;        // meters
  heading_ = 0.0;  // radians
}

/**
 * @brief Set physical parameters of the robot
 * @param wheel_separation Distance between left and right wheels (meters)
 * @param wheel_base Distance between front and back wheels (meters)
 * @param front_left_wheel_radius Radius of front left wheel (meters)
 * @param front_right_wheel_radius Radius of front right wheel (meters)
 * @param back_left_wheel_radius Radius of back left wheel (meters)
 * @param back_right_wheel_radius Radius of back right wheel (meters)
 */
void Odometry::setWheelParams(
  double wheel_separation,
  double wheel_base,
  double front_left_wheel_radius,
  double front_right_wheel_radius,
  double back_left_wheel_radius,
  double back_right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheel_base_ = wheel_base;
  front_left_wheel_radius_ = front_left_wheel_radius;
  front_right_wheel_radius_ = front_right_wheel_radius;
  back_left_wheel_radius_ = back_left_wheel_radius;
  back_right_wheel_radius_ = back_right_wheel_radius;
}

/**
 * @brief Set size of velocity rolling window
 * @param velocity_rolling_window_size Number of samples to average
 */
void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

/**
 * @brief Integrate robot motion using Runge-Kutta 2nd order method
 * Used for small angular velocities
 * @param linear_x Distance moved forward/backward (meters)
 * @param linear_y Distance moved left/right (meters)
 * @param angular Angle rotated (radians)
 */
void Odometry::integrateRungeKutta2(
  double linear_x, double linear_y, double angular)
{
  // Calculate intermediate heading
  const double direction = heading_ + angular * 0.5;

  // Runge-Kutta 2nd order integration
  const double cos_h = cos(direction);
  const double sin_h = sin(direction);

  // Transform velocities from robot frame to world frame
  x_ += linear_x * cos_h - linear_y * sin_h;  // meters
  y_ += linear_x * sin_h + linear_y * cos_h;  // meters
  heading_ += angular;  // radians
}

/**
 * @brief Integrate robot motion using exact method or RK2 for small angles
 * @param linear_x Distance moved forward/backward (meters)
 * @param linear_y Distance moved left/right (meters)
 * @param angular Angle rotated (radians)
 */
void Odometry::integrateExact(
  double linear_x, double linear_y, double angular)
{
  // Use RK2 for very small angular motions
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear_x, linear_y, angular);
  }
  else
  {
    // Exact integration for larger angular motions
    const double heading_old = heading_;
    const double r = sqrt(linear_x * linear_x + linear_y * linear_y) / angular;
    heading_ += angular;

    // Calculate the angle of the linear velocity vector
    const double vel_angle = atan2(linear_y, linear_x);

    // Transform from robot frame to world frame
    x_ += r * (sin(heading_ + vel_angle) - sin(heading_old + vel_angle));  // meters
    y_ += -r * (cos(heading_ + vel_angle) - cos(heading_old + vel_angle)); // meters
  }
}

/**
 * @brief Reset velocity rolling mean accumulators
 */
void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace mecanum_drive_controller