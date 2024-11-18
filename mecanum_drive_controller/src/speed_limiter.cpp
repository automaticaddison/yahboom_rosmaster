/**
 * @file speed_limiter.cpp
 * @brief Implementation of velocity, acceleration, and jerk limits for mecanum drive robots.
 *
 * This implements the speed limiting functionality for each velocity component
 * (x, y, angular) of a mecanum drive robot. It ensures smooth and safe motion by:
 * - Limiting maximum and minimum velocities
 * - Controlling acceleration to prevent sudden speed changes
 * - Managing jerk (rate of acceleration change) for smooth motion
 *
 * This speed limiter is used by creating three instances in the controller:
 * - One for x velocity (forward/backward)
 * - One for y velocity (left/right)
 * - One for angular velocity (rotation)
 *
 * @author Addison Sears-Collins
 * @date November 18, 2024
 */

#include <algorithm>
#include <stdexcept>
#include "mecanum_drive_controller/speed_limiter.hpp"

namespace mecanum_drive_controller
{

/**
 * @brief Constructor implementation for SpeedLimiter
 *
 * @param has_velocity_limits Enable/disable velocity limits
 * @param has_acceleration_limits Enable/disable acceleration limits
 * @param has_jerk_limits Enable/disable jerk limits
 * @param min_velocity Minimum allowed velocity (defaults to -max_velocity if not specified)
 * @param max_velocity Maximum allowed velocity
 * @param min_acceleration Minimum allowed acceleration (defaults to -max_acceleration if not specified)
 * @param max_acceleration Maximum allowed acceleration
 * @param min_jerk Minimum allowed jerk (defaults to -max_jerk if not specified)
 * @param max_jerk Maximum allowed jerk
 */
SpeedLimiter::SpeedLimiter(
  bool has_velocity_limits,
  bool has_acceleration_limits,
  bool has_jerk_limits,
  double min_velocity,
  double max_velocity,
  double min_acceleration,
  double max_acceleration,
  double min_jerk,
  double max_jerk)
: has_velocity_limits_(has_velocity_limits),
  has_acceleration_limits_(has_acceleration_limits),
  has_jerk_limits_(has_jerk_limits),
  min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration),
  min_jerk_(min_jerk),
  max_jerk_(max_jerk)
{
  // Check velocity limits and set defaults
  if (has_velocity_limits_)
  {
    // We must have a maximum velocity specified if limits are enabled
    if (std::isnan(max_velocity_))
    {
      throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
    }

    // If min velocity is not specified, make it the negative of max velocity
    if (std::isnan(min_velocity_))
    {
      min_velocity_ = -max_velocity_;
    }
  }

  // Check acceleration limits and set defaults
  if (has_acceleration_limits_)
  {
    // We must have a maximum acceleration specified if limits are enabled
    if (std::isnan(max_acceleration_))
    {
      throw std::runtime_error("Cannot apply acceleration limits if max_acceleration is not specified");
    }

    // If min acceleration is not specified, make it the negative of max acceleration
    if (std::isnan(min_acceleration_))
    {
      min_acceleration_ = -max_acceleration_;
    }
  }

  // Check jerk limits and set defaults
  if (has_jerk_limits_)
  {
    // We must have a maximum jerk specified if limits are enabled
    if (std::isnan(max_jerk_))
    {
      throw std::runtime_error("Cannot apply jerk limits if max_jerk is not specified");
    }

    // If min jerk is not specified, make it the negative of max jerk
    if (std::isnan(min_jerk_))
    {
      min_jerk_ = -max_jerk_;
    }
  }
}

/**
 * @brief Apply all enabled limits (jerk, acceleration, velocity) to the commanded velocity
 *
 * @param v Current commanded velocity to be limited
 * @param v0 Previous velocity command
 * @param v1 Velocity command before v0
 * @param dt Time step between commands
 * @return Limiting factor (1.0 if no limiting was needed)
 */
double SpeedLimiter::limit(double & v, double v0, double v1, double dt)
{
  // Store original velocity for calculating the limiting factor
  const double original_velocity = v;

  // Apply limits in sequence: first jerk, then acceleration, finally velocity
  limit_jerk(v, v0, v1, dt);
  limit_acceleration(v, v0, dt);
  limit_velocity(v);

  // Calculate and return the limiting factor
  // If the original velocity was 0, return 1.0 to avoid division by zero
  return original_velocity != 0.0 ? v / original_velocity : 1.0;
}

/**
 * @brief Apply velocity limits to the commanded velocity
 *
 * @param v Velocity to be limited
 * @return Limiting factor (1.0 if no limiting was needed)
 */
double SpeedLimiter::limit_velocity(double & v)
{
  // Store original velocity for calculating the limiting factor
  const double original_velocity = v;

  // If velocity limits are enabled, clamp the velocity between min and max
  if (has_velocity_limits_)
  {
    v = std::clamp(v, min_velocity_, max_velocity_);
  }

  // Calculate and return the limiting factor
  return original_velocity != 0.0 ? v / original_velocity : 1.0;
}

/**
 * @brief Apply acceleration limits to the commanded velocity
 *
 * @param v Current commanded velocity to be limited
 * @param v0 Previous velocity command
 * @param dt Time step between commands
 * @return Limiting factor (1.0 if no limiting was needed)
 */
double SpeedLimiter::limit_acceleration(double & v, double v0, double dt)
{
  // Store original velocity for calculating the limiting factor
  const double original_velocity = v;

  if (has_acceleration_limits_)
  {
    // Calculate the maximum allowed velocity changes in this time step
    const double dv_min = min_acceleration_ * dt;  // Min velocity change
    const double dv_max = max_acceleration_ * dt;  // Max velocity change

    // Calculate actual velocity change and clamp it to limits
    const double dv = std::clamp(v - v0, dv_min, dv_max);

    // Apply the limited velocity change
    v = v0 + dv;
  }

  // Calculate and return the limiting factor
  return original_velocity != 0.0 ? v / original_velocity : 1.0;
}

/**
 * @brief Apply jerk limits to the commanded velocity
 *
 * @param v Current commanded velocity to be limited
 * @param v0 Previous velocity command
 * @param v1 Velocity command before v0
 * @param dt Time step between commands
 * @return Limiting factor (1.0 if no limiting was needed)
 */
double SpeedLimiter::limit_jerk(double & v, double v0, double v1, double dt)
{
  // Store original velocity for calculating the limiting factor
  const double original_velocity = v;

  if (has_jerk_limits_)
  {
    // Calculate velocity changes
    const double dv = v - v0;    // Current velocity change
    const double dv0 = v0 - v1;  // Previous velocity change

    // Calculate time-adjusted jerk limits
    const double dt2 = 2.0 * dt * dt;  // dt² for jerk calculations
    const double da_min = min_jerk_ * dt2;  // Min acceleration change
    const double da_max = max_jerk_ * dt2;  // Max acceleration change

    // Limit the change in acceleration (jerk)
    const double da = std::clamp(dv - dv0, da_min, da_max);

    // Apply the limited acceleration change
    v = v0 + dv0 + da;
  }

  // Calculate and return the limiting factor
  return original_velocity != 0.0 ? v / original_velocity : 1.0;
}

}  // namespace mecanum_drive_controller