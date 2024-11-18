/**
 * @file speed_limiter.hpp
 * @brief Speed limiter for mecanum drive velocity components
 *
 * This class provides velocity, acceleration, and jerk limiting functionality
 * for individual velocity components (x, y, angular) of a mecanum drive robot.
 *
 * @author Addison Sears-Collins
 * @date November 18, 2024
 */

#ifndef MECANUM_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_
#define MECANUM_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_

#include <cmath>

namespace mecanum_drive_controller
{
class SpeedLimiter
{
public:
  /**
   * @brief Constructor for speed limiter
   * @param has_velocity_limits     If true, applies velocity limits
   * @param has_acceleration_limits If true, applies acceleration limits
   * @param has_jerk_limits        If true, applies jerk limits
   * @param min_velocity           Minimum velocity [m/s], usually <= 0
   * @param max_velocity           Maximum velocity [m/s], usually >= 0
   * @param min_acceleration       Minimum acceleration [m/s^2], usually <= 0
   * @param max_acceleration       Maximum acceleration [m/s^2], usually >= 0
   * @param min_jerk              Minimum jerk [m/s^3], usually <= 0
   * @param max_jerk              Maximum jerk [m/s^3], usually >= 0
   */
  SpeedLimiter(
    bool has_velocity_limits = false,
    bool has_acceleration_limits = false,
    bool has_jerk_limits = false,
    double min_velocity = NAN,
    double max_velocity = NAN,
    double min_acceleration = NAN,
    double max_acceleration = NAN,
    double min_jerk = NAN,
    double max_jerk = NAN);

  /**
   * @brief Limit the velocity, acceleration, and jerk
   * @param v  Current velocity command [m/s]
   * @param v0 Previous velocity to v [m/s]
   * @param v1 Previous velocity to v0 [m/s]
   * @param dt Time step [s]
   * @return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  /**
   * @brief Limit the velocity
   * @param v Velocity [m/s]
   * @return Limiting factor (1.0 if none)
   */
  double limit_velocity(double & v);

  /**
   * @brief Limit the acceleration
   * @param v  Current velocity [m/s]
   * @param v0 Previous velocity [m/s]
   * @param dt Time step [s]
   * @return Limiting factor (1.0 if none)
   */
  double limit_acceleration(double & v, double v0, double dt);

  /**
   * @brief Limit the jerk (rate of change of acceleration)
   * @param v  Current velocity [m/s]
   * @param v0 Previous velocity to v [m/s]
   * @param v1 Previous velocity to v0 [m/s]
   * @param dt Time step [s]
   * @return Limiting factor (1.0 if none)
   * @see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
   */
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  // Enable/Disable velocity/acceleration/jerk limits:
  bool has_velocity_limits_;
  bool has_acceleration_limits_;
  bool has_jerk_limits_;

  // Velocity limits:
  double min_velocity_;     // [m/s]
  double max_velocity_;     // [m/s]

  // Acceleration limits:
  double min_acceleration_; // [m/s^2]
  double max_acceleration_; // [m/s^2]

  // Jerk limits:
  double min_jerk_;        // [m/s^3]
  double max_jerk_;        // [m/s^3]
};

}  // namespace mecanum_drive_controller
#endif  // MECANUM_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_