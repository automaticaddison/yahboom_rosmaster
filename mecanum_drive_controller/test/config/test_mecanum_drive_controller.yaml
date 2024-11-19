mecanum_drive_controller:
  ros__parameters:
    front_left_joint_name: front_left_wheel_joint # Name of a joint that controls a front left wheel.
    front_right_joint_name: front_right_wheel_joint # Name of a joint that controls a front right wheel.
    back_left_joint_name: back_left_wheel_joint # Name of a joint that controls a back left wheel.
    back_right_joint_name: back_right_wheel_joint # Name of a joint that controls a back right wheel.

    wheel_base: 0.16 # Longitudinal distance between front and back wheels, in meters.
    wheel_separation: 0.169 # Lateral distance between left and right wheels, in meters.
    wheel_radius: 0.0325 # Wheel radius in meters.

    wheel_separation_multiplier: 1.0 # Correction factor for wheel separation

    front_left_wheel_radius_multiplier: 1.0 # Correction factor when radius of wheel differs from the nominal value in wheel_radius parameter.
    front_right_wheel_radius_multiplier: 1.0 # Correction factor when radius of wheel differs from the nominal value in wheel_radius parameter.
    back_left_wheel_radius_multiplier: 1.0 # Correction factor when radius of wheel differs from the nominal value in wheel_radius parameter.
    back_right_wheel_radius_multiplier: 1.0 # Correction factor when radius of wheel differs from the nominal value in wheel_radius parameter.

    tf_frame_prefix_enable: false # Enables or disables appending tf_prefix to tf frame ids
    tf_frame_prefix: "" # Prefix to be appended to the tf frames, will be added to odom_id and base_frame_id before publishing. If the parameter is empty, controller’s namespace will be used.

    odom_frame_id: odom # Name of the frame for odometry. This frame is parent of base_frame_id when controller publishes odometry.
    base_frame_id: base_link # Name of the robot’s base frame that is child of the odometry frame.
    pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Odometry covariance for the encoder output of the robot for the pose.
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Odometry covariance for the encoder output of the robot for the speed.

    position_feedback: true # Is there position feedback from hardware?
    open_loop: false # If set to true the odometry of the robot will be calculated from the commanded values and not from feedback.
    enable_odom_tf: true # Publish transformation between odom_frame_id and base_frame_id.

    cmd_vel_timeout: 0.5 # Timeout in seconds, after which input command on cmd_vel topic is considered outdated.
    publish_limited_velocity: false # Publish limited velocity value.
    velocity_rolling_window_size: 10 # Number of velocity samples the system will use to calculate the mean velocity for odometry updates.

    publish_rate: 50.0 # Publishing rate (Hz) of the odometry and TF messages.

    linear.x.has_velocity_limits: false
    linear.x.has_acceleration_limits: false
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 0.0
    linear.x.min_velocity: 0.0
    linear.x.max_acceleration: 0.0
    linear.x.max_jerk: 0.0
    linear.x.min_jerk: 0.0

    linear.y.has_velocity_limits: false
    linear.y.has_acceleration_limits: false
    linear.y.has_jerk_limits: false
    linear.y.max_velocity: 0.0
    linear.y.min_velocity: 0.0
    linear.y.max_acceleration: 0.0
    linear.y.max_jerk: 0.0
    linear.y.min_jerk: 0.0

    angular.z.has_velocity_limits: false
    angular.z.has_acceleration_limits: false
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 0.0
    angular.z.min_velocity: 0.0
    angular.z.max_acceleration: 0.0
    angular.z.min_acceleration: 0.0
    angular.z.max_jerk: 0.0
    angular.z.min_jerk: 0.0