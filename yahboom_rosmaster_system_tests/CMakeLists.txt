cmake_minimum_required(VERSION 3.8)
project(yahboom_rosmaster_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yahboom_rosmaster_msgs REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Add the square mecanum controller executable
add_executable(square_mecanum_controller
  src/square_mecanum_controller.cpp
)

# Add the mecanum parameters node executable
add_executable(mecanum_parameters_node
  src/mecanum_parameters_node.cpp
)

# Add the timed rotation action server executable
add_executable(timed_rotation_action_server
  src/timed_rotation_action_server.cpp
)

# Add the timed rotation action client executable
add_executable(timed_rotation_action_client
  src/timed_rotation_action_client.cpp
)

# Add a service server
add_executable(set_cleaning_state_service
  src/set_cleaning_state_service.cpp
)

# Specify dependencies for the target
ament_target_dependencies(square_mecanum_controller
  rclcpp
  geometry_msgs
  std_msgs
)

# Specify dependencies for the mecanum parameters node
ament_target_dependencies(mecanum_parameters_node
  rclcpp
  geometry_msgs
  std_msgs
  rcl_interfaces
)

# Specify dependencies for the timed rotation action server
ament_target_dependencies(timed_rotation_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  yahboom_rosmaster_msgs
)

# Specify dependencies for the timed rotation action client
ament_target_dependencies(timed_rotation_action_client
  rclcpp
  rclcpp_action
  yahboom_rosmaster_msgs
  std_msgs
)

# Specify dependencies for the service server
ament_target_dependencies(set_cleaning_state_service
  rclcpp
  yahboom_rosmaster_msgs
)


# Install the executables
install(TARGETS
  square_mecanum_controller
  mecanum_parameters_node
  timed_rotation_action_server
  timed_rotation_action_client
  set_cleaning_state_service
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
