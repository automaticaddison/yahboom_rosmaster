cmake_minimum_required(VERSION 3.8)
project(yahboom_rosmaster_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(cmd_vel_relay src/cmd_vel_relay_node.cpp)
ament_target_dependencies(cmd_vel_relay
  rclcpp
  geometry_msgs
)

install(TARGETS
  cmd_vel_relay
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/assisted_teleoperation.py
  scripts/nav_to_pose.py
  scripts/test_nav_to_pose.py
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
