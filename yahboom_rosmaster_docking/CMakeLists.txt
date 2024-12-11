cmake_minimum_required(VERSION 3.8)
project(yahboom_rosmaster_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(image_proc REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(detected_dock_pose_publisher src/detected_dock_pose_publisher.cpp)
ament_target_dependencies(detected_dock_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros
)

install(TARGETS
  detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
