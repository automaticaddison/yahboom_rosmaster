cmake_minimum_required(VERSION 3.16)
project(mecanum_drive_controller LANGUAGES CXX)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable
                     -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                     -Werror=missing-braces)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pal_statistics_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

# Generate parameter library
generate_parameter_library(mecanum_drive_controller_parameters
  src/mecanum_drive_controller_parameter.yaml
)

# Create controller library
add_library(mecanum_drive_controller SHARED
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_compile_features(mecanum_drive_controller PUBLIC cxx_std_17)

target_include_directories(mecanum_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mecanum_drive_controller>
)

target_link_libraries(mecanum_drive_controller PUBLIC mecanum_drive_controller_parameters)

ament_target_dependencies(mecanum_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Visibility macros
target_compile_definitions(mecanum_drive_controller PRIVATE "MECANUM_DRIVE_CONTROLLER_BUILDING_DLL")

# Export plugin
pluginlib_export_plugin_description_file(controller_interface mecanum_drive_plugin.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test file directory definition
  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")

  # Controller loading test
  #ament_add_gmock(test_load_mecanum_drive_controller
  #  test/test_load_mecanum_drive_controller.cpp)
  #ament_target_dependencies(test_load_mecanum_drive_controller
  #  controller_manager
  #  ros2_control_test_assets
  #)
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include/mecanum_drive_controller
)

install(TARGETS mecanum_drive_controller mecanum_drive_controller_parameters
  EXPORT export_mecanum_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export
ament_export_targets(export_mecanum_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
