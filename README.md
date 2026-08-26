# yahboom_rosmaster #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/resolute)
![ROS_2](https://img.shields.io/ros/v/lyrical/rclcpp)

Automatic Addison support for the ROSMASTER X3 mecanum wheel robot robot by Yahboom - ROS 2

![ROSMASTER X3 in Gazebo](https://automaticaddison.com/wp-content/uploads/2024/11/gazebo-800-square-mecanum-controller.gif)

![ROSMASTER X3 in RViz](https://automaticaddison.com/wp-content/uploads/2024/11/rviz-800-square-mecanum-controller.gif)

## Getting Started
For a complete step-by-step walkthrough on how to build this repository from scratch, start with this tutorial:
[Create and Visualize a Robotic Arm with URDF](https://automaticaddison.com/create-and-visualize-a-robotic-arm-with-urdf-ros-2-lyrical/)

## A Note on Navigation

The `yahboom_rosmaster_navigation` and `yahboom_rosmaster_docking` packages
build, but you cannot run navigation on Lyrical yet.

Nav2 is only partly released for Lyrical. The individual servers are there
(amcl, planner, controller, bt_navigator, costmap_2d, map_server, and the
rest), but `nav2_bringup` and the `navigation2` metapackage are not Debian
packages yet, so `sudo apt install ros-lyrical-nav2-bringup` has nothing to
install. `rosmaster_x3_navigation.launch.py` looks for `nav2_bringup`, so it
will not start until that package ships.

Everything else in this repository works. The navigation tutorials go up as
soon as Nav2 is installable with apt.
