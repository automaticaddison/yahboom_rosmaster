#!/bin/bash
# Single script to launch the Yahboom ROSMASTERX3 with Gazebo, Nav2 and ROS 2 Controllers

cleanup() {
   echo "Cleaning up..."
   echo "Restarting ROS 2 daemon..."
   ros2 daemon stop
   sleep 1
   ros2 daemon start
   echo "Terminating all ROS 2-related processes..."
   kill 0
   exit
}

# Set up cleanup trap
trap 'cleanup' SIGINT

# For cafe.world -> z:=0.20
# For house.world -> z:=0.05
# To change Gazebo camera pose: gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

echo "Launching Gazebo simulation with Nav2..."
ros2 launch yahboom_rosmaster_navigation rosmaster_x3_navigation_launch.py \
   headless:=False \
   load_controllers:=true \
   world_file:=cafe.world \
   use_rviz:=true \
   use_robot_state_pub:=true \
   use_sim_time:=true \
   x:=0.0 \
   y:=0.0 \
   z:=0.20 \
   roll:=0.0 \
   pitch:=0.0 \
   yaw:=0.0 &

echo "Waiting 25 seconds for simulation to initialize..."
sleep 25

echo "Adjusting camera position..."
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

# Keep the script running until Ctrl+C
wait