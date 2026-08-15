#!/usr/bin/env bash
# R2: build the patched isaac_ros_visual_slam fork into the existing ros2_ws overlay on the
# BIG disk (/home/lucas/UE5/hercules-sim-big/ros2_ws -- root is at 99%, nothing large under /).
# run_fleet_radio.sh already sources this overlay, so the patched node lands on the path the
# script uses once :131 is repointed at the overlay binary.
set -e
WS=/home/lucas/UE5/hercules-sim-big/ros2_ws
SRC=/home/lucas/hercules-sim/src/isaac_ros_visual_slam
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
ln -sfn "$SRC/isaac_ros_visual_slam"            "$WS/src/isaac_ros_visual_slam"
ln -sfn "$SRC/isaac_ros_visual_slam_interfaces" "$WS/src/isaac_ros_visual_slam_interfaces"
cd "$WS"
colcon build --symlink-install \
  --packages-select isaac_ros_visual_slam_interfaces isaac_ros_visual_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "--- built binary ---"
ls -la "$WS/install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam"
