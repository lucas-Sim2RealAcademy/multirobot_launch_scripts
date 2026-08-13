#!/usr/bin/env bash
# FULL-REAL perception run:
#   AirSim stereo+IMU+depth -> cuVSLAM -> odom_correction -> nvblox -> FIS -> planner
# usage: run_cuvslam_sim.sh <label> [seconds]
set -e
LABEL=${1:-cuvslam_run}; SECS=${2:-150}
BASE=/home/lucas/hercules-sim
NVBLOX_YAML=$BASE/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
mkdir -p $BASE/e1_frames

# fresh sim instance
pkill -f "UnrealEditor.*[B]locks" 2>/dev/null || true
for i in $(seq 1 30); do
  pgrep -f "UnrealEditor.*[B]locks" >/dev/null || break
  sleep 2
done
pkill -9 -f "UnrealEditor.*[B]locks" 2>/dev/null || true
sleep 2
while ss -ltn | grep -q 41451; do sleep 2; done
cd $BASE/HERCULES/Unreal/Environments/Blocks
/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "$PWD/Blocks.uproject" \
  -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
  -settings=$BASE/settings-fleet-sim.json -log -stdout -unattended -nosplash \
  > $BASE/e1_frames/ue_$LABEL.log 2>&1 &
cd $BASE
for i in $(seq 1 60); do ss -ltn | grep -q 41451 && break; sleep 3; done
ss -ltn | grep -q 41451 || { echo "UE failed to start"; exit 1; }
sleep 3
echo "airsim up"

# REAL cuVSLAM — parameters copied from their flight launch (vslam.launch.py)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
/opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam --ros-args \
  -p num_cameras:=2 -p min_num_images:=2 \
  -p enable_localization_n_mapping:=false \
  -p enable_imu_fusion:=true \
  -p gyro_noise_density:=0.000244 -p gyro_random_walk:=0.000019393 \
  -p accel_noise_density:=0.001862 -p accel_random_walk:=0.003 \
  -p calibration_frequency:=200.0 \
  -p rig_frame:=base_link -p imu_frame:=camera0_gyro_optical_frame \
  -p map_frame:=map -p odom_frame:=odom -p base_frame:=camera0_link \
  -p publish_odom_to_base_tf:=false \
  -p enable_rectified_pose:=true -p enable_image_denoising:=false \
  -p rectified_images:=true \
  -p camera_optical_frames:="[camera0_infra1_optical_frame, camera0_infra2_optical_frame]" \
  -r visual_slam/image_0:=/sim/ir_left/image \
  -r visual_slam/camera_info_0:=/sim/ir_left/camera_info \
  -r visual_slam/image_1:=/sim/ir_right/image \
  -r visual_slam/camera_info_1:=/sim/ir_right/camera_info \
  -r visual_slam/imu:=/sim/imu \
  > $BASE/e1_frames/cuvslam_$LABEL.log 2>&1 &
VSLAM_PID=$!
echo "cuvslam pid $VSLAM_PID"
sleep 3

# REAL odom_correction (+20deg tilt correction -> odom->camera0_link TF)
$BASE/ros2_ws/install/active_exploration/lib/active_exploration/odom_correction \
  > $BASE/e1_frames/odomcorr_$LABEL.log 2>&1 &
ODOM_PID=$!
echo "odom_correction pid $ODOM_PID"

# REAL nvblox (fork, CUDA)
$BASE/ros2_ws/install/nvblox_ros/lib/nvblox_ros/nvblox_node --ros-args \
  --params-file "$NVBLOX_YAML" \
  -p use_lidar:=false -p use_color:=false -p use_segmentation:=false \
  -p global_frame:=odom \
  -r camera_0/depth/image:=/sim/depth/image \
  -r camera_0/depth/camera_info:=/sim/depth/camera_info \
  > $BASE/e1_frames/nvblox_$LABEL.log 2>&1 &
NVBLOX_PID=$!
echo "nvblox pid $NVBLOX_PID"
sleep 3

# REAL FIS
ros2 launch active_exploration fis.launch.py flight_height:=1.0 \
  > $BASE/e1_frames/fis_$LABEL.log 2>&1 &
FIS_PID=$!

# REAL planner (8/12 tip)
python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
  --ros-args -p debug_skip_arm_check:=true -p flight_height:=1.0 \
  > $BASE/e1_frames/planner_$LABEL.log 2>&1 &
PLANNER_PID=$!

NB_OUT=$BASE/e1_frames/$LABEL NB_SECONDS=$SECS \
  $BASE/venv/bin/python -u $BASE/cuvslam_sim_bridge.py

kill $PLANNER_PID $FIS_PID $NVBLOX_PID $ODOM_PID $VSLAM_PID 2>/dev/null || true
sleep 1
echo "run $LABEL complete"
