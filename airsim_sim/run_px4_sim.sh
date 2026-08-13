#!/usr/bin/env bash
# HARDWARE-PARITY sim run:
#   PX4 v1.15.2 SITL (field params, EKF2 EV fusion) <-lockstep-> AirSim
#   MicroXRCEAgent UDP <-> /fmu/* topics
#   stereo+IMU -> cuVSLAM -> odom_correction -> nvblox -> FIS -> planner
#   cuVSLAM -> vio_bridge -> /fmu/in/vehicle_visual_odometry -> EKF2
#   planner -> reactive_depth_guard (SOLE /fmu/in writer) -> PX4
# Flight entry mirrors the field: commander takeoff, then Offboard.
# usage: run_px4_sim.sh <label> [seconds]
set -e
LABEL=${1:-px4_run}; SECS=${2:-180}
BASE=/home/lucas/hercules-sim
NVBLOX_YAML=$BASE/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0   # agent inherits PX4's UXRCE_DDS_DOM_ID=0
mkdir -p $BASE/e1_frames
LOG=$BASE/e1_frames

# ---- clean slate ----
pkill -f "UnrealEditor.*[B]locks" 2>/dev/null || true
pkill -9 -x px4 2>/dev/null || true
rm -f /tmp/px4_lock* /tmp/px4-sock* 2>/dev/null || true
pkill -f "[M]icroXRCEAgent" 2>/dev/null || true
for i in $(seq 1 30); do pgrep -f "UnrealEditor.*[B]locks" >/dev/null || break; sleep 2; done
pkill -9 -f "UnrealEditor.*[B]locks" 2>/dev/null || true
sleep 2
while ss -ltn | grep -qE "41451|4560"; do sleep 2; done

# ---- PX4 SITL first (listens on 4560) with a console FIFO ----
PXH=/tmp/pxh_$LABEL.fifo
rm -f $PXH; mkfifo $PXH
cd $BASE/PX4-1.15.2
( exec 3<>$PXH; make px4_sitl_default none_iris <&3 > $LOG/px4_$LABEL.log 2>&1 ) &
cd $BASE
exec 4>$PXH   # keep writer open
for i in $(seq 1 40); do grep -q "Waiting for simulator" $LOG/px4_$LABEL.log 2>/dev/null && break; sleep 3; done
echo "px4 sitl up"

# ---- UE with the PX4 settings (AirSim connects to 4560) ----
cd $BASE/HERCULES/Unreal/Environments/Blocks
/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "$PWD/Blocks.uproject" \
  -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
  -settings=$BASE/settings-fleet-px4.json -log -stdout -unattended -nosplash \
  > $LOG/ue_$LABEL.log 2>&1 &
cd $BASE
for i in $(seq 1 80); do grep -q "Simulator connected" $LOG/px4_$LABEL.log 2>/dev/null && break; sleep 3; done
grep -q "Simulator connected" $LOG/px4_$LABEL.log || { echo "PX4<->AirSim link failed"; exit 1; }
for i in $(seq 1 60); do ss -ltn | grep -q 41451 && break; sleep 3; done
echo "airsim + px4 linked"

# ---- XRCE agent (UDP; SITL client connects to 8888) ----
MicroXRCEAgent udp4 -p 8888 > $LOG/xrce_$LABEL.log 2>&1 &
sleep 3

# ---- perception stack (all real) ----
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
/opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam --ros-args \
  -p num_cameras:=2 -p min_num_images:=2 \
  -p enable_localization_n_mapping:=false -p enable_imu_fusion:=true \
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
  > $LOG/cuvslam_$LABEL.log 2>&1 &
$BASE/ros2_ws/install/active_exploration/lib/active_exploration/odom_correction \
  > $LOG/odomcorr_$LABEL.log 2>&1 &
$BASE/ros2_ws/install/nvblox_ros/lib/nvblox_ros/nvblox_node --ros-args \
  --params-file "$NVBLOX_YAML" \
  -p use_lidar:=false -p use_color:=false -p use_segmentation:=false \
  -p global_frame:=odom \
  -r camera_0/depth/image:=/sim/depth/image \
  -r camera_0/depth/camera_info:=/sim/depth/camera_info \
  > $LOG/nvblox_$LABEL.log 2>&1 &
ros2 launch active_exploration fis.launch.py flight_height:=1.0 \
  > $LOG/fis_$LABEL.log 2>&1 &

# ---- REAL vio_bridge: cuVSLAM VIO -> PX4 EKF2 ----
$BASE/ros2_ws/install/px4_offboard/lib/px4_offboard/vio_bridge \
  > $LOG/viobridge_$LABEL.log 2>&1 &

# ---- REAL reactive_depth_guard: sole /fmu/in writer ----
ros2 launch active_exploration reactive_guard.launch.py \
  camera_tilt_deg:=20.0 \
  depth_topic:=/sim/depth/image cam_info_topic:=/sim/depth/camera_info \
  > $LOG/guard_$LABEL.log 2>&1 &

# ---- REAL planner, REAL arm/offboard gating (no debug skip) ----
python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
  --ros-args -p flight_height:=1.0 \
  > $LOG/planner_$LABEL.log 2>&1 &

# ---- sensors + captures ----
NB_OUT=$LOG/$LABEL NB_SECONDS=$SECS \
  $BASE/venv/bin/python -u $BASE/px4_sim_bridge.py > $LOG/bridge_$LABEL.log 2>&1 &
BRIDGE_PID=$!

# ---- flight entry (field-mirroring): wait for EKF2+EV, takeoff, offboard ----
sleep 35
echo "commander takeoff" >&4
sleep 12
echo "commander mode offboard" >&4
echo "flight entry issued"

wait $BRIDGE_PID || true
echo "commander land" >&4
sleep 8
pkill -9 -x px4 || true
rm -f /tmp/px4_lock* /tmp/px4-sock* 2>/dev/null || true
pkill -f "[M]icroXRCEAgent" || true
pkill -f "[i]saac_ros_visual_slam" || true
pkill -f "[o]dom_correction" || true
pkill -f "[n]vblox_node" || true
pkill -f "[f]rontier_info" || true
pkill -f "[r]eactive_depth_guard" || true
pkill -f "[v]io_bridge" || true
pkill -f "[s]imple_exploration_planner" || true
exec 4>&-
echo "run $LABEL complete"
