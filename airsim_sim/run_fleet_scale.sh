#!/usr/bin/env bash
# Fleet scale test: N drones, each with its OWN ROS domain (1..N, like the
# real fleet) running real cuVSLAM + odom_correction + nvblox + FIS + planner.
# usage: run_fleet_scale.sh <N> [seconds]
set -e
N=${1:-2}; SECS=${2:-120}
BASE=/home/lucas/hercules-sim
NVBLOX_YAML=$BASE/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
NAMES=(ghost delta buckshee thunderstrike)
LABEL=fleet$N
LOG=$BASE/e1_frames
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
mkdir -p $LOG

# ---- fresh UE ----
pkill -f "UnrealEditor.*[B]locks" 2>/dev/null || true
for i in $(seq 1 30); do pgrep -f "UnrealEditor.*[B]locks" >/dev/null || break; sleep 2; done
pkill -9 -f "UnrealEditor.*[B]locks" 2>/dev/null || true
sleep 2
while ss -ltn | grep -q 41451; do sleep 2; done
cd $BASE/HERCULES/Unreal/Environments/Blocks
/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "$PWD/Blocks.uproject" \
  -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
  -settings=$BASE/settings-fleet-${N}drone.json -log -stdout -unattended -nosplash \
  > $LOG/ue_$LABEL.log 2>&1 &
cd $BASE
for i in $(seq 1 60); do ss -ltn | grep -q 41451 && break; sleep 3; done
ss -ltn | grep -q 41451 || { echo "UE failed"; exit 1; }
sleep 3
echo "airsim up with $N vehicles"

# ---- metrics sampler ----
(
  echo "t,gpu_util,vram_mb,load1,mem_used_gb" > $LOG/metrics_$LABEL.csv
  while true; do
    g=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    l=$(cut -d' ' -f1 /proc/loadavg)
    m=$(free -m | awk '/^Mem:/{printf "%.1f", $3/1024}')
    echo "$(date +%s),${g},${l},${m}" >> $LOG/metrics_$LABEL.csv
    sleep 5
  done
) &
METRICS_PID=$!

# ---- per-drone stacks, one ROS domain each ----
PIDS=()
for idx in $(seq 0 $((N-1))); do
  VEH=${NAMES[$idx]}
  DOM=$((idx+1))
  (
    export ROS_DOMAIN_ID=$DOM
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
      > $LOG/cuvslam_${LABEL}_$VEH.log 2>&1 &
    $BASE/ros2_ws/install/active_exploration/lib/active_exploration/odom_correction \
      > $LOG/odomcorr_${LABEL}_$VEH.log 2>&1 &
    $BASE/ros2_ws/install/nvblox_ros/lib/nvblox_ros/nvblox_node --ros-args \
      --params-file "$NVBLOX_YAML" \
      -p use_lidar:=false -p use_color:=false -p use_segmentation:=false \
      -p global_frame:=odom \
      -r camera_0/depth/image:=/sim/depth/image \
      -r camera_0/depth/camera_info:=/sim/depth/camera_info \
      > $LOG/nvblox_${LABEL}_$VEH.log 2>&1 &
    ros2 launch active_exploration fis.launch.py flight_height:=1.0 \
      > $LOG/fis_${LABEL}_$VEH.log 2>&1 &
    python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
      --ros-args -p debug_skip_arm_check:=true -p flight_height:=1.0 \
      > $LOG/planner_${LABEL}_$VEH.log 2>&1 &
    CAP=0; [ "$VEH" = "ghost" ] && CAP=1
    NB_OUT=$LOG/${LABEL}_$VEH NB_SECONDS=$SECS NB_VEH=$VEH \
      NB_CAPTURE=$CAP NB_STAGGER=$((idx*4)) \
      $BASE/venv/bin/python -u $BASE/cuvslam_sim_bridge.py \
      > $LOG/bridge_${LABEL}_$VEH.log 2>&1
    kill $(jobs -p) 2>/dev/null || true
  ) &
  PIDS+=($!)
done

for p in "${PIDS[@]}"; do wait $p || true; done
kill $METRICS_PID 2>/dev/null || true
pkill -f "[i]saac_ros_visual_slam" || true
pkill -f "[o]dom_correction" || true
pkill -f "[n]vblox_node" || true
pkill -f "[f]rontier_info" || true
pkill -f "[s]imple_exploration_planner" || true
echo "run $LABEL complete"

# ---- fidelity scorecard (Q7) ----
$BASE/fidelity_scorecard.sh "$LABEL" "$LOG" > $LOG/scorecard_$LABEL.txt 2>&1 || true
echo "--- fidelity scorecard ($LABEL): $(grep -c FAIL $LOG/scorecard_$LABEL.txt 2>/dev/null) FAIL lines -> $LOG/scorecard_$LABEL.txt"
