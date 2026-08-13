#!/usr/bin/env bash
# RUN A-FAST (Q9) — run_fleet_coord.sh with the Python sensor loops replaced by the C++
# airsim_realsense_node (one per drone, dedicated AirSim RPC connections per stream).
#
# Same topology as run_fleet_coord.sh: per-drone real cuVSLAM + odom_correction + nvblox +
# FIS + coordination_stage + planner on domains 1..N, plus N zenoh-bridge-ros2dds on
# loopback with the as-flown generated configs.
#
# What changes vs run_fleet_coord.sh:
#   * sensors: airsim_realsense_node publishes /sim/ir_left|ir_right|imu|depth at field rates
#     (measured on this host: 30 Hz stereo / 30 Hz depth / 200 Hz IMU single-drone;
#      ~20 Hz stereo / ~16-20 Hz depth / 200 Hz IMU per drone at N=4)
#     instead of cuvslam_sim_bridge.py's 2.4 Hz / 8 Hz / 22-30 Hz Python RPC loops.
#   * cuvslam_sim_bridge.py runs with NB_SENSORS=0: it still flies the drone, stubs
#     /fmu/out/vehicle_local_position + vehicle_status and executes planner setpoints, and
#     still does the ghost chase-cam capture + meta_*.json — but publishes no sensors and
#     no static TFs (the C++ node owns those now).  Its meta JSON imu_hz/stereo_n fields
#     therefore read 0 by design; the per-stream counters live in the node log
#     ($LOG/rsnode_<label>_<veh>.log, one line every 10 s).
#   * nvblox gets -p input_qos:=SENSOR_DATA — mandatory, because the node publishes
#     SensorDataQoS (BEST_EFFORT) and nvblox_base.yaml:55 defaults to SYSTEM_DEFAULT
#     (RELIABLE), which is an incompatible pairing and would silently deliver nothing.
#     cuVSLAM needs no such flag (image_qos/imu_qos already default to SENSOR_DATA).
#   * depth ships as 16UC1 millimetres (field parity with the realsense splitter) rather
#     than 32FC1 metres.  Override with HERC_DEPTH_ENCODING=32FC1.
#
# env knobs:
#   HERC_CAPTURE=0        disable the ghost chase-cam capture (the only remaining render
#                         competitor on rate-critical runs — Q9 recommends this)
#   HERC_DEPTH_ENCODING   16UC1 (default) | 32FC1
#   HERC_STEREO_HZ / HERC_DEPTH_HZ / HERC_IMU_HZ   node rate setpoints
#
# usage: run_fleet_fast.sh <N> [seconds]
set -e
N=${1:-4}; SECS=${2:-180}
BASE=/home/lucas/hercules-sim
NVBLOX_YAML=$BASE/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml
ALIGN_YAML=$BASE/src/multi_drone_nvblox/config/swarm_alignment.yaml
RS_NODE=/home/lucas/UE5/hercules_wrapper_ws/install/airsim_ros_pkgs/lib/airsim_ros_pkgs/airsim_realsense_node
NAMES=(ghost delta buckshee thunderstrike)
LABEL=fast$N
LOG=$BASE/e1_frames
CAPTURE=${HERC_CAPTURE:-1}
DEPTH_ENC=${HERC_DEPTH_ENCODING:-16UC1}
STEREO_HZ=${HERC_STEREO_HZ:-30.0}
DEPTH_HZ=${HERC_DEPTH_HZ:-30.0}
IMU_HZ=${HERC_IMU_HZ:-200.0}
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
mkdir -p $LOG

[ -x "$RS_NODE" ] || { echo "airsim_realsense_node not built at $RS_NODE"; exit 1; }

# ---- host prep: loopback DDS discovery for the bridges (runbook 1.2) ----
sudo -n ip link set lo multicast on || true
sudo -n ip route replace 239.255.0.0/16 dev lo || true

# ---- zenoh configs: as-flown generator, loopback subnet trick (runbook 1.3) ----
GEN=$BASE/src/multi_drone_nvblox/scripts/gen_zenoh_config.py
for i in $(seq 1 $N); do
  peers=$(for j in $(seq 1 $N); do if [ $j -ne $i ]; then printf '%s ' $j; fi; done)
  python3 "$GEN" --self-id $i --peer-ids "${peers% }" --domain-id $i \
      --subnet 127.0.0 --output /tmp/zenoh_sim_d$i.json5
done

# ---- fresh UE ----
pkill -f "UnrealEditor.*[B]locks" 2>/dev/null || true
pkill -f "[z]enoh-bridge-ros2dds" 2>/dev/null || true
pkill -f "[a]irsim_realsense_node" 2>/dev/null || true
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

# ---- zenoh bridges (runbook 1.4) ----
ZB=$BASE/bin/zenoh-bridge-ros2dds
for i in $(seq 1 $N); do
  env CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>' \
      RUST_LOG='zenoh=warn,zenoh_plugin_ros2dds=info' ROS_DOMAIN_ID=$i \
      "$ZB" -c /tmp/zenoh_sim_d$i.json5 > $LOG/zenoh_${LABEL}_d$i.log 2>&1 &
done
sleep 2

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

# ---- per-drone stacks ----
PIDS=()
for idx in $(seq 0 $((N-1))); do
  VEH=${NAMES[$idx]}
  DOM=$((idx+1))
  (
    export ROS_DOMAIN_ID=$DOM
    # C++ sensor node FIRST so cuVSLAM/nvblox find live publishers as they come up.
    "$RS_NODE" --ros-args \
      -p vehicle_name:=$VEH -p host_port:=41451 \
      -p stereo_hz:=$STEREO_HZ -p depth_hz:=$DEPTH_HZ -p imu_hz:=$IMU_HZ \
      -p depth_encoding:=$DEPTH_ENC \
      > $LOG/rsnode_${LABEL}_$VEH.log 2>&1 &
    sleep 2
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
      -p input_qos:=SENSOR_DATA \
      -r camera_0/depth/image:=/sim/depth/image \
      -r camera_0/depth/camera_info:=/sim/depth/camera_info \
      > $LOG/nvblox_${LABEL}_$VEH.log 2>&1 &
    ros2 launch active_exploration fis.launch.py flight_height:=1.0 \
      > $LOG/fis_${LABEL}_$VEH.log 2>&1 &
    # REAL coordination stage — as-flown (embedded lora_bridge, no radio)
    ros2 launch multi_drone_nvblox coordination_stage.launch.py \
      drone_id:=$DOM alignment_yaml:=$ALIGN_YAML connectivity_mode:=full \
      > $LOG/coord_${LABEL}_$VEH.log 2>&1 &
    python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
      --ros-args -p debug_skip_arm_check:=true -p flight_height:=1.0 \
      > $LOG/planner_${LABEL}_$VEH.log 2>&1 &
    CAP=0; [ "$VEH" = "ghost" ] && CAP=$CAPTURE
    # NB_SENSORS=0: flight + PX4 stub + chase capture only, sensors owned by the C++ node.
    NB_OUT=$LOG/${LABEL}_$VEH NB_SECONDS=$SECS NB_VEH=$VEH \
      NB_CAPTURE=$CAP NB_SENSORS=0 NB_STAGGER=$((idx*4)) \
      $BASE/venv/bin/python -u $BASE/cuvslam_sim_bridge.py \
      > $LOG/bridge_${LABEL}_$VEH.log 2>&1
    kill $(jobs -p) 2>/dev/null || true
  ) &
  PIDS+=($!)
done

for p in "${PIDS[@]}"; do wait $p || true; done
kill $METRICS_PID 2>/dev/null || true
pkill -f "[a]irsim_realsense_node" || true
pkill -f "[z]enoh-bridge-ros2dds" || true
pkill -f "[i]saac_ros_visual_slam" || true
pkill -f "[o]dom_correction" || true
pkill -f "[n]vblox_node" || true
pkill -f "[f]rontier_info" || true
pkill -f "[s]imple_exploration_planner" || true
pkill -f "[c]oordination_node" || true
pkill -f "[l]ora_bridge_node" || true
pkill -f "[a]lignment_manager" || true
pkill -f "[k]eyframe_exchange" || true
pkill -f "[p]eer_map_integrator" || true
echo "run $LABEL complete"

# ---- per-stream sensor rates (counters now live in the node, not the bridge) ----
for idx in $(seq 0 $((N-1))); do
  VEH=${NAMES[$idx]}
  echo "--- sensors $VEH: $(grep -a 'stereo ' $LOG/rsnode_${LABEL}_$VEH.log 2>/dev/null | tail -1)"
done

# ---- fidelity scorecard (Q7) ----
$BASE/fidelity_scorecard.sh "$LABEL" "$LOG" > $LOG/scorecard_$LABEL.txt 2>&1 || true
echo "--- fidelity scorecard ($LABEL): $(grep -c FAIL $LOG/scorecard_$LABEL.txt 2>/dev/null) FAIL lines -> $LOG/scorecard_$LABEL.txt"
