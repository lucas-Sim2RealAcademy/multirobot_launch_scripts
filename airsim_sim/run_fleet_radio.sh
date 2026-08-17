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
NVBLOX_YAML=${HERC_NVBLOX_YAML:-$BASE/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml}
ALIGN_YAML=$BASE/src/multi_drone_nvblox/config/swarm_alignment.yaml
RS_NODE=/home/lucas/UE5/hercules_wrapper_ws/install/airsim_ros_pkgs/lib/airsim_ros_pkgs/airsim_realsense_node
# R2: patched cuVSLAM wrapper built from src/isaac_ros_visual_slam into the ros2_ws overlay
# (which line 49 already sources).  HERC_VSLAM=apt selects the stock binary for an A/B.
VSLAM_NODE=$BASE/ros2_ws/install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam
[ "${HERC_VSLAM:-patched}" = "apt" ] && \
  VSLAM_NODE=/opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam
NAMES=(ghost delta buckshee thunderstrike)
LABEL=radio$N
LOG=$BASE/e1_frames
CAPTURE=${HERC_CAPTURE:-0}   # chase cam is not a field sensor; ground truth is decoupled (+0.5-1.2 Hz stereo)
DEPTH_ENC=${HERC_DEPTH_ENCODING:-16UC1}
STEREO_HZ=${HERC_STEREO_HZ:-30.0}
DEPTH_HZ=${HERC_DEPTH_HZ:-10.0}   # MUST be a float (int kills the node); nvblox update_esdf_rate_hz is 10 anyway (+24% stereo)
IMU_HZ=${HERC_IMU_HZ:-200.0}
FLIGHT_HEIGHT=${HERC_FLIGHT_HEIGHT:-1.0}   # m AGL commanded everywhere (FIS band, planner z, bridge takeoff)
export HERC_FLIGHT_HEIGHT=$FLIGHT_HEIGHT
# Altitude-layered exploration (vertical deconfliction): per-drone flight
# heights, space-separated in spawn order.  Empty = everyone at FLIGHT_HEIGHT.
#   HERC_FLIGHT_HEIGHTS="0.9 1.5 2.1 2.7"
read -ra FLIGHT_HEIGHTS <<< "${HERC_FLIGHT_HEIGHTS:-}" 
# ---- environment/map selection (JapanFest port) --------------------------
# HERC_SETTINGS   AirSim settings json (default: the Blocks 4-drone file)
# HERC_UE_MAP     package path of the map to load, e.g. /Game/Maps/JapanFest_Street.
#                 Empty = Blocks default (GameDefaultMap=/Game/FlyingCPP/Maps/
#                 FlyingExampleMap, whose WorldSettings already names AirSimGameMode).
#                 Any other map needs the GameMode supplied on the URL, because
#                 GlobalDefaultGameMode is /Script/Blocks.BlocksGameMode (a class that
#                 does not exist in this project) and only FlyingExampleMap's
#                 WorldSettings carries the AirSim override.  UGameInstance::
#                 CreateGameModeForURL gives ?game= precedence over both.
SETTINGS=${HERC_SETTINGS:-$BASE/settings-fleet-${N}drone.json}
[ -f "$SETTINGS" ] || { echo "settings file not found: $SETTINGS"; exit 1; }
# fidelity_scorecard.sh (also via archive_run.sh) autodetects spawn offsets by
# globbing settings-fleet-*.json; without this it would score a JapanFest run
# against the Blocks spawn line and silently report wrong coverage.
export HERC_SETTINGS="$SETTINGS"
UE_MAP=${HERC_UE_MAP:-}
UE_MAP_ARG=()
[ -n "$UE_MAP" ] && UE_MAP_ARG=("${UE_MAP}?game=/Script/AirSim.AirSimGameMode")
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(find /opt/ros/humble/share/isaac_ros_gxf/gxf/lib -maxdepth 1 -type d | tr '\n' ':')
mkdir -p $LOG

[ -x "$RS_NODE" ] || { echo "airsim_realsense_node not built at $RS_NODE"; exit 1; }

# ---- FIX 1 / BUG-LIST-VERIFIED.md #8 (fis-bbox): one box, both consumers ----
# FIS defaults to a hardcoded +/-10 m AABB pinned to its own launch origin
# (frontier_info_structure_node.cpp:18-23) and this script used to override
# nothing: `fis.launch.py flight_height:=1.0` and a planner started with no
# bbox args BOTH silently took that default, so each drone only ever looked
# for frontiers inside a 20x20 m box around its own spawn -- a different box
# per vehicle, and smaller than the arena the fidelity scorecard measures.
# Hand both the same TEAM box: the union of every vehicle's +/-ARENA_HALF box
# around its spawn (exactly fidelity_scorecard.sh's arena), expressed in each
# vehicle's own odom/FLU frame.  HERC_TEAM_BBOX=0 restores the old defaults
# for an A/B without editing this file.
ARENA_HALF=${HERC_ARENA_HALF_M:-10.0}
TEAM_BBOX=${HERC_TEAM_BBOX:-1}
# ---- relaxed-clearance knob (sim cinematic runs; defaults stay stock) ----
# HERC_COLLISION_THRESH / HERC_GOAL_THRESH override the planner's A* clearance
# gates (stock 1.5 / 1.0 m).  JapanFest's market street has a ~1 m tall free
# tube (clutter tops 0.6-1.1 m AGL, banner bottoms 1.9-2.1 m) so the stock
# thresholds wall the fleet at the first well-integrated banner row even
# though the tube is physically flyable at 1.78 m AGL.
PLN_THRESH_ARGS=()
[ -n "${HERC_COLLISION_THRESH:-}" ] && \
  PLN_THRESH_ARGS+=(-p collision_threshold:=$HERC_COLLISION_THRESH)
[ -n "${HERC_GOAL_THRESH:-}" ] && \
  PLN_THRESH_ARGS+=(-p goal_threshold:=$HERC_GOAL_THRESH)
BBOX=()
if [ "$TEAM_BBOX" = "1" ]; then
  mapfile -t BBOX < <(python3 "$BASE/investigation/vio/team_box.py" \
      "$SETTINGS" "$ARENA_HALF" "${NAMES[@]:0:$N}")
  echo "team box (own odom frame), shared by FIS and planner:"
  for idx in $(seq 0 $((N-1))); do
    echo "  ${NAMES[$idx]}: ${BBOX[$idx]}"
  done
fi

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
  "${UE_MAP_ARG[@]}" \
  -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
  ${HERC_UE_LOWSPEC:+-ExecCmds="sg.ShadowQuality 0, sg.GlobalIlluminationQuality 0, sg.ReflectionQuality 0, sg.EffectsQuality 0, r.Shadow.Virtual.Enable 0, sg.ViewDistanceQuality 0, r.ViewDistanceScale 0.4, r.DetailMode 1, sg.AntiAliasingQuality 0, foliage.LODDistanceScale 0.25, foliage.MinimumScreenSize 0.01"} \
  -settings=$SETTINGS -log -stdout -unattended -nosplash \
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

# ---- virtual LoRa radio (Run B): real radiohive bridges talk to it ----
pkill -f "[v]irtual_lora_radio" 2>/dev/null || true
rm -rf /tmp/hercules_lora; mkdir -p /tmp/hercules_lora
LORA_N=$N LORA_RESET_ON_OPEN=0 python3 $BASE/virtual_lora_radio_v2.py \
  > $LOG/radio_$LABEL.log 2>&1 &
sleep 3
echo "virtual LoRa radio up ($N nodes, 190ms TDMA)"

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
  FH=${FLIGHT_HEIGHTS[$idx]:-$FLIGHT_HEIGHT}   # this drone's altitude band
  # FIX 1: this vehicle's slice of the team box, or empty (stock defaults)
  FIS_BBOX_ARGS=(); PLN_BBOX_ARGS=()
  if [ ${#BBOX[@]} -gt 0 ]; then
    read -r BX0 BY0 BX1 BY1 <<< "${BBOX[$idx]}"
    FIS_BBOX_ARGS=(bbox_min_x:=$BX0 bbox_min_y:=$BY0
                   bbox_max_x:=$BX1 bbox_max_y:=$BY1)
    PLN_BBOX_ARGS=(-p bbox_min_x:=$BX0 -p bbox_min_y:=$BY0
                   -p bbox_max_x:=$BX1 -p bbox_max_y:=$BY1)
  fi
  (
    export ROS_DOMAIN_ID=$DOM
    # C++ sensor node FIRST so cuVSLAM/nvblox find live publishers as they come up.
    "$RS_NODE" --ros-args \
      -p vehicle_name:=$VEH -p host_port:=41451 \
      -p stereo_hz:=$STEREO_HZ -p depth_hz:=$DEPTH_HZ -p imu_hz:=$IMU_HZ \
      -p depth_encoding:=$DEPTH_ENC \
      > $LOG/rsnode_${LABEL}_$VEH.log 2>&1 &
    sleep 2
    # R2 (VIO-STABILIZATION-PLAN.md §2): the patched fork, NOT the APT binary at
    # /opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam.  The APT build
    # registers every IMU sample of a batch at the image timestamp (zero-duration
    # preintegration window, so enable_imu_fusion:=true was a no-op) and feeds the
    # sequencer millisecond thresholds it compares against nanosecond deltas.
    "$VSLAM_NODE" --ros-args \
      -p num_cameras:=2 -p min_num_images:=2 \
      -p enable_localization_n_mapping:=false -p enable_imu_fusion:=true \
      -p image_jitter_threshold_ms:=200.0 -p imu_jitter_threshold_ms:=15.0 \
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
      --ros-args -p z_anchor_tau_s:=${HERC_Z_ANCHOR_TAU:-20.0} \
      > $LOG/odomcorr_${LABEL}_$VEH.log 2>&1 &
    $BASE/ros2_ws/install/nvblox_ros/lib/nvblox_ros/nvblox_node --ros-args \
      --params-file "$NVBLOX_YAML" \
      -p use_lidar:=false -p use_color:=false -p use_segmentation:=false \
      -p global_frame:=odom \
      -p decay_tsdf_rate_hz:=${HERC_TSDF_DECAY_HZ:-0.0} \
      -p input_qos:=SENSOR_DATA \
      -r camera_0/depth/image:=/sim/depth/image \
      -r camera_0/depth/camera_info:=/sim/depth/camera_info \
      > $LOG/nvblox_${LABEL}_$VEH.log 2>&1 &
    ros2 launch active_exploration fis.launch.py flight_height:=$FH \
      geofence_margin:=${HERC_FIS_GEOFENCE_MARGIN:-1.4} \
      height_band:=${HERC_FIS_HEIGHT_BAND:-1.0} \
      vp_z_levels:=${HERC_FIS_VP_Z_LEVELS:-1} \
      vp_dz:=${HERC_FIS_VP_DZ:-0.5} \
      frontier_z_neighbors:=${HERC_FIS_FRONTIER_Z_NEIGHBORS:-false} \
      cluster_max_size_z:=${HERC_FIS_CLUSTER_MAX_SIZE_Z:-1000000000.0} \
      vp_z_top:=${HERC_FIS_VP_Z_TOP:-false} \
      fov_pitch_deg:=${HERC_FIS_FOV_PITCH_DEG:-0.0} \
      publish_grid_3d:=${HERC_FIS_PUBLISH_GRID_3D:-false} \
      "${FIS_BBOX_ARGS[@]}" \
      > $LOG/fis_${LABEL}_$VEH.log 2>&1 &
    # REAL coordination stage + REAL radiohive bridge on the virtual radio
    ros2 launch multi_drone_nvblox coordination_stage.launch.py \
      drone_id:=$DOM alignment_yaml:=$ALIGN_YAML connectivity_mode:=wifi_only \
      > $LOG/coord_${LABEL}_$VEH.log 2>&1 &
    ros2 launch radiohive lora_bridge.launch.py \
      namespace:=d$DOM serial_port:=/tmp/hercules_lora/drone$DOM \
      > $LOG/lorabridge_${LABEL}_$VEH.log 2>&1 &
    python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
      --ros-args -p debug_skip_arm_check:=true -p flight_height:=$FH -p vehicle_id:=$DOM \
      -p odom_topic:=${HERC_ODOM_TOPIC:-/visual_slam/tracking/odometry_level} \
      "${PLN_BBOX_ARGS[@]}" "${PLN_THRESH_ARGS[@]}" \
      > $LOG/planner_${LABEL}_$VEH.log 2>&1 &
    CAP=$CAPTURE   # capture every drone (2x2 grid video)
    # NB_SENSORS=0: flight + PX4 stub + chase capture only, sensors owned by the C++ node.
    NB_OUT=$LOG/${LABEL}_$VEH NB_SECONDS=$SECS NB_VEH=$VEH \
      HERC_FLIGHT_HEIGHT=$FH \
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
pkill -f "[v]irtual_lora_radio" || true
pkill -f "[l]ora_bridge_node" || true
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
