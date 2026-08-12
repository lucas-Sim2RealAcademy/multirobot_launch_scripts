#!/usr/bin/env bash
# Real-perception sim run: AirSim depth -> real nvblox -> real FIS -> real planner.
# usage: run_nvblox_sim.sh <label> [seconds]
set -e
LABEL=${1:-nvblox_run}; SECS=${2:-150}
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

# real nvblox (fork, CUDA)
$BASE/ros2_ws/install/nvblox_ros/lib/nvblox_ros/nvblox_node --ros-args \
  --params-file "$NVBLOX_YAML" \
  -p use_lidar:=false -p use_color:=false -p use_segmentation:=false \
  -p global_frame:=odom \
  -r camera_0/depth/image:=/sim/depth/image \
  -r camera_0/depth/camera_info:=/sim/depth/camera_info \
  > $BASE/e1_frames/nvblox_$LABEL.log 2>&1 &
NVBLOX_PID=$!
echo "nvblox pid $NVBLOX_PID"
sleep 5

# real FIS (frontier detection via nvblox esdf service)
ros2 launch active_exploration fis.launch.py flight_height:=1.0 \
  > $BASE/e1_frames/fis_$LABEL.log 2>&1 &
FIS_PID=$!
echo "fis pid $FIS_PID"

# real planner (8/12 tip)
python3 $BASE/src/active_exploration/scripts/simple_exploration_planner.py \
  --ros-args -p debug_skip_arm_check:=true -p flight_height:=1.0 \
  > $BASE/e1_frames/planner_$LABEL.log 2>&1 &
PLANNER_PID=$!
echo "planner pid $PLANNER_PID"

NB_OUT=$BASE/e1_frames/$LABEL NB_SECONDS=$SECS \
  $BASE/venv/bin/python -u $BASE/nvblox_sim_bridge.py

kill $PLANNER_PID $FIS_PID $NVBLOX_PID 2>/dev/null || true
sleep 1
echo "run $LABEL complete"
