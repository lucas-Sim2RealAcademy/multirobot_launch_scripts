#!/usr/bin/env bash
# E1 A/B driver: run the real planner (pre-fix vs tip) against AirSim.
# usage: run_e1.sh <label> <planner_script> <seconds>
set -e
LABEL=$1; PLANNER=$2; SECS=${3:-110}
BASE=/home/lucas/hercules-sim
source /opt/ros/humble/setup.bash
source $BASE/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

# fresh sim instance (Cosys reset() leaves SimpleFlight unable to take off)
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
UE_PID=$!
cd $BASE
for i in $(seq 1 60); do ss -ltn | grep -q 41451 && break; sleep 3; done
ss -ltn | grep -q 41451 || { echo "UE failed to start"; exit 1; }
sleep 3
echo "airsim up (pid $UE_PID)"

python3 "$PLANNER" --ros-args \
  -p debug_skip_arm_check:=true -p flight_height:=1.0 \
  > "$BASE/e1_frames/planner_$LABEL.log" 2>&1 &
PLANNER_PID=$!
echo "planner ($LABEL) pid $PLANNER_PID"

E1_OUT=$BASE/e1_frames/$LABEL E1_SECONDS=$SECS E1_GROUND=${E1_GROUND:-1.2} \
  $BASE/venv/bin/python -u $BASE/e1_sim_bridge.py

kill $PLANNER_PID 2>/dev/null || true
wait $PLANNER_PID 2>/dev/null || true
echo "run $LABEL complete"
