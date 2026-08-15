#!/usr/bin/env bash
# Bag-only fleet capture (EXECUTION-PLAN-V6 §1.1): 4 domain relays -> viewing
# domain 42, ros2 bag, then the guarded 4-drone run.  NO rviz, NO scrot, NO
# ffmpeg/x11grab during the flight — mrq6 proved live capture load starves the
# stereo feed (18.5->11 Hz) and wrecks the flight.  The merged-map video is
# re-shot later from the bag with reshot_from_bag.sh.
#
# usage: [HERC_* env] capture_fleet_bagonly.sh <label> [secs]
#        (label must be new for run_exp; HERC_SETTINGS/HERC_ARENA_HALF_M/
#         HERC_CAPTURE/HERC_DEPTH_HZ pass through to run_exp -> run_fleet_radio)
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
LABEL=${1:?label}
SECS=${2:-240}
VIEW_DOM=42

# ---- PREFLIGHT GATE (new in v6) --------------------------------------------
# run_exp.sh checks stack orphans but NOT host load: its load1_pre was already
# 3.14 before mrq6 launched.  Refuse to fly on a loaded host or with any known
# render competitor alive.
#
# The previous flight's UE is EXPECTED to still be running (run_fleet_radio.sh
# leaves it up and kills it at the start of the next run) and idles at ~5
# cores, which would trip the load gate forever.  Kill it here first, then
# give the 1-min load average time to decay before judging the host.
if pgrep -f "UnrealEditor.*[B]locks" >/dev/null; then
  echo "preflight: killing leftover flight-sim UE (run_fleet_radio leaves it up)"
  pkill -f "UnrealEditor.*[B]locks" 2>/dev/null
  sleep 5
  pkill -9 -f "UnrealEditor.*[B]locks" 2>/dev/null
fi
for i in $(seq 1 36); do
  LOAD1=$(cut -d' ' -f1 /proc/loadavg)
  python3 -c "import sys; sys.exit(0 if float('$LOAD1') < 4.0 else 1)" && break
  [ $i -eq 1 ] && echo "preflight: waiting for load1 ($LOAD1) to decay below 4.0"
  sleep 10
done
LOAD1=$(cut -d' ' -f1 /proc/loadavg)
if python3 -c "import sys; sys.exit(0 if float('$LOAD1') >= 4.0 else 1)"; then
  echo "PREFLIGHT FAIL: load1=$LOAD1 >= 4.0 — host is busy, refusing to fly"
  exit 3
fi
BAD=$( { pgrep -af '[U]nrealEditor.*[K]ungfu'; \
         pgrep -af '(^|/)[x]264'; \
         pgrep -af '[f]fmpeg.*x11grab'; } 2>/dev/null )
if [ -n "$BAD" ]; then
  echo "PREFLIGHT FAIL: render competitor processes alive, refusing to fly:"
  echo "$BAD"
  exit 3
fi
CHROME_HEAVY=$(ps -eo pcpu,comm,args | awk '$2 ~ /chrome/ && $1 > 50 {print}' | head -5)
if [ -n "$CHROME_HEAVY" ]; then
  echo "PREFLIGHT WARN: busy chrome processes (render/join?) — flight continues:"
  echo "$CHROME_HEAVY"
fi
echo "preflight ok: load1=$LOAD1, no render competitors"

source /opt/ros/humble/setup.bash
source /home/lucas/hercules-sim/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

mkdir -p $M

# ---- relays (one per drone, src domain 1..4 -> sink 42) ----
pkill -f "[r]elay_drone.py" 2>/dev/null
sleep 1
i=1
for name in ghost delta buckshee thunderstrike; do
  python3 $M/relay_drone.py $name --src-domain $i --sink-domain $VIEW_DOM \
    > $M/relay_${LABEL}_$name.log 2>&1 &
  i=$((i+1))
done
echo "relays up"
sleep 3

# ---- bag record in the viewing domain ----
rm -rf $M/bag_$LABEL
ROS_DOMAIN_ID=$VIEW_DOM ros2 bag record -o $M/bag_$LABEL \
  /d1/mesh /d2/mesh /d3/mesh /d4/mesh \
  /d1/path /d2/path /d3/path /d4/path \
  /d1/drone /d2/drone /d3/drone /d4/drone \
  /tf /tf_static > $M/bag_${LABEL}.log 2>&1 &
BAG_PID=$!
echo "bag pid $BAG_PID"
sleep 2

# ---- the guarded run (blocks until flight + validation done) ----
/home/lucas/hercules-sim/investigation/run_exp.sh "$LABEL" "$SECS" \
  > $M/run_exp_$LABEL.out 2>&1
RC=$?
echo "run_exp rc=$RC"

# let the last mesh messages drain into the bag
sleep 5

# ---- teardown ----
kill -INT $BAG_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $BAG_PID 2>/dev/null || break; sleep 1; done
kill -9 $BAG_PID 2>/dev/null
pkill -f "[r]elay_drone.py" 2>/dev/null
echo "CAPTURE_DONE rc=$RC" | tee $M/capture_done_$LABEL.txt
date >> $M/capture_done_$LABEL.txt
exit $RC
