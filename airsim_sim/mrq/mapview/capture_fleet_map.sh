#!/usr/bin/env bash
# Merged-map fleet capture: 4 domain relays -> viewing domain 42, ros2 bag,
# rviz2 on :2, x11grab, timed verification screenshots, then the guarded
# 4-drone run (stock config, HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0).
#
# usage: capture_fleet_map.sh <label> [secs]   (label must be new for run_exp)
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
LABEL=${1:?label}
SECS=${2:-240}
VIEW_DOM=42

source /opt/ros/humble/setup.bash
source /home/lucas/hercules-sim/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

mkdir -p $M
rm -f $M/shot_${LABEL}_t*.png

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

# ---- rviz on :2 ----
pkill -f "[r]viz2 -d $M/fleet_map.rviz" 2>/dev/null
sleep 1
ROS_DOMAIN_ID=$VIEW_DOM DISPLAY=:2 nohup rviz2 -d $M/fleet_map.rviz \
  > $M/rviz_${LABEL}.log 2>&1 &
RVIZ_PID=$!
echo "rviz pid $RVIZ_PID"
sleep 12
DISPLAY=:2 scrot -o $M/shot_${LABEL}_pre.png

# ---- x11grab ----
python3 -c 'import time; print("%.3f" % time.time())' > $M/grab_start_$LABEL.txt
DISPLAY=:2 ffmpeg -y -loglevel warning -f x11grab -framerate 30 -video_size 1920x1042 \
  -i :2 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \
  $M/mapview_${LABEL}_raw.mkv > $M/ffmpeg_$LABEL.log 2>&1 &
FF_PID=$!
echo "ffmpeg pid $FF_PID"
sleep 2

# ---- verification screenshots, timed from FIRST MESH seen by any relay ----
(
  for i in $(seq 1 300); do
    if grep -q "FIRST_MESH" $M/relay_${LABEL}_*.log 2>/dev/null; then break; fi
    sleep 2
  done
  python3 -c 'import time; print("%.3f" % time.time())' > $M/mapstart_$LABEL.txt
  sleep 20; DISPLAY=:2 scrot -o $M/shot_${LABEL}_t20.png
  sleep 40; DISPLAY=:2 scrot -o $M/shot_${LABEL}_t60.png
  sleep 60; DISPLAY=:2 scrot -o $M/shot_${LABEL}_t120.png
) &
SHOT_PID=$!

# ---- the guarded run (blocks until flight + validation done) ----
HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 \
  /home/lucas/hercules-sim/investigation/run_exp.sh "$LABEL" "$SECS" \
  > $M/run_exp_$LABEL.out 2>&1
RC=$?
echo "run_exp rc=$RC"

# hold the final merged map on screen briefly
sleep 8
DISPLAY=:2 scrot -o $M/shot_${LABEL}_final.png

# ---- teardown ----
python3 -c 'import time; print("%.3f" % time.time())' > $M/grab_stop_$LABEL.txt
kill -INT $FF_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $FF_PID 2>/dev/null || break; sleep 1; done
kill -9 $FF_PID 2>/dev/null
kill -INT $BAG_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $BAG_PID 2>/dev/null || break; sleep 1; done
kill $RVIZ_PID 2>/dev/null
kill $SHOT_PID 2>/dev/null
pkill -f "[r]elay_drone.py" 2>/dev/null
echo "CAPTURE_DONE rc=$RC" | tee $M/capture_done_$LABEL.txt
date >> $M/capture_done_$LABEL.txt
