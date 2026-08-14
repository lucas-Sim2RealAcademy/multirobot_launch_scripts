#!/usr/bin/env bash
# MRQ pipeline step 1: fresh JapanFest run + time-aligned RViz capture on :2.
# RViz runs in ghost's domain (ROS_DOMAIN_ID=1). Screen recorded via x11grab.
# Time alignment: we log the ffmpeg STOP epoch and compute the effective start
# as stop_epoch - nframes/30 (x11grab emits CFR 30fps), which is exact against
# the wall-clock t field in meta_*.json.
W=/home/lucas/UE5/hercules-sim-big/mrq_work
LABEL=${1:-mrq1}
SECS=${2:-150}

source /opt/ros/humble/setup.bash
source /home/lucas/hercules-sim/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

# --- fresh rviz on :2, ghost domain ---
[ -f $W/rviz/rviz_test.pid ] && kill $(grep -o '[0-9]*' $W/rviz/rviz_test.pid) 2>/dev/null
pkill -f "[r]viz2 -d $W/rviz/fleet_ghost.rviz" 2>/dev/null
sleep 2
ROS_DOMAIN_ID=1 DISPLAY=:2 nohup rviz2 -d $W/rviz/fleet_ghost.rviz \
  > $W/rviz/rviz_${LABEL}.log 2>&1 &
RVIZ_PID=$!
echo "rviz pid $RVIZ_PID"
sleep 10

# --- x11grab recording ---
python3 -c 'import time; print("%.3f" % time.time())' > $W/rviz/grab_start_approx_${LABEL}.txt
DISPLAY=:2 ffmpeg -y -loglevel warning -f x11grab -framerate 30 -video_size 1920x1042 \
  -i :2 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \
  $W/rviz/rviz_${LABEL}.mkv > $W/rviz/ffmpeg_${LABEL}.log 2>&1 &
FF_PID=$!
echo "ffmpeg pid $FF_PID"
sleep 2

# --- the guarded run ---
HERC_SETTINGS=/home/lucas/hercules-sim/settings-japanfest-4drone.json \
HERC_UE_MAP=/Game/Maps/JapanFest_Street \
  /home/lucas/hercules-sim/investigation/run_exp.sh "$LABEL" "$SECS" \
  > $W/run_exp_${LABEL}.out 2>&1
RC=$?
echo "run_exp rc=$RC"

# let rviz show the final integrated state briefly
sleep 5

# --- stop recording; log stop epoch the instant we signal ffmpeg ---
python3 -c 'import time; print("%.3f" % time.time())' > $W/rviz/grab_stop_${LABEL}.txt
kill -INT $FF_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $FF_PID 2>/dev/null || break; sleep 1; done
kill -9 $FF_PID 2>/dev/null
kill $RVIZ_PID 2>/dev/null

echo "rc=$RC" > $W/capture_done_${LABEL}.txt
date >> $W/capture_done_${LABEL}.txt
