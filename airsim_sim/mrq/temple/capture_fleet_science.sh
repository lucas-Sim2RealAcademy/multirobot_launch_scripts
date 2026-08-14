#!/usr/bin/env bash
# Fleet science-view capture: rviz2 on :2 + fleet_replay.py publisher, x11grab.
# Follows mrq_work/run_capture.sh conventions (1920x1042 grab, epoch logging).
W=/home/lucas/UE5/hercules-sim-big/mrq_work/temple
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=77
SPEED=${1:-1.0}

pkill -f "[r]viz2 -d $W/fleet_science.rviz" 2>/dev/null
sleep 2
DISPLAY=:2 nohup rviz2 -d $W/fleet_science.rviz > $W/rviz_science.log 2>&1 &
RVIZ_PID=$!
echo "rviz pid $RVIZ_PID"
sleep 12
DISPLAY=:2 scrot -o $W/science_pre.png 2>/dev/null || DISPLAY=:2 import -window root $W/science_pre.png

python3 -c 'import time; print("%.3f" % time.time())' > $W/science_grab_start.txt
DISPLAY=:2 ffmpeg -y -loglevel warning -f x11grab -framerate 30 -video_size 1920x1042 \
  -i :2 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \
  $W/rviz_fleet_science_raw.mkv > $W/ffmpeg_science.log 2>&1 &
FF_PID=$!
echo "ffmpeg pid $FF_PID"
sleep 2

# mid-run verification screenshot (approx t+42s of replay)
( sleep 50; DISPLAY=:2 scrot -o $W/science_mid.png 2>/dev/null || DISPLAY=:2 import -window root $W/science_mid.png ) &

python3 $W/fleet_replay.py --speed $SPEED > $W/fleet_replay.log 2>&1
RC=$?
echo "replay rc=$RC"

python3 -c 'import time; print("%.3f" % time.time())' > $W/science_grab_stop.txt
kill -INT $FF_PID 2>/dev/null
for i in $(seq 1 20); do kill -0 $FF_PID 2>/dev/null || break; sleep 1; done
kill -9 $FF_PID 2>/dev/null
kill $RVIZ_PID 2>/dev/null
echo "CAPTURE_DONE rc=$RC"
