#!/usr/bin/env bash
# Re-shoot the merged-map view from the recorded bag (no sim needed).
# usage: reshot_from_bag.sh <tag> <rate> [record=1] [shot_at_secs ...]
#   tag: output suffix; rate: bag play -r; record=1 -> x11grab to
#   reshot_<tag>.mkv; any further args: take a screenshot N secs after play
#   starts (shot_<tag>_pN.png).
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
TAG=${1:?tag}
RATE=${2:?rate}
RECORD=${3:-1}
shift 3 || shift $#

source /opt/ros/humble/setup.bash
source /home/lucas/hercules-sim/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42

pkill -f "[r]viz2 -d $M/fleet_map.rviz" 2>/dev/null
pkill -f "[r]os2 bag play" 2>/dev/null
sleep 2
DISPLAY=:2 nohup rviz2 -d $M/fleet_map.rviz > $M/rviz_reshot_$TAG.log 2>&1 &
RVIZ_PID=$!
sleep 12

FF_PID=""
if [ "$RECORD" = "1" ]; then
  python3 -c 'import time; print("%.3f" % time.time())' > $M/reshot_${TAG}_grabstart.txt
  DISPLAY=:2 ffmpeg -y -loglevel warning -f x11grab -framerate 30 -video_size 1920x1042 \
    -i :2 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \
    $M/reshot_$TAG.mkv > $M/ffmpeg_reshot_$TAG.log 2>&1 &
  FF_PID=$!
  sleep 2
fi

for s in "$@"; do
  ( sleep "$s"; DISPLAY=:2 scrot -o $M/shot_${TAG}_p${s}.png ) &
done

python3 -c 'import time; print("%.3f" % time.time())' > $M/reshot_${TAG}_playstart.txt
ros2 bag play $M/bag_mapview2 -r $RATE --clock > $M/bagplay_$TAG.log 2>&1
RC=$?
sleep 3

if [ -n "$FF_PID" ]; then
  kill -INT $FF_PID 2>/dev/null
  for i in $(seq 1 20); do kill -0 $FF_PID 2>/dev/null || break; sleep 1; done
  kill -9 $FF_PID 2>/dev/null
fi
kill $RVIZ_PID 2>/dev/null
echo "RESHOT_DONE $TAG rc=$RC"
