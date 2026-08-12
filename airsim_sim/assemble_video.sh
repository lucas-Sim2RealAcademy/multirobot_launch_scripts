#!/usr/bin/env bash
# Assemble the 3-pane sim video: chase cam on top, front RGB + depth below.
set -e
cd /home/lucas/hercules-sim/video_frames
read N DT FPS < capture_meta.txt
FPS=${FPS%.*}; [ "$FPS" -lt 1 ] && FPS=5
OUT=/home/lucas/hercules-sim/ghost_sim_flight.mp4
ffmpeg -y \
  -framerate "$FPS" -i chase_%05d.png \
  -framerate "$FPS" -i front_%05d.png \
  -framerate "$FPS" -i depth_%05d.png \
  -filter_complex "\
    [1:v]scale=480:360[rgb];[2:v]scale=480:360[dep];\
    [rgb][dep]hstack[bottom];\
    [0:v]scale=960:540[top];\
    [top][bottom]vstack,format=yuv420p,\
    drawtext=text='HERCULES AirSim - ghost pawn - D435i-like RGBD 20deg down + downward 1D lidar':x=10:y=10:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.5" \
  -c:v libx264 -preset medium -crf 20 "$OUT"
echo "wrote $OUT"
