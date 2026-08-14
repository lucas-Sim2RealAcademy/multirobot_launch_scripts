#!/usr/bin/env bash
# Step 5: encode MRQ frames, cut the time-aligned RViz window, composite PiP,
# prepend a title card, deliver the final mp4.
#
# Time alignment: rviz x11grab is CFR 30 fps, so its effective start epoch =
# stop_epoch (logged the instant ffmpeg got SIGINT) - nframes/30. The MRQ window
# start epoch t0_epoch is in the keys JSON. RViz in-clip offset = t0 - start.
#
# usage: compose_final.sh <keys.json> <frames_dir> <rviz.mkv> <grab_stop.txt> <out.mp4>
set -eu
KEYS=$1; FRAMES=$2; RVIZ=$3; STOPF=$4; OUT=$5
W=/home/lucas/UE5/hercules-sim-big/mrq_work/composite
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf

T0=$(python3 -c "import json;print(json.load(open('$KEYS'))['t0_epoch'])")
NF=$(python3 -c "import json;print(json.load(open('$KEYS'))['nframes'])")
DUR=$(python3 -c "print($NF/30.0)")
STOP=$(cat "$STOPF")
RN=$(ffprobe -v error -count_frames -select_streams v -show_entries stream=nb_read_frames -of csv=p=0 "$RVIZ")
RSTART=$(python3 -c "print($STOP - $RN/30.0)")
OFF=$(python3 -c "print(max(0.0, $T0 - $RSTART))")
echo "rviz: frames=$RN start=$RSTART; window t0=$T0 dur=$DUR -> in-clip offset $OFF s"

# 1) encode the MRQ main pane
ffmpeg -y -loglevel warning -framerate 30 -i "$FRAMES/frame_%04d.png" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart \
  "$W/mrq_main.mp4"

# 2) cut + crop the RViz pane (3D viewport region of the 1920x1042 grab)
ffmpeg -y -loglevel warning -ss "$OFF" -i "$RVIZ" -t "$DUR" \
  -vf "crop=1520:952:376:89" -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p \
  "$W/rviz_cut.mp4"

# 3) composite: main 1920x1080 + RViz PiP bottom-right (640x400 + 3px border) + labels
ffmpeg -y -loglevel warning -i "$W/mrq_main.mp4" -i "$W/rviz_cut.mp4" -filter_complex "
[1:v]scale=640:400,pad=646:406:3:3:color=0xdadde3[pip];
[0:v][pip]overlay=1246:646[base];
[base]drawtext=fontfile=$FONT:text='HERCULES fleet - 4 drones - recorded autonomy replay':fontsize=34:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='onboard autonomy (ghost) - nvblox map + frontiers + planner':fontsize=21:fontcolor=white:borderw=2:bordercolor=black@0.7:x=w-tw-28:y=618[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$W/body.mp4"

# 4) title card (2.5 s) + concat
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES Fleet Exploration':fontsize=72:fontcolor=white:x=(w-tw)/2:y=h/2-110,
drawtext=fontfile=$FONT:text='JapanFest Street - Movie Render Queue cinematic':fontsize=36:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+4,
drawtext=fontfile=$FONT:text='4x autonomous exploration (cuVSLAM + nvblox + FIS) - ground-truth trajectory replay':fontsize=26:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+70" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$W/title.mp4"

printf "file '%s'\nfile '%s'\n" "$W/title.mp4" "$W/body.mp4" > "$W/concat.txt"
ffmpeg -y -loglevel warning -f concat -safe 0 -i "$W/concat.txt" -c copy "$OUT"
ffprobe -v error -select_streams v -show_entries stream=width,height,avg_frame_rate -show_entries format=duration -of default=nw=1 "$OUT"
echo "COMPOSITE_OK $OUT"
