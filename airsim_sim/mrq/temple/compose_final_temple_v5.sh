#!/usr/bin/env bash
# FINAL composite v5 (user feedback pass 3: "more drones moving"):
#   1. title card -> temple cinematic, now THREE segments:
#        seg1 frames    0- 359 (frames_clean)  12 s establishing push-in
#        seg2 frames  900-1289 (frames_v4s2)   13 s NEW elevated wide "fleet at
#             work" shot -- contains the run's only 3-simultaneous-mover window
#             (delta+buckshee+thunderstrike, ~f1055-1100) + sustained d/b motion
#        seg3 frames 1500-2399 (frames_clean)  30 s hero-track (starts 100
#             frames earlier than v4 to include the fleet-speed peak 243 cm/s)
#   2. transition card -> merged-map RViz FULL-SCREEN (quadrotor markers +
#      recent/history paths, re-shot from the shaded bag at 2x, honest chip)
# Also emits the standalone merged-map video (2x, full length) + mobiles.
set -eu
SPEED_LABEL="2x"
W=/home/lucas/UE5/hercules-sim-big/mrq_work/temple
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
C=$W/composite_v5
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
FRAMES=$W/out/frames_clean
FRAMES2=$W/out/frames_v4s2
RAW=$M/reshot_final2x.mkv
OUT=/home/lucas/hercules-sim/mrq_fleet_temple.mp4
OUTM=/home/lucas/hercules-sim/mrq_fleet_temple_mobile.mp4
SCI_OUT=/home/lucas/hercules-sim/rviz_fleet_science.mp4
SCI_OUTM=/home/lucas/hercules-sim/rviz_fleet_science_mobile.mp4
mkdir -p "$C"

# first-mesh moment inside the re-shot clip:
#   (playstart - grabstart) + (mapstart_epoch - bag_start_epoch)/2   [2x play]
G=$(cat $M/reshot_final2x_grabstart.txt)
P=$(cat $M/reshot_final2x_playstart.txt)
BAG0=1786745009.296848393          # ros2 bag info start (bag_mapview2[_shaded])
MAPSTART=$(cat $M/mapstart_mapview2.txt)
OFF=$(python3 -c "print(max(0.0, ($P - $G) + ($MAPSTART - $BAG0)/2.0 - 4.0))")
echo "map segment starts at $OFF s into re-shot clip (${SPEED_LABEL} playback)"

# ---- 1) temple cinematic: three trims ----
ffmpeg -y -loglevel warning -framerate 30 -start_number 0 -i "$FRAMES/frame_%04d.png" \
  -frames:v 360 -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/cine_seg1.mp4"
ffmpeg -y -loglevel warning -framerate 30 -start_number 900 -i "$FRAMES2/frame_%04d.png" \
  -frames:v 390 -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/cine_seg2.mp4"
ffmpeg -y -loglevel warning -framerate 30 -start_number 1500 -i "$FRAMES/frame_%04d.png" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/cine_seg3.mp4"

# labels on the cinematic (small, top-left only)
for seg in cine_seg1 cine_seg2 cine_seg3; do
  ffmpeg -y -loglevel warning -i "$C/$seg.mp4" -vf "
drawtext=fontfile=$FONT:text='HERCULES fleet - Ancient Temple - recorded autonomy replay':fontsize=32:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24" \
    -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/${seg}_lab.mp4"
done

# ---- 2) title card ----
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES Fleet Exploration':fontsize=72:fontcolor=white:x=(w-tw)/2:y=h/2-110,
drawtext=fontfile=$FONT:text='Ancient Temple Ruins - Movie Render Queue cinematic':fontsize=36:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+4,
drawtext=fontfile=$FONT:text='4x autonomous exploration (cuVSLAM + nvblox + FIS) - ground-truth trajectory replay':fontsize=26:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+70" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/title.mp4"

# ---- 3) transition card ----
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.2:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='one shared map - live':fontsize=58:fontcolor=white:x=(w-tw)/2:y=h/2-52,
drawtext=fontfile=$FONT:text='four drones, four onboard nvblox maps, one world - captured during the flight':fontsize=27:fontcolor=0x8f98a8:x=(w-tw)/2:y=h/2+34" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/transition.mp4"

# ---- 4) merged-map segment (already 2x from bag playback; honest chip) ----
CHIP="drawtext=fontfile=$FONT:text='${SPEED_LABEL} speed':fontsize=28:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,"
ffmpeg -y -loglevel warning -ss "$OFF" -i "$RAW" -filter_complex "
[0:v]crop=1520:934:376:92,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - shared map, captured live during the run':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
$CHIP
drawtext=fontfile=$FONT:text='GHOST':fontsize=26:fontcolor=0x3f8cff:borderw=2:bordercolor=black@0.8:x=28:y=h-44,
drawtext=fontfile=$FONT:text='DELTA':fontsize=26:fontcolor=0x3fff72:borderw=2:bordercolor=black@0.8:x=160:y=h-44,
drawtext=fontfile=$FONT:text='BUCKSHEE':fontsize=26:fontcolor=0xff4cf2:borderw=2:bordercolor=black@0.8:x=292:y=h-44,
drawtext=fontfile=$FONT:text='THUNDERSTRIKE':fontsize=26:fontcolor=0xff941e:borderw=2:bordercolor=black@0.8:x=488:y=h-44,
drawtext=fontfile=$FONT:text='four onboard nvblox maps growing into one shared world':fontsize=24:fontcolor=0xaab2c0:borderw=2:bordercolor=black@0.7:x=w-tw-28:y=h-44[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/map_seg.mp4"

# ---- 5) concat main deliverable ----
printf "file '%s'\nfile '%s'\nfile '%s'\nfile '%s'\nfile '%s'\nfile '%s'\n" \
  "$C/title.mp4" "$C/cine_seg1_lab.mp4" "$C/cine_seg2_lab.mp4" \
  "$C/cine_seg3_lab.mp4" "$C/transition.mp4" "$C/map_seg.mp4" > "$C/concat.txt"
ffmpeg -y -loglevel warning -f concat -safe 0 -i "$C/concat.txt" -c copy "$OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$OUT"

# ---- 6) mobile main (<5MB) ----
crf=28
while :; do
  ffmpeg -y -loglevel warning -i "$OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$OUTM"
  sz=$(stat -c %s "$OUTM"); echo "main mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 40 ] && break
done

# ---- 7) standalone merged-map video (full mission, 2x, chipped) ----
ffmpeg -y -loglevel warning -ss "$OFF" -i "$RAW" -filter_complex "
[0:v]crop=1520:934:376:92,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - shared nvblox map (live-recorded run, 2x playback)':fontsize=28:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='2x speed':fontsize=26:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,
drawtext=fontfile=$FONT:text='GHOST':fontsize=26:fontcolor=0x3f8cff:borderw=2:bordercolor=black@0.8:x=28:y=h-44,
drawtext=fontfile=$FONT:text='DELTA':fontsize=26:fontcolor=0x3fff72:borderw=2:bordercolor=black@0.8:x=160:y=h-44,
drawtext=fontfile=$FONT:text='BUCKSHEE':fontsize=26:fontcolor=0xff4cf2:borderw=2:bordercolor=black@0.8:x=292:y=h-44,
drawtext=fontfile=$FONT:text='THUNDERSTRIKE':fontsize=26:fontcolor=0xff941e:borderw=2:bordercolor=black@0.8:x=488:y=h-44[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart "$SCI_OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$SCI_OUT"

# ---- 8) standalone mobile (<5MB) ----
crf=28
while :; do
  ffmpeg -y -loglevel warning -i "$SCI_OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$SCI_OUTM"
  sz=$(stat -c %s "$SCI_OUTM"); echo "science mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 40 ] && break
done
echo "COMPOSITE_V5_OK"
