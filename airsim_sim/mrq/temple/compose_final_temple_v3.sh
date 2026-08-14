#!/usr/bin/env bash
# FINAL composite v3: title + cinematic body (coverage paint + RViz ghost PiP +
# honest sweep chip) + science-view transition + 2.5x fleet-RViz segment.
# Also emits the standalone science capture (1x) + mobile variants.
set -eu
W=/home/lucas/UE5/hercules-sim-big/mrq_work/temple
C=$W/composite
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
KEYS=$W/fleet_keys_temple_v2.json
FRAMES=$W/out/frames
RVIZ=/home/lucas/UE5/hercules-sim-big/mrq_work/rviz/rviz_mrq6.mkv
STOPF=/home/lucas/UE5/hercules-sim-big/mrq_work/rviz/grab_stop_mrq6.txt
SCI_RAW=$W/rviz_fleet_science_raw.mkv
OUT=/home/lucas/hercules-sim/mrq_fleet_temple.mp4
OUTM=/home/lucas/hercules-sim/mrq_fleet_temple_mobile.mp4
SCI_OUT=/home/lucas/hercules-sim/rviz_fleet_science.mp4
SCI_OUTM=/home/lucas/hercules-sim/rviz_fleet_science_mobile.mp4
mkdir -p "$C"

T0=$(python3 -c "import json;print(json.load(open('$KEYS'))['t0_epoch'])")
NF=$(python3 -c "import json;print(json.load(open('$KEYS'))['nframes'])")
DUR=$(python3 -c "print($NF/30.0)")
STOP=$(cat "$STOPF")
RN=$(ffprobe -v error -count_frames -select_streams v -show_entries stream=nb_read_frames -of csv=p=0 "$RVIZ")
RSTART=$(python3 -c "print($STOP - $RN/30.0)")
OFF=$(python3 -c "print(max(0.0, $T0 - $RSTART))")
echo "ghost-PiP offset $OFF s"

# science raw timing: cut relative to publisher start epoch
GSTART=$(cat $W/science_grab_start.txt)
PSTART=$(grep start_wall $W/fleet_replay_epochs.txt | awk '{print $2}')
SOFF=$(python3 -c "print(max(0.0, $PSTART - $GSTART))")
echo "science in-clip offset $SOFF s"

# 1) cinematic main pane
ffmpeg -y -loglevel warning -framerate 30 -start_number 0 -i "$FRAMES/frame_%04d.png" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart "$C/mrq_main.mp4"

# 2) ghost-PiP cut (unchanged recipe)
ffmpeg -y -loglevel warning -ss "$OFF" -i "$RVIZ" -t "$DUR" \
  -vf "crop=1520:952:376:89" -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$C/rviz_cut.mp4"

# 3) body: PiP + labels + honest history-sweep chip (68 s replayed in the
#    first 2 s of the shot = 35x)
ffmpeg -y -loglevel warning -i "$C/mrq_main.mp4" -i "$C/rviz_cut.mp4" -filter_complex "
[1:v]scale=640:400,pad=646:406:3:3:color=0xdadde3[pip];
[0:v][pip]overlay=1246:646[base];
[base]drawtext=fontfile=$FONT:text='HERCULES fleet - Ancient Temple - recorded autonomy replay':fontsize=34:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='coverage paint + flight ribbons - ground-truth telemetry':fontsize=21:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=70,
drawtext=fontfile=$FONT:text='first 68 s of flight - replayed at 35x':fontsize=26:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=(w-tw)/2:y=120:alpha='if(lt(t,2.4),1,max(0,1-(t-2.4)/0.8))':enable='lt(t,3.4)',
drawtext=fontfile=$FONT:text='onboard autonomy (ghost) - nvblox map + frontiers + planner':fontsize=21:fontcolor=white:borderw=2:bordercolor=black@0.7:x=w-tw-28:y=618[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/body.mp4"

# 4) title card
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES Fleet Exploration':fontsize=72:fontcolor=white:x=(w-tw)/2:y=h/2-110,
drawtext=fontfile=$FONT:text='Ancient Temple Ruins - Movie Render Queue cinematic':fontsize=36:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+4,
drawtext=fontfile=$FONT:text='4x autonomous exploration (cuVSLAM + nvblox + FIS) - ground-truth trajectory replay':fontsize=26:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+70" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/title.mp4"

# 5) science transition card
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.0:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='- the science view -':fontsize=56:fontcolor=white:x=(w-tw)/2:y=h/2-50,
drawtext=fontfile=$FONT:text='same mission - shared-frame fleet telemetry, claims and coverage (recorded)':fontsize=26:fontcolor=0x8f98a8:x=(w-tw)/2:y=h/2+30" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/transition.mp4"

# 6) science segment for the main video: skip 2s of pre-roll, 2.5x speed
SS=$(python3 -c "print($SOFF + 2.0)")
ffmpeg -y -loglevel warning -ss "$SS" -t 85 -i "$SCI_RAW" -filter_complex "
[0:v]crop=1520:936:376:90,setpts=PTS/2.5,fps=30,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - coordination replay (mrq6 telemetry)':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='2.5x speed':fontsize=26:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,
drawtext=fontfile=$FONT:text='claims (translucent discs) = frontier assignments - no two drones chase the same goal':fontsize=22:fontcolor=0xaab2c0:borderw=2:bordercolor=black@0.7:x=28:y=h-44[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/science_seg.mp4"

# 7) concat main deliverable
printf "file '%s'\nfile '%s'\nfile '%s'\nfile '%s'\n" \
  "$C/title.mp4" "$C/body.mp4" "$C/transition.mp4" "$C/science_seg.mp4" > "$C/concat.txt"
ffmpeg -y -loglevel warning -f concat -safe 0 -i "$C/concat.txt" -c copy "$OUT"
ffprobe -v error -select_streams v -show_entries stream=width,height -show_entries format=duration,size -of default=nw=1 "$OUT"

# 8) mobile main (<5MB)
crf=26
while :; do
  ffmpeg -y -loglevel warning -i "$OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$OUTM"
  sz=$(stat -c %s "$OUTM"); echo "main mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 38 ] && break
done

# 9) standalone science capture (1x, full length incl. pre-roll + hold)
ffmpeg -y -loglevel warning -ss "$SOFF" -t 88 -i "$SCI_RAW" -filter_complex "
[0:v]crop=1520:936:376:90,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - mrq6 coordination replay (recorded telemetry, real time)':fontsize=28:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart "$SCI_OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$SCI_OUT"

crf=26
while :; do
  ffmpeg -y -loglevel warning -i "$SCI_OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$SCI_OUTM"
  sz=$(stat -c %s "$SCI_OUTM"); echo "science mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 38 ] && break
done
echo "COMPOSITE_V3_OK"
