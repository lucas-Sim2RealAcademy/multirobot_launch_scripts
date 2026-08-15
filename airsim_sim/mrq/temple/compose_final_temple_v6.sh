#!/usr/bin/env bash
# FINAL composite v6 (EXECUTION-PLAN-V6 §3): 10-segment timeline.
#   0 title 2.5s | 1-6 world shots S1..S6 (54s, MRQ frames) | 7 transition 2.2s
#   | 8 merged-map RViz 2x: 30s growth from first mesh + 5s final hold | 9 end
#   title 3s.  Total ~96.7s.
# Conventions from v5: card colors/fonts, honesty label, amber speed chips,
# per-drone legend colors (3f8cff/3fff72/ff4cf2/ff941e), <5MB 960x540 mobiles.
#
# env: KEYS (final stitched keys json), FRAMES (MRQ frames dir),
#      RESHOT (reshot mkv), GRABSTART/PLAYSTART (epoch files), BAG (bag dir),
#      RATE (bag play rate, default 2)
set -eu
W=/home/lucas/UE5/hercules-sim-big/mrq_work/temple
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
C=$W/composite_v6
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
KEYS=${KEYS:?final keys json}
FRAMES=${FRAMES:?frames dir}
RESHOT=${RESHOT:?reshot mkv}
GRABSTART=${GRABSTART:?grabstart file}
PLAYSTART=${PLAYSTART:?playstart file}
BAG=${BAG:?bag dir}
RATE=${RATE:-2}
OUT=/home/lucas/hercules-sim/mrq_fleet_temple.mp4
OUTM=/home/lucas/hercules-sim/mrq_fleet_temple_mobile.mp4
SCI_OUT=/home/lucas/hercules-sim/rviz_fleet_science.mp4
SCI_OUTM=/home/lucas/hercules-sim/rviz_fleet_science_mobile.mp4
mkdir -p "$C"

# ---- shot table from the keys json ----
mapfile -t SHOT_LINES < <(python3 - "$KEYS" <<'PYEOF'
import json, sys
K = json.load(open(sys.argv[1]))
for s in K["info"]["shots"]:
    print(f"{s['name']} {s['start']} {s['end']} {s.get('retime',1.0)}")
PYEOF
)
echo "shots:"; printf '  %s\n' "${SHOT_LINES[@]}"

# ---- first-mesh offset inside the reshot clip ----
# OFF = (playstart - grabstart) + (first_mesh_ts - bag_start_ts)/RATE - lead
OFF=$(python3 - "$BAG" "$GRABSTART" "$PLAYSTART" "$RATE" <<'PYEOF'
import glob, sqlite3, sys
bag, gs_f, ps_f, rate = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
db = glob.glob(bag + "/*.db3")[0]
con = sqlite3.connect(db)
t0 = con.execute("select min(timestamp) from messages").fetchone()[0]
tm = con.execute(
    "select min(timestamp) from messages where topic_id in "
    "(select id from topics where name like '%/mesh')").fetchone()[0]
gs = float(open(gs_f).read().strip())
ps = float(open(ps_f).read().strip())
off = max(0.0, (ps - gs) + (tm - t0) / 1e9 / rate - 2.0)
print(f"{off:.2f}")
PYEOF
)
DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$RESHOT")
echo "map growth starts at ${OFF}s of reshot (dur ${DUR}s)"

# ---- world segments (one trim per shot; caption/chips per plan) ----
LABEL_TXT='HERCULES fleet - Ancient Temple - recorded autonomy replay'
seg_i=0
CONCAT="$C/concat.txt"; : > "$CONCAT"

# title card
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES Fleet Exploration':fontsize=72:fontcolor=white:x=(w-tw)/2:y=h/2-110,
drawtext=fontfile=$FONT:text='Ancient Temple Ruins - Movie Render Queue cinematic':fontsize=36:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+4,
drawtext=fontfile=$FONT:text='4x autonomous exploration (cuVSLAM + nvblox + FIS) - ground-truth trajectory replay':fontsize=26:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+70" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/title.mp4"
echo "file '$C/title.mp4'" >> "$CONCAT"

for line in "${SHOT_LINES[@]}"; do
  read -r name start end retime <<< "$line"
  n=$((end - start))
  seg="$C/seg_$name.mp4"
  VF="drawtext=fontfile=$FONT:text='$LABEL_TXT':fontsize=32:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24"
  # amber speed chip on retimed segments
  is_fast=$(python3 -c "print(1 if float('$retime') > 1.05 else 0)")
  if [ "$is_fast" = "1" ]; then
    chip=$(python3 -c "print(f\"{float('$retime'):.1f}x speed\")")
    VF="$VF,drawtext=fontfile=$FONT:text='$chip':fontsize=28:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24"
  fi
  # micro-captions (plan §3): S1 + S5 only
  if [ "$name" = "s1_starburst" ]; then
    VF="$VF,drawtext=fontfile=$FONT:text='four goals claimed over the shared radio':fontsize=26:fontcolor=0xd8e0ec:borderw=2:bordercolor=black@0.7:x=28:y=h-52"
  elif [ "$name" = "s5_ribbonarc" ]; then
    VF="$VF,drawtext=fontfile=$FONT:text='flown paths - one 5-minute flight':fontsize=26:fontcolor=0xd8e0ec:borderw=2:bordercolor=black@0.7:x=28:y=h-52"
  fi
  ffmpeg -y -loglevel warning -framerate 30 -start_number "$start" \
    -i "$FRAMES/frame_%04d.png" -frames:v "$n" -vf "$VF" \
    -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$seg"
  echo "file '$seg'" >> "$CONCAT"
  seg_i=$((seg_i+1))
done

# transition card
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.2:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='one shared map - live':fontsize=58:fontcolor=white:x=(w-tw)/2:y=h/2-52,
drawtext=fontfile=$FONT:text='four drones, four onboard nvblox maps, one world - captured during the flight':fontsize=27:fontcolor=0x8f98a8:x=(w-tw)/2:y=h/2+34" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/transition.mp4"
echo "file '$C/transition.mp4'" >> "$CONCAT"

# ---- map segment: 30s growth + 5s final hold (both from the 2x reshot) ----
LEGEND="drawtext=fontfile=$FONT:text='GHOST':fontsize=26:fontcolor=0x3f8cff:borderw=2:bordercolor=black@0.8:x=28:y=h-44,
drawtext=fontfile=$FONT:text='DELTA':fontsize=26:fontcolor=0x3fff72:borderw=2:bordercolor=black@0.8:x=160:y=h-44,
drawtext=fontfile=$FONT:text='BUCKSHEE':fontsize=26:fontcolor=0xff4cf2:borderw=2:bordercolor=black@0.8:x=292:y=h-44,
drawtext=fontfile=$FONT:text='THUNDERSTRIKE':fontsize=26:fontcolor=0xff941e:borderw=2:bordercolor=black@0.8:x=488:y=h-44"
MAPVF="[0:v]crop=1520:934:376:92,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - shared map, captured live during the run':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='${RATE}x speed':fontsize=28:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,
$LEGEND,
drawtext=fontfile=$FONT:text='four onboard nvblox maps growing into one shared world':fontsize=24:fontcolor=0xaab2c0:borderw=2:bordercolor=black@0.7:x=w-tw-28:y=h-44[v]"
ffmpeg -y -loglevel warning -ss "$OFF" -t 30 -i "$RESHOT" -filter_complex "$MAPVF" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/map_growth.mp4"
HOLD_SS=$(python3 -c "print(max(0.0, float('$DUR') - 5.5))")
ffmpeg -y -loglevel warning -ss "$HOLD_SS" -t 5 -i "$RESHOT" -filter_complex "
[0:v]crop=1520:934:376:92,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='final shared map - one 5-minute flight':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
$LEGEND[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/map_hold.mp4"
echo "file '$C/map_growth.mp4'" >> "$CONCAT"
echo "file '$C/map_hold.mp4'" >> "$CONCAT"

# end title
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=3:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES':fontsize=84:fontcolor=white:x=(w-tw)/2:y=h/2-80,
drawtext=fontfile=$FONT:text='four autonomous drones - one flight - one shared map':fontsize=32:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+18" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/endtitle.mp4"
echo "file '$C/endtitle.mp4'" >> "$CONCAT"

# ---- concat main deliverable ----
ffmpeg -y -loglevel warning -f concat -safe 0 -i "$CONCAT" -c copy "$OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$OUT"

# ---- mobile main (<5MB, 960x540, +faststart) ----
crf=28
while :; do
  ffmpeg -y -loglevel warning -i "$OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$OUTM"
  sz=$(stat -c %s "$OUTM"); echo "main mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 40 ] && break
done

# ---- standalone merged-map science video (full mission, chipped) ----
ffmpeg -y -loglevel warning -ss "$OFF" -i "$RESHOT" -filter_complex "
[0:v]crop=1520:934:376:92,scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='HERCULES fleet - shared nvblox map (live-recorded run, ${RATE}x playback)':fontsize=28:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
drawtext=fontfile=$FONT:text='${RATE}x speed':fontsize=26:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,
$LEGEND[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p -movflags +faststart "$SCI_OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$SCI_OUT"

crf=28
while :; do
  ffmpeg -y -loglevel warning -i "$SCI_OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$SCI_OUTM"
  sz=$(stat -c %s "$SCI_OUTM"); echo "science mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 40 ] && break
done
echo "COMPOSITE_V6_OK"
