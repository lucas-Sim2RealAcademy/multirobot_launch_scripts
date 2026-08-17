#!/usr/bin/env bash
# OVERNIGHT VIDEO LOOP — produce watchable videos, not metrics.
#
# Phase 1: run every existing bamboo take through the full pipeline.
# Phase 2: when takes run out, fly a new one and immediately video it.
# Every iteration ends with either a finished mp4 or a logged reason why not.
# Nothing here is allowed to abort the loop.
#
# usage: overnight_video_loop.sh [hours]   (default 8)
set -u
H=/home/lucas/hercules-sim
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
T=/home/lucas/UE5/hercules-sim-big/mrq_work/temple
OUT=$H/overnight
LOG=$OUT/loop.log
DEADLINE=$(( $(date +%s) + ${1:-8}*3600 ))
mkdir -p $OUT

say() { echo "[$(date +%H:%M)] $*" | tee -a $LOG; }

# ---------- flight config (the working bamboo recipe) ----------
fly() {                      # fly <label> <seconds>
  local lbl=$1 secs=${2:-300}
  cd $M
  env HERC_UE_LOWSPEC=1 HERC_TAKEOFF_RESCUE=1 HERC_TAKEOFF_MIN_AGL=1.0 \
    HERC_NVBLOX_YAML=$H/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_bamboo.yaml \
    HERC_SETTINGS=$H/settings-bamboo-4drone.json \
    HERC_UE_MAP="/Game/Bamboo_Forest/Maps/Bamboo_Forest_LOD_Map" \
    HERC_ARENA_HALF_M=16 HERC_CAPTURE=0 HERC_DEPTH_HZ=6.0 \
    HERC_FLIGHT_HEIGHT=1.0 HERC_W_OUTWARD=1.2 HERC_GAIN_EXP=1.3 \
    HERC_FIS_GEOFENCE_MARGIN=1.4 HERC_VISITED_COOLDOWN_S=150 \
    HERC_KF_DEPTH_TOPIC=/sim/depth/image HERC_KF_INFO_TOPIC=/sim/depth/camera_info \
    ./capture_fleet_bagonly.sh "$lbl" "$secs" >> $OUT/fly_$lbl.log 2>&1
}

# ---------- map segment: shade the bag, replay in RViz, screen-record ----------
map_segment() {              # map_segment <label>
  local lbl=$1
  cd $M
  [ -d bag_${lbl}_shaded ] || python3 retint_bag.py bag_$lbl bag_${lbl}_shaded \
      >> $OUT/map_$lbl.log 2>&1
  BAG=$M/bag_${lbl}_shaded RVIZ_CFG=$M/fleet_map_bamboo.rviz \
    AUG_ARGS="--spawn-ned 1:0.0:-4.5,2:0.0:-1.5,3:0.0:1.5,4:0.0:4.5" \
    ./reshot_from_bag.sh ${lbl}_map 2 1 40 120 240 >> $OUT/map_$lbl.log 2>&1
  [ -s $M/reshot_${lbl}_map.mkv ]
}

# ---------- previews (always produced: cheap and always watchable) ----------
previews() {                 # previews <label>
  local lbl=$1
  cd $H
  python3 tools/extract_mesh_cloud.py "$lbl" >> $OUT/prev_$lbl.log 2>&1 || true
  HERC_SETTINGS=$H/settings-bamboo-4drone.json HERC_ARENA_HALF_M=16 \
    /home/lucas/miniconda3/bin/python3 tools/make_perdrone_preview.py "$lbl" \
    >> $OUT/prev_$lbl.log 2>&1 || true
  HERC_SETTINGS=$H/settings-bamboo-4drone.json HERC_ARENA_HALF_M=16 \
    /home/lucas/miniconda3/bin/python3 tools/make_path_preview3d.py "$lbl" \
    --mesh $H/e1_frames/runs/$lbl/mesh_cloud_$lbl.npz --zx 1.2 --anim \
    >> $OUT/prev_$lbl.log 2>&1 || true
}

# ---------- compose whatever exists into a watchable mp4 ----------
compose() {                  # compose <label>
  local lbl=$1
  local mkv=$M/reshot_${lbl}_map.mkv
  local out=$H/bamboo_${lbl}.mp4
  local FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
  local C=$OUT/c_$lbl; mkdir -p $C
  local score
  score=$(cd $H && python3 investigation/score_run.py "$lbl" 2>/dev/null | grep -oE "SCORE = [0-9.]+" | head -1)

  ffmpeg -y -loglevel error -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" \
    -vf "drawtext=fontfile=$FONT:text='HERCULES Fleet — Bamboo Forest':fontsize=64:fontcolor=white:x=(w-tw)/2:y=h/2-90,
drawtext=fontfile=$FONT:text='4 drones · 2D planner · 3D nvblox mapping':fontsize=32:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+10,
drawtext=fontfile=$FONT:text='take ${lbl}   ${score}':fontsize=24:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+70" \
    -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p $C/title.mp4 >> $OUT/comp_$lbl.log 2>&1

  : > $C/list.txt
  echo "file '$C/title.mp4'" >> $C/list.txt

  if [ -s "$mkv" ]; then
    ffmpeg -y -loglevel error -i "$mkv" -vf "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x0d0e14,
drawtext=fontfile=$FONT:text='shared nvblox map — 2x speed':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24" \
      -c:v libx264 -preset fast -crf 21 -pix_fmt yuv420p $C/map.mp4 >> $OUT/comp_$lbl.log 2>&1 \
      && echo "file '$C/map.mp4'" >> $C/list.txt
  fi

  local orb=$H/e1_frames/runs/$lbl/path3d_$lbl.mp4
  if [ -s "$orb" ]; then
    ffmpeg -y -loglevel error -i "$orb" -vf "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x14161e,
drawtext=fontfile=$FONT:text='flown paths + reconstruction':fontsize=30:fontcolor=white:x=28:y=24" \
      -c:v libx264 -preset fast -crf 21 -pix_fmt yuv420p $C/orbit.mp4 >> $OUT/comp_$lbl.log 2>&1 \
      && echo "file '$C/orbit.mp4'" >> $C/list.txt
  fi

  [ $(wc -l < $C/list.txt) -lt 2 ] && { say "  compose: nothing to show for $lbl"; return 1; }
  ffmpeg -y -loglevel error -f concat -safe 0 -i $C/list.txt -c copy "$out" >> $OUT/comp_$lbl.log 2>&1
  [ -s "$out" ] || return 1
  ffmpeg -y -loglevel error -i "$out" -vf scale=960:540 -c:v libx264 -preset fast -crf 28 \
    -pix_fmt yuv420p -movflags +faststart "$H/bamboo_${lbl}_mobile.mp4" >> $OUT/comp_$lbl.log 2>&1
  say "  VIDEO READY: bamboo_${lbl}.mp4 ($(du -h "$out" | cut -f1))"
}

quality() {                  # quality <label> -> single number
  cd $H
  /home/lucas/miniconda3/bin/python3 - "$1" <<'PY'
import glob, json, sys
import numpy as np
lbl = sys.argv[1]
R = "/home/lucas/hercules-sim/e1_frames/runs"
try:
    c = np.load(f"{R}/{lbl}/mesh_cloud_{lbl}.npz")["cloud"]
    band = c[(c[:,2] > 0.3) & (c[:,2] < 9)]
    mapped = len(band)
    extent = float(np.ptp(band[:,0]) * np.ptp(band[:,1])) if len(band) else 0.0
except Exception:
    mapped, extent = 0, 0.0
paths = []
for d in ("ghost","delta","buckshee","thunderstrike"):
    fs = sorted(glob.glob(f"{R}/{lbl}/{lbl}_{d}/meta_*.json"))
    pts = []
    for f in fs[::4]:
        try: pts.append(json.load(open(f))["ned"][:2])
        except Exception: pass
    paths.append(sum(((pts[i][0]-pts[i-1][0])**2 + (pts[i][1]-pts[i-1][1])**2) ** 0.5
                     for i in range(1, len(pts))))
balance = (min(paths)/max(paths)) if paths and max(paths) > 0 else 0.0
print(round(mapped/1000.0 + extent/50.0 + 300.0*balance, 1))
PY
}

produce() {                  # produce <label>  — never aborts the loop
  local lbl=$1
  say "produce $lbl"
  previews "$lbl"
  map_segment "$lbl" || say "  map segment failed for $lbl (continuing)"
  if compose "$lbl"; then
    local q best
    q=$(quality "$lbl"); best=$(cat $OUT/best_score 2>/dev/null || echo 0)
    say "  quality $lbl = $q (champion $best)"
    if awk "BEGIN{exit !($q > $best)}"; then
      mv -f $H/bamboo_${lbl}.mp4        $H/bamboo_best.mp4
      mv -f $H/bamboo_${lbl}_mobile.mp4 $H/bamboo_best_mobile.mp4 2>/dev/null
      echo "$q" > $OUT/best_score; echo "$lbl" > $OUT/best_label
      cp -f $H/e1_frames/runs/$lbl/perdrone_$lbl.png $OUT/best_perdrone.png 2>/dev/null
      say "  *** NEW CHAMPION $lbl (quality $q) -> bamboo_best.mp4"
    else
      rm -f $H/bamboo_${lbl}.mp4 $H/bamboo_${lbl}_mobile.mp4
      say "  $lbl did not beat champion - discarded"
    fi
  else
    say "  compose failed for $lbl"
  fi
}

# ================= phase 1: existing takes =================
say "=== OVERNIGHT VIDEO LOOP start (deadline $(date -d @$DEADLINE +%H:%M)) ==="
for lbl in bmb_d bmb_e bmb2d_b bmb_f1 bmb_f3 bmb_f2; do
  [ $(date +%s) -ge $DEADLINE ] && break
  [ -d $H/e1_frames/runs/$lbl ] || continue
  [ -d $M/bag_$lbl ] || { say "skip $lbl (no bag)"; continue; }
  produce "$lbl"
done

# ================= phase 2: fly + video, repeat =================
i=0
while [ $(date +%s) -lt $DEADLINE ]; do
  i=$((i+1))
  lbl="ovn$i"
  say "fly $lbl"
  fly "$lbl" 300
  if [ -d $H/e1_frames/runs/$lbl ] && [ -d $M/bag_$lbl ]; then
    produce "$lbl"
  else
    say "  $lbl did not archive (likely INVALID) — next"
  fi
done

say "=== LOOP DONE ==="
say "CHAMPION $(cat $OUT/best_label 2>/dev/null) quality $(cat $OUT/best_score 2>/dev/null)"
ls -la $H/bamboo_best*.mp4 2>/dev/null | tee -a $LOG
