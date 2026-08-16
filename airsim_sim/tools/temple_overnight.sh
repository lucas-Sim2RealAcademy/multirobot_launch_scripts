#!/usr/bin/env bash
# Overnight temple tuning grid: 12x300s takes, auto-scored.
set -u
cd /home/lucas/UE5/hercules-sim-big/mrq_work/mapview
i=0
for ZB in 0.2 0.4; do
  for CT in 0.8 1.0; do
    for SP in A B C; do
      i=$((i+1)); lbl="tn_${SP}_zb${ZB/./}_ct${CT/./}"
      case $SP in
        A) python3 - <<PY
import json
S = json.load(open("/home/lucas/hercules-sim/settings-temple-4drone.json"))
pos = {"ghost": (9.5,-6.5), "delta": (-8.0,-8.0), "buckshee": (-6.0,2.0), "thunderstrike": (1.0,-8.0)}
for n,(x,y) in pos.items():
    k = n.capitalize() if n.capitalize() in S["Vehicles"] else n
    S["Vehicles"][k]["X"], S["Vehicles"][k]["Y"] = x, y
json.dump(S, open("/tmp/settings_tn.json","w"), indent=2)
PY
        ;;
        B) python3 - <<PY
import json
S = json.load(open("/home/lucas/hercules-sim/settings-temple-4drone.json"))
pos = {"ghost": (7.0,-4.0), "delta": (-6.0,-6.0), "buckshee": (-4.0,0.5), "thunderstrike": (2.0,-5.5)}
for n,(x,y) in pos.items():
    k = n.capitalize() if n.capitalize() in S["Vehicles"] else n
    S["Vehicles"][k]["X"], S["Vehicles"][k]["Y"] = x, y
json.dump(S, open("/tmp/settings_tn.json","w"), indent=2)
PY
        ;;
        C) python3 - <<PY
import json
S = json.load(open("/home/lucas/hercules-sim/settings-temple-4drone.json"))
pos = {"ghost": (5.0,-2.0), "delta": (-5.0,-4.5), "buckshee": (-2.5,1.5), "thunderstrike": (0.5,-4.0)}
for n,(x,y) in pos.items():
    k = n.capitalize() if n.capitalize() in S["Vehicles"] else n
    S["Vehicles"][k]["X"], S["Vehicles"][k]["Y"] = x, y
json.dump(S, open("/tmp/settings_tn.json","w"), indent=2)
PY
        ;;
      esac
      echo "=== TAKE $i/12 $lbl $(date +%H:%M) ==="
      env HERC_UE_LOWSPEC=1 \
        HERC_SETTINGS=/tmp/settings_tn.json \
        HERC_UE_MAP="/Game/AncientTempleRuins/Levels/L_Showcase_01" \
        HERC_ARENA_HALF_M=10 HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 \
        HERC_FLIGHT_HEIGHT=1.0 HERC_W_OUTWARD=1.0 HERC_GAIN_EXP=1.3 \
        HERC_FIS_GEOFENCE_MARGIN=1.4 HERC_VISITED_COOLDOWN_S=150 \
        HERC_COLLISION_THRESH=$CT HERC_GOAL_THRESH=0.6 \
        HERC_PLAN_3D=1 HERC_FIS_PUBLISH_GRID_3D=true HERC_FIS_VP_Z_LEVELS=3 \
        HERC_FIS_HEIGHT_BAND=3.0 HERC_Z_MIN=0.6 HERC_Z_MAX=3.0 \
        HERC_FIS_VP_DZ=0.8 HERC_FIS_FRONTIER_Z_NEIGHBORS=true \
        HERC_FIS_CLUSTER_MAX_SIZE_Z=1.5 HERC_FIS_VP_Z_TOP=true \
        HERC_FIS_FOV_PITCH_DEG=20.0 HERC_W_ZBAND=$ZB HERC_VISITED_3D=1 \
        HERC_KF_DEPTH_TOPIC=/sim/depth/image HERC_KF_INFO_TOPIC=/sim/depth/camera_info \
        ./capture_fleet_bagonly.sh "$lbl" 300
      (cd /home/lucas/hercules-sim && python3 investigation/score_run.py "$lbl" 2>&1 | grep -E "SCORE =|path=|VERDICT")
      sleep 5
    done
  done
done
echo OVERNIGHT_DONE
