#!/usr/bin/env bash
# 3x300s outward-campaign attempts, scored after each.
set -u
cd /home/lucas/UE5/hercules-sim-big/mrq_work/mapview
for lbl in out3d_c1 out3d_c2 out3d_c3; do
  echo "=== ATTEMPT $lbl $(date +%H:%M:%S) ==="
  env HERC_SETTINGS=/home/lucas/hercules-sim/settings-fleet-4drone-cross.json \
    HERC_ARENA_HALF_M=12 HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 \
    HERC_FLIGHT_HEIGHT=1.0 HERC_W_OUTWARD=1.0 HERC_GAIN_EXP=1.3 \
    HERC_FIS_GEOFENCE_MARGIN=1.4 HERC_VISITED_COOLDOWN_S=150 \
    HERC_PLAN_3D=1 HERC_FIS_PUBLISH_GRID_3D=true HERC_FIS_VP_Z_LEVELS=3 \
    HERC_FIS_HEIGHT_BAND=3.0 HERC_Z_MIN=0.9 \
    ./capture_fleet_bagonly.sh "$lbl" 300
  echo "--- score $lbl ---"
  (cd /home/lucas/hercules-sim && python3 investigation/score_run.py "$lbl" 2>&1 | grep -E "SCORE|outwardness|VERDICT|G[1-4]:")
  sleep 10
done
echo CAMPAIGN_DONE
