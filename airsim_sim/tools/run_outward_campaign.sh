#!/usr/bin/env bash
# 3x300s outward-campaign attempts, scored after each.
set -u
cd /home/lucas/UE5/hercules-sim-big/mrq_work/mapview
for lbl in temple3d_c1 temple3d_c2 temple3d_c3; do
  echo "=== ATTEMPT $lbl $(date +%H:%M:%S) ==="
  env HERC_SETTINGS=/home/lucas/hercules-sim/settings-fleet-4drone-cross.json \
    HERC_ARENA_HALF_M=12 HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 \
    HERC_FLIGHT_HEIGHT=1.0 HERC_W_OUTWARD=1.0 HERC_GAIN_EXP=1.3 \
    HERC_FIS_GEOFENCE_MARGIN=1.4 HERC_VISITED_COOLDOWN_S=150 \
    HERC_PLAN_3D=1 HERC_FIS_PUBLISH_GRID_3D=true HERC_FIS_VP_Z_LEVELS=3 \
    HERC_FIS_HEIGHT_BAND=3.0 HERC_Z_MIN=0.6 HERC_Z_MAX=3.0 \
    HERC_FLIGHT_HEIGHTS="0.9 1.5 2.1 2.7" HERC_FIS_VP_DZ=0.8 \
    HERC_FIS_FRONTIER_Z_NEIGHBORS=true HERC_FIS_CLUSTER_MAX_SIZE_Z=1.5 \
    HERC_FIS_VP_Z_TOP=true HERC_FIS_FOV_PITCH_DEG=20.0 \
    HERC_W_ZBAND=0.3 HERC_VISITED_3D=1 \
    HERC_SETTINGS=/home/lucas/hercules-sim/settings-temple-4drone.json \
    HERC_UE_MAP="/Game/AncientTempleRuins/Levels/L_Showcase_01" \
    HERC_ARENA_HALF_M=10 \
    HERC_KF_DEPTH_TOPIC=/sim/depth/image HERC_KF_INFO_TOPIC=/sim/depth/camera_info \
    ./capture_fleet_bagonly.sh "$lbl" 300
  echo "--- score $lbl ---"
  (cd /home/lucas/hercules-sim && python3 investigation/score_run.py "$lbl" 2>&1 | grep -E "SCORE|outwardness|VERDICT|G[1-4]:")
  sleep 10
done
echo CAMPAIGN_DONE
