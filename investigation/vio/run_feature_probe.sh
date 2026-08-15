#!/usr/bin/env bash
# R3 validation: bring up Blocks + AirSim, fly ghost to the 8 diagnostic waypoints, capture
# stereo+depth, and score Shi-Tomasi / usable-disparity / grid occupancy.
#
#   run_feature_probe.sh <tag>      -> frames scored, copies saved to frames_<tag>/
set -e
TAG=${1:-textured}
BASE=/home/lucas/hercules-sim
FP=$BASE/investigation/feature_probe
LOG=$BASE/e1_frames
# apt cv2 (4.5.4) is built against numpy 1.x while pip numpy 2.2.6 shadows it; this venv
# pins numpy<2 with --system-site-packages so `import cv2` works again.
PROBE_PY=/home/lucas/UE5/hercules-sim-big/venv_probe/bin/python

pkill -f 'UnrealEditor.*[B]locks' 2>/dev/null || true
for i in $(seq 1 30); do pgrep -f 'UnrealEditor.*[B]locks' >/dev/null || break; sleep 2; done
pkill -9 -f 'UnrealEditor.*[B]locks' 2>/dev/null || true
while ss -ltn | grep -q 41451; do sleep 2; done

cd "$BASE/HERCULES/Unreal/Environments/Blocks"
/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "$PWD/Blocks.uproject" \
  -game -RenderOffscreen -windowed -ResX=1280 -ResY=720 \
  -settings=$BASE/settings-fleet-4drone.json -log -stdout -unattended -nosplash \
  > $LOG/ue_probe_$TAG.log 2>&1 &
cd "$BASE"
for i in $(seq 1 60); do ss -ltn | grep -q 41451 && break; sleep 3; done
ss -ltn | grep -q 41451 || { echo "UE failed"; exit 1; }
sleep 5
echo "airsim up"

rm -f "$FP"/frames/*.npy
"$BASE/venv/bin/python" "$FP/capture_waypoints.py"
"$PROBE_PY" "$FP/measure_features.py" | tee "$FP/metrics_$TAG.txt"
cp -a "$FP/frames" "$FP/frames_$TAG" 2>/dev/null || { rm -rf "$FP/frames_$TAG"; cp -a "$FP/frames" "$FP/frames_$TAG"; }
pkill -f 'UnrealEditor.*[B]locks' 2>/dev/null || true
echo "probe $TAG done -> $FP/metrics_$TAG.txt"
