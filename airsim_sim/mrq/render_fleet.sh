#!/usr/bin/env bash
# MRQ pipeline stages A (sequence bake) + B (headless -game render).
#
# Stage A: UnrealEditor-Cmd -ExecutePythonScript=build_fleet_seq.py  (nullrhi)
# Stage B: UnrealEditor-Cmd MoviePipelineEntryMap -game \
#            -MoviePipelineConfig="MovieRenderPipeline/QueueManifest.utxt"
#          (manifest path is relative to the project Saved dir; the CLI bootstrap
#           in MovieRenderPipelineCoreModule picks it up with the in-process
#           executor and quits when done.)
#
# usage: render_fleet.sh <keys.json> <out_frames_dir> [TEST] [TEST_START] [TEST_END]
set -e
KEYS=${1:?keys json}
OUT=${2:?frames out dir}
TEST=${3:-0}
TEST_START=${4:-600}
TEST_END=${5:-615}

UE=/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux
PROJ=/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks
W=/home/lucas/UE5/hercules-sim-big/mrq_work

# refuse to fight a live AirSim run
if ss -ltn | grep -q 41451; then echo "AirSim RPC port busy - a sim run is active; refusing"; exit 1; fi

mkdir -p "$OUT"

echo "=== STAGE A: bake sequence + manifest ==="
rc=1
for attempt in 1 2 3; do
  rm -f "$W/ue/stageA_progress.txt"
  KEYS_JSON=$KEYS OUT_DIR=$OUT TEST=$TEST TEST_START=$TEST_START TEST_END=$TEST_END \
  "$UE/UnrealEditor-Cmd" "$PROJ/Blocks.uproject" \
    -ExecutePythonScript="$W/ue/build_fleet_seq.py" \
    -nullrhi -stdout -unattended -nosplash > "$W/ue/stageA.log" 2>&1 || true
  if grep -q "BUILD_OK" "$W/ue/stageA_progress.txt" 2>/dev/null; then rc=0; break; fi
  rc=1
  echo "  stage A attempt $attempt failed - retrying (UE cold-start segfaults ~50%)"
  sleep 5
done
cat "$W/ue/stageA_progress.txt" 2>/dev/null
grep -E "LogPython: Error" "$W/ue/stageA.log" | tail -15
[ $rc -eq 0 ] || { echo "STAGE A FAILED"; exit 1; }

echo "=== STAGE B: MRQ render ==="
rc=1
for attempt in 1 2 3; do
  "$UE/UnrealEditor-Cmd" "$PROJ/Blocks.uproject" MoviePipelineEntryMap \
    -game -MoviePipelineConfig="MovieRenderPipeline/QueueManifest.utxt" \
    -RenderOffscreen -windowed -ResX=1920 -ResY=1080 \
    -log -stdout -unattended -nosplash -notexturestreaming > "$W/ue/stageB.log" 2>&1 || true
  n=$(ls "$OUT"/frame_*.png 2>/dev/null | wc -l)
  echo "  stage B attempt $attempt frames=$n"
  [ "$n" -gt 0 ] && { rc=0; break; }
  rc=1
  sleep 5
done
grep -E "LogMovieRenderPipeline|Error:" "$W/ue/stageB.log" | tail -25
[ $rc -eq 0 ] || { echo "STAGE B FAILED"; exit 1; }
echo "RENDER_OK $(ls "$OUT"/frame_*.png | wc -l) frames in $OUT"
