#!/usr/bin/env bash
# R3: run texturize.py as a UE5.2.1 python commandlet against the Blocks project.
# Foreground (a backgrounded UE launch can fail to create its redirect targets).
set -e
B=/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks
UE=/home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor
OUT=${1:-/tmp/texturize.log}

# Only Blocks -- the bracket keeps the pattern from matching this script's own argv, and it
# must not match the unrelated UE5.6 SwarmReplay editor another workload is running.
pkill -f 'UnrealEditor.*[B]locks' 2>/dev/null || true
for i in $(seq 1 30); do pgrep -f 'UnrealEditor.*[B]locks' >/dev/null || break; sleep 2; done
pkill -9 -f 'UnrealEditor.*[B]locks' 2>/dev/null || true
while ss -ltn | grep -q 41451; do sleep 2; done

# Back up the two UMaterials that are actually edited (see texturize.py's corrections:
# GrayMaterial is only an instance of BaseMaterial), plus GrayMaterial itself for safety.
for f in "$B/Content/Flying/Meshes/BaseMaterial.uasset" \
         "$B/Content/Flying/Meshes/GrayMaterial.uasset" \
         "$B/Content/Geometry/Meshes/CubeMaterial.uasset"; do
  [ -f "$f.bak" ] || cp -a "$f" "$f.bak"
done
echo "backups: $(ls -la "$B"/Content/Flying/Meshes/*.bak "$B"/Content/Geometry/Meshes/*.bak | wc -l)"

cd "$B"
"$UE" "$PWD/Blocks.uproject" \
  -run=pythonscript -script=/home/lucas/hercules-sim/investigation/vio/texturize.py \
  -unattended -nosplash -RenderOffscreen -stdout > "$OUT" 2>&1 || true
echo "--- texturize output ---"
grep -E "texturize|LogPython|Error|error" "$OUT" | head -40
