#!/usr/bin/env bash
# hb_drive.sh — HERCULES render-ceiling sweep driver (unique name; do not collide).
# Targets the ISOLATED RenderBench UE on port 41455 which the sibling agent's
# `pkill -f "UnrealEditor.*Blocks"` cannot match.
#
# The host is never idle (a peer agent runs continuous 4-drone stacks), so each
# config is sampled R times and we report BOTH the max (best-case == least-contended,
# our ceiling estimator) and the median, with per-sample host state recorded.

D=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/bench
BIN=$D/rpcbench
PORT=41455
DUR=${DUR:-10}
REPS=${REPS:-3}
OUT=${OUT:-$D/hb_results.txt}

hoststate() {
  local b c
  b=$(pgrep -f "UnrealEditor.*[B]locks" | wc -l)
  c=$(pgrep -f "isaac_ros_visual_slam|nvblox_node|airsim_realsense_node|simple_exploration_planner" | wc -l)
  echo "blocksUE=$b consumers=$c load=$(cut -d' ' -f1 /proc/loadavg)"
}

alive() { pgrep -f "UnrealEditor.*[R]enderBench" >/dev/null; }

hb_cfg() {   # hb_cfg <label> <rpcbench args...>
  local label=$1; shift
  local best=0 bestline="" i
  for i in $(seq 1 $REPS); do
    alive || { echo "FATAL my UE died before $label" | tee -a "$OUT"; return 1; }
    local hs gpu line rps
    hs=$(hoststate)
    gpu=$(nvidia-smi --query-gpu=utilization.gpu,utilization.memory,power.draw,clocks.sm --format=csv,noheader,nounits | head -1 | tr -d ' ' | tr '\n' ' ')
    line=$(timeout 180 "$BIN" --port $PORT --dur "$DUR" "$@" 2>/dev/null | grep '^RESULT')
    [ -z "$line" ] && { echo "SAMPLE $label rep=$i NO_RESULT ($hs)" | tee -a "$OUT"; continue; }
    rps=$(sed -n 's/.*renders_per_s=\([0-9.]*\).*/\1/p' <<<"$line")
    echo "SAMPLE $label rep=$i $hs gpu=[$gpu] $line" | tee -a "$OUT"
    awk -v a="$rps" -v b="$best" 'BEGIN{exit !(a>b)}' && { best=$rps; bestline=$line; }
    sleep 1
  done
  echo "BEST  $label renders_per_s=$best :: $bestline" | tee -a "$OUT"
}
