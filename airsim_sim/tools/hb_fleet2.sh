#!/usr/bin/env bash
D=/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/bench
DUR=${DUR:-10}; SHZ=${SHZ:-0}; DHZ=${DHZ:-0}
S=$(mktemp); T=$(mktemp)
"$D/rpcbench" --port 41455 --dur "$DUR" --clients 4 --mode stereo --compress 0 --hz "$SHZ" > "$S" 2>/dev/null &
p1=$!
"$D/rpcbench" --port 41455 --dur "$DUR" --clients 4 --mode depth  --compress 0 --hz "$DHZ" > "$T" 2>/dev/null &
p2=$!
wait $p1 $p2
g(){ sed -n "s/.*$2=\([0-9.]*\).*/\1/p" "$1"; }
awk -v sr="$(g $S renders_per_s)" -v dr="$(g $T renders_per_s)" -v sc="$(g $S calls_per_s)" \
    -v dc="$(g $T calls_per_s)" -v sl="$(g $S lat_p50)" -v dl="$(g $T lat_p50)" -v sh="$SHZ" -v dh="$DHZ" 'BEGIN{
 printf "target(stereo=%sHz,depth=%sHz) total_rps=%.1f | per_drone_stereo=%.2fHz per_drone_depth=%.2fHz | s_lat_p50=%.1f d_lat_p50=%.1f\n",
  (sh=="0"?"max":sh),(dh=="0"?"max":dh), sr+dr, sc/4, dc/4, sl, dl}'
rm -f "$S" "$T"
