#!/usr/bin/env bash
# capture_host_load.sh <label> [logdir] — run ALONGSIDE a sim run to produce the
# host_${label}.pidstat sidecar that fidelity_scorecard.sh's compute-parity metric reads.
# Field reference: one Orin NX per drone = busiest core 77%@1420MHz, RAM 7.5/15.6 GB
# (ghost_debug/run01/data/A_wifi_on/tegrastats.txt). Kill with Ctrl+C or when the run ends.
LABEL="${1:?usage: capture_host_load.sh <label> [logdir]}"
LOGDIR="${2:-/home/lucas/hercules-sim/e1_frames}"
exec pidstat -h -u -r 5 > "$LOGDIR/host_${LABEL}.pidstat"
