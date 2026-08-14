#!/usr/bin/env bash
# fidelity_scorecard.sh — HERCULES sim2real fidelity scorecard.
# Runs against any sim run's logs and prints sim-vs-field per metric with PASS/WARN/FAIL.
#
# Usage:
#   ./fidelity_scorecard.sh <label> [logdir] [--live]
#     label   run label used in log filenames, e.g. coord4, fleet4, hw4, e1
#     logdir  default /home/lucas/hercules-sim/e1_frames
#     --live  additionally probe live ROS topics (run DURING a sim run; needs
#             sourced ROS 2 env + correct ROS_DOMAIN_ID per drone)
#
# Exit code = number of FAILed metrics (capped at 125).
#
# FIELD REFERENCE SOURCES (do not edit without re-verifying):
#   [F1] ghost_debug/run01/data/A_wifi_on/c_hz_vslam_odom.txt   -> vslam odom 30.05 Hz
#   [F2] ghost_debug/run01/data/A_wifi_on/c_hz_camera_imu.txt   -> united IMU 200.05 Hz
#   [F3] ghost_debug/run01/data/A_wifi_on/c_hz_fmu_in_vvo.txt   -> /fmu/in/vehicle_visual_odometry 29.94 Hz
#   [F4] AGENT_HANDOFF.md:207-208  -> D435i depth 60 fps, infra pair via splitter, IMU 200 Hz
#   [F5] AGENT_HANDOFF.md:230-233  -> vslam odom ~30 Hz, local_position ~50 Hz, status ~1 Hz
#   [F6] cuVSLAM frame-delta threshold 34 ms (logged by cuVSLAM itself; RUN-A-REPORT.md:41)
#   [F7] STUCK-BUG-INVESTIGATION.md:57 -> field stutters 66-851 ms cluster in STARTUP only;
#         steady state clean; EKF2_DELAY_MAX=200 ms
#   [F8] RADIO-V2-DESIGN.md:54 -> radio CYCLE_MS=190 (4 slots x 40 + 30 guard); claim is
#         atomic with pose, delivered within 1 cycle or persists to next (<=380 ms)
#   [F9] FINAL-SYNTHESIS.md:18 -> plan_cycle_budget_s=10.0 (fix c92b35a); field worst 105 s
#   [F10] nvblox_base.yaml:14-18 -> integrate_depth 40 Hz, update_esdf 10 Hz, mesh 5 Hz
#   [F11] ghost_debug/run01/data/A_wifi_on/tegrastats.txt -> per-drone Orin NX envelope:
#         busiest core ~77%@1420MHz, 3 cores ~20%@1420, 4 cores ~30%@729;
#         RAM 7.5/15.6 GB (base stack); GR3D 7-24%@305MHz; VDD_IN ~8.6 W
#         (=> per-drone CPU budget ~2.3 core-equivalents at Orin clocks)
#   [F12] tegrastats timestamps "12-31-1969" -> field Jetsons boot at epoch 0, NTP only ghost
#   [F13] AGENT_HANDOFF.md:216-217 -> EKF2_EV_CTRL=15, EKF2_HGT_REF=vision; field EKF2
#         ACCEPTS the EV stream (arms + flies Offboard)
#   [F14] ghost_debug/run01/data/A_wifi_on/c_topic_list.txt -> golden field topic shape
#   [F15] RUN-A-REPORT.md:41 -> field cuVSLAM expectation "34 ms"; sim coord4 measured
#         2.4-2.9 Hz stereo = the dominant gap

set -u
BASE=/home/lucas/hercules-sim
LABEL="${1:?usage: fidelity_scorecard.sh <label> [logdir] [--live]}"
LOGDIR="${2:-$BASE/e1_frames}"
LIVE=0
for a in "$@"; do [ "$a" = "--live" ] && LIVE=1; done
GOLDEN_TOPICS=$BASE/src/multirobot_launch_scripts/ghost_debug/run01/data/A_wifi_on/c_topic_list.txt

FAILS=0; WARNS=0
printf "%-38s %-16s %-16s %-7s %s\n" "METRIC" "FIELD_REF" "SIM" "RATIO" "STATUS"
printf "%.0s-" {1..90}; echo

row() { # metric field sim ratio status
  printf "%-38s %-16s %-16s %-7s %s\n" "$1" "$2" "$3" "$4" "$5"
  case "$5" in FAIL*) FAILS=$((FAILS+1));; WARN*) WARNS=$((WARNS+1));; esac
}

# duration of the run per drone, from first/last stamped line of the log
span_of() {
  grep -aoE '\[1[0-9]{9}\.[0-9]+\]' "$1" 2>/dev/null | tr -d '[]' | \
    awk 'NR==1{t0=$1} {t1=$1} END{ if(t0&&t1>t0) printf "%.1f", t1-t0 }'
}
cnt() { local c; c=$(grep -ac "$1" "$2" 2>/dev/null); echo "${c:-0}"; }

drones=$(ls "$LOGDIR"/rsnode_"${LABEL}"_*.log 2>/dev/null | sed -E "s/.*rsnode_${LABEL}_(.*)\.log/\1/")
[ -z "$drones" ] && drones=$(ls "$LOGDIR"/bridge_"${LABEL}"_*.log 2>/dev/null | sed -E "s/.*bridge_${LABEL}_(.*)\.log/\1/")
[ -z "$drones" ] && [ -f "$LOGDIR/bridge_${LABEL}.log" ] && drones="_single"

for d in $drones; do
  sfx="_${d}"; [ "$d" = "_single" ] && sfx=""
  BLOG=$LOGDIR/bridge_${LABEL}${sfx}.log
  RLOG=$LOGDIR/rsnode_${LABEL}${sfx}.log   # C++ sensor node (Q9) — authoritative
  PLOG=$LOGDIR/planner_${LABEL}${sfx}.log
  CLOG=$LOGDIR/cuvslam_${LABEL}${sfx}.log
  FLOG=$LOGDIR/fis_${LABEL}${sfx}.log
  KLOG=$LOGDIR/coord_${LABEL}${sfx}.log
  GLOG=$LOGDIR/guard_${LABEL}${sfx}.log
  DUR=$(span_of "$PLOG"); [ -z "$DUR" ] && DUR=$(span_of "$FLOG"); [ -z "$DUR" ] && DUR=190

  # ---- 1. stereo rate [F4][F15]: field 30 Hz (infra pair). pass>=27 warn>=15
  if [ -f "$RLOG" ] || [ -f "$BLOG" ]; then
    hz=""
    if [ -f "$RLOG" ]; then
      hz=$(grep -aoE 'stereo [0-9.]+ Hz' "$RLOG" | tail -1 | grep -aoE '[0-9.]+')
    fi
    pairs=$(grep -aoE 'stereo pairs=[0-9]+' "$BLOG" 2>/dev/null | tail -1 | grep -aoE '[0-9]+')
    if [ -z "$hz" ] && [ -n "${pairs:-}" ]; then
      hz=$(awk -v p="$pairs" -v t="$DUR" 'BEGIN{printf "%.2f", p/t}')
    fi
    if [ -n "${hz:-}" ]; then
      st=$(awk -v h="$hz" 'BEGIN{print (h>=27)?"PASS":(h>=15)?"WARN":"FAIL"}')
      row "stereo_rate_hz[$d]" "30.0 [F1,F4]" "$hz" "$(awk -v h=$hz 'BEGIN{printf "%.2f",h/30}')" "$st"
    fi
  fi

  # ---- 2. IMU rate [F2]: field 200 Hz. pass>=180 warn>=100
  if [ -f "$RLOG" ] || [ -f "$BLOG" ]; then
    hz=""
    if [ -f "$RLOG" ]; then
      hz=$(grep -aoE 'imu [0-9.]+ Hz' "$RLOG" | tail -1 | grep -aoE '[0-9.]+')
    fi
    imun=$(grep -aoE 'imu=[0-9]+' "$BLOG" 2>/dev/null | tail -1 | grep -aoE '[0-9]+')
    if [ -z "$hz" ] && [ -n "${imun:-}" ]; then
      hz=$(awk -v p="$imun" -v t="$DUR" 'BEGIN{printf "%.1f", p/t}')
    fi
    if [ -n "${hz:-}" ]; then
      st=$(awk -v h="$hz" 'BEGIN{print (h>=180)?"PASS":(h>=100)?"WARN":"FAIL"}')
      row "imu_rate_hz[$d]" "200.0 [F2]" "$hz" "$(awk -v h=$hz 'BEGIN{printf "%.2f",h/200}')" "$st"
    fi
  fi

  # ---- 3. cuVSLAM frame-delta stutter [F6][F7]: field = startup-only. Count
  #      violations AFTER first 30 s, per minute. pass<=2/min warn<=10/min
  if [ -f "$CLOG" ]; then
    read -r viol vmax <<<"$(awk -F'[][]' '
      /Delta between current/ { if(!t0)t0=$4+0; if($4+0 > t0+30){n++; if($8+0>mx)mx=$8+0} }
      END{printf "%d %.0f", n+0, mx+0}' "$CLOG")"
    vpm=$(awk -v n="$viol" -v t="$DUR" 'BEGIN{printf "%.1f", n/((t-30)/60)}')
    st=$(awk -v v="$vpm" 'BEGIN{print (v<=2)?"PASS":(v<=10)?"WARN":"FAIL"}')
    row "cuvslam_stutter_per_min[$d]" "<=2 (steady)[F7]" "$vpm (max ${vmax}ms)" "-" "$st"
  fi

  # ---- 4. VIO divergence believed-vs-truth [RUN-A §3: the only valid health metric]
  #      last meta JSON has ned truth + believed pos in same record. pass<1m warn<3m
  META_DIR=$(ls -d "$LOGDIR/${LABEL}_${d}" 2>/dev/null | head -1)
  if [ -n "${META_DIR:-}" ] && ls "$META_DIR"/meta_*.json >/dev/null 2>&1; then
    div=$(python3 - "$META_DIR" <<'EOF'
import json,glob,re,sys,math
f=sorted(glob.glob(sys.argv[1]+"/meta_*.json"))[-1]
m=json.load(open(f))
ned=m["ned"]; g=re.search(r"pos=\(([-0-9.]+),([-0-9.]+)\)", m.get("log",""))
if not g: print("NA"); sys.exit()
bx,by=float(g.group(1)),float(g.group(2))
# believed planner frame (x fwd, y left) vs AirSim NED (x N, y E): compare planar distance
print(f"{math.hypot(bx-ned[0], by-(-ned[1])):.1f}")
EOF
)
    if [ "$div" != "NA" ]; then
      st=$(awk -v v="$div" 'BEGIN{print (v<1)?"PASS":(v<3)?"WARN":"FAIL"}')
      row "vio_divergence_m[$d]" "<1.0" "$div" "-" "$st"
    fi
  fi

  # ---- 5. planner cycle time [F9]: budget 10 s; healthy p95 <= 1 s
  if [ -f "$PLOG" ]; then
    read -r p95 mx <<<"$(grep -aoE 'took [0-9]+ms' "$PLOG" | grep -aoE '[0-9]+' | sort -n | awk '{a[NR]=$1} END{if(NR)printf "%d %d", a[int(NR*0.95)+((NR*0.95==int(NR*0.95))?0:0)], a[NR]; else print "0 0"}')"
    st=$(awk -v p="$p95" -v m="$mx" 'BEGIN{print (p<=1000&&m<=10000)?"PASS":(m<=10000)?"WARN":"FAIL"}')
    row "planner_p95/max_ms[$d]" "<=1000/10000[F9]" "${p95}/${mx}" "-" "$st"
    # visited-set blackout signature (RUN-A bug #2): NO PATH burst bounded by ~60 s
    nps=$(cnt "NO PATH" "$PLOG")
    row "no_path_count[$d]" "sporadic" "$nps" "-" "$(awk -v n=$nps 'BEGIN{print (n<=10)?"PASS":(n<=60)?"WARN":"FAIL"}')"
  fi

  # ---- 6. FIS cycle [1 Hz timer; e1 field-comparable 14-48 ms on grid 401x401x61]
  if [ -f "$FLOG" ]; then
    fmx=$(grep -aoE ', [0-9.]+ ms' "$FLOG" | grep -aoE '[0-9.]+' | sort -n | tail -1)
    st=$(awk -v v="${fmx:-0}" 'BEGIN{print (v<=500)?"PASS":(v<=1000)?"WARN":"FAIL"}')
    row "fis_max_ms[$d]" "<=500 (1Hz)" "${fmx:-0}" "-" "$st"
  fi

  # ---- 7. coordination cycle [F8]: banner must say 190 ms
  if [ -f "$KLOG" ]; then
    cyc=$(grep -aoE 'cycle [0-9]+ ms' "$KLOG" | head -1 | grep -aoE '[0-9]+')
    row "coord_cycle_ms[$d]" "190 [F8]" "${cyc:-none}" "-" "$([ "${cyc:-0}" = "190" ] && echo PASS || echo FAIL)"
    # radio actually attached (RUN-A gate 2): silent no-port = no 'opened LoRa serial'
    if grep -aq "opened LoRa serial" "$KLOG"; then rst=PASS; else rst="FAIL(radio dead)"; fi
    row "lora_attached[$d]" "opened [RUN-A§5]" "$(grep -ac 'opened LoRa serial' "$KLOG")" "-" "$rst"
    # peer exchange liveness (F8 test): last peers{} line must show ok>0 for all peers
    okz=$(grep "peers {" "$KLOG" | tail -1 | grep -aoE 'ok=[0-9]+' | grep -aoE '[0-9]+' | awk '$1==0{z++} END{print z+0}')
    row "peer_exchange_dead_links[$d]" "0" "${okz:-NA}" "-" "$([ "${okz:-1}" = "0" ] && echo PASS || echo FAIL)"
  fi

  # ---- 8. guard depth staleness (field signature: 'Depth image stale (0.2s)')
  if [ -f "$GLOG" ]; then
    ds=$(cnt "Depth image stale" "$GLOG")
    row "depth_stale_events[$d]" "0" "$ds" "-" "$(awk -v n=$ds 'BEGIN{print (n==0)?"PASS":(n<=5)?"WARN":"FAIL"}')"
  fi

  # ---- 9. clock sanity [F12]: log epoch must be current era in sim (field boots 1969;
  #      replicating THAT is a separate spec — here we check stamps are monotonic+sane)
  if [ -f "$PLOG" ]; then
    ep=$(grep -aoE '\[[0-9]{9,10}\.' "$PLOG" | head -1 | grep -aoE '[0-9]+')
    st=PASS; [ "${ep:-0}" -lt 1000000000 ] && st="WARN(epoch<2001: 1969-boot mode?)"
    row "clock_epoch_sane[$d]" ">1e9" "${ep:-NA}" "-" "$st"
  fi
done

# ---- 10. PX4-in-loop metrics (hw runs): EV acceptance + offboard entry [F13]
for pxl in "$LOGDIR"/px4_${LABEL}*.log; do
  [ -f "$pxl" ] || continue
  n=$(basename "$pxl" .log)
  yaw=$(cnt "Preflight Fail: Yaw estimate error" "$pxl")
  miss=$(cnt "Preflight Fail: ekf2 missing data" "$pxl")
  armed=$(cnt "Armed by" "$pxl")
  # offboard command answered with 'Landing at current position' = rejection signature
  rej=$(grep -A2 "commander mode offboard" "$pxl" 2>/dev/null | grep -ac "Landing at current position"); rej=${rej:-0}
  if [ "$armed" -ge 1 ] && [ "$rej" -eq 0 ] && [ "$yaw" -eq 0 ]; then st=PASS
  elif [ "$armed" -ge 1 ]; then st="FAIL(EV rejected: yaw=$yaw rej=$rej)"
  else st="FAIL(never armed: miss=$miss)"; fi
  row "ekf2_ev_offboard[$n]" "accept [F13]" "armed=$armed yaw=$yaw" "-" "$st"
done

# ---- 11. zenoh mesh alive (RUN-A gate 1)
for zl in "$LOGDIR"/zenoh_${LABEL}*.log "$LOGDIR"/zenoh*_${LABEL}.log; do
  [ -f "$zl" ] || continue
  if grep -aq "panicked" "$zl"; then st="FAIL(panic)"; else st=PASS; fi
  row "zenoh_bridge[$(basename "$zl" .log)]" "no panic" "-" "-" "$st"
done

# ---- 12. host compute sidecar [F11]: per-drone Orin envelope ~2.3 core-equivalents,
#      RAM 7.5 GB. Needs a pidstat capture: run alongside the sim:
#      pidstat -h -r -u 5 -e ... > $LOGDIR/host_${LABEL}.pidstat  (see capture_host_load.sh)
if [ -f "$LOGDIR/host_${LABEL}.pidstat" ]; then
  # sum %CPU of each drone's process tree (matched by ROS_DOMAIN_ID env tag in cmdline)
  tot=$(awk '$8+0>0{s+=$8} END{printf "%.0f", s/100}' "$LOGDIR/host_${LABEL}.pidstat")
  row "host_cpu_cores_total" "<=9.2 (4x2.3)" "${tot}" "-" "$(awk -v t=$tot 'BEGIN{print (t<=9.2)?"PASS":(t<=14)?"WARN":"FAIL"}')"
else
  row "host_cpu_cores_total" "<=9.2 [F11]" "no sidecar" "-" "SKIP(capture host_${LABEL}.pidstat)"
fi

# ---- 13. live probes (--live): rates only measurable on a running system
if [ "$LIVE" = "1" ]; then
  for t in /visual_slam/tracking/odometry:/30:F1 /fmu/in/vehicle_visual_odometry:/29.9:F3 \
           /fmu/out/vehicle_local_position:/50:F5 /camera0/depth/image_rect_raw:/60:F4 \
           /camera0/imu:/200:F2; do
    top=${t%%:*}; ref=$(echo "$t" | cut -d: -f2 | tr -d /); src=${t##*:}
    hz=$(timeout 12 ros2 topic hz "$top" --window 100 2>/dev/null | grep -aoE 'average rate: [0-9.]+' | tail -1 | grep -aoE '[0-9.]+')
    if [ -n "${hz:-}" ]; then
      st=$(awk -v h="$hz" -v r="$ref" 'BEGIN{print (h>=0.9*r)?"PASS":(h>=0.5*r)?"WARN":"FAIL"}')
      row "live${top}" "$ref [$src]" "$hz" "$(awk -v h=$hz -v r=$ref 'BEGIN{printf "%.2f",h/r}')" "$st"
    else
      row "live${top}" "$ref [$src]" "silent" "-" "FAIL(no msgs)"
    fi
  done
  # topic-shape parity vs golden field list [F14]
  if [ -f "$GOLDEN_TOPICS" ]; then
    missing=$(comm -23 <(sort "$GOLDEN_TOPICS") <(ros2 topic list 2>/dev/null | sed -E 's|^/d[0-9]+||' | sort -u) | wc -l)
    row "topic_shape_missing" "0 [F14]" "$missing" "-" "$(awk -v m=$missing 'BEGIN{print (m==0)?"PASS":(m<=10)?"WARN":"FAIL"}')"
  fi
fi

printf "%.0s-" {1..90}; echo
# ---- fleet coverage + overlap (COORDINATION-REDESIGN.md A6) ----------------
# Four corrections over the original block, which unioned every drone's
# odom-frame track as if it were world frame (all four tracks start at (0,0),
# so the fleet looked like it launched from one point):
#   1. spawn offsets from the settings file used by the run are applied before
#      unioning, so cells are ARENA-relative;
#   2. a drone whose ground track leaves its own geofence by >2x is reported
#      as its own odom_divergence row and dropped from the overlap denominator
#      (an odometry runaway otherwise *lowers* reported overlap, so a VIO fix
#      reads as a coordination regression);
#   3. overlap_floor_pct = max(0, 1 - arena_cells/effort_cells) is reported and
#      graded against, because <10% is arithmetically impossible at the effort
#      typically flown into a 20 m arena;
#   4. fleet_overlap_concurrent_pct (cells hit by >=2 drones within 10 s) is the
#      coordination-attributable share; the spatial-only number conflates poor
#      assignment with "launched from the same line".
# HERC_SETTINGS overrides settings-file autodetection.
# HERC_ARENA_HALF_M overrides the planner bbox half-extent (default 10.0).
HERC_SETTINGS="${HERC_SETTINGS:-}" HERC_ARENA_HALF_M="${HERC_ARENA_HALF_M:-10.0}" \
python3 - "$LABEL" "$LOGDIR" "$BASE" <<'PYEOF' 2>/dev/null || true
import json, glob, sys, math, os

label, logdir, base = sys.argv[1], sys.argv[2], sys.argv[3]
ARENA_HALF = float(os.environ.get('HERC_ARENA_HALF_M') or 10.0)
DT_CONCURRENT_S = 10.0
DIVERGENCE_FACTOR = 2.0            # track outside 2x its own geofence = runaway
DISC = [(dx, dy) for dx in range(-4, 5) for dy in range(-4, 5)
        if dx * dx + dy * dy <= 16]


def row(metric, ref, sim, status):
    print("%-38s %-16s %-16s %-7s %s" % (metric, ref, sim, '-', status))


# ---- per-drone ground tracks (AirSim NED truth, relative to own spawn) ----
tracks = {}
for d in sorted(glob.glob('%s/%s_*/' % (logdir, label))):
    v = d.rstrip('/').split('_')[-1]
    pts = []
    for m in sorted(glob.glob(d + 'meta_*.json')):
        try:
            j = json.load(open(m))
        except Exception:
            continue
        ned = j.get('ned') or []
        if len(ned) < 2:
            continue
        pts.append((float(j.get('t', 0.0)), float(ned[0]), float(ned[1])))
    if pts:
        pts.sort()
        tracks[v] = pts
if not tracks:
    sys.exit(0)

# ---- spawn offsets: the settings file this run actually flew ----
cand = [os.environ['HERC_SETTINGS']] if os.environ.get('HERC_SETTINGS') else []
cand += sorted(glob.glob('%s/settings-fleet-*.json' % base))
spawn, settings_used = {}, None
for path in cand:
    try:
        veh = json.load(open(path)).get('Vehicles', {})
    except Exception:
        continue
    if set(tracks) <= set(veh):
        spawn = {k: (float(v.get('X', 0.0)), float(v.get('Y', 0.0)))
                 for k, v in veh.items()}
        settings_used = os.path.basename(path)
        break
if not spawn:
    row('fleet_spawn_offsets', 'settings json', 'NOT FOUND', 'FAIL(no spawn offsets)')
    spawn = {v: (0.0, 0.0) for v in tracks}
    settings_used = 'none'
else:
    row('fleet_spawn_offsets', 'settings json', settings_used, 'INFO')

# ---- divergence gate: its own row per drone, never a silent denominator edit --
# The geofence is +/-ARENA_HALF around each drone's OWN odom origin (= its
# spawn), so the excursion is measured on the raw, spawn-relative track.
drones = sorted(tracks)
included, excluded = [], []
for v in drones:
    worst = max(max(abs(x), abs(y)) for _t, x, y in tracks[v])
    if worst > DIVERGENCE_FACTOR * ARENA_HALF:
        excluded.append(v)
        row('odom_divergence_m[%s]' % v,
            '<=%.0f (%gx fence)' % (DIVERGENCE_FACTOR * ARENA_HALF, DIVERGENCE_FACTOR),
            '%.1f' % worst, 'FAIL(odom_divergence)')
    else:
        included.append(v)
        row('odom_divergence_m[%s]' % v,
            '<=%.0f (%gx fence)' % (DIVERGENCE_FACTOR * ARENA_HALF, DIVERGENCE_FACTOR),
            '%.1f' % worst, 'PASS')

# ---- arena: union of every drone's geofence, in arena cells ----
# All drones, not just the surviving ones: the arena is a property of the
# fleet's launch geometry, and keeping it fixed is what makes the headline
# comparable across runs that gate different drone sets.
arena = set()
for v in drones:
    ox, oy = spawn.get(v, (0.0, 0.0))
    for cx in range(int(ox - ARENA_HALF), int(ox + ARENA_HALF) + 1):
        for cy in range(int(oy - ARENA_HALF), int(oy + ARENA_HALF) + 1):
            arena.add((cx, cy))

# ---- coverage: 4 m disc around every arena-frame track sample ----
per_cells, per_times, per_all = {}, {}, {}
for v in drones:
    ox, oy = spawn.get(v, (0.0, 0.0))
    cells, times, allcells = set(), {}, set()
    for t, x, y in tracks[v]:
        # int() truncation, not floor(): keeps cell indices identical to the
        # pre-A6 block so the only change to the headline is the frame fix.
        cx = int(x + ox)
        cy = int(y + oy)
        for dx, dy in DISC:
            c = (cx + dx, cy + dy)
            allcells.add(c)
            if c in arena:
                cells.add(c)
                # visit INTERVALS [t_in, t_out], ascending (pts is time-sorted).
                # A sample within DT of the running interval extends it; a
                # later return opens a new one. Keeps the list short without
                # losing when the drone actually occupied the cell.
                tl = times.get(c)
                if tl is None:
                    times[c] = [[t, t]]
                elif t - tl[-1][1] > DT_CONCURRENT_S:
                    tl.append([t, t])
                else:
                    tl[-1][1] = t
    per_cells[v], per_times[v], per_all[v] = cells, times, allcells

def stats(names):
    eff = sum(len(per_cells[v]) for v in names)
    uni = set().union(*[per_cells[v] for v in names]) if names else set()
    ov = 100.0 * (1.0 - len(uni) / eff) if eff else 0.0
    fl = 100.0 * max(0.0, 1.0 - len(arena) / eff) if eff else 0.0
    return eff, uni, ov, fl


# Headline: ALL drones, spawn-corrected, clipped to the arena. Arena clipping
# already removes the runaway transit that inflated the old denominator
# (buckshee's 1705 cells at (150, 88) are simply not arena cells), so the
# headline stays measurable even when every drone trips the gate.
effort, unique, overlap, floor = stats(drones)
# Gated variant (the A6 literal): diverged drones out of the denominator.
g_effort, g_unique, g_overlap, g_floor = stats(included)

# raw (spawn-corrected, un-gated, un-clipped) for continuity with old runs
raw_effort = sum(len(c) for c in per_all.values())
raw_unique = set().union(*per_all.values()) if per_all else set()
raw_overlap = 100.0 * (1.0 - len(raw_unique) / raw_effort) if raw_effort else 0.0

# ---- concurrent overlap: cells two drones hit within DT_CONCURRENT_S ----
conc = 0
for c in unique:
    holders = [per_times[v][c] for v in drones if c in per_times[v]]
    if len(holders) < 2:
        continue
    hit = False
    for i in range(len(holders)):
        for j in range(i + 1, len(holders)):
            for a in holders[i]:
                for b in holders[j]:
                    # gap between two occupancy intervals (<=0 = overlapping)
                    if max(a[0], b[0]) - min(a[1], b[1]) <= DT_CONCURRENT_S:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                break
        if hit:
            break
    if hit:
        conc += 1
conc_pct = 100.0 * conc / len(unique) if unique else 0.0

for v in drones:
    row('coverage_m2[%s]' % v, '>=600 (solo)', '%d' % len(per_cells[v]),
        'PASS' if len(per_cells[v]) >= 600 else 'WARN')
row('fleet_arena_cells', 'geofence union', '%d' % len(arena), 'INFO')
row('fleet_effort_m2', 'in-arena', '%d' % effort, 'INFO')
row('fleet_unique_m2', 'maximize', '%d' % len(unique), 'INFO')
row('overlap_floor_pct', '1-arena/effort', '%.1f' % floor, 'INFO')
st = ('PASS' if overlap <= floor + 5.0
      else 'WARN' if overlap <= floor + 15.0 else 'FAIL')
row('fleet_overlap_pct', '<=floor+5', '%.1f' % overlap, st)
row('fleet_overlap_concurrent_pct', 'minimize (dt<=10s)', '%.1f' % conc_pct, 'INFO')
row('fleet_overlap_raw_pct', 'ungated/unclipped', '%.1f' % raw_overlap, 'INFO')
if excluded:
    row('fleet_drones_excluded', '0', ','.join(excluded), 'WARN')
    if len(included) >= 2:
        row('fleet_unique_m2_gated', 'maximize', '%d' % len(g_unique), 'INFO')
        row('fleet_overlap_gated_pct', 'floor %.1f' % g_floor,
            '%.1f' % g_overlap, 'INFO')
    else:
        row('fleet_overlap_gated_pct', 'needs >=2 drones',
            '%d valid' % len(included), 'WARN(unmeasurable)')
PYEOF
echo "RESULT: $FAILS FAIL, $WARNS WARN  (label=$LABEL logdir=$LOGDIR)"
exit $((FAILS>125?125:FAILS))

