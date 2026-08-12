# Fleet Frontier-Exploration Stall — Root-Cause Synthesis & Sim Repro Plan

Synthesized 2026-08-12 from the ghost-debug, current-launchers, cross-drone-diff, git-forensics, hercules-sim, and px4-params fact sheets. Symptom under investigation: **drone arms in Position, enters Offboard, explores, then gets stuck with errors while coordinating over zenoh + LoRa.**

Ranking basis: (a) how precisely the mechanism's trigger time matches "works until coordination starts", (b) verification strength (CONFIRMED = file-evidenced; PLAUSIBLE = evidence real, mechanism partly inferred), (c) blast radius. Note the one fully debugged prior incident (ghost, June) had a *from-boot* failure signature; the current symptom is *mid-flight*, so causes that fire at coordination onset outrank statically-broken-config causes.

One contradiction between fact sheets was resolved during synthesis: POSTMORTEM.md's claim that thunderstrike sets `ROS_LOCALHOST_ONLY=1` refers to the **archived** launcher (`archive/launch_thunderstrike_jetson_eduroam_python_frontierexplore.sh:21`); the current tmux launcher sets `=0` like the other three.

---

## 1. Ranked candidate root causes

### R1. radiohive lora_bridge opens /dev/ttyUSB0 (57600) under the PX4 XRCE agent (921600) — serial link death at coordination onset  [PLAUSIBLE — hazard CONFIRMED in docs, occurrence inferred]

**Mechanism.** radiohive's default serial port is `/dev/ttyUSB0 @57600` — the same device MicroXRCEAgent holds `@921600`. Linux serial opens are non-exclusive: the second open succeeds, reprograms the baud, and both processes interleave reads. XRCE framing corrupts → PX4 `Payload rx` stops → EKF2 loses EV, offboard_control_mode/trajectory_setpoint stream dies at the FMU → offboard failsafe / position-invalid hold → "stuck with errors" exactly when the LoRa link comes up. Secondary variant: with LoRa **hardware now attached** (the June docs said it wasn't; the user's symptom says it is used), two USB-serial devices make `/dev/ttyUSB0`/`ttyUSB1` enumeration nondeterministic across boots — the agent can open the LoRa radio and radiohive the FMU, garbling both links (inferred). Corroboration from params: `MAV_0_CONFIG=101 (TELEM1) @ SER_TEL1_BAUD=57600` — the LoRa radio is meant to be a 57600 MAVLink radio on the FMU's TELEM4-adjacent wiring, so the host-side radiohive default port reads like a mis-target of the FMU's USB-serial.

**Evidence.** AGENT_HANDOFF.md:252 ("default serial port /dev/ttyUSB0 @57600 would steal the PX4 link if ever launched"; also "Radio hardware NOT attached" as of that snapshot); AGENT_HANDOFF.md:394-395 health check "lsof /dev/ttyUSB0 — exactly one holder"; fleet_ctl:242 pkills `[l]ora_bridge` and :232 greps `loratest` sessions (it is anticipated to run); no launcher chmods/lsofs ttyUSB0 (only archive/run_delta.sh:31); px4_params VIO15: UXRCE on TELEM4 @921600 + MAV_0 @57600. Timing question (is lora_bridge started at bringup or at coordination phase?) is private-repo (fleet_bringup.launch.py / radiohive) — see §3 Q1, Q13.

**Sim repro (HERCULES).** Run PX4 SITL's XRCE transport through a socat pty pair (agent on `ptyA`); mid-exploration, have a second process open the same pty at 57600 and write LoRa-shaped traffic. Observe: XRCE session drop, /fmu/out silence, offboard failsafe, planner still publishing. Cheap and decisive. (Under SimpleFlight-only, emulate as: kill the VVO+setpoint relay to the "FMU" stand-in a few seconds after a "coordination start" marker.)

**Real-drone instrumentation.** `lsof /dev/ttyUSB0 /dev/ttyUSB1` logged every 5 s to a file from launch through failure; `udevadm info` persistent IDs for both USB-serial devices (then pin radiohive and the agent to `/dev/serial/by-id/...` paths); agent console `Payload rx` counters; timestamp of lora_bridge start vs first error.

---

### R2. Zenoh bridge lifecycle churn: kill-before-start on every window re-attach, two coexisting bridge mechanisms, stranded mesh, and a possibly wrong-domain tab8 bridge  [PLAUSIBLE — operational defects CONFIRMED, coordination-node reaction inferred]

**Mechanism.** Four confirmed defects converge on "peer topics vanish/reappear or silently never arrive while coordinating":
1. **Re-attach bounce (fleet path):** `launch_fleet_tmux.sh:51-53` prepends the ZENOH string (which begins `tmux kill-session -t zenoh; ... pkill -f [z]enoh-bridge-ros2dds; sleep 1; new-session ...`, fleet_ctl:156-161) unconditionally, *before* the `tmux has-session -t fleet ||` guard. The documented recovery for an SSH drop (common on field WiFi) is to re-run the window — every Enter bounces the live bridge mid-flight with fresh DDS discovery.
2. **Cleanup mismatch:** fleet_ctl stop's session grep `^(fleet|zenoh|rviz|loratest)` (fleet_ctl:232) misses `gh_/dl_/bk_/ts__mesh_zenoh`; single-drone `kill` = `tmux kill-server` SIGHUPs the mesh script past its Ctrl+C WiFi-restore path — drone left on 802.11s with the bridge dead (silent blackhole: mesh up, TCP 7447 unreachable) or bounced to LAN while peers stay on mesh.
3. **Two bridge mechanisms coexist:** bench zenoh (fleet_ctl) and mesh zenoh (tab8) can run simultaneously; nothing prevents it.
4. **tab8 domain gap:** tab8 runs `sudo bash launch_mesh_zenoh.sh $NODE_ID $PEER_IDS` with **no ROS_DOMAIN_ID in env** (launcher line 165), while fleet_ctl deliberately uses `sudo -S env ROS_DOMAIN_ID=<vid>` — if launch_mesh_zenoh.sh (host script, not in repo) doesn't derive the domain from NODE_ID, the bridge sits on domain 0 and bridges *nothing* from the domain-1..4 container stacks: coordination silently empty, loop_closure/octomap nodes wait forever. NODE_ID/PEER_IDS are free-text operator prompts — typo-prone.

If loop_closure_sp_node/octomap_exchange block on peer responses or the planner waits on merged-map updates (private-repo behavior, inferred), any of these wedges exploration precisely "while coordinating over the zenoh bridge".

**Evidence.** launch_fleet_tmux.sh:8-15, 51-53; fleet_ctl:149-161, 215-220, 226-246; launch_ghost_frontierexplore_tmux.sh:155-174, 179-183; AGENT_HANDOFF.md:35, 175-179 (mesh switch + bridge allowlist; broken eduroam restore); cross-drone-diff fact on tab8's missing env.

**Sim repro (HERCULES).** Two-drone sim, two ROS domains on this box, two `zenoh-bridge-ros2dds` over loopback TCP 7447. Fault schedule: (a) SIGKILL+restart one bridge with 1 s gap at random intervals; (b) kill one bridge and leave it dead (blackhole); (c) start one bridge on domain 0 (empty bridge). Observe whether stand-in coordination consumers and the exploration loop recover, wedge, or error — match the field signature.

**Real-drone instrumentation.** Bridge PID + start-time logging (`/tmp/zenoh_tmux.log` already exists — add timestamps); `ss -tnp | grep 7447` on both drones each 5 s; count zenoh session restarts vs the stuck timestamp; log every window re-attach (wrap launch_fleet_tmux.sh to append to a log); on tab8 runs, capture `env` of the bridge process to check its effective domain.

---

### R3. Offboard keepalive gap > 1 s → PX4 silently demotes to Position and never re-enters — the terminal common pathway (+ RC-loss → RTL-at-1.5 m trap)  [Params CONFIRMED; trigger inferred]

**Mechanism.** `COM_OF_LOSS_T=1.0 s` with `COM_OBL_RC_ACT=0`: any ≥1 s gap in reactive_depth_guard's `/fmu/in/offboard_control_mode`+`trajectory_setpoint` stream (guard crash, CPU stall, DDS hiccup, serial glitch — R1/R6/R7/R8 all funnel here) drops PX4 to POSCTL, and **PX4 never re-enters Offboard by itself**. Unless the planner/guard detects nav_state change and re-commands Offboard (private repo, unknown — §3 Q4), the drone hovers forever while the planner keeps planning: the exact "stuck" phenotype. Separately, `COM_RCL_EXCEPT=0` + `NAV_RCL_ACT=2` + `COM_RC_LOSS_T=0.5 s`: an RC dropout at field range during Offboard triggers RTL — and VIO15 sets `RTL_RETURN_ALT=RTL_DESCEND_ALT=1.5 m`, so the drone silently flies home at 1.5 m *through the unexplored environment* (reads as "stuck/veered off with errors"). Coordination onset is a plausible trigger for the 1 s gap because tab6 SuperPoint+LightGlue and octomap exchange spin up right then on the same 16 GB Orin (see R4).

**Evidence.** px4_params VIO15/all: COM_OF_LOSS_T=1.0 (VIO15:289), COM_OBL_RC_ACT=0 (:287), COM_RC_LOSS_T=0.5, NAV_RCL_ACT=2 (:674), COM_RCL_EXCEPT=0 (:298), RTL_RETURN_ALT/DESCEND_ALT=1.5 (VIO15:884,889 vs 30/10 in STABLE); COM_POS_FS_DELAY=1 s / EPH 5 m / EVH 1 m/s; reactive_depth_guard = sole /fmu/in writer (background + AGENT_HANDOFF.md:249-251).

**Sim repro (HERCULES).** PX4 SITL with the VIO15 failsafe params loaded. During sim exploration: (a) pause the setpoint stand-in for 0.8 / 1.2 / 3 s and confirm nav_state 14→2 transition, no self-recovery; (b) simulate RC loss (SITL `param set NAV_RCL_ACT 2` + manual_control timeout) mid-Offboard and watch the 1.5 m RTL path clip obstacles in Blocks. Record the /fmu/out/vehicle_status transition signature to compare against field ulogs.

**Real-drone instrumentation.** Log `/fmu/out/vehicle_status.nav_state` + `failsafe` + `nav_state_user_intention` continuously (this alone will classify most field stalls); `ros2 topic hz` on `/fmu/in/trajectory_setpoint` recorded to file; pull the FMU .ulg after every stuck event and check commander mode-change + failsafe messages; RC link RSSI at the stuck timestamp.

---

### R4. Coordination-onset compute burst on the Orin degrades cuVSLAM/VIO → EKF2 EV timeout/reset mid-Offboard  [PLAUSIBLE]

**Mechanism.** At coordination start, SuperPoint+LightGlue (tab6), octomap exchange, and zenoh serialization join nvblox+cuVSLAM+FIS+planner on one Orin NX 16 GB. Bench captures already show cuVSLAM frame-delta stutters (66-851 ms vs desired 34 ms), "Lost IMU" bursts, and one hard `CUVSLAM_INVALID_ARG` non-monotonic-timestamp error even in the *passing* baseline — though verification showed those cluster in the startup/USB-contention window, not steady state. If load grows gaps past `EKF2_DELAY_MAX=200 ms` or triggers an EV reset, xy_valid can flip false mid-Offboard → position failsafe → hold. Also feeds R3 (a >1 s guard stall under the same load). RAM headroom is thin: 7.5 GB used with only the VIO stack; a duplicate-stack event (R7) took it to 12.6/15.6 GB.

**Evidence.** run03/data/tab1_pane.txt:325-345 (stutters, CUVSLAM_INVALID_ARG); run03/data/hz_vvo.txt (max gap 0.061 s healthy); A_wifi_on/tegrastats.txt vs B_radio_off/tegrastats.txt; EKF2_DELAY_MAX=200, EKF2_NOAID_TOUT=5 s, EKF2_EV_QMIN=0 (no quality gate — garbage VIO is fused, not rejected) from px4_params.

**Sim repro (HERCULES).** Don't simulate the Orin — inject the *effect*: a VVO relay node that (a) inserts 100-500 ms gaps, (b) occasionally emits a backwards timestamp, (c) rate-halves for 5 s, during the coordination phase. Against PX4 SITL with VIO15 EKF2 params, observe EV rejection/reset, local_position validity flags, and guard/planner reaction. Sweep gap length to find the cliff.

**Real-drone instrumentation.** `tegrastats` + per-process CPU/GPU (`top -b`, `nvidia-smi`-equivalent jtop log) time-aligned with the stuck event; `ros2 topic hz /visual_slam/tracking/odometry` and `/fmu/in/vehicle_visual_odometry` logged continuously; count cuVSLAM "Delta between frame" warnings per minute before vs after coordination start; EKF2 estimator_status flags from the ulog.

---

### R5. Cross-drone clock skew (three drones boot at 1969, NTP only on ghost) breaks stamped coordination data  [Precondition CONFIRMED; consumer behavior inferred]

**Mechanism.** BENCH_INTERNET.md: bench-internet.service (NTP) is installed on ghost only ("others TODO"); Jetsons cold-boot to 1969 and NTP was deliberately disabled on ghost during debugging at one point. So in a fleet sortie, drone A stamps ROS msgs near epoch-0/boot-time while drone B stamps 2026 wall-time. Everything *within* one drone is consistent, but the moment loop_closure_sp_node (publish_tf:=True) and octomap_exchange trade stamped data/TF over zenoh, cross-drone stamp comparisons, TF buffer lookups, and PCM windows are off by ~56 years → immediate "errors while coordinating" (TF extrapolation errors, rejected matches, or worse: silently empty association). Whether the coordination nodes compare cross-drone stamps is private-repo (§3 Q11), but this is the single cleanest match to "errors appear exactly when coordination starts, multi-drone only".

**Evidence.** BENCH_INTERNET.md:21 (ghost only); A_wifi_on/meta.txt (`date` = Dec 31 1969, uptime 51 min); run03/data/tab2_pane.txt:1 (ROS log dir 1970-01-01); FINDINGS.md R3 (NTP disabled); tab6/tab7 launch args (publish_tf:=True, use_pcm:=True, use_sim_time:=False).

**Sim repro (HERCULES).** Two-domain sim fleet; run one drone's coordination stand-ins (or the real private nodes once available) under `libfaketime` pinned to 1970 while the other uses wall clock; bridge with zenoh; observe TF/stamp errors and whether the exploration loop on the *healthy-clock* drone stalls.

**Real-drone instrumentation.** `date +%s` from all four drones logged at sortie start (one ssh loop); compare stamp fields of received peer octomap/loop-closure msgs vs local clock inside the container (`ros2 topic echo --once` on a peer topic and diff header.stamp vs now); install bench-internet.service on the other three (the fix is already written for ghost).

---

### R6. UXRCE_DDS_DOM_ID vs ROS_DOMAIN_ID mismatch on the three never-verified drones (+ param-restore regression trap) → dead teammate wedges coordination  [Mechanism CONFIRMED on ghost; fleet-wide state unverified]

**Mechanism.** The June ghost bug: the agent creates DDS entities on the domain the *PX4 client* requests (`UXRCE_DDS_DOM_ID`, default 0), ignoring container env; script exports domain 3 → /fmu/* invisible → EKF2 never gets VIO. Only ghost's FMU param was fixed. Delta/buckshee/thunderstrike carry "VERIFY/SET in QGC before flying!" warnings, and **all three reference param dumps in px4_params/ still contain `UXRCE_DDS_DOM_ID=0`** — any FMU restored from the canonical params silently regresses (including ghost). A mismatched drone fails *from boot* (can't hold Position), which doesn't match the flying drone's symptom — but in a fleet sortie a mismatched *teammate* never shows up on zenoh, and if frontier assignment/loop closure waits on the peer, the healthy drone "gets stuck while coordinating" (peer-wait behavior is private-repo, §3 Q10).

**Evidence.** POSTMORTEM.md:16-27, 56-64; FINDINGS.md:22 (run03 PASS after fix); launch_delta_frontierexplore_tmux.sh:39-41; px4_params VIO15:1077 `UXRCE_DDS_DOM_ID 0`.

**Sim repro (HERCULES).** Two SITL drones: set drone B's `UXRCE_DDS_DOM_ID` to 0 while its ROS stack runs domain 2. B never publishes /fmu/out on domain 2 (verify), stays grounded; watch whether the coordination stand-ins on drone A stall or degrade gracefully.

**Real-drone instrumentation.** One-time audit: `param show UXRCE_DDS_DOM_ID` via QGC/mavlink shell on all four FMUs vs launcher domain (1/2/3/4); add the check to a preflight script; never re-flash from the July dumps without patching the param.

---

### R7. Stale/duplicate process contamination across relaunches (dead-shell attach, straggler nodes, double agents/guards, USB busy)  [Phenomenon CONFIRMED; link to stall inferred]

**Mechanism.** Multiple confirmed teardown gaps mean "what is actually running" diverges from what the operator believes: `tmux has-session ||` re-attaches to dead shells (stack looks up, planner not running → exploration never advances after Offboard); tmux kill alone leaves docker-exec'd processes alive holding /dev/ttyUSB0 (clean_ghost.sh:2-3); for ~4 days every fleet stop left one straggler per drone (self-matching pkill, fixed 2a43776); current stop patterns miss `octomap_exchange_node`/`odom_correction` (no `.py`, not in a container matched by the sweep); fleet_ctl start does no pre-start sweep, so mixing launcher families yields two MicroXRCEAgents on ttyUSB0 or two setpoint writers (PX4 consumes last-writer → interleaved conflicting setpoints → hover-stuck/oscillation); run01-B showed the end state: two full stacks, camera unclaimable, RAM 12.6/15.6 GB. This class generates the *intermittency* ("depends what survived the last kill").

**Evidence.** B_radio_off/c_pgrep.txt (two PID sets incl. two vio_bridges); run03/data/tab1_pane.txt:44-62 (RS2_USB_STATUS_BUSY); fleet_ctl:193, 201-205, 230-246; commits 4859cd3, 2a43776, 2ba2258; FINDINGS.md:44-46.

**Sim repro (HERCULES).** (a) Two concurrent setpoint stand-ins with slightly different targets against one SITL → observe oscillation/hold; (b) "planner dead but session alive": stop the exploration stand-in right after Offboard entry and record the do-nothing signature (hover at first setpoint) to compare with field behavior.

**Real-drone instrumentation.** Preflight assertion script (run before every arm): exactly one MicroXRCEAgent, one vio_bridge, one guard, one planner (`pgrep -af` with bracket patterns), exactly one holder of /dev/ttyUSB0, zero `Tab_*`/`fleet*` tmux sessions from prior runs, RAM < 9 GB. Log its output with the sortie.

---

### R8. MicroXRCEAgent DDS binding unpinned (lost wrapper) + WiFi IP change under bound sockets at the mesh switch  [PLAUSIBLE]

**Mechanism.** Current launchers export *no* DDS profile at all (the FASTRTPS line is commented out; the FastDDS-3.5 wrapper fix was lost in a container recreate and is confirmed "CURRENTLY UNFIXED"). Bench captures show the agent and vio_bridge holding unicast sockets bound to the WiFi IP (192.168.0.50). Tab8's mesh switch *removes that IP* (radio moves to 192.168.77.x) mid-session; an AP roam or mesh flap does the same. DDS endpoints with stale locators can drop to the publisher=1/subscription=0 half-matched state → PX4 "Payload rx: 0" → EV dropout timed with the coordination phase. Counterweight from verification: run03 proved the *unpinned* env delivers VIO fine on a stable LAN, and the June symptom had a different root cause — so this risk concentrates specifically at **interface-change moments**, not steady state.

**Evidence.** AGENT_HANDOFF.md:296-309; all four launchers line 38 (commented) + 42; A_wifi_on/ss_tulpn.txt (sockets on 192.168.0.50); B_radio_off/ss_tulpn.txt (stale bound sockets after rfkill); AGENT_HANDOFF.md:35, 175-177 (mesh switch changes the single radio's IP).

**Sim repro (HERCULES).** Hard to reproduce faithfully without the container; emulate the effect: mid-exploration, firewall (iptables) the agent↔SITL UDP path or restart the agent so /fmu/in delivery silently stops while ROS-side publishers stay matched; verify the downstream signature equals R3's demotion. On the real bench (not sim): reproduce directly by flipping the WiFi IP under a running stack and watching `/fmu/in/vehicle_visual_odometry` pub/sub counts.

**Real-drone instrumentation.** `ss -tulpn | grep -E 'Agent|vio'` each 10 s; `ros2 topic info -v /fmu/in/vehicle_visual_odometry` (publisher/subscription counts) at coordination start and at failure; agent verbose logging (`-v6`) to catch session re-establishment; reinstall the FASTDDS_DEFAULT_PROFILES_FILE wrapper (documented fix) and pin agent + nodes to explicit interfaces.

---

### R9. Fleet launch path passes no bbox/flight_height — planner geofence vs valid-ESDF divergence → classic frontier wedge  [PLAUSIBLE — divergence CONFIRMED, defaults unknown]

**Mechanism.** Single-drone tab5 pins `flight_height:=1.0 bbox ±24 m` (deliberately 1 m inside nvblox's ±25 m workspace "so ESDF stays valid at the fence edge"; indoor value was ±10). fleet_bringup.launch.py gets *neither* argument — it flies on internal defaults nobody can read from this machine. If those defaults are stale (±10, or no fence, or fence ≥ workspace), a fleet-launched drone chases frontiers outside the valid ESDF or clamps at a fence with invalid ESDF: all remaining frontiers unreachable/filtered → planner idles while the guard holds position → stuck with no obvious error, or repeated planner errors. The ±24 bbox is a recent addition to the single-drone launchers (post-dates AGENT_HANDOFF), making non-propagation into the private launch file quite believable.

**Evidence.** fleet_ctl:143-146 (five args only); ghost launcher:131-135; archive launchers pass no bbox; AGENT_HANDOFF.md:124.

**Sim repro (HERCULES).** With the sim exploration stand-in (or real planner once repos are available): set exploration bbox larger than the mapped/valid region (or than the Blocks arena) and confirm the no-progress loop signature — frontiers permanently pending, guard holding, no setpoint progress. Compare against a correctly-inset bbox run.

**Real-drone instrumentation.** At fleet launch, `ros2 param dump` the planner/FIS nodes (bbox + flight_height actual values) and diff vs tab5's; log frontier-count + "selected frontier" topic (if any) so a stall shows whether the planner is idle-empty or retry-looping.

---

### R10. GPS/EV fusion fight outdoors: GPS trusted at 1 cm alongside full EV fusion, height vision-only, mag disabled  [Params CONFIRMED; occurrence field-only, inferred]

**Mechanism.** VIO15 keeps GPS enabled (`SYS_HAS_GPS=1, GPS_1_CONFIG=201, EKF2_GPS_CTRL=7`) and sets `EKF2_GPS_P_NOISE=0.01 m / V_NOISE=0.01 m/s` (vs sane 0.5/0.3) while also fusing EV pos+vel+yaw (`EKF2_EV_CTRL=15`), height solely from vision (`EKF2_HGT_REF=3`), mag off (`EKF2_MAG_TYPE=5`). On the 50×50 m *field*, a mid-flight GPS fix (even poor) gets fused at absurd confidence against the drifting VIO frame → position/yaw jumps or EKF resets → setpoint tracking error spikes → guard reacts / position-failsafe (`COM_POS_FS_EPH=5 m`) → hold. Indoors this is dormant (no fix) — a clean discriminator: does the stall only happen outdoors?

**Evidence.** VIO15:352, 370, 372, 380, 395, 483; COM_POS_FS_* (all files).

**Sim repro (HERCULES).** PX4 SITL fuses simulated GPS by default: load VIO15's EKF2 params, feed EV odometry from AirSim ground truth + drift, then enable/disable the SITL GPS mid-flight (`param set SYS_HAS_GPS`, or failure injection `failure gps off/ok`) and watch estimator resets and Offboard tracking. Sweep GPS noise params 0.01 vs 0.5.

**Real-drone instrumentation.** From the ulog: estimator_status reset counters, GPS fix/used flags, innovation test ratios around the stuck timestamp. Config fix candidate regardless: `EKF2_GPS_CTRL=0` for VIO flights or restore sane GPS noise.

---

### R11. Operator kill hazards: bare/typo'd `fleet_ctl stop` pkill-9s ALL FOUR drones mid-flight  [CONFIRMED behavior; per-incident applicability unknown]

**Mechanism.** `vehicles()` returns the whole fleet when given no valid drone name — `./fleet_ctl stop` bare or `stop gost` (typo filtered) sweeps every drone: pkill -9 of MicroXRCEAgent (kills the offboard keepalive at the FMU → R3 failsafe on a still-flying drone), vio_bridge, ros2 launch, plus host zenoh (severing coordination for drones meant to stay up). An operator "stopping the other drone" during a two-drone sortie silently kills the flying one.

**Evidence.** fleet_ctl:91-95, 226-246; AGENT_HANDOFF.md:221-223, 251.

**Sim repro (HERCULES).** During a two-drone sim run, execute the equivalent sweep against one drone's processes and record the failure signature (offboard timeout → hold/RTL, planner wedged) for comparison with field logs.

**Real-drone instrumentation.** Shell-history + timestamped fleet_ctl invocation log (one-line append in fleet_ctl main); guard rail: make `stop` with no args require `--all`. Cross-check any field stall timestamp against the invocation log.

---

## 2. Sim bring-up plan (this machine)

### 2.0 Machine state (verified today — better than assumed)

- UE 5.2.1 editor: `/home/lucas/UE5.2.1/Engine/Binaries/Linux/UnrealEditor` (exists; UE5.6 also present but AirSim stays on 5.2.1).
- Blocks is **already restored and editor-built**: `/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks/` has `Blocks.uproject`, `Content/`, `Plugins/`, and `Binaries/Linux/libUnrealEditor-Blocks.so` — the ".uproject and Content are gitignored" blocker from the fact sheet is already solved locally. AirLib `build_release/` exists too.
- A prior fleet-sim settings file exists: `/home/lucas/hercules-sim/settings-fleet-sim.json` — single SimpleFlight vehicle named `ghost`, `front_center` camera at **Pitch −20** (Scene FOV 69 + DepthVis FOV 87 — matches D435i RGB 69°/depth 87°), downward Distance sensor (Pitch −90, 0.05–12 m), IMU, chase cam, SubWindows. Plus `fly_and_capture.py` and a `venv/` alongside — a flight harness was already started.
- `~/Documents/AirSim/settings.json` currently holds the **Husky/Niricson demo** (SkidVehicle) — must be swapped (the many `.bak` files show the established convention: back up before overwrite).
- `/home/lucas/PX4-Autopilot` exists at `v1.17.0-alpha1-1600-gd6057df0a7` with `build/px4_sitl_default` already built. Caveat: fleet FMUs run **v1.15.2** (param dumps) — for faithful failsafe repro either `git worktree add /home/lucas/PX4-1.15.2 v1.15.2` and build that, or accept v1.17-alpha and document param drift.
- `MicroXRCEAgent` installed at `/usr/local/bin/MicroXRCEAgent`; ROS 2 Humble at `/opt/ros/humble`; clang-12 present.
- `zenoh-bridge-ros2dds` **not installed** — needed for R2/R5 repro (install the eclipse-zenoh release binary or `cargo install zenoh-bridge-ros2dds`, matching the fleet's version if discoverable).

### Phase A — SimpleFlight single drone (today; perception-side + harness)

1. Back up and swap settings: `cp ~/Documents/AirSim/settings.json ~/Documents/AirSim/settings.json.niricson.bak.$(date +%s)` then copy `/home/lucas/hercules-sim/settings-fleet-sim.json` into place.
2. Amend the settings file (keep everything else — it is already right):
   - In `front_center` CaptureSettings, add `{ "ImageType": 1, "Width": 640, "Height": 480, "FOV_Degrees": 87 }` (DepthPlanar — metric depth for an nvblox-like consumer; keep ImageType 3 only for the human SubWindow).
   - Optionally add a D435i-style stereo IR pair for cuVSLAM-shaped input: `front_left`/`front_right`, ImageType 0, 640×480, FOV 87, `Y: -0.025` / `Y: +0.025` (50 mm baseline), `X: 0.10`, `Pitch: -20` — same pattern as dataset_pipeline's stereo_left/right.
   - Default pawn: leave `PawnPath` unset (requirement satisfied).
   - `ClockType: ScalableClock` is fine for SimpleFlight; it MUST become `SteppableClock` in Phase C.
3. Launch Blocks: `/home/lucas/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks/Blocks.uproject"` and press Play (remember the PIE realtime toggle if the viewport looks frozen). Headless alternative for long runs: package Blocks once from the editor, or use `HERCULES/docker/download_blocks_env_binary.sh` (upstream Cosys 5.2 binary) — but the editor path is zero-setup here.
4. Smoke-fly on RPC 41451 with the existing `venv` + `fly_and_capture.py` (or PythonClient MultirotorClient): arm, takeoff to −1.0 m (matching `flight_height:=1.0`), fly a lawnmower inside ±24 m.
5. ROS 2 bridge: build/run the fork's `hercules_node` (`airsim_node_hercules.launch.py`, `host_port:=41451`) for live topics; use `hercules_synced_node` (`sync_mode:=lockstep`) when determinism matters (stamps t = step·dt — also the clean way to fault-inject reproducibly).

### Phase B — fault-injection harness (no private repos required; runs under SimpleFlight)

Small stand-in nodes (write once, reuse in Phase C), all in `/home/lucas/hercules-sim/src/` (new `sim_faults/` package):

- `vvo_relay.py` — subscribes sim odometry, republishes as `/fmu/in/vehicle_visual_odometry`-shaped stream with knobs: drop-window (100–500 ms), backwards-timestamp injection, rate halving, +20° mount rotation (mimics odom_correction/vio_bridge frame handling). Covers R4.
- `guard_standin.py` — publishes offboard_control_mode + trajectory_setpoint at 20 Hz toward a goal list; knobs: pause N ms, duplicate-instance mode (second copy with offset goals). Covers R3, R7.
- `explore_standin.py` — greedy frontier-ish waypoint chooser over the depth/Distance topics with a bbox parameter. Covers R9 (bbox > mapped region → wedge signature).
- Coordination rig: two ROS_DOMAIN_IDs on this host + two zenoh-bridge-ros2dds over loopback 7447 + a dummy octomap/loop-closure chatter topic; kill/restart/blackhole scripts. Covers R2. Run one side under `libfaketime` @1970 for R5.
- `socat -d -d pty,raw,echo=0 pty,raw,echo=0` pair for the serial-collision rig (R1) — used for real in Phase C.

### Phase C — PX4 SITL in the loop (the failure classes live here)

1. PX4 version: prefer `git -C /home/lucas/PX4-Autopilot worktree add /home/lucas/PX4-1.15.2 v1.15.2 && cd /home/lucas/PX4-1.15.2 && make px4_sitl_default none_iris` (fleet-matching firmware); fall back to the existing v1.17 build if the worktree build fights.
2. Switch the vehicle in settings.json to fleet-shape PX4: `"VehicleType": "PX4Multirotor", "UseSerial": false, "UseTcp": true, "TcpPort": 4560, "LockStep": true, "ControlPortLocal": 14540, "ControlPortRemote": 14580, "LocalHostIp": "127.0.0.1"`, add `"Barometer": { "SensorType": 1, "Enabled": true, "PressureFactorSigma": 0.0001825 }`, set top-level `"ClockType": "SteppableClock"` (gotcha: with a PX4 vehicle present the default silently becomes ScalableClock, which breaks LockStep). Keep the Phase-A cameras/IMU/Distance (Distance has ExternalController default true → forwarded to PX4 as DISTANCE_SENSOR, matching the real downward rangefinder into EKF2_RNG_CTRL=1).
3. Params: apply the VIO15 dump's behavior-relevant subset to SITL (EKF2_EV_* incl. EV_CTRL=15/EV_DELAY=33/HGT_REF=3/QMIN=0, EKF2_GPS_* incl. the 0.01 noises, COM_OF_LOSS_T/COM_OBL_RC_ACT/COM_RCL_EXCEPT/NAV_RCL_ACT/COM_RC_LOSS_T, RTL_*=1.5, COM_POS_FS_*, MPC_*) — via the vehicle's `Parameters{}` block in settings.json or `param set` lines in a SITL rcS snippet. Skip hardware cals/PWM.
4. uXRCE loop (the real fleet transport): SITL runs `uxrce_dds_client` (UDP, default port 8888) → `/usr/local/bin/MicroXRCEAgent udp4 -p 8888` → `/fmu/*` topics on the agent's domain. This natively reproduces R6 (set `param set UXRCE_DDS_DOM_ID 0` vs ROS_DOMAIN_ID=3) and R3 (starve the stream). For R1, run the client over serial instead: point the agent at one end of the socat pty pair and open the other end's sibling at 57600 mid-run.
5. Fly Offboard with `guard_standin.py` + `vvo_relay.py` (EV from AirSim ground truth + noise) and verify the healthy precondition equals run03's ground truth: nav_state 2 → Offboard, xy/z/v valid true, VVO ~30 Hz — then run the R1–R11 fault schedule one cause at a time, recording `/fmu/out/vehicle_status` + local_position flags as the classifier signature.
6. Scale to the fleet: 4 vehicles in `Vehicles{}` (Y = 2i offsets), `PX4Scripts/run_airsim_sitl.sh 0..3` (TcpPort 4560+i, control ports +i), per-drone ROS_DOMAIN_ID 1–4, per-drone agent, per-drone zenoh bridge over loopback — a 1:1 topology mirror of the real fleet on one box. 2 drones is enough for every coordination-layer cause (R2, R5, R6, R11).

**Milestone order:** A-hover → A-topics → B-harness green → C-single-PX4 Offboard healthy baseline → C fault matrix (one cause per run, signatures archived) → 2-drone coordination faults. Once the private repos are cloned onto this machine, swap stand-ins for the real nodes (they run on ROS 2 Humble, which the host has) and re-run the same fault matrix.

---

## 3. Open questions answerable ONLY with the 7 private repos

**multi_drone_nvblox** (fleet_bringup.launch.py, loop_closure_sp_node.py, octomap_exchange_node, launch_bench_zenoh.sh / launch_mesh_zenoh.sh):
1. Does `fleet_bringup.launch.py` start `lora_bridge`, with what `serial_port`/baud args, and at bringup or on a later trigger? (Decides R1's timing plausibility outright.)
2. What are fleet_bringup's internal defaults for bbox/flight_height (or does it omit the planner)? (Confirms/kills R9.)
3. Do loop_closure_sp_node/octomap_exchange block synchronously on peer responses, hold stale alignment state, or degrade gracefully when a peer/bridge vanishes? Retry/timeout values? (Ranks R2/R6.)
4. How do they handle cross-drone header stamps and TF (`publish_tf:=True`) — wall clock vs boot time, any skew tolerance, PCM time-window assumptions? (Confirms/kills R5.)
5. Does `launch_mesh_zenoh.sh` set ROS_DOMAIN_ID (e.g., derived from NODE_ID) for the bridge, and what are the bridge allowlist/config differences vs `launch_bench_zenoh.sh`? (R2 item 4.)

**active_exploration** (reactive_depth_guard, simple_exploration_planner, fis):
6. reactive_depth_guard: what input staleness (ESDF, depth, odom) makes it stop or hold its offboard_control_mode/trajectory_setpoint stream? Is there a watchdog that keeps the keepalive alive when the planner goes silent? (R3's trigger.)
7. Does anything detect nav_state falling out of Offboard and re-command Offboard mode? What does `debug_skip_arm_check` actually skip? (Whether R3 is permanent by design.)
8. Planner idle conditions: frontier blacklist/retry logic, behavior when all frontiers are filtered/unreachable, and whether it waits on merged multi-drone map/frontier claims from peers. (R9, R2.)
9. QoS profiles on `/planning/trajectory_setpoint` and guard inputs — explicit QoSProfile or default-10? (Known silent-drop failure class on this team; a mismatch is invisible without the source.)
10. The exact error strings the operator sees when it "gets stuck" — grep targets to classify field incidents against the R1–R11 signatures.

**px4_offboard** (vio_bridge.launch.py):
11. The exact MicroXRCEAgent exec line: device path (raw /dev/ttyUSB0 vs by-id), env it inherits, whether any FASTDDS/FASTRTPS profile is set in-launch, and restart-on-death behavior. (R1 enumeration variant, R8.)
12. vio_bridge timestamp source (boot vs wall clock) and whether it forwards cuVSLAM covariance (EKF2_EV_NOISE_MD=0 says PX4 ignores it — confirm intent), plus the division of the +20° compensation between vio_bridge and odom_correction (double-compensation risk between the PX4 path and the nvblox path).

**radiohive:**
13. Confirm default port/baud in code, whether the serial opens eagerly at node start or lazily on first TX (sets the collision timing), any port-in-use check, and the zenoh↔LoRa failover logic (does LoRa activation change topic routing that coordination nodes depend on?).

**isaac_ros_* forks (visual_slam, nvblox):**
14. cuVSLAM timestamp handling on non-monotonic input (drop vs reset vs error state) and whether an EKF2/nvblox pose jump corrupts ESDF (map-poisoning after an estimator reset — feeds R4/R10 into R9's wedge).
15. nvblox workspace bounds (confirm ±25 m in nvblox_base.yaml) and ESDF-staleness behavior at the fence edge; `use_sim_time` support across all of the above (required to run the real nodes against lockstep sim).

Key artifact paths referenced: `/home/lucas/hercules-sim/settings-fleet-sim.json`, `/home/lucas/hercules-sim/fly_and_capture.py`, `/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks/Blocks.uproject`, `/home/lucas/UE5.2.1/Engine/Binaries/Linux/UnrealEditor`, `/home/lucas/PX4-Autopilot` (v1.17-alpha built; fleet dumps are v1.15.2), `/usr/local/bin/MicroXRCEAgent`, `/home/lucas/hercules-sim/src/multirobot_launch_scripts/px4_params/prometheus_uav_VIO_fused_PX4_params_july-15-2025.params`.