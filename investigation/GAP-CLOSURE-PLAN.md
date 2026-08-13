# HERCULES SIM2REAL GAP CLOSURE PLAN
**Host:** Ryzen 7 9800X3D (8C/16T) · 30 GB RAM · RTX 5090 32 GB · Ubuntu 22.04 · ROS2 Humble · rig at `/home/lucas/hercules-sim/`
**Baseline (measured, not estimated):** `fidelity_scorecard.sh` is BUILT and validated on real logs — **coord4 run = 26 FAIL, hw4 PX4 run = 5 FAIL**. Exit code = FAIL count. Everything below moves those two numbers.
**Disk law (verified):** `/` is at **1.6 GB free (100%)** — every build artifact, DDC, and asset lands on `/home/lucas/UE5` (1.6 TB free) or the build dies mid-compile.
**Verification status:** all 8 top-priority specs were empirically verified on this machine; all came back NEEDS_FIX. The queue below is the **post-fix** version — the verifier's corrections (including one polarity inversion that would have made things worse) are baked in and marked ✓fix.

---

## 1) GAP SCORECARD — NOW

Field references are embedded in `fidelity_scorecard.sh` as `[F1]–[F15]` with file:line provenance (ghost_debug/run01 captures, AGENT_HANDOFF.md:207-233, RADIO-V2-DESIGN.md:54-83, FINAL-SYNTHESIS.md:18, nvblox_base.yaml:14-18, tegrastats.txt).

| # | Metric | FIELD value | SIM value | Gap |
|---|--------|-------------|-----------|-----|
| 1 | Stereo pair rate /drone | 30 Hz effective (D435i 640×480×60 emitter-split; `realsense_emitter_flashing.yaml:28-55`, `c_hz_vslam_odom.txt`) | **2.40–2.60 Hz** (PNG+PIL Python RPC loop, `cuvslam_sim_bridge.py:269-281`; RUN-A-REPORT.md:41) | **0.08×** FAIL — dominant gap |
| 2 | IMU rate + stamping | 200.05 Hz, sensor-stamped (`c_hz_camera_imu.txt`) | 26.8–31.5 Hz, **wall-clock at publish** → RPC jitter injected into VIO alignment (`cuvslam_sim_bridge.py:199-213`) | **0.15×** FAIL |
| 3 | Depth rate + encoding | 30 Hz, 16UC1 mm via splitter | ~8 Hz, 32FC1 m (`cuvslam_sim_bridge.py:216-234,293-303`) | **0.27×** + wrong encoding FAIL |
| 4 | cuVSLAM inter-frame stutter (>34 ms) | ~0/min steady state (STUCK-BUG-INVESTIGATION.md:57) | **138–153/min**, max 582 ms | FAIL |
| 5 | cuVSLAM odometry rate | 30.05 Hz | ~2.6 Hz | **0.09×** FAIL |
| 6 | VIO believed-vs-physical divergence (the rig's one valid health metric, RUN-A-REPORT.md:42) | pass < 1 m | ghost **9.9 m** /190 s (meta_*.json) | FAIL |
| 7 | EV → EKF2 acceptance | 29.94 Hz, EKF2 **accepts** (AGENT_HANDOFF.md:216-217) | median dt **384 ms**, 94.7% > 200 ms hard limit (`EV_MAX_INTERVAL`, PX4 `EKF/common.h:70`); cs_ev_pos duty **0.215**; z excursion **−408.6…+36.3 m** at 1.2 m true alt (00_23_32.ulg) | FAIL — structural below 5 Hz |
| 8 | EV trust defense | quality-gated tracker | vio_bridge hardcodes var=0.01, quality=100 (`vio_bridge_node.cpp:121-125`) — garbage fused at 10 cm σ | FAIL |
| 9 | Flight entry + failsafes | RC pilot: stick-gesture arm → Position takeoff → Offboard switch; RC-loss→RTL@1.5 m, Offboard→POSCTL@1 s reachable (FINAL-SYNTHESIS.md:68) | blind `sleep 35; commander takeoff` FIFO (`run_px4_sim.sh:109-113`) — takeoff gated on valid local pos, so the field failure regime + both failsafes are **unreachable** | divergent |
| 10 | Pipeline shape / QoS | `/camera0/*` realsense+splitter topics, SENSOR_DATA QoS (`realsense.launch.py:52-74`) | `/sim/*` bypass, default-reliable | divergent |
| 11 | Compute /drone | 8× A78AE ≈ 2.3 host-core-equiv; 7.5–10.0/15.6 GB; Orin iGPU 8 SM ≈ 1.88 TFLOPS (tegrastats) | 4 stacks free-run on 16 threads + 170-SM 5090 | ~**7× CPU-, ~50× GPU-optimistic** |
| 12 | Clock topology | 3 drones boot 1969-epoch, ghost NTP; F9 = NTP step silently freezes cuVSLAM (FINAL-SYNTHESIS.md:106-108) | all wall clock; F9 unreproducible | divergent |
| 13 | Determinism | — (regression need) | free-run ScalableClock; VIO collapse is host-load-dependent | divergent (gap 9) |
| 14 | Environment | outdoor textured field (desert/forest) | Blocks gray grid: feature-sparse, photometrically perfect | divergent |
| 15 | Coordination cycle | 190 ms (RADIO-V2-DESIGN.md:54) | 190 ms | **PASS** |
| 16 | Sim-rig health | field OK | zenoh panic ×4, LoRa attach 0/4, peer exchange dead, planner NO PATH ×120, splanner CSV collision | FAIL — detected by scorecard; owned by radio-v2 + rig-bug workstreams, not this queue |

---

## 2) PRIORITIZED IMPLEMENTATION QUEUE

Ordering = dependency first, then leverage. Every ✓fix is a verifier correction applied to the original spec.

### DO NOW (hours each)

| Q | Item | Gaps | Verifier fixes applied | Validation gate |
|---|------|------|------------------------|-----------------|
| **Q1** | Build/adopt airsim_ros_pkgs C++ wrapper on the big disk | foundation for 1/6/9 | ✓fix no src symlink (breaks `AIRSIM_ROOT` lexically — **reproduced**); use `--base-paths`; CMakeLists:4 Release edit is **mandatory**; recipe already proven end-to-end in `/home/lucas/UE5/.verify_wrapper_ws` (all 4 binaries installed) | `ros2 run … hercules_synced_node` logs "connected to AirSim"; the follow-on AcquisitionTimeout abort is expected (car node vs drone sensors) and does **not** fail this step |
| **Q2** | vio_bridge hardening: real covariance + tracking-health quality gate + sanity clamp | 2 (also flies on the Jetsons) | ✓fix **vo_state==1 is Success** (spec had ==2 = Failed — inverted gate would fuse only garbage); ✓fix add `isaac_ros_visual_slam_interfaces` to package.xml/CMakeLists (build fails without); ✓fix pyulog not installed — install to venv | z stays in [−2.5, 0.5] while VIO is garbage (vs −408.6/+36.3 baseline); quality drops to 0 on tracking loss |
| **Q3** | Virtual RC pilot: MANUAL_CONTROL Position takeoff → Offboard | 8 (+unblocks 2 debugging) | (env area; pymavlink 2.4.49 verified in `venv2`; `COM_RC_IN_MODE=1` already set in settings-fleet-px4.json:28) | stick-gesture arm in log; nav_state 2→14; `--drop-rc-at 60` reproduces RC-loss→RTL@1.5 m field signature |
| **Q4** | Staged EV bring-up ladder in run_px4_sim.sh | 2 | ✓fix inject params **after line 50** ("airsim + px4 linked"), NOT line 38 — AirSim pushes `settings-fleet-px4.json:31-33` Parameters at connect (`MavLinkMultirotorApi.hpp:531,1552`) and would overwrite; ✓fix delete `EKF2_EV_CTRL`/`EKF2_HGT_REF` from the JSON; ✓fix stage-2 check via px4 log string "starting ev_hgt fusion, resetting state" (`ev_height_control.cpp:195-197`) — no ROS topic carries `height_sensor_ref`; ✓fix `timeout`-wrap the `ros2 topic hz` offboard gate; SITL persists params in `eeprom/parameters.bson` → each stage sets the **full** triple both directions | per-stage `listener estimator_status_flags` via pxh FIFO: stage1 cs_ev_pos/vel/yaw true; stage2 +cs_ev_hgt; stage3 cs_gps false, hover z=−1.2±0.3, no "vision data stopped" |
| **Q5** | Bridge monotonic patch | 7 prereq | (compute-clock finding) `time.time()`→`time.monotonic()` at `cuvslam_sim_bridge.py:237,240,347-348,373,375`; keep line 319 `meta['t']` as wall | a clock step no longer terminates the run loop |
| **Q6** | Orin CPU+RAM budgets via `sudo -n systemd-run` scopes + new `fleet_drone_stack.sh` | 5 | (verified live: user-scope delegation lacks `cpu` on 22.04 — must be `sudo -n systemd-run --scope --uid=lucas`); `CPUQuota=270%` `CPUQuotaPeriodSec=10ms` `AllowedCPUs=2-7,10-15` `MemoryHigh=9G MemoryMax=10G MemorySwapMax=0`; parent `hercules-drones.slice MemoryMax=24G`; zenoh bridges **move into** the drone scopes (they currently escape at `run_fleet_coord.sh:54-58`; field zenoh runs on the Orin); UE5 stays unconfined | `systemd-cgtop -m hercules-drones.slice` shows 4 scopes ≤2.7 CPU/≤10 G; `cpu.stat nr_throttled>0`; A/B vs `HERC_CPUQUOTA=1600%` shifts cuVSLAM/nvblox rates toward field; UE5 frame time ±10% |
| **Q7** | Scorecard wired into every run script + pre-flight aborts | meas. | (metrics spec) append scorecard call + `capture_host_load.sh` sidecar to run_e1/fleet_coord/fleet_scale/px4_sim/cuvslam_sim; 30 s-in abort on zenoh "panicked" / missing "opened LoRa serial" | every run auto-emits `scorecard_<label>.txt` + `host_<label>.pidstat`; deliberately broken zenoh aborts <60 s |
| **Q8** | QoS/encoding parity — now-portion (pre-C++-node) | 6 | ✓fix audit with `ros2 topic info -v` (NOT `ros2 node info` — shows no QoS); ✓fix add missing `--depth-encoding {32FC1,16UC1}` switch to both Python bridges (`np.clip(d*1000,0,65535).astype(uint16)`, hardcoded today at `nvblox_sim_bridge.py:179`, `cuvslam_sim_bridge.py:224`); sim nvblox `-p input_qos:=SENSOR_DATA` (base yaml:55 is SYSTEM_DEFAULT; field specialization is SENSOR_DATA); inventory `/sim/*` stragglers (hits: run_cuvslam_sim.sh:49-53,70-71; run_nvblox_sim.sh:38-39; run_fleet_coord.sh:93-106; run_px4_sim.sh:70-73; both bridges) | zero "incompatible QoS" in cuvslam/nvblox logs; 16UC1 vs 32FC1 150 s A/B: FIS cluster counts within noise → ship 16UC1; `/sim/` grep of active scripts → empty after Q9 |

### NEXT DAY (day each; Q9 is the payload)

| Q | Item | Gaps | Verifier fixes applied | Validation gate |
|---|------|------|------------------------|-----------------|
| **Q9** | **`airsim_realsense_node.cpp`** — per-drone C++ node impersonating realsense-ros + splitter; kills the Python sensor loops | **1+6** (the dominant gap) | ✓fix append to `install(TARGETS…)` at CMakeLists:169 (mirroring :128-142 alone never installs); ✓fix **RPC worker-pool patch**: `SimModeBase.cpp:1912` `start(false, spawned_actors_.Num()+4)` = 8 workers — 4×stereo+4×depth blocking renders saturate it and starve the 200 Hz IMU connections → change to `spawned_actors_.Num()*4+8`, rebuild Blocks plugin; ✓fix stamp **both** stereo images+infos from `responses[0].time_stamp` (batch stamps not guaranteed identical); ✓fix pair/imu/depth counters move into the node (gutted bridge reads 0); ✓fix "zero 34 ms lines" unattainable at 30.0 Hz (nominal 33.3 ms sits 0.7 ms under threshold) → gate is <5/150 s, never consecutive; settings-fleet-4drone.json has **no** SubWindows key (no-op) — the render competitor is chase capture → `NB_CAPTURE=0` on rate-critical runs | single drone: infra_1/2 ≥30 Hz identical stamps, `/camera0/imu` 195–205 Hz, depth ≥30 Hz; odometry ~30 Hz (vs 2.6); 4-drone: ≥3800 pairs/drone/190 s (baseline 503–589), IMU holds 200 Hz under full image load (proves pool patch) |
| **Q10** | EV cadence ≥10 Hz acceptance gate on Q9 | 2 | ✓fix reset counting via `estimator_event_flags.reset_hgt_to_ev` — the ECL_INFO string is `PX4_DEBUG` under MODULE_NAME (`estimator_interface.h:47-48`), **never appears in ulog**; ✓fix system python3 (venvs lack pyulog) | pyulog: frac(dt>200 ms)==0.0, p99<150 ms, cs_ev_pos duty>0.95, ev_hgt resets==1 |
| **Q11** | Frame-convention harness (gravity-alignment ambiguity) | 2 residual [inf] | ✓fix run under **SimpleFlight** (`moveByVelocityAsync` doesn't drive PX4 vehicles; Offboard path circularly blocked by gap 2) in the `ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1` env; ✓fix innovations via pxh `listener` or ulog — `estimator_innovation_test_ratios` is NOT in `dds_topics.yaml`; ✓fix explicit `--qos-reliability best_effort --qos-durability transient_local` on echo; if verdict flips, re-audit odom_correction's own +20° correction (same exposure) [inf] | parked pitch(q) ≈ −20° vs identity decides convention; 2 m forward → (+2,0,0)±0.1, |dz|<0.05 (0.68 m = conviction); 90° yaw → −90°±3; then GPS-denied hover z=−1.2±0.3, EV ratios <0.5 |
| **Q12** | Full QoS audit post-Q9 (`ros2 topic info -v` all six `/camera0/*`; encoding byte-parity vs a field bag via `ros2 bag play`+`echo --field encoding` — `bag info` doesn't show encoding) | 6 | ✓fix as noted | every cuVSLAM/nvblox subscription matched, compatible QoS, zero rmw warnings |
| **Q13** | Lockstep multirotor port (`airsim_node_drone_synced.cpp` from the car node) | 9+1 | ✓fix register CMake target + install (spec omitted → nothing builds); ✓fix host_port default 41451 (car default is 41452); ✓fix cadence is **35 ms** (every 7th 5 ms tick = 28.57 Hz; "33.3 ms" in the original was wrong); SteppableClock settings variant; single-DDS-domain namespaces = accepted deviation for regression runs; RTF fallback `sync_dt=0.01` if measured RTF <0.1 | stamps differ by exactly 35 000 000 ns; 200 Hz IMU in sim time at any host load; two identical-setpoint 60 s runs agree <5 cm |
| **Q14** | Per-drone GPU budget via CUDA MPS (`CUDA_MPS_ACTIVE_THREAD_PERCENTAGE=2` ≈ 3.4 SM ≈ Orin 1.88 TFLOPS; `PINNED_DEVICE_MEM_LIMIT=0=6G`) | 5 | MPS-on-Blackwell-GeForce untested [inf] — 5-min liveness probe is step 1; fallback = document GPU as unthrottled residual (never `-lgc`, it cripples UE5's Vulkan render) | nvidia-smi shows M+C server + clients; nvblox per-frame ms rises at 2%; `0=2G` deliberately reproduces the flights-70/71 CUDA-OOM class |
| **Q15** | Field clock chaos via libfaketime (per-drone 1970 epochs + F9 NTP-step on ghost) | 7 | build **preserved this session** → `/home/lucas/hercules-sim/bin/libfaketime.so.1` (v0.9.10 w/ FAKE_SETTIME; apt 0.9.8 unusable); requires Q5 + Q6's `fleet_drone_stack.sh`; `FAKETIME_DONT_FAKE_MONOTONIC=1` (Jetson monotonic is boot-relative); UE5/PX4 excluded; `env -u LD_PRELOAD` fallback for cuVSLAM/nvblox if CUDA+interposer misbehaves [inf] | domains 2-4 stamp sec≈100–600, ghost ≈1.79e9; `echo '+0' > /tmp/ft_ghost` at T+90 s reproduces F9: odometry→0 Hz with **no** error line + mass peer expiry ≤13 s; `HERC_FAKETIME=0` runs byte-identical |
| **Q16** | Field50: Blocks → textured 50×50 m outdoor field (Poly Haven CC0, headless editor-python) | 4 | disk offload symlinks (DDC + Content/Field → `/home/lucas/UE5/hercules-sim-big/blocks_offload/`) are step 1 — mandatory on a 1.6 GB-free root; anti-tiling macro-blend (repeated descriptors = false loop-closure ambiguity); NoiseSettings camera knob (`docs/settings.md:338`); `MAP=` env var into all run scripts; GameMode automatic (project-wide `DefaultEngine.ini:11`) | A/B vs Blocks: ≥3× cuVSLAM features, zero tracking loss on yaw, UE fps ≥ baseline, root-disk free unchanged |
| **Q17** | Instrumentation holes: depth counter + 10 s rate lines in bridges; sim-only nvblox `print_rates_to_console:true`; all-drone `meta_*.json` capture (RUN-A gate 4); splanner CSV drone-ID fix (RUN-A bug 4) | meas. | as specced | scorecard depth/nvblox rows show numbers not SKIP; divergence per drone; 4 distinct CSVs |
| **Q18** | EKF2 innovation deep metric (`logger on` → pyulog rows: ev ratio p95, ev_yaw latched, hgt_runaway, reset counts; same extraction works on field .ulg) | meas.+2 | as specced | baseline FAILs hgt_runaway/ev_yaw (matches hw4); post-fix rows prove fusing, not merely arming |

### NEEDS USER
| Q | Item | Ask |
|---|------|-----|
| **Q19** | Fab claim + Rural Australia (+Chestnuts) — the actual HERCULES outdoor packs | Lucas: ~20 min at fab.com "Add to My Library" (listings in `docs/downloading_hercules_environments.md:19-21`); download via Lutris Epic launcher (known-good) or Jonathan's Windows box (`ssh jonathan@100.111.87.69`); content lands on big disk, symlinked as `Content/RuralAustralia` (exact expected name, docs:73-74). **Skip City Sample** (reference machine had 96 GB RAM vs our 30) |
| **Q20** | Orin calibration of throttle constants (CPU 0.33× [inf], GPU 2% [inf], 6G VRAM [inf] → measured) | Sai/bench: `stress-ng` 1c+8c, tegrastats 60 s under full stack, nvblox-on-bag timing; tightens compute parity ±45%→~±10% |
| **Q21** | Field reference refresh: run01-style capture bundle on all 4 drones **with coordination stage** (run01 was VIO-only → current envelope is a lower bound) + one EV-fused .ulg per drone | keeps `[F1]–[F15]` honest; feeds Q18's field baseline |

### WON'T FIX (with reason)
- **Memory-bandwidth parity** — no per-cgroup MBA on consumer AMD; host's shared ~85 GB/s vs 102.4 GB/s/Orin makes sim mildly *pessimistic* per drone. Documented residual.
- **GPU per-kernel latency + VRAM-bandwidth parity** — MPS bounds throughput only; MIG absent on GeForce; green contexts need closed-source cuVSLAM changes. Q20's p95-vs-median check records the floor.
- **Radio channel fidelity (gap 3)** — explicitly out of scope; radio v2 design owns it.
- **UE5.6 asset-pack ports / Cosys engine port to 5.6** — packs on disk are EngineAssociation 5.6, unloadable in 5.2.1; engine port is multi-week [inf]; dominated by Q16+Q19.
- **City Sample** — RAM-bound (30 vs 96 GB reference). Revisit on RAM upgrade (known 5090-box bottleneck).
- **Per-drone DDS domains under the lockstep node** — one process can't span domains; namespaced single-domain is an accepted deviation for regression-only runs (async Q9 node keeps field topology).
- **`EV_MAX_INTERVAL` 200→500 ms patch** — contingency for plumbing tests only; field PX4 keeps 200 ms, so it never ships as a fix and any run using it is marked parity-divergent.

---

## 3) TOP 3 DO-NOW — FULL IMPLEMENTATION DETAIL

### Q1 — Build/adopt the C++ wrapper workspace (foundation)

Status found on disk right now: the verifier's scratch build **completed** — `/home/lucas/UE5/.verify_wrapper_ws/install/airsim_ros_pkgs/lib/airsim_ros_pkgs/` contains `airsim_node`, `hercules_node`, `hercules_synced_node`, `pd_position_controller_simple_node`. However `HERCULES/ros2/src/airsim_ros_pkgs/CMakeLists.txt:4` still reads `set(CMAKE_BUILD_TYPE Debug)`; the plain `set()` shadows the `-DCMAKE_BUILD_TYPE=Release` cache value during flag selection, so that scratch build's AirLib is Debug-flagged (3–5× slower image copy path). Production = one-line edit + rebuild with the proven recipe.

```bash
# 0) deps — all 9 verified already 'ii' on this host (no-op confirmation)
sudo apt install -y ros-humble-geographic-msgs ros-humble-tf2-sensor-msgs ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-pcl-conversions libpcl-dev libyaml-cpp-dev libopencv-dev

# 1) MANDATORY code delta (not belt-and-suspenders: line 4 silently overrides --cmake-args)
#    /home/lucas/hercules-sim/HERCULES/ros2/src/airsim_ros_pkgs/CMakeLists.txt:4
-set(CMAKE_BUILD_TYPE Debug)
+set(CMAKE_BUILD_TYPE Release)

# 2) workspace on the big disk — NO src symlink.
#    (Symlinked src PROVEN BROKEN: AIRSIM_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../../ at CMakeLists.txt:31
#     collapses lexically to /home/lucas/UE5/ and add_subdirectory at :33-35 dies with
#     "does not contain a CMakeLists.txt file" — reproduced on this machine.)
mkdir -p /home/lucas/UE5/hercules_wrapper_ws && cd /home/lucas/UE5/hercules_wrapper_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths /home/lucas/hercules-sim/HERCULES/ros2/src \
  --packages-up-to airsim_ros_pkgs --cmake-args -DCMAKE_BUILD_TYPE=Release
# builds airsim_interfaces (~6 s) then airsim_ros_pkgs (rpclib+AirLib+MavLinkCom in-tree, gcc-11, ~5-10 min)
# then: rm -rf /home/lucas/UE5/.verify_wrapper_ws   # scratch build superseded
```

Pitfalls (re-verified): never `CC/CXX=clang` — ROS Humble links libstdc++; the clang-12/libc++ archives in `HERCULES/build_release` are ABI-incompatible and correctly NOT reused (`CommonSetup.cmake:59-64` has a native gcc branch). rpclib headers present at `HERCULES/external/rpclib/rpclib-2.3.0` (CMakeLists.txt:43). No `-Werror` anywhere in the chain. `/tmp` is on the full root disk — never build there.

**Validation:** `source /home/lucas/UE5/hercules_wrapper_ws/install/setup.bash && ros2 pkg prefix airsim_ros_pkgs`; then against the running Blocks (port 41451, vehicle `ghost` in settings-fleet-4drone.json): `ros2 run airsim_ros_pkgs hercules_synced_node --ros-args -p sync_mode:=atomic -p "vehicles:=[ghost]" -p host_port:=41451` → PASS = line `hercules_synced_node connected to AirSim at localhost:41451.` (`airsim_node_hercules_synced.cpp:385`). An AcquisitionTimeout abort ~10 s later is EXPECTED (it is the Husky/car lockstep node — `CarRpcLibClient` at :382, camera `front_center`/`LidarSensor1` defaults) and does not fail this step; the multirotor adaptation is Q13.

### Q2 — vio_bridge hardening (real covariance, health-gated quality, sanity clamp)

File: `/home/lucas/hercules-sim/src/px4_offboard/src/vio_bridge_node.cpp` (symlinked into `ros2_ws/src` — verified).

Code deltas:
1. **Lines 121-123** (hardcoded variances) → pass-through: `position_variance[i] = max(msg->pose.covariance[0|7|14], 0.01)`; `orientation_variance` from `pose.covariance[21|28|35]`; `velocity_variance` from `twist.covariance[0|7|14]` (diagonal max() acceptable first cut; rotate through `R_cb_` if off-diagonals matter).
2. **Line 125** (`quality=100`) → subscribe `/visual_slam/status` (`isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus`). **CORRECTED SEMANTICS (verifier-caught inversion):** on this machine's installed 3.2.x msg, `vo_state` is 0=Unknown, **1=Success**, 2=Failed → `quality = (vo_state==1) ? 100 : 0` (the original `==2` would fuse *only failed* tracking). Plus staleness: last status older than 500 ms → quality=0.
3. **New build deps or it won't compile:** `package.xml` add `<depend>isaac_ros_visual_slam_interfaces</depend>`; `CMakeLists.txt` add `find_package(isaac_ros_visual_slam_interfaces REQUIRED)` + append to the vio_bridge `ament_target_dependencies` (currently only rclcpp px4_msgs nav_msgs). Debian 3.2.5 with C++ typesupport already installed.
4. **Physical-envelope clamp** before publish: `|v| > 6 m/s` (2× MPC_XY_VEL_MAX) or `|Δp| > 1.5 m` between samples → quality=0 for this **and the next 5** samples (EKF2's start gate checks previous AND newest buffered quality, `PX4-1.15.2/src/modules/ekf2/EKF/ev_control.cpp:55-58` — cleanly blocks restart on a diverged tracker).

```bash
cd /home/lucas/hercules-sim/ros2_ws && colcon build --packages-select px4_offboard --symlink-install
# enable the gate in PX4: run_px4_sim.sh, insert after line 38 ("px4 sitl up"):
echo "param set EKF2_EV_QMIN 10" >&4     # FD4 = pxh FIFO opened at :36; buffers until pxh reads
/home/lucas/hercules-sim/venv/bin/pip install pyulog   # absent from system+venv — validation needs it
```

**Validation:** `run_px4_sim.sh qmin_test 120`. Live: `ros2 topic echo /fmu/in/vehicle_visual_odometry --field quality` drops to 0 whenever cuVSLAM warns/loses tracking; `/fmu/out/vehicle_local_position --field z` stays in **[−2.5, 0.5]** even with garbage VIO (fusion never starts; baro holds). Post-run: `/home/lucas/hercules-sim/venv/bin/python -c "from pyulog import ULog; u=ULog('<newest .ulg>',['vehicle_local_position']); z=u.data_list[0].data['z']; print(z.min(), z.max())"` → no excursion beyond a few meters (vs −408.6/+36.3 in 00_23_32.ulg, at `PX4-1.15.2/build/px4_sitl_default/rootfs/log/<date>/`).

### Q3 — Virtual RC pilot (field flight-entry ritual + reachable failsafes)

New file `/home/lucas/hercules-sim/tools/virtual_pilot.py`, run with `/home/lucas/hercules-sim/venv2/bin/python` (pymavlink 2.4.49 verified there). No PX4 param changes: `COM_RC_IN_MODE=1` already in settings-fleet-px4.json:28; MANUAL_CONTROL scaling per `mavlink_receiver.cpp:2083-2087` (x/y/r −1000..1000; z throttle 0..1000, 500=center); SITL GCS link 18570→14550 with LOCAL_POSITION_NED @50 Hz (`px4-rc.mavlink:11,14,16`) — no port conflict with AirSim's 4560/14540/14580.

Core sequence:
```python
m = mavutil.mavlink_connection('udpin:0.0.0.0:14550'); m.wait_heartbeat()
# 50 Hz daemon thread: m.mav.manual_control_send(m.target_system, x, y, z, r, 0) from shared stick dict
# (a) wait LOCAL_POSITION_NED finite/quiet          -> EKF2 origin ready
# (b) DO_SET_MODE base=CUSTOM_MODE_ENABLED, main_mode 3 (POSCTL)
# (c) arm by stick gesture: z=0, r=1000 held 2.5 s  (MAN_ARM_GESTURE default on, ManualControl.cpp:301)
#     verify SAFETY_ARMED in HEARTBEAT; fallback COMPONENT_ARM_DISARM after 6 s
# (d) takeoff on sticks: z=800 until lpos.z <= -alt (--alt 1.0 NED), then z=500 hold 5 s
#     <- this stick-held Position hover is the exact window where the field height-runaway must reproduce
# (e) Offboard switch only after the guard's setpoint stream is live; DO_SET_MODE main_mode 6
# (f) KEEP streaming centered sticks all flight     <- what makes Offboard->POSCTL demotion possible
# (g) land: POSCTL, z=250 until landed, disarm gesture z=0, r=-1000 3 s
# flags: --drop-rc-at SEC (kill MANUAL_CONTROL mid-Offboard), --no-offboard (pure EV/EKF2 debugging)
```

Delta in `/home/lucas/hercules-sim/run_px4_sim.sh`: replace the four flight-entry lines (`sleep 35; echo "commander takeoff" >&4; sleep 12; echo "commander mode offboard" >&4`, :109-113) with:
```bash
/home/lucas/hercules-sim/venv2/bin/python -u $BASE/tools/virtual_pilot.py --alt 1.0 > $LOG/pilot_$LABEL.log 2>&1 &
```
Keep the pxh FIFO as emergency console and `commander land` in teardown.

**Validation:** (1) entry parity — `px4_<label>.log` + `/fmu/out/vehicle_status` show arming 1→2 via gesture (pilot log confirms not-fallback) and nav_state 2 (POSCTL) → 14 (OFFBOARD); (2) failsafe signatures — `--drop-rc-at 60` → RC-loss→RTL with return alt 1.5 m (NAV_RCL_ACT=2, RTL_RETURN_ALT=1.5 already in JSON), matching FINAL-SYNTHESIS.md:68; killing reactive_depth_guard mid-Offboard → Offboard→POSCTL at COM_OF_LOSS_T=1.0 s + planner "Lost offboard/armed — returning to INIT" (first-ever sim reproduction); (3) gap-2 leverage — `--no-offboard` stick hover in the EV config reaches the in-flight height-runaway regime that `commander takeoff`'s local-position gate made unreachable.

---

## 4) PROJECTED SCORECARD — AFTER THE QUEUE LANDS

| # | Metric | NOW | AFTER (landing item) | Residual |
|---|--------|-----|----------------------|----------|
| 1 | Stereo /drone | 2.4–2.6 Hz | **≥30 Hz single, 20–30 Hz ×4** (Q9; ceiling = UE render, ~360 renders/s req. [inf]) | UE frame hitches; lockstep Q13 removes even those for regression runs |
| 2 | IMU | ~30 Hz wall-jittered | **195–205 Hz, sensor-stamped** (Q9 + pool patch) | none material |
| 3 | Depth | ~8 Hz 32FC1 | **30 Hz 16UC1 mm** (Q8/Q9) | none |
| 4 | cuVSLAM stutter | 138–153/min | **<5 per 150 s, never consecutive** (Q9); **0 by construction** in lockstep (Q13) | 33.3 ms sits 0.7 ms under the 34 ms threshold |
| 5 | cuVSLAM odom | ~2.6 Hz | **~30 Hz** (Q9) | — |
| 6 | VIO divergence | 9.9 m/190 s | **<1 m** expected (Q9 rates put cuVSLAM in envelope) | re-verify divergence sign convention on first healthy run |
| 7 | EV→EKF2 | duty 0.215, z −408..+36 m | **frac>200 ms = 0, cs_ev duty >0.95, exactly 1 hgt reset, GPS-denied hover z=−1.2±0.3** (Q9+Q2+Q4+Q10+Q11) | EKF2_EV_DELAY retune (~50 ms) after rate lands |
| 8 | EV defense | var 0.01 / q=100 always | **covariance pass-through + health gate + clamp** (Q2) — same binary flies on the Jetsons | — |
| 9 | Flight entry | commander FIFO, failsafes unreachable | **gesture-arm → POSCTL → Offboard; RCL→RTL and Offboard→POSCTL reproducible on demand** (Q3) | — |
| 10 | Pipeline shape | /sim/* reliable | **byte-parity `/camera0/*`, SENSOR_DATA, field launch remaps** (Q8/Q9/Q12); endgame = actual nvblox_examples_bringup launch files minus driver | nvblox `input_qos` confirm against a live drone |
| 11 | Compute | unthrottled | **CPU 270%/10 ms + 10 G cap ≈ Orin (±20%), GPU MPS 2% ≈ Orin (±45%)** (Q6/Q14) → **±10–15% after Q20 calibration** | mem-bandwidth pessimism; GPU latency/VRAM-BW desktop-class (documented floor) |
| 12 | Clock | all wall | **3×1969-epoch + ghost NTP, push-button F9 step repro, zenoh-HLC cross-epoch answer** (Q15, `bin/libfaketime.so.1` preserved) | cuVSLAM-under-LD_PRELOAD untested pairing [inf] — fallback documented |
| 13 | Determinism | free-run | **lockstep mode: exact 35 ms/5 ms cadence, bit-stable stamps, load-immune** (Q13, RTF 0.3–0.7 [inf]) | single-DDS-domain topology (accepted) |
| 14 | Environment | Blocks grid | **Field50 textured field + noise + seeded scatter** (Q16); **Rural Australia = fidelity ceiling** (Q19) | City Sample deferred (RAM) |
| 15 | Coord cycle | PASS | PASS | — |
| 16 | Rig health | 26 FAIL coord4 / 5 FAIL hw4 | **hw4-class → 0 FAIL; coord4-class → ~6–8 FAIL** remaining (zenoh panic, LoRa attach, peer exchange, planner NO-PATH are radio-v2 + rig-bug workstreams — scorecard keeps counting them; Q7 aborts runs early on them) | those workstreams close the rest |

---

## 5) PERMANENT FIDELITY-TRACKING WORKFLOW

Already on disk and validated: `/home/lucas/hercules-sim/fidelity_scorecard.sh` (17 metrics, `[F1]–[F15]` field constants with file:line provenance, exit code = FAIL count) and `/home/lucas/hercules-sim/capture_host_load.sh` (pidstat sidecar; sysstat installed).

**Per-run ritual (every run, every run type):**
```bash
./capture_host_load.sh <label> &                       # 1) sidecar BEFORE the run (compute-parity row)
./run_fleet_coord.sh 4 190                              # 2) the run — after Q7, auto-emits scorecard_<label>.txt
#   optional during run, from a sourced drone domain (ROS_DOMAIN_ID 1-4):
./fidelity_scorecard.sh <label> e1_frames --live        #    live topic-hz + topic-shape-vs-golden rows
./fidelity_scorecard.sh <label> [logdir]; echo "FAILs=$?"   # 3) post-run (or read the auto-emitted file)
echo "$(date +%F) <label> $FAILS" >> scorecard_history.tsv  # 4) trend line — the sim2real gap as one number/run
```

**Rules:**
- **Gate on the exit code.** A run that raises the FAIL count vs `scorecard_history.tsv` is a regression, full stop — this is what catches the next "zenoh was dead the entire run and nobody noticed."
- **Pre-flight aborts (Q7):** 30 s in, zenoh logs grep "panicked" or missing "opened LoRa serial" → abort, don't burn a 190 s run.
- **Constants are sourced, not vibes:** never edit an `[F#]` threshold without re-verifying its cited capture; refresh via Q21 (all-4-drone bundle with coordination stage — run01 is a lower bound) whenever drone firmware/config changes.
- **Both run classes stay tracked:** coord4-class (SimpleFlight fleet) and hw4-class (PX4-in-loop) each keep their own baseline; Q18 adds the pyulog EKF2 rows so PX4 runs are scored on *fusion*, not merely arming — and the same extraction runs on any field `.ulg`, giving true sim-vs-field innovation ratios.
- **Definition of done for this plan:** hw4-class = 0 FAIL; coord4-class = 0 FAIL excluding rows explicitly owned by radio-v2/rig-bug workstreams; `scorecard_history.tsv` flat or falling for a week of runs.