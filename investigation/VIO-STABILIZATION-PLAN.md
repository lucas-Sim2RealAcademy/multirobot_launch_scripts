# VIO Stabilization Plan — HERCULES 4-drone AirSim fleet

**Published:** https://claude.ai/code/artifact/ac14f84a-b239-4b00-819a-68be01893ccf

---

## 1. Root cause

**cuVSLAM is running as pure stereo visual odometry, in a scene where stereo cannot observe translation, while the planner commands the one motion that destroys a rotation-only solution.**

It is not drift. It is a step-jump, and onset is at **t+1.9 to t+13.0 s**, not t+150 s — the 100–600 m runaway is the terminal state of a failure that starts in the first seconds. Horizontal error crosses 2 m at t+0.7 s to t+23.5 s in **20 of 22 traces**. Across 4054 paired samples there are 887 ticks with |db| > 3 m (21.9%).

Three defects, each confirmed by direct source read. Any one fixed alone leaves the failure.

### Defect A — IMU fusion is a no-op `[CONFIRMED IN SOURCE]`

`run_fleet_radio.sh:133` asserts `-p enable_imu_fusion:=true`. It buys nothing. At `visual_slam_impl.cpp:682-688`, every IMU sample in a batch is registered against the **image** timestamp:

```cpp
int64_t imu_ts = static_cast<int64_t>(timestamp.nanoseconds());
RCLCPP_DEBUG(node.get_logger(), "Using imu msg timestamp [%ld]", imu_ts);
const CUVSLAM_Status status = CUVSLAM_RegisterImuMeasurement(
  cuvslam_handle, latest_ts, &imu_measurement);   // imu_ts never used
```

`CUVSLAM_ImuMeasurement` carries no timestamp of its own (`cuvslam.h:135-138`); the API requires the passed timestamp to increment (`cuvslam.h:700-707`). All ~13 samples per window land at one instant → zero-duration preintegration window. No rejection occurs: zero `failed to register an IMU measurement` lines across all four `cuvslam_radio4_*.log`. Combined with `enable_localization_n_mapping:=false`, the stack is stereo-only with no loop closure.

**Cross-stream correction:** two diagnostic streams attributed the runaway to double-integrated accel bias ("0.0089 m/s² → 100 m over 150 s") and gravity leak ("Δθ=15° → 5.1 m"). **Both mechanisms are void** — they require live IMU integration, and Defect A proves there is none. The magnitude agreement is coincidental. This changes sequencing: re-deriving `accel_noise_density` is near-pointless until the IMU is connected, so that fix moves *after* R2.

### Defect B — no triangulable geometry `[MEASURED ON LIVE FRAMES]`

`fx = (640/2)/tan(43.5°) = 337.2 px` (`airsim_realsense_node.cpp:338`) with a 50 mm baseline (`settings-fleet-4drone.json`, front_left Y=−0.025 / front_right Y=+0.025) makes **1 px of disparity = 16.9 m of depth**. Every feature Blocks offers is a box silhouette at 20–85 m → 0.2–0.8 px. Points at infinity; they constrain rotation only.

Measured on real captured frames: Shi-Tomasi 23–215 corners/frame (median ~85), but **usable triangulable landmarks median 16** (range 3–31). 39–90% of stereo matches are sub-1-px.

This is a **depth-band mismatch**, not "too few corners". The ground plane fills 40–60% of every frame at 2.4–4.8 m, where 98–100% of pixels would exceed 2 px — ideal geometry, and the real D435i's actual working band. It yields 0–39 features because `/Game/Flying/Meshes/GrayMaterial` is untextured. Blocks puts 100% of its features outside the sensor's usable range while 100% of its in-range surface is blank. Geometry is also degenerate: only 4–15 of 48 grid cells occupied; at two waypoints *all* features lie on a single vertical box edge — occluding contours, which slide across the object and inject systematic bias, not noise.

### Defect C — we command the worst possible motion `[CONFIRMED IN SOURCE]`

`simple_exploration_planner.py:905-913` — on heading error > 45°, the planner latches current NED position and republishes with a new yaw, i.e. deliberately spins in place:

```python
if abs(yaw_error) > math.radians(45):
    if not self.yaw_align_latched:
        self.yaw_align_ned_x = self.ned_x
        self.yaw_align_ned_y = self.ned_y
        self.yaw_align_latched = True
    ned_yaw = self.flu_yaw_to_ned(travel_yaw)
    self._publish_sp(self.yaw_align_ned_x, self.yaw_align_ned_y, ned_yaw)  # same xy, new yaw
```

`cuvslam_sim_bridge.py:253-258` forwards it as `airsim.YawMode(False, yaw_deg)` — absolute target, **no rate limit** — so SimpleFlight slews as fast as it can. Rotate-in-place is maximum inter-frame rotation with zero translational parallax: the one motion a rotation-only solution cannot survive.

Measured over 9510 GT samples: p90 = 81 °/s, p95 = 125 °/s, **p99 = 207 °/s, max 256 °/s**. 10.1% of flight time exceeds 80 °/s. At 4-drone cadence (85–250 ms gaps) that is 5–50° of rotation between consecutive frames.

### Dose-response is monotone; controls are decisive

| GT yaw rate | Median direction error | Median scale \|db\|/\|dg\| | Mean \|db−dg\| |
|---|---|---|---|
| 0–3 °/s | 6.5° | 1.15 | — |
| 3–10 °/s | 11.0° | — | — |
| 10–20 °/s | 33.4° | — | — |
| 20–40 °/s | 28.2° | 1.40 | — |
| 40–80 °/s | 73.0° | 2.45 | — |
| > 80 °/s | — | **7.13** | **7.52 m** |

The same data bucketed by stereo frame gap shows **no** monotone relationship. Causal ordering holds in every trace — the first GT yaw > 40 °/s precedes or coincides with the first 2 m of error:

- `prev_radio4/thunderstrike` tracked cleanly **297.8 s** (0.67 m), hit its first >40 °/s yaw at t+297.8 s, crossed 2 m **1.4 s later**
- `hw4` (single drone): first >40 °/s at t+51.3 s, err>2 m at t+52.5 s
- `ev1` (single drone): first >40 °/s at t+23.1 s, err>2 m at t+23.5 s
- The only two traces that never exceeded 40 °/s are the only two that stayed bounded: `hw3` (max 35 °/s → **0.4 m over 476 s**) and `coord4_ghost` (max 38 °/s → best 4-drone result, 9.9 m)

### The feedback loop, in the logs

`simple_exploration_planner.py:920` recomputes `travel_yaw_pub = atan2(wy - self.flu_y, wx - self.flu_x)` from the VIO position on *every* setpoint. Once belief is wrong the command whips: `planner_hw4.log` t+50.3→61.8 s shows commanded yaw −14.6° → −233.2° → +87.0° → +139.3° → −18.9° → −81.9° while the aircraft physically moves under 1 m. In `run2_stageA/thunderstrike`, after t+21 s the drone is parked at (11.5, −21.9) while belief sits at (0.5, −1.2) — 23 m standing error, no recovery.

### Three aggravators (real, secondary)

- **The stack cannot report its own failure.** The only non-init message across all four `cuvslam_radio4_*.log` is the frame-delta warning. No tracking-loss, no reset, no relocalization. It publishes a diverging pose at 29.5 Hz and the planner consumes it. Telemetry exists but is off by default (`isaac_ros_visual_slam_core.launch.py:36-37,128-133`).
- **`odom_correction` is a no-op for the planner.** `odom_correction_node.cpp` only calls `tf_broadcaster_->sendTransform(t)` — no Odometry topic — while the planner subscribes to `visual_slam/tracking/odometry` directly (`simple_exploration_planner.py:363-365`). The 20° tilt correction never reaches the consumer, leaving a permanent ~6% scale bias (`cos 20° = 0.94`).
- **ms/ns unit bug disables the sequencer.** `sync` is built with `1e6 * sync_matching_threshold_ms_` but `sequencer` receives raw ms values (`visual_slam_impl.cpp:100-103`), compared against nanosecond deltas (`message_stream_sequencer.hpp:66-67, 88-91, 138-139`). Effective thresholds of 10 ns / 34 ns are always exceeded → both timeouts permanently true → degenerates to "flush every buffered image on every IMU message".

### What is NOT the cause `[FALSIFIED]`

Two independent diagnostic streams converge on these negatives using different evidence. They retire entire workstreams.

| Hypothesis | Verdict | Evidence |
|---|---|---|
| Frame rate / N=4 contention | **Falsified** | Single-drone runs diverge as hard: `ev1` 5765 m, `vio3` 2547 m, `hw4` 2427 m. In radio4 the *highest*-rate drone (buckshee, 17.9 Hz) had the *worst* divergence (607 m). r(stereo Hz, log divergence) = −0.315 over 12 drone-runs, entirely a between-run step. |
| Host load | **Falsified** | Load at jump epochs indistinguishable from baseline (load1 13.95 / GPU 46.5% vs run-wide 12.79 / 56.8%). The fast4 correlation (r=+0.672) is a time confound — r(elapsed, load1)=+0.931; partial flips sign between runs. |
| Frame-timing hitches | **Falsified** | No catastrophic jump coincides with a >200 ms gap; the largest gaps of the run (1078/1089 ms, both at startup) produced no divergence. |
| The ~800/min "Delta … above threshold" warnings | **Cosmetic** | Upstream warns then calls `Track()` regardless — no reset, no drop. At 15 Hz every frame trips the 34 ms default. Loudest red herring in the logs. |
| `rig_frame` / `enable_rectified_pose` tuning | **Dead params** | Verified: neither string exists in `libvisual_slam_node.so`'s parameter table. `run_fleet_radio.sh:137,140` are silently ignored. The rig runs `base_frame=camera0_link`, which is why nvblox logs `Lookup transform failed for frame base_link` all run. |

---

## 2. Ranked fix queue

Ranked by leverage ÷ cost. Tiers are wall-clock effort, not priority — **R4 is ranked fourth but must execute first**, because nothing else can be graded without it.

| # | Fix | Tier | Removes | Conf. |
|---|---|---|---|---|
| **R1** | **Slew-limit yaw to 20 °/s in the bridge** | minutes | The trigger | high |
| **R2** | **Patch cuVSLAM IMU timestamp + ms/ns unit bug** | hours | Defect A | high |
| **R3** | **Texture ground + boxes in place** | hours | Defect B | high |
| R4 | Enable VIO telemetry; jitter threshold → 250 ms | minutes | Blindness *(do first)* | high |
| R5 | `HERC_DEPTH_HZ=5 HERC_CAPTURE=0` | minutes | Rate as a confound | measured |
| R6 | `odom_correction` publishes Odometry + health gate | hours | Silent propagation | high |
| R7 | Kill rotate-in-place; low-pass travel yaw | hours | The feedback loop | high |
| R8 | Stamp images at capture instant, not readback | hours | 35 ms t_d + jitter | measured |
| R9 | Re-derive IMU noise densities *(after R2)* | minutes | Over-trust | [inf] |
| R10 | Poll IMU at 333 Hz; drop dedupe losses | hours | Aliasing | [inf] |
| R11 | DepthPerspective + cosine LUT in our node | hours | 43% of render budget | measured |
| R12 | Q16 Field50 + near-field clutter scatter | day | Defect B, properly | high |
| — | Lockstep (Q13) — **defer**, fund as debugging instrument | 3–5 days | <20% expected | low |
| — | **Do not do:** `NoiseSettings`, 424×240 stereo, capture stagger | — | Nothing | harmful / null |

**Sequencing:** run R4 + R5 first (same 10 minutes, zero code change) so the run is instrumented and rate is off the table. Then **R1 alone**, as a clean single-variable A/B — the cheapest possible test of the whole diagnosis. Then R2 and R3, which make the system robust rather than merely un-triggered.

---

### R1 — Slew-rate-limit the yaw command to 20 °/s `[minutes]`

Caps inter-frame rotation at 20 °/s × 0.25 s (worst gap) = 5°, moving every sample out of the >12° bucket (scale 7.13, mean |db−dg| 5.95 m) into the 3–6° bucket (scale 1.86) or better. The `hw3`/`coord4_ghost` controls predict this alone moves time-to-2 m from t+2 s past t+60 s.

**Code delta — `/home/lucas/hercules-sim/cuvslam_sim_bridge.py`**

```python
# near the top, with the other module constants
+ YAW_RATE_LIMIT_DEG_S = float(os.environ.get('HERC_YAW_RATE_LIMIT', '20.0'))

# in __init__
+ self._yaw_cmd = None
+ self._yaw_cmd_t = None

# in _apply_sp, replacing direct use of yaw_deg at :253-258
  yaw_deg = math.degrees(self.sp.yaw) if math.isfinite(self.sp.yaw) else 0.0
+ now = time.monotonic()
+ if self._yaw_cmd is None:
+     self._yaw_cmd, self._yaw_cmd_t = yaw_deg, now
+ else:
+     err = (yaw_deg - self._yaw_cmd + 180.0) % 360.0 - 180.0
+     max_step = YAW_RATE_LIMIT_DEG_S * max(0.0, now - self._yaw_cmd_t)
+     self._yaw_cmd += max(-max_step, min(max_step, err))
+     self._yaw_cmd_t = now
  self.ctl.moveToPositionAsync(
      px, py, pz, 1.5,
      drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
-     yaw_mode=airsim.YawMode(False, yaw_deg), vehicle_name=VEH)
+     yaw_mode=airsim.YawMode(False, self._yaw_cmd), vehicle_name=VEH)
```

```bash
cd /home/lucas/hercules-sim
HERC_YAW_RATE_LIMIT=20 HERC_DEPTH_HZ=5 HERC_CAPTURE=0 ./run_fleet_radio.sh 4 180
```

**Validation.** PASS = max GT yaw rate < 40 °/s on all four AND `t(err>2m)` moves from 1.9–13.0 s to >60 s on ≥3 of 4. Confirm yaw p99 drops 207 → <25 °/s. **If it does not, the limiter is being bypassed by SimpleFlight's own controller and you need R7 instead** — that is the specific disconfirmation to watch for.

```bash
python3 investigation/vio/lead.py    # causal ordering + t(err>2m) per drone
python3 investigation/vio/stats.py   # yaw-rate distribution, expect p99 < 25
```

---

### R2 — Patch cuVSLAM so the IMU is actually fused `[hours]`

A one-token change turns `enable_imu_fusion:=true` from a lie into the truth. The preintegrator gets a real non-zero window, so the inertial prior can bridge the 60–190 ms visual gaps — and cuVSLAM should start reporting tracking loss *honestly* instead of silently publishing wrong poses. Bug confirmed present on upstream `main`. Installed version is **3.2.6**, not 3.2.5 (`version_info.yaml`, commit `f3d2447`).

**Code delta — `/home/lucas/UE5/hercules-sim-big/src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp`**

```cpp
// :687-688 — register each sample at its own timestamp
  const CUVSLAM_Status status = CUVSLAM_RegisterImuMeasurement(
-   cuvslam_handle, latest_ts, &imu_measurement);
+   cuvslam_handle, imu_ts, &imu_measurement);

// :102-103 — ms -> ns, matching the sync ctor two lines above
- sequencer(node.imu_buffer_size_, node.imu_jitter_threshold_ms_,
-   node.image_buffer_size_, node.image_jitter_threshold_ms_),
+ sequencer(node.imu_buffer_size_, 1e6 * node.imu_jitter_threshold_ms_,
+   node.image_buffer_size_, 1e6 * node.image_jitter_threshold_ms_),
```

```bash
cd /home/lucas/UE5/hercules-sim-big
colcon build --packages-select isaac_ros_visual_slam isaac_ros_visual_slam_interfaces \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Repoint `run_fleet_radio.sh:131` from `/opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam` to the built overlay; source that overlay after `ros2_ws` in the script header.

**Required companion change.** Once the unit bug is fixed the thresholds bite in real units, and the defaults (34 ms image / 10 ms IMU) sit below the measured p50 frame spacing of 64 ms — the sequencer would force-flush constantly. Pass both explicitly:

```
-p image_jitter_threshold_ms:=200.0 -p imu_jitter_threshold_ms:=15.0
```

**Validation.** (1) Single drone with `-p verbosity:=2`; confirm the `Using imu msg timestamp` DEBUG lines are strictly increasing and ~5 ms apart, not N repeats of the image stamp. (2) Confirm no burst of `CUVSLAM has failed to register an IMU measurement` (would mean out-of-order stamps). (3) Re-run the fleet: PASS = no drone's |belief|−|truth| gap exceeds 10 m at t+150 s (today buckshee is +457.5 m).

**Only after this passes**, apply R9 — measured densities accel 0.0138/0.0394/0.0169 vs configured 0.001862, gyro 6.2e-4/2.6e-4/5.7e-4 vs 2.44e-4. Note the sign is opposite the usual guess: cuVSLAM is told the accelerometer is **8–21× quieter** than it measurably is.

---

### R3 — Texture the world in place: two materials, no new assets `[hours]`

The map contains exactly one `Ground` actor using `/Game/Flying/Meshes/GrayMaterial` plus many `TemplateCube_Rounded_*` using `/Game/Geometry/Meshes/CubeMaterial`. Editing those two re-textures the entire world with no map edit and no disk cost. Projecting a world-locked pattern onto the *real captured depth maps* raises Shi-Tomasi from 23–215 to 149–1530 and usable landmarks from median 16 to ~60–130 — putting features on the ground at 2.4–4.8 m (3.5–7 px disparity), restoring metric translation observability.

```bash
pkill -f 'UnrealEditor.*[B]locks'          # wait for 41451 to close
B=/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks
cp $B/Content/Flying/Meshes/GrayMaterial.uasset{,.bak}
cp $B/Content/Geometry/Meshes/CubeMaterial.uasset{,.bak}

cd $B && /home/lucas/UE5/UE5.2.1/Engine/Binaries/Linux/UnrealEditor "$PWD/Blocks.uproject" \
  -run=pythonscript -script=/home/lucas/hercules-sim/investigation/vio/texturize.py \
  -unattended -nosplash -RenderOffscreen -stdout
```

**`investigation/vio/texturize.py`**

```python
import unreal
MEL = unreal.MaterialEditingLibrary
for path, scale in (("/Game/Flying/Meshes/GrayMaterial", 8.0),
                    ("/Game/Geometry/Meshes/CubeMaterial", 4.0)):
    m   = unreal.EditorAssetLibrary.load_asset(path)
    wp  = MEL.create_material_expression(m, unreal.MaterialExpressionWorldPosition, -900, 0)
    mul = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -700, 0)
    mul.set_editor_property("const_b", 0.01)          # UE cm -> m
    n1  = MEL.create_material_expression(m, unreal.MaterialExpressionNoise, -500,  60)
    n2  = MEL.create_material_expression(m, unreal.MaterialExpressionNoise, -500, -60)
    for n, s in ((n1, scale), (n2, 0.35)):            # TWO octaves - see trap below
        n.set_editor_property("scale", s)
        n.set_editor_property("levels", 4)            # keep <= 4: per-pixel cost x8 renders
        n.set_editor_property("output_min", 0.20)
        n.set_editor_property("output_max", 1.0)
        n.set_editor_property("function", unreal.NoiseFunction.NOISEFUNCTION_SIMPLEX_TEX)
    lerp = MEL.create_material_expression(m, unreal.MaterialExpressionLinearInterpolate, -300, 0)
    MEL.connect_material_expressions(wp, "", mul, "A")
    MEL.connect_material_expressions(mul, "", n1, "Position")
    MEL.connect_material_expressions(mul, "", n2, "Position")
    MEL.connect_material_expressions(n1, "", lerp, "A")
    MEL.connect_material_expressions(n2, "", lerp, "B")
    MEL.connect_material_property(lerp, "", unreal.MaterialProperty.MP_BASE_COLOR)
    MEL.recompile_material(m)
    unreal.EditorAssetLibrary.save_asset(path)
```

**Two traps, both measured.** (a) Use two octaves, never a single frequency or a checker — a pure repeating pattern *lowered* usable landmarks at 2 of 8 waypoints through descriptor aliasing (bk_t08: 20 → 15). Macro variation is what makes matches unique; matches the anti-tiling note at `GAP-CLOSURE-PLAN.md:62`. (b) **Do not touch `NoiseSettings`.** It is a screen-space post-process (`docs/settings.md:338-347`): screen-locked so it yields no trackable landmark, generated independently per camera so it destroys stereo correspondence, and at low `RandSpeed` becomes a fixed screen pattern biasing VIO toward zero motion. It adds anti-features.

**Validation.**

```bash
/home/lucas/hercules-sim/venv/bin/python \
  /home/lucas/hercules-sim/investigation/feature_probe/capture_waypoints.py
python3 /home/lucas/hercules-sim/investigation/feature_probe/measure_features.py
```

PASS = Shi-Tomasi ≥ 3× baseline at every waypoint (the existing Q16 criterion, `GAP-CLOSURE-PLAN.md:62`) AND usable(>2 px) ≥ 100 at 6 of 8 waypoints AND occupied grid cells ≥ 30/48. Confirm UE renders/s has not fallen below the 227/s baseline.

Cheap and independent, same session: `-p enable_ground_constraint_in_odometry:=true` — verified a real 3.2.6 parameter (unlike `rig_frame`), and the fleet flies at constant ~1 m over a perfectly flat plane, exactly the case it exists for. Also delete the dead overrides at `run_fleet_radio.sh:137,140`.

---

## 3. Validation protocol

**Standard run**

```bash
HERC_DEPTH_HZ=5 HERC_CAPTURE=0 ./run_fleet_radio.sh 4 190
# in the cuVSLAM block of run_fleet_radio.sh:
#   -p enable_landmarks_view:=true -p enable_observations_view:=true
#   -p image_jitter_threshold_ms:=250.0
# and per drone:
ros2 bag record -o $LOG/vslam_${LABEL}_$VEH \
  /visual_slam/status /visual_slam/vis/landmarks_cloud \
  /visual_slam/vis/observations_cloud /visual_slam/tracking/odometry &
```

**Do this before anything else.** The analysis scripts that produced every number in this document (`align.py`, `big.py`, `lead.py`, `div.py`, `div2.py`, `an.py`, `jump.py`, `stats.py`) currently live in an **ephemeral session scratchpad** and will be lost. Move them to `/home/lucas/hercules-sim/investigation/vio/` and commit them — otherwise none of the thresholds below are reproducible. All paths here assume that move.

| Metric | How | Now | Gate A | Gate B | Gate C |
|---|---|---|---|---|---|
| **t(err > 2 m)** per drone | `lead.py` | 1.9–13.0 s | >60 s, 3/4 | >150 s, 4/4 | no crossing |
| **Max horizontal error** | `div.py` | 10/80/670/23 m | <50 m all | <25 m all | <5 m all |
| **Error at t+150 s** | `div2.py` | +457.5 m worst | <25 m | <10 m | <3 m |
| **Step-jump rate** (ticks >3 m) | `jump.py` | 887/4054 = 21.9% | <5% | <1% | 0 |
| **Scale** \|db\|/\|dg\|, all yaw buckets | `big.py` | 1.15 → 7.13 | <2.0 | 0.9–1.2 | 0.9–1.2 |
| **Direction error**, all yaw buckets | `big.py` | 6.5° → 73° | <30° | <15° | <15° |
| **GT yaw rate p99** | `stats.py` | 207 °/s | <25 °/s | <25 °/s | <25 °/s |
| **Frame delta p50 / p90** | `an.py` | 63.8 / 118.2 ms | <50 / <80 | <50 / <80 | <40 / <60 |
| **Usable landmarks** / frame | `observations_cloud` | 16 median | — | ≥100 | ≥150 |
| **`vo_state==2`** count | `/visual_slam/status` | unobservable | reported at all | >0 and honest | 0 |
| **% plans "Path complete"** | planner logs | 11% | ≥35% | ≥50% | ≥75% |

**On stutter/min.** Report the frame-delta *distribution*, not the warning count. Once `image_jitter_threshold_ms` is raised the count becomes meaningless by construction — it is ~800/min per drone only because the 34 ms default sits below every frame interval at 15 Hz. The honest measure is p50/p90/p99 of the interval series, plus the fraction of wall-clock inside >34 ms gaps (today: 161.1 s of 163.0 s = 99%).

### The decisive experiment

One experiment separates "yaw is the trigger" from "the scene has no features" and fills a genuine hole: **there is no existing data point for one drone, C++ sensor node, 30 Hz stereo / 200 Hz IMU, full stack, in Blocks.** The `ev1`/`fleet2`/`coord4` runs all predate the C++ node and ran the 2.4–2.6 Hz Python bridge, so every N=4 conclusion is confounded until this exists.

1. N=1, `HERC_STEREO_HZ=30 HERC_DEPTH_HZ=30 HERC_IMU_HZ=200 HERC_CAPTURE=0`, 190 s
2. Scripted trajectory containing a deliberate 180° in-place yaw at t+60 s
3. Run the 2×2 matrix: {untextured, textured} × {unlimited yaw, 20 °/s limit}
4. Three repeats per cell, to see whether divergence is reproducible at N=1 or still random

PASS for "yaw was the trigger": error < 2 m across the yaw event and < 5 m at t+190 s in the limited cells. PASS for "texture was necessary": untextured/limited still degrades while textured/limited holds. **If the untextured/unlimited cell does not blow past 100 m within 10 s of the yaw** — contradicting `ev1` and `hw4` — the diagnosis is wrong and the investigation restarts. Cross-check rates held: `rsnode` must show stereo ≥ 29 Hz, IMU 195–205 Hz, `err s/d/i 0/0/0`.

### Automate the gate

Fold the divergence extractor into `fidelity_scorecard.sh` as a per-drone row exiting non-zero above 5 m; record every run in `scorecard_history.tsv`. `div.py` already resolves the y-sign convention by fitting the first 15 samples, closing the open residual at `GAP-CLOSURE-PLAN.md` row 6.

---

## 4. What this unblocks

Stage B coordination needs ≥50% of plans reaching "Path complete". We are at 11% — and that is not a planner quality problem, it is a direct arithmetic consequence of the VIO failure through four compounding paths:

- **Goals become unreachable by definition.** The planner declares "Path complete" on *believed* arrival. Once belief is 100 m from truth, the aircraft chases a phantom and the real goal is never approached. With onset at t+2 s on most drones, nearly every plan in a run is already poisoned.
- **The map is built in the wrong frame.** nvblox integrates depth at the diverged pose, so the ESDF encodes obstacles that are not there and misses ones that are. FIS frontier selection then operates on a fictional map — and at least one drone lost its mapper outright (`nvblox_radio4_thunderstrike.log` ends in a `MultiThreadedExecutor::run` crash trace).
- **Inter-drone coordination is meaningless.** The zenoh + virtual-LoRa layer exists to share positions and deconflict. Sharing a position 100–600 m wrong makes task allocation, deconfliction, and multi-drone map merging strictly worse than not coordinating at all.
- **Results are uninterpretable.** A different drone runs away each run, so any coordination A/B is dominated by which aircraft happened to lose VIO, not by the policy under test.

Projected recovery **[inf]**: Gate A (R1 + R4 + R5) should reach roughly 35–60%, since the 11% is dominated by drones diverging in the first 2–13 s and the `hw3` control held 0.4 m over 476 s once yaw stayed under 40 °/s. Gate B (adding R2 + R3) should clear 50% with margin and, more importantly, clear it *robustly* — R1 alone avoids the trigger, while R2 and R3 remove the reason the trigger is fatal. **Run the Stage B gate only after Gate B**, or a passing score will not survive the first trajectory that yaws.

Secondary unblocks: the R6 health gate converts a silent 670 m runaway into a detectable, recoverable hold (prerequisite for any autonomous multi-hour run); R4's telemetry makes "cuVSLAM lost tracking at t=95 s with 11 landmarks" a measurement rather than an inference.

---

## 5. What cannot be fixed in sim

Inherent to Blocks and AirSim's architecture. Carry as known deviations; do not let a green sim scorecard imply the real rig is safe.

1. **Photometric perfection.** No sensor noise, rolling shutter, motion blur, auto-exposure, or vignetting. Texturing restores feature *count* but not the *statistics* of real imagery — descriptor stability, illumination change, exposure transients are what break VIO outdoors and none exist here. AirSim's only noise knob is screen-space and left/right-uncorrelated, so it makes stereo strictly worse.
2. **The IMU is not a sensor model.** `ImuSimple` emits numerically-differentiated physics acceleration (`ImuSimple.hpp:57`) with `GenerateNoise` defaulting false (`ImuSimpleParams.hpp:73`). Measured white sigma 0.25–0.72 m/s², but *zero* bias, bias random walk, scale-factor error, temperature drift, and saturation. Sim IMU noise is qualitatively wrong in both directions at once. You can match the white term; you can never exercise a real VIO's bias-estimation path. Conclusions about bias observability are untransferable.
3. **Image timestamps are architecturally late.** Stamp taken on the render thread after GPU readback (`RenderRequest.cpp:187`) while the baked pose is sampled on the game thread (`RenderRequest.cpp:68-74`) — measured 35 ms late at correlation 1.0000. R8 removes the mean offset, but `ReadSurfaceData` is explicitly the "undocumented method that avoids flushing", so returned pixels can be from an older frame than the request. Fixing that requires flushing, which costs throughput. Residual load-dependent t_d jitter (sd 11.7–14.0 ms at N=4) is inherent; real hardware timestamps at the sensor.
4. **The render ceiling is real and shared.** 227 renders/s, 16 renders per cycle at N=4 → 14.2 Hz, matching the measured 12.9–16.9 Hz exactly. R5 + R11 buy ~2×, reaching ~25–29 Hz. Beyond N≈4–5 this host cannot deliver field rates at any resolution, because per-batch overhead (5–8 ms) does not scale with pixels — which is also why 424×240 buys only +20–30% while destroying features. Real drones do not share a renderer.
5. **Recovery is impossible by construction outside the field.** `buckshee`'s ground truth ends 108 m from spawn, where the only visible surface is bare ground. Sim failures are *terminal* where real ones might be transient — the sim overstates failure severity in this specific way. Extend the field or geofence the planner.
6. **Loop closure cannot be validated here.** `enable_localization_n_mapping:=false` is our choice and can be flipped, but with 16 usable landmarks and dozens of visually identical gray boxes, place recognition would be unreliable or actively harmful. The sim cannot exercise this path honestly even when enabled.
7. **Lockstep may not reach 200 Hz IMU.** `SteppableClock` is constructed with `DefaultStepSize * clock_speed` = 20 ms (`SimModeBase.cpp:1710-1711`) rather than `getPhysicsLoopPeriod()` = 3 ms (`SimModeWorldBase.h:75`). If sim time advances in 20 ms grains the IMU ceiling is 50 Hz, not the 200 Hz cuVSLAM is configured for. **Settle this in 30 minutes before writing any Q13 port code** — if unpatchable, the port is dead on arrival.
8. **Per-drone `ROS_DOMAIN_ID` and a single `/clock` are incompatible.** One synced node publishes `/clock` on one domain; with `use_sim_time` and no clock, rclcpp returns time 0 forever and nvblox, message_filters, and the planner all silently stop. Collapsing to one domain destroys the zenoh + virtual-LoRa topology the fleet experiment exists to test.
9. **CUDA nondeterminism inside cuVSLAM.** Even under bit-exact lockstep, reduction order can vary. If divergence still varies run-to-run under identical stamps, the cause is inside the closed-source tracker and no simulator work can reach it.

**Verdict on lockstep (Q13):** it will not fix divergence. The mechanism it removes — frame-interval jitter — is demonstrably not the mechanism causing divergence: warnings are cosmetic, jumps do not coincide with gaps, and N=1 on an idle host already diverges. Expect <20% reduction for 3–5 days of work at RTF 0.22–0.27 (a 190 s mission becomes 12–14 minutes). Its real value is *reproducibility* — converting "a different drone each run" into a deterministic failure you can bisect. Fund it as a debugging instrument after Gate B. If built, build the reduced form: 30 ms image cadence (not Q13's 35 ms, which trips the 34 ms threshold on every frame), depth at 5 Hz via DepthPerspective, `/clock` derived from `responses[0].time_stamp` rather than `step*dt` (otherwise a ~56-year epoch skew silently kills every TF lookup), and the four paused-sim vehicle reads issued in parallel across four connections for RTF 0.7–1.0.

---

*All source claims verified by direct read at the cited file:line. Measurements from `e1_frames/` logs (radio4, run1_senderonly, prev_radio4) and live probes. `[inf]` marks inference; everything else is measured or read from source. Note `e1_frames/run2_stageA/` is a byte-identical copy of the current root radio4 logs (md5 `a3b1606b…`) — there are three distinct radio4 runs, not four.*