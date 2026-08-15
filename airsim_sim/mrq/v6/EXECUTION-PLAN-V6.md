# HERCULES Fleet Cinematic — EXECUTION PLAN (v6)

**Diagnosis folded in:** mrq6's weak flight was **host load** (live rviz+x264 during flight → stereo 18.5→11 Hz, stall streak 39s→197s, thunderstrike odom diverged 32.3m), not planner drift. The stall is a pinned geofence-edge mechanism (goal at fence x=9.6 → A* waypoint outside fence → skip path never appends `visited_positions` → same goal replanned at 2 Hz). Fix strategy = **remove load, widen spawns, re-fly + select, frame around residue**. The cinematography rebuild = 6 designed shots with numeric pre-render QC, anchored by a top-down "starburst" opening and a world→map match cut.

**KILLED (do not attempt):** planner code changes (incl. the visited-positions one-liner — selection + framing covers it, patching mid-shoot invalidates scorecard comparability); per-drone spawn **yaw** (breaks `team_box.py`/scorecard NED→FLU yaw-0 assumption); claim-radius knobs (not wired into `run_fleet_radio.sh`); `HERC_GOAL_THRESH`/`HERC_COLLISION_THRESH` as stall fixes (geofence-skip bypasses arrival); zero-stall expectations (min archived streak is 31s); live rviz/ffmpeg during flight; 9:16 primary (this is the established 16:9 pipeline; compose already emits `*_mobile.mp4`).

All file paths below verified on disk 2026-08-14.

---

## 1) FLIGHT ACQUISITION PROTOCOL

### 1.1 New capture script (bag-only, kills the load)
Create `/home/lucas/UE5/hercules-sim-big/mrq_work/mapview/capture_fleet_bagonly.sh` from `capture_fleet_map.sh`:
- **KEEP:** 4× `relay_drone.py` (src domains 1–4 → sink 42, lines ~20–29), `ros2 bag record -o $M/bag_$LABEL /d{1..4}/mesh /d{1..4}/path /d{1..4}/drone /tf /tf_static` (lines 32–34), the guarded call `/home/lucas/hercules-sim/investigation/run_exp.sh "$LABEL" "$SECS"` (lines 76–77), teardown.
- **DELETE:** rviz2 block (43–48), all `scrot` blocks (50, 61–70, 83), ffmpeg x11grab block (54–58).
- **ADD preflight gate** (run_exp.sh checks stack orphans but NOT host load — its `load1_pre` was already 3.14 before mrq6): refuse launch if `cut -d' ' -f1 /proc/loadavg` ≥ 4.0, or `pgrep -f UnrealEditor-.*Kungfu\|x264\|ffmpeg.*x11grab` non-empty; print a warning on any `chrome` render/join processes.

### 1.2 Config — Tier 1 (primary): cross spawns + bigger arena
Create `/home/lucas/hercules-sim/settings-fleet-4drone-cross.json` = copy of `settings-fleet-4drone.json` changing ONLY per-vehicle X,Y (current column: ghost −7,−7 / delta −7,−4 / buckshee −7,−1 / thunderstrike −7,2 — all facing +x east, which is why the east fence is the shared stall attractor):

| drone | X | Y | Yaw |
|---|---|---|---|
| ghost | −13 | −7 | absent/0 |
| delta | −13 | 5 | absent/0 |
| buckshee | −1 | −7 | absent/0 |
| thunderstrike | −1 | 5 | absent/0 |

12m spacing diversifies initial frontiers and delays fence contact. Safe because `team_box.py` derives per-drone bboxes from spawn X,Y (yaw 0 assumed), `HERC_SETTINGS` is honored (`run_fleet_radio.sh:63`), relocated spawns are proven (`settings-overview*-4drone.json` at x=15/x=21), and MAP-SURVEY-52.md documents a 169×99m clear box at the spawn line. Footprint ≈36×36m fits the 38m clear square.

**Env:** `HERC_ARENA_HALF_M=12` (honored `run_fleet_radio.sh:91` + `fidelity_scorecard.sh:257–263`; divergence gate auto-scales to 2×fence=24m), `HERC_CAPTURE=0`, `HERC_DEPTH_HZ=10.0`. **Duration 300s** (larger arena; ~600–750 unique cells projected at L1 coverage rates [inf]).

### 1.3 Shoot command (per attempt)
```bash
cd /home/lucas/UE5/hercules-sim-big/mrq_work/mapview
HERC_SETTINGS=/home/lucas/hercules-sim/settings-fleet-4drone-cross.json \
HERC_ARENA_HALF_M=12 HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 \
./capture_fleet_bagonly.sh shoot_cN 300      # N = 1..5; labels must be new (run_exp refuses reuse)
```
Attempt 1 doubles as config validation: require no "Waypoint outside geofence" storm before t+90s (grep planner logs), PLAN OK ×4, stereo ≥16 Hz. If the cross layout misbehaves structurally (spawn collision, instant geofence storm) → **Tier A fallback:** stock `settings-fleet-4drone.json` + `HERC_ARENA_HALF_M=10`, 240s (= L1_base conditions + bag; L1_base hit 607/630 in 150s under these).

### 1.4 Protocol: n = 5 attempts, score each immediately, early-stop
~8–9 min/attempt ≈ 60–75 min wall. Score each with `score_run.py` (below) right after teardown. **Stop early** when SCORE ≥ 640 AND all camera floors met. If 0/5 pass floors: extend to 7; if still 0, take best-by-score and shoot around its stall (§1.6 timing rule). Every attempt archives normally under `/home/lucas/hercules-sim/e1_frames/runs/` (failures auto-archived `<label>_INVALID`).

### 1.5 Gates (reject the run outright)
- **G1** run_exp validation passed (no `_INVALID`), PLAN OK all 4 planners.
- **G2** scorecard odom divergence ≤ 2×fence for ALL drones — no `fleet_drones_excluded` (blocks drift-inflated "exploration"; this is what disqualified mrq6-thunderstrike's fake 35m). From `runs/<label>/scorecard_<label>.txt`.
- **G3** bag ≥ **300** `/dN/mesh` msgs **per drone** (`ros2 bag info` or reuse `mapview/scan_bag_mesh.py`; mapview2 baseline 626–794 per 240s).
- **G4** stereo ≥ **16 Hz** all drones (grep `--- sensors` in `run_*.out`; L1_base 18.5–19.3, mrq6 11.1–11.8).

### 1.6 Selection score — new `/home/lucas/hercules-sim/investigation/score_run.py`
Inputs all exist per run dir: `scorecard_<label>.txt`, `planner_<label>_<drone>.log`, `<label>_<drone>/meta_%05d.json` (fields t/ned/state, ~1090 frames/drone/240s).

```
SCORE = U + 1.5·P_min + 120·F_min − 0.8·T_stall
  U       = fleet unique cells (GATED variant if present) from the scorecard
  P_min   = min per-drone XY path length (m) from consecutive meta ned deltas
  F_min   = min per-drone moving fraction (|v| > 0.3 m/s between metas)
  T_stall = max per-drone longest CONSECUTIVE same-goal streak (s), regex goal=\([^)]*\) in planner logs
```
Calibration on archive (sanity for the script): L1_base ≈ 659, tb2 ≈ 657 … mrq6 ≈ 345, mapview2 ≈ 278.

**Camera-worthiness floors for the winner:** `P_min ≥ 30 m` AND `F_min ≥ 0.15` AND `T_stall ≤ 90 s` (L1-class runs pass all three; mrq6 fails all three).

**Stall-timing rule:** `score_run.py --timeline` emits `stall_timeline_<label>.json` (per-drone start/end of every streak > 20s). If the winner's worst streak falls inside the **first 90s** (the fan-out beat), take the runner-up — the opening must show all four moving; late stalls are cuttable.

### 1.7 Handoff bundle to the render stage (one run, both deliverables)
- `runs/<label>/<label>_<drone>/meta_*.json` → UE trajectory export.
- `mapview/bag_<label>/` → merged-map replay. **Same label for both — hard mandate.**
- `stall_timeline_<label>.json` + arena bbox printout (drives temple SCALE, §4 step 3).
- Window report from `window_select.py` (§2.0) proving the run supports every shot before any render.

---

## 2) SHOT LIST

Constants (verified: `build_fleet_seq6_clean.py:382–386`): 1920×1080, CineCamera 18.0mm, filmback 23.76×13.365 → half-HFOV 33.4°, half-VFOV 20.35°. Drone screen size ≈ **145600/d_cm px** (146px @10m, 73px @20m, 49px @30m). Key trick: **lock camera yaw to the fleet's PCA spread axis** so spread uses the horizontal 33.4° half-angle → required slant = 2.0R instead of 3.42R (drones 1.7× bigger).

### 2.0 Window selection + QC harness (build FIRST)
- `temple/window_select.py` — scores candidate windows from the winning keys: **W_open** = 15s from first motion, maximizing min-per-drone displacement + goal-bearing spread (need ≥8m each, spread ≥120°; if takeoff is slow, retime ≤2× with the existing amber "2x speed" chip); **W_main** = 80–100s contiguous with min per-drone path ≥10m, ≥50% frames with ≥3 simultaneous movers (|v|>0.3), and one pairwise closest approach in 6–14m (all 6 pairs computed). Refuses to bless a run that can't feed every shot.
- `temple/shot_qc.py` — extract `project()` (design_camera_v3.py:63, 50cm near-clip), `moving_avg` (:45), yaw-unwrap into a shared module. Metrics per shot: per-drone worst frame-frac, px min/max, in-frame fraction, screen-velocity bearing spread, pairwise screen separation, cumulative screen displacement, occlusion rays vs `offender_aabbs.json` + `probe/{corridor,fine}.binvox`, near-clip violations. Every `design_camera_*` script imports it and **refuses to write keys that fail its thresholds**. Second gate per shot: 5 sparse MRQ frames (first/25/50/75/last) via the two-stage headless render, adjacent-frame diff must count the claimed number of moving blobs. Bad takes die in <1s CPU, not 4-min renders.

### SHOT 1 — "STARBURST" rising god shot — 10s — serves: THE opening (fan-out money shot)
Spread geometry only reads from above. Camera starts 900cm above spawn centroid, pitch −75° (drones ~160px); per frame compute fleet PCA axis and slant D(k)=R(k)/tan(0.8·33.4°) with yaw locked to PCA (slerp ≤10°/s); rise = running-max(D) + smoothstep (crane only pulls up/back, never reframes down); aim = 1.0s moving-avg centroid; ease pitch −75°→−55° over last 3s to introduce the temple horizon (hands off to Shot 2). New `temple/design_camera_opening.py` (~150 lines on v3 kernels). Vertical clearance over the spawn zone verified with `probe/slab_analysis.py` before commit (camera lives 9–32m up).
**QC:** all-4 in frame every frame at frac ≤ 0.85; min pairwise screen separation ≥ 0.15 frame-widths (288px) by t=6s; screen-velocity bearing spread ≥ 180° (divergence, not co-travel); min drone ≥ 60px; occlusion rays = 0 hits; blob check = 4.

### SHOT 2 — "THROUGH-THE-LINE" retreating crane — 7s — serves: opening energy, second read of divergence
Camera low ahead of the fleet on its mean initial heading: p_cam(k) = centroid_smooth(k) + u_heading·(1400cm + 1.3·∫centroid_speed), z 250→900cm smoothstep; aim centroid, 0.8s smoothing, unwrapped yaw. Drones fly toward camera and split past the frame edges. Same script, mode 2.
**QC:** ≥2 drones change screen-x sign during shot; hero frac ≤ 0.6; min approach ≥ 350cm; nothing behind near-clip until final 15 frames. Shorten to 6s if the fleet splits early.

### SHOT 3 — low lateral TRACK with a crosser — 10s — serves: mid-cut speed/alive feel
Hero = longest-path drone in W_main; window = pair closest-approach event (6–14m). Camera 180cm AGL, 600cm lateral offset from hero's 2.5s-smoothed path; aim per playbook 8-frame finite-diff lookahead. New `temple/design_camera_track.py`.
**QC:** hero 120–260px all frames; second drone in frame ≥40% of frames at ≥35px; hero cumulative screen displacement ≥1.5 frame-widths; occlusion pass. **Fallback** (no qualifying approach): v2 elevated hero-track (`design_camera_v2.py`, already user-accepted).

### SHOT 4 — depth-stack dolly (v3, re-solved) — 10s — serves: "the whole fleet at work" (user-accepted grammar)
Automated axis search over new keys: grid azimuth (5° steps) × distance 8–14m minimizing max angular off-axis spread of all 4 over the max-simultaneous-movers window; 220cm smoothstep dolly; aim 1.0s-smoothed mover centroid. Parameterize `design_camera_v3.py`'s P0/P1/axis (it already computes the metrics).
**QC:** all-4 within 5° of axis; near drone ≥120px, far ≥45px; frac ≤ 0.9; mover screen-speed sum ≥ the v4 rendered baseline (read exact value from the v4 run log [inf]); foliage probe on the new azimuth (the check that caught the v3 southwest curtain).

### SHOT 5 — 120° ribbon arc over the covered area — 12s — serves: "large area swept," literal
Spawn `ribbon_{ghost,delta,buckshee,thunderstrike}.obj` (from `bake_ribbons.py`, regenerate for the new run) via sequencer visibility track on this shot's first frame, 2s fade-in. Arc: z=1800cm, pitch −30°, radius 1.25× footprint half-diagonal, az(t) = 240° + 120°·(1−smoothstep) so it **ends at temple azimuth 240°** = Shot 6's angle (seamless handoff); yaw rate ≤12°/s; aim fixed at trajectory-bbox center. New `temple/design_camera_arc.py`.
**QC:** ≥90% of every drone-trajectory bbox corner projections in frame at frac ≤ 0.95 through the whole arc; all 4 final drone positions in frame; occlusion rays from 25 arc samples × 8 bbox corners pick the clean sector.

### SHOT 6 — map-matched continuity hold — 5s — serves: world→map proof cut
Mirror `mapview/fleet_map.rviz` (verified: Distance 38, Pitch 0.62 rad = 35.5°, Yaw π, Focal (−5.8,−1.2,0)): UE camera at temple azimuth 240° from footprint center, **slant = 3800·SCALE cm** (scales with the shrunk footprint so screen extents match the unscaled RViz map), pitch −35.5°: cam = center + slant·[cos35.5°·cos240°, cos35.5°·sin240°, sin35.5°], aim center. Very slow push (E_PUSH ≈ 0.05, v2 convention). Hard cut or 12-frame crossfade to the RViz reshot opened at the same elapsed-time point.
**QC:** map-frame→blocks-world axis alignment is [inf] — verify with ONE frame: quadrant sign-match of all 4 drone endpoints between the UE test frame and the RViz first frame (reference `shot_mapview2_final.png`); rotate UE azimuth by any residual yaw. Footprint screen-extent ratio between the two frames within 20%.

---

## 3) CUT STRUCTURE — target ~97s (spec 90–120s)

| # | Segment | Dur | Source |
|---|---|---|---|
| 0 | Title card (0x14161e, "recorded autonomy replay" honesty line) | 2.5s | compose |
| 1 | SHOT 1 starburst fan-out | 10s | W_open |
| 2 | SHOT 2 through-the-line | 7s | W_open tail |
| 3 | SHOT 3 low track + crosser | 10s | W_main |
| 4 | SHOT 4 depth-stack dolly | 10s | W_main |
| 5 | SHOT 5 ribbon arc | 12s | run end state |
| 6 | SHOT 6 map-matched hold | 5s | last world frames |
| 7 | Transition card | 2.2s | compose |
| 8 | Merged-map RViz at 2x (amber "2x speed" chip): 30s growth slice from first mesh + 5s final-map hold | 35s | `bag_<label>` reshot |
| 9 | End title | 3s | compose |

**Total ≈ 96.7s.** World footage need ≈ 54s of distinct windows — `window_select.py` confirms feasibility **before** any render. Micro-captions only on SHOT 1 ("t+0s: four goals claimed over the shared radio") and SHOT 5 ("flown paths — one 5-minute flight") [inf — matches small-label taste, confirm at review]. Per-drone legend colors: GHOST 0x3f8cff / DELTA 0x3fff72 / BUCKSHEE 0xff4cf2 / THUNDERSTRIKE 0xff941e. Shot order tells the story: fan-out → energy → fleet-at-work → coverage → map proof.

---

## 4) PIPELINE INTEGRATION (reuse vs new, then ordered steps)

**REUSED unchanged** (all paths verified): `/home/lucas/hercules-sim/run_fleet_radio.sh`, `/home/lucas/hercules-sim/investigation/run_exp.sh`, `/home/lucas/hercules-sim/fidelity_scorecard.sh`, `mapview/relay_drone.py`, `mapview/augment_replay.py`, `mapview/reshot_from_bag.sh` (usage `<tag> <rate> [record] [shot_secs...]`, `BAG=` override), `mapview/fleet_map.rviz`, `mapview/scan_bag_mesh.py`, `temple/export_quad_52.py`, `temple/transform_keys_temple.py` (env `TEMPLE_ANCHOR/YAW_DEG/SCALE/ZLIFT`), `temple/fix_keys_foliage.py` + `offender_aabbs.json`, `temple/design_camera_v2.py`, `temple/build_fleet_seq6_clean.py`, `temple/render_fleet_temple_clean.sh`, `temple/bake_ribbons.py`, `temple/scout_map_56.py`, `probe/{corridor_probe.py,fine_probe.py,slab_analysis.py,corridor.binvox,fine.binvox}`, `/home/lucas/UE5/educationalVideos/VIDEO_SUCCESS_PLAYBOOK.md` (binding rules: motivated moving camera, MRQ+TSR 30fps, motion blur, per-beat renders, honest labels).

**EXTENDED:** `temple/design_camera_v3.py` (axis-search parameterization for SHOT 4), `temple/compose_final_temple_v5.sh` → **`compose_final_temple_v6.sh`** (same card/label/chip conventions, new 10-segment timeline).

**NEW files:** `mapview/capture_fleet_bagonly.sh`; `investigation/score_run.py` (+`--timeline`); `temple/window_select.py`; `temple/shot_qc.py`; `temple/design_camera_opening.py` (S1+S2); `temple/design_camera_track.py` (S3); `temple/design_camera_arc.py` (S5+S6).

**Ordered steps:**
1. **Build** `capture_fleet_bagonly.sh` + `score_run.py`; smoke-validate on one throwaway 150s run (stereo ≥16Hz, ≥100 mesh msgs/drone). (~1h)
2. **Shoot** §1.3 protocol → winning label + bag + stall timeline + arena bbox. (~1.5h)
3. **Temple fit:** `SCALE = clamp(min(1.0, 4000/X_extent_cm, 2800/Y_extent_cm), 0.6, 1.0)` (target ≤40×28m at anchor `TEMPLE_ANCHOR=5000,-1900,802`, `YAW_DEG=60`, ZLIFT 40 — rigid uniform scale keeps relative motion honest). Export: `export_quad_52.py` → `transform_keys_temple.py`. Re-probe occupancy at that scale (`scout_map_56.py`, probe binvox, `slab_analysis.py` for SHOT 1's vertical column) → re-run `fix_keys_foliage.py` dodge pass. (~0.5d)
4. **Windows + harness:** `window_select.py` blesses W_open/W_main; build `shot_qc.py`. (~1d)
5. **Design shots 1–6**, each gated by shot_qc then 5-frame sparse render. (~2.5d)
6. **Regenerate ribbons** (`bake_ribbons.py` on new keys); build sequence via `build_fleet_seq6_clean.py` with per-shot camera keys + ribbon visibility track.
7. **Render per beat** (playbook Pillar F): one `render_fleet_temple_clean.sh` invocation per shot (~4 min warm each; ~54s world footage total).
8. **Map segment:** `BAG=$M/bag_<label> ./reshot_from_bag.sh <label>_2x 2 1 20 60 120` (augment_replay at `--rate 2`, screenshots for QC); SHOT 6 continuity check against its first frame.
9. **Compose** `compose_final_temple_v6.sh` → final + `_mobile` variant.
10. **Review package:** shot_qc reports + score table + stall timeline alongside the mp4.

Total ≈ 5 working days, flight day first.

---

## 5) ACCEPTANCE CRITERIA — FINAL VIDEO (all must pass; every item machine-checkable from keys/QC output before human review)

1. **Same-run mandate:** every world shot's keys and the map segment trace to ONE label; `bag_<label>` has ≥300 `/dN/mesh` msgs per drone; SHOT 6 → map continuity QC passed (4/4 endpoint quadrant match, extent ratio ≤20%).
2. **Flight quality on camera:** winning run passed G1–G4 + floors (P_min ≥ 30m, F_min ≥ 0.15, T_stall ≤ 90s); no framed drone appears parked >10s in shots 1–4 (shot windows avoid all `stall_timeline` streaks >20s).
3. **Opening reads "four drones fanning out":** SHOT 1 holds all 4 drones in frame 100% of frames at ≥60px; **each drone's cumulative screen displacement ≥ 250px** within the shot; min pairwise screen separation ≥ **288px (0.15 frame-widths) by t=6s**; screen-velocity bearing spread ≥ **180°**; SHOT 2 has ≥2 drones exiting opposite frame edges.
4. **Fleet presence:** SHOT 1 and SHOT 4: **100%** of frames contain all 4 drones; across all fleet shots (1, 2, 4, 5) combined: **≥80%** of frames contain ≥3 drones; SHOT 5 ends with all 4 final positions + all 4 ribbon paths in frame.
5. **Coverage is visible:** SHOT 5 ribbons span ≥90% of the trajectory bbox on screen; map segment shows growth from near-empty to final state with all 4 drone colors and mesh from all 4 drones (≥300 mesh msgs each), final-map hold ≥4s.
6. **Camera craft:** every shot has camera or subject motion 100% of the time (playbook); no cut shorter than 5s; yaw rate ≤12°/s on wides; zero occlusion-ray hits and zero near-clip violations in delivered frames.
7. **Format + honesty:** 1920×1080 30fps MRQ+TSR with motion blur, total 90–120s (target 97s); "recorded autonomy replay" label present; amber "2x speed" chip on the map segment and any retimed world segment; `_mobile` variant emitted.
8. **Evidence on disk:** `shot_qc` per-shot reports, `window_select` report, `score_run.py` table for all attempts, and `stall_timeline_<label>.json` accompany the delivery for review.