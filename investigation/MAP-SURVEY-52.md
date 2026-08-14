All numbers verified against the archived runs and live rig state. Writing the survey.

---

# HERCULES 4-Drone Fleet Sim — UE 5.2.1 MAP SURVEY

**Question:** replace the featureless gray AirSim `Blocks` map for the HERCULES 4-drone exploration rig.
**Answer:** do not replace it. **Texturized Blocks wins and stays the default.** Of 12 UE5.2-openable projects and ~30 maps on this host, exactly one candidate reached the full stack (`/Game/Maps/Overview`) and it lost on the metric the fleet exists to move — unique explored area — by 31–55%.

Survey date 2026-08-14. Host verified quiet at write time: `load1 1.09`, no foreign `UnrealEditor`, root disk 9.2 GB free.
Rig: `/home/lucas/hercules-sim` (→ `/home/lucas/UE5/hercules-sim-big`), runner `run_fleet_radio.sh`, guarded wrapper `investigation/run_exp.sh`.

---

## 0. Headline

The three selection criteria are **anti-correlated across this entire host**, and the split is clean:

- Every **textured** map is an architectural pack — street, village, subway, interior. Dense, narrow, high draw-call. `Stylized_Street`/`JapanFest` 174 textures / 2119 actors / 16 m open. `Asian_Village` 115 textures / 3004 actors / 18 m open. `SubwayTrain` 141 textures / 341 actors / 16 m open.
- Every **open + cheap** map is flat-shaded or solid-colour. `LowPolyMoonPack` 4 textures / 10 actors / 54 m open. `Ghosthawk` showroom (props literally named `MI_MasterSolidColor_*`) / 293 actors / 60 m open. `crazyFlieSim` bare landscape / 60 m open / repeating checkerboard.

There is no textured-and-open map to port. **JapanFest was not a near miss in a field of alternatives — it was the best textured option, and the whole textured family fails openness the same way.**

The winning move was therefore never a new map. It is **an open map plus the existing zero-cost texturize hack**, and the rig already is that: Blocks (open, 196 draw calls) with the R3 procedural-noise material applied. Verified live in the rig right now:

| file | bytes | Noise nodes |
|---|---|---|
| `Content/Geometry/Meshes/CubeMaterial.uasset` | 10,689 | 1 |
| `Content/Geometry/Meshes/CubeMaterial.uasset.bak` (pristine) | 8,350 | 0 |
| `Content/Flying/Meshes/BaseMaterial.uasset` | 11,946 | 1 |
| `Content/Flying/Meshes/BaseMaterial.uasset.bak` (pristine) | 9,607 | 0 |

The hack was previously *asserted* zero-cost. It is now **measured** zero-cost: interleaved A/B, 3 reps per arm, isolated single-RPC 640×480 stereo — **TEX 14.24 ms vs PLAIN 14.27 ms = 0.9979 ratio (−0.2%)** against a 1.1% within-arm spread, `load1` 3.63–4.66 on every row.

---

## 1. Complete UE 5.2.1 map inventory

### Reading the cost column

Three independent probes were built, each with its own Blocks control. **Absolute ms are not comparable across probes — only the ×Blocks ratio within a probe is.** Probes are tagged **[A]** (Go2Sim sweep, warmed, isolated), **[B]** (CrazyflieAirSim sweep, quiet bracketed block, foreign-UE counter on every row), **[C]** (stereo-only single-RPC A/B rig).

Control values: **[A]** Blocks 12.91 / 13.18 ms (agree to 2.1%). **[B]** Blocks 12.99 / 12.60 / 12.49 ms, mean 12.69 (spread 3.9%). **[C]** texturized Blocks 14.24 ms (n=5).

Each probe was calibrated against the known JapanFest reject before any candidate was judged — see §6.

### 1a. Maps that were actually measured

| Project | Map | Actors / colliding / prim comps / draw-call est | Open volume | Texture score | Cost ×Blocks | Verdict |
|---|---|---|---|---|---|---|
| **rig Blocks** | **`/Game/FlyingCPP/Maps/FlyingExampleMap` — TEXTURIZED** | **181 / 171 / 196 / 196** | **630/630 arena cells; 169×99 m clear box at the spawn line; 38–54 m clear square** | **ST 126–1793 (med 1128); ground feat 13–1031 (med 328)** [C] | **1.00 (reference)** | ✅ **WINNER — DEFAULT** |
| rig Blocks | same, **PLAIN** (`.bak` materials) | 181 / 171 / 196 / 196 | identical | ST 23–215 (med 84); ground 0–39 (**med 11**, reproduces the documented value exactly) [C] | 1.0021 [C] | ⚠️ superseded — feature-starved |
| S2R_Go2Sim *(byte-identical in S2R_CrazyflieAirSim)* | `/Game/Maps/Overview` | 1167 / 1147 / 1071 SMC / 1430 mat slots | 625/630 voxel cells at 1–3 m; clear box **14.5×28.0 m** → **24.0×29.0 m** at a 2.5–4.5 m cruise band; 77.7% → 84.5% of ±10 m arena | ground-half 51–230 (**med 131**) vs Blocks med 3, JapanFest 476–747 [B]; usable disparity hits 82.2% / 89.9%; median disparity 4.59–7.54 px | **0.91** [B, clean, load1 3.48] — *1.51 [A] was load-contaminated, see §6* | ❌ **FULL-STACK REJECT (coverage)** |
| S2R_Go2Sim | `/Game/Maps/JapanFest_Street` *(prior experiment)* | 2119 / 2104 / 2517 / **2939** | **357/630** (documented 301/630); **16 m** open square; ~6 m alleys; GT yaw **816 °/s** | ST 923–1369; ground 476–747 (med 552) — the only success axis | **1.75** [C] · **2.08** [A warmed] · 1.39 (original cold probe) | ❌ REJECTED (open + cost) |
| S2R_Go2Sim / S2R_CrazyflieAirSim | `/Game/LowPolyMoonPack/Maps/Demo` | 10 / 5–10 / 17 SMC / 23 mat slots + **426,400 HISM instances** + 1 Landscape | **580/580 arena; 54×54 m clear square**; 90.0% of ±10 m at 1–3 m → 97.1% at 2–3 m | ground 21–99 (med 37) [A]; ground-half 0–75 (med 26) [B] — **flat-shaded, no surface texture** | **1.01** [A] · **1.00** [B] | ❌ SCREENED-OUT (texture) — best remaining *shell* |
| S2R_Go2Sim / S2R_CrazyflieAirSim | `/Game/Maps/Demonstration` | **11,870 / 11,107 / 8731 SMC / 11,109 mat slots**; 601 dynamic lights (349 Point + 231 Spot), 1687 DecalActors | 22×22 m square; 473/580; largest box 54.0×**6.0 m** min side | ST 362–1317 (med 901); ground 63–672 (**med 425**); usable disparity 39–97% — JapanFest-class | **3.07** [A] · **3.26** [B] | ❌ SCREENED-OUT (cost — 1.5× worse than the already-rejected JapanFest) |
| S2R_CrazyflieAirSim | `/Game/crazyFlieSim` | 73 World-Partition external actors (64 `LandscapeStreamingProxy`) | **100% of ±10 m arena; 60×60 m clear; zero blocked cells above ground** | ground-half 11–109 (med 31); **median disparity ≈0**; **repeating checkerboard** | **1.20** [B] | ❌ SCREENED-OUT (texture + aliasing hazard) |
| S2R_CrazyflieAirSim | `/Game/Untitled` | 73 WP external actors, same landscape (VolumetricCloud vs SkyAtmosphere) | identical to `crazyFlieSim` | ground-half 11–125 (**med 44**); median disparity ≈0; same checkerboard | **1.20** [B] | ❌ SCREENED-OUT (texture) — marginally the better bare shell |
| S2R_CrazyflieAirSim | `/Game/WIS/Maps/Showroom` | 80 / 60 / 150 / 8 meshes / 7 mats | sealed 30.4×30.3×11.7 m room; 12 m clear square; 85.2% of ±10 m *(the 18.5×60 m "clear box" is a probe artifact spanning outside the walls)* | ST 70–284 (**med 113 — a third of Blocks' 331**); ground-half med **0**; mean luminance 18–63 vs Blocks 197–213 | **0.96** [B] | ❌ SCREENED-OUT (texture + open) |
| S2R_Go2Sim / S2R_CrazyflieAirSim | `/Game/Maps/TechArt` | 219 / 121–206 / 155 SMC / 205 mat slots; **79 Emitters, 6 SceneCapture2D** | **void at spawn** — drone free-falls 44.16–62.97 m; 24/580 cells | **ST 0, mean luminance 0 at every pose** — 0 DirectionalLight, 0 SkyLight; renders pure black | 0.87 [B] *(meaningless — rendering nothing)* | ❌ SCREENED-OUT |
| S2R_Go2Sim | `/Game/Home_Interior/Maps/Home_Interior` | 267 / 266 / 245 SMC / 534 mat slots | **void at spawn** — falls 69.62 m (house sits at UE −1180,1710); 24/580; interior rooms a few m across | invalid (depth 14,704–14,728 m, FAST 0 at every pose) | n/m | ❌ SCREENED-OUT |
| S2R_Go2Sim | `/Game/Home_Interior/Maps/Home_Interior_Overview` | 102 / 102 / 100 SMC / 252 mat slots — genuinely cheap | **PASSES: 53×53 m clear square; 580/580 arena; 4-drone line fits** | **FATAL: usable disparity >2 px = 0% at ALL 8 poses.** Ground is an **invisible collision plane** — 2915 grounded voxel cells but depth 13,280–13,672 m, 0–13% of pixels within 50 m | n/m | ❌ SCREENED-OUT (texture) — **the trap: a naive screen promotes it** |
| S2R_Go2Sim / S2R_CrazyflieAirSim | `/Game/LowPolyMoonPack/Maps/Overview` | 21 / 17 / 16 SMC | **void at spawn** — falls 47.84–62.48 m; **12/580 = 2%**, worst measured; 42×12 m prop strip | **invalid** — ST 2000 / ground 938–1010 is the saturated-sky artifact: **LK matches = 0, usable disparity 0%, depth 14,696 m** | 0.79–1.0 [A/B] *(meaningless)* | ❌ SCREENED-OUT |

### 1b. Screened out on static structure (no UE launch spent)

| Project | Map | Actors / draw-call est | Open volume | Texture | Verdict |
|---|---|---|---|---|---|
| `/home/lucas/UE5/HERCULES/.../Blocks` | `/Game/CacaoField/Maps/CacaoField` | 579 / 579 colliding / **1779** ; 513 tree colliders | **3.1% free at 1–3 m; clear box 65×1 m; spawn cell blocked; 0/609 arena.** Tree NN spacing med 2.80 m (p10 2.05 m) | not measured (would likely pass) | ❌ **OPEN** — and *robustly*: shrinking trees to bare poles (8.23→3.0→1.0→0.5 m) gives 3.9%→30.1%→77.5%→87.2% free, but the clear box **never exceeds 8 m wide at any trunk model** because orchard row spacing hard-caps it. JapanFest's corridor failure with 8 m rows instead of 6 m alleys. *(Note: the copy at `/home/lucas/UE5/HERCULES/...` also references `/Game/UltimateFarming` 12× and that content root does not exist on this host — it would load with every mesh missing.)* |
| grafted pack | `/Game/Asian_Village/maps/Asian_Village_Demo` | 3003–3004 / 3244 prim / **8085** | **PASSES** — 107×56 m clear box, 630/630 (18 m square by the stricter probe) | 115 textures, 68/127 mats sample | ❌ **COST** — 2.75× JapanFest's draw calls, 41× Blocks |
| grafted pack | `/Game/Asian_Village/maps/Overview_Map` | 232 / 241 prim / 734 | clear box **2×123 m** corridor; 460/630 | would pass | ❌ OPEN |
| grafted pack | `/Game/Stylized_Street/Maps/L_Showcase` | **2119 / 2517 / 2939 — identical to JapanFest on every metric** | identical | identical | ❌ **DUPLICATE OF THE REJECTED MAP.** JapanFest_Street was derived from this pack. Spend no run on it. |
| grafted pack | `/Game/Stylized_Street/Maps/L_Overview` | 115 / 178 prim / 203 (cheap) | 2.3% free; spawn blocked; **42/630** | would pass | ❌ OPEN |
| S2R_CrazyflieAirSim | `/Game/SubwayTrain/Maps/Demonstration` | 341 / 638 prim / 97 meshes / 42 mats | **16 m** square — same as JapanFest; enclosed station, 1715/3600 ground | 141 textures, 72/80 mats sample | ❌ OPEN (rotation-without-parallax in a smaller box) |
| S2R_RainDemo | `/Game/DV_UH_H60_Ghosthawk/Maps/Demonstration` | 293 / **1404 prim** / 12 meshes / 9 mats | **60 m square (grid limit); 3600/3600, 0 blocked** | **marginal** — 76 textures are all on the *helicopter*; env props are `MI_MasterSolidColor_Dark/Orange`; terrain mat has 1 Texture2D + 2 layers | ❌ TEXTURE — 1404 prim comps vs the moon's 155 for no visual gain |
| S2R_RainDemo | `/Game/Maps/RainDemoLevel` | 19 / 34 prim / 2 meshes / 3 mats | 60 m, 3600/3600 | essentially none — a bare plane | ❌ TEXTURE (worse than Blocks' 181 cubes) |
| S2R_Go2Sim | `/Game/StarterContent/Maps/StarterMap` | 252 / 334 prim / 335 | **8 m** square; whole level is 26×43 m — cannot contain a 25×25 m volume *plus* the 630-cell arena | 106 textures, 56/68 mats sample — would pass | ❌ OPEN / SIZE |
| S2R_Go2Sim | `/Game/StarterContent/Maps/Minimal_Default` | 15 / 21 prim / 22 (cheapest on host) | trivially open (empty) | **7 colliders vs Blocks' 171** — less geometry than the baseline | ❌ TEXTURE |
| S2R_SimulationMaker | `/Game/Sim2Real/Maps/AirSim_Test` | 6 / 17 prim / 46; **2 colliders** | 626/630 — open because empty | one untextured 50×50 m plane, zero vertical structure → predicted **at or below plain Blocks' 0–39** | ❌ TEXTURE |
| S2R_SimulationMaker | `/Game/Sim2Real/Maps/SmokeTest_Matrice` | 4 / 7 prim / 40; **1 collider (the drone)** | no ground plane at all | none | ❌ TEXTURE (total) |

### 1c. Structurally excluded — do not spend time on these

| Item | Why |
|---|---|
| `SAM/Maps/SAM_Main` | **4 actors, ZERO collision boxes** (no ground, drones fall through) **and** requires the `UnrealSplat` plugin, declared `Type=Editor` with an **empty `Binaries/Linux`** — cannot load in `-game`. It is a 3DGS cinematic, not a physics environment. |
| `S2R_SimulationMaker/Sim2Real/Maps/SimulationStarter` | 254 actors but only 8 resolved colliders — the real content is a `Cesium3DTileset` **streamed from CesiumIonSaaS at runtime**. Cost is unbounded and network/LOD-dependent → **unreproducible**, fatal for a benchmark whose premise is that the map is the only variable. Streaming hitches land directly on the cuVSLAM stutter metric. |
| `Niricson_Demo/Glenmore_Demo` + `AirSim_Test`, `SmokeTest_Matrice`, `MaterialTest` | CesiumForUnreal dependency; the latter three are ~14–15 KB stubs. |
| `S2R_StrayosBlastDemo/Maps/BlastTest`, `BlastSolo` | 48 exports / 14,922 B — test stubs. |
| **Any Cesium / UnrealSplat map** | Adding a plugin to Blocks **breaks the "binary + plugin + GameMode bit-identical, map is the only variable" guarantee** the entire comparison rests on. Non-negotiable. |
| `kb3d_missiontominerva` (3.8 GB, 976 assets, **is** 5.2-compatible) | Ships ~230 per-prop sublevels of 7.7–8.9 KB each. **No assembled environment exists.** Usable only by authoring a level — a different project, not a port. Worth remembering as the one build-to-spec option. |
| `RenderBench`, `LockstepEnv`, `USB_restore/HERCULES` `FlyingExampleMap` | Byte-identical (4,988,416 B) or near-identical (4,995,984 B) copies of Blocks. Not alternatives. |
| Vendor overview/thumbnail stubs | `SubwayTrain/Overview` (261 exports), `FirstPerson/FirstPersonMap` (28), `StarterContent/Advanced_Lighting`, `Ghosthawk/Maps/Overview`, `Home_Interior_Overview`, `LowPolyMoonPack/Overview`, `crazyFlieSim.umap` (16 exports). Asset catalogues, not environments. |
| **UE 5.6 / 5.3 / 4.27 projects** | MountainTops, RuralCabins, MedievalVillageMegascans, DerelictCorridorMegascans, NiricsonRecon56, HGS, KungfuRender, GetupRender, ClassroomFX (5.6); Windows MedievalVillageMegascansS (5.3); crazyflie-airsim-bridge/airsim-fresh (4.27). **Package version is forward-only — 5.2.1 cannot open them.** MedievalVillage and MountainTops would have been strong open+textured candidates; this is the single biggest missed opportunity on the host. |
| **Epic VaultCache** — 38 GB, 8 items, `/home/lucas/Data/Games/epic-games-store/drive_c/ProgramData/Epic/EpicGamesLauncher/VaultCache` | Manifest engine versions decoded: AncientT **5.5.0**, BambooTr **5.8.0**, Derelict **5.7.0**, FREEMoun **5.8.0**, MedievalGame **5.3.1**, ModularR **5.8.0**, Stylized **5.7.0** — all newer than 5.2, unopenable. Only `Futurist*` (**5.1.0**, 3.9 GB) would load forward, but it is chunked vault data requiring the Epic launcher under Wine, already a documented dead end on this host. **Treat the vault as unavailable, not as a reserve.** |

---

## 2. Full-stack results — finalists vs the Blocks baseline

Six 150 s 4-drone runs through `investigation/run_exp.sh` (PREFLIGHT / WIPE / VALIDATE guards). Arms interleaved `ov1 → tb1 → ov2 → tb2 → ov3 → ov4`. All runs single-tenant, verified by `ps`/`pgrep` before each. `ov1`/`ov2` archived `_INVALID` (see spawn brittleness below) and are excluded from comparison.

### 2a. Run conditions

| Run | Map | Settings | load1 pre | load1 in-run min/med/max | Launch attempts |
|---|---|---|---|---|---|
| tb1 | texturized Blocks | `settings-fleet-4drone.json` | 2.64 | 2.7 / **9.7** / 13.5 | **1 — clean** |
| tb2 | texturized Blocks | `settings-fleet-4drone.json` | 2.13 | 2.8 / **11.2** / 16.7 | **1 — clean** |
| ov3 | `/Game/Maps/Overview` | `settings-overviewB-4drone.json` | 7.11 | 4.5 / **11.1** / 13.2 | 2 (attempt 1 segfaulted) |
| ov4 | `/Game/Maps/Overview` | `settings-overviewB-4drone.json` | 5.66 | 7.8 / **8.7** / 10.0 | 2 (attempt 1 segfaulted) |

**Launch robustness differs by map:** Blocks validated on attempt 1 in **2 of 2** runs. Overview needed retries in **3 of 4** runs — **5 of 9 Overview launches segfaulted or hung** (`Signal 11` in `ue_radio4.log`, rsnode reporting `170s | stereo 0.00 Hz, meta=0`). Budget ~2.5× wall-clock per Overview run.

### 2b. The decisive metric — exploration coverage (630-cell arena, identical both maps)

| | tb1 | tb2 | **Blocks** | ov3 | ov4 | **Overview** |
|---|---|---|---|---|---|---|
| `fleet_unique_m2` | **575** | **620** | **575–620** | 276 | 428 | **276–428** |
| `fleet_effort_m2` | 935 | 1101 | 935–1101 | 388 | 537 | 388–537 |
| `fleet_arena_cells` | 630 | 630 | 630 | 630 | 630 | 630 |
| coverage vs Blocks | — | — | 1.00 | **−52%** | **−31%** | **−31% to −55%** |
| per-drone `coverage_m2` (ghost/delta/buckshee/thunderstrike) | 149 / 359 / 239 / 188 | 178 / 203 / 393 / 327 | | 58 / 129 / 58 / 143 | 58 / 126 / 58 / 295 | |
| **odom excursion, m** (distance flown from spawn) | 11.1 / 14.9 / 14.6 / 9.6 | 10.5 / 15.7 / 14.5 / 45.1 | **9.6–15.7 — every drone flew** | **0.8** / 10.6 / **0.7** / 23.4 | **0.6** / 11.1 / **0.8** / 10.9 | **two of four drones never left the spawn area, both runs** |
| PLAN OK / Path complete | 301 / **47** (15.6%) | 264 / **60** (22.7%) | **15.6–22.7%** | 316 / **7** (2.2%) | 330 / **19** (5.8%) | **2.2–5.8%** |
| PATH BLOCKED / Replan timeout | 1 / 6 | 0 / 7 | ~0 | 29 / 9 | 12 / 10 | |
| NO PATH | 1 | 0 | 0–1 | **213** (thunderstrike 204) | 1 | |

**Mechanism of the stall** — planner message histograms, ov4: `ghost` 160 EXEC, 12 PLAN OK, 7 PATH BLOCKED, 4 Replan timeout, **0 Path complete**. `buckshee` 162 EXEC, 9 PLAN OK, 4 PATH BLOCKED, 4 Replan timeout, **0 Path complete**. Compare tb2 `ghost`: 147 EXEC, 130 PLAN OK, **10 Path complete**.

FIS was **not** the bottleneck (ov4 ghost 697 clusters / 25,255 cells; buckshee 1053 / 31,029). **Overview's free space is fragmented at the planner's inflation radius even though it is not fragmented at 1 m voxel resolution.** Paths get issued, then invalidated by clutter, and the drone burns the run re-executing a stale path. This is the same criterion JapanFest failed grossly (6 m alleys). Overview fails it *subtly* — and the offline voxel probe, which scored it **625/630 cells flyable**, did not predict it.

### 2c. Texture / VIO — Overview genuinely passes

| | tb1 | tb2 | ov3 | ov4 |
|---|---|---|---|---|
| delivered stereo Hz (g/d/b/t) | 16.2 / 20.3 / 20.2 / 19.6 | 15.7 / 15.1 / 17.0 / 29.6 | 29.8 / 12.4 / 17.3 / 12.6 | 17.5 / 17.0 / 27.1 / 16.4 |
| cuVSLAM stutter/min | 4.5 / 1.7 / 4.2 / 1.7 | 14.0 / 11.4 / 14.4 / 11.9 | 10.1 / 18.3 / 13.8 / 14.7 | **2.1 / 1.8 / 1.6 / 2.9** |
| max belief-vs-truth \|err\|, m | 1.5 / 1.6 / 1.3 / 1.3 | 1.5 / 2.3 / 1.9 / **470.4** | — / — / — / 39.1 | **1.0 / 1.8 / 1.6 / 1.3** |
| `vio_divergence_m` scorecard FAILs | none | thunderstrike **454.6** | thunderstrike | **none** |
| GT yaw p99 / max, °/s | 41.5 / 62.4 | 41.6 / 65.0 | 28.5 / 59.8 | **31.2 / 52.0** |

Both maps beat the quoted plain-Blocks baseline of 2.1–2.5 m divergence. **Feature starvation is cured in both.** Neither shows any trace of the JapanFest 816 °/s rotation pathology. Overview costs **nothing** in delivered stereo rate despite 6.4× the actor count — the isolated 0.91× figure survives 4-drone contention, unlike JapanFest whose isolated 1.39× amplified to ~2.2×.

### 2d. Two corrections recorded, both against my own earlier readings

1. **`ov1`'s apparent collapse was not the map.** It delivered 9.6–11.4 Hz and 25–33 stutters/min, which looked like a JapanFest-style throughput failure. Its metrics come from `run_exp.sh` **attempt 3 of 3**, run back-to-back with orphan contention at load1 median 13.9. Clean Overview runs deliver 12.4–29.8 Hz.
2. **The control arm is not noise-free either.** In tb2, `thunderstrike` blew up to max \|err\| 470.4 m (odom_divergence 45.1 m, scorecard FAIL) on *texturized Blocks* while its three peers held 1.5–2.3 m, and tb2's stutter rate was 11.4–14.4/min against tb1's 1.7–4.5/min at similar load. **One catastrophic VIO divergence per ~8 drone-runs is the background rate on this rig regardless of map.** A single-drone divergence outlier is not evidence about a map.

### 2e. Spawn brittleness — a reproducible hard failure invisible to every offline probe

`settings-overview-4drone.json` (X=15, Y=−15/−12/−9/−6) was chosen from a 113-point runtime probe as the best open+textured band. **All 4 flying attempts (ov1 ×3, ov2 ×1) failed `run_exp.sh` validation** because `delta` at (15, −12) logged `PLAN_OK=0`.

Root cause traced: delta's FIS reported **0 clusters / 0 cells for the entire run** while its three peers reported 900–2134 from an identical 401×581×61 team box (verified identical ESDF AABB requests, `[-10,-16,0]+[20,29,3]`, in both Blocks and Overview). Its nvblox map stayed empty and the planner sat at `INIT: odom=True front=False armed=True` forever.

Moving the line 6 m north to X=21 (`settings-overviewB-4drone.json`) fixed it — ov3 and ov4 both validated with all four `PLAN_OK>0`. **The failure is spawn-local, not map-wide, and neither the voxel probe nor the feature probe predicted the bad cell.** Cost: 4 wasted 150 s runs.

### 2f. Finalist not run

`/Game/LowPolyMoonPack/Maps/Demo` was deprioritised, not flown. On the decisive TEXTURE axis it is last of the three (ground-feature median 26–37 vs Overview 131 vs texturized Blocks 328) and by its own screening does not cure feature starvation — it merely matches the incumbent while adding a 426k-instance foliage field. Its only remaining advantage, a 54 m clear square vs Blocks' 38 m, is worthless: **Blocks already delivers 630/630 arena cells and the fleet is nowhere near geofence-limited.** Revisit only if the R3 texturize hack is ported onto the moon terrain — and even then it competes for the slot Blocks already occupies. **Caveat if it is ever revisited: the 426,400 foliage rocks have collision disabled** (3600/3600 ground hits, 0 blocked cells). They texture the view without creating obstacles — arguably ideal here, but the map cannot double as an obstacle-avoidance test.

---

## 3. THE RECOMMENDATION

> ### The rig's default map is **texturized Blocks**: `/Game/FlyingCPP/Maps/FlyingExampleMap` with the R3 procedural-noise materials, spawned from `settings-fleet-4drone.json`.
>
> **That is the current state of the rig. No change is required. `HERC_UE_MAP` stays unset.**

It is the only arm that puts all four drones in motion for a whole run, and it produces **1.3–2.1× the unique coverage of the best Overview run at equal or lower cost**. It sits inside the quoted Blocks baseline band on every axis and improves two of them:

| axis | quoted plain-Blocks baseline | texturized Blocks measured | change |
|---|---|---|---|
| delivered stereo | 17.4–18.5 Hz | 15.1–29.6 Hz | parity |
| cuVSLAM stutter | 0–4 /min | 1.7–4.5 (tb1) / 11.4–14.4 (tb2) | parity, run-dependent |
| max divergence | 2.1–2.5 m | **1.3–1.6 m (tb1)** | **improved** |
| unique coverage | 446–607 / 630 | 575 / **620 = 98.4%** | **new high-water mark** |
| ground features | 0–39 (med 11) | **13–1031 (med 328)** | **30× — starvation cured** |
| render cost | 1.00 | **0.998** | free |

### Exact command

```bash
cd /home/lucas/hercules-sim

# Guarded (PREFLIGHT / WIPE / VALIDATE / auto-retry on UE segfault) — USE THIS
./investigation/run_exp.sh <label> 150

# Or the runner directly
HERC_CAPTURE=0 HERC_DEPTH_HZ=10.0 ./run_fleet_radio.sh 4 150
```

**Env: nothing to set.** `HERC_CAPTURE=0` and `HERC_DEPTH_HZ=10.0` are already the script defaults (`run_fleet_radio.sh:48,51`). `HERC_UE_MAP` empty ⇒ Blocks' `GameDefaultMap`, whose `WorldSettings` already names `AirSimGameMode`, so no `?game=` is needed. `HERC_SETTINGS` defaults to `$BASE/settings-fleet-4drone.json` and the script **exports it itself** (`run_fleet_radio.sh:68`) so `fidelity_scorecard.sh` autodetects the right spawn line. Stereo stays 640×480 for field parity.

**One caveat:** if you run `fidelity_scorecard.sh` **standalone** in a fresh shell, export `HERC_SETTINGS` yourself — it falls back to `sorted(glob('settings-fleet-*.json'))` (`fidelity_scorecard.sh:295-296`) and will silently score against whichever file sorts first.

### Verifying / restoring the texturize state

```bash
# Verify (both should print bytes 10689 / 11946 and Noise-hits=1)
B=/home/lucas/hercules-sim/HERCULES/Unreal/Environments/Blocks
for f in $B/Content/Geometry/Meshes/CubeMaterial.uasset \
         $B/Content/Flying/Meshes/BaseMaterial.uasset; do
  printf "%s %s Noise=%s\n" "$f" "$(stat -c%s $f)" "$(grep -ac Noise $f)"
done

# Re-apply after an accidental revert (idempotent, ~2 min, needs UE off)
./investigation/vio/run_texturize.sh

# Revert to plain Blocks for an A/B (the .bak files are pristine, 0 Noise nodes)
for f in $B/Content/Geometry/Meshes/CubeMaterial.uasset \
         $B/Content/Flying/Meshes/BaseMaterial.uasset \
         $B/Content/Flying/Meshes/GrayMaterial.uasset; do cp -a "$f.bak" "$f"; done
```

### Running Overview as an optional second environment

Overview is **not** the exploration baseline, but it is a legitimate second environment for VIO/feature work — it is textured, it is cheaper than Blocks in isolation, and ov4's 1.0–1.8 m divergence at 1.6–2.9 stutters/min is the best VIO quality measured in this survey.

```bash
cd /home/lucas/hercules-sim
export HERC_SETTINGS=/home/lucas/hercules-sim/settings-overviewB-4drone.json   # X=21, NOT X=15
export HERC_UE_MAP=/Game/Maps/Overview
./investigation/run_exp.sh ovN 150
```

Requires the symlink graft (already in place, verified today):
`Content/Maps/Overview.umap` and `Content/Maps/Overview_BuiltData.uasset` → `S2R_Go2Sim/Content/Maps/`, plus the `Meshes`, `Materials`, `Textures`, `Blueprints`, `Fx`, `UI`, `LevelPrototyping` roots.
Never use `settings-overview-4drone.json` (X=15) — `delta` at (15,−12) is the dead nvblox cell, 4/4 failures.

### Rig state as left

- Texturize **applied and verified**; `.bak` originals intact (8,350 / 9,607 B, zero Noise nodes).
- `Blocks/Content/` graft in place: `Asian_Village`, `Blueprints`, `Fx`, `Go2`, `Home_Interior`, `LevelPrototyping`, `LowPolyMoonPack`, `Materials`, `Meshes`, `StarterContent`, `Stylized_Street`, `Textures`, `UI` + `Maps/{Overview,Demonstration,TechArt,JapanFest_Street}.umap`. **Zero copies, zero rebuild, disk unchanged at 9.2 GB free.** `BLOCKS2` measured 12.91 ms *with* the full graft present, matching the pre-graft control — **the grafted content imposes no runtime cost.**
- ⚠️ **Name-collision hazard in the shared graft:** `S2R_Go2Sim` and `S2R_CrazyflieAirSim` share root names (`Maps`, `Meshes`, `Materials`, `Blueprints`, `Fx`, `LowPolyMoonPack`, `Collections`, `Developers`). `HERC_UE_MAP=/Game/Maps/Overview` against this graft resolves to **Go2Sim's** copy. For cross-project screening, build an isolated clone (`Binaries`/`Plugins`/`Config`/`Source` symlinked, private `Content`) as was done at `/home/lucas/UE5/screen-cf/Blocks` and `/home/lucas/UE5/screen-p2/Blocks`.
- No git commits. No other project's content or GameModes modified.

---

## 4. What to do if none beat texturized Blocks

**This is the realised case.** The standing plan, in priority order:

**1 — Accept it. Done.** Texturized Blocks is the default. It is not merely un-beaten; it is now verified on all three criteria with independent instruments: cost 0.998× plain Blocks, openness 630/630 with a 169×99 m clear box at the spawn line, texture ST median 1128 / ground median 328 — landing in the same band as the JapanFest cure (ST 923–1369, ground 476–747) while costing **1.75× less**. It cures feature starvation without importing corridor geometry or draw-call load.

**2 — The one honest weakness, and the one thing worth building.** Blocks' 181 obstacles are **axis-aligned, identical grey cubes**. The noise material fixes *feature density*; it does not fix *place recognition*. Loop closure against dozens of visually identical boxes remains unusable, and that is a real gap for a mapping fleet. **The cheapest fix is not a new map — it is per-cube material variation in the existing one:** extend `texturize.py` to drive the noise seed / hue / scale from the actor's world position so each cube gets a distinguishable signature. Same draw calls, same 196 estimate, same materials count. This is the highest-value remaining move and it costs one commandlet run.

**3 — Texturize an open shell, if more parallax is genuinely needed.** Three untried, all zero-cost, all already grafted or trivially graftable:
- `/Game/Untitled` — bare 64-tile landscape, **100% arena flyable, 60×60 m clear**, ground-half med 44, cost 1.20×. Only needs a landscape-material swap. Currently a repeating checkerboard, which is an *active aliasing hazard* (confident wrong stereo matches) — replacing it with world-locked noise removes a hazard and adds features in one edit.
- `/Game/crazyFlieSim` — same landscape, ground-half med 31.
- `/Game/LowPolyMoonPack/Maps/Demo` — **54×54 m clear, 580/580, cost 1.00–1.01×**, 426k non-repeating rock silhouettes giving real 3D parallax at every heading, 17 meshes / 14 materials the script can hit in one pass. Best *shell* on the host. Note its rocks have no collision.

**4 — Only if a genuinely different environment is required.** Two options, both real projects rather than ports: (a) re-acquire a **5.2-era** outdoor pack — MedievalVillage / MountainTops-class content is exactly the missing open+textured quadrant, and every equivalent on this host is 5.6/5.8 and unopenable; (b) author an arena from `kb3d_missiontominerva` (3.8 GB, 976 assets, **is** 5.2-compatible, ships only per-prop sublevels). Build to spec: ≥30×30 m clear at 1–3 m, ≤600 draw calls, textured non-repeating ground.

**5 — Do not repeat these.** JapanFest / `L_Showcase` (same map, rejected). `Maps/Demonstration` (3.07–3.26× cost). `Asian_Village_Demo` (8085 draw calls). `CacaoField` (8 m rows at any trunk model; also missing `/Game/UltimateFarming`). Anything needing Cesium or UnrealSplat. Anything 5.3+. The Epic VaultCache. `Home_Interior_Overview` — it *looks* like a pass and is worse than raw Blocks.

---

## 5. The rule for judging any future candidate map

> **Screen every candidate against the same two anchors — plain Blocks (the known failure) and JapanFest (the known cure that was still rejected) — measured with one probe in one quiet bracketed block, and promote nothing that has not cleared all four gates in this order.**
> **(1) COST**, cheapest to measure and the hardest failure: take a warmed steady-state stereo round-trip (≥200 warm-up calls; the cold-cache reading under-predicted JapanFest's real 2.08× as 1.39×) on a single isolated RPC connection, expressed only as **×Blocks control measured the same hour**, and reject above **~1.5× warmed** — JapanFest died at 2.08×, `Maps/Demonstration` at 3.26×. Note that **actor count is a poor predictor below ~10k actors and would have screened out the best candidate**: Overview has 6.4× Blocks' actors and renders *cheaper*; only draw-call submission diversity (unique meshes × material slots) and dynamic light/decal counts actually track cost, so **measure, do not infer**.
> **(2) TEXTURE**, and always gate the metric before trusting it: a raw Shi-Tomasi count is meaningless without `FAST > 0`, `LK matches > 0`, finite depth and non-zero usable disparity — a blank sky scores a saturated ST 2000 with zero matches, and an invisible ground plane scores 0% usable disparity at every pose while looking wide open on a voxel grid. Require ground-half features clearly above the plain-Blocks anchor (median 11, range 0–39) and reject anything at or below it.
> **(3) OPEN — and never trust a static voxel grid alone.** A 1 m occupancy grid scored Overview 625/630 flyable and it still stalled half the fleet, because free space fragments at the planner's *inflation radius*, not at 1 m. The only trustworthy openness measure is a short full-stack run scored on **`fleet_unique_m2` and planner `Path complete` / `PLAN OK` ratio**: reject below ~500/630 unique or below ~10% goal completion (Blocks 575–620 and 15.6–22.7%; Overview 276–428 and 2.2–5.8%). Editor `-nullrhi` collision queries are useless here — StaticMeshActor body setups do not register in a commandlet world, and actor-AABB blocking over-blocks; use AirSim `simCreateVoxelGrid` for a first pass and the full stack for the verdict.
> **(4) SPAWN**, which no offline probe predicts: before comparing anything, fly the candidate once and confirm all four drones log `PLAN_OK > 0` and non-zero FIS clusters — a single dead cell (Overview at X=15, Y=−12) silently returns 0 clusters / 0 cells for a whole run and invalidates the comparison; move the line and re-validate rather than reading the numbers.
> **Finally, three invariants that make any of this meaningful:** the port must stay **symlink-only with `?game=/Script/AirSim.AirSimGameMode`** so binary, plugins and GameMode remain bit-identical and the map is the only variable — **any map needing a plugin (Cesium, UnrealSplat) or streaming content from the network is disqualified on reproducibility alone, whatever it scores**; the host must be verified single-tenant with `load1` recorded on every row, because contention swings these numbers up to **3.9×** and one contaminated block already produced a 1.51× cost figure for a map that truly costs 0.91×; and remember the rig's own noise floor — **one catastrophic VIO divergence per ~8 drone-runs occurs on the control arm too**, so a single-drone outlier is never evidence about a map.

---

## 6. Appendix — instrument calibration and the traps found

Every probe was validated against the known JapanFest result **before** any candidate was judged.

| axis | instrument reading | documented value | verdict |
|---|---|---|---|
| actor count | JapanFest **2119** (also 2120 on a second parser) | 2119 | exact |
| cost | JapanFest 24.85 ms vs texturized Blocks 14.24 = **1.75×**; warmed 2.08× | 29.1 / 21.0 = 1.39× cold | same direction and order; the warmed ratio is the one that predicted the observed 18 → 8.4 Hz (2.1×) full-stack collapse |
| openness | JapanFest **357/630** arena cells, spawn cell blocked; 16 m largest open square | 301/630; "~6 m alleys" | ~19% more permissive — conservative in the safe direction |
| texture | JapanFest ground 0–760 (med 480) / 476–747 | 476–747 (med 552) | reproduces |
| texture, negative control | plain Blocks ground 0–39, **median 11** | median 11 | exact |

**Traps that produced "everything is wide open" or "empty sky is the best-textured map," all found and fixed:**

1. **`USceneComponent::Bounds` is a bare C++ member, not a UPROPERTY** — `get_editor_property('bounds')` raises and yields *zero* obstacles. Use the `AActor::GetActorBounds` UFUNCTION.
2. **A single global ground plane is wrong.** JapanFest's widest flat colliders are a 6563 m² basement slab (z 3.8–5.8) and an identical roof slab (z 54.2–56.2) while the street is at z≈22.5 — a fixed cruise band floats above the entire city and the map scores 100% open. Anchor the floor to the 5th percentile of collider z-min *local to the analysis window*.
3. **Sky-sphere / world-bound shells must be excluded as obstacles** — their AABB covers every cell. Filter z-extent > 500 m or footprint > 1e5 m².
4. **UE 5.2 editor-commandlet collision is unreliable.** A downward trace that hit the Blocks ground from 100 m up *missed the identical column* from 3.5 m; Blocks scored a nonsensical 3674/3721 clear. `PrimitiveComponent` also has no readable `collision_enabled` property in 5.2 Python.
5. **binvox layout.** `WorldSimApi.cpp:193` writes `idx = x + nx*(z + nz*y)` but emits a header of `dim nx nz ny`. Reshape to `(ny, nz, nx)` with **vertical as axis 1**; height for vertical index k is `(k − nz/2)*res`. Find ground **per column**, never from a fixed `spawn_z + 1 m` rule — the Blocks floor slab is 3 voxels thick and the NED origin sits ~1.9 m above it.
6. **World Partition maps do not stream actors in a `-run=pythonscript` commandlet** — `crazyFlieSim` and `Untitled` read as 9 actors / 0 colliding, and `world_partition` is not exposed in 5.2. Their real content is 73 external actor packages under `Content/__ExternalActors__/`; parse them offline.
7. **Teleport-and-capture probes must not `simSetVehiclePose` + sleep** — the drone free-falls and every "blocked" reading of `dmid=0.42 m` is the camera resting on the floor. `simPause(True)` is *worse*: it freezes the renderer too (56 distinct poses returned byte-identical frames) and persists across client disconnects, silently poisoning the next probe process. Re-assert the pose every 40 ms for ~0.4 s and capture immediately after a final set.
8. **Mine dependency roots from the name table, not the import table** — every import package path is interned there, and it is version-independent. These maps span legacy −7/−8 and ue5 versions 0/1004/1009; `SoftObjectPaths` count/offset exists only when ue5 ≥ 1008 and `LocalizationId` only when ue4 ≥ 516, and getting either wrong desynchronises the summary.

**Persisted artifacts, all re-runnable without another UE launch:**
- `/home/lucas/hercules-sim/investigation/phase2_go2sim/results/` — `mapscan.json` (static counts, 9 maps), `openness.json`, `probe_<LABEL>.json` (timing + per-pose load1), `features_<LABEL>.json`, raw `.binvox` grids, `contact_sheet.png` (visual proof of the void maps)
- `/home/lucas/hercules-sim/investigation/phase2_go2sim/scripts/` — `runtime_probe.py`, `voxan.py`, `feat.py`, `timeonly.py`, `mapscan.py`, `run_map.sh`, `sweep.sh`
- `/home/lucas/hercules-sim/investigation/vio/` — `texturize.py`, `run_texturize.sh`, `probe_materials.py`, `run_feature_probe.sh`, `coverage_report.py`, `team_box.py`
- `/home/lucas/hercules-sim/investigation/feature_probe/` — `measure_features.py`, `capture_waypoints.py`, `frames_baseline_blocks/`, `frames_pretexture_R3/`, `frames_textured/`
- `/home/lucas/hercules-sim/e1_frames/runs/{tb1,tb2,ov3,ov4}` (valid) and `{ov1_INVALID,ov2_INVALID}` — each with `scorecard_*.txt`, `metrics_*.csv`, `load_*.txt`, per-node logs
- `/home/lucas/hercules-sim/settings-overview-4drone.json` (dead spawn — keep as the counter-example), `settings-overviewB-4drone.json` (working)