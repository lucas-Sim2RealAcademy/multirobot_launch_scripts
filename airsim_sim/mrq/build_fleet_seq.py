# Runs HEADLESS in UE 5.2 (UnrealEditor-Cmd Blocks.uproject -ExecutePythonScript=...).
# Builds /Game/MRQ/SEQ_FleetJF: 4 spawnable AirSim quad meshes keyed from
# fleet_keys JSON (30 fps world-space keys) + a spawnable CineCamera with a
# designed dolly, camera-cut track, then writes the MRQ queue manifest for the
# -game render pass (proven m5 recipe: PNG + deferred + TSR temporal 4).
#
# Env: KEYS_JSON (required), OUT_DIR (frames), TEST=0/1, TEST_START/TEST_END,
#      DRONE_SCALE (default 1.0), YAW_OFFSET (deg, default 0)
import json
import os
import unreal

_PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/ue/stageA_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


KEYS_JSON = os.environ["KEYS_JSON"]
OUT_DIR = os.environ.get("OUT_DIR", "/home/lucas/UE5/hercules-sim-big/mrq_work/out/frames")
TEST = os.environ.get("TEST", "0") == "1"
TEST_START = int(os.environ.get("TEST_START", "600"))
TEST_END = int(os.environ.get("TEST_END", "615"))
DRONE_SCALE = float(os.environ.get("DRONE_SCALE", "1.0"))
YAW_OFFSET = float(os.environ.get("YAW_OFFSET", "0.0"))

MAP_PATH = "/Game/Maps/JapanFest_Street"
SEQ_DIR = "/Game/MRQ"
SEQ_NAME = "SEQ_FleetJF"
MESH_CANDIDATES = [
    "/AirSim/Models/QuadRotor1/QuadCopter",
    "/AirSim/Models/QuadRotor1/Quadrotor1",
    "/AirSim/Models/MiniQuadCopter/QuadcopterBody",
]

K = json.load(open(KEYS_JSON))
FPS, N = K["fps"], K["nframes"]
P(f"[fleet] keys: fps={FPS} nframes={N} run={K['info']['run']}")

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

# ---------- load the map (never saved) ----------
ok = les.load_level(MAP_PATH)
P(f"[fleet] load_level({MAP_PATH}) -> {ok}")
world = ues.get_editor_world()

# ---------- sanity: PlayerStart anchor + road trace ----------
for a in eas.get_all_level_actors():
    if isinstance(a, unreal.PlayerStart):
        L = a.get_actor_location()
        P(f"[fleet] PlayerStart at ({L.x:.0f},{L.y:.0f},{L.z:.0f}) cm (expected {K['info']['anchor']})")
g0 = K["drones"]["ghost"][0]
try:
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(g0[0], g0[1], g0[2]),
        unreal.Vector(g0[0], g0[1], g0[2] - 1500.0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        ip = hit.to_tuple()[4]  # impact_point
        P(f"[fleet] ghost frame0 z={g0[2]:.0f}, ground below at z={ip.z:.0f} (clearance {g0[2]-ip.z:.0f} cm)")
    else:
        P("[fleet] WARNING: no ground hit below ghost frame0")
except Exception as e:
    P(f"[fleet] trace sanity skipped: {e}")

# ---------- fresh sequence asset ----------
seq_path = f"{SEQ_DIR}/{SEQ_NAME}"
if unreal.EditorAssetLibrary.does_asset_exist(seq_path):
    unreal.EditorAssetLibrary.delete_asset(seq_path)
at = unreal.AssetToolsHelpers.get_asset_tools()
seq = at.create_asset(SEQ_NAME, SEQ_DIR, unreal.LevelSequence, unreal.LevelSequenceFactoryNew())
seq.set_display_rate(unreal.FrameRate(FPS, 1))
seq.set_playback_start(0)
seq.set_playback_end(N)
P(f"[fleet] created {seq_path}")

# 5.2 name is SequenceTimeUnit; 5.3+ renamed it MovieSceneTimeUnit
_TU = getattr(unreal, "SequenceTimeUnit", None) or getattr(unreal, "MovieSceneTimeUnit")
DR = _TU.DISPLAY_RATE


def key_channel(ch, vals):
    add = ch.add_key
    for f, v in enumerate(vals):
        add(unreal.FrameNumber(f), v, 0.0, DR, unreal.MovieSceneKeyInterpolation.LINEAR)


def add_transform_keys(binding, loc_xyz, rot_pyr, scale=1.0):
    """loc_xyz: ([x],[y],[z]) cm; rot_pyr: ([roll],[pitch],[yaw]) deg or None channels."""
    tr = binding.add_track(unreal.MovieScene3DTransformTrack)
    sec = tr.add_section()
    sec.set_range(0, N)
    ch = sec.get_all_channels()  # LocX,LocY,LocZ,Roll,Pitch,Yaw,SclX,SclY,SclZ
    for i in range(3):
        key_channel(ch[i], loc_xyz[i])
    for i, vals in enumerate(rot_pyr):
        if vals is not None:
            key_channel(ch[3 + i], vals)
    for i in (6, 7, 8):
        ch[i].set_default(scale)
    return sec


# ---------- pick the quad mesh ----------
mesh = None
for cand in MESH_CANDIDATES:
    m = unreal.EditorAssetLibrary.load_asset(cand)
    if isinstance(m, unreal.StaticMesh):
        mesh = m
        bb = m.get_bounding_box()
        P(f"[fleet] using mesh {cand} extent="
              f"({bb.max.x-bb.min.x:.0f},{bb.max.y-bb.min.y:.0f},{bb.max.z-bb.min.z:.0f}) cm")
        break
if mesh is None:
    raise RuntimeError("no static quad mesh found among candidates")

# ---------- drone spawnables ----------
for name, rows in K["drones"].items():
    tmp = eas.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -100000))
    smc = tmp.get_editor_property("static_mesh_component")
    smc.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    smc.set_static_mesh(mesh)
    smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    tmp.set_actor_label(f"Fleet_{name}")
    tmp.set_actor_scale3d(unreal.Vector(DRONE_SCALE, DRONE_SCALE, DRONE_SCALE))
    sp = seq.add_spawnable_from_instance(tmp)
    sp.set_name(f"Fleet_{name}")
    eas.destroy_actor(tmp)
    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    zs = [r[2] for r in rows]
    yaws = [r[3] + YAW_OFFSET for r in rows]
    add_transform_keys(sp, (xs, ys, zs), (None, None, yaws), scale=DRONE_SCALE)
    P(f"[fleet] baked {name}: {N} frames")

# ---------- camera spawnable ----------
cam_rows = K["camera"]
tmp = eas.spawn_actor_from_class(unreal.CineCameraActor, unreal.Vector(0, 0, -100000))
cc = tmp.get_cine_camera_component()
fb = cc.get_editor_property("filmback")
fb.sensor_width = 23.76
fb.sensor_height = 13.365
cc.set_editor_property("filmback", fb)
cc.set_editor_property("current_focal_length", 18.0)
fs = cc.get_editor_property("focus_settings")
fs.focus_method = unreal.CameraFocusMethod.DISABLE
cc.set_editor_property("focus_settings", fs)
tmp.set_actor_label("FleetCam")
cam_sp = seq.add_spawnable_from_instance(tmp)
cam_sp.set_name("FleetCam")
eas.destroy_actor(tmp)
add_transform_keys(
    cam_sp,
    ([r[0] for r in cam_rows], [r[1] for r in cam_rows], [r[2] for r in cam_rows]),
    (None, [r[3] for r in cam_rows], [r[4] for r in cam_rows]))

cut = seq.add_track(unreal.MovieSceneCameraCutTrack)
cs = cut.add_section()
cs.set_range(0, N)
try:
    bid = cam_sp.get_binding_id()
except Exception:
    bid = unreal.MovieSceneSequenceExtensions.get_binding_id(cam_sp)
cs.set_camera_binding_id(bid)
P("[fleet] camera + cut track done")

unreal.EditorAssetLibrary.save_directory(SEQ_DIR, only_if_is_dirty=False)
P("[fleet] saved /Game/MRQ")

# ---------- MRQ queue -> manifest ----------
qsub = unreal.get_editor_subsystem(unreal.MoviePipelineQueueSubsystem)
q = qsub.get_queue()
for j in list(q.get_jobs()):
    q.delete_job(j)
job = q.allocate_new_job(unreal.MoviePipelineExecutorJob)
job.job_name = "FleetJF"
job.map = unreal.SoftObjectPath(f"{MAP_PATH}.{MAP_PATH.split('/')[-1]}")
job.sequence = unreal.SoftObjectPath(f"{seq_path}.{SEQ_NAME}")
cfg = job.get_configuration()

outset = cfg.find_or_add_setting_by_class(unreal.MoviePipelineOutputSetting)
outset.output_directory = unreal.DirectoryPath(OUT_DIR)
outset.file_name_format = "frame_{frame_number}"
outset.output_resolution = unreal.IntPoint(1920, 1080)
outset.override_existing_output = True
outset.zero_pad_frame_numbers = 4
if TEST:
    outset.use_custom_playback_range = True
    outset.custom_start_frame = TEST_START
    outset.custom_end_frame = TEST_END
    P(f"[fleet] TEST range [{TEST_START},{TEST_END})")

cfg.find_or_add_setting_by_class(unreal.MoviePipelineImageSequenceOutput_PNG)
cfg.find_or_add_setting_by_class(unreal.MoviePipelineDeferredPassBase)
aa = cfg.find_or_add_setting_by_class(unreal.MoviePipelineAntiAliasingSetting)
aa.spatial_sample_count = 1
aa.temporal_sample_count = 4
aa.override_anti_aliasing = True
aa.anti_aliasing_method = unreal.AntiAliasingMethod.AAM_TSR
cfg.find_or_add_setting_by_class(unreal.MoviePipelineGameOverrideSetting)
cv = cfg.find_or_add_setting_by_class(unreal.MoviePipelineConsoleVariableSetting)
cv.add_or_update_console_variable("r.MotionBlurQuality", 0.0)
cv.add_or_update_console_variable("r.Tonemapper.Sharpen", 0.3)

res = unreal.MoviePipelineEditorLibrary.save_queue_to_manifest_file(q)
P(f"[fleet] manifest saved: {res}")
P("[fleet] BUILD_OK")
