# Diagnostic: 8-frame sequence, one camera vantage per frame (constant keys),
# to locate a clear, lit camera position in JapanFest_Street. Also inventories
# light actors. Renders via the same manifest flow as the main build.
import json
import os
import unreal

_PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/ue/diag_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


KEYS_JSON = os.environ["KEYS_JSON"]
OUT_DIR = os.environ.get("OUT_DIR", "/home/lucas/UE5/hercules-sim-big/mrq_work/out/diag_frames")
MAP_PATH = "/Game/Maps/JapanFest_Street"

K = json.load(open(KEYS_JSON))
k = 600
cents = []
for kk in (0, 600, 1200, 1800):
    cx = sum(K["drones"][v][kk][0] for v in K["drones"]) / 4
    cy = sum(K["drones"][v][kk][1] for v in K["drones"]) / 4
    cz = sum(K["drones"][v][kk][2] for v in K["drones"]) / 4
    cents.append((cx, cy, cz))
C = cents[1]
PS = (-1006.0, -119.0, 2348.0)
ROAD = 2250.0

import math


def look(px, py, pz, tx, ty, tz):
    dx, dy, dz = tx - px, ty - py, tz - pz
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return (px, py, pz, pitch, yaw)


if os.environ.get("CAM_FROM_KEYS", "0") == "1":
    # sample the exported follow-camera path at 8 spots across the window
    n = K["nframes"]
    VANTAGES = [tuple(K["camera"][min(n - 1, int(i * (n - 1) / 7))]) for i in range(8)]
else:
    VANTAGES = [
        look(PS[0], PS[1], PS[2] + 250, PS[0] - 2000, PS[1], PS[2]),      # 0 from PlayerStart look -X (spawn line dir)
        look(PS[0], PS[1], PS[2] + 250, PS[0] + 2000, PS[1], PS[2]),      # 1 look +X
        look(C[0], -419.0, ROAD + 2500, C[0] + 1, -419.0, ROAD),          # 2 top-down over centroid
        look(C[0] - 550, -1050.0, 2680.0, C[0], C[1], C[2]),              # 3 = planned cam (reproduce)
        look(C[0] - 550, -419.0, 2680.0, C[0], C[1], C[2]),               # 4 street centerline
        look(C[0] - 550, 250.0, 2680.0, C[0], C[1], C[2]),                # 5 north side
        look(K["drones"]["ghost"][k][0] - 300, K["drones"]["ghost"][k][1], K["drones"]["ghost"][k][2] + 120,
             K["drones"]["ghost"][k][0], K["drones"]["ghost"][k][1], K["drones"]["ghost"][k][2]),  # 6 chase ghost
        look(C[0] - 1200, -419.0, ROAD + 600, C[0], -419.0, ROAD + 150),  # 7 low fwd down-street
    ]

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ok = les.load_level(MAP_PATH)
P(f"[diag] load_level -> {ok}")

# light inventory
n = 0
for a in eas.get_all_level_actors():
    cn = a.get_class().get_name()
    if "Light" in cn or "Sky" in cn or "Fog" in cn or "PostProcess" in cn:
        mob = ""
        try:
            comp = a.get_component_by_class(unreal.LightComponentBase)
            if comp:
                mob = str(comp.get_editor_property("mobility"))
                mob += f" intensity={comp.get_editor_property('intensity'):.1f}"
        except Exception:
            pass
        P(f"[diag] LIGHTACTOR {a.get_actor_label()} class={cn} {mob}")
        n += 1
P(f"[diag] {n} light/sky/fog/pp actors")

SEQ_DIR, SEQ_NAME = "/Game/MRQ", "SEQ_Diag"
seq_path = f"{SEQ_DIR}/{SEQ_NAME}"
if unreal.EditorAssetLibrary.does_asset_exist(seq_path):
    unreal.EditorAssetLibrary.delete_asset(seq_path)
at = unreal.AssetToolsHelpers.get_asset_tools()
seq = at.create_asset(SEQ_NAME, SEQ_DIR, unreal.LevelSequence, unreal.LevelSequenceFactoryNew())
seq.set_display_rate(unreal.FrameRate(30, 1))
seq.set_playback_start(0)
seq.set_playback_end(len(VANTAGES))

_TU = getattr(unreal, "SequenceTimeUnit", None) or getattr(unreal, "MovieSceneTimeUnit")
DR = _TU.DISPLAY_RATE

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
cam_sp = seq.add_spawnable_from_instance(tmp)
cam_sp.set_name("DiagCam")
eas.destroy_actor(tmp)

tr = cam_sp.add_track(unreal.MovieScene3DTransformTrack)
sec = tr.add_section()
sec.set_range(0, len(VANTAGES))
ch = sec.get_all_channels()
for f, (px, py, pz, pitch, yaw) in enumerate(VANTAGES):
    fn = unreal.FrameNumber(f)
    CONST = unreal.MovieSceneKeyInterpolation.CONSTANT
    ch[0].add_key(fn, px, 0.0, DR, CONST)
    ch[1].add_key(fn, py, 0.0, DR, CONST)
    ch[2].add_key(fn, pz, 0.0, DR, CONST)
    ch[4].add_key(fn, pitch, 0.0, DR, CONST)
    ch[5].add_key(fn, yaw, 0.0, DR, CONST)
    P(f"[diag] v{f}: pos=({px:.0f},{py:.0f},{pz:.0f}) pitch={pitch:.1f} yaw={yaw:.1f}")
for i in (6, 7, 8):
    ch[i].set_default(1.0)

cut = seq.add_track(unreal.MovieSceneCameraCutTrack)
cs = cut.add_section()
cs.set_range(0, len(VANTAGES))
try:
    bid = cam_sp.get_binding_id()
except Exception:
    bid = unreal.MovieSceneSequenceExtensions.get_binding_id(cam_sp)
cs.set_camera_binding_id(bid)

# drones posed at the sampled window frames (composition check)
if os.environ.get("CAM_FROM_KEYS", "0") == "1":
    n = K["nframes"]
    ks = [min(n - 1, int(i * (n - 1) / 7)) for i in range(8)]
    mesh = None
    for cand in ["/AirSim/Models/QuadRotor1/Quadrotor1", "/AirSim/Models/QuadRotor1/QuadCopter"]:
        m = unreal.EditorAssetLibrary.load_asset(cand)
        if isinstance(m, unreal.StaticMesh):
            mesh = m
            break
    for name, rows in K["drones"].items():
        tmpd = eas.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -100000))
        smc = tmpd.get_editor_property("static_mesh_component")
        smc.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        smc.set_static_mesh(mesh)
        smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        tmpd.set_actor_label(f"Diag_{name}")
        spd = seq.add_spawnable_from_instance(tmpd)
        spd.set_name(f"Diag_{name}")
        eas.destroy_actor(tmpd)
        trd = spd.add_track(unreal.MovieScene3DTransformTrack)
        secd = trd.add_section()
        secd.set_range(0, len(VANTAGES))
        chd = secd.get_all_channels()
        CONST = unreal.MovieSceneKeyInterpolation.CONSTANT
        for f, kk in enumerate(ks):
            r = rows[kk]
            fn = unreal.FrameNumber(f)
            chd[0].add_key(fn, r[0], 0.0, DR, CONST)
            chd[1].add_key(fn, r[1], 0.0, DR, CONST)
            chd[2].add_key(fn, r[2], 0.0, DR, CONST)
            chd[5].add_key(fn, r[3], 0.0, DR, CONST)
        for i in (6, 7, 8):
            chd[i].set_default(1.0)
    P(f"[diag] drones posed at ks={ks}")

unreal.EditorAssetLibrary.save_directory(SEQ_DIR, only_if_is_dirty=False)

qsub = unreal.get_editor_subsystem(unreal.MoviePipelineQueueSubsystem)
q = qsub.get_queue()
for j in list(q.get_jobs()):
    q.delete_job(j)
job = q.allocate_new_job(unreal.MoviePipelineExecutorJob)
job.job_name = "Diag"
job.map = unreal.SoftObjectPath(f"{MAP_PATH}.{MAP_PATH.split('/')[-1]}")
job.sequence = unreal.SoftObjectPath(f"{seq_path}.{SEQ_NAME}")
cfg = job.get_configuration()
outset = cfg.find_or_add_setting_by_class(unreal.MoviePipelineOutputSetting)
outset.output_directory = unreal.DirectoryPath(OUT_DIR)
outset.file_name_format = "diag_{frame_number}"
outset.output_resolution = unreal.IntPoint(1280, 720)
outset.override_existing_output = True
outset.zero_pad_frame_numbers = 4
cfg.find_or_add_setting_by_class(unreal.MoviePipelineImageSequenceOutput_PNG)
cfg.find_or_add_setting_by_class(unreal.MoviePipelineDeferredPassBase)
aa = cfg.find_or_add_setting_by_class(unreal.MoviePipelineAntiAliasingSetting)
aa.spatial_sample_count = 1
aa.temporal_sample_count = 1
aa.override_anti_aliasing = True
aa.anti_aliasing_method = unreal.AntiAliasingMethod.AAM_TSR
cfg.find_or_add_setting_by_class(unreal.MoviePipelineGameOverrideSetting)
res = unreal.MoviePipelineEditorLibrary.save_queue_to_manifest_file(q)
P(f"[diag] manifest saved: {res}")
P("[diag] BUILD_OK")
