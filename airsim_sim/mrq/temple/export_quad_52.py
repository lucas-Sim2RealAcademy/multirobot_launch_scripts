# Runs HEADLESS in UE 5.2 (Blocks project): export the AirSim QuadCopter static
# mesh to FBX + its textures to TGA, and print material slot info for the 5.6 rebuild.
import os
import unreal

OUT = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple/quad_export"
os.makedirs(OUT, exist_ok=True)
PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/temple/export_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    PROG.write(s + "\n")
    PROG.flush()


for a in unreal.EditorAssetLibrary.list_assets("/AirSim/Models/QuadRotor1"):
    P("ASSET", a)
mesh = None
for cand in ["/AirSim/Models/QuadRotor1/Quadrotor1",
             "/AirSim/Models/QuadRotor1/QuadCopter",
             "/AirSim/Models/MiniQuadCopter/QuadcopterBody"]:
    m = unreal.EditorAssetLibrary.load_asset(cand)
    if isinstance(m, unreal.StaticMesh):
        mesh = m
        P("MESH", cand)
        break
if mesh is None:
    raise RuntimeError("no StaticMesh among candidates")
bb = mesh.get_bounding_box()
P("EXTENT_CM", round(bb.max.x - bb.min.x, 1), round(bb.max.y - bb.min.y, 1), round(bb.max.z - bb.min.z, 1))

try:
    mats = mesh.get_editor_property("static_materials")
    for i, m in enumerate(mats):
        mi = m.get_editor_property("material_interface")
        P("SLOT", i, m.get_editor_property("material_slot_name"),
          mi.get_path_name() if mi else None)
        # dig texture params if it's a material instance / material
        try:
            if mi:
                texs = unreal.MaterialEditingLibrary.get_used_textures(mi.get_base_material()) if hasattr(unreal, "MaterialEditingLibrary") else []
                for t in texs:
                    P("  USES_TEX", t.get_path_name())
        except Exception as e:
            P("  tex-dig skipped:", e)
except Exception as e:
    P("SLOTS_ERR", e)

task = unreal.AssetExportTask()
task.object = mesh
task.filename = OUT + "/QuadCopter.fbx"
task.automated = True
task.replace_identical = True
task.prompt = False
opts = unreal.FbxExportOption()
opts.collision = False
opts.level_of_detail = False
task.options = opts
ok = unreal.Exporter.run_asset_export_task(task)
P("EXPORT_FBX", ok, task.filename, os.path.exists(task.filename))

for t in ["T_QuadCopter_Body_2", "T_QuadCopter_Frame", "T_QuadCopter_Prop", "T_QuadCopter_Carmera"]:
    a = unreal.EditorAssetLibrary.load_asset("/AirSim/Models/QuadRotor1/" + t)
    if not a:
        P("EXPORT_TEX_MISSING", t)
        continue
    tt = unreal.AssetExportTask()
    tt.object = a
    tt.filename = OUT + "/" + t + ".tga"
    tt.automated = True
    tt.replace_identical = True
    tt.prompt = False
    r = unreal.Exporter.run_asset_export_task(tt)
    P("EXPORT_TEX", t, r, os.path.exists(tt.filename))

P("EXPORT_OK")
