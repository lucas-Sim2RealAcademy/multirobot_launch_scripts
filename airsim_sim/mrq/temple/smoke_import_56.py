# HEADLESS smoke test: (1) OBJ -> StaticMesh import (scale/axis/UV/nanite),
# (2) RGBA16 PNG texture import settings. Everything under /Game/FleetMRQ.
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
PROG = open(W + "/smoke_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    PROG.write(s + "\n")
    PROG.flush()


# asymmetric test quad: X extent 100, Y extent 200, Z extent 300, one-sided uv
obj = W + "/smoke_tri.obj"
open(obj, "w").write(
    "o smoke\nv 0 0 0\nv 100 0 0\nv 0 200 0\nv 0 0 300\n"
    "vt 0.25 0\nvt 0.5 0\nvt 0.75 0\nvt 1.0 0\n"
    "f 1/1 2/2 3/3\nf 1/1 3/3 4/4\n")

at = unreal.AssetToolsHelpers.get_asset_tools()
for ap in ["/Game/FleetMRQ/smoke_tri", "/Game/FleetMRQ/coverage_mrq6"]:
    if unreal.EditorAssetLibrary.does_asset_exist(ap):
        unreal.EditorAssetLibrary.delete_asset(ap)

tasks = []
t1 = unreal.AssetImportTask()
t1.filename = obj
t1.destination_path = "/Game/FleetMRQ"
t1.automated = True
t1.save = False
t1.replace_existing = True
tasks.append(t1)
t2 = unreal.AssetImportTask()
t2.filename = W + "/coverage_mrq6.png"
t2.destination_path = "/Game/FleetMRQ"
t2.automated = True
t2.save = False
t2.replace_existing = True
tasks.append(t2)
at.import_asset_tasks(tasks)

for a in unreal.EditorAssetLibrary.list_assets("/Game/FleetMRQ"):
    if "smoke" in a.lower() or "coverage" in a.lower():
        P("ASSET", a)

m = unreal.EditorAssetLibrary.load_asset("/Game/FleetMRQ/smoke_tri")
if isinstance(m, unreal.StaticMesh):
    bb = m.get_bounding_box()
    P("OBJ_BOUNDS min", bb.min, "max", bb.max)
    P("OBJ_NUMUV", m.get_num_uv_channels(0) if hasattr(m, "get_num_uv_channels") else "?")
    try:
        ns = m.get_editor_property("nanite_settings")
        P("OBJ_NANITE", ns.get_editor_property("enabled"))
    except Exception as e:
        P("OBJ_NANITE_ERR", e)
else:
    P("OBJ_IMPORT_FAILED", m)

tex = unreal.EditorAssetLibrary.load_asset("/Game/FleetMRQ/coverage_mrq6")
if isinstance(tex, unreal.Texture2D):
    P("TEX", tex.blueprint_get_size_x(), "x", tex.blueprint_get_size_y())
    P("TEX_SRGB", tex.get_editor_property("srgb"),
      "COMP", tex.get_editor_property("compression_settings"),
      "MIPGEN", tex.get_editor_property("mip_gen_settings"),
      "FILTER", tex.get_editor_property("filter"))
else:
    P("TEX_IMPORT_FAILED", tex)
P("SMOKE_OK")
