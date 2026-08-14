"""Report what the Blocks map actually uses, so R3 edits the right assets.

The plan assumes /Game/Flying/Meshes/GrayMaterial and /Game/Geometry/Meshes/CubeMaterial
are both UMaterial.  Verify class, resolve MaterialInstanceConstant -> parent UMaterial,
and tally which actors reference what (map must be loaded explicitly in a commandlet).
"""
import unreal

EAL = unreal.EditorAssetLibrary
MAP = "/Game/FlyingCPP/Maps/FlyingExampleMap"


def log(s):
    # unreal.log() lands in LogPython Display, which the commandlet filters out of -stdout;
    # log_warning survives.
    unreal.log_warning("[probe] %s" % s)


for p in ("/Game/Flying/Meshes/GrayMaterial",
          "/Game/Geometry/Meshes/CubeMaterial",
          "/Game/Flying/Meshes/BaseMaterial"):
    a = EAL.load_asset(p)
    if a is None:
        log("%-42s MISSING" % p)
        continue
    cls = a.get_class().get_name()
    line = "%-42s %s" % (p, cls)
    if isinstance(a, unreal.MaterialInstanceConstant):
        par = a.get_editor_property("parent")
        line += "  parent=%s (%s)" % (
            par.get_path_name() if par else None,
            par.get_class().get_name() if par else "-")
    log(line)

try:
    unreal.EditorLoadingAndSavingUtils.load_map(MAP)
    log("loaded map %s" % MAP)
except Exception as e:
    log("load_map failed: %s" % e)

try:
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = eas.get_all_level_actors()
except Exception as e:
    log("no actor subsystem: %s" % e)
    actors = []

log("actors in level: %d" % len(actors))
tally = {}
for a in actors:
    for c in a.get_components_by_class(unreal.StaticMeshComponent):
        try:
            mats = c.get_materials()
        except Exception:
            continue
        for m in mats:
            if m is None:
                continue
            tally.setdefault(m.get_path_name().split('.')[0], []).append(a.get_actor_label())
for p in sorted(tally, key=lambda k: -len(tally[k])):
    log("MATUSE %-52s %4d actors  e.g. %s"
        % (p, len(tally[p]), ", ".join(sorted(set(tally[p]))[:3])))
