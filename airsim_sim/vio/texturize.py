"""R3 (VIO-STABILIZATION-PLAN.md §2) -- texture the Blocks world in place.

Blocks' only in-sensor-range surface (the ground plane at 2.4-4.8 m, where 98-100% of
pixels would exceed 2 px of stereo disparity) is painted with an untextured material, so
it yields 0-39 features.  Every feature the map *does* offer is a box silhouette at
20-85 m -> 0.2-0.8 px of disparity with a 50 mm baseline and fx=337 px.  The scene
therefore constrains rotation only.

This edits the two materials the whole map uses -- no new assets, no map edit, no disk
cost -- projecting a *world-locked* two-octave noise pattern into BaseColor.  World-locked
(not UV-locked, not screen-locked) is the point: the pattern is a static 3-D property of
the surface, so it triangulates and it is identical in both stereo eyes.

TWO CORRECTIONS to the plan's §2 R3 snippet, both verified by probe_materials.py against
the live map (181 actors):

  1. The plan has the two materials SWAPPED.  It says Ground uses GrayMaterial and the
     TemplateCube_Rounded_* use CubeMaterial.  In fact:
       /Game/Geometry/Meshes/CubeMaterial  -> 1 actor  : Ground        (a UMaterial)
       /Game/Flying/Meshes/GrayMaterial    -> 155 actors: the boxes    (a MaterialInstance)
     The scales therefore attach the other way round from the plan's listing.
  2. GrayMaterial is a MaterialInstanceConstant, not a UMaterial, so
     MaterialEditingLibrary.create_material_expression rejects it
     ("Cannot nativize 'MaterialInstanceConstant' as 'Material'").  Its parent
     /Game/Flying/Meshes/BaseMaterial is the UMaterial to edit; all 155 boxes inherit.

Run headless via investigation/vio/run_texturize.sh.
"""
import unreal

MEL = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary

# (asset path, micro-octave scale).  Scale is in 1/metres after the cm->m multiply below.
TARGETS = (
    # Ground, 1 actor, seen at 2.4-4.8 m -> ~12 cm features -> ~17 px of disparity texture
    ("/Game/Geometry/Meshes/CubeMaterial", 8.0),
    # the 155 TemplateCube_Rounded_* boxes at 20-85 m -> ~25 cm features -> ~4 px at 20 m.
    # (parent of the GrayMaterial instance the boxes actually reference)
    ("/Game/Flying/Meshes/BaseMaterial", 4.0),
)
MACRO_SCALE = 0.35          # ~3 m macro octave -- anti-tiling (GAP-CLOSURE-PLAN.md:62)


def log(s):
    # unreal.log() lands in LogPython Display, which the commandlet filters out of -stdout.
    unreal.log_warning("[texturize] %s" % s)


def texturize(path, micro_scale):
    m = EAL.load_asset(path)
    if m is None:
        log("MISSING %s" % path)
        return False
    if not isinstance(m, unreal.Material):
        log("SKIP %s: %s is not a UMaterial" % (path, m.get_class().get_name()))
        return False

    wp = MEL.create_material_expression(m, unreal.MaterialExpressionWorldPosition, -900, 0)
    mul = MEL.create_material_expression(m, unreal.MaterialExpressionMultiply, -700, 0)
    mul.set_editor_property("const_b", 0.01)              # UE cm -> m

    n1 = MEL.create_material_expression(m, unreal.MaterialExpressionNoise, -500, 60)
    n2 = MEL.create_material_expression(m, unreal.MaterialExpressionNoise, -500, -60)
    for n, s in ((n1, micro_scale), (n2, MACRO_SCALE)):   # TWO octaves, never one
        # FOURTH correction: the plan sets `function`, but UMaterialExpressionNoise::Function
        # is protected in UE 5.2's Python bindings ("Property 'Function' ... is protected and
        # cannot be set").  Its C++ default is already NOISEFUNCTION_SIMPLEX_TEX, which is
        # what the plan asks for, so the set is unnecessary -- read it back to prove it.
        for k, v in (("scale", s), ("levels", 4),         # levels <=4: per-pixel cost x8 renders
                     ("output_min", 0.20), ("output_max", 1.0)):
            try:
                n.set_editor_property(k, v)
            except Exception as e:
                log("  cannot set %s: %s" % (k, e))
        log("  noise: scale=%s levels=%s min=%s max=%s (function is protected, C++ default"
            " NOISEFUNCTION_SIMPLEX_TEX applies)" % (
                n.get_editor_property("scale"), n.get_editor_property("levels"),
                n.get_editor_property("output_min"), n.get_editor_property("output_max")))

    lerp = MEL.create_material_expression(
        m, unreal.MaterialExpressionLinearInterpolate, -300, 0)
    # THIRD correction: the plan leaves Alpha unconnected, and UE's default ConstAlpha is
    # 0.0 -- which makes Lerp == A and silently drops the macro octave entirely, defeating
    # the anti-tiling requirement the plan itself states.  0.5 blends both octaves evenly.
    lerp.set_editor_property("const_alpha", 0.5)

    MEL.connect_material_expressions(wp, "", mul, "A")
    MEL.connect_material_expressions(mul, "", n1, "Position")
    MEL.connect_material_expressions(mul, "", n2, "Position")
    MEL.connect_material_expressions(n1, "", lerp, "A")
    MEL.connect_material_expressions(n2, "", lerp, "B")
    MEL.connect_material_property(lerp, "", unreal.MaterialProperty.MP_BASE_COLOR)

    MEL.recompile_material(m)
    ok = EAL.save_asset(path)
    log("%s micro=%.2f macro=%.2f saved=%s" % (path, micro_scale, MACRO_SCALE, ok))
    return ok


for p, s in TARGETS:
    texturize(p, s)
log("done")
