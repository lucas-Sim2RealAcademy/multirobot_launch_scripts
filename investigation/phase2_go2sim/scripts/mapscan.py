"""PHASE-2 static screen: per-map actor cost + open-volume occupancy.

Runs as a -run=pythonscript commandlet with -nullrhi (no shader compile, no DDC growth).
For each candidate map:
  COST   total actors, actors with blocking collision, StaticMeshComponent count,
         ISM/HISM instance count, material-slot count (draw-call proxy), light count.
  OPEN   downward traces to find ground, sphere overlaps (r=0.5 m) at spawn_z+1/2/3 m
         over a +-30 m survey grid -> flyable mask -> largest clear axis-aligned box,
         4-drone line test (X=-7, Y=-7..+2, 3 m spacing), and the 630-cell arena
         (X_ned -17..+2, Y_ned -17..+11) flyable count.
Everything is printed with a [SCAN] prefix; log_warning survives commandlet filtering.
"""
import json
import math
import unreal

MAPS = [
    "/Game/Maps/Overview",
    "/Game/Maps/TechArt",
    "/Game/Maps/Demonstration",
    "/Game/Home_Interior/Maps/Home_Interior",
    "/Game/Home_Interior/Maps/Home_Interior_Overview",
    "/Game/LowPolyMoonPack/Maps/Overview",
    "/Game/LowPolyMoonPack/Maps/Demo",
    "/Game/FlyingCPP/Maps/FlyingExampleMap",   # Blocks incumbent, control
    "/Game/Maps/JapanFest_Street",             # tested/rejected, control
]

HALF = 30          # survey half-extent, metres
STEP = 1           # metres
R_DRONE = 50.0     # cm, sphere radius for clearance (1 m dia drone box)
HEIGHTS_M = [1.0, 2.0, 3.0]

OTQ = [unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY1,   # WorldStatic
       unreal.ObjectTypeQuery.OBJECT_TYPE_QUERY2]   # WorldDynamic


def log(s):
    unreal.log_warning("[SCAN] %s" % s)


def get_world():
    try:
        return unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    except Exception:
        return unreal.EditorLevelLibrary.get_editor_world()


def blocking(actor):
    """True if the actor has any primitive component that blocks the Pawn channel."""
    try:
        prims = actor.get_components_by_class(unreal.PrimitiveComponent)
    except Exception:
        return False
    for p in prims:
        try:
            if not p.get_editor_property("collision_enabled") in (
                    unreal.CollisionEnabled.QUERY_ONLY,
                    unreal.CollisionEnabled.QUERY_AND_PHYSICS,
                    unreal.CollisionEnabled.PROBE_ONLY,
                    unreal.CollisionEnabled.QUERY_AND_PROBE):
                continue
        except Exception:
            pass
        try:
            if p.get_collision_response_to_channel(
                    unreal.CollisionChannel.ECC_PAWN) == unreal.CollisionResponse.ECR_BLOCK:
                return True
        except Exception:
            return True
    return False


def largest_box(mask, nx, ny):
    """Maximal all-True axis-aligned rectangle in a nx*ny boolean grid. Returns
    (w, h, x0, y0) in cells."""
    best = (0, 0, 0, 0)
    best_area = 0
    heights = [0] * ny
    for i in range(nx):
        for j in range(ny):
            heights[j] = heights[j] + 1 if mask[i * ny + j] else 0
        # largest rectangle in histogram, tracking the row span
        stack = []
        for j in range(ny + 1):
            h = heights[j] if j < ny else 0
            start = j
            while stack and stack[-1][1] > h:
                sj, sh = stack.pop()
                area = sh * (j - sj)
                if area > best_area:
                    best_area = area
                    best = (sh, j - sj, i - sh + 1, sj)   # (x_cells, y_cells, x0, y0)
                start = sj
            stack.append((start, h))
    return best + (best_area,)


def scan(mp):
    out = {"map": mp}
    try:
        unreal.EditorLoadingAndSavingUtils.load_map(mp)
    except Exception as e:
        out["error"] = "load_map: %s" % e
        return out
    w = get_world()
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors = eas.get_all_level_actors()
    out["actors_total"] = len(actors)

    ncol = 0
    nsmc = 0
    nslots = 0
    ninst = 0
    nlight = 0
    nskel = 0
    nlandscape = 0
    cls_tally = {}
    for a in actors:
        cn = a.get_class().get_name()
        cls_tally[cn] = cls_tally.get(cn, 0) + 1
        if "Landscape" in cn:
            nlandscape += 1
        if blocking(a):
            ncol += 1
        try:
            for c in a.get_components_by_class(unreal.StaticMeshComponent):
                nsmc += 1
                try:
                    nslots += c.get_num_materials()
                except Exception:
                    pass
                if isinstance(c, unreal.InstancedStaticMeshComponent):
                    try:
                        ninst += c.get_instance_count()
                    except Exception:
                        pass
            nskel += len(a.get_components_by_class(unreal.SkeletalMeshComponent))
            nlight += len(a.get_components_by_class(unreal.LightComponent))
        except Exception:
            pass
    out.update(colliding_actors=ncol, static_mesh_components=nsmc,
               material_slots=nslots, ism_instances=ninst,
               light_components=nlight, skeletal_components=nskel,
               landscape_actors=nlandscape)
    out["top_classes"] = sorted(cls_tally.items(), key=lambda kv: -kv[1])[:8]

    # ---- reference frame: AirSim NED origin = PlayerStart, else world origin ----
    ps = [a for a in actors if "PlayerStart" in a.get_class().get_name()]
    if ps:
        L = ps[0].get_actor_location()
        origin = (L.x, L.y, L.z)
        out["playerstart"] = [round(L.x, 1), round(L.y, 1), round(L.z, 1)]
    else:
        origin = (0.0, 0.0, 0.0)
        out["playerstart"] = None
    ox, oy, oz = origin

    # ---- ground height per cell (downward trace from +100 m over origin plane) ----
    n = 2 * HALF // STEP + 1
    ground = [None] * (n * n)
    nohit = 0
    for i in range(n):
        for j in range(n):
            x = ox + (i * STEP - HALF) * 100.0
            y = oy + (j * STEP - HALF) * 100.0
            hit = unreal.SystemLibrary.line_trace_single(
                w, unreal.Vector(x, y, oz + 10000.0), unreal.Vector(x, y, oz - 10000.0),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, [],
                unreal.DrawDebugTrace.NONE, True)
            if hit:
                ground[i * n + j] = hit.to_tuple()[4].z
            else:
                nohit += 1
    hits = [g for g in ground if g is not None]
    out["ground_trace_hits"] = len(hits)
    out["ground_trace_miss"] = nohit
    if hits:
        hits_s = sorted(hits)
        out["ground_z_cm_rel_spawn"] = [round(hits_s[0] - oz, 1),
                                        round(hits_s[len(hits_s) // 2] - oz, 1),
                                        round(hits_s[-1] - oz, 1)]

    # ---- clearance: sphere overlap at spawn_z + 1/2/3 m ----
    mask = [False] * (n * n)
    per_h = {h: 0 for h in HEIGHTS_M}
    for i in range(n):
        for j in range(n):
            x = ox + (i * STEP - HALF) * 100.0
            y = oy + (j * STEP - HALF) * 100.0
            allclear = True
            for h in HEIGHTS_M:
                z = oz + h * 100.0
                hitl = unreal.SystemLibrary.sphere_overlap_actors(
                    w, unreal.Vector(x, y, z), R_DRONE, OTQ, None, [])
                if hitl:
                    allclear = False
                else:
                    per_h[h] += 1
            mask[i * n + j] = allclear
    out["cells_survey_total"] = n * n
    out["cells_clear_1to3m"] = sum(mask)
    out["cells_clear_per_h"] = {str(k): v for k, v in per_h.items()}

    bx, by, x0, y0, area = largest_box(mask, n, n)
    out["largest_clear_box_m"] = [bx * STEP, by * STEP]
    out["largest_clear_box_area_m2"] = area * STEP * STEP
    out["largest_clear_box_origin_ned_m"] = [(x0 * STEP - HALF), (y0 * STEP - HALF)]

    # ---- 630-cell arena: X_ned -17..+2, Y_ned -17..+11 (team box union) ----
    arena_tot = 0
    arena_ok = 0
    for cx in range(-17, 3):
        for cy in range(-17, 12):
            i = cx + HALF
            j = cy + HALF
            if 0 <= i < n and 0 <= j < n:
                arena_tot += 1
                if mask[i * n + j]:
                    arena_ok += 1
    out["arena630_cells_tested"] = arena_tot
    out["arena630_cells_flyable"] = arena_ok

    # ---- 4-drone spawn line: X_ned=-7, Y_ned=-7,-4,-1,+2, at 1/2/3 m ----
    line = []
    for yv in (-7, -4, -1, 2):
        i = -7 + HALF
        j = yv + HALF
        line.append(bool(mask[i * n + j]))
    out["spawn_line_clear"] = line
    out["spawn_line_fits"] = all(line)
    return out


results = []
for mp in MAPS:
    log("==== %s ====" % mp)
    try:
        r = scan(mp)
    except Exception as e:
        r = {"map": mp, "error": repr(e)}
    results.append(r)
    log("RESULT " + json.dumps(r))

with open("/tmp/claude-1000/-home-lucas/9a5d12ea-e4d4-4347-82ae-9c0fc68a49d6/scratchpad/mapscan.json", "w") as f:
    json.dump(results, f, indent=1)
log("WROTE mapscan.json")
