# Headless (nullrhi) UE 5.6 scout: sweep ALL renderable bounds that the
# line-trace clearance sweep could NOT see -- ISM/HISM/foliage instances
# (any collision setting; per-frame body-radius test, unlike the every-10th
# -frame straight-segment traces) and StaticMeshActors that are NOT
# traceable on Visibility (no collision / physics-only / visibility ignore).
# Traceable SMAs within the corridor are ALSO tested and reported in a
# separate list (near-miss review only, the trace sweep owns them).
#
# READ-ONLY: loads the map, never saves anything.
# Env: KEYS_JSON, OUT_JSON, RAD (cm, default 70), CAM_RAD (default 30)
import json
import math
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
_PROG = open(W + "/scout_foliage_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


KEYS_JSON = os.environ.get("KEYS_JSON", W + "/fleet_keys_temple_v2.json")
OUT_JSON = os.environ.get("OUT_JSON", W + "/foliage_sweep_v2.json")
RAD = float(os.environ.get("RAD", "70"))
CAM_RAD = float(os.environ.get("CAM_RAD", "30"))
MAP_PATH = "/Game/AncientTempleRuins/Levels/L_Showcase_01"

K = json.load(open(KEYS_JSON))
paths = {n: [(r[0], r[1], r[2]) for r in rows] for n, rows in K["drones"].items()}
paths["camera"] = [(r[0], r[1], r[2]) for r in K["camera"]]
P(f"[scout] keys={KEYS_JSON} nframes={K['nframes']} RAD={RAD} CAM_RAD={CAM_RAD}")

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ok = les.load_level(MAP_PATH)
P(f"[scout] load_level -> {ok}")

# ---------------- flight corridor AABB (all paths, inflated) ----------------
allp = [p for pts in paths.values() for p in pts]
cor_min = [min(p[i] for p in allp) for i in range(3)]
cor_max = [max(p[i] for p in allp) for i in range(3)]
P(f"[scout] corridor X[{cor_min[0]:.0f},{cor_max[0]:.0f}] "
  f"Y[{cor_min[1]:.0f},{cor_max[1]:.0f}] Z[{cor_min[2]:.0f},{cor_max[2]:.0f}]")

ML = unreal.MathLibrary
NC = unreal.CollisionEnabled.NO_COLLISION
PO = unreal.CollisionEnabled.PHYSICS_ONLY


def is_traceable(comp):
    try:
        ce = comp.get_collision_enabled()
        if ce == NC or ce == PO:
            return False
        resp = comp.get_collision_response_to_channel(
            unreal.CollisionChannel.ECC_VISIBILITY)
        if resp == unreal.CollisionResponse.ECR_IGNORE:
            return False
        return True
    except Exception:
        return True


# candidate units: dict(kind, label, mesh, xf(Transform world), lmin, lmax,
#                       traceable, wmin, wmax)
cands = []
n_inst_total = 0
n_actors = 0
CORR_PAD = 600.0   # prefilter pad (cm): bsphere+rad margin handled per-mesh


def corridor_dist_xy(x, y):
    dx = max(cor_min[0] - x, 0.0, x - cor_max[0])
    dy = max(cor_min[1] - y, 0.0, y - cor_max[1])
    return math.hypot(dx, dy)


def world_aabb(xf, lmin, lmax):
    wmin = [1e18, 1e18, 1e18]
    wmax = [-1e18, -1e18, -1e18]
    for cx in (lmin.x, lmax.x):
        for cy in (lmin.y, lmax.y):
            for cz in (lmin.z, lmax.z):
                w = ML.transform_location(xf, unreal.Vector(cx, cy, cz))
                wmin[0] = min(wmin[0], w.x); wmax[0] = max(wmax[0], w.x)
                wmin[1] = min(wmin[1], w.y); wmax[1] = max(wmax[1], w.y)
                wmin[2] = min(wmin[2], w.z); wmax[2] = max(wmax[2], w.z)
    return wmin, wmax


def add_cand(kind, label, meshname, xf, lmin, lmax, traceable, iidx):
    wmin, wmax = world_aabb(xf, lmin, lmax)
    # reject if world AABB (inflated by RAD) misses the corridor AABB
    for i in range(3):
        if wmin[i] - RAD > cor_max[i] or wmax[i] + RAD < cor_min[i]:
            return
    cands.append(dict(kind=kind, label=label, mesh=meshname, xf=xf,
                      lmin=lmin, lmax=lmax, traceable=traceable,
                      inst=iidx, wmin=wmin, wmax=wmax))


for a in eas.get_all_level_actors():
    n_actors += 1
    try:
        lbl = a.get_actor_label()
    except Exception:
        lbl = "?"
    # ---- ISM / HISM / foliage components (per-instance) ----
    try:
        isms = list(a.get_components_by_class(unreal.InstancedStaticMeshComponent))
    except Exception:
        isms = []
    for comp in isms:
        sm = comp.get_editor_property("static_mesh")
        if sm is None:
            continue
        bb = sm.get_bounding_box()
        n = comp.get_instance_count()
        n_inst_total += n
        tr = is_traceable(comp)
        meshname = sm.get_name()
        # mesh-local bounding sphere radius for cheap prefilter
        ext = unreal.Vector(bb.max.x - bb.min.x, bb.max.y - bb.min.y,
                           bb.max.z - bb.min.z)
        for i in range(n):
            xf = comp.get_instance_transform(i, True)
            loc = xf.translation
            sc = xf.scale3d
            smax = max(abs(sc.x), abs(sc.y), abs(sc.z))
            rr = 0.5 * smax * max(ext.x, ext.y, ext.z)
            if corridor_dist_xy(loc.x, loc.y) > rr + RAD + CORR_PAD:
                continue
            add_cand("ISM", lbl, meshname, xf, bb.min, bb.max, tr, i)
    # ---- plain StaticMeshActors ----
    if isinstance(a, unreal.StaticMeshActor):
        smc = a.static_mesh_component
        if smc is None:
            continue
        sm = smc.get_editor_property("static_mesh")
        if sm is None:
            continue
        bb = sm.get_bounding_box()
        xf = smc.get_component_transform() if hasattr(smc, "get_component_transform") \
            else a.get_actor_transform()
        loc = xf.translation
        sc = xf.scale3d
        smax = max(abs(sc.x), abs(sc.y), abs(sc.z))
        ext = unreal.Vector(bb.max.x - bb.min.x, bb.max.y - bb.min.y,
                           bb.max.z - bb.min.z)
        rr = 0.5 * smax * max(ext.x, ext.y, ext.z)
        tr = is_traceable(smc) and a.get_actor_enable_collision()
        if corridor_dist_xy(loc.x, loc.y) > rr + RAD + CORR_PAD:
            continue
        add_cand("SMA", lbl, sm.get_name(), xf, bb.min, bb.max, tr, -1)

P(f"[scout] actors={n_actors} ism_instances_total={n_inst_total} "
  f"candidates_in_corridor={len(cands)} "
  f"(non-traceable: {sum(1 for c in cands if not c['traceable'])})")

# ---------------- grid bucket by XY (world AABB inflated by RAD) -----------
CELL = 400.0
grid = {}
for ci, c in enumerate(cands):
    x0 = int((c["wmin"][0] - RAD) // CELL); x1 = int((c["wmax"][0] + RAD) // CELL)
    y0 = int((c["wmin"][1] - RAD) // CELL); y1 = int((c["wmax"][1] + RAD) // CELL)
    for gx in range(x0, x1 + 1):
        for gy in range(y0, y1 + 1):
            grid.setdefault((gx, gy), []).append(ci)


def sphere_obb_dist(c, px, py, pz):
    """world distance from point to instance's local-space box (approx exact
    under uniform scale; conservative enough for radius gating)."""
    lp = ML.inverse_transform_location(c["xf"], unreal.Vector(px, py, pz))
    qx = min(max(lp.x, c["lmin"].x), c["lmax"].x)
    qy = min(max(lp.y, c["lmin"].y), c["lmax"].y)
    qz = min(max(lp.z, c["lmin"].z), c["lmax"].z)
    wq = ML.transform_location(c["xf"], unreal.Vector(qx, qy, qz))
    return math.dist((px, py, pz), (wq.x, wq.y, wq.z)), (wq.x, wq.y, wq.z)


offend = {}       # (path, ci) -> dict
near_trace = {}   # same but traceable SMAs (review only)
for pname, pts in paths.items():
    rad = CAM_RAD if pname == "camera" else RAD
    for f, (px, py, pz) in enumerate(pts):
        key = (int(px // CELL), int(py // CELL))
        seen = set()
        for dgx in (-1, 0, 1):
            for dgy in (-1, 0, 1):
                for ci in grid.get((key[0] + dgx, key[1] + dgy), ()):
                    if ci in seen:
                        continue
                    seen.add(ci)
                    c = cands[ci]
                    # cheap world-AABB reject
                    if (px < c["wmin"][0] - rad or px > c["wmax"][0] + rad or
                            py < c["wmin"][1] - rad or py > c["wmax"][1] + rad or
                            pz < c["wmin"][2] - rad or pz > c["wmax"][2] + rad):
                        continue
                    d, wq = sphere_obb_dist(c, px, py, pz)
                    if d >= rad:
                        continue
                    book = near_trace if c["traceable"] else offend
                    e = book.setdefault((pname, ci), dict(
                        path=pname, kind=c["kind"], actor=c["label"],
                        mesh=c["mesh"], inst=c["inst"],
                        top_z=c["wmax"][2], frames=[], min_d=1e9,
                        loc=[round(c["xf"].translation.x, 1),
                             round(c["xf"].translation.y, 1),
                             round(c["xf"].translation.z, 1)]))
                    e["frames"].append(f)
                    if d < e["min_d"]:
                        e["min_d"] = round(d, 1)
                        e["closest_world"] = [round(v, 1) for v in wq]
                        e["path_z_at_min"] = round(pz, 1)

    P(f"[scout] {pname}: done ({len(pts)} frames)")


def pack(book):
    out = []
    for e in book.values():
        fr = e.pop("frames")
        # contiguous ranges
        rngs = []
        s = fr[0]
        for i in range(1, len(fr)):
            if fr[i] != fr[i - 1] + 1:
                rngs.append([s, fr[i - 1]])
                s = fr[i]
        rngs.append([s, fr[-1]])
        e["frame_ranges"] = rngs
        e["n_frames"] = len(fr)
        out.append(e)
    out.sort(key=lambda e: e["min_d"])
    return out


res = dict(keys=KEYS_JSON, rad=RAD, cam_rad=CAM_RAD,
           offenders=pack(offend), traceable_near=pack(near_trace))
json.dump(res, open(OUT_JSON, "w"), indent=1)
P(f"[scout] OFFENDERS (non-traceable, r<{RAD}cm): {len(res['offenders'])}")
for e in res["offenders"][:20]:
    P(f"   {e['path']}: {e['mesh']} ({e['kind']} {e['actor']}#{e['inst']}) "
      f"min_d={e['min_d']}cm frames={e['frame_ranges']} top_z={e['top_z']:.0f} "
      f"path_z@min={e['path_z_at_min']}")
P(f"[scout] traceable near-miss: {len(res['traceable_near'])}")
for e in res["traceable_near"][:10]:
    P(f"   {e['path']}: {e['mesh']} min_d={e['min_d']}cm frames={e['frame_ranges']}")
P("[scout] SCOUT_OK")
