# Runs HEADLESS in UE 5.6 (KungfuRender project, -nullrhi): per-shot occlusion
# truth for the v6 stitched keys.  READ-ONLY — loads the map, never saves.
#
#   1. cam -> drone line traces (every 5th frame, every drone framed in the
#      shot; s5/s6 trace all four) — the definitive traceable-geometry
#      occlusion check (offender AABBs only cover non-traceable foliage).
#   2. SHOT 1 vertical column: up-traces from ground+50 to cam z+200 along the
#      s1 camera path (crane lives 9-32 m up; nothing may hang above it).
#
# Env: KEYS_JSON (stitched v6 keys), OUT_JSON
import json
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
_PROG = open(W + "/probe_shots_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


KEYS_JSON = os.environ["KEYS_JSON"]
OUT_JSON = os.environ.get("OUT_JSON", W + "/probe_shots_v6.json")
MAP_PATH = os.environ.get("MRQ_MAP_PATH", "/Game/AncientTempleRuins/Levels/L_Showcase_01")
DRONES = ["ghost", "delta", "buckshee", "thunderstrike"]

K = json.load(open(KEYS_JSON))
GZ = K["info"]["road_z"]
cam = K["camera"]
D = K["drones"]
shots = K["info"]["shots"]

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
ok = les.load_level(MAP_PATH)
world = ues.get_editor_world()
P("PROBE_LOADED", MAP_PATH, ok)


def trace(a, b):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(a[0], a[1], a[2]), unreal.Vector(b[0], b[1], b[2]),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        ip = hit.to_tuple()[4]
        return (round(ip.x), round(ip.y), round(ip.z))
    return None


out = {"map": MAP_PATH, "keys": KEYS_JSON, "shots": {}}
for s in shots:
    name, A, B = s["name"], s["start"], s["end"]
    vehs = s.get("framed") or DRONES
    hits = []
    n_rays = 0
    for k in range(A, B, 5):
        c = cam[k]
        for v in vehs:
            p = D[v][k]
            n_rays += 1
            h = trace((c[0], c[1], c[2]), (p[0], p[1], p[2]))
            if h is not None:
                hits.append({"frame": k, "drone": v, "at": h})
    out["shots"][name] = {"rays": n_rays, "hits": hits[:50],
                          "n_hits": len(hits)}
    P(f"PROBE {name}: rays={n_rays} occlusion_hits={len(hits)}"
      + (f" first={hits[0]}" if hits else ""))

# SHOT 1 vertical column
s1 = shots[0]
col_hits = []
n_col = 0
for k in range(s1["start"], s1["end"], 5):
    c = cam[k]
    n_col += 1
    h = trace((c[0], c[1], GZ + 50.0), (c[0], c[1], c[2] + 200.0))
    if h is not None and h[2] > GZ + 250.0:      # ignore ground-level clutter
        col_hits.append({"frame": k, "at": h})
out["s1_column"] = {"cols": n_col, "hits": col_hits[:50], "n_hits": len(col_hits)}
P(f"PROBE s1_column: cols={n_col} hits={len(col_hits)}")

json.dump(out, open(OUT_JSON, "w"), indent=1)
P("PROBE_OK", OUT_JSON)
