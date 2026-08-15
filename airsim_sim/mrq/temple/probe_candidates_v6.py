# Headless UE 5.6 (KungfuRender, -nullrhi): trace the s4/s5 candidate rays
# from probe_requests_v6.json against REAL geometry, report hit counts per
# candidate.  READ-ONLY.
import json
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/temple"
_PROG = open(W + "/probe_cand_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


REQ = json.load(open(W + "/probe_requests_v6.json"))
for _k in ("s4","s5","s2"):
    REQ.setdefault(_k, [])
OUT = os.environ.get("OUT_JSON", W + "/probe_results_v6.json")
MAP_PATH = "/Game/AncientTempleRuins/Levels/L_Showcase_01"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
ok = les.load_level(MAP_PATH)
world = ues.get_editor_world()
P("PC_LOADED", ok)


def trace(a, b):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(a[0], a[1], a[2]), unreal.Vector(b[0], b[1], b[2]),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    return hit is not None and bool(hit.to_tuple()[0])


res = {"s4": [], "s5": [], "s2": []}
TAGS = REQ.get("tags")
for kind in ("s4", "s5", "s2"):
    for cand in REQ[kind]:
        hits = 0
        hitlist = []
        for ri, (a, b) in enumerate(cand["rays"]):
            if trace(a, b):
                hits += 1
                if TAGS and kind == "s4":
                    hitlist.append(TAGS[ri])
        if TAGS and kind == "s4":
            P("HIT_TAGS", json.dumps(hitlist))
        meta = {k: v for k, v in cand.items() if k != "rays"}
        meta["hits"] = hits
        meta["rays"] = len(cand["rays"])
        res[kind].append(meta)
        P(f"PC {kind} {meta}")
json.dump(res, open(OUT, "w"))
P("PC_OK", OUT)
