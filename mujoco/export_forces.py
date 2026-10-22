# Exports the statics of every pull-test case for the browser viewer.
#
#   python export_forces.py        # writes ../build/forces.json (a few minutes)
#
# For each case the belt load is stepped up and held at several fractions, so
# the viewer can show the bolt preload handing over to the belt load. At each
# step the file lists, per mount:
#   applied   the belt load at the eye
#   tension   the bolt's axial force (from the nut's spring)
#   forces    every force between two parts: the part it acts on, the part it
#             comes from, its resultant and centroid, and its per-pad pieces.
#             Each contact appears twice (action and reaction); the reaction is
#             flagged so the overview draws each contact once.
# All points are in mm in the FreeCAD frame; forces are in N.

import json
import os

import mujoco
import numpy as np

from pull_test import CASES, PullTest, load_vector, M

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "build", "forces.json")
FRACTIONS = (0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0)
STEP_RAMP, STEP_HOLD = 0.04, 0.2    # s per load step; long enough for the 10 t bodies to settle
SETTLE_FIRST = 0.3                   # s of preload-only settling before the first step

# Interface -> (part the stored force acts on, part it comes from). pull_test
# stores the force on the interface's face geom.
ACTS_ON = {
    "spreader": ("plug", "spreader"),
    "collar": ("spreader", "eyebolt"),
    "wall": ("eyebolt", "spreader"),
    "nut": ("backing", "nut"),
    "backing": ("plug", "backing"),
    "hole": ("plug", "eyebolt"),
}


def mm(v):
    return [round(float(x) * 1e3, 3) for x in v]


def newtons(v):
    return [round(float(x), 2) for x in v]


def snapshot(pt, x0):
    out = {}
    raw = pt.forces()
    for k in M.MOUNTS:
        f = raw[k]
        forces = []
        plug_sum = np.zeros(3)
        for name, (on, frm) in ACTS_ON.items():
            pads = {}
            for pos, fw, g in f[name]:
                p = pads.setdefault(g, [np.zeros(3), np.zeros(3), 0.0])
                w = np.linalg.norm(fw)
                p[0] += pos * w
                p[1] += fw
                p[2] += w
            pads = [(p[0] / p[2], p[1]) for p in pads.values() if p[2] > 1e-6]
            total = sum((v for _, v in pads), np.zeros(3))
            if on == "plug":
                plug_sum += total
            w = sum(np.linalg.norm(v) for _, v in pads)
            centroid = sum((p * np.linalg.norm(v) for p, v in pads), np.zeros(3)) / w if w > 1e-6 else None
            point = mm(centroid) if centroid is not None else None
            forces.append({"name": name, "on": on, "from": frm, "total": newtons(total), "point": point,
                           "pads": [[mm(p), newtons(v)] for p, v in pads]})
            # Newton's third law: the same contact pushes back on the other part
            forces.append({"name": name, "on": frm, "from": on, "total": newtons(-total), "point": point,
                           "pads": [[mm(p), newtons(-v)] for p, v in pads], "reaction": True})
        # Thread: whatever holds the nut in equilibrium against the backing plate.
        # Its axial part is the bolt tension; the joint also passes side load.
        T = float(f["tension"])
        on_nut = -sum((np.array(x["total"]) for x in forces if x["on"] == "nut"), np.zeros(3))
        nut_pt = [x0[k], 0.0, M.Z_NUT_TOP - M.NUT["h"] / 2]
        forces.append({"name": "thread", "on": "nut", "from": "eyebolt", "total": newtons(on_nut),
                       "point": nut_pt, "pads": []})
        forces.append({"name": "thread", "on": "eyebolt", "from": "nut", "total": newtons(-on_nut),
                       "point": nut_pt, "pads": [], "reaction": True})
        # The panel holds the plug: weld reaction on the plug, and its opposite on the panel
        plug_pt = [x0[k], 0.0, M.PANEL / 2]
        forces.append({"name": "panel", "on": "plug", "from": "panel", "total": newtons(-plug_sum),
                       "point": plug_pt, "pads": []})
        forces.append({"name": "punch-through", "on": "panel", "from": "plug", "total": newtons(plug_sum),
                       "point": plug_pt, "pads": []})
        seated = abs(sum(v[1][2] for v in f["collar"])) > 50 if f["collar"] else False
        out[k] = {"applied": {"point": mm(f["eye"]), "total": newtons(f["applied"])},
                  "tension": round(T, 1), "shoulder_seated": bool(seated), "forces": forces}
    return out


def run_case(pt, load_vec):
    mujoco.mj_resetData(pt.model, pt.data)
    pt.load = load_vec
    d, dt = pt.data, pt.model.opt.timestep
    while d.time < SETTLE_FIRST:
        pt.step(0.0)
    frames, prev = [], 0.0
    x0 = {k: x for k, x in M.MOUNTS.items()}
    for frac in FRACTIONS:
        t0 = d.time
        while d.time < t0 + STEP_RAMP:
            pt.step(prev + (frac - prev) * min((d.time - t0) / STEP_RAMP, 1.0))
        while d.time < t0 + STEP_RAMP + STEP_HOLD:
            pt.step(frac)
        prev = frac
        frames.append({"fraction": frac, "mounts": snapshot(pt, x0)})
        print("  %3.0f%%" % (frac * 100), flush=True)
    return frames


def main():
    pt = PullTest()
    out = {"units": {"length": "mm", "force": "N"}, "cases": {}}
    for case, (desc, load, tilt, heading) in CASES.items():
        print(case, flush=True)
        frames = run_case(pt, {k: load_vector(load, tilt, heading) for k in M.MOUNTS})
        out["cases"][case] = {"description": desc, "load_N": load, "tilt_deg": tilt, "frames": frames}
    with open(OUT, "w") as fh:
        json.dump(out, fh)
    print("wrote %s (%.0f kB)" % (OUT, os.path.getsize(OUT) / 1e3))


if __name__ == "__main__":
    main()
