# Stresses in the belt mount from the MuJoCo interface forces.
#
#   python analyze.py          # runs every pull-test case, writes ../build/report.md and results.json
#
# MuJoCo gives the load on each interface (bolt tension, the force on each
# contact pad, the punch-through load on the panel plug). This script applies
# hand-calc stress formulas to those forces:
#   eyebolt shank  axial + bending + shear at the thread root, von Mises
#   nut threads    stripping shear
#   backing plate  bending of the part outside the bushing (it is clamped over the
#                  bushing; the syntactic-annulus pads load the overhang)
#   spreader       same, at the counterbore wall
#   inner skin     punch-through shear on the backing-plate perimeter (compared with the SES)
#   skin bearing   peak pad pressure under the backing plate and spreader
#   bushing        compression under the bolt load (it carries the clamp through the core)

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "freecad"))
import mount_model as M  # noqa: E402
from build_mjcf import PADS, R_OUT  # noqa: E402
from pull_test import CASES, PullTest, load_vector  # noqa: E402

BUILD = os.path.join(HERE, "..", "build")

# Allowables, MPa: (yield, ultimate)
STEEL = {"8.8 steel": (640, 800), "A286 (AMS 5737)": (655, 965)}
ALU = {"6061-T6": (276, 310), "7050-T7451": (455, 524)}
SES_SKIN_SHEAR = 61.0       # MPa, "skin shear strength (if tested)" in the SES
SES_PERIMETER = 130.0       # mm, backing plate perimeter as entered in the SES
SKIN_BEARING = 300.0        # MPa, typical quasi-isotropic CFRP bearing, unverified
NUT_SHEAR_FRACTION = 0.58   # shear ultimate / tensile ultimate
SWEEP = (0, 3, 10, 20, 30)       # belt angles off the panel normal, degrees
SWEEP_PRELOAD = (8e3, 25e3)       # N: nyloc snug vs. torqued high-strength eyebolt


def overhang_stress(P_out, r_root, r_load, t):
    """Bending stress (MPa) at radius r_root of a plate ring cantilevered from there and
    carrying P_out (N) at radius r_load: moment per unit circumference over t^2/6. mm units."""
    return 6 * P_out * (r_load - r_root) / (2 * math.pi * r_root * t ** 2)


def shank_section(f, z_cut):
    """Axial force, shear and bending moment in the bolt at height z_cut (world, mm),
    from everything that acts on the bolt above the cut."""
    x0 = f["eye"][0]
    c = np.array([x0, 0.0, z_cut * 1e-3])
    F = f["applied"].copy()
    Mo = np.cross(f["eye"] - c, f["applied"])
    # Stored forces act on the face geom: the spreader/plug for collar and hole
    # (the bolt feels the opposite), the bolt's own skirt for the counterbore wall.
    for name, sign in (("collar", -1), ("hole", -1), ("wall", 1)):
        for pos, fw, _ in f[name]:
            if pos[2] * 1e3 > z_cut:
                F += sign * fw
                Mo += np.cross(pos - c, sign * fw)
    return F[2], math.hypot(F[0], F[1]), math.hypot(Mo[0], Mo[1]) * 1e3   # N, N, N·mm


def pad_peak(items, tag=""):
    """Largest pad normal force and the total, with pads summed over their contact points.
    tag selects a pad ring by name (e.g. "_bpo" for the backing plate's outer ring)."""
    per_pad = {}
    for _, fw, g in items:
        if tag in g:
            per_pad[g] = per_pad.get(g, 0.0) + abs(fw[2])
    vals = list(per_pad.values()) or [0.0]
    return max(vals), sum(vals)


def evaluate(f):
    b = M.BOLT
    r = {"tension_N": f["tension"]}
    dm = b["d_minor"]
    Z = math.pi * dm ** 3 / 32
    shank = []
    for z in (M.Z_OUTER, M.SKIN, 0.0):     # thread root at the outer skin, mid-panel, inner skin
        N, V, Mb = shank_section(f, z)
        s = N / b["As"] + Mb / Z
        tau = V / (math.pi * dm ** 2 / 4)
        shank.append((math.sqrt(s ** 2 + 3 * tau ** 2), N, V, Mb))
    vm, N, V, Mb = max(shank)
    r.update(bolt_vm_MPa=vm, bolt_axial_MPa=N / b["As"], bolt_bending_Nmm=Mb, bolt_shear_N=V)

    # Neck between the collar and the eye: only the belt load acts above it,
    # so nothing in the spreader can relieve this section
    N, V, Mb = shank_section(f, M.Z_COLLAR_TOP)
    dn = b["neck_d"]
    s = N / (math.pi * dn ** 2 / 4) + Mb / (math.pi * dn ** 3 / 32)
    r.update(neck_vm_MPa=math.sqrt(s ** 2 + 3 * (V / (math.pi * dn ** 2 / 4)) ** 2), neck_bending_Nmm=Mb)

    d_pitch = b["d"] - 0.6495 * 1.75
    r["nut_strip_MPa"] = f["tension"] / (0.5 * math.pi * d_pitch * M.NUT["h"])

    # Overhangs: worst outer pad scaled to a full ring, so a tilted load shows up as a heavier ring
    r_bush = M.BUSHING["d"] / 2
    peak_out, total_out = pad_peak(f["backing"], "_bpo")
    _, total_bp = pad_peak(f["backing"])
    r["backing_annulus_share"] = total_out / total_bp if total_bp else 0.0
    r["backing_MPa"] = overhang_stress(peak_out * PADS, r_bush, R_OUT, M.BACKING["t"])
    peak_sp, _ = pad_peak(f["spreader"], "_spo")
    cb_r = M.SPREADER["cb_d"] / 2
    r["spreader_MPa"] = overhang_stress(peak_sp * PADS, cb_r, R_OUT, M.SPREADER["h"] - M.SPREADER["cb_h"])

    # Panel plug: net force through the plug = punch-through on the inner skin
    plug = np.zeros(3)
    for name in ("backing", "spreader", "hole"):
        for _, fw, _ in f[name]:
            plug += fw
    r["punch_N"] = plug[2]
    r["punch_MPa"] = plug[2] / (math.pi * M.BACKING["d"] * M.SKIN)
    r["punch_ses_MPa"] = plug[2] / (SES_PERIMETER * M.SKIN)
    # Bearing on the skin over the bushing (inner pads), the stiffest and most loaded spot
    pad_area = math.pi / 4 * (M.BUSHING["d"] ** 2 - M.HOLE ** 2) / PADS
    r["skin_bearing_MPa"] = max(pad_peak(f["backing"], "_bpi")[0], pad_peak(f["spreader"], "_spi")[0]) / pad_area
    r["bushing_MPa"] = f["tension"] / (math.pi / 4 * (M.BUSHING["d"] ** 2 - M.HOLE ** 2))
    return r


def margins(r):
    out = {}
    for name, (sy, su) in STEEL.items():
        out["eyebolt shank, %s" % name] = (r["bolt_vm_MPa"], sy, su)
        out["eyebolt neck, %s" % name] = (r["neck_vm_MPa"], sy, su)
    out["nut threads (8 nut)"] = (r["nut_strip_MPa"], None, NUT_SHEAR_FRACTION * 800)
    for name, (sy, su) in ALU.items():
        out["backing plate, %s" % name] = (r["backing_MPa"], sy, su)
        out["spreader, %s" % name] = (r["spreader_MPa"], sy, su)
        out["bushing compression, %s" % name] = (r["bushing_MPa"], sy, None)
    out["inner skin punch-through (Ø55 perimeter)"] = (r["punch_MPa"], None, SES_SKIN_SHEAR)
    out["inner skin punch-through (SES 130 mm perimeter)"] = (r["punch_ses_MPa"], None, SES_SKIN_SHEAR)
    out["skin bearing under pads"] = (r["skin_bearing_MPa"], None, SKIN_BEARING)
    return out


def main():
    pt = PullTest()
    results, lines = {}, []
    for case, (desc, load, tilt, heading) in CASES.items():
        forces = pt.run({k: load_vector(load, tilt, heading) for k in M.MOUNTS})
        r = evaluate(forces["lap"])      # both mounts see the same load in every case
        results[case] = {"description": desc, **r,
                         "margins": {k: {"stress": s, "yield": y, "ultimate": u}
                                     for k, (s, y, u) in margins(r).items()}}
        if tilt:
            lines += ["", "> Off-normal load: the shank result is statics (the eye's lever arm), but the split "
                      "between spreader, backing plate and pads is indeterminate in a rigid-body model. "
                      "Treat the plate, spreader and bearing rows as indicative only."]
        lines += ["", "## %s" % desc, "",
                  "Bolt tension %.1f kN (applied %.1f kN), shank bending %.1f N·m, shear %.1f kN. "
                  "Share of the backing-plate load on the syntactic annulus: %.0f%%."
                  % (r["tension_N"] / 1e3, load / 1e3, r["bolt_bending_Nmm"] / 1e3, r["bolt_shear_N"] / 1e3,
                     100 * r["backing_annulus_share"]), "",
                  "| Check | Stress (MPa) | SF on yield | SF on failure |", "| --- | ---: | ---: | ---: |"]
        for k, (s, y, u) in margins(r).items():
            sf = lambda a: "–" if a is None or s <= 0 else ("**%.2f**" % (a / s) if a / s < 1 else "%.2f" % (a / s))
            lines.append("| %s | %.0f | %s | %s |" % (k, s, sf(y), sf(u)))
    lines += ["", "## Belt angle sweep (15 kN, 8.8 eyebolt)", "",
              "Preload decides whether the eyebolt's shoulder stays seated on the spreader. "
              "Once the belt load lifts it, the shank is a cantilever from the nut and the eye's lever arm bends it.", "",
              "| Preload (kN) | Angle off normal | Bolt tension (kN) | Shoulder seated | Shank bending (N·m) "
              "| Shank von Mises (MPa) | SF on yield |",
              "| ---: | ---: | ---: | :---: | ---: | ---: | ---: |"]
    sweep = []
    sy = STEEL["8.8 steel"][0]
    for preload in SWEEP_PRELOAD:
        spt = PullTest(preload=preload)
        for angle in SWEEP:
            f = spt.run({k: load_vector(15e3, angle, 0) for k in M.MOUNTS})["lap"]
            r = evaluate(f)
            seated = bool(abs(sum(x[1][2] for x in f["collar"])) > 100)
            sweep.append({"preload": preload, "angle": angle, "seated": seated, **r})
            lines.append("| %g | %g° | %.1f | %s | %.1f | %.0f | %.2f |" % (
                preload / 1e3, angle, r["tension_N"] / 1e3, "yes" if seated else "no",
                r["bolt_bending_Nmm"] / 1e3, r["bolt_vm_MPa"], sy / r["bolt_vm_MPa"]))
    results["sweep"] = sweep
    os.makedirs(BUILD, exist_ok=True)
    with open(os.path.join(BUILD, "results.json"), "w") as fh:
        json.dump(results, fh, indent=1)
    text = "# Belt mount pull test (MuJoCo + hand calcs)\n" + "\n".join(lines) + "\n"
    with open(os.path.join(BUILD, "report.md"), "w") as fh:
        fh.write(text)
    print(text)


if __name__ == "__main__":
    main()
