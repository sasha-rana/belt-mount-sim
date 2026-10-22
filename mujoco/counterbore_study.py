# Does a deeper, tighter counterbore in the spreader let the mount take an
# angled belt? Rebuilds the MuJoCo model for each spreader variant and sweeps
# the belt angle at two preloads.
#
#   python counterbore_study.py        # writes ../build/counterbore_study.md

import os

import build_mjcf
from analyze import BUILD, STEEL, evaluate
from pull_test import M, PullTest, load_vector

# name: (spreader height, counterbore depth, collar height, collar diameter), mm
VARIANTS = {
    "current: 4 mm deep, 0.5 mm gap": (10.0, 4.0, 6.0, 21.0),
    "deep: 9 mm deep, 0.1 mm gap": (15.0, 9.0, 10.0, 21.8),
}
PRELOADS = (8e3, 25e3)
ANGLES = (0, 3, 10, 20, 30)


def build_variant(h, cb_h, collar_h, collar_d):
    M.SPREADER.update(h=h, cb_h=cb_h)
    M.BOLT.update(collar_h=collar_h, collar_d=collar_d)
    M.update_stack()
    path = os.path.join(BUILD, "counterbore_variant.xml")
    with open(path, "w") as fh:
        fh.write(build_mjcf.build(os.path.join(build_mjcf.HERE, "..", "build", "stl")))
    return path


def main():
    sy = STEEL["8.8 steel"][0]
    lines = ["# Counterbore study (15 kN belt, 8.8 eyebolt)", "",
             "Shank = thread root in the panel. Neck = the section between the collar and the eye, "
             "which only the eyebolt itself can carry. SF on 8.8 yield (640 MPa).", "",
             "| Spreader | Preload (kN) | Angle | Wall load (kN) | Shank bending (N·m) | Shank SF "
             "| Neck bending (N·m) | Neck SF |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name, dims in VARIANTS.items():
        path = build_variant(*dims)
        for preload in PRELOADS:
            pt = PullTest(path, preload=preload)
            for angle in ANGLES:
                f = pt.run({k: load_vector(15e3, angle, 0) for k in M.MOUNTS})["lap"]
                r = evaluate(f)
                wall = sum(abs(x[1][0]) + abs(x[1][1]) for x in f["wall"]) / 1e3
                lines.append("| %s | %g | %g° | %.1f | %.1f | %.2f | %.1f | %.2f |" % (
                    name, preload / 1e3, angle, wall, r["bolt_bending_Nmm"] / 1e3, sy / r["bolt_vm_MPa"],
                    r["neck_bending_Nmm"] / 1e3, sy / r["neck_vm_MPa"]))
                print(lines[-1], flush=True)
    with open(os.path.join(BUILD, "counterbore_study.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
