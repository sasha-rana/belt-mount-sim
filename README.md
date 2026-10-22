# Lap / anti-sub belt mount: FreeCAD model + MuJoCo pull test

Two separate eyebolt mounts, 150 mm apart on the bulkhead, so each one needs 15 kN instead of 30 kN. The stack is the one in the plan: spreader Ø55→Ø30 × 10, Al bushing Ø26 × 12 inside a syntactic annulus Ø55 × 12 replacing the core, backing plate Ø55 × 3, M12 eyebolt and nyloc nut.

<p align="center"><img src="docs/pull-test.gif" width="560" alt="MuJoCo pull test: 15 kN normal, then 15 kN at 30 degrees"></p>

## Pipeline

Same loop as the Desktop-Robot-Arm project: FreeCAD → STL → MuJoCo → analysis.

```bash
python3 -m venv .venv && .venv/bin/pip install mujoco numpy pillow trimesh

FC=/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd
(cd freecad && $FC build_mount.py)        # BeltMount.FCStd, build/stl/*.stl, build/parts.json (masses)
cd mujoco
../.venv/bin/python build_mjcf.py         # belt_mount.xml
../.venv/bin/python analyze.py            # build/report.md + build/results.json (about 2 min)
../.venv/bin/mjpython view.py             # live viewer, rule test looping (--case shared|belt30)
../.venv/bin/python view.py --record ../docs/pull-test.gif
```

Every dimension lives in `freecad/mount_model.py`, and both FreeCAD and MuJoCo read it. To change a part, edit it there and rerun the four commands.

| File | Purpose |
| --- | --- |
| `freecad/mount_model.py` | Dimensions, stack heights, materials |
| `freecad/build_mount.py` | Builds the FreeCAD document, exports STLs and part masses |
| `mujoco/build_mjcf.py` | MuJoCo model: rigid parts, contact-pad rings, bolt spring, plug weld |
| `mujoco/pull_test.py` | Preload, ramp the belt load, hold, read every interface force |
| `mujoco/analyze.py` | Turns those forces into stresses and safety factors |
| `mujoco/view.py` | Viewer and GIF recorder, with parts shaded by utilisation |

## How stresses come out of a rigid-body engine

MuJoCo has no stress solver, so the work is split:

1. **MuJoCo solves the load path.** Each part is a rigid body. Every bearing interface (spreader on skin, collar in the counterbore, nut on the plate, plate on skin) is a ring of contact pads with friction. The eyebolt and nut sit on a slide joint whose spring is the bolt's EA/L, preloaded through `springref`. The panel plug (skins + bushing + annulus) is welded to the panel, so its weld force is the punch-through load. Pads over the Al bushing are about 5× stiffer than pads over the syntactic foam, so MuJoCo splits the plate load between them. That split comes out at 83% / 17%.
2. **`analyze.py` applies the stress formulas**: shank von Mises at the thread root (axial + bending + shear from a free body of the bolt), nut thread stripping, overhang bending of the backing plate and spreader, skin punch-through shear (Ø55 perimeter, and the SES's 130 mm perimeter), bearing on the skin, and bushing compression.

The masses are set to 10 t so the stiff contacts are stable at a 20 µs step. The test is quasi-static, so the forces don't depend on mass. The ramp has a small dynamic transient when the spreader lifts off, and the report uses the settled values.

## Results

See `build/report.md` for the full tables.

## Limitations

- No FEA. Stress concentrations (thread roots, the spreader's counterbore corner, the eye) are not resolved. Use this to rank risks and size parts, then confirm with FreeCAD FEM (CalculiX) on the backing plate and the physical pull test.
- For off-normal loads, the split between the spreader, the backing plate and the pads is statically indeterminate in a rigid model, so those rows are indicative. The shank bending is statics and is reliable.
- The syntactic foam modulus (3 GPa), the CFRP bearing allowable (300 MPa) and the 30° belt angle are assumptions. The skin shear allowable (61 MPa) comes from the SES.
- The eyebolt is a generic long-shank M12. Swap in the real part's dimensions and rating once it's chosen.
