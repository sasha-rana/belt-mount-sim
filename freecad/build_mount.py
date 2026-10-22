# Builds the belt-mount assembly in FreeCAD from mount_model.py, saves
# BeltMount.FCStd, exports one STL per part to ../build/stl and writes
# ../build/parts.json (volume and mass of every part).
#
#   freecadcmd build_mount.py
#
# To redesign a part, change it in mount_model.py and rerun, or edit it in the
# GUI keeping its Label and rerun export_parts() from the macro console.

import json
import math
import os
import sys

import FreeCAD as App
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mount_model as M  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, ".."))
STL_OUT = os.path.join(ROOT, "build", "stl")
V = App.Vector


def cyl(d, h, z, x=0.0):
    return Part.makeCylinder(d / 2, h, V(x, 0, z))


def ring(d_out, d_in, h, z, x=0.0):
    return cyl(d_out, h, z, x).cut(cyl(d_in, h + 2, z - 1, x))


def spreader(x):
    s = M.SPREADER
    body = Part.makeCone(s["d_base"] / 2, s["d_top"] / 2, s["h"], V(x, 0, M.Z_OUTER))
    body = body.cut(cyl(M.HOLE, 40, M.Z_OUTER - 5, x))
    return body.cut(cyl(s["cb_d"], s["cb_h"] + 1, M.Z_CB, x))


def eyebolt(x):
    b = M.BOLT
    shank = cyl(b["d"], M.Z_CB - M.Z_BOLT_END, M.Z_BOLT_END, x)
    collar = cyl(b["collar_d"], b["collar_h"], M.Z_CB, x)
    neck = cyl(b["neck_d"], M.Z_EYE - M.EYE_R - M.Z_COLLAR_TOP + 2, M.Z_COLLAR_TOP, x)
    eye = Part.makeTorus(M.EYE_R, b["eye_wire"] / 2, V(x, 0, M.Z_EYE), V(0, 1, 0))
    return shank.fuse([collar, neck, eye]).removeSplitter()


def nut(x):
    n = M.NUT
    r = n["af"] / math.sqrt(3)  # hexagon corner radius
    pts = [V(x + r * math.cos(math.radians(60 * i)), r * math.sin(math.radians(60 * i)),
             M.Z_NUT_TOP - n["h"]) for i in range(7)]
    hexa = Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, n["h"]))
    return hexa.cut(cyl(M.BOLT["d"], n["h"] + 2, M.Z_NUT_TOP - n["h"] - 1, x))


def panel():
    box = lambda z, h: Part.makeBox(M.PANEL_X, M.PANEL_Y, h, V(-M.PANEL_X / 2, -M.PANEL_Y / 2, z))
    inner, core, outer = box(0, M.SKIN), box(M.SKIN, M.CORE), box(M.SKIN + M.CORE, M.SKIN)
    for x in M.MOUNTS.values():
        inner = inner.cut(cyl(M.HOLE, 10, -1, x))
        outer = outer.cut(cyl(M.HOLE, 30, -1, x))
        core = core.cut(cyl(M.ANNULUS["d"], 30, -1, x))
    return {"inner-skin": inner, "core": core, "outer-skin": outer}


def parts():
    out = dict(panel())
    for name, x in M.MOUNTS.items():
        out[name + "-spreader"] = spreader(x)
        out[name + "-bushing"] = ring(M.BUSHING["d"], M.HOLE, M.CORE, M.SKIN, x)
        out[name + "-annulus"] = ring(M.ANNULUS["d"], M.BUSHING["d"], M.CORE, M.SKIN, x)
        out[name + "-backing"] = ring(M.BACKING["d"], M.HOLE, M.BACKING["t"], -M.BACKING["t"], x)
        out[name + "-eyebolt"] = eyebolt(x)
        out[name + "-nut"] = nut(x)
    return out


def main():
    os.makedirs(STL_OUT, exist_ok=True)
    doc = App.newDocument("BeltMount")
    report = {}
    for label, shape in parts().items():
        obj = doc.addObject("Part::Feature", label.replace("-", "_"))
        obj.Label = label
        obj.Shape = shape
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.05, AngularDeflection=0.15)
        mesh.write(os.path.join(STL_OUT, label + ".stl"))
        kind = label.split("-", 1)[-1]
        density = M.DENSITY[M.PART_MATERIAL.get(kind, "panel")]
        report[label] = {"volume_cm3": round(shape.Volume / 1000, 3),
                         "mass_g": round(shape.Volume / 1000 * density, 1)}
    doc.recompute()
    doc.saveAs(os.path.join(HERE, "BeltMount.FCStd"))
    with open(os.path.join(ROOT, "build", "parts.json"), "w") as f:
        json.dump(report, f, indent=1)
    for k, v in report.items():
        App.Console.PrintMessage("%-18s %8.2f cm3 %8.1f g\n" % (k, v["volume_cm3"], v["mass_g"]))


if __name__ in ("__main__", "build_mount"):
    main()
