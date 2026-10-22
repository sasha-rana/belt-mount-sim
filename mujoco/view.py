# Viewer and GIF recorder for the belt-mount pull test.
#
#   mjpython view.py                    # live viewer looping the rule test (15 kN normal)
#   mjpython view.py --case belt30      # ... a 30 deg belt pull
#   python view.py --record pull.gif    # renders the rule test and the 30 deg pull off-screen
#
# Contact-force arrows show the load path. The eyebolt shades from green to red
# with shank stress over yield (8.8 steel), and the plug with skin shear over the
# SES skin shear strength.

import argparse
import os
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from analyze import SES_SKIN_SHEAR, STEEL, evaluate
from pull_test import CASES, PullTest, load_vector, M

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_DT = 0.008


def heat(u):
    """Green (0) -> yellow -> red (>= 1) for a utilisation ratio."""
    u = float(np.clip(u, 0, 1))
    return (min(1, 2 * u), min(1, 2 * (1 - u)), 0.15, 1)


class Scene:
    def __init__(self):
        self.pt = PullTest()
        m = self.pt.model
        self.geoms = {name: [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(name).id and m.geom_group[g] == 1]
                      for name in ("lap_head", "lap_plug", "antisub_head", "antisub_plug")}
        self.opt = mujoco.MjvOption()
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        # See-through clamp parts so the contact-force arrows inside the stack show
        for g in range(m.ngeom):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            name = m.mesh(m.geom_dataid[g]).name
            if any(p in name for p in ("spreader", "backing", "annulus", "bushing")):
                m.geom_rgba[g][3] = 0.35
            elif "skin" in name or name == "core":
                m.geom_rgba[g][3] = 0.15

    def shade(self):
        """Colour the eyebolts and plugs by utilisation; returns the lap mount's numbers."""
        f = self.pt.forces()
        out = {}
        for k in M.MOUNTS:
            r = evaluate(f[k])
            for g in self.geoms[k + "_head"]:
                self.pt.model.geom_rgba[g][:3] = heat(r["bolt_vm_MPa"] / STEEL["8.8 steel"][0])[:3]
            for g in self.geoms[k + "_plug"]:
                self.pt.model.geom_rgba[g][:3] = heat(r["punch_MPa"] / SES_SKIN_SHEAR)[:3]
            out[k] = r
        return out["lap"]


def camera():
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (0, 0, 0.018)
    cam.distance, cam.azimuth, cam.elevation = 0.36, 105, -14
    return cam


def caption(img, case, scale, r):
    desc, load, tilt, _ = CASES[case]
    d = ImageDraw.Draw(img)
    lines = ["%s" % desc,
             "belt load %5.1f kN   bolt tension %5.1f kN" % (load * scale / 1e3, r["tension_N"] / 1e3),
             "shank %4.0f MPa (yield 640)   skin shear %4.1f MPa (SES 61)" % (r["bolt_vm_MPa"], r["punch_MPa"])]
    d.rectangle((0, 0, img.width, 74), fill=(255, 255, 255))
    for i, s in enumerate(lines):
        d.text((12, 8 + 21 * i), s, fill=(20, 20, 20))
    return img


def record(path, cases):
    s = Scene()
    m = s.pt.model
    renderer = mujoco.Renderer(m, 480, 640)
    cam = camera()
    frames = []

    for case in cases:
        _, load, tilt, heading = CASES[case]

        def grab(t, scale):
            r = s.shade()
            renderer.update_scene(s.pt.data, cam, s.opt)
            frames.append(caption(Image.fromarray(renderer.render()), case, scale, r))
        s.pt.run({k: load_vector(load, tilt, heading) for k in M.MOUNTS}, grab, FRAME_DT)
        frames += [frames[-1]] * 20
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=60, loop=0, optimize=True)
    print("wrote %s (%d frames)" % (path, len(frames)))


def live(case):
    import mujoco.viewer
    s = Scene()
    _, load, tilt, heading = CASES[case]
    with mujoco.viewer.launch_passive(s.pt.model, s.pt.data) as v:
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        cam = camera()
        v.cam.lookat[:], v.cam.distance, v.cam.azimuth, v.cam.elevation = cam.lookat, cam.distance, cam.azimuth, cam.elevation

        def show(t, scale):
            r = s.shade()
            v.set_texts((None, None, "load %.1f kN" % (load * scale / 1e3),
                         "bolt %.1f kN, shank %.0f MPa" % (r["tension_N"] / 1e3, r["bolt_vm_MPa"])))
            v.sync()
            time.sleep(0.02)
        while v.is_running():
            s.pt.run({k: load_vector(load, tilt, heading) for k in M.MOUNTS}, show, FRAME_DT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="rule", choices=list(CASES))
    ap.add_argument("--record")
    a = ap.parse_args()
    if a.record:
        record(a.record, ["rule", "belt30"])
    else:
        live(a.case)


if __name__ == "__main__":
    main()
