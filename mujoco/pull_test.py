# Quasi-static pull test on belt_mount.xml: preload the bolts, ramp a belt load
# onto each eye, hold it, and read back the force on every interface.
#
#   python pull_test.py            # prints the interface forces for each case

import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "freecad"))
import mount_model as M  # noqa: E402

MODEL = os.path.join(HERE, "belt_mount.xml")
SETTLE, RAMP, HOLD = 0.08, 0.30, 0.08   # s

# name: (description, load in N, tilt from the panel normal in degrees, tilt direction in the panel plane)
CASES = {
    "rule": ("15 kN normal to the panel (rule test, mounts >= 125 mm apart)", 15e3, 0, 0),
    "shared": ("30 kN normal (rule load if the mounts were < 125 mm apart)", 30e3, 0, 0),
    "belt30": ("15 kN at 30 deg off normal (belt angle in the car)", 15e3, 30, 0),
}

# Interface name -> (pad geom prefix, face geom suffix).
INTERFACES = {"spreader": ("_sp", "_plug_top"), "collar": ("_col", "_sp_floor"), "wall": ("_wall", "_skirt"),
              "nut": ("_nut", "_bp_bot"), "backing": ("_bp", "_plug_bot"), "hole": ("_shank", "_hole")}


def load_vector(load, tilt, heading):
    t, h = math.radians(tilt), math.radians(heading)
    return load * np.array([math.sin(t) * math.cos(h), math.sin(t) * math.sin(h), math.cos(t)])


class PullTest:
    def __init__(self, model_path=MODEL, preload=None):
        """preload (N) overrides the bolt preload built into the model."""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.load = {m: np.zeros(3) for m in M.MOUNTS}
        self.scale = 0.0
        m = self.model
        self.head = {k: m.body(k + "_head").id for k in M.MOUNTS}
        self.eye = {k: m.site(k + "_eye").id for k in M.MOUNTS}
        self.slide = {k: m.joint(k + "_bolt").dofadr[0] for k in M.MOUNTS}
        if preload is not None:
            for k in M.MOUNTS:
                j = m.joint(k + "_bolt")
                m.qpos_spring[j.qposadr[0]] = preload / j.stiffness[0]

    def step(self, scale):
        d = self.data
        self.scale = scale
        d.qfrc_applied[:] = 0
        for k in M.MOUNTS:
            mujoco.mj_applyFT(self.model, d, self.load[k] * scale, np.zeros(3), d.site_xpos[self.eye[k]],
                              self.head[k], d.qfrc_applied)
        mujoco.mj_step(self.model, d)

    def run(self, load_by_mount, on_frame=None, frame_dt=None):
        """Settle the preload, ramp the load, hold. on_frame(t, scale) is called every frame_dt."""
        mujoco.mj_resetData(self.model, self.data)
        self.load = load_by_mount
        dt = self.model.opt.timestep
        total = SETTLE + RAMP + HOLD
        next_frame = 0.0
        while self.data.time < total:
            t = self.data.time
            scale = min(max((t - SETTLE) / RAMP, 0.0), 1.0)
            self.step(scale)
            if on_frame and self.data.time >= next_frame:
                on_frame(self.data.time, scale)
                next_frame += frame_dt
        return self.forces()

    def bolt_tension(self, k):
        return self.data.qfrc_spring[self.slide[k]]

    def forces(self):
        """Per mount: tension, applied load, and per-interface lists of (position, force on the face side)."""
        m, d = self.model, self.data
        out = {k: {"tension": self.bolt_tension(k), "applied": self.load[k] * self.scale,
                   "eye": d.site_xpos[self.eye[k]].copy(), **{i: [] for i in INTERFACES}} for k in M.MOUNTS}
        f6 = np.zeros(6)
        for c in range(d.ncon):
            con = d.contact[c]
            g1, g2 = m.geom(con.geom1).name, m.geom(con.geom2).name
            mujoco.mj_contactForce(m, d, c, f6)
            # mj_contactForce is in the contact frame; the normal points from geom1 to geom2,
            # so this is the force geom1 puts on geom2. MuJoCo may list either side first.
            fw = con.frame.reshape(3, 3).T @ f6[:3]
            for k in M.MOUNTS:
                for name, (pad, face) in INTERFACES.items():
                    if g1.startswith(k + pad) and k + face in g2:
                        out[k][name].append((con.pos.copy(), fw, g1))
                    elif g2.startswith(k + pad) and k + face in g1:
                        out[k][name].append((con.pos.copy(), -fw, g2))
        return out


def summarise(f):
    rows = {}
    for k, v in f.items():
        tot = {i: np.sum([x[1] for x in v[i]], axis=0) if v[i] else np.zeros(3) for i in INTERFACES}
        rows[k] = (v["tension"], tot)
    return rows


def main():
    pt = PullTest()
    for case, (desc, load, tilt, heading) in CASES.items():
        f = pt.run({k: load_vector(load, tilt, heading) for k in M.MOUNTS})
        print("\n== %s: %s" % (case, desc))
        for k, (tension, tot) in summarise(f).items():
            print("  %-8s bolt %7.0f N | " % (k, tension) +
                  " ".join("%s %s" % (i, np.round(v).astype(int)) for i, v in tot.items()))


if __name__ == "__main__":
    main()
