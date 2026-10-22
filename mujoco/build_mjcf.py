# Writes belt_mount.xml: the two belt mounts from the FreeCAD model as a
# quasi-static pull test.
#
#   python build_mjcf.py [--stl-dir ../build/stl] [--out belt_mount.xml]
#
# MuJoCo is a rigid-body engine, so it doesn't give stresses directly. This model
# gives the load path. Every load-bearing interface is a ring of contact pads,
# so the simulation reports how the belt load splits between the parts and how
# it spreads around each ring. analyze.py turns those forces into stresses.
#
# Per mount:
#   plug      the panel disc that carries the insert (skins + bushing + annulus).
#             It is welded to the panel with a compliant weld, and the weld
#             force is the punch-through load on the skin.
#   spreader  clamped between the eyebolt collar and the outer skin
#   eyebolt   free body loaded at the eye centre; the nut is on a slide joint
#             whose spring is the bolt's axial stiffness, preloaded by springref
#   backing   pulled into the inner skin by the nut
#
# Masses are set to 10 t so the stiff contacts stay stable at a usable
# timestep. The test is quasi-static, so the mass doesn't change the forces.

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "freecad"))
import mount_model as M  # noqa: E402

MM = 0.001
PADS = 12                  # contact pads per ring
K_BOLT = 200e9 * M.BOLT["As"] * 1e-6 / ((M.Z_CB - M.Z_NUT_TOP) * MM)  # EA/L, N/m
# Shank bending stiffness EI/L (N·m/rad) about the top of the panel, L = panel + backing plate
K_BEND = 200e9 * math.pi * (M.BOLT["d_minor"] * MM) ** 4 / 64 / ((M.PANEL + M.BACKING["t"]) * MM)
PRELOAD = 8000.0           # N; nyloc on an eyebolt, ~25 N·m
# Explicit <pair> contacts don't inherit geom friction/solref, so they're set on each pair.
PAIR = 'solref="-%.4g -%.4g" solimp="0.99 0.99 0.0001" friction="0.3 0.3 0.005 0.0001 0.0001" condim="3"'
K_PAD = 2e8                # pad stiffness over the aluminium bushing (MuJoCo units)
# The skins are carried by the aluminium bushing (r 6.75-13 mm) and the syntactic
# annulus (r 13-27.5 mm) side by side. The pads over the annulus are softer by the
# ratio of their column stiffness E*A/h, so MuJoCo splits the load between them.
E_ALU, E_SYNTACTIC = 70e3, 3e3   # MPa; syntactic foam modulus is an estimate
K_RATIO_ANNULUS = (E_SYNTACTIC * (M.ANNULUS["d"] ** 2 - M.BUSHING["d"] ** 2)) / (
    E_ALU * (M.BUSHING["d"] ** 2 - M.HOLE ** 2))
WELD = 'solref="-1e9 -6e4" solimp="0.99 0.99 0.0001"'
INERTIA = 'mass="1e4" diaginertia="100 100 100"'

PART_RGBA = {
    "spreader": "0.75 0.77 0.80 1", "bushing": "0.65 0.68 0.72 1", "annulus": "0.85 0.80 0.62 1",
    "backing": "0.70 0.72 0.76 1", "eyebolt": "0.17 0.42 0.30 1", "nut": "0.35 0.35 0.38 1",
    "inner-skin": "0.12 0.12 0.13 0.55", "core": "0.80 0.78 0.55 0.18", "outer-skin": "0.12 0.12 0.13 0.55",
}


def f(*v):
    return " ".join("%.6g" % x for x in v)


def pair(g1, g2, ratio=1.0):
    k = K_PAD * ratio
    return '<pair geom1="%s" geom2="%s" %s/>' % (g1, g2, PAIR % (k, 2 * math.sqrt(k)))


def ring(prefix, r, z, half, cls, extra=""):
    """PADS boxes on a circle of radius r (mm) at height z (mm), tangential."""
    out = []
    for i in range(PADS):
        a = 2 * math.pi * i / PADS
        out.append('<geom name="%s%d" class="%s" type="box" size="%s" pos="%s" euler="0 0 %.6g" %s/>'
                   % (prefix, i, cls, f(*(h * MM for h in half)),
                      f(r * math.cos(a) * MM, r * math.sin(a) * MM, z * MM), a, extra))
    return out


def mesh_geom(name, x0):
    return '<geom class="visual" mesh="%s" pos="%s" rgba="%s"/>' % (
        name, f(-x0 * MM, 0, 0), PART_RGBA.get(name, PART_RGBA.get(name.split("-", 1)[-1])))


# Pad rings under the spreader and on the backing plate: one over the bushing, one over the annulus
R_IN = (M.HOLE / 2 + M.BUSHING["d"] / 2) / 2
R_OUT = (M.BUSHING["d"] / 2 + M.BACKING["d"] / 2) / 2
PAD_IN = (2.5, 2.2, 0.5)     # half-sizes (radial, tangential, thickness), mm
PAD_OUT = (6.0, 4.5, 0.5)


def mount(m, x0):
    t = 0.5  # pad half-thickness, mm
    body = []
    pairs = []
    hole_r = M.HOLE / 2
    r_out = M.BACKING["d"] / 2

    # plug: welded to the panel; flat faces for the pads, a ring of wall pads for the shank
    body += ['<body name="%s_plug" pos="%s">' % (m, f(x0 * MM, 0, 0)), '<freejoint/>',
             '<inertial pos="0 0 0" %s/>' % INERTIA,
             mesh_geom(m + "-bushing", x0), mesh_geom(m + "-annulus", x0),
             '<geom name="%s_plug_top" class="face" type="box" size="%s" pos="%s"/>'
             % (m, f(r_out * MM, r_out * MM, t * MM), f(0, 0, (M.Z_OUTER - t) * MM)),
             '<geom name="%s_plug_bot" class="face" type="box" size="%s" pos="%s"/>'
             % (m, f(r_out * MM, r_out * MM, t * MM), f(0, 0, t * MM))]
    body += ring(m + "_hole", hole_r + 1.0, M.PANEL / 2, (1.0, 1.8, M.PANEL / 2), "face")
    body.append('</body>')

    # spreader: pads on its base, a flat floor in the counterbore for the collar
    body += ['<body name="%s_spreader" pos="%s">' % (m, f(x0 * MM, 0, 0)), '<freejoint/>',
             '<inertial pos="0 0 %.6g" %s/>' % (M.Z_OUTER * MM, INERTIA), mesh_geom(m + "-spreader", x0),
             '<geom name="%s_sp_floor" class="face" type="box" size="%s" pos="%s"/>'
             % (m, f(11 * MM, 11 * MM, t * MM), f(0, 0, (M.Z_CB - t) * MM))]
    body += ring(m + "_spi", R_IN, M.Z_OUTER + t, PAD_IN, "pad")
    body += ring(m + "_spo", R_OUT, M.Z_OUTER + t, PAD_OUT, "pad")
    # counterbore wall: pads facing the collar, so a side load can bear on it
    cb_r, cb_h = M.SPREADER["cb_d"] / 2, M.SPREADER["cb_h"]
    body += ring(m + "_wall", cb_r + t, M.Z_CB + cb_h / 2, (t, 2 * math.pi * cb_r / PADS / 2, cb_h / 2), "pad")
    body.append('</body>')

    # eyebolt: the shank body carries the nut on the bolt's axial spring; the head
    # (collar + eye) sits on two hinges at the top of the panel whose stiffness is
    # the shank's bending stiffness, so a side load tips the collar into the counterbore wall
    r_col = (hole_r + M.BOLT["collar_d"] / 2) / 2
    skirt_h = min(M.BOLT["collar_h"], M.SPREADER["cb_h"])
    body += ['<body name="%s_bolt" pos="%s">' % (m, f(x0 * MM, 0, 0)), '<freejoint/>',
             '<inertial pos="0 0 %.6g" %s/>' % (M.Z_OUTER / 2 * MM, INERTIA),
             '<geom name="%s_shank" class="face" type="capsule" size="%s" fromto="%s"/>'
             % (m, f(M.BOLT["d"] / 2 * MM), f(0, 0, 1 * MM, 0, 0, (M.PANEL - 1) * MM)),
             '<body name="%s_head">' % m]
    for axis in ("1 0 0", "0 1 0"):
        body.append('<joint type="hinge" axis="%s" pos="%s" stiffness="%.6g" damping="%.6g"/>'
                    % (axis, f(0, 0, M.Z_OUTER * MM), K_BEND, 2 * math.sqrt(K_BEND * 100)))
    body += ['<inertial pos="0 0 %.6g" %s/>' % (M.Z_CB * MM, INERTIA), mesh_geom(m + "-eyebolt", x0),
             '<site name="%s_eye" pos="%s" size="0.003"/>' % (m, f(0, 0, M.Z_EYE * MM)),
             '<geom name="%s_skirt" class="face" type="cylinder" size="%s" pos="%s"/>'
             % (m, f(M.BOLT["collar_d"] / 2 * MM, skirt_h / 2 * MM), f(0, 0, (M.Z_CB + skirt_h / 2) * MM))]
    body += ring(m + "_col", r_col, M.Z_CB + t, (1.2, 2.0, t), "pad")
    body += ['</body>',
             '<body name="%s_nut">' % m,
             '<joint name="%s_bolt" type="slide" axis="0 0 1" stiffness="%.6g" springref="%.6g" damping="%.6g"/>'
             % (m, K_BOLT, PRELOAD / K_BOLT, 2 * math.sqrt(K_BOLT * 1e4)),
             '<inertial pos="0 0 %.6g" %s/>' % ((M.Z_NUT_TOP - 6) * MM, INERTIA), mesh_geom(m + "-nut", x0)]
    body += ring(m + "_nut", (M.BOLT["d"] / 2 + M.NUT["af"] / 2) / 2, M.Z_NUT_TOP - t, (1.0, 2.0, t), "pad")
    body += ['</body>', '</body>']

    # backing plate: pads on top against the inner skin, a flat face below for the nut
    body += ['<body name="%s_backing" pos="%s">' % (m, f(x0 * MM, 0, 0)), '<freejoint/>',
             '<inertial pos="0 0 %.6g" %s/>' % (-1.5 * MM, INERTIA), mesh_geom(m + "-backing", x0),
             '<geom name="%s_bp_bot" class="face" type="box" size="%s" pos="%s"/>'
             % (m, f(r_out * MM, r_out * MM, t * MM), f(0, 0, (-M.BACKING["t"] + t) * MM))]
    body += ring(m + "_bpi", R_IN, -t, PAD_IN, "pad")
    body += ring(m + "_bpo", R_OUT, -t, PAD_OUT, "pad")
    body.append('</body>')

    for i in range(PADS):
        pairs += [pair("%s_spi%d" % (m, i), m + "_plug_top"),
                  pair("%s_spo%d" % (m, i), m + "_plug_top", K_RATIO_ANNULUS),
                  pair("%s_col%d" % (m, i), m + "_sp_floor"),
                  pair("%s_wall%d" % (m, i), m + "_skirt"),
                  pair("%s_nut%d" % (m, i), m + "_bp_bot"),
                  pair("%s_bpi%d" % (m, i), m + "_plug_bot"),
                  pair("%s_bpo%d" % (m, i), m + "_plug_bot", K_RATIO_ANNULUS),
                  pair(m + "_shank", "%s_hole%d" % (m, i))]
    weld = '<weld name="%s_weld" body1="%s_plug" %s/>' % (m, m, WELD)
    return body, pairs, weld


def build(stl_dir):
    names = ["inner-skin", "core", "outer-skin"] + [
        "%s-%s" % (m, p) for m in M.MOUNTS for p in ("spreader", "bushing", "annulus", "backing", "eyebolt", "nut")]
    bodies, pairs, welds = [], [], []
    for m, x0 in M.MOUNTS.items():
        b, p, w = mount(m, x0)
        bodies += b
        pairs += p
        welds.append(w)
    meshes = ['<mesh name="%s" file="%s" scale="0.001 0.001 0.001"/>' % (n, os.path.join(stl_dir, n + ".stl"))
              for n in names]
    panel = [mesh_geom(n, 0) for n in ("inner-skin", "core", "outer-skin")]
    return "\n".join([
        '<mujoco model="belt_mount">',
        '<compiler angle="radian" autolimits="true"/>',
        '<option timestep="2e-5" gravity="0 0 0" integrator="implicitfast" cone="elliptic" noslip_iterations="0">',
        '<flag contact="enable"/></option>',
        '<visual><global offwidth="1280" offheight="960"/><map force="0.00012"/>'
        '<scale forcewidth="0.04" contactwidth="0.06" contactheight="0.02"/></visual>',
        '<default>',
        '<default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="1" mass="0"/></default>',
        '<default class="pad"><geom contype="0" conaffinity="0" group="3" mass="0" rgba="0.9 0.3 0.2 0.6"/></default>',
        '<default class="face"><geom contype="0" conaffinity="0" group="3" mass="0" rgba="0.2 0.5 0.9 0.3"/></default>',
        '</default>',
        '<asset>', *meshes,
        '<texture name="grid" type="2d" builtin="checker" rgb1=".93 .94 .95" rgb2=".86 .88 .9" width="300" height="300"/>',
        '<material name="grid" texture="grid" texrepeat="8 8"/>', '</asset>',
        '<worldbody>',
        '<light pos="0 -0.3 0.5" dir="0 0.5 -1" diffuse="0.8 0.8 0.8"/>',
        '<light pos="0.3 0.3 0.4" dir="-0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>',
        '<geom type="plane" size="0.4 0.3 0.01" pos="0 0 -0.06" material="grid" contype="0" conaffinity="0"/>',
        *panel, *bodies,
        '</worldbody>',
        '<contact>', *pairs, '</contact>',
        '<equality>', *welds, '</equality>',
        '</mujoco>'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl-dir", default=os.path.join(HERE, "..", "build", "stl"))
    ap.add_argument("--out", default=os.path.join(HERE, "belt_mount.xml"))
    a = ap.parse_args()
    with open(a.out, "w") as fh:
        fh.write(build(os.path.abspath(a.stl_dir)))
    print("wrote %s (bolt stiffness %.3g N/m, preload %.0f N)" % (a.out, K_BOLT, PRELOAD))


if __name__ == "__main__":
    main()
