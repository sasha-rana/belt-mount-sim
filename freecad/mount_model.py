# Shared dimensions for the lap / anti-sub belt mount (mm). Both the FreeCAD
# builder and the MuJoCo builder read this file, so edit a dimension here once.
#
# Frame: Z is the panel normal pointing to the belt side (the test load is +Z).
# z = 0 is the underside of the inner skin; X runs along the panel between mounts.

# Panel (from the SES: bulkhead layup, skin-insert-skin)
SKIN = 2.84            # each skin
CORE = 12.0            # 3.1 pcf honeycomb, replaced locally by the insert
PANEL = SKIN + CORE + SKIN
PANEL_X, PANEL_Y = 300.0, 160.0

# Two separate mounts (>= 125 mm apart, so each needs 15 kN instead of 30 kN)
MOUNTS = {"lap": -75.0, "antisub": 75.0}   # x position of each mount

# Parts (from the plan PDF)
HOLE = 13.5                       # clearance hole for M12
SPREADER = dict(d_base=55.0, d_top=30.0, h=10.0, cb_d=22.0, cb_h=4.0)
BUSHING = dict(d=26.0, h=CORE)    # aluminium, carries clamp load through the core
ANNULUS = dict(d=55.0, h=CORE)    # syntactic foam, fills the core cut-out
BACKING = dict(d=55.0, t=3.0)

# M12 steel eyebolt with a long shank (DIN 580's 22 mm thread is too short for
# this stack) and an M12 nyloc nut
BOLT = dict(d=12.0, d_minor=9.85, As=84.3, collar_d=21.0, collar_h=6.0,
            eye_inner=30.0, eye_wire=10.0, neck_d=14.0, below_nut=3.0)
NUT = dict(af=19.0, h=12.0)

# Stack heights. Call update_stack() after changing a dimension at runtime.
def update_stack():
    global Z_OUTER, Z_CB, Z_COLLAR_TOP, EYE_R, Z_EYE, Z_NUT_TOP, Z_BOLT_END
    Z_OUTER = PANEL                                      # top of the outer skin
    Z_CB = Z_OUTER + SPREADER["h"] - SPREADER["cb_h"]    # counterbore floor = collar seat
    Z_COLLAR_TOP = Z_CB + BOLT["collar_h"]
    EYE_R = (BOLT["eye_inner"] + BOLT["eye_wire"]) / 2   # torus centre-line radius
    Z_EYE = Z_COLLAR_TOP + EYE_R + BOLT["eye_wire"] / 2 + 2.0  # centre of the eye
    Z_NUT_TOP = -BACKING["t"]
    Z_BOLT_END = Z_NUT_TOP - NUT["h"] - BOLT["below_nut"]


update_stack()

# Densities, g/cm^3
DENSITY = {"aluminium": 2.81, "steel": 7.9, "syntactic": 0.65, "panel": 1.0}
PART_MATERIAL = {
    "spreader": "aluminium", "bushing": "aluminium", "annulus": "syntactic",
    "backing": "aluminium", "eyebolt": "steel", "nut": "steel",
}
