"""Halved-joint stay bodies: open-sided rebates, and 45-degree bevels.

The frame stay parts (seatstay/chainstay pairs) stacked three unknowns:

1. A REBATE running out through the part edge has no closed mouth on the
   machined face — its opening is a notch in the OUTER wire — so mouth
   matching stranded it and the whole part was refused. Stranded floors now
   retry through the floors-mode regardless of fragmentation.
2. A 45-degree PLANAR face (a long scarf bevel on the rebate wall) is neither
   a plate face, a wall, nor freeform — it was invisible and would have been
   left as a square shoulder. Angled planar faces now become SURFACE regions:
   drop-cutter roughing plus ball finish, with raster lanes clipped to the
   region (a diagonal bevel's bounding box spans the whole part, and
   unclipped lanes skimmed off the sheet).
3. Both faces carry blind features, so the flip-jig runs — and the far side's
   bevels must ride into the SIDE 2 program with the far pockets.
"""

import re
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))
sys.path.insert(0, str(ROOT / "cam" / "router_cam"))

pytest.importorskip("OCP", reason="OpenCascade not installed")

T = 15.0


def _box(x, y, z, dx, dy, dz):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def _cut(a, b):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    return BRepAlgoAPI_Cut(a, b).Shape()


def _stay():
    """200x60x15 bar: a 7.5-deep x 25-wide rebate along the top edge (open to
    the edge and both ends), a 45-degree bevel wedge on the rebate wall, and a
    O12 x 2.5 recess in the BOTTOM face -> two-sided."""
    import math
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Pnt, gp_Ax1, gp_Ax2, gp_Dir, gp_Trsf
    s = _box(0, 0, 0, 200, 60, T)
    # rebate: open along y=60 edge, full length
    s = _cut(s, _box(-1, 35, 7.5, 202, 26, 8.5))
    # 45-degree bevel on the rebate wall: a square prism centred on the wall's
    # top corner line, rotated 45 degrees about X, cuts a planar 45 face
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0, 35, 15.0), gp_Dir(1, 0, 0)), math.radians(45))
    wedge = BRepBuilderAPI_Transform(
        _box(-1, 35 - 6.0, 15.0 - 6.0, 202, 12.0, 12.0), tr, True).Shape()
    s = _cut(s, wedge)
    # far-side recess
    s = _cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(60, 15, -1), gp_Dir(0, 0, 1)), 6.0, 3.5).Shape())
    return s


def test_open_rebate_and_bevel_read_completely():
    from backend.services import wood_geometry as WG
    import collections
    g = WG.part_geometry(_stay(), "stay")
    roles = collections.Counter(lp["role"] for lp in g["loops"])
    assert g["unmatched_pockets"] == 0, "the open-sided rebate stranded its floor"
    assert roles.get("POCKET", 0) >= 1, roles
    assert roles.get("SURFACE", 0) >= 1, "the 45-degree bevel was not surfaced"
    surf = next(lp for lp in g["loops"] if lp["role"] == "SURFACE")
    assert surf.get("zmap") is not None and surf.get("mask") is not None
    # rebate floor sits at 7.5: it must be a pocket at that depth
    depths = [round(lp.get("depth", 0), 1) for lp in g["loops"] if lp["role"] == "POCKET"]
    assert 7.5 in depths, depths
    assert g["two_sided"], "the far-side recess makes this a flip-jig part"


def _half_lap_bar():
    """200x35x15 bar with a half-lap at EACH end on OPPOSITE faces — end A cut
    from the top, end B cut from the bottom — and a bolt hole through each lap.
    Neither main face is the full footprint: the top face is missing end A,
    the bottom face end B. The real seatstay bodies are built exactly like
    this, and the profile cut along "the biggest face" severed a lap (and its
    bolt holes) clean off the part."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = _box(0, 0, 0, 200, 35, T)
    s = _cut(s, _box(180, -1, 7.5, 21, 37, 8.5))        # lap A: top half gone
    s = _cut(s, _box(-1, -1, -1, 21, 37, 8.5))          # lap B: bottom half gone
    for x in (10, 190):
        s = _cut(s, BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(x, 17.5, -1), gp_Dir(0, 0, 1)), 4.0, T + 2).Shape())
    return s


def test_half_lap_footprint_is_the_union_of_both_faces():
    """Laps on opposite faces at different ends: the outline must cover the
    WHOLE bar, not just the larger face, and the far-face read (which shapes
    the flip mould) must agree — or pass 1 cuts across the lap and the mould
    is built expecting the severed part."""
    from backend.services import wood_geometry as WG
    for side in (None, "TOP", "BOT"):
        g = WG.part_geometry(_half_lap_bar(), "halflap", force_side=side)
        out = next(lp for lp in g["loops"] if lp["role"] == "OUTSIDE")
        xs = [q[0] for q in out["pts"]]
        ys = [q[1] for q in out["pts"]]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        assert {round(w), round(h)} == {200, 35}, (side, w, h)
        # every feature stays INSIDE the outline bbox — the severed-lap bug
        # left the lap pocket and its bolt holes hanging outside it
        for lp in g["loops"][1:]:
            for q in lp["pts"]:
                assert min(xs) - 0.1 <= q[0] <= max(xs) + 0.1, (side, lp["role"])
                assert min(ys) - 0.1 <= q[1] <= max(ys) + 0.1, (side, lp["role"])
        assert g["two_sided"]
        depths = [round(lp.get("depth", 0), 1)
                  for lp in g["loops"] if lp["role"] == "POCKET"]
        assert depths.count(7.5) == 1, depths          # this side's lap only


def test_diagonal_part_reads_axis_aligned():
    """A part exported at an angle used to READ at that angle, and the nester
    only turns in quarter turns — so it nested diagonally across the sheet.
    The projection basis now rotates so the outline is axis-aligned."""
    import math
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Pnt, gp_Ax1, gp_Dir, gp_Trsf
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(40))
    s = BRepBuilderAPI_Transform(_half_lap_bar(), tr, True).Shape()
    from backend.services import wood_geometry as WG
    g = WG.part_geometry(s, "diagonal")
    out = next(lp for lp in g["loops"] if lp["role"] == "OUTSIDE")
    xs = [q[0] for q in out["pts"]]
    ys = [q[1] for q in out["pts"]]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    assert (round(w), round(h)) in ((200, 35), (35, 200)), (w, h)


def test_stay_generates_two_gated_programs_with_ball_work():
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS, ToolTable
    import dataclasses
    # a surfacing job: the ball nose IS fitted, so T01 is ticked on (it ships
    # unticked — see DEFAULT_TOOL_TABLE)
    tools = ToolTable([dataclasses.replace(s, loaded=True) if s.kind == "ballnose" else s
                       for s in DEFAULT_TOOL_TABLE.seats])
    part = types.SimpleNamespace(name="stay", qty=1, thickness_mm=T,
                                 shape=_stay(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"stay"})
    assert diag[0]["status"] == "ok", diag[0]["reason"]
    res = generate_from_parts(parts, diag, "stay-test",
                              DEFAULT_MATERIALS[0], tools,
                              sheet_w=1220, sheet_h=2440, border_mm=20.0)
    kinds = [(bool(p.get("side2")), p["manifest"]["safety_gate"]) for p in res.programs]
    assert kinds == [(False, "PASS"), (True, "PASS")], \
        [(k, p["manifest"].get("safety_gate_reasons")) for k, p in zip(kinds, res.programs)]
    # the near-side bevel is ball-finished in pass 1 (T01 appears)
    p1 = res.programs[0]["nc"]
    assert re.search(r"T0?1M06", p1), "no ball toolchange in pass 1"
    # raster lanes stay clipped: nothing left of the border region
    for prog in res.programs:
        xs, ys = [], []
        for ln in prog["nc"].splitlines():
            mx = re.search(r"X(-?[\d.]+)", ln); my = re.search(r"Y(-?[\d.]+)", ln)
            if mx: xs.append(float(mx.group(1)))
            if my: ys.append(float(my.group(1)))
        assert min(xs) > 0 and min(ys) > 0, "toolpath left the sheet"
