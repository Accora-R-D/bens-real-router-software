"""Groove-heavy jigs: fragmented machined faces, and tool-width slots.

A backrest jig — a 15mm board with 105 grooves 4-5mm wide — produced "No wood
part yielded a usable outline". Three separate failures stacked:

1. The grooves SPLIT the machined face into coplanar fragments, so the
   mouth-matching reader (which walks one face and expects every opening as an
   inner wire) matched none of the 105 floors and refused the part. The
   floors-mode built for boss parts reads these now.
2. The refusal's fallback ("cut as through-features only") returned nothing,
   because wood parts never get a flat outline at ingest — only laser parts
   do. The fallback now extracts the outline from the solid.
3. Grooves exactly one cutter wide cannot be area-cleared (no room to ring),
   but they are standard jig practice — cut as a single centreline pass. The
   engine now does that, and only when the width matches the tool: a wider
   tool would oversize the groove, a narrower one undersize it, and both are
   wrong parts, not fallbacks.
"""

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


def _grooved_board():
    """200x120x15 board with three full-width 4mm grooves (6 deep) and one
    5mm groove (10 deep) — the grooves cut the top face into strips."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 120, T).Shape()
    for x in (40.0, 90.0, 140.0):
        g = BRepPrimAPI_MakeBox(gp_Pnt(x, -1, T - 6.0), 4.0, 122, 7.0).Shape()
        s = BRepAlgoAPI_Cut(s, g).Shape()
    g = BRepPrimAPI_MakeBox(gp_Pnt(170.0, -1, T - 10.0), 5.0, 122, 11.0).Shape()
    s = BRepAlgoAPI_Cut(s, g).Shape()
    # a through hole in a strip the walked fragment does not cover — these
    # were silently LOST when a fragmented face was read from its floors
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(115.0, 60.0, -1), gp_Dir(0, 0, 1)), 4.0, T + 2).Shape()).Shape()
    return s


def test_a_fragmented_face_reads_from_its_floors():
    from backend.services import wood_geometry as WG
    g = WG.part_geometry(_grooved_board(), "grooves")
    assert g["unmatched_pockets"] == 0, \
        "grooves splitting the machined face stranded their floors"
    pockets = [lp for lp in g["loops"] if lp["role"] == "POCKET"]
    assert len(pockets) == 4
    out = next(lp for lp in g["loops"] if lp["role"] == "OUTSIDE")
    xs = [q[0] for q in out["pts"]]; ys = [q[1] for q in out["pts"]]
    assert sorted([max(xs) - min(xs), max(ys) - min(ys)]) == \
        pytest.approx([120.0, 200.0], abs=0.5), "outline is a fragment, not the part"
    # the through hole in an unwalked strip is found, exactly ONCE (it shows on
    # both faces of the board — the dedupe must keep a single drill)
    drills = [lp for lp in g["loops"] if lp["role"] == "DRILL"]
    assert len(drills) == 1, f"{len(drills)} drills — hole lost or doubled"
    assert drills[0]["circle"][2] == pytest.approx(4.0, abs=0.05)


def test_slot_centreline_accepts_only_a_matching_tool():
    import cam_syntec as cs
    slot = [(0, 0), (60, 0), (60, 4), (0, 4)]                # 4mm x 60 groove
    line = cs.slot_centreline(slot, 2.0)                     # O4 tool
    assert line is not None
    (x0, y0), (x1, y1) = line
    assert y0 == pytest.approx(2.0, abs=0.05) and y1 == pytest.approx(2.0, abs=0.05)
    assert sorted([x0, x1]) == pytest.approx([2.0, 58.0], abs=0.05)
    assert cs.slot_centreline(slot, 3.0) is None, "an O6 would oversize the groove"
    assert cs.slot_centreline(slot, 1.5) is None, "an O3 would undersize the groove"
    ell = [(0, 0), (60, 0), (60, 4), (10, 4), (10, 40), (0, 40)]
    assert cs.slot_centreline(ell, 2.0) is None, "an L is not a straight slot"


def test_the_grooved_jig_cuts_completely_with_the_default_tools():
    """With the O4 in T07 (workshop-confirmed default), every groove is cut:
    the 4mm ones as centreline slots, the 5mm one ringed — no skips, gate PASS,
    floors at their exact modelled depths."""
    import re
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = types.SimpleNamespace(name="grooves", qty=1, thickness_mm=T,
                                 shape=_grooved_board(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"grooves"})
    assert diag[0]["status"] == "ok", diag[0]["reason"]
    res = generate_from_parts(parts, diag, "groove-test",
                              DEFAULT_MATERIALS[0], DEFAULT_TOOL_TABLE,
                              sheet_w=1220, sheet_h=2440)
    prog = res.programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS"
    assert not [w for w in prog["warnings"] if "SKIP" in w.upper()], prog["warnings"]
    zs = {round(float(m.group(1)), 2) for ln in prog["nc"].splitlines()
          for m in [re.search(r"G01.*?Z(-?[\d.]+)", ln)] if m}
    assert 9.0 in zs, "4mm groove floors (15-6) not cut"
    assert 5.0 in zs, "5mm groove floor (15-10) not cut"


def test_an_unreadable_wood_part_still_yields_its_outline():
    """When the solid read fails outright, the part must still cut its profile
    and through features instead of killing the whole job with 'no wood part
    yielded a usable outline'."""
    from unittest.mock import patch
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    from backend.services import wood_cam_service as WCS
    from backend.services import wood_geometry
    shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 150, 90, 18.0).Shape()
    part = types.SimpleNamespace(name="stubborn", qty=1, thickness_mm=18.0,
                                 shape=shape, part_number="", edges=None)
    with patch.object(wood_geometry, "part_geometry",
                      side_effect=wood_geometry.GeometryError("boom")):
        parts, diag = WCS.cam_parts_and_diag([part], names={"stubborn"})
    assert len(parts) == 1, diag[0]["reason"]
    assert diag[0]["status"] == "ok"
    assert diag[0]["pocket_source"] == "flat outline"
    loops = parts[0][1]
    assert any(lp["role"] == "OUTSIDE" for lp in loops)


def _ringed_board():
    """200x120x15 board with a closed rectangular groove loop (4 wide, 6 deep)
    that never reaches an edge, and a through hole dead on the loop's centre.
    The loop's outer boundary is then an inner wire of the top face, and its
    centroid is over empty air — exactly the Calf jig."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 120, T).Shape()
    for (x, y, dx, dy) in ((40, 20, 124, 4), (40, 100, 124, 4),
                           (40, 20, 4, 84), (160, 20, 4, 84)):
        s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeBox(gp_Pnt(x, y, T - 6.0), dx, dy, 7.0).Shape()).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(102.0, 62.0, -1), gp_Dir(0, 0, 1)), 5.0, T + 2).Shape()).Shape()
    return s


def test_a_groove_loop_round_a_hole_is_not_a_cut_out():
    """The Calf jig: its groove network's outer boundary had a through hole at
    its centroid, one probe there found air, and the whole middle of the jig
    was cut out as a slug. The Thigh jig's centroid fell on a groove floor and
    was fine. A ring enclosing any material is stock, not a bore."""
    from backend.services import wood_geometry as WG
    g = WG.part_geometry(_ringed_board(), "ringed")
    inside = [lp for lp in g["loops"] if lp["role"] == "INSIDE"]
    assert not inside, [len(lp["pts"]) for lp in inside]
    drills = [lp for lp in g["loops"] if lp["role"] == "DRILL"]
    assert len(drills) == 1 and drills[0]["circle"][2] == pytest.approx(5.0, abs=0.05)
    assert g["unmatched_pockets"] == 0
    pockets = [lp for lp in g["loops"] if lp["role"] == "POCKET"]
    assert pockets, "the groove loop's floor was lost"


def test_the_sliced_probe_answers_exactly_as_the_solid_does():
    """The island-or-bore test asks a 2-D slice of the solid instead of the
    3-D classifier (one slice per depth instead of ~6 ms per point — the router
    spent minutes here on the backrest jigs). Same question, same answer: over
    a grid across the grooved, holed board, at a groove depth and near the
    bottom, the slice and the solid agree point for point."""
    from backend.services import wood_geometry as WG
    shape = _ringed_board()
    origin, u, v, normal = (0.0, 0.0, T), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    for drop in (3.0, T - 0.5):
        assert WG._section_region(shape, origin, u, v, normal, drop) is not None
        for i in range(41):
            for j in range(25):
                xy = (-5.0 + i * 5.25, -5.0 + j * 5.25)      # off-grid with the geometry
                assert WG._material_at(shape, origin, u, v, normal, xy, drop) == \
                    WG._solid_below(shape, origin, u, v, normal, xy, drop), (xy, drop)
