"""A part the size of the whole sheet is machined without its outer profile.

Part 13 (2026-11-24) was a full 2440 x 1220 MDF panel with two cut-outs and a
row of counterbored holes. The cutter runs one radius outside an outline, off
the board, so the nest skipped it as too big — and the empty program then
crashed the safety gate with "min() iterable argument is empty". Its outside
needs no cut: the sheet's factory edges are its edges.
"""

import os
import re
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")


def _panel(w=2440.0, h=1220.0, notch=False):
    """An 18 mm board with a through cut-out and a through hole; optionally a
    20 x 20 notch in one edge."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), w, h, 18).Shape()
    cuts = [BRepPrimAPI_MakeBox(gp_Pnt(300, 400, -1), 500, 300, 20).Shape(),
            BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(40, 40, -1), gp_Dir(0, 0, 1)), 5, 20).Shape()]
    if notch:
        cuts.append(BRepPrimAPI_MakeBox(gp_Pnt(w / 2, -1, -1), 20, 21, 20).Shape())
    for c in cuts:
        shape = BRepAlgoAPI_Cut(shape, c).Shape()
    return shape


def _run(shape, sheet=(1220, 2440)):
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = types.SimpleNamespace(name="Panel", qty=1, thickness_mm=18.0,
                                 shape=shape, part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"Panel"})
    return generate_from_parts(parts, diag, "full-sheet-test", DEFAULT_MATERIALS[0],
                               DEFAULT_TOOL_TABLE, sheet_w=sheet[0], sheet_h=sheet[1],
                               border_mm=20)


def _extent(nc):
    xs = [float(v) for v in re.findall(r"X(-?[\d.]+)", nc)]
    ys = [float(v) for v in re.findall(r"Y(-?[\d.]+)", nc)]
    return min(xs), max(xs), min(ys), max(ys)


def test_a_sheet_sized_panel_is_machined_without_its_outside():
    prog = _run(_panel()).programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS", prog["manifest"]["safety_gate_reasons"]
    assert any("is the whole 1220x2440 sheet" in w for w in prog["warnings"]), prog["warnings"]
    placed = prog["layout"][0]["parts"]
    assert [p["name"] for p in placed] == ["Panel"]
    p = placed[0]
    assert not p["tabbed"]
    assert len(p["cutouts"]) == 1 and len(p["drills"]) == 1
    # every move on the board: the cut-out and the hole, no profile round the edge
    x0, x1, y0, y1 = _extent(prog["nc"])
    assert 0 < x0 and x1 < 1220 and 0 < y0 and y1 < 2440, (x0, x1, y0, y1)


def test_a_notched_panel_is_not_taken_for_the_sheet():
    """The factory edge cannot cut a notch: a shaped outline still needs its
    profile, so a sheet-sized notched panel stays too big — and says so."""
    prog = _run(_panel(notch=True)).programs[0]
    assert not any("is the whole" in w for w in prog["warnings"])
    assert prog["manifest"]["safety_gate"] == "FAIL"
    assert any("no toolpath" in r for r in prog["manifest"]["safety_gate_reasons"])


def test_a_part_too_big_fails_the_gate_with_a_reason_not_a_crash():
    prog = _run(_panel(2600.0, 1300.0)).programs[0]
    assert prog["manifest"]["safety_gate"] == "FAIL"
    assert any("no toolpath" in r for r in prog["manifest"]["safety_gate_reasons"])
    assert any("too big" in w for w in prog["warnings"]), prog["warnings"]


def test_only_a_match_within_half_a_millimetre_counts():
    from nest import full_sheet_rotation          # as router_cam.engine imports it
    rect = lambda w, h: {"footprint": [(0, 0), (w, 0), (w, h), (0, h)]}
    assert full_sheet_rotation(rect(2440, 1220), 1220, 2440) in (1, 3)
    assert full_sheet_rotation(rect(1220.4, 2439.6), 1220, 2440) == 0
    assert full_sheet_rotation(rect(1219, 2440), 1220, 2440) is None
    assert full_sheet_rotation(rect(2440, 1220), 1220, 2440, rotations=(0, 2)) is None
