"""Boards with MODELLED edge chamfers: 2.5D, cut with the vee.

A chamfered plate used to confuse the reader three ways at once (Part 13, a
12mm head/footboard panel with a 2mm bevel on every edge of both faces): the
profile was read from the machined face, which the bevel insets, so the part
was cut a chamfer-width undersize all round; the bevels were taken for
sculpted regions and skipped for want of a ball nose; and the far face's
bevels were not seen at all. Now the profile is the board's true through-
boundary, the near bevels are a vee CHAMFER in pass 1, and far bevels send the
part through the flip-jig: pass 1 also cuts a mould, and the SIDE 2 program
chamfers the flipped part in it.
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

W, H, T, C = 300.0, 200.0, 12.0, 2.0
HOLE_R = 10.0


def _board(top=True, bottom=True, hole=True, one_edge=False):
    """W x H x T plate with a O20 through hole; C x 45-degree chamfers on the
    outline (and the hole) at the top and/or bottom face. `one_edge` bevels
    a single top edge only — a partial chamfer."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), W, H, T).Shape()
    if hole:
        cyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(80, 120, -1), gp_Dir(0, 0, 1)),
                                       HOLE_R, T + 2).Shape()
        s = BRepAlgoAPI_Cut(s, cyl).Shape()
    mk = BRepFilletAPI_MakeChamfer(s)
    exp = TopExp_Explorer(s, TopAbs_ShapeEnum.TopAbs_EDGE)
    seen = []
    while exp.More():
        e = TopoDS.Edge_s(exp.Current())
        exp.Next()
        if any(e.IsSame(x) for x in seen):
            continue
        seen.append(e)
        c = BRepAdaptor_Curve(e)
        p0, p1 = c.Value(c.FirstParameter()), c.Value(c.LastParameter())
        if abs(p0.Z() - p1.Z()) > 1e-6:
            continue                                  # a vertical edge
        z = p0.Z()
        if one_edge:
            if abs(z - T) < 1e-6 and abs(p0.Y()) < 1e-6 and abs(p1.Y()) < 1e-6:
                mk.Add(C, e)
            continue
        if (top and abs(z - T) < 1e-6) or (bottom and abs(z) < 1e-6):
            mk.Add(C, e)
    mk.Build()
    assert mk.IsDone()
    return mk.Shape()


def _run(shape, name):
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = types.SimpleNamespace(name=name, qty=1, thickness_mm=T,
                                 shape=shape, part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={name})
    res = generate_from_parts(parts, diag, f"chamfer-{name}",
                              DEFAULT_MATERIALS[0], DEFAULT_TOOL_TABLE,
                              sheet_w=1220, sheet_h=2440)
    return parts, diag, res


def _zs(nc):
    return [float(m.group(1)) for ln in nc.splitlines()
            for m in [re.search(r"G0[123].*?Z([-\d.]+)", ln)] if m]


def _tools(nc):
    return [int(t) for t in re.findall(r"T(\d+)M06", nc)]


@pytest.fixture(scope="module")
def both():
    return _run(_board(), "BothSides")


def test_profile_is_the_true_boundary_not_the_inset_face(both):
    parts, _diag, _res = both
    _n, loops, _meta = parts[0]
    out = next(lp for lp in loops if lp["role"] == "OUTSIDE")
    xs = [q[0] for q in out["pts"]]; ys = [q[1] for q in out["pts"]]
    assert sorted([max(xs) - min(xs), max(ys) - min(ys)]) == \
        pytest.approx([H, W], abs=0.05), "profile cut along the inset chamfer edge"
    hole = next(lp for lp in loops if lp.get("circle"))
    assert hole["circle"][2] == pytest.approx(HOLE_R, abs=0.05), \
        "hole read from the chamfer mouth, not the bore"


def test_bevels_are_chamfers_not_sculpted_regions(both):
    parts, _diag, res = both
    _n, loops, _meta = parts[0]
    assert not [lp for lp in loops if lp["role"] == "SURFACE"]
    tagged = [lp for lp in loops if lp.get("chamfer")]
    assert {lp["role"] for lp in tagged} == {"OUTSIDE", "DRILL"}
    for lp in tagged:
        # both rings are chord-sampled, so a curved bevel reads a few hundredths
        # wide — well inside the 3-degree vee-mismatch warning
        assert lp["chamfer"]["width"] == pytest.approx(C, abs=0.06)
        assert lp["chamfer"]["angle"] == pytest.approx(90.0, abs=2.0)
        assert lp["chamfer_far"]["width"] == pytest.approx(C, abs=0.05)
    assert not any("ball-nose" in w for w in res.warnings)


def test_far_bevels_go_through_the_flip_jig(both):
    _parts, diag, res = both
    assert [bool(p.get("side2")) for p in res.programs] == [False, True]
    for p in res.programs:
        assert p["manifest"]["safety_gate"] == "PASS", \
            (p.get("side2"), p["manifest"].get("safety_gate_reasons"))
    assert "chamfers on both faces" in diag[0]["reason"]
    lay = res.programs[0]["layout"][0]
    mould = next(lp for lp in lay["parts"] if lp.get("mould"))
    mx = [q[0] for q in mould["outline"]]; my = [q[1] for q in mould["outline"]]
    assert sorted([max(mx) - min(mx), max(my) - min(my)]) == \
        pytest.approx([H, W], abs=0.5), "mould is not the part's true footprint"


def test_pass1_chamfers_the_top_with_the_vee(both):
    _parts, _diag, res = both
    nc = res.programs[0]["nc"]
    assert 5 in _tools(nc), "the vee (T05) never ran in pass 1"
    zs = _zs(nc)
    assert min(zs) < 0, "pass 1 must cut the profile through"
    assert any(abs(z - (T - C)) < 0.01 for z in zs), "no vee trace at chamfer depth"


def test_side2_is_vee_only_at_chamfer_depth(both):
    _parts, _diag, res = both
    nc = res.programs[1]["nc"]
    assert set(_tools(nc)) == {5}, f"side 2 tools {_tools(nc)}"
    # the hole bevel is traced too, not countersunk: a O20 hole sunk with the
    # point would take it r + C = 12mm deep, through the board
    zs = _zs(nc)
    assert min(zs) == pytest.approx(T - C, abs=0.06), min(zs)


def test_top_only_chamfer_is_one_program():
    parts, _diag, res = _run(_board(bottom=False, hole=False), "TopOnly")
    assert [bool(p.get("side2")) for p in res.programs] == [False]
    assert res.programs[0]["manifest"]["safety_gate"] == "PASS"
    out = next(lp for lp in parts[0][1] if lp["role"] == "OUTSIDE")
    assert out["chamfer"]["width"] == pytest.approx(C, abs=0.05)
    assert not out.get("chamfer_far")
    assert 5 in _tools(res.programs[0]["nc"])


def test_bottom_only_chamfer_still_flips():
    """Bevelled only on the face that lies down: the near side has nothing
    for the vee, the far side all of it."""
    _parts, _diag, res = _run(_board(top=False, hole=False), "BottomOnly")
    assert [bool(p.get("side2")) for p in res.programs] == [False, True]
    for p in res.programs:
        assert p["manifest"]["safety_gate"] == "PASS"
    assert 5 not in _tools(res.programs[0]["nc"])
    assert set(_tools(res.programs[1]["nc"])) == {5}


def test_a_partly_chamfered_edge_is_left_square_and_said_so():
    parts, diag, res = _run(_board(bottom=False, hole=False, one_edge=True), "OneEdge")
    out = next(lp for lp in parts[0][1] if lp["role"] == "OUTSIDE")
    assert not out.get("chamfer"), "a one-edge bevel must not chamfer the whole outline"
    assert "only part of their length" in diag[0]["reason"]
    assert not [lp for lp in parts[0][1] if lp["role"] == "SURFACE"]
