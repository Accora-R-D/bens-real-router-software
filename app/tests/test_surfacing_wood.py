"""3D surfacing of sculpted wood parts (ball nose in a 'ballnose' seat).

A part with a freeform region on its machined face — a dished seat, a formed
relief — used to be misread: the region's boundary ring matched no pocket
floor, fell through to INSIDE, and the sculpt was CUT OUT of the part as a
slug. Such rings are now probed against the solid (a through feature is empty
all the way down; a sculpt has material) and become SURFACE regions carrying
a heightfield. The engine roughs them with the widest flat tool and raster-
finishes with the ball nose — both paths from the drop-cutter field, which is
gouge-safe by construction. Parts without freeform faces never enter any of
this: detection short-circuits on the face-type scan.
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

T = 18.0


def _dished_board():
    """220x140x18 board with a spherical dish (~O60 mouth, 8 deep) + a thru hole."""
    from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere,
                                 BRepPrimAPI_MakeCylinder)
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 220, 140, T).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeSphere(
        gp_Pnt(110, 70, T + 52.0), 60.0).Shape()).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(30, 30, -1), gp_Dir(0, 0, 1)), 4.0, T + 2).Shape()).Shape()
    return s


def test_a_sculpted_region_is_surfaced_not_cut_out():
    from backend.services import wood_geometry as WG
    g = WG.part_geometry(_dished_board(), "dish")
    roles = [lp["role"] for lp in g["loops"]]
    assert "SURFACE" in roles, roles
    assert "INSIDE" not in roles, "the dish boundary was cut out as a slug"
    lp = next(l for l in g["loops"] if l["role"] == "SURFACE")
    import numpy as np
    assert lp["zmap"] is not None and lp["top"] == pytest.approx(T)
    assert np.nanmin(lp["zmap"]) == pytest.approx(T - 8.06, abs=0.2)


def test_flat_parts_never_enter_the_surfacing_path():
    from backend.services import wood_geometry as WG
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 120, T).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(60, 60, -1), gp_Dir(0, 0, 1)), 5.0, T + 2).Shape()).Shape()
    g = WG.part_geometry(s, "flat")
    assert all(lp["role"] != "SURFACE" for lp in g["loops"])


@pytest.fixture(scope="module")
def dish_result():
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS, ToolTable
    import dataclasses
    # a surfacing job: the ball nose IS fitted, so T01 is ticked on (it ships
    # unticked — see DEFAULT_TOOL_TABLE)
    tools = ToolTable([dataclasses.replace(s, loaded=True) if s.kind == "ballnose" else s
                       for s in DEFAULT_TOOL_TABLE.seats])
    part = types.SimpleNamespace(name="dish", qty=1, thickness_mm=T,
                                 shape=_dished_board(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"dish"})
    return generate_from_parts(parts, diag, "surf-test",
                               DEFAULT_MATERIALS[0], tools,
                               sheet_w=1220, sheet_h=2440)


def _tool_z_ranges(nc):
    cur, out = None, {}
    for ln in nc.splitlines():
        m = re.search(r"T0?(\d+)M06", ln)
        if m:
            cur = int(m.group(1))
            out.setdefault(cur, [])
            continue
        if cur is not None and "G01" in ln:
            mz = re.search(r"Z([-\d.]+)", ln)
            if mz:
                out[cur].append(float(mz.group(1)))
    return out


def test_rough_flat_then_ball_finish_then_the_rest(dish_result):
    prog = dish_result.programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS", \
        prog["manifest"].get("safety_gate_reasons")
    order = [int(m.group(1)) for ln in prog["nc"].splitlines()
             for m in [re.search(r"T0?(\d+)M06", ln)] if m]
    assert order == [4, 1, 4], f"toolchange order {order}"
    zs = _tool_z_ranges(prog["nc"])
    # ball finish reaches the dish bottom; roughing leaves stock above it
    assert min(zs[1]) == pytest.approx(T - 8.06, abs=0.15)
    assert min(zs[4]) == pytest.approx(-0.5, abs=0.05), "profile no longer cuts through"
    # the surfaced region reaches the plan preview + its legend flag
    lay = prog["layout"][0]["parts"][0]
    assert lay.get("surface"), "surfaced region missing from the plan layout"


def test_no_ball_nose_skips_surfacing_loudly_and_safely():
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS, ToolTable, Seat
    tools = ToolTable([Seat(4, 6.0, 2, "upcut", True),
                       Seat(5, 0.0, 2, "vee", True)])
    part = types.SimpleNamespace(name="dish", qty=1, thickness_mm=T,
                                 shape=_dished_board(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"dish"})
    res = generate_from_parts(parts, diag, "surf-noball",
                              DEFAULT_MATERIALS[0], tools,
                              sheet_w=1220, sheet_h=2440)
    prog = res.programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS"
    assert any("no ball-nose" in w for w in prog["warnings"]), prog["warnings"]
    assert "T01M06" not in prog["nc"] and "T1M06" not in prog["nc"]
    # and crucially: the dish is left UNCUT — nothing cuts inside its region
    # (before SURFACE existed the ring was bored out as a through slug)
    region = prog["layout"][0]["parts"][0].get("surface")
    assert region, "surface region should still be reported"
    cx = sum(q[0] for q in region[0]) / len(region[0])
    cy = sum(q[1] for q in region[0]) / len(region[0])
    x = y = 0.0
    for ln in prog["nc"].splitlines():
        mx = re.search(r"X([-\d.]+)", ln); my = re.search(r"Y([-\d.]+)", ln)
        if mx: x = float(mx.group(1))
        if my: y = float(my.group(1))
        if "G01" in ln and (mx or my):
            assert (x - cx) ** 2 + (y - cy) ** 2 > 15.0 ** 2, \
                f"cut inside the unmachinable dish: {ln}"
