"""3D jobs: shaped parts, chosen at upload, kept apart from the flat path.

A tapered block (flat base, sloping sides, smaller flat top) read as a flat
plate machines its BIG face — every slope then faces away from the cutter —
and came out as a plain rectangle. A 3D job reads it from the side its shape
faces: footprint profile, one heightfield surfaced over the whole part, rough
with the flat cutter then finish with the ball nose. With no ball nose loaded
it is roughed to size in fine steps instead (terraced slopes), so parts can
be cut before one is fitted. Flat jobs are untouched by any of it.
"""
import math
import os
import re
import sys
import dataclasses
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from backend.services import step_ingest_service as S                    # noqa: E402

T = 18.0


def _frustum(L=100.0, W=40.0, top_inset=10.0, h=T):
    """A tapered block like Ben's 'Part 1': L x W base, sides sloping in by
    `top_inset` to a smaller flat top, h tall."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.gp import gp_Pnt

    def rect(x0, y0, x1, y1, z):
        p = BRepBuilderAPI_MakePolygon(gp_Pnt(x0, y0, z), gp_Pnt(x1, y0, z),
                                       gp_Pnt(x1, y1, z), gp_Pnt(x0, y1, z), True)
        return p.Wire()
    ts = BRepOffsetAPI_ThruSections(True, True)
    ts.AddWire(rect(0, 0, L, W, 0))
    ts.AddWire(rect(top_inset, top_inset, L - top_inset, W - top_inset, h))
    ts.Build()
    return ts.Shape()


def _dome():
    """A 120 x 120 x 18 block with a shallow spherical top — gentle slopes."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.gp import gp_Pnt
    box = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 120, 120, T).Shape()
    ball = BRepPrimAPI_MakeSphere(gp_Pnt(60, 60, T - 160.0), 160.0).Shape()
    return BRepAlgoAPI_Common(box, ball).Shape()


def _part(shape, name="Part 1"):
    import types
    return types.SimpleNamespace(name=name, qty=1, thickness_mm=T, shape=shape,
                                 part_number="", merged_qty={})


def _tools(ball):
    from router_cam.models import DEFAULT_TOOL_TABLE, ToolTable
    return ToolTable([dataclasses.replace(s, loaded=ball) if s.kind == "ballnose" else s
                      for s in DEFAULT_TOOL_TABLE.seats])


def _generate(shape, ball, qty=1):
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS
    p = _part(shape)
    p.qty = qty
    parts, diag = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")
    return generate_from_parts(parts, diag, f"t3d-{ball}-{qty}", DEFAULT_MATERIALS[0],
                               _tools(ball), sheet_w=1220, sheet_h=2440, border_mm=20)


def _surface_z(nc, tool):
    """Z of every cutting move made with `tool` (Txx M06 ... next M06)."""
    zs, cur = [], None
    for ln in nc.splitlines():
        m = re.search(r"T0?(\d+)M06", ln)
        if m:
            cur = int(m.group(1))
        elif cur == tool and re.match(r"N\d+G01", ln):
            z = re.search(r"Z(-?[\d.]+)", ln)
            if z:
                zs.append(float(z.group(1)))
    return zs


# ------------------------------------------------------------ the problem

def test_a_flat_job_still_reads_a_tapered_block_as_its_outline():
    """The flat path is unchanged: this is what a flat job does with it."""
    from backend.services import wood_geometry as WG
    g = WG.part_geometry(_frustum(), "Part 1")
    assert [lp["role"] for lp in g["loops"]] == ["OUTSIDE"]


def test_shaped_parts_are_suggested_and_boards_are_not():
    from backend.services.wood_3d import looks_3d
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopoDS import TopoDS
    from OCP.gp import gp_Pnt
    assert looks_3d(_frustum())
    assert looks_3d(_dome())
    board = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 400, 300, T).Shape()
    assert not looks_3d(board)
    # a 3 mm chamfer round the top is an edge break, not a shape
    ch = BRepFilletAPI_MakeChamfer(board)
    exp = TopExp_Explorer(board, TopAbs_EDGE)
    while exp.More():
        e = TopoDS.Edge_s(exp.Current())
        ch.Add(3.0, e)
        exp.Next()
    assert not looks_3d(ch.Shape())


# --------------------------------------------------------------- geometry

def test_a_3d_part_is_profiled_round_its_footprint_and_surfaced_all_over():
    from backend.services.wood_3d import part_geometry_3d, SKIN_MM
    g = part_geometry_3d(_frustum())
    roles = [lp["role"] for lp in g["loops"]]
    assert roles == ["OUTSIDE", "SURFACE"]
    assert g["thickness"] == pytest.approx(T, abs=0.01)
    o = g["loops"][0]["pts"]
    w = max(p[0] for p in o) - min(p[0] for p in o)
    h = max(p[1] for p in o) - min(p[1] for p in o)
    assert sorted((round(w, 1), round(h, 1))) == [40.0, 100.0], "not the base footprint"
    s = g["loops"][1]
    assert s["zmap"].min() == pytest.approx(SKIN_MM), "off the part must stop at the skin"
    assert s["zmap"].max() == pytest.approx(T, abs=0.01)


# ------------------------------------------------------------ the programs

def test_with_the_ball_nose_it_roughs_then_finishes_then_profiles():
    from backend.services.wood_3d import SKIN_MM
    prog = _generate(_frustum(), ball=True).programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS", prog["manifest"].get("safety_gate_reasons")
    order = [int(m.group(1)) for m in re.finditer(r"T0?(\d+)M06", prog["nc"])]
    assert order == [4, 1, 4], order
    ball_z = _surface_z(prog["nc"], 1)
    assert min(ball_z) >= SKIN_MM - 0.2, "the ball went below the skin"
    assert max(ball_z) == pytest.approx(T, abs=0.05)
    assert not any("no ball-nose" in w for w in prog["warnings"])


def test_with_no_ball_nose_it_is_roughed_to_size_not_skipped():
    from backend.services.wood_3d import SKIN_MM
    prog = _generate(_frustum(), ball=False).programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS", prog["manifest"].get("safety_gate_reasons")
    assert re.findall(r"T0?(\d+)M06", prog["nc"]) == ["4"]
    assert any("ROUGHED ONLY" in w for w in prog["warnings"]), prog["warnings"]
    # waterlines every 1 mm: the cutter visits (nearly) every integer level
    levels = {round(z, 3) for z in _surface_z(prog["nc"], 4) if z > SKIN_MM}
    whole = {float(k) for k in range(2, int(T))}
    assert len(whole & levels) >= len(whole) - 1, sorted(levels)


def _stamp_check(shape, ball):
    """(worst gouge, 95th-percentile leftover) of the surfacing alone, by
    stamping every surfacing move into the part's own heightfield."""
    import copy
    import numpy as np
    import cam_syntec as CS
    import nest
    from backend.services import wood_cam_service as WCS
    p = _part(shape)
    parts, _diag = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")
    _generate(shape, ball)                      # sets the engine's tool table
    name, loops, meta = parts[0]
    pp = nest.prepare_part(name, copy.deepcopy(loops), dict(meta, tool_map={}))
    s = next(lp for lp in loops if lp["role"] == "SURFACE")
    allp = [q for lp in loops for q in lp["pts"]]
    mnx, mny = min(q[0] for q in allp), min(q[1] for q in allp)
    z, g = s["zmap"], s["grid"]
    res, ox, oy = g["res"], g["ox"] - mnx, g["oy"] - mny
    stock = np.full(z.shape, float(s["top"]))
    jj, ii = np.mgrid[0:z.shape[0], 0:z.shape[1]]
    X, Y = ox + ii * res, oy + jj * res
    for role, paths, _pd, t in pp["jobs"]:
        if role != "SURFACE":
            continue
        R = CS.TOOLS[t]["dia"] / 2
        isball = t == 1
        for path in paths:
            for (x0, y0, z0), (x1, y1, z1) in zip(path, path[1:]):
                n = max(1, int(math.dist((x0, y0), (x1, y1)) / 0.25))
                for k in range(n + 1):
                    f = k / n
                    x, y, zz = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, z0 + (z1 - z0) * f
                    i0, j0 = max(0, int((x - R - ox) / res)), max(0, int((y - R - oy) / res))
                    i1, j1 = int((x + R - ox) / res) + 1, int((y + R - oy) / res) + 1
                    d2 = (X[j0:j1, i0:i1] - x) ** 2 + (Y[j0:j1, i0:i1] - y) ** 2
                    h = zz + (R - np.sqrt(np.maximum(R * R - d2, 0))) if isball else zz
                    blk = stock[j0:j1, i0:i1]
                    np.minimum(blk, np.where(d2 <= R * R, h, np.inf), out=blk)
    on = z > 3.0                         # clear of the ball-radius fillet at the foot
    d = stock - z
    return float(d[on].min()), float(np.percentile(d[on], 95))


@pytest.mark.parametrize("make", [_frustum, _dome], ids=["steep", "gentle"])
def test_finished_surface_is_close_and_never_gouged(make):
    gouge, left = _stamp_check(make(), ball=True)
    assert gouge > -0.15, f"gouged {gouge:.2f} mm"
    assert left < 0.25, f"95% leftover {left:.2f} mm"


@pytest.mark.parametrize("make", [_frustum, _dome], ids=["steep", "gentle"])
def test_roughed_only_surface_is_within_a_terrace_and_never_gouged(make):
    gouge, left = _stamp_check(make(), ball=False)
    assert gouge > -0.05, f"gouged {gouge:.2f} mm"
    assert left < 1.5, f"95% leftover {left:.2f} mm"


def test_nested_copies_do_not_surface_each_other():
    """Surfacing reaches past the outline; the nest must leave room for it."""
    res = _generate(_frustum(), ball=True, qty=6)
    prog = res.programs[0]
    assert prog["manifest"]["safety_gate"] == "PASS"
    import cam_syntec as CS
    import nest
    from backend.services import wood_cam_service as WCS
    p = _part(_frustum())
    (name, loops, meta), = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")[0]
    pp = nest.prepare_part(name, loops, dict(meta, tool_map={}))
    from shapely.geometry import Point, Polygon
    keep = Polygon(pp["keepout"])
    for role, paths, _pd, t in pp["jobs"]:
        if role == "SURFACE":
            r = CS.TOOLS[t]["dia"] / 2
            for path in paths:
                for x, y, _z in path[::25]:
                    assert keep.contains(Point(x, y).buffer(r)), (x, y)


def test_3d_parts_are_not_engraved_and_flat_parts_still_are():
    from backend.services import wood_cam_service as WCS
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    p3 = _part(_frustum())
    (_n, _l, meta3), = WCS.cam_parts_and_diag([p3], names={p3.name}, mode="3d")[0]
    assert meta3.get("no_engrave") is True
    pf = _part(BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 300, 200, T).Shape(), "Board")
    (_n, _l, metaf), = WCS.cam_parts_and_diag([pf], names={pf.name})[0]
    assert "no_engrave" not in metaf


# ------------------------------------------------------------------ routes

def test_upload_chooses_the_job_type_and_flat_jobs_suggest_3d(tmp_path):
    from fastapi.testclient import TestClient
    from backend.main import app
    step = tmp_path / "Part 1.step"
    S.write_solid_step(_frustum(), str(step))
    files = [("files", (step.name, step.read_bytes(), "application/octet-stream"))]
    with TestClient(app) as c:
        flat = c.post("/cam/upload", files=files).json()
        assert flat["mode"] == "flat" and flat["suggest_3d"] == ["Part 1"]
        three = c.post("/cam/upload", files=files, data={"mode": "3d"}).json()
        assert three["mode"] == "3d" and three["suggest_3d"] == []
        # the suggestion's switch: same parts, read the 3D way on the next Nest
        sw = c.post(f"/cam/mode/{flat['job_id']}", json={"mode": "3d"}).json()
        assert sw["mode"] == "3d"
        assert c.get(f"/cam/wood/{flat['job_id']}").json()["mode"] == "3d"
        d = c.get("/cam/defaults").json()
        g = c.post(f"/cam/generate/{flat['job_id']}", json={
            "material": d["materials"][0], "tools": d["seats"],
            "sheet_w": 1220, "sheet_h": 2440, "border_mm": 20})
        assert g.status_code == 200, g.text
        prog = g.json()["programs"][0]
        assert any("ROUGHED ONLY" in w for w in prog["warnings"])   # T01 ships unticked


def test_the_page_offers_the_choice_before_the_drop():
    page = (ROOT / "frontend" / "cam.html").read_text(encoding="utf-8")
    assert 'name="jobmode" value="flat" checked' in page
    assert 'name="jobmode" value="3d"' in page
    assert 'form.append("mode"' in page and "/cam/mode/" in page
