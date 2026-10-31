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


def test_a_seam_between_triangles_is_not_a_hole():
    """Part 1 (7), a solid tapered block, read on the Linux server with a
    zero-area hole in its footprint (three points at one spot) and was
    reported as a slot too narrow for any tool. Specks go; real holes stay."""
    from shapely.geometry import Polygon
    from backend.services import wood_3d as W3
    outer = [(0, 0), (150, 0), (150, 80), (0, 80)]
    speck = [(31.21, 29.14), (31.2100001, 29.14), (31.21, 29.1400001)]
    hole = [(60, 30), (70, 30), (70, 40), (60, 40)]
    foot = Polygon(outer, [speck, hole])
    clean = W3._drop_specks(foot)
    assert len(clean.interiors) == 1
    assert Polygon(clean.interiors[0]).area == pytest.approx(100.0)
    assert W3._drop_specks(Polygon(outer, [hole])).equals(Polygon(outer, [hole]))


def test_a_forced_roughing_seat_roughs_the_3d_part():
    """Tools & feeds on a 3D job offers Roughing / Finishing / Outer profile.
    Forcing the Ø4 (T07) for roughing must put the surfacing on T07."""
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS
    p = _part(_frustum())
    parts, diag = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")
    res = generate_from_parts(parts, diag, "t3d-rough7", DEFAULT_MATERIALS[0], _tools(False),
                              sheet_w=1220, sheet_h=2440, border_mm=20,
                              tool_map={"rough": 7})
    prog = res.programs[0]
    assert _surface_z(prog["nc"], 7), "nothing was cut with T07"
    assert any("3D roughing" in w for w in prog["warnings"])     # the override is stated


def test_the_page_hides_flat_only_settings_on_a_3d_job():
    page = (ROOT / "frontend" / "cam.html").read_text(encoding="utf-8")
    assert "body.mode3d .flat-only" in page
    for marker in ('id="engrave"', 'id="machine3d"', 'id="dogbone"', 'id="edgebreak"'):
        i = page.index(marker)
        assert page.rfind("flat-only", 0, i) > page.rfind('class="pane"', 0, i), marker
    assert '["rough", "Roughing (flat cutter)"]' in page


def test_a_3d_program_carries_its_placed_scene():
    """The 3D job view draws each part's model and toolpaths where the nest put
    them: the model must sit on the layout's outline, and a flat job has none."""
    res = _generate(_frustum(), ball=True)
    prog = res.programs[0]
    (part,) = prog["viewer"]["scene"]
    xs = part["tris"][0::3]
    outline = prog["layout"][0]["parts"][0]["outline"]
    assert min(xs) == pytest.approx(min(q[0] for q in outline), abs=0.05)
    assert max(xs) == pytest.approx(max(q[0] for q in outline), abs=0.05)
    assert {o["op"] for o in part["ops"]} == {"surface", "profile"}
    assert {o["tool"] for o in part["ops"] if o["op"] == "surface"} == {1, 4}


def test_the_page_has_a_3d_view_beside_the_plan_for_3d_jobs():
    page = (ROOT / "frontend" / "cam.html").read_text(encoding="utf-8")
    assert 'id="view3d"' in page and 'id="planpane"' in page
    assert "body.mode3d #viewsplit" in page and "minmax(0, 3fr) minmax(0, 1fr)" in page
    for box in ("v3model", "v3rough", "v3finish", "v3profile", "v3stock"):
        assert f'id="{box}"' in page
    assert "draw3DJob();" in page and "async function viewerData(prog)" in page


# ------------------------------------------------------ plates in a 3D job

PLATE_T = 15.0


def _windowed_plate():
    """A 220 x 36 x 15 stay with a window drawn as a closed spline (as a sketch
    spline extrudes: a B-spline wall, square to the board) and a O8 hole."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace,
                                    BRepBuilderAPI_MakeWire)
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism
    from OCP.GeomAPI import GeomAPI_Interpolate
    from OCP.TColgp import TColgp_HArray1OfPnt
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec
    plate = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 220, 36, PLATE_T).Shape()
    ring = [(60, 10), (100, 8), (140, 12), (150, 20), (130, 27), (90, 28), (65, 22)]
    arr = TColgp_HArray1OfPnt(1, len(ring))
    for i, (x, y) in enumerate(ring, 1):
        arr.SetValue(i, gp_Pnt(x, y, -1.0))
    it = GeomAPI_Interpolate(arr, True, 1e-6)
    it.Perform()
    wire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(it.Curve()).Edge()).Wire()
    window = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire).Face(),
                                   gp_Vec(0, 0, PLATE_T + 2)).Shape()
    hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(20, 18, -1), gp_Dir(0, 0, 1)), 4.0, PLATE_T + 2).Shape()
    for tool in (window, hole):
        plate = BRepAlgoAPI_Cut(plate, tool).Shape()
    return plate


def _standing(shape):
    """The same part as it arrives modelled on edge: board face vertical."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
    t = gp_Trsf()
    t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi / 2)
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


def test_a_spline_window_does_not_make_a_plate_look_3d():
    """A sketch-spline cut-out is a B-spline wall, not a cylinder: 15 mm of it
    stood square to every windowed stay and they were all flagged as shaped."""
    from backend.services.wood_3d import looks_3d
    assert not looks_3d(_windowed_plate())
    assert not looks_3d(_standing(_windowed_plate()))


def test_a_3d_job_cuts_a_plate_flat_as_a_board_and_draws_it():
    """Stays dropped into a 3D job were stood on edge (36 mm stock for a 15 mm
    plate) and roughed as a heightfield. A plate is now read like a flat job's
    board — laid on its face, profile, cut-outs and holes — and the 3D view
    still draws it, sitting on its nested outline."""
    from backend.services import wood_cam_service as WCS
    p = _part(_standing(_windowed_plate()), "Stay")
    p.thickness_mm = 36.0                           # what the bbox said on edge
    (name, loops, meta), = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")[0]
    _parts, (row,) = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")
    assert row["thickness_mm"] == pytest.approx(PLATE_T, abs=0.01)
    assert "flat plate" in row["reason"]
    roles = sorted(lp["role"] for lp in loops)
    assert roles == ["DRILL", "INSIDE", "OUTSIDE"]
    assert meta.get("no_engrave") is True

    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS
    parts, diag = WCS.cam_parts_and_diag([p], names={p.name}, mode="3d")
    res = generate_from_parts(parts, diag, "t3d-plate", DEFAULT_MATERIALS[0], _tools(True),
                              sheet_w=1220, sheet_h=2440, border_mm=20)
    (prog,) = res.programs
    assert prog["thickness"] == pytest.approx(PLATE_T, abs=0.01)
    assert "G18" not in prog["nc"] and not _surface_z(prog["nc"], 1), "a plate is not surfaced"
    (part,) = prog["viewer"]["scene"]
    xs, ys, zs = part["tris"][0::3], part["tris"][1::3], part["tris"][2::3]
    outline = prog["layout"][0]["parts"][0]["outline"]
    assert min(xs) == pytest.approx(min(q[0] for q in outline), abs=0.05)
    assert max(ys) == pytest.approx(max(q[1] for q in outline), abs=0.05)
    assert min(zs) == pytest.approx(0, abs=0.01) and max(zs) == pytest.approx(PLATE_T, abs=0.01)
    ops = [o["op"] for o in part["ops"]]
    assert ops.count("profile") == 2, "outline and window drawn round at full depth"
    assert {"pocket", "plunge"} & set(ops), "the O8 hole drawn as it is cleared"


def _trussed_plate(n=6, W=36.0, L=220.0):
    """A stay with a row of n spline windows, alternately pointing up and down:
    enough window wall that a side view reaches more of it than the top does."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace,
                                    BRepBuilderAPI_MakeWire)
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
    from OCP.GeomAPI import GeomAPI_Interpolate
    from OCP.TColgp import TColgp_HArray1OfPnt
    from OCP.gp import gp_Pnt, gp_Vec
    plate = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), L, W, PLATE_T).Shape()
    pitch = (L - 30) / n
    for k in range(n):
        x0, up = 15 + k * pitch, k % 2 == 0
        y0, y1 = (5, W - 5) if up else (W - 5, 5)
        tri = [(x0 + 2, y0), (x0 + pitch - 2, y0), (x0 + pitch / 2, y1)]
        mids = [((a[0] + b[0]) / 2 + (0.6 if up else -0.6), (a[1] + b[1]) / 2)
                for a, b in zip(tri, tri[1:] + tri[:1])]
        ring = [q for pair in zip(tri, mids) for q in pair]
        arr = TColgp_HArray1OfPnt(1, len(ring))
        for i, (x, y) in enumerate(ring, 1):
            arr.SetValue(i, gp_Pnt(x, y, -1.0))
        it = GeomAPI_Interpolate(arr, True, 1e-6)
        it.Perform()
        face = BRepBuilderAPI_MakeFace(
            BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(it.Curve()).Edge()).Wire()).Face()
        plate = BRepAlgoAPI_Cut(plate, BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, PLATE_T + 2)).Shape()).Shape()
    return plate


def test_a_3d_read_lays_a_windowed_plate_flat_not_on_edge():
    """Most surface reached from above stood this stay on edge — a side view
    sees half its window walls — and roughed it from 36 mm stock, undercutting
    the other half. The least undercut decides, then the lowest stock."""
    from backend.services.wood_3d import part_geometry_3d
    g = part_geometry_3d(_standing(_trussed_plate()))
    assert g["thickness"] == pytest.approx(PLATE_T, abs=0.05)
    assert not g["warnings"], "lying flat, nothing is undercut"
