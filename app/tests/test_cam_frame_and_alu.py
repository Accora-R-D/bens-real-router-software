"""Router: the opening frame, and the reviewed 12 mm aluminium preset.

Frame: before anything is cut the tool traces the outline of the whole nest at
the clearance height, so the operator sees the origin is on the stock (and that
the rapid plane clears the clamps) before the first plunge.

Aluminium: reviewed for a first physical run (2026-10-15) and set conservatively —
feeds that come out as written, shallow pecks, tabs on, and no feed move that goes
down faster than the plunge feed.
"""
import math
import os
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from backend.services import step_ingest_service as S        # noqa: E402


def _plate(tmp_path, t=12.0, name="Plate"):
    """A plate with a Ø6 through hole (pecked) and a shallow pocket."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 150, 100, t).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeBox(gp_Pnt(20, 20, t - 3), 40, 30, 4).Shape()).Shape()
    s = BRepAlgoAPI_Cut(s, BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(110, 70, -1), gp_Dir(0, 0, 1)), 3.0, t + 2).Shape()).Shape()
    path = tmp_path / f"{t:g}mm_1off_{name}.step"
    S.write_solid_step(s, str(path))
    return path


def _run(tmp_path, material_name, t, **overrides):
    from fastapi.testclient import TestClient
    from backend.main import app
    step = _plate(tmp_path, t)
    with TestClient(app) as c:
        job = c.post("/cam/upload", files=[("files", (step.name, step.read_bytes(),
                                                      "application/octet-stream"))]).json()["job_id"]
        d = c.get("/cam/defaults").json()
        mat = dict(next(m for m in d["materials"] if m["name"] == material_name), **overrides)
        g = c.post(f"/cam/generate/{job}", json={"material": mat, "tools": d["seats"],
                                                  "sheet_w": 1220, "sheet_h": 2440})
        assert g.status_code == 200, g.text
        p = g.json()["programs"][0]
        nc = c.get(f"/cam/nc/{job}", params={"thickness": p["key"]}).text
    return p, nc, mat


def _moves(nc):
    """(g, x, y, z, f, line) for every motion line, modal state carried."""
    num = re.compile(r"([GXYZFIJ])(-?\d+\.?\d*)")
    g = 0; x = y = z = 0.0; f = 0.0
    for raw in nc.splitlines():
        line = re.sub(r"^N\d+", "", raw)
        if re.search(r"G53|G28", line):
            continue
        t = {m.group(1): float(m.group(2)) for m in num.finditer(line)}
        if "G" in t and t["G"] in (0, 1, 2, 3):
            g = int(t["G"])
        if "F" in t:
            f = t["F"]
        if any(k in t for k in "XYZ"):
            px, py, pz = x, y, z
            x, y, z = t.get("X", x), t.get("Y", y), t.get("Z", z)
            yield g, (px, py, pz), (x, y, z), f, raw.strip()


# ------------------------------------------------------------------- frame
def test_the_frame_traces_the_nest_at_the_rapid_height_before_any_cut(tmp_path):
    p, nc, mat = _run(tmp_path, "18mm MDF", 18.0)
    assert p["manifest"]["safety_gate"] == "PASS", p["manifest"].get("safety_gate_reasons")
    body = [re.sub(r"^N\d+", "", ln) for ln in nc.splitlines()]
    spindle = next(i for i, ln in enumerate(body) if re.fullmatch(r"S\d+M0?3", ln))
    first_cut = next(i for i, ln in enumerate(body) if re.match(r"G0?[123](?!\d)", ln))
    frame = body[spindle + 1: spindle + 7]
    clear = 18.0 + mat["clearance_over"]
    assert frame[0] == f"G00Z{clear:.3f}"
    assert all(ln.startswith("G00X") and "Z" not in ln for ln in frame[1:])
    assert frame[1] == frame[5], "the frame closes back on its first corner"
    assert first_cut > spindle + 6, "the frame comes before the first cutting move"
    # the rectangle is what the cutter will touch: every cut centre, grown by R
    cuts = [(b[0], b[1]) for g, _a, b, _f, _l in _moves(nc) if g in (1, 2, 3)]
    corners = [tuple(float(v) for v in re.findall(r"[XY](-?\d+\.\d+)", ln)) for ln in frame[1:5]]
    xs = [c[0] for c in corners]; ys = [c[1] for c in corners]
    r = 3.0                                   # Ø6 profile cutter
    assert min(xs) == pytest.approx(max(min(c[0] for c in cuts) - r, 0), abs=0.01)
    assert max(xs) == pytest.approx(max(c[0] for c in cuts) + r, abs=0.01)
    assert min(ys) == pytest.approx(max(min(c[1] for c in cuts) - r, 0), abs=0.01)
    assert max(ys) == pytest.approx(max(c[1] for c in cuts) + r, abs=0.01)
    assert "Watch the opening frame" in p["job_sheet"]
    assert p["manifest"]["frame"]["z"] == pytest.approx(clear)


def test_the_frame_can_be_turned_off(tmp_path):
    p, nc, _mat = _run(tmp_path, "18mm MDF", 18.0, frame_nest=False)
    body = [re.sub(r"^N\d+", "", ln) for ln in nc.splitlines()]
    spindle = next(i for i, ln in enumerate(body) if re.fullmatch(r"S\d+M0?3", ln))
    assert not (body[spindle + 1].startswith("G00Z") and body[spindle + 2].startswith("G00X")
                and body[spindle + 6] == body[spindle + 2]), "a frame was emitted anyway"
    assert "frame" not in p["manifest"]
    assert p["manifest"]["safety_gate"] == "PASS"


# --------------------------------------------------------------- aluminium
def test_the_aluminium_preset_is_the_reviewed_one():
    from router_cam.models import DEFAULT_MATERIALS
    al = next(m for m in DEFAULT_MATERIALS if m.name == "12mm Aluminium")
    assert (al.spindle_rpm, al.chipload, al.plunge_chipload, al.max_step) == (8000, 0.03, 0.01, 0.5)
    assert (al.drill_rpm, al.drill_feed, al.drill_peck) == (8000, 150.0, 1.0)
    assert (al.chamfer_rpm, al.chamfer_feed) == (8000, 300.0)
    assert (al.tabs_mode, al.tab_remain, al.limit_descent) == ("on", 0.8, True)
    md = al.mat_dict()
    assert (md["cut_feed"], md["plunge_feed"]) == (480.0, 160.0), "feeds must come out as written"


def test_aluminium_never_plunges_faster_than_the_plunge_feed(tmp_path):
    p, nc, mat = _run(tmp_path, "12mm Aluminium", 12.0)
    assert p["manifest"]["safety_gate"] == "PASS", p["manifest"].get("safety_gate_reasons")
    assert re.search(r"S8000M3", nc) and not re.search(r"S(?!8000)\d+M3", nc)
    worst = 0.0
    for g, (px, py, pz), (x, y, z), f, _line in _moves(nc):
        if g == 1 and z < pz - 1e-9 and z < 12.0:
            L = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
            worst = max(worst, f * (pz - z) / L)
    assert worst <= 160.0 + 2.0, f"descended into the plate at {worst:.0f} mm/min"
    # the Ø6 hole is pecked 1 mm at a time at the drill feed
    pecks = [ln for g, a, b, f, ln in _moves(nc)
             if g == 1 and b[2] < a[2] and abs(b[0] - a[0]) < 1e-9 and abs(b[1] - a[1]) < 1e-9
             and f == 150.0]
    assert len(pecks) == math.ceil((12.0 + 0.2) / 1.0)
    assert any("Tabs" in ln and "holding tabs" in ln for ln in p["job_sheet"].splitlines())


def test_mdf_keeps_its_descent_behaviour():
    """limit_descent is a metal setting; MDF programs are unchanged by it."""
    from router_cam.models import DEFAULT_MATERIALS
    for m in DEFAULT_MATERIALS:
        if "MDF" in m.name:
            assert m.limit_descent is False and m.drill_peck == 4.0


def test_feed_rounding_only_refines_slow_feeds():
    from router_cam.models import _feed
    assert all(_feed(v) == round(v, -2) for v in range(1000, 10000))
    assert (_feed(480), _feed(160), _feed(7008), _feed(0)) == (480.0, 160.0, 7000.0, 0.0)
