"""Router: 'Name identical parts separately'.

Identical bodies are folded into one part (one entry, N off) and every copy is
engraved with one name. Ticked, each folded-in body comes back under its own
STEP name and count, so each is engraved with its own name. Off by default.
"""
import os
import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from test_cam_upload_names import _assembly_step, _board     # noqa: E402


def _jig(tmp_path):
    step = tmp_path / "Leg Jig.step"
    leg = _board(0, 0, 0, 300, 120, 18)
    _assembly_step(step, [
        ("Left Leg", leg, [(0, 0, 0)]),
        ("Right Leg", _board(0, 0, 0, 300, 120, 18), [(0, 0, 400)]),
        ("Back", _board(0, 0, 0, 400, 200, 18), [(600, 0, 0), (600, 0, 400)]),
    ])
    return step


def _generate(tmp_path, **extra):
    from fastapi.testclient import TestClient
    from backend.main import app
    step = _jig(tmp_path)
    with TestClient(app) as c:
        up = c.post("/cam/upload", files=[("files", (step.name, step.read_bytes(),
                                                      "application/octet-stream"))]).json()
        d = c.get("/cam/defaults").json()
        mat = dict(d["materials"][0], unique_names=extra.pop("unique_names", False))
        body = {"material": mat, "tools": d["seats"], "sheet_w": 1220, "sheet_h": 2440}
        body.update(extra)
        g = c.post(f"/cam/generate/{up['job_id']}", json=body)
        assert g.status_code == 200, g.text
        layout = g.json()["programs"][0]["layout"]
    placed = [p for sheet in layout for p in sheet["parts"]]
    for p in placed:
        assert p["engrave_text"] == p["name"], "each copy is engraved with its own entry's name"
    return up, Counter(p["name"] for p in placed)


def test_off_by_default_identical_parts_stay_one_entry(tmp_path):
    up, placed = _generate(tmp_path)
    assert up["names"] == ["Left Leg", "Back"] or set(up["names"]) == {"Left Leg", "Back"}
    assert "Right Leg" not in placed and placed["Left Leg"] == 2, placed
    assert sum(placed.values()) == 4, placed           # 2 legs + 2 backs, as before


def test_ticked_each_identical_part_keeps_its_own_name(tmp_path):
    _up, placed = _generate(tmp_path, unique_names=True)
    assert placed["Left Leg"] == 1 and placed["Right Leg"] == 1, placed
    assert placed["Back"] == 2, "same name, same part: still one entry, 2 off"
    assert sum(placed.values()) == 4


def test_an_unticked_part_stays_out_when_split(tmp_path):
    _up, placed = _generate(tmp_path, unique_names=True, exclude=["Left Leg"])
    assert "Right Leg" not in placed and "Left Leg" not in placed, placed


def test_an_edited_quantity_keeps_the_part_whole(tmp_path):
    _up, placed = _generate(tmp_path, unique_names=True, quantities={"Left Leg": 5, "Back": 2})
    assert placed["Left Leg"] == 5 and "Right Leg" not in placed, placed


def test_the_engraving_label_is_each_parts_own_name():
    from backend.routes.cam_routes import _split_identical

    class P:
        def __init__(self, name, qty, merged):
            self.name, self.qty, self.merged_qty = name, qty, merged

    loops = [{"role": "OUTSIDE", "pts": [(0, 0), (1, 0), (1, 1)]}]
    parts = [("Left Leg", loops, {"qty": 3, "label": "Left Leg"})]
    out = _split_identical(parts, [P("Left Leg", 3, {"Right Leg": 1, "Spare Leg": 1})])
    assert [(n, m["qty"], m["label"]) for n, _l, m in out] == [
        ("Left Leg", 1, "Left Leg"), ("Right Leg", 1, "Right Leg"), ("Spare Leg", 1, "Spare Leg")]
    assert out[1][1] is not loops and out[1][1] == loops, "each entry needs its own loops"
