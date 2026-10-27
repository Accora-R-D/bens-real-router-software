"""Router: "Add assembly initials to engraved names" (off by default).

Parts read from several assemblies can share a name, and the duplicates were
told apart only by " *" suffixes in upload order. Ticked, each engraving reads
"<name> <initials of its file>" — X1 from Foot End Complete is "X1 FEC".
"""
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from backend.routes.cam_routes import assembly_initials           # noqa: E402
from test_cam_upload_names import _assembly_step, _board          # noqa: E402


def test_initials_and_clashes():
    assert assembly_initials(["Foot End Complete"]) == {"Foot End Complete": "FEC"}
    got = assembly_initials(["Head_End lift", "Head_End leg", "Foot_End leg"])
    assert got == {"Head_End lift": "HELI", "Head_End leg": "HELE", "Foot_End leg": "FEL"}
    assert assembly_initials(["Head End Complete (2)"]) == {"Head End Complete (2)": "HEC"}
    assert assembly_initials(["18mm_2off_Part_61"])["18mm_2off_Part_61"] == "P61"


def _two_assemblies(tmp_path):
    a, b = tmp_path / "Foot End Complete.step", tmp_path / "Head End Complete.step"
    _assembly_step(a, [("X1", _board(0, 0, 0, 300, 120, 18), [(0, 0, 0)]),
                       ("Y1", _board(0, 0, 0, 200, 90, 18), [(400, 0, 0)])])
    _assembly_step(b, [("X1", _board(0, 0, 0, 260, 100, 18), [(0, 0, 0)]),
                       ("Y2", _board(0, 0, 0, 180, 80, 18), [(400, 0, 0)])])
    return [a, b]


def _labels(tmp_path, **mat):
    from fastapi.testclient import TestClient
    from backend.main import app
    files = _two_assemblies(tmp_path)
    with TestClient(app) as c:
        up = c.post("/cam/upload", files=[("files", (f.name, f.read_bytes(), "application/octet-stream"))
                                          for f in files]).json()
        d = c.get("/cam/defaults").json()
        g = c.post(f"/cam/generate/{up['job_id']}", json={
            "material": dict(d["materials"][0], **mat), "tools": d["seats"],
            "sheet_w": 1220, "sheet_h": 2440})
        assert g.status_code == 200, g.text
    return sorted(p["engrave_text"] for s in g.json()["programs"][0]["layout"] for p in s["parts"])


def test_off_by_default_engravings_are_unchanged(tmp_path):
    assert _labels(tmp_path) == ["X1", "X1 *", "Y1", "Y2"]


def test_ticked_each_engraving_carries_its_assemblys_initials(tmp_path):
    assert _labels(tmp_path, engrave_assembly=True) == ["X1 FEC", "X1 HEC", "Y1 FEC", "Y2 HEC"]
