"""The router does not keep every job's geometry for ever.

Each upload kept its parsed STEP and each Nest its 3-D simulation payload for
the life of the server, ~47 MB per round; the 512 MB Render instance was killed
mid-request after a few rounds and the page showed the proxy's HTML error page
("Unexpected token '<' ... is not valid JSON", 2026-10-12). Only the most
recently used jobs keep the heavy parts now.
"""
import os
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from backend.services import step_ingest_service as S          # noqa: E402


def _board_step(tmp_path, name):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    path = tmp_path / f"18mm_1off_{name}.step"
    S.write_solid_step(BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 300, 200, 18).Shape(), str(path))
    return path


def _upload(c, path):
    r = c.post("/cam/upload", files=[("files", (path.name, path.read_bytes(),
                                                "application/octet-stream"))])
    assert r.status_code == 200, r.text
    return r.json()["job_id"]


def _generate(c, job_id):
    d = c.get("/cam/defaults").json()
    return c.post(f"/cam/generate/{job_id}", json={
        "material": d["materials"][0], "tools": d["seats"],
        "sheet_w": 1220, "sheet_h": 2440})


def test_only_the_most_recent_jobs_keep_their_geometry(tmp_path):
    from fastapi.testclient import TestClient
    from backend.main import app
    from backend.routes.cam_routes import _JOBS
    from backend.routes import cam_routes
    with TestClient(app) as c:
        ids = []
        for k in range(4):
            ids.append(_upload(c, _board_step(tmp_path, f"Board_{k}")))
            assert _generate(c, ids[-1]).status_code == 200
        heavy = [j for j in ids if any(key in _JOBS[j] for key in cam_routes._HEAVY_KEYS)]
        assert len(heavy) <= cam_routes._KEEP_HEAVY, heavy
        assert ids[-1] in heavy, "the job in hand was released"
        # the NC of a released job is small and still downloadable
        old = ids[0]
        assert _JOBS[old].get("cam_nc"), "a released job lost its programs"
        # going back to a released job says so in words, as JSON
        r = _generate(c, old)
        assert r.status_code == 410
        assert "Drop the STEP files in again" in r.json()["detail"]
        r = c.get(f"/cam/viewer/{old}")
        assert r.status_code == 410 and "cleared" in r.json()["detail"]
        # the job in hand is untouched
        assert _generate(c, ids[-1]).status_code == 200


def test_upload_temp_files_are_deleted(tmp_path, monkeypatch):
    """The STEP was written to a temp file to be read and never deleted."""
    from backend.routes.cam_routes import ingest_uploads
    made = []
    real = tempfile.NamedTemporaryFile

    def spy(*a, **k):
        f = real(*a, **k)
        made.append(f.name)
        return f
    monkeypatch.setattr(tempfile, "NamedTemporaryFile", spy)
    path = _board_step(tmp_path, "Temp_Check")
    job = ingest_uploads([(path.name, path.read_bytes())])
    assert job["_step_parts"]
    assert made and not any(Path(m).exists() for m in made), made


def test_the_memory_page_reports_what_is_held(tmp_path):
    from fastapi.testclient import TestClient
    from backend.main import app
    with TestClient(app) as c:
        _upload(c, _board_step(tmp_path, "Mem_Page"))
        m = c.get("/cam/memory").json()
    assert m["limit_mb"] != 0 and m["jobs"] >= 1
    assert 1 <= m["jobs_holding_geometry"] <= m["keep_geometry_for"]
    assert m["rss_mb"] != 0


def test_the_allocator_helpers_are_safe_off_linux():
    from backend.services import memory_service as M
    M.give_back()                      # must never raise, on any platform
    assert isinstance(M.limit_arenas(2), bool)
