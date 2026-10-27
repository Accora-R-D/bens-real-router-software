"""Forcing a seat for a feature is deliberate, so the program says so.

The operator can force the tool used for a feature — a long-flute bit in a
spare seat for holes the standard one cannot reach through. It is confirmed
in a dialog on the page; here the program itself must carry the fact, the
way an overridden thickness does: first in the warnings and on the job sheet.
"""

import os
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


def _board():
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    return BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 300, 200, 18).Shape()


def _run(tool_map):
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = types.SimpleNamespace(name="board", qty=1, thickness_mm=18.0,
                                 shape=_board(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"board"})
    return generate_from_parts(parts, diag, "override-test", DEFAULT_MATERIALS[0],
                               DEFAULT_TOOL_TABLE, sheet_w=1220, sheet_h=2440,
                               tool_map=tool_map)


def test_a_forced_seat_leads_the_warnings_and_is_on_the_job_sheet():
    res = _run({"profile": 7})
    prog = res.programs[0]
    assert prog["warnings"] and prog["warnings"][0].startswith("! TOOL OVERRIDE"), prog["warnings"]
    assert "outer profiles -> T07" in prog["warnings"][0]
    assert prog["manifest"]["tool_overrides"] == {"outer profiles": "T07 (Ø4mm)"}
    assert "TOOL OVERRIDE in force" in prog["job_sheet"]
    assert "outer profiles: T07" in prog["job_sheet"]
    # and it really cut the profile with the Ø4
    assert "T7M06" in prog["nc"]


def test_auto_leaves_no_trace():
    res = _run({})
    prog = res.programs[0]
    assert not any(w.startswith("! TOOL OVERRIDE") for w in prog["warnings"])
    assert "tool_overrides" not in prog["manifest"]
    assert "TOOL OVERRIDE" not in prog["job_sheet"]
