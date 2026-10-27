"""The wood-parts handoff: exporter STEP outlines -> router CAM Syntec G-code.

This is the seam that the whole integration rests on, so it is tested against a
REAL STEP file and the REAL post, not a synthetic outline. If the loop mapping
is wrong the failure is a wrong toolpath, which is not something to discover at
the machine.

The safety gate is the assertion that matters: `verify.check` is what gates the
"send to machine" button in the CAM app, so a program that fails it here would
never have been cuttable.
"""

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))            # router_cam, cam_core
sys.path.insert(0, str(ROOT / "cam" / "engine"))  # cam_syntec, nest, verify

from backend.services import wood_cam_service as W       # noqa: E402


# --------------------------------------------------------------- unit level

def _ring(cx, cy, r, n=48):
    """A closed circle, sampled the way the exporter closes its loops."""
    pts = [[cx + r * math.cos(2 * math.pi * i / n),
            cy + r * math.sin(2 * math.pi * i / n)] for i in range(n)]
    pts.append(pts[0])
    return pts


def _rect(w, h):
    return [[0, 0], [w, 0], [w, h], [0, h], [0, 0]]


def test_largest_loop_becomes_the_profile():
    loops = W.cam_loops([_ring(50, 50, 5), _rect(200, 100), _ring(150, 50, 5)])
    outer = [lp for lp in loops if lp["role"] == "OUTSIDE"]
    assert len(outer) == 1, "exactly one profile"
    assert outer[0]["isOuter"] is True
    # the rectangle is the biggest area, so it is the profile
    assert len(outer[0]["pts"]) == 4


def test_round_holes_become_drills_with_a_centre_and_radius():
    loops = W.cam_loops([_rect(200, 100), _ring(50, 50, 5), _ring(150, 50, 3.2)])
    drills = [lp for lp in loops if lp["role"] == "DRILL"]
    assert len(drills) == 2
    got = sorted((round(c["circle"][0], 2), round(c["circle"][1], 2),
                  round(c["circle"][2], 2)) for c in drills)
    assert got == [(50.0, 50.0, 5.0), (150.0, 50.0, 3.2)]


def test_a_slot_is_not_mistaken_for_a_hole():
    """A rounded slot must stay an INSIDE cut-out — drilling it would plunge
    once in the middle and leave the slot uncut."""
    slot = [[20, 45], [60, 45]]
    slot += [[60 + 5 * math.cos(a), 50 + 5 * math.sin(a)]
             for a in [(-0.5 + i / 10) * math.pi for i in range(11)]]
    slot += [[60, 55], [20, 55]]
    slot += [[20 + 5 * math.cos(a), 50 + 5 * math.sin(a)]
             for a in [(0.5 + i / 10) * math.pi for i in range(11)]]
    slot.append(slot[0])
    loops = W.cam_loops([_rect(200, 100), slot])
    assert [lp["role"] for lp in loops if not lp["isOuter"]] == ["INSIDE"]


def test_closing_vertex_is_dropped():
    """A repeated final point is a zero-length segment to the offsetter."""
    loops = W.cam_loops([_rect(200, 100)])
    pts = loops[0]["pts"]
    assert pts[0] != pts[-1]
    assert len(pts) == 4


def test_wood_names_takes_the_override():
    decisions = [
        {"name": "Ply Base", "predicted_process": "flat", "final_process": "wood"},
        {"name": "Steel Bracket", "predicted_process": "flat", "final_process": "flat"},
        {"name": "MDF Top", "predicted_process": "wood", "final_process": "wood"},
        {"name": "Changed Away", "predicted_process": "wood", "final_process": "flat"},
    ]
    assert W.wood_names(decisions) == {"Ply Base", "MDF Top"}


def test_no_wood_means_no_popup():
    """The whole trigger: nothing classified wood -> nothing handed over."""
    decisions = [{"name": "Steel Bracket", "final_process": "flat"}]
    assert W.wood_names(decisions) == set()
    assert W.summary([], decisions)["count"] == 0


# ------------------------------------------------------- real STEP end-to-end

def _sample_steps(limit=25):
    """Sample STEPs shipped in the repo, nearest-first.

    Most are tube or folded parts with no flat outline, so the caller scans for
    one that actually yields a plate rather than assuming the first file is one.
    """
    packs = ROOT / "data" / "fabrication_pack"
    return sorted(packs.glob("*/All Files/*.step"))[:limit]


@pytest.mark.skipif(not _sample_steps(1), reason="no sample STEP in the repo")
def test_real_step_part_produces_a_program_that_passes_the_safety_gate():
    ocp = pytest.importorskip("OCP", reason="OpenCascade not installed")   # noqa: F841
    from backend.services import step_ingest_service as S
    import cam_syntec
    import verify

    flats = []
    for f in _sample_steps():
        try:
            flats = [p for p in S.read_step(str(f))
                     if p.process == "flat" and getattr(p, "edges", None)]
        except Exception:
            continue
        if flats:
            break
    if not flats:
        pytest.skip("no sample STEP yielded a flat part with an outline")

    # pretend the operator called it wood
    named = {flats[0].name}
    cam = W.cam_parts(flats, names=named)
    assert cam, "the bridge produced no CAM part"

    name, loops, meta = cam[0]
    assert any(lp["role"] == "OUTSIDE" for lp in loops)
    assert meta["qty"] >= 1
    assert meta["thickness"] > 0

    thickness = meta["thickness"]
    nc, jobs, depths, _ = cam_syntec.gen_part(name, loops, thickness=thickness)
    assert nc.strip().endswith("M30"), "Syntec footer missing"
    assert jobs, "no toolpaths generated"

    out = Path(cam_syntec.OUT) / "_wood_bridge_test.NC"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(nc, encoding="utf-8")
    xs = [p[0] for lp in loops for p in lp["pts"]]
    ys = [p[1] for lp in loops for p in lp["pts"]]
    ok, fails, warns = verify.check(
        str(out), stock_top=thickness,
        sheet_w=max(xs) + 4 * cam_syntec.MARGIN,
        sheet_h=max(ys) + 4 * cam_syntec.MARGIN,
        final_z_expected=cam_syntec.MAT["final_z"],
        clearance=thickness + cam_syntec.MAT["clearance_over"])
    assert ok, f"safety gate failed: {fails}"
    assert not fails


@pytest.mark.skipif(not _sample_steps(1), reason="no sample STEP in the repo")
def test_whole_chain_step_to_nested_program():
    """STEP -> wood bridge -> generate_from_parts -> nested, gated NC.

    The per-part test above proves the loop mapping; this proves the handoff the
    popup actually performs, including that the CAM engine's own safety gate
    passes on the nested sheet rather than on a single part.
    """
    pytest.importorskip("OCP", reason="OpenCascade not installed")
    from backend.services import step_ingest_service as S
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS, DEFAULT_TOOL_TABLE

    flats = []
    for f in _sample_steps():
        try:
            flats = [p for p in S.read_step(str(f))
                     if p.process == "flat" and getattr(p, "edges", None)]
        except Exception:
            continue
        if flats:
            break
    if not flats:
        pytest.skip("no sample STEP yielded a flat part with an outline")

    cam = W.cam_parts(flats, names={p.name for p in flats})
    assert cam, "the bridge produced no CAM parts"

    # nest at the material whose thickness matches what the STEP actually is,
    # so the gate's cut-through rule is checked against the real stock
    thickness = cam[0][2]["thickness"]
    material = min(DEFAULT_MATERIALS,
                   key=lambda m: abs(m.thickness_mm - thickness))
    for name, loops, meta in cam:
        meta["thickness"] = material.thickness_mm

    res = generate_from_parts(cam, [], "test-job", material, DEFAULT_TOOL_TABLE,
                              sheet_w=2440.0, sheet_h=1220.0, prefix="WOOD")
    assert res.nc.strip().endswith("M30")
    assert res.manifest.get("safety_gate") == "PASS", res.manifest
    assert res.job_id == "test-job"
    print(f"\n  nested {len(cam)} part(s) at {material.name}: "
          f"{len(res.nc.splitlines())} NC lines, gate "
          f"{res.manifest.get('safety_gate')}")
