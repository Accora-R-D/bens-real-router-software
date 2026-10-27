"""Depth-aware extraction: pockets, islands, holes and stepped recesses.

The sample STEPs in the repo are laser-cut steel plate and have no blind
features, so they can only prove the new path agrees with the old one. Pocket
detection itself is proved against solids built here, where the answer is known
exactly: a 5 mm pocket must come back as 5 mm, and an island must survive, or
the router either cuts air or machines away a boss that was meant to stay.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))

pytest.importorskip("OCP", reason="OpenCascade not installed")

from backend.services import wood_geometry as G       # noqa: E402
from backend.services import wood_cam_service as W    # noqa: E402

PLATE_W, PLATE_H, PLATE_T = 200.0, 100.0, 18.0


def _box(x, y, z, dx, dy, dz):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def _cut(a, b):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    return BRepAlgoAPI_Cut(a, b).Shape()


def _cyl(x, y, z, r, h):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1)), r, h).Shape()


def _plate():
    return _box(0, 0, 0, PLATE_W, PLATE_H, PLATE_T)


def _roles(g):
    out = {}
    for lp in g["loops"]:
        out[lp["role"]] = out.get(lp["role"], 0) + 1
    return out


def _extents(pts):
    """Sorted [short, long] size of a loop — rotation-agnostic."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return sorted([max(xs) - min(xs), max(ys) - min(ys)])


# ------------------------------------------------------------------- basics

def test_plain_plate_has_a_profile_and_nothing_else():
    g = G.part_geometry(_plate())
    assert _roles(g) == {"OUTSIDE": 1}
    assert g["thickness"] == pytest.approx(PLATE_T, abs=0.01)
    assert g["fragmented"] is False


def test_a_blind_pocket_arrives_with_its_depth():
    """5 mm deep recess in the top face."""
    shape = _cut(_plate(), _box(40, 30, PLATE_T - 5, 60, 40, 10))
    g = G.part_geometry(shape)
    assert _roles(g) == {"OUTSIDE": 1, "POCKET": 1}
    pk = next(lp for lp in g["loops"] if lp["role"] == "POCKET")
    assert pk["depth"] == pytest.approx(5.0, abs=0.05)
    # The mouth must be the real 60 x 40, not the plate outline. Compared as
    # sorted extents because the machined frame is the engine's (make_basis),
    # which is rotated relative to world X/Y — the part simply arrives at the
    # nester turned, which is what the nester rotates anyway.
    assert _extents(pk["pts"]) == pytest.approx([40.0, 60.0], abs=0.1)


def test_a_through_hole_is_a_drill_not_a_pocket():
    """A through feature has no floor, so it must never read as a pocket."""
    shape = _cut(_plate(), _cyl(150, 50, -1, 4.0, PLATE_T + 2))
    g = G.part_geometry(shape)
    assert _roles(g) == {"OUTSIDE": 1, "DRILL": 1}
    dr = next(lp for lp in g["loops"] if lp["role"] == "DRILL")
    cx, cy, r = dr["circle"]
    assert r == pytest.approx(4.0, abs=0.05)


def test_pocket_hole_and_cutout_together():
    shape = _plate()
    shape = _cut(shape, _box(20, 25, PLATE_T - 6, 50, 50, 10))     # pocket
    shape = _cut(shape, _cyl(150, 50, -1, 4.0, PLATE_T + 2))       # through hole
    shape = _cut(shape, _box(100, 20, -1, 18, 12, PLATE_T + 2))    # through cut-out
    g = G.part_geometry(shape)
    r = _roles(g)
    assert r["OUTSIDE"] == 1
    assert r["POCKET"] == 1
    assert r["DRILL"] == 1
    assert r["INSIDE"] == 1
    pk = next(lp for lp in g["loops"] if lp["role"] == "POCKET")
    assert pk["depth"] == pytest.approx(6.0, abs=0.05)


def test_an_island_in_a_pocket_survives():
    """A boss standing in the recess must be kept, or the clearing machines it
    away instead of going around it."""
    island = _box(60, 45, PLATE_T - 5, 15, 15, 10)
    recess = _cut(_box(40, 30, PLATE_T - 5, 70, 45, 10), island)
    g = G.part_geometry(_cut(_plate(), recess))
    pk = next(lp for lp in g["loops"] if lp["role"] == "POCKET")
    assert pk["depth"] == pytest.approx(5.0, abs=0.05)
    assert pk.get("islands"), "the boss was lost"
    assert _extents(pk["islands"][0]) == pytest.approx([15.0, 15.0], abs=0.1)


def test_an_island_part_is_not_rejected_as_fragmented():
    """Regression: the bridge used to reject any part whose machined face had
    coplanar neighbours. An island's top face is coplanar with the main face by
    definition, so that threw away exactly the parts islands exist for — the
    pocket vanished and the part was cut as a plain plate."""
    from backend.services.wood_cam_service import _read_geometry

    island = _box(60, 45, PLATE_T - 5, 15, 15, 10)
    recess = _cut(_box(40, 30, PLATE_T - 5, 70, 45, 10), island)
    shape = _cut(_plate(), recess)

    class _P:
        name = "Island Part"
        edges = None          # no flat fallback available, so a reject would be fatal
    p = _P()
    p.shape = shape

    loops, geom, why = _read_geometry(p)
    assert geom is not None, f"fell back instead of reading the solid: {why}"
    assert geom["fragmented"] is True, "the island should still REPORT as coplanar"
    pockets = [lp for lp in loops if lp["role"] == "POCKET"]
    assert len(pockets) == 1
    assert pockets[0].get("islands"), "the island was lost"


def test_stepped_pockets_at_two_depths():
    """Two recesses at different depths must each keep their own depth."""
    shape = _plate()
    shape = _cut(shape, _box(20, 20, PLATE_T - 4, 50, 60, 10))
    shape = _cut(shape, _box(110, 20, PLATE_T - 9, 50, 60, 12))
    g = G.part_geometry(shape)
    depths = sorted(lp["depth"] for lp in g["loops"] if lp["role"] == "POCKET")
    assert len(depths) == 2
    assert depths[0] == pytest.approx(4.0, abs=0.05)
    assert depths[1] == pytest.approx(9.0, abs=0.05)


def test_a_pocket_in_the_underside_flips_the_part():
    """Machined pocket-side up: a bottom-only recess must set flip."""
    shape = _cut(_plate(), _box(40, 30, -1, 60, 40, 1 + 5))   # 5 mm into the BOTTOM
    g = G.part_geometry(shape)
    assert g["up_side"] == "BOT"
    assert g["flip"] is True
    assert g["two_sided"] is False
    pk = next(lp for lp in g["loops"] if lp["role"] == "POCKET")
    assert pk["depth"] == pytest.approx(5.0, abs=0.05)


def test_pockets_on_both_faces_are_reported_as_two_sided():
    """Two-sided needs a second setup; only the up side is cut."""
    shape = _plate()
    shape = _cut(shape, _box(20, 30, PLATE_T - 5, 50, 40, 10))
    shape = _cut(shape, _box(120, 30, -1, 50, 40, 1 + 5))
    g = G.part_geometry(shape)
    assert g["two_sided"] is True
    assert g["up_side"] == "TOP"


# ------------------------------------------------- straight through the post

def test_a_pocketed_part_posts_and_passes_the_safety_gate():
    import cam_syntec
    import verify
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_MATERIALS, DEFAULT_TOOL_TABLE

    shape = _plate()
    shape = _cut(shape, _box(20, 25, PLATE_T - 6, 50, 50, 10))
    shape = _cut(shape, _cyl(150, 50, -1, 4.0, PLATE_T + 2))
    g = G.part_geometry(shape)

    material = next(m for m in DEFAULT_MATERIALS if m.thickness_mm == PLATE_T)
    meta = {"flip": g["flip"], "two_sided": g["two_sided"], "up_side": g["up_side"],
            "pockets": g["pockets"], "label": "POCKET-TEST", "qty": 1,
            "thickness": PLATE_T}
    res = generate_from_parts([("POCKET-TEST", g["loops"], meta)], [], "pocket-job",
                              material, DEFAULT_TOOL_TABLE,
                              sheet_w=1220.0, sheet_h=800.0, prefix="POCKET")
    assert res.manifest.get("safety_gate") == "PASS", res.manifest
    assert res.nc.strip().endswith("M30")
    # the pocket must actually be machined, not merely carried through
    sheet = res.layout[0]
    part = sheet["parts"][0]
    assert part["pockets"], "the pocket did not reach the toolpath layout"
    print(f"\n  pocketed part: {len(res.nc.splitlines())} NC lines, "
          f"gate {res.manifest.get('safety_gate')}, "
          f"{len(part['pockets'])} pocket(s) in the layout")
