"""A boss standing up from the plate must be surfaced around, not cut off.

The part is still cut from flat stock: the stock top is the BOSS top, and
everything inside the profile is milled down to its own level — the main field
becomes a pocket over the whole footprint with the boss left as an island.

Before this worked, such a part was read with the wrong thickness (the boss was
ignored when picking the plate faces), and the boss's base ring on the top face
classified as an INSIDE cut-out — the machine would have cut the boss off.

The primary path — 2D parts with through features and ordinary pockets — must
not change at all: the boss branch only runs when faces stand proud of the
plate, which a normal part never has.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("OCP", reason="OpenCascade not installed")

from backend.services import wood_geometry as WG      # noqa: E402

W, H = 300.0, 200.0
PLATE_T = 16.0        # the field everything is milled down to
STOCK_T = 18.0        # boss top = stock top
BOSS_W, BOSS_H = 40.0, 30.0


def _fuse(a, b):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    op = BRepAlgoAPI_Fuse(a, b)
    op.Build()
    return op.Shape()


def _cut(a, b):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    return BRepAlgoAPI_Cut(a, b).Shape()


def _box(x, y, z, dx, dy, dz):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def _cyl(x, y, r, z0, h):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, h).Shape()


def _boss_part():
    """A 300x200 plate at 16mm with a 40x30 boss up to the 18mm stock top,
    a through hole, and an ordinary 6mm-deep pocket in the field."""
    s = _box(0, 0, 0, W, H, PLATE_T)
    s = _fuse(s, _box(60, 80, 0, BOSS_W, BOSS_H, STOCK_T))       # boss to stock top
    s = _cut(s, _cyl(250, 100, 6.0, -1, STOCK_T + 2))            # through hole
    s = _cut(s, _cyl(160, 50, 15.0, PLATE_T - 6.0, 20.0))        # 6mm pocket in the field
    return s


@pytest.fixture(scope="module")
def geom():
    return WG.part_geometry(_boss_part())


def test_the_stock_is_the_boss_top(geom):
    """Thickness must span bottom to BOSS top — 18mm stock, not the 16mm field.
    Reading 16 would cut the whole part from the wrong board."""
    assert geom["thickness"] == pytest.approx(STOCK_T, abs=0.1)
    assert geom["unmatched_pockets"] == 0
    assert not geom["two_sided"]


def test_the_outline_is_the_full_footprint(geom):
    out = next(lp for lp in geom["loops"] if lp["role"] == "OUTSIDE")
    xs = [q[0] for q in out["pts"]]
    ys = [q[1] for q in out["pts"]]
    dims = sorted((max(xs) - min(xs), max(ys) - min(ys)))
    assert dims == pytest.approx([H, W], abs=0.5), \
        "the outline must be the plate footprint, not the boss top"


def test_the_field_is_surfaced_with_the_boss_as_an_island(geom):
    pockets = [lp for lp in geom["loops"] if lp["role"] == "POCKET"]
    field = max(pockets, key=lambda p: WG._area2d(p["pts"]))
    assert field["depth"] == pytest.approx(STOCK_T - PLATE_T, abs=0.1)
    isl = field.get("islands") or []
    assert len(isl) >= 1, "the boss was not kept as an island — it would be milled away"
    areas = [WG._area2d(w) for w in isl]
    assert any(abs(a - BOSS_W * BOSS_H) < 0.1 * BOSS_W * BOSS_H for a in areas), \
        f"no island the size of the boss in {areas}"


def test_the_boss_base_is_not_a_cutout(geom):
    """The old failure: the boss's base ring classified INSIDE and the machine
    cut it off."""
    for lp in geom["loops"]:
        if lp["role"] != "INSIDE":
            continue
        a = WG._area2d(lp["pts"])
        assert abs(a - BOSS_W * BOSS_H) > 0.1 * BOSS_W * BOSS_H, \
            "a cut-out the size of the boss — the boss would be cut off"


def test_ordinary_features_survive_alongside_the_boss(geom):
    radii = [round(lp["circle"][2], 1) for lp in geom["loops"]
             if lp["role"] == "DRILL" and lp.get("circle")]
    assert 6.0 in radii, f"the through hole is missing: {radii}"
    pockets = [lp for lp in geom["loops"] if lp["role"] == "POCKET"]
    # the ordinary pocket: floor 6mm below the FIELD = 8mm below the stock top
    deep = [p for p in pockets if abs(p["depth"] - 8.0) < 0.2]
    assert deep, f"the 6mm field pocket is missing: {[p['depth'] for p in pockets]}"


def test_a_plain_plate_does_not_enter_the_boss_branch():
    """The primary path — 2D with through features — is byte-for-byte the old
    behaviour: no proud faces, no surfacing pocket, thickness of the plate."""
    s = _cut(_box(0, 0, 0, W, H, STOCK_T), _cyl(150, 100, 6.0, -1, STOCK_T + 2))
    g = WG.part_geometry(s)
    assert g["thickness"] == pytest.approx(STOCK_T, abs=0.1)
    assert [lp["role"] for lp in g["loops"]] == ["OUTSIDE", "DRILL"]
