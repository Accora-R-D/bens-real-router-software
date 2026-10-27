"""A counterbore must come out as a recess PLUS a drilled bore.

Regression for geometry that destroyed parts. A counterbore's floor is an
annulus, so its face area is pi(R^2 - r^2) while the mouth ring above it is
pi R^2. The mouth was matched against the floor's NET area, so the match failed
by exactly the bore's cross-section — over the 50mm2 tolerance for any bore
above about O8. The mouth then fell through to the circle test and was drilled
STRAIGHT THROUGH the plate at counterbore diameter, and the unmatched pocket
tipped the whole part onto the flat-outline fallback, losing every other recess
with it.

The other half is telling a bore from a boss. Both are inner wires on the pocket
floor and look identical in 2D. Treating the bore as an island machined around
it and never drilled it; treating a boss as a bore would cut it away. The solid
is probed instead.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("OCP", reason="OpenCascade not installed")

from backend.services import wood_geometry as WG      # noqa: E402

W, H, T = 300.0, 200.0, 18.0


def _cyl(x, y, r, z0, h):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, h).Shape()


def _plate():
    """18mm plate with a counterbore, a recess round a boss, and a plain hole."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), W, H, T).Shape()
    # counterbore: O20 recess 6 deep, O10 through its floor
    s = BRepAlgoAPI_Cut(s, _cyl(70, 100, 10.0, T - 6.0, 7.0)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(70, 100, 5.0, -1, T + 2)).Shape()
    # a genuine island: O40 recess 6 deep with a O16 boss left standing full height
    ring = _cyl(170, 100, 20.0, T - 6.0, 7.0)
    boss = _cyl(170, 100, 8.0, T - 6.0, 7.0)
    s = BRepAlgoAPI_Cut(s, BRepAlgoAPI_Cut(ring, boss).Shape()).Shape()
    # a plain through hole
    s = BRepAlgoAPI_Cut(s, _cyl(260, 100, 6.0, -1, T + 2)).Shape()
    return s


@pytest.fixture(scope="module")
def geom():
    return WG.part_geometry(_plate())


def _roles(geom, role):
    return [lp for lp in geom["loops"] if lp["role"] == role]


def test_every_recess_is_matched(geom):
    """An unmatched pocket tips the whole part onto the flat outline, which
    silently loses every blind feature on it."""
    assert geom["unmatched_pockets"] == 0


def test_the_counterbore_is_a_recess_not_a_through_hole(geom):
    pockets = _roles(geom, "POCKET")
    assert len(pockets) == 2, "expected the counterbore and the boss recess"
    cb = min(pockets, key=lambda p: WG._area2d(p["pts"]))
    assert cb["depth"] == pytest.approx(6.0, abs=0.1)
    assert WG._area2d(cb["pts"]) == pytest.approx(314.0, rel=0.05)   # O20 mouth
    assert cb.get("through") is False


def test_the_bore_under_the_counterbore_is_drilled(geom):
    """It used to be handed over as an island — machined around, never cut."""
    radii = sorted(round(lp["circle"][2], 2) for lp in _roles(geom, "DRILL"))
    assert 5.0 in radii, f"no O10 bore in the drilled holes {radii}"
    cb = min(_roles(geom, "POCKET"), key=lambda p: WG._area2d(p["pts"]))
    assert not cb.get("islands"), "the bore was kept as an island"


def test_nothing_is_bored_through_at_recess_diameter(geom):
    """The failure this guards: O20 and O40 mouths drilled clean through."""
    radii = [round(lp["circle"][2], 2) for lp in _roles(geom, "DRILL")]
    assert 10.0 not in radii, "the counterbore mouth was drilled through"
    assert 20.0 not in radii, "the boss recess was drilled through"


def test_a_real_boss_is_still_kept_as_an_island(geom):
    big = max(_roles(geom, "POCKET"), key=lambda p: WG._area2d(p["pts"]))
    assert len(big.get("islands") or []) == 1, "the standing boss was lost"


def test_the_plain_through_hole_survives(geom):
    radii = sorted(round(lp["circle"][2], 2) for lp in _roles(geom, "DRILL"))
    assert 6.0 in radii


def _swing_arm_plate():
    """The swing-arm board's failure in miniature: three identical counterbores
    and two through holes whose mouth area sits within the 50mm2 match
    tolerance of the counterbore's (O12 thru = 113mm2 vs O11.2 mouth = 99mm2)."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), W, H, T).Shape()
    for x, y in ((40, 40), (60, 160), (90, 90)):        # three O11.24 x 6 cbores
        s = BRepAlgoAPI_Cut(s, _cyl(x, y, 5.62, T - 6.0, 7.0)).Shape()
        s = BRepAlgoAPI_Cut(s, _cyl(x, y, 3.2, -1, T + 2)).Shape()
    for x, y in ((200, 60), (250, 150)):                # two O12 THRU holes
        s = BRepAlgoAPI_Cut(s, _cyl(x, y, 6.0, -1, T + 2)).Shape()
    return s


def test_a_floor_only_matches_the_mouth_directly_above_it():
    """Matched by area alone, the two O12 through holes each claimed a
    counterbore floor from across the part: two phantom 6mm pockets where the
    router should drill through, and two real counterbores left undetected —
    drilled straight through at counterbore diameter."""
    g = WG.part_geometry(_swing_arm_plate())
    pockets = _roles(g, "POCKET")
    assert len(pockets) == 3, f"expected all 3 counterbores, got {len(pockets)}"
    for p in pockets:
        assert p["depth"] == pytest.approx(6.0, abs=0.1)
        assert WG._area2d(p["pts"]) == pytest.approx(99.2, rel=0.05)
    assert g["unmatched_pockets"] == 0
    drills = sorted(round(lp["circle"][2], 2) for lp in _roles(g, "DRILL"))
    assert drills == [3.2, 3.2, 3.2, 6.0, 6.0], \
        f"drilled {drills} — a thru hole was turned into a pocket or a " \
        "counterbore mouth was drilled through"
