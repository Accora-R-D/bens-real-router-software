"""Two things the operator saw on the bed (2026-09-16, a 15 mm MDF sheet).

The cutter traversed between parts a few millimetres over the sheet — fine on a
flat board, a collision on a bowed one — and the V-bit engraved just ABOVE the
surface, so the sheet came off with no marking. Rapids now sit 15 mm over the
nominal top, and every engrave stroke bites 1 mm below it on top of its own depth.
"""

import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))

import cam_syntec as cs                                   # noqa: E402
from router_cam.models import Material, DEFAULT_MATERIALS  # noqa: E402

TOP = 18.0
CLEAR = TOP + 15.0


def _mat():
    cs.MAT.clear()
    cs.MAT.update(dict(cut_feed=6300, plunge_feed=2500, max_step=6.0, final_z=-0.5,
                       clearance_over=15.0, spindle=18000))


def _zs(lines, code="G01"):
    # lines carry N-numbers: "N20G01Z16.400F2500.0"
    return [float(m.group(1)) for ln in lines
            for m in [re.search(rf"^(?:N\d+)?{code}[^Z]*Z([-\d.]+)", ln)] if m]


class TestRapidsClearABowedSheet:
    def test_every_material_rapids_fifteen_over_the_stock(self):
        assert Material().clearance_over == 15.0
        for m in DEFAULT_MATERIALS:
            assert m.clearance_over == 15.0, m.name

    def test_the_clearance_reaches_the_post(self):
        """The Material's value is what the post rapids at, not the engine's
        own 6 mm default."""
        assert Material("18mm MDF", 18.0).mat_dict()["clearance_over"] == 15.0


class TestEngravingBitesIntoTheWood:
    def test_a_single_stroke_cuts_one_millimetre_deeper_than_its_depth(self):
        _mat()
        p = cs.Prog()
        p.engrave([(0.0, 0.0), (10.0, 0.0)], CLEAR, TOP, 0.6)
        plunges = _zs(p.lines)
        assert plunges, p.lines
        assert plunges[0] == pytest.approx(TOP - 0.6 - 1.0)

    def test_a_label_of_strokes_does_the_same(self):
        _mat()
        p = cs.Prog()
        p.engrave_group([([(0.0, 0.0), (10.0, 0.0)], 0.6),
                         ([(0.0, 5.0), (10.0, 5.0)], 0.15)], CLEAR, TOP)
        plunges = sorted(_zs(p.lines))
        assert plunges == pytest.approx(sorted([TOP - 0.6 - 1.0, TOP - 0.15 - 1.0]))
        # the hops between strokes stay above the surface, and the label ends
        # retracted to the clearance plane
        rapids = _zs(p.lines, "G00")
        assert all(z >= TOP for z in rapids), rapids
        assert rapids[-1] == pytest.approx(CLEAR)

    def test_the_smallest_text_still_reaches_the_wood(self):
        """The shallowest scaled depth plus the bite is a real groove."""
        assert cs.engrave_depth_for(3.0) + cs.ENGRAVE_BITE >= 1.15
