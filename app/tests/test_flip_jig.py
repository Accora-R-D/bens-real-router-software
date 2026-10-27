"""Two-sided 2.5D parts: the flip-jig (inverse mould) second pass.

A part with recesses on BOTH faces used to be cut one-sided with a warning.
Now pass 1 cuts the face-up side plus an inverse mould — the part's far-face
outline as a snug through-cavity anywhere in the nest — and a SEPARATE side-2
program cuts only the far face's recesses. The operator flips the finished
part into the cavity between passes; the sheet never moves, so registration
comes from the fit alone.

The registration invariant this pins: the far face is read in its own outward
frame (the mirrored view the machine sees after the flip), and the pass-2
features ride the mould's placement transform — so a recess modelled at
(x, y) on a W-wide part must land at (W - x, y) relative to the mould corner.
"""

import math
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "cam"))
sys.path.insert(0, str(ROOT / "cam" / "engine"))
sys.path.insert(0, str(ROOT / "cam" / "router_cam"))

pytest.importorskip("OCP", reason="OpenCascade not installed")

W, H, T = 300.0, 200.0, 18.0


def _cyl(x, y, r, z0, h):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    return BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, h).Shape()


def _double_board():
    """Counterbore O20x6 from the TOP at (70,100); from the BOTTOM an O24x5
    counterbore at (230,50) and an O16x3 recess at (80,150); a plain O8
    through hole. Two distinct bottom features at generic positions make the
    registration check below sensitive to mirroring."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt
    s = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), W, H, T).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(70, 100, 10.0, T - 6.0, 7.0)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(70, 100, 5.0, -1, T + 2)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(230, 50, 12.0, -1.0, 6.0)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(230, 50, 5.0, -1, T + 2)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(80, 150, 8.0, -1.0, 4.0)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(150, 150, 4.0, -1, T + 2)).Shape()
    return s


@pytest.fixture(scope="module")
def result():
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = types.SimpleNamespace(name="DoubleBoard", qty=1, thickness_mm=T,
                                 shape=_double_board(), part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"DoubleBoard"})
    assert parts[0][2].get("backside_loops"), "far side was not extracted"
    return generate_from_parts(parts, diag, "flipjig-test",
                               DEFAULT_MATERIALS[0], DEFAULT_TOOL_TABLE,
                               sheet_w=1220, sheet_h=2440)


def test_two_programs_both_gated(result):
    kinds = [(p["thickness"], bool(p.get("side2"))) for p in result.programs]
    assert kinds == [(T, False), (T, True)]
    for p in result.programs:
        assert p["manifest"]["safety_gate"] == "PASS", \
            (p.get("side2"), p["manifest"].get("safety_gate_reasons"))


def test_mould_is_nested_and_flagged(result):
    lay = result.programs[0]["layout"][0]
    assert lay.get("two_pass") is True
    moulds = [lp for lp in lay["parts"] if lp.get("mould")]
    assert len(moulds) == 1
    mx = [q[0] for q in moulds[0]["outline"]]
    my = [q[1] for q in moulds[0]["outline"]]
    dims = sorted([max(mx) - min(mx), max(my) - min(my)])
    assert dims == pytest.approx([H, W], abs=0.5), "mould is not the part's footprint"


def _rel_positions(mould):
    """Each pass-2 pocket's (diameter, centre) relative to the mould bbox corner."""
    mx = [q[0] for q in mould["outline"]]; my = [q[1] for q in mould["outline"]]
    out = {}
    for pk in mould["pockets"]:
        xs = [q[0] for q in pk["outline"]]; ys = [q[1] for q in pk["outline"]]
        dia = round(max(xs) - min(xs))
        out[dia] = ((max(xs) + min(xs)) / 2 - min(mx),
                    (max(ys) + min(ys)) / 2 - min(my), pk["depth"])
    return out


def _under_rot(pos, k):
    """Model-frame (x, y) inside the W x H rectangle -> position relative to the
    rotated rectangle's bbox corner, for a 90-degree-step rotation k."""
    x, y = pos
    return [(x, y), (H - y, x), (W - x, H - y), (y, W - x)][k % 4]


def test_pass2_features_land_mirrored_on_the_mould(result):
    """The bottom-face recesses modelled at (230,50) and (80,150) must present
    at their MIRRORED positions (W-x, y) relative to the mould — where the
    physically flipped part puts them — under whatever rotation the nest chose
    for the mould. The unmirrored positions must match under NO rotation: a
    match there would cut the flipped part's features in the wrong places."""
    lay = result.programs[0]["layout"][0]
    mould = next(lp for lp in lay["parts"] if lp.get("mould"))
    got = _rel_positions(mould)
    assert set(got) == {24, 16}, f"pass-2 pockets {set(got)}"
    assert got[24][2] == pytest.approx(5.0, abs=0.1)
    assert got[16][2] == pytest.approx(3.0, abs=0.1)

    def matches(expect):
        for k in range(4):
            if all(abs(got[d][0] - _under_rot(e, k)[0]) < 0.3
                   and abs(got[d][1] - _under_rot(e, k)[1]) < 0.3
                   for d, e in expect.items()):
                return True
        return False

    mirrored = {24: (W - 230.0, 50.0), 16: (W - 80.0, 150.0)}
    unmirrored = {24: (230.0, 50.0), 16: (80.0, 150.0)}
    assert matches(mirrored), f"pass-2 features not at the flipped positions: {got}"
    assert not matches(unmirrored), \
        "pass-2 features match the UNFLIPPED view — they would be cut mirrored"


def test_side2_cuts_only_recesses_never_through(result):
    import re
    side2 = result.programs[1]
    zs = [float(m.group(1)) for ln in side2["nc"].splitlines()
          for m in [re.search(r"G0[123].*?Z([-\d.]+)", ln)] if m]
    assert min(zs) == pytest.approx(T - 5.0, abs=0.05), \
        f"side 2 deepest cut {min(zs)} — expected the 5mm counterbore floor only"


def test_boss_topped_part_still_gets_a_part_sized_mould():
    """A boss on the top face puts the top read on the proud path — and the
    forced far-face read used to take that path too, building the mould from
    the BOSS outline (40x40 instead of 200x120): the flipped part could never
    seat. The far face is a clean full-footprint plane and must read as one."""
    import types
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
    from OCP.gp import gp_Pnt
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    body = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 120, 15.0).Shape()
    boss = BRepPrimAPI_MakeBox(gp_Pnt(80, 40, 15.0), 40, 40, 3.0).Shape()
    op = BRepAlgoAPI_Fuse(body, boss); op.Build()
    s = op.Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(40, 60, 10.0, -1.0, 6.0)).Shape()   # cavity below
    s = BRepAlgoAPI_Cut(s, _cyl(170, 60, 4.0, -1, 20)).Shape()
    part = types.SimpleNamespace(name="BossBack", qty=1, thickness_mm=18.0,
                                 shape=s, part_number="")
    parts, diag = WCS.cam_parts_and_diag([part], names={"BossBack"})
    res = generate_from_parts(parts, diag, "flipjig-boss",
                              DEFAULT_MATERIALS[0], DEFAULT_TOOL_TABLE,
                              sheet_w=1220, sheet_h=2440)
    assert [bool(p.get("side2")) for p in res.programs] == [False, True]
    for p in res.programs:
        assert p["manifest"]["safety_gate"] == "PASS"
    lay = res.programs[0]["layout"][0]
    mould = next(lp for lp in lay["parts"] if lp.get("mould"))
    mx = [q[0] for q in mould["outline"]]; my = [q[1] for q in mould["outline"]]
    dims = sorted([max(mx) - min(mx), max(my) - min(my)])
    assert dims == pytest.approx([120.0, 200.0], abs=0.5), \
        f"mould is {dims} — built from the boss, not the part footprint"
    # side 2 cuts the O20 x 5 cavity: deepest Z = 18 - 5 = 13
    import re
    zs = [float(m.group(1)) for ln in res.programs[1]["nc"].splitlines()
          for m in [re.search(r"G0[123].*?Z([-\d.]+)", ln)] if m]
    assert min(zs) == pytest.approx(13.0, abs=0.05)


def _boss_carried_part():
    """The workshop's worry case: a part carried by a small full-thickness boss
    (40x40 on 200x120 — under 7% contact) with a cavity on the far side. A
    plain through-cavity would leave it perched on the boss alone."""
    import types
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
    from OCP.gp import gp_Pnt
    body = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 200, 120, 15.0).Shape()
    boss = BRepPrimAPI_MakeBox(gp_Pnt(30, 40, 15.0), 40, 40, 3.0).Shape()
    op = BRepAlgoAPI_Fuse(body, boss); op.Build()
    s = op.Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(150, 60, 10.0, -1.0, 6.0)).Shape()
    s = BRepAlgoAPI_Cut(s, _cyl(170, 100, 4.0, -1, 20)).Shape()
    return types.SimpleNamespace(name="BossCarried", qty=1, thickness_mm=18.0,
                                 shape=s, part_number="")


@pytest.fixture(scope="module")
def boss_result():
    from backend.services import wood_cam_service as WCS
    from router_cam.engine import generate_from_parts
    from router_cam.models import DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS
    part = _boss_carried_part()
    parts, diag = WCS.cam_parts_and_diag([part], names={part.name})
    return generate_from_parts(parts, diag, "flipjig-inverse",
                               DEFAULT_MATERIALS[0], DEFAULT_TOOL_TABLE,
                               sheet_w=1220, sheet_h=2440)


def test_small_contact_part_gets_a_sculpted_inverse_mould(boss_result):
    """Under MOULD_SUPPORT_MIN contact the mould must be the part's 2.5D
    inverse: a floor at (stock - field drop) carrying the thin regions, a
    through void where the boss lands, corner reliefs — and both gates PASS."""
    for p in boss_result.programs:
        assert p["manifest"]["safety_gate"] == "PASS", \
            (p.get("side2"), p["manifest"].get("safety_gate_reasons"))
    lay = boss_result.programs[0]["layout"][0]
    mould = next(lp for lp in lay["parts"] if lp.get("mould"))
    depths = sorted({round(pk["depth"], 1) for pk in mould["pockets"]})
    # 15 = supporting floor (18 - 3 field drop); 18.5 = through voids;
    # 5 = the pass-2 cavity
    assert depths == [5.0, 15.0, 18.5], f"mould levels {depths}"
    floor = max((pk for pk in mould["pockets"]), key=lambda pk:
                (max(q[0] for q in pk["outline"]) - min(q[0] for q in pk["outline"])) *
                (max(q[1] for q in pk["outline"]) - min(q[1] for q in pk["outline"])))
    assert floor["depth"] == pytest.approx(15.0, abs=0.05), \
        "the big region must be the supporting floor, not a through void"
    # a through void close to the boss size exists (boss + clearance)
    voids = [pk for pk in mould["pockets"] if pk["depth"] > 18]
    assert any(abs((max(q[0] for q in pk["outline"]) -
                    min(q[0] for q in pk["outline"])) - 42.0) < 1.0
               for pk in voids), "no boss-sized through void in the mould"


def test_plenty_of_contact_keeps_the_fast_plain_mould(result):
    """The flat double board touches the spoilboard over ~97% of its face —
    its mould must stay the plain through-cavity (one INSIDE cut), not the
    slow sculpted inverse."""
    lay = result.programs[0]["layout"][0]
    mould = next(lp for lp in lay["parts"] if lp.get("mould"))
    # a plain mould previews only its pass-2 features; a sculpted one would
    # also carry the near-full-footprint floor region
    big = [pk for pk in mould["pockets"]
           if (max(q[0] for q in pk["outline"]) -
               min(q[0] for q in pk["outline"])) > 100]
    assert not big, "footprint-sized mould pocket — the plain mould was sculpted"


def test_pass1_still_cuts_the_top_side_and_the_part_through(result):
    import re
    p1 = result.programs[0]
    zs = [float(m.group(1)) for ln in p1["nc"].splitlines()
          for m in [re.search(r"G0[123].*?Z([-\d.]+)", ln)] if m]
    assert min(zs) < 0, "pass 1 must cut the profile through"
    # the TOP counterbore floor at Z12 is cut in pass 1
    assert any(abs(z - (T - 6.0)) < 0.05 for z in zs), "top-side counterbore missing"
