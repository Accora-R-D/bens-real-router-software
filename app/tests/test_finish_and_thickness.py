"""Finishing pass, and the stock-thickness override.

Both were asked for by the workshop. They are the two settings that change how
DEEP and how WIDE the machine cuts, so each is pinned against the thing it could
plausibly get wrong rather than merely exercised:

  * the finishing lap must land the wall EXACTLY where it would have been with no
    finishing at all — otherwise switching it on silently resizes every part;
  * a feature too narrow to rough at the wider offset must fall back to a single
    cut-to-size pass, or the skin becomes the only thing ever cut;
  * the finishing lap must be the LAST cut, so the part comes free at the end;
  * the override must move the through-cut depth and the stock top the safety gate
    checks against together, or the gate approves the wrong board.
"""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for sub in ("", "cam", "cam/engine", "cam/router_cam"):
    sys.path.insert(0, str(ROOT / sub) if sub else str(ROOT))

import cam_syntec as cs                                       # noqa: E402
from router_cam.models import (Material, clamp_material,      # noqa: E402
                               DEFAULT_TOOL_TABLE, DEFAULT_MATERIALS)

PLATE = [(0, 0), (100, 0), (100, 60), (0, 60)]
WINDOW = [(30, 20), (60, 20), (60, 40), (30, 40)]


@pytest.fixture(autouse=True)
def _hermetic():
    """cam_syntec is configured by module globals; restore them around each test."""
    keep = {a: getattr(cs, a) for a in
            ("TOOLS", "TOOL_PREF", "DEFAULT_TOOL", "DOGBONE", "CLEAR_POCKETS",
             "FINISH_PASS", "FINISH_ALLOWANCE", "FINISH_STEPPED")}
    cs.TOOLS.clear()
    cs.TOOLS.update({4: {"dia": 6.0, "step": 6.2, "flutes": 2, "kind": "upcut",
                     "feed": 7000.0, "plunge": 2500.0}})
    cs.TOOL_PREF[:] = [4]
    cs.DEFAULT_TOOL, cs.DOGBONE, cs.CLEAR_POCKETS = 4, False, True
    cs.FINISH_PASS, cs.FINISH_ALLOWANCE, cs.FINISH_STEPPED = False, 0.5, False
    yield
    for a, v in keep.items():
        cur = getattr(cs, a)
        if isinstance(cur, dict):
            cur.clear(); cur.update(v)
        elif isinstance(cur, list):
            cur[:] = v
        else:
            setattr(cs, a, v)


def _loops(inner=WINDOW):
    out = [{"role": "OUTSIDE", "pts": PLATE, "circle": None, "isOuter": True}]
    if inner:
        out.append({"role": "INSIDE", "pts": inner, "circle": None, "isOuter": False})
    return out


def _extent(jobs, role):
    for r, path, _pd, _t in jobs:
        if r == role:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            return (round(min(xs), 3), round(max(xs), 3),
                    round(min(ys), 3), round(max(ys), 3))
    return None


def _roles(jobs):
    return [r for r, *_ in jobs]


class TestFinishingPass:
    def test_off_by_default_nothing_changes(self):
        jobs, _ = cs.compute_jobs("p", _loops())
        assert "OUTSIDE_FIN" not in _roles(jobs)
        assert "INSIDE_FIN" not in _roles(jobs)

    def test_the_finished_wall_lands_where_it_would_with_no_finishing(self):
        """The safety of the whole feature: switching it on must not resize parts."""
        plain, _ = cs.compute_jobs("p", _loops())
        cs.FINISH_PASS = True
        fin, _ = cs.compute_jobs("p", _loops())
        assert _extent(fin, "OUTSIDE_FIN") == _extent(plain, "OUTSIDE")
        assert _extent(fin, "INSIDE_FIN") == _extent(plain, "INSIDE")

    def test_roughing_is_offset_away_from_the_wall_by_the_allowance(self):
        cs.FINISH_PASS = True
        cs.FINISH_ALLOWANCE = 0.5
        jobs, _ = cs.compute_jobs("p", _loops())
        # outer profile: tool centre further OUT, so the part keeps a 0.5 skin
        assert _extent(jobs, "OUTSIDE") == (-3.5, 103.5, -3.5, 63.5)
        # internal window: tool centre further IN, same skin on its wall
        assert _extent(jobs, "INSIDE") == (33.5, 56.5, 23.5, 36.5)

    def test_a_feature_too_narrow_to_rough_is_cut_to_size_in_one_pass(self):
        """A 6mm tool plus a 0.5 skin each side needs 7mm of slot. A 6.5mm slot
        cannot be roughed at the wider offset, and leaving a skin there would make
        the skin the only thing ever cut — so that feature keeps its single pass."""
        cs.FINISH_PASS = True
        jobs, _ = cs.compute_jobs(
            "p", _loops([(10, 10), (16.5, 10), (16.5, 50), (10, 50)]))
        assert "INSIDE_FIN" not in _roles(jobs)
        assert "OUTSIDE_FIN" in _roles(jobs)          # the profile still gets one

    def test_the_finishing_lap_is_ordered_after_its_roughing_and_cut_last(self):
        cs.FINISH_PASS = True
        roles = _roles(cs.compute_jobs("p", _loops())[0])
        assert roles.index("INSIDE") < roles.index("INSIDE_FIN")
        assert roles.index("OUTSIDE") < roles.index("OUTSIDE_FIN")
        assert roles[-1] == "OUTSIDE_FIN", "the part must come free on the last cut"

    def test_full_depth_by_default_and_stepped_on_request(self):
        """One lap is what leaves a wall with no step marks; stepped is the opt-out
        for deep boards, and must genuinely produce more Z levels."""
        cs.FINISH_PASS = True
        cs.MAT.update(DEFAULT_MATERIALS[0].mat_dict())
        jobs, _ = cs.compute_jobs("p", _loops())
        job = next(j for j in jobs if j[0] == "OUTSIDE_FIN")

        def zlevels(stepped):
            cs.FINISH_STEPPED = stepped
            prog = cs.Prog()
            prog.header(4)
            cs._emit_one(prog, job[0], job[1], job[2], False, 30.0, 18.0, job[3])
            return [ln for ln in prog.lines if "G01" in ln and "Z" in ln]

        assert len(zlevels(False)) < len(zlevels(True))


class TestThicknessOverride:
    def test_off_is_zero_and_a_fraction_of_a_millimetre_is_not_a_board(self):
        m, notes = clamp_material(Material(thickness_override=0.0))
        assert m.thickness_override == 0.0
        assert not any("override" in n for n in notes)
        m, notes = clamp_material(Material(thickness_override=0.4))
        assert m.thickness_override == 1.0
        assert any("thickness_override" in n for n in notes)

    def test_an_allowance_deeper_than_the_cut_per_pass_is_clamped(self):
        """Otherwise the finishing lap is the heavier of the two, which is backwards."""
        m, notes = clamp_material(Material(finish_pass=True, finish_allowance=4.0,
                                          max_step=2.0))
        assert m.finish_allowance == 2.0
        assert any("finish_allowance" in n for n in notes)

    def test_parts_of_different_thickness_collapse_onto_the_one_board(self):
        """Two programs claiming the same stock would have the operator cut the
        sheet twice, so the override collapses the groups, not just their names."""
        import router_cam.engine as eng
        seen = {}

        def fake(parts_loops, material, *a, **k):
            seen[material.thickness_mm] = len(parts_loops)
            return {"thickness": material.thickness_mm}

        real, eng._nest_one = eng._nest_one, fake
        try:
            parts = [("a", [], {"thickness": 12.0}), ("b", [], {"thickness": 18.0})]
            mat = replace(DEFAULT_MATERIALS[0], thickness_override=0.0)
            eng._nest_programs(parts, mat, DEFAULT_TOOL_TABLE, 1220, 2440, None, "P", 1)
            assert sorted(seen) == [12.0, 18.0]
            seen.clear()
            mat = replace(DEFAULT_MATERIALS[0], thickness_override=20.0)
            eng._nest_programs(parts, mat, DEFAULT_TOOL_TABLE, 1220, 2440, None, "P", 1)
            assert seen == {20.0: 2}
        finally:
            eng._nest_one = real


class TestTheOverrideReachesTheOperator:
    """An override that only lives in a browser form is the dangerous version of
    this feature: the file reaches the machine on a USB stick, hours later, with
    nobody remembering a checkbox. So it has to be in the filename."""

    def _client(self, overridden):
        from fastapi.testclient import TestClient
        from backend.main import app
        from backend.routes.cam_routes import _JOBS
        _JOBS["jobA"] = {
            "cam_nc": {"18": "N0G40", "18-side2": "N0G40"},
            "cam_gate": {"18": "PASS", "18-side2": "PASS"},
            "cam_override": {"18": overridden, "18-side2": overridden},
        }
        return TestClient(app)

    def test_an_overridden_board_is_named_in_the_downloaded_file(self):
        c = self._client(True)
        r = c.get("/cam/nc/jobA?thickness=18")
        assert r.status_code == 200
        cd = r.headers["content-disposition"]
        assert "18mm" in cd and "OVERRIDE" in cd, cd

    def test_the_flip_pass_is_marked_too(self):
        """It is cut from the same board, so it carries the same warning."""
        c = self._client(True)
        cd = c.get("/cam/nc/jobA?thickness=18-side2").headers["content-disposition"]
        assert "OVERRIDE" in cd and "SIDE2" in cd, cd

    def test_a_normal_job_is_not_marked(self):
        c = self._client(False)
        cd = c.get("/cam/nc/jobA?thickness=18").headers["content-disposition"]
        assert "18mm" in cd and "OVERRIDE" not in cd, cd
