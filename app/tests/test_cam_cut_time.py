"""Router Simulation tab: the estimated cut time under the preview.

Each move is timed at its own feed over its 3D length, rapids include their Z
travel, and each tool change adds ~20 s.
"""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_the_simulation_tab_shows_the_cut_time():
    page = (ROOT / "frontend" / "cam.html").read_text(encoding="utf-8")
    assert "${simTimeLine(key)}" in page and "Estimated cut time:" in page


def test_the_estimate_times_each_move_at_its_own_feed():
    """With acceleration taken out, each move costs its length over its feed,
    rapids their 3-D length, and a tool change 20 s."""
    sys.path.insert(0, str(ROOT / "cam" / "engine"))
    import simulate
    nc = "\n".join(["G00X0Y0Z10", "G01X100Y0Z10F1000", "G01X100Y100Z10F100", "G00X0Y0Z10"])
    segs = simulate.parse_segments(nc)
    feeds = [simulate.RAPID_RATE if s["kind"] == "rapid" else s["f"] for s in segs]
    t = simulate._planned_minutes(segs, feeds, accel=1e12) + simulate.TOOL_CHANGE_MIN
    expect = 0.1 + 1.0 + (2 ** 0.5 * 100) / simulate.RAPID_RATE + 20 / 60
    assert t == pytest.approx(expect, rel=1e-6)


def test_acceleration_costs_time_at_corners_not_on_straight_runs():
    sys.path.insert(0, str(ROOT / "cam" / "engine"))
    import simulate
    a = simulate.MACHINE_ACCEL
    # one straight 1000 mm cut at 6000 mm/min from rest to rest: the trapezoid
    one = simulate.parse_segments("G00X0Y0Z0\nG01X1000Y0Z0F6000")
    v = 100.0                                         # mm/s
    expect = (2 * v / a + (1000 - v * v / a) / v) / 60
    assert simulate.stats(one)["minutes"] == pytest.approx(expect, rel=1e-6)
    # the same 1000 mm as ten straight 100 mm pieces costs (almost) nothing more...
    straight = "\n".join(["G00X0Y0Z0"] + [f"G01X{100 * k}Y0Z0F6000" for k in range(1, 11)])
    assert simulate.stats(simulate.parse_segments(straight))["minutes"] == pytest.approx(expect, rel=1e-3)
    # ...but the same length through right-angle corners does not: the machine
    # all but stops at each one (50 corners, 20 mm apart)
    pts = [(0, 0)]
    for k in range(50):
        x, y = pts[-1]
        pts.append((x + 20, y) if k % 2 == 0 else (x, y + 20))
    zig = "\n".join(["G00X0Y0Z0"] + [f"G01X{x}Y{y}Z0F6000" for x, y in pts[1:]])
    flat = "\n".join(["G00X0Y0Z0"] + [f"G01X{20 * k}Y0Z0F6000" for k in range(1, 51)])
    t_zig = simulate.stats(simulate.parse_segments(zig))["minutes"]
    t_flat = simulate.stats(simulate.parse_segments(flat))["minutes"]
    assert t_zig > 1.5 * t_flat


def test_the_fitted_machine_reproduces_the_timed_base_plate():
    """Base Plate (1), 12 mm, ran 45 min on the machine (2026-10-28). Its
    program is not in the repo; this pins the constant it was fitted with."""
    sys.path.insert(0, str(ROOT / "cam" / "engine"))
    import simulate
    assert simulate.MACHINE_ACCEL == 380.0 and simulate.JUNCTION_DEV == 0.05
