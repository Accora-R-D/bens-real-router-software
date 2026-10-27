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
    sys.path.insert(0, str(ROOT / "cam" / "engine"))
    import simulate
    nc = "\n".join(["G00X0Y0Z10", "G01X100Y0Z10F1000", "G01X100Y100Z10F100", "G00X0Y0Z10"])
    st = simulate.stats(simulate.parse_segments(nc), tool_changes=1)
    # 100 mm at 1000 + 100 mm at 100 = 0.1 + 1.0 min; rapid back ~141 mm; 20 s change
    expect = 0.1 + 1.0 + (2 ** 0.5 * 100) / simulate.RAPID_RATE + 20 / 60
    assert st["minutes"] == pytest.approx(expect, rel=1e-6)
