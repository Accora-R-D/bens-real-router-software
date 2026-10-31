"""A through window must not be taken for an island because one of its grid
probes lands a hair inside its wall (chainstay plate, 2026-10-31: a probe 0.023 mm
in from the edge read as material and the window vanished from the cut)."""
from backend.services.wood_geometry import _interior_probes, _dist_to_ring, PROBE_EDGE_MM


def test_probe_hugging_the_wall_is_dropped():
    # The 5x5 grid over this triangle's box puts (1, 9) 0.04 mm inside the
    # sloping side: inside the ring, but too close to the wall to ask about.
    ring = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.1)]
    probes = _interior_probes(ring)
    assert (1.0, 9.0) not in probes
    assert probes, "the triangle's interior still gets probed"
    assert all(_dist_to_ring(ring, p) >= PROBE_EDGE_MM for p in probes)


def test_small_round_hole_keeps_its_probes():
    import math
    ring = [(2.0 * math.cos(t), 2.0 * math.sin(t))
            for t in (i * 2 * math.pi / 24 for i in range(24))]      # O4 hole
    assert len(_interior_probes(ring)) >= 5
