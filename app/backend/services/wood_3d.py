"""3D jobs: a shaped part cut as one heightfield from the side it is seen from.

Why this is separate
--------------------
The flat path (`wood_geometry`) reads a part as a PLATE: two parallel faces, a
profile around the big one, and recesses into it. That is what the workshop cuts
every day and it is left alone. A shaped part — a tapered block, a domed pad —
is not a plate: its sides slope or curve, so the flat reader picks the big base
as the face to machine, finds every slope facing away from the cutter, and cuts
a plain outline. A 3D job is chosen deliberately at upload and comes here
instead, so nothing about the flat path changes.

What a 3D part becomes
----------------------
The solid is meshed and looked at from the direction that reaches the most of
its surface (`surface_extract.orientation_report`) — for a tapered block, small
face up, base on the bed. Then:

    OUTSIDE   the footprint seen from above (the base outline), profiled
              through last, with tabs, like any part;
    INSIDE /  through holes in that footprint;
    DRILL
    SURFACE   one heightfield over the whole part, roughed with the flat
              cutter and finished with the ball nose.

Off the part, the heightfield is pinned at SKIN_MM above the board bottom. The
surfacing therefore clears down to a thin skin round the part instead of to
the spoilboard: the part stays held by the sheet until the profile cuts it out,
and nothing goes below the board. The price is a SKIN_MM-high vertical band
at the very foot of the part, where the profile trims it square.

Not handled (reported, not hidden): undercuts — anything facing down that is
not the base cannot be reached from above and is left as the tool sees it.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

SKIN_MM = 1.0            # stock left under the surfacing off the part (see above)
MESH_DEFLECTION = 0.05   # mm — the footprint becomes the profile, so as tight
                         # as the flat path's outline sampling (CHORD_TOL_MM)
WINDOW_MM = 10.0         # heightfield margin round the footprint
REACH_MM = 6.0           # surfacing lanes run this far past the footprint, so
                         # the ball reaches the foot of every slope
OVERHANG_WARN_PCT = 1.0  # more undercut than this is reported
HOLE_MIN_AREA_MM2 = 0.5  # a footprint hole smaller than this is a seam between
                         # triangles, not a hole (Ø0.8 mm; no cutter goes in)

# A part is suggested for a 3D job when it carries this much sloped or curved
# face, counting only faces wider than SUGGEST_MIN_WIDTH_MM — so a chamfer or
# small edge round on an ordinary board does not set it off.
SUGGEST_MIN_AREA_MM2 = 200.0
SUGGEST_MIN_WIDTH_MM = 5.0


def _engine():
    eng = Path(__file__).resolve().parents[2] / "cam" / "engine"
    if str(eng) not in sys.path:
        sys.path.insert(0, str(eng))
    import surface_extract as SE
    import surfacing as SF
    return SE, SF


# ------------------------------------------------------------- suggestion

def looks_3d(shape) -> bool:
    """Does this part have shaped faces a flat job will not cut?

    Measured against the part's dominant plane (its biggest flat face): planar
    faces tilted 5..85 degrees to it, and curved faces other than walls and
    holes square to it, count — when they are wide enough to be shape rather
    than an edge break."""
    from . import wood_geometry as WG
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepGProp import BRepGProp
    from OCP.GeomAbs import GeomAbs_SurfaceType as ST
    from OCP.GProp import GProp_GProps

    faces = WG._faces(shape)
    best, axis = 0.0, None
    for f in faces:
        pl = WG._plane_of(f)
        if pl is not None:
            a = WG._area_mm2(f)
            if a > best:
                best, axis = a, pl[1]
    if axis is None:
        return True                             # no flat face at all: shaped

    shaped = 0.0
    for f in faces:
        area = WG._area_mm2(f)
        pl = WG._plane_of(f)
        if pl is not None:
            d = abs(WG._dot(pl[1], axis))
            if not (0.087 < d < 0.996):         # parallel or square to the board
                continue
        else:
            s = BRepAdaptor_Surface(f)
            t = s.GetType()
            if t in (ST.GeomAbs_Cylinder, ST.GeomAbs_Cone):
                ax = (s.Cylinder() if t == ST.GeomAbs_Cylinder else s.Cone()).Axis().Direction()
                if abs(WG._dot((ax.X(), ax.Y(), ax.Z()), axis)) > 0.99:
                    continue                    # a hole or a rounded corner wall
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(f, props)
        perim = float(props.Mass())
        if perim > 0 and 2.0 * area / perim < SUGGEST_MIN_WIDTH_MM:
            continue                            # a strip: chamfer or edge round
        shaped += area
    return shaped >= SUGGEST_MIN_AREA_MM2


# --------------------------------------------------------------- geometry

def _footprint(mesh, frame):
    """The part seen from above: shapely Polygon of the union of every
    non-vertical facet projected onto the bed, in frame (u, v) mm."""
    import numpy as np
    from shapely import union_all
    from shapely.geometry import Polygon
    SE, _SF = _engine()
    tf = SE._to_frame(mesh["tris"].astype(float), frame)[..., :2]
    a = tf[:, 1] - tf[:, 0]
    b = tf[:, 2] - tf[:, 0]
    keep = np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) > 1e-6
    polys = [Polygon(t) for t in tf[keep]]
    shape = union_all(polys).buffer(0)
    if shape.geom_type == "MultiPolygon":
        shape = max(shape.geoms, key=lambda g: g.area)
    return _drop_specks(shape.simplify(0.01, preserve_topology=True))


def _drop_specks(foot):
    """The footprint without holes that are only seams between triangles.

    Unioning thousands of projected facets can leave a zero-area hole where
    their edges almost meet. On the Linux server Part 1 (7), a solid tapered
    block, came out with one: three points at the same spot, read as a
    cut-out, and reported as "slot too narrow for any loaded tool — SKIPPED".
    A real through-hole has area and some width; a speck has neither.
    """
    from shapely.geometry import Polygon
    keep = [ring for ring in foot.interiors
            if Polygon(ring).area >= HOLE_MIN_AREA_MM2
            and not Polygon(ring).buffer(-0.05).is_empty]
    if len(keep) == len(foot.interiors):
        return foot
    return Polygon(foot.exterior, keep)


def _ring(coords):
    pts = [(float(x), float(y)) for x, y in coords]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    return pts


def part_geometry_3d(shape, name: str = "") -> dict:
    """The geometry dict `wood_cam_service` hands the engine, for a 3D part.
    Same keys as `wood_geometry.part_geometry`, plus `warnings`."""
    import numpy as np
    from matplotlib.path import Path as MplPath
    from . import wood_geometry as WG
    from .wood_cam_service import _circle
    SE, SF = _engine()

    mesh = WG._occ_mesh(shape, deflection=MESH_DEFLECTION)
    report = SE.orientation_report(mesh)
    frame = SE.choose_frame(mesh, report["recommended_vector"])
    z = SE._to_frame(mesh["tris"].reshape(-1, 3).astype(float), frame)[:, 2]
    thickness = round(float(z.max()), 3)
    if thickness <= 0.5:
        raise WG.GeometryError("part has no height to surface")

    foot = _footprint(mesh, frame)
    if foot.is_empty or foot.area < 1.0:
        raise WG.GeometryError("no footprint could be read")
    loops = [{"role": "OUTSIDE", "pts": _ring(foot.exterior.coords),
              "circle": None, "isOuter": True}]
    for hole in foot.interiors:
        ring = _ring(hole.coords)
        if len(ring) < 3:
            continue
        circ = _circle(ring)
        loops.append({"role": "DRILL" if circ else "INSIDE", "pts": ring,
                      "circle": circ, "isOuter": False})

    x0, y0, x1, y1 = foot.bounds
    window = (x0 - WINDOW_MM, x1 + WINDOW_MM, y0 - WINDOW_MM, y1 + WINDOW_MM)
    res = max(0.4, max(x1 - x0, y1 - y0) / 400.0)
    zmap, grid = SE.build_zmap(mesh, frame, res_mm=res, window=window)
    # Off the part (and in through holes): a skin, never the spoilboard.
    zmap = np.where(np.isfinite(zmap), np.maximum(zmap, SKIN_MM), SKIN_MM)

    gx = grid["ox"] + np.arange(grid["nx"]) * grid["res"]
    gy = grid["oy"] + np.arange(grid["ny"]) * grid["res"]
    GX, GY = np.meshgrid(gx, gy)
    pts2d = np.column_stack([GX.ravel(), GY.ravel()])
    reach = foot.buffer(REACH_MM)
    mask = np.zeros(pts2d.shape[0], dtype=bool)
    for poly in getattr(reach, "geoms", [reach]):
        mask |= MplPath(np.asarray(poly.exterior.coords)).contains_points(pts2d)
    mask = mask.reshape(zmap.shape)
    # Leave out what is already at full height (a flat top): the ball would
    # only skim it. Kept within a ball's reach of anything lower, so the edge
    # of the top is still finished.
    low = (zmap < thickness - 0.01).astype(float)
    near_low = SF._max_filter_square(np.where(low > 0, 1.0, -np.inf),
                                     int(math.ceil(REACH_MM / grid["res"])))
    mask &= near_low > 0

    # The part itself, in the same frame as its loops, for the 3D job view:
    # triangles (n, 3, 3), base on z = 0, as the nest will place them.
    model = np.round(SE._to_frame(mesh["tris"].reshape(-1, 3).astype(float), frame)
                     .reshape(-1, 3, 3), 3)
    loops.append({"role": "SURFACE", "pts": _ring(foot.exterior.coords),
                  "circle": None, "isOuter": False, "zmap": zmap, "grid": grid,
                  "mask": mask, "top": thickness, "rough_without_ball": True,
                  "steep_cross": True, "model": model})

    warnings = []
    if report["recommended_overhang_pct"] > OVERHANG_WARN_PCT:
        warnings.append(f"{report['recommended_overhang_pct']:.0f}% of the surface "
                        "is undercut and can't be reached from above — it is left "
                        "as the cutter sees it")
    return {"loops": loops, "pockets": [], "up_side": "TOP", "flip": False,
            "two_sided": False, "thickness": thickness, "fragmented": False,
            "unmatched_pockets": 0, "far_pockets": [], "warnings": warnings}


def read_geometry_3d(part):
    """(loops, geom_or_None, why) for `cam_parts_and_diag`, like `_read_geometry`.
    No flat fallback: a 3D part cut as a plain outline is the wrong part."""
    try:
        geom = part_geometry_3d(getattr(part, "shape", None), part.name)
        return geom["loops"], geom, "; ".join(geom["warnings"])
    except ImportError:
        return [], None, "OpenCascade not available — 3D parts cannot be read"
    except Exception as exc:                            # noqa: BLE001
        return [], None, f"3D read failed: {type(exc).__name__}: {exc}"[:160]
