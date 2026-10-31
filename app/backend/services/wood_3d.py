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
            if _square_to(s, axis):
                continue                        # any wall standing square to the board
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(f, props)
        perim = float(props.Mass())
        if perim > 0 and 2.0 * area / perim < SUGGEST_MIN_WIDTH_MM:
            continue                            # a strip: chamfer or edge round
        shaped += area
    return shaped >= SUGGEST_MIN_AREA_MM2


def _square_to(surf, axis, tol=0.02) -> bool:
    """Is every normal of this face square to the board? A cut-out drawn as a
    spline in the sketch is a B-spline wall, not a cylinder: 15 mm of it stood
    straight up through a chainstay plate and counted as shaped surface, so every
    windowed plate looked 3D and a 3D job stood them on edge."""
    from OCP.BRepLProp import BRepLProp_SLProps
    u0, u1 = surf.FirstUParameter(), surf.LastUParameter()
    v0, v1 = surf.FirstVParameter(), surf.LastVParameter()
    seen = False
    for i in range(5):
        for j in range(5):
            pr = BRepLProp_SLProps(surf, u0 + (u1 - u0) * (i + 0.5) / 5,
                                   v0 + (v1 - v0) * (j + 0.5) / 5, 1, 1e-6)
            if not pr.IsNormalDefined():
                continue
            n = pr.Normal()
            if abs(n.X() * axis[0] + n.Y() * axis[1] + n.Z() * axis[2]) > tol:
                return False
            seen = True
    return seen


# A plate in a 3D job is cut as a board (`wood_cam_service`), and the 3D view
# still wants to draw it: the solid meshed coarser than a footprint needs.
PLATE_MODEL_DEFLECTION = 0.1


def plate_model(shape, geom):
    """The solid's triangles in a FLAT read's frame, for the 3D job view: x, y
    as the plate's loops (`geom["frame"]`, shift included), z up from the board
    bottom with the machined face at the plate's thickness."""
    import numpy as np
    from . import wood_geometry as WG
    fr = geom["frame"]
    tris = WG._occ_mesh(shape, deflection=PLATE_MODEL_DEFLECTION)["tris"].reshape(-1, 3).astype(float)
    d = tris - np.asarray(fr["origin"], float)
    xyz = np.column_stack([d @ np.asarray(fr["u"], float) - fr["shift"][0],
                           d @ np.asarray(fr["v"], float) - fr["shift"][1],
                           geom["thickness"] + d @ np.asarray(fr["normal"], float)])
    return np.round(xyz.reshape(-1, 3, 3), 3)


# The machining direction for a shaped part. Most surface reached from above
# alone stood flat plates on edge — a 15 mm chainstay roughed out of 36 mm
# stock, a rocker out of 161 mm — because a side view also sees half the
# window walls, at the price of undercutting the rest. Undercut decides first:
# any direction leaving no more of it than the best one does (within this
# slack) is acceptable, and of those the lowest stock wins — the least to rough
# away. Reach only breaks ties.
UNDERCUT_SLACK_PCT = 2.0
# Undercut here leaves out facets within this of vertical (sin of ~6 degrees).
# A curved wall's mesh joins top and bottom points that do not line up, so a
# truly square spline window side meshes as facets leaning up to ~4 degrees, and
# `surface_extract` counts every one as undercut: a flat stay read 5.6 % undercut
# and was warned about. The cutter's flank grazes those; nothing is left behind.
WALL_SIN = 0.1


def _undercut_pct(mesh, vec):
    import numpy as np
    t = mesh["tris"].astype(float)
    area = np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1) / 2.0
    nm = mesh["fnormals"].astype(float) @ np.asarray(vec, float)
    under = (nm < -WALL_SIN) & (nm >= -0.98)          # below -0.98 is the bed
    return 100.0 * float(area[under].sum()) / (float(area.sum()) or 1.0)


def _pick_up(mesh, report):
    """The report candidate to machine from: least undercut, then lowest stock,
    then most reach. Returned with its own `undercut_pct` (see WALL_SIN)."""
    import numpy as np
    pts = mesh["tris"].reshape(-1, 3).astype(float)
    cands = [dict(c, undercut_pct=_undercut_pct(mesh, c["vector"])) for c in report["candidates"]]
    least = min(c["undercut_pct"] for c in cands)

    def height(c):
        h = pts @ np.asarray(c["vector"], float)
        return float(h.max() - h.min())
    ok = [c for c in cands if c["undercut_pct"] <= least + UNDERCUT_SLACK_PCT]
    return min(ok, key=lambda c: (round(height(c)), -c["surfaceable_pct"]))


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
    up = _pick_up(mesh, report)
    frame = SE.choose_frame(mesh, up["vector"])
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
    if up["undercut_pct"] > OVERHANG_WARN_PCT:
        warnings.append(f"{up['undercut_pct']:.0f}% of the surface "
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
