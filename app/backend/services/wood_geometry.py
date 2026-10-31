"""Depth-aware geometry for wood parts: profile, holes, cut-outs AND pockets.

Why this is separate from step_ingest_service
---------------------------------------------
The exporter's outline extraction reads ONE planar face per part, because a
laser cuts through and a blind feature is not its business. A router's are:
rebates and routed recesses are ordinary in wood, and a part machined as though
they were not there is under-machined. This module reads the solid instead of a
single face, so a pocket arrives with its depth and its islands.

Reusing the proven detector
---------------------------
`cam/engine/extract.py:detect_pockets` already does the hard part -- deciding
which planar faces are the two plate faces and which are blind floors, handling
plates that stand on edge, and rejecting walls and impossible depths. It is pure
geometry: all it wants per face is an area, an outward normal and an origin. So
it is fed OCC faces through a unit adapter rather than being reimplemented,
which keeps one detector in one place with the behaviour that was already
proven at the machine.

It expects Onshape's units (area in m2, origin in metres); OCC gives mm. That
conversion is the whole of the adapter.

Frames
------
Every loop must land in ONE frame -- the machined face's plane -- or a pocket
floor would be drawn in its own coordinates and machined in the wrong place.
That is the difference from `_face_outline_2d`, which normalises each face to
its own bounding box.

What is still not handled
-------------------------
A machined face split into several coplanar fragments by its own recesses.
Walking such a face misses the openings, and `extract.py` has a second "robust"
path for exactly this. Here that case is DETECTED and reported
(`fragmented=True`) and the part falls back to through-features only, rather
than silently producing a profile with the pockets missing.
"""

from __future__ import annotations

import math

CHORD_TOL_MM = 0.05      # matches the exporter's outline sampling
_MM_PER_M = 1000.0


class GeometryError(RuntimeError):
    """The solid could not be read well enough to machine from."""


# --------------------------------------------------------------------- OCC

def _occ():
    from OCP.BRep import BRep_Tool                                  # noqa: F401
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve  # noqa: F401
    from OCP.BRepGProp import BRepGProp                             # noqa: F401
    from OCP.GProp import GProp_GProps                              # noqa: F401
    from OCP.TopAbs import TopAbs_ShapeEnum, TopAbs_Orientation     # noqa: F401
    from OCP.TopExp import TopExp_Explorer                          # noqa: F401
    from OCP.TopoDS import TopoDS                                   # noqa: F401
    return True


def _faces(shape):
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    out, exp = [], TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        out.append(TopoDS.Face_s(exp.Current()))
        exp.Next()
    return out


def _area_mm2(face) -> float:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    return float(props.Mass())


def _plane_of(face):
    """(origin_mm, outward_normal) for a planar face, or None."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    from OCP.TopAbs import TopAbs_Orientation
    surf = BRepAdaptor_Surface(face)
    if surf.GetType() != GeomAbs_SurfaceType.GeomAbs_Plane:
        return None
    pln = surf.Plane()
    o = pln.Location()
    d = pln.Axis().Direction()
    n = (d.X(), d.Y(), d.Z())
    # A REVERSED face runs against its surface normal; the geometric normal is
    # then pointing INTO the solid. detect_pockets decides which main face a
    # floor opens through from this direction, so getting it wrong swaps top and
    # bottom pockets.
    if face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
        n = (-n[0], -n[1], -n[2])
    return (o.X(), o.Y(), o.Z()), n


def planar_records(shape):
    """Planar faces as the records `detect_pockets` consumes.

    Areas go out in m2 and origins in metres because that is what the detector
    was written against (Onshape SI); `orientation` is always True because the
    outward normal is resolved here rather than being left as a flag.
    """
    recs = []
    for f in _faces(shape):
        pl = _plane_of(f)
        if pl is None:
            continue
        (ox, oy, oz), n = pl
        recs.append({
            "area": _area_mm2(f) / 1e6,
            "orientation": True,
            "surface": {"type": "PLANE",
                        "normal": {"x": n[0], "y": n[1], "z": n[2]},
                        "origin": {"x": ox / _MM_PER_M,
                                   "y": oy / _MM_PER_M,
                                   "z": oz / _MM_PER_M}},
            "_occ": f,
        })
    return recs


# ------------------------------------------------------------------- frames

def _basis(n):
    """The engine's own in-plane axes for a unit normal.

    Deliberately its function and not a parallel one: the frame decides how
    every part is oriented when it reaches the nester, and two implementations
    that drifted apart would put the wood path and the proven path in different
    coordinate systems for the same plate.
    """
    from pathlib import Path
    import sys
    eng = Path(__file__).resolve().parents[2] / "cam" / "engine"
    if str(eng) not in sys.path:
        sys.path.insert(0, str(eng))
    from extract import make_basis
    return make_basis(n)


def _point_in_ring(ring, pt) -> bool:
    """Ray cast — is (x, y) inside this closed 2-D ring?"""
    x, y = pt
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xi = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if xi > x:
                inside = not inside
    return inside


def _solid_below(shape, origin, u, v, normal, xy, drop) -> bool:
    """Is there material `drop` mm beneath the machined face at this (u, v) point?

    `_project` maps a 3-D point to (d.u, d.v) about `origin`, so the inverse is
    origin + u*x + v*y; `normal` is the machined face's OUTWARD normal, so going
    into the part is along -normal.
    """
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_State
    p = tuple(origin[i] + u[i] * xy[0] + v[i] * xy[1] - normal[i] * drop
              for i in range(3))
    with _PROBE_LOCK:
        cls = _classifier(shape)
        cls.Perform(gp_Pnt(*p), 1e-6)
        return cls.State() == TopAbs_State.TopAbs_IN


# Building a BRepClass3d_SolidClassifier loads every face of the solid; asking it
# about a point afterwards is cheap. It used to be built afresh for every probe,
# 30-70 ms each on a jig — harmless at two probes per ring, but the island-or-bore
# test now probes a grid inside each ring (see _split_floor_wires), and the Calf
# jig's ~1000 probes took 75-107 s of the router's "generate", during which the
# single-worker server answered nothing at all. One classifier per solid, reused.
_CLASSIFIERS: list = []          # [(shape, classifier)], most recent last
_CLASSIFIER_KEEP = 4
# The router's heavy work now runs on worker threads, and an OpenCascade
# classifier is not safe to Perform() from two threads at once. Probes are
# milliseconds; serialising them costs nothing worth having.
import threading as _threading
_PROBE_LOCK = _threading.RLock()


def _classifier(shape):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    for i, (sh, cls) in enumerate(_CLASSIFIERS):
        if sh is shape or sh.IsSame(shape):
            return cls
    cls = BRepClass3d_SolidClassifier(shape)
    _CLASSIFIERS.append((shape, cls))
    del _CLASSIFIERS[:-_CLASSIFIER_KEEP]
    return cls


# ------------------------------------------------------------ 3D surfacing

def _has_freeform(shape, plate_normal) -> bool:
    """Does the solid carry sculpted faces — non-planar geometry that is not a
    hole/fillet wall (a cylinder whose axis runs along the plate normal)?
    Cached on the shape wrapper via id() for the duration of one extraction."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    for f in _faces(shape):
        s = BRepAdaptor_Surface(f)
        t = s.GetType()
        if t == GeomAbs_SurfaceType.GeomAbs_Plane:
            continue
        if t == GeomAbs_SurfaceType.GeomAbs_Cylinder:
            d = s.Cylinder().Axis().Direction()
            along = (d.X() * plate_normal[0] + d.Y() * plate_normal[1]
                     + d.Z() * plate_normal[2])
            if abs(along) > 0.99:                # hole / vertical fillet wall
                continue
        if _area_mm2(f) >= 25.0:                 # ignore sliver blends
            return True
    return False


def _occ_mesh(shape, deflection=0.15, angular=0.35):
    """The solid as the surfacing engine's mesh dict — (M,3,3) mm triangles
    with outward per-facet normals — from OCC's own tessellation."""
    import numpy as np
    from .step_thumbnail_service import _tessellate
    tris = np.asarray(_tessellate(shape, deflection=deflection, max_tris=400_000,
                                  angular=angular), dtype=np.float64)
    if tris.size == 0:
        raise GeometryError("no mesh could be built for surfacing")
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    return {"tris": tris.astype(np.float32), "fnormals": (n / ln).astype(np.float32)}


def _angled_surface_loops(recs, origin, u, v, normal, min_area=25.0):
    """SURFACE regions for ANGLED planar faces — neither plate-parallel nor a
    wall (5..85 degrees to the machined normal), facing the machined side.
    A 45-degree ramp at a rebate end is a plane, so the freeform detector
    never fires, and the region was silently left as a square shoulder. Its
    footprint ring drives the same drop-cutter rough + ball finish as a
    sculpted surface."""
    out = []
    for r in recs:
        n = (r["surface"]["normal"]["x"], r["surface"]["normal"]["y"],
             r["surface"]["normal"]["z"])
        d = _dot(n, normal)
        if not (0.996 > d > 0.087):            # 5..85 degrees, machined side
            continue
        if r["area"] * 1e6 < min_area:
            continue
        try:
            ring, _inner = face_loops(r["_occ"], origin, u, v)
        except Exception:                      # noqa: BLE001
            continue
        if len(ring) >= 3:
            out.append({"role": "SURFACE", "pts": ring, "circle": None,
                        "isOuter": False})
    return out


def _attach_surface_maps(shape, loops, origin, u, v, normal, thickness):
    """Build each SURFACE region's heightfield, in the SAME 2-D frame as the
    loops so the raster paths land exactly on the nested part.

    zmap heights are absolute part Z (0 = bottom face, thickness = stock top),
    matching how the emitter references depths. Inside the region the sculpted
    surface is sampled from the mesh; the band OUTSIDE the region (within the
    cutter-margin window) is pinned to stock top so the drop-cutter lifts the
    ball correctly at the boundary instead of gouging the surrounding face."""
    surf = [lp for lp in loops if lp["role"] == "SURFACE"]
    if not surf:
        return
    import numpy as np
    from matplotlib.path import Path as _MplPath
    from pathlib import Path as _P
    import sys
    eng = _P(__file__).resolve().parents[2] / "cam" / "engine"
    if str(eng) not in sys.path:
        sys.path.insert(0, str(eng))
    import surface_extract as SE

    mesh = _occ_mesh(shape)
    base = np.asarray(origin, float) - np.asarray(normal, float) * float(thickness)
    frame = {"O": base, "u": np.asarray(u, float),
             "v": np.asarray(v, float), "m": np.asarray(normal, float)}
    for lp in surf:
        xs = [q[0] for q in lp["pts"]]; ys = [q[1] for q in lp["pts"]]
        margin = 4.0
        window = (min(xs) - margin, max(xs) + margin,
                  min(ys) - margin, max(ys) + margin)
        # adaptive pitch: fine enough for a O6 ball, capped so one big sculpt
        # cannot eat the request
        ext = max(window[1] - window[0], window[3] - window[2])
        res = max(0.4, ext / 400.0)
        zmap, grid = SE.build_zmap(mesh, frame, res_mm=res, window=window)
        gx = grid["ox"] + np.arange(grid["nx"]) * grid["res"]
        gy = grid["oy"] + np.arange(grid["ny"]) * grid["res"]
        GX, GY = np.meshgrid(gx, gy)
        # The zmap keeps the REAL surface everywhere in the window — the
        # drop-cutter needs true neighbouring heights to lift the ball at a
        # rim and to overrun a bevel that runs out through the part edge
        # (off-part cells stay NaN, so the tool drops into the waste there,
        # like a profile overcut). CUTTING is restricted separately: the
        # raster lanes are clipped to the region grown by a ball-radius
        # allowance, or a long diagonal bevel would raster its whole
        # bounding box — skimming across the part and off the sheet.
        from cam_syntec import offset as _cs_offset
        grown = _cs_offset(lp["pts"], 5.0)
        rings = grown if grown else [lp["pts"]]
        pts2d = np.column_stack([GX.ravel(), GY.ravel()])
        m = np.zeros(pts2d.shape[0], dtype=bool)
        for gring in rings:
            m |= _MplPath(gring).contains_points(pts2d)
        lp["mask"] = m.reshape(zmap.shape)
        lp["zmap"] = zmap
        lp["grid"] = grid
        lp["top"] = float(thickness)


def _floor_footprint(cand, origin, u, v) -> "tuple[float, tuple | None]":
    """(gross area, 2-D centroid) of the floor's OUTER wire, cached on `cand`.

    `cand["area"]` is the face's NET area. A counterbore's floor is an annulus,
    so it measures pi(R^2 - r^2) while the mouth ring above it measures pi R^2 —
    comparing those two rejects every counterbore whose bore is larger than the
    match tolerance, which is any bore over about O8 (pi r^2 > 50 mm2). Measuring
    the outer wire compares like with like. The centroid is what ties a floor to
    the mouth DIRECTLY ABOVE it rather than to any similar-sized ring elsewhere.
    """
    if "_footprint" in cand:
        return cand["_footprint"]
    outer, _inner = face_loops(cand["face"]["_occ"], origin, u, v)
    if len(outer) < 3:
        cand["_footprint"] = (cand["area"], None)
        return cand["_footprint"]
    # Area-weighted polygon centroid — a plain vertex mean is skewed by the
    # uneven point spacing the curve sampler produces.
    a2 = cx = cy = 0.0
    for i in range(len(outer)):
        x0, y0 = outer[i]
        x1, y1 = outer[(i + 1) % len(outer)]
        cross = x0 * y1 - x1 * y0
        a2 += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(a2) < 1e-9:
        centre = (sum(p[0] for p in outer) / len(outer),
                  sum(p[1] for p in outer) / len(outer))
    else:
        centre = (cx / (3.0 * a2), cy / (3.0 * a2))
    cand["_footprint"] = (_area2d(outer), centre)
    return cand["_footprint"]


def _split_floor_wires(shape, wires, origin, u, v, normal, depth, thickness):
    """A pocket floor's inner wires, split into bosses to machine around and
    bores to cut.

    The two are identical in 2-D, so the solid is asked instead: probe just under
    the floor, and again near the far face. Material under the floor is a boss.
    Empty under the floor but material near the bottom is a deeper blind step,
    which is left standing rather than guessed at. Empty all the way through is a
    bore, and a bore has to be CUT — treating a counterbore's bolt hole as an
    island machined around it and left it undrilled.
    """
    from .wood_cam_service import _circle          # same hole test as the flat path
    islands, bores = [], []
    near = min(depth + 0.5, max(thickness - 0.5, depth + 0.1))
    far = max(thickness - 0.5, near)
    for w in wires:
        if len(w) < 3:
            continue
        probes = _interior_probes(w)
        if not probes:
            islands.append(w)                     # can't probe it safely: leave the stock
            continue
        # A bore is empty everywhere inside its ring, so EVERY probe has to come
        # back empty at both depths. One probe at the centroid was not a test of
        # that: on a grooved jig the outer boundary of the whole groove network
        # is an inner wire of the machined face, and whether it counted as
        # material depended on where its centroid happened to fall. On the Calf
        # jig it fell on a through hole and the entire middle of the part was
        # cut out as one slug; on the Thigh jig it fell on a groove floor and all
        # was well. A ring that encloses ANY material is not a bore.
        if any(_material_at(shape, origin, u, v, normal, c, near) or
               _material_at(shape, origin, u, v, normal, c, far) for c in probes):
            islands.append(w)
            continue
        circ = _circle(w)
        bores.append({"role": "DRILL" if circ else "INSIDE", "pts": w,
                      "circle": circ, "isOuter": False})
    return islands, bores


def _material_at(shape, origin, u, v, normal, xy, drop) -> bool:
    """_solid_below, answered from a 2-D slice of the solid at that depth.

    The island-or-bore test asks up to 26 points per ring at two depths; on a
    backrest jig that was 3,569 questions at ~6 ms each against the 3-D solid —
    20 s of the router's generate on a laptop, a minute or more on the server.
    Every question at one depth has the same answer source: the solid's cross-
    section on that plane. Slice it once, and each point is a 2-D containment
    test. Falls back to the 3-D classifier if the slice cannot be made.
    """
    region = _section_region(shape, origin, u, v, normal, drop)
    if region is None:
        return _solid_below(shape, origin, u, v, normal, xy, drop)
    from shapely.geometry import Point
    return region.contains(Point(xy))


_SECTIONS: list = []             # [(key, shape, prepared region | None)]
_SECTION_KEEP = 16


def _section_region(shape, origin, u, v, normal, drop):
    """The solid's cross-section `drop` below the machined face, as a prepared
    shapely region in the (u, v) frame; None if it could not be built."""
    key = (round(drop, 4),) + tuple(round(c, 6) for c in (*origin, *u, *v, *normal))
    with _PROBE_LOCK:
        for k, sh, region in _SECTIONS:
            if k == key and (sh is shape or sh.IsSame(shape)):
                return region
        region = None
        try:
            region = _slice(shape, origin, u, v, normal, drop)
        except Exception:                              # noqa: BLE001 - kernel
            region = None
        _SECTIONS.append((key, shape, region))
        del _SECTIONS[:-_SECTION_KEEP]
        return region


def _slice(shape, origin, u, v, normal, drop):
    from OCP.gp import gp_Pln, gp_Pnt, gp_Dir, gp_Ax3
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from shapely.prepared import prep

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    x0, y0, z0, x1, y1, z1 = box.Get()
    half = 2.0 * max(x1 - x0, y1 - y0, z1 - z0, 1.0)
    p0 = tuple(origin[i] - normal[i] * drop for i in range(3))
    plane = gp_Pln(gp_Ax3(gp_Pnt(*p0), gp_Dir(*normal), gp_Dir(*u)))
    face = BRepBuilderAPI_MakeFace(plane, -half, half, -half, half).Face()
    op = BRepAlgoAPI_Common(shape, face)
    op.Build()
    if not op.IsDone():
        return None
    polys = []
    for f in _faces(op.Shape()):
        outer, inner = face_loops(f, origin, u, v)
        if len(outer) < 3:
            continue
        poly = Polygon(outer, [h for h in inner if len(h) >= 3]).buffer(0)
        if not poly.is_empty:
            polys.append(poly)
    if not polys:
        # Nothing at this depth is a real answer (air), not a failure — but
        # only if the solid really is absent there; let the 3-D classifier
        # decide rather than trusting an empty boolean.
        return None
    return prep(unary_union(polys))


# A probe this close to its ring's edge is not asking about the ring's interior.
# The ring and the slice it is tested against are two separate chord samplings
# (CHORD_TOL_MM each) of the same wall, so they can disagree by up to twice that
# about which side of the wall a point is on. On a chainstay plate a grid point
# 0.023 mm inside a through window read as material, the window was taken for an
# island and dropped from the cut with no warning. Probes must clear the wall by
# well over that disagreement.
PROBE_EDGE_MM = 0.25


def _dist_to_ring(ring, pt) -> float:
    x, y = pt
    best = math.inf
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        best = min(best, math.hypot(x - x0 - t * dx, y - y0 - t * dy))
    return best


def _interior_probes(ring, grid=5):
    """Points spread over a ring's interior to probe the solid at: the centroid
    and a coarse grid over the ring's box, keeping only those inside the ring
    and at least PROBE_EDGE_MM clear of its edge."""
    xs = [p[0] for p in ring]; ys = [p[1] for p in ring]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    pts = [(sum(xs) / len(xs), sum(ys) / len(ys))]
    for i in range(grid):
        for j in range(grid):
            pts.append((x0 + (x1 - x0) * (i + 0.5) / grid,
                        y0 + (y1 - y0) * (j + 0.5) / grid))
    return [c for c in pts
            if _point_in_ring(ring, c) and _dist_to_ring(ring, c) >= PROBE_EDGE_MM]


def _project(p, origin, u, v):
    d = (p[0] - origin[0], p[1] - origin[1], p[2] - origin[2])
    return (d[0] * u[0] + d[1] * u[1] + d[2] * u[2],
            d[0] * v[0] + d[1] * v[1] + d[2] * v[2])


# A loop whose edges do not meet within this is not trustworthy: the gap gets
# bridged by a straight line across the part. Same value, and same reasoning, as
# the exporter's outline extraction.
OUTLINE_GAP_TOL_MM = 0.05


def _wire_points(wire, origin, u, v, face=None):
    """One wire as a closed 2-D polyline in the given frame.

    Follows the exporter's proven outline walk rather than inventing a second
    one, because both of its subtleties were learned the hard way:

    * The explorer is given the FACE. Without it an inner wire comes back in an
      arbitrary edge order, and chaining that yields a bowtie whose area is
      wrong — a 60 x 40 pocket mouth measured 1200 mm2 instead of 2400 and so
      never matched its floor.
    * Edge direction is anchored to the vertex the wire ENTERS the edge by
      (`we.CurrentVertex()`), not to whichever end is nearer. Proximity picks the
      wrong end on short edges and leaves the loop open.

    Raises GeometryError rather than closing a gap silently: a bridged gap means
    the toolpath is not the part.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    we = (BRepTools_WireExplorer(wire, face) if face is not None
          else BRepTools_WireExplorer(wire))
    pts, gaps = [], []
    while we.More():
        edge = we.Current()
        if BRep_Tool.Degenerated_s(edge):
            we.Next()
            continue
        curve = BRepAdaptor_Curve(edge)
        try:
            samp = GCPnts_QuasiUniformDeflection(curve, CHORD_TOL_MM,
                                                 curve.FirstParameter(),
                                                 curve.LastParameter())
            seq = [curve.Value(samp.Parameter(i + 1)) for i in range(samp.NbPoints())]
        except Exception:
            seq = [curve.Value(curve.FirstParameter()),
                   curve.Value(curve.LastParameter())]
        seq = [_project((p.X(), p.Y(), p.Z()), origin, u, v) for p in seq]
        if seq:
            vtx = BRep_Tool.Pnt_s(we.CurrentVertex())
            start = _project((vtx.X(), vtx.Y(), vtx.Z()), origin, u, v)
            if math.dist(seq[-1], start) < math.dist(seq[0], start):
                seq.reverse()
            if not pts:
                pts = seq
            else:
                gap = math.dist(pts[-1], seq[0])
                if gap > OUTLINE_GAP_TOL_MM:
                    gaps.append(gap)
                pts.extend(seq[1:])
        we.Next()

    if len(pts) >= 3:
        close_gap = math.dist(pts[0], pts[-1])
        if close_gap > OUTLINE_GAP_TOL_MM:
            gaps.append(close_gap)
    if gaps:
        raise GeometryError(
            f"loop does not close: {len(gaps)} gap(s), largest {max(gaps):.2f} mm")

    # drop repeated closing vertices; the CAM treats loops as implicitly closed
    while len(pts) > 1 and math.dist(pts[0], pts[-1]) < 1e-7:
        pts.pop()
    return pts


def _wires(face):
    from OCP.BRepTools import BRepTools
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    outer = BRepTools.OuterWire_s(face)
    inner, exp = [], TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_WIRE)
    while exp.More():
        w = TopoDS.Wire_s(exp.Current())
        if not w.IsSame(outer):
            inner.append(w)
        exp.Next()
    return outer, inner


def face_loops(face, origin, u, v):
    """(outer_pts, [inner_pts, ...]) for one face, in the given frame."""
    outer, inner = _wires(face)
    return (_wire_points(outer, origin, u, v, face),
            [_wire_points(w, origin, u, v, face) for w in inner])


# ------------------------------------------------------------------ helpers

def _area2d(pts) -> float:
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _union_rings(rings):
    """Outer boundary of the 2-D union of closed rings, or None.

    None on any failure — callers keep the ring they already had, so a missing
    pyclipper or a degenerate input can never make a part WORSE than before.
    """
    try:
        import pyclipper
    except ImportError:
        return None
    scale = 1000.0
    pc = pyclipper.Pyclipper()
    added = False
    for r in rings:
        if len(r) < 3 or _area2d(r) < 1e-6:
            continue
        # Normalise orientation: NONZERO filling treats a reversed ring as a
        # hole, and a union with a hole in it is exactly the bug being fixed.
        a = 0.0
        for i in range(len(r)):
            x0, y0 = r[i]
            x1, y1 = r[(i + 1) % len(r)]
            a += x0 * y1 - x1 * y0
        pts = list(r) if a >= 0 else list(r)[::-1]
        try:
            pc.AddPath([(int(round(x * scale)), int(round(y * scale)))
                        for x, y in pts], pyclipper.PT_SUBJECT, True)
            added = True
        except pyclipper.ClipperException:
            continue
    if not added:
        return None
    try:
        sol = pc.Execute(pyclipper.CT_UNION, pyclipper.PFT_NONZERO,
                         pyclipper.PFT_NONZERO)
    except pyclipper.ClipperException:
        return None
    if not sol:
        return None
    best = max(sol, key=lambda p: abs(pyclipper.Area(p)))
    return [(x / scale, y / scale) for x, y in best]


def _convex_hull(pts):
    """Monotone-chain hull, CCW, no duplicate endpoint."""
    p = sorted(set((round(x, 6), round(y, 6)) for x, y in pts))
    if len(p) <= 2:
        return p

    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2 and (
                    (out[-1][0] - out[-2][0]) * (q[1] - out[-2][1])
                    - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(q)
        return out
    lower = half(p)
    upper = half(reversed(p))
    return lower[:-1] + upper[:-1]


def _outline_align_angle(pts):
    """Rotation (radians) that axis-aligns the outline, or 0.0 to leave it be.

    The nester only turns parts in quarter turns, so a part whose outline is
    diagonal in its own STEP frame nests diagonally and wastes the sheet. Test
    every hull-edge direction (one of them is always an edge of the minimum
    bounding rectangle) and pick the one with the smallest box, long side on X.
    Parts that are already close to axis-aligned return exactly 0.0 — a 10%
    saving is required before anything rotates, so every existing part keeps
    its frame bit-for-bit.
    """
    hull = _convex_hull(pts)
    if len(hull) < 3:
        return 0.0
    xs = [q[0] for q in hull]
    ys = [q[1] for q in hull]
    base = (max(xs) - min(xs)) * (max(ys) - min(ys))
    if base <= 0.0:
        return 0.0
    best_area, best_ang = base, 0.0
    for (x1, y1), (x2, y2) in zip(hull, hull[1:] + hull[:1]):
        ang = math.atan2(y2 - y1, x2 - x1) % (math.pi / 2.0)
        if ang < 1e-9:
            continue
        ca, sa = math.cos(-ang), math.sin(-ang)
        rx = [ca * q[0] - sa * q[1] for q in hull]
        ry = [sa * q[0] + ca * q[1] for q in hull]
        w, h = max(rx) - min(rx), max(ry) - min(ry)
        if w * h < best_area - 1e-9:
            best_area, best_ang = w * h, ang
    if best_area > 0.90 * base:
        return 0.0
    # Long side on X for the natural "bar lying down" orientation.
    ca, sa = math.cos(-best_ang), math.sin(-best_ang)
    rx = [ca * q[0] - sa * q[1] for q in hull]
    ry = [sa * q[0] + ca * q[1] for q in hull]
    if (max(rx) - min(rx)) < (max(ry) - min(ry)):
        best_ang += math.pi / 2.0
    return best_ang


def _detect_pockets():
    """The CAM engine's detector, imported lazily.

    Lazily because this module is imported by the exporter's service layer,
    which must keep loading in environments where the CAM engine's own
    dependencies are absent.
    """
    import sys
    from pathlib import Path
    eng = Path(__file__).resolve().parents[2] / "cam" / "engine"
    if str(eng) not in sys.path:
        sys.path.insert(0, str(eng))
    from extract import detect_pockets
    return detect_pockets


# ------------------------------------------------------------------- public

def part_geometry(shape, name: str = "", force_side: "str | None" = None) -> dict:
    """See _part_geometry. The probe caches (classifier, slices) are scoped to the
    one solid being read: kept across parts they held every recent part's
    shape alive, so a job released to free memory was not actually freed."""
    clear_probe_caches()
    try:
        return _part_geometry(shape, name, force_side)
    finally:
        clear_probe_caches()


def clear_probe_caches() -> None:
    with _PROBE_LOCK:
        _CLASSIFIERS.clear()
        _SECTIONS.clear()


def _part_geometry(shape, name: str = "", force_side: "str | None" = None) -> dict:
    """Machinable geometry for one solid, machined pocket-side up.

    Returns {loops, pockets, up_side, flip, two_sided, thickness, fragmented}
    where loops carry OUTSIDE / INSIDE / DRILL / POCKET, POCKET carries `depth`
    (and `islands` where the recess has bosses standing in it), all in the
    machined face's 2-D frame.

    `fragmented` says the machined face is split into coplanar pieces by its own
    recesses. Walking such a face misses the openings, so the caller is told
    rather than handed a profile with the pockets quietly absent.

    `force_side` ("TOP"/"BOT") machines the NAMED face instead of choosing by
    where the pockets open. The two-sided flip-jig path reads the part twice —
    once per face — and the "BOT" read comes back in the bottom face's own
    outward frame, which is exactly the mirrored view the machine sees once the
    part is flipped into its mould.
    """
    _occ()
    from .wood_cam_service import _circle          # same hole test as the flat path

    recs = planar_records(shape)
    if len(recs) < 2:
        raise GeometryError("fewer than two planar faces — not a plate")

    mains, pockets, thickness = _detect_pockets()(recs)
    if mains is None:
        raise GeometryError("no parallel face pair — not a clean plate")

    axis, hi, lo = mains["axis"], mains["hi"], mains["lo"]
    sides = {p["side"] for p in pockets}
    two_sided = len(sides) > 1
    # Machine the side the pockets open through. Two-sided parts cut TOP first
    # and need a second setup for the rest, which is the engine's own rule.
    if force_side in ("TOP", "BOT"):
        up_side, flip = force_side, force_side == "BOT"
    elif "BOT" in sides and not two_sided:
        up_side, flip = "BOT", True
    else:
        up_side, flip = "TOP", False
    up_main = hi if up_side == "TOP" else lo

    # Is the machined face one whole region, or split by its own recesses?
    coplanar = [r for r in recs
                if abs(_dot((r["surface"]["origin"]["x"], r["surface"]["origin"]["y"],
                             r["surface"]["origin"]["z"]), axis) * _MM_PER_M
                       - up_main["off"]) < 0.3
                and abs(_dot((r["surface"]["normal"]["x"], r["surface"]["normal"]["y"],
                              r["surface"]["normal"]["z"]), axis)) > 0.9]
    fragmented = len(coplanar) != 1 or bool(mains.get("proud"))

    up_rec = up_main["face"]
    up_face = up_rec["_occ"]
    origin = tuple(up_rec["surface"]["origin"][k] * _MM_PER_M for k in ("x", "y", "z"))
    normal = tuple(up_rec["surface"]["normal"][k] for k in ("x", "y", "z"))
    u, v = _basis(normal)

    outer, inner = face_loops(up_face, origin, u, v)
    if len(outer) < 3:
        raise GeometryError("machined face has no usable outer wire")

    # _basis() picks an arbitrary in-plane frame, so a part exported at an
    # angle READS at that angle — and the nester only turns in quarter turns,
    # so it also NESTS at that angle, wasting sheet. Rotate the basis itself
    # so the outline is axis-aligned: every later projection (floors, holes,
    # surface z-maps, the far face on a force_side read) lands in the same
    # frame for free. Near-axis-aligned parts return 0.0 and are untouched.
    align = _outline_align_angle(outer)
    if abs(align) > 1e-9:
        ca, sa = math.cos(align), math.sin(align)
        u, v = (tuple(ca * u[i] + sa * v[i] for i in range(3)),
                tuple(-sa * u[i] + ca * v[i] for i in range(3)))
        outer, inner = face_loops(up_face, origin, u, v)

    # On a forced read (the flip-jig reads a two-sided part from both faces),
    # also project the OTHER side's pocket regions into THIS frame. They tell
    # the mould builder where the flipped part is thinner than full stock —
    # a part that only touches the spoilboard on a small full-thickness boss
    # needs the mould floor sculpted to support the rest (see prepare_mould).
    far_pockets = []
    if force_side in ("TOP", "BOT"):
        for p in pockets:
            if p["side"] == up_side:
                continue
            try:
                f_outer, f_inner = face_loops(p["face"]["_occ"], origin, u, v)
            except Exception:                              # noqa: BLE001
                continue
            if len(f_outer) >= 3:
                far_pockets.append({"pts": f_outer, "depth": p["depth"],
                                    "inner": [w for w in f_inner if len(w) >= 3]})

    # ---- proud features: a boss standing up from the plate -----------------
    # detect_pockets re-anchors the stock top to the OUTERMOST plane when planar
    # faces stand proud of the area-chosen mains, so `up_main` here is the BOSS
    # TOP — a face the size of the boss, not the plate. The plate's former top
    # face registers as an ordinary pocket floor below it (depth = how far the
    # field is milled down), which is exactly how the part is machined: face
    # everything inside the profile down to its own level and leave the boss.
    #
    # The mouth-matching path below assumes the machined face carries every
    # feature as an inner wire; on a boss part it carries almost nothing, so
    # this mode builds the loops from the FLOORS directly instead:
    #   outline  — the outer wire of the OPPOSITE main (the full footprint;
    #              the boss-top face's outer wire is just the boss).
    #   fields   — one POCKET per floor, boundary = the floor's own outer wire,
    #              at the floor's stock-top-referenced depth. Its inner wires go
    #              through the same solid-probing as counterbores: material
    #              below is a boss/step to machine around (the boss base ring
    #              probes solid and becomes an island; a deeper terrace probes
    #              blind and is left to its own pocket), empty-through is a bore
    #              to cut.
    #   holes    — the boss top's own inner wires, through ones only; anything
    #              with material below it belongs to some floor's field.
    # A normal plate (no proud faces) never enters this branch, so the primary
    # 2D-with-through-features path is untouched.
    #
    # The branch only applies when the MACHINED face is the proud (boss-top)
    # one. A force_side read of the OTHER face of a boss part — the flip-jig
    # reads a two-sided part from both faces — machines a clean full-footprint
    # plane, and taking this branch there built the OUTSIDE from the boss top:
    # the inverse mould came out boss-sized and the flipped part could never
    # seat. The machined face is the proud one exactly when its outer wire
    # encloses less than the opposite main's.
    # A machined face SPLIT into coplanar fragments by its own recesses is the
    # other case with the same shape: the walked fragment carries only a share
    # of the openings, so mouth-matching strands the rest (a jig with 105
    # grooves matched none of them and the whole part was refused). Those parts
    # take the floors-mode too — but only AFTER the normal path has actually
    # failed to match: a boss top standing inside a recess is also a coplanar
    # island at the machined level, and those parts match perfectly well.
    frag_up = len(coplanar) != 1
    proud_up = False
    footprint = outer
    if mains.get("proud") or frag_up:
        opp_main = lo if up_main is hi else hi
        opp_outer, _opp_inner = face_loops(opp_main["face"]["_occ"], origin, u, v)
        a_up = abs(_area2d(outer))
        a_opp = abs(_area2d(opp_outer)) if len(opp_outer) >= 3 else 0.0
        if mains.get("proud"):
            if a_opp <= 0.0:
                raise GeometryError("footprint face has no usable outer wire")
            proud_up = a_up < 0.95 * a_opp
        if proud_up or frag_up:
            footprint = opp_outer if a_opp > a_up else outer

    def _floors_mode():
        # An open-sided recess EATS the machined face: a rebate running out
        # through the edge leaves the face's outer wire as just the un-rebated
        # strip. The part's true footprint is whichever main face encloses the
        # most area — almost always the untouched opposite face.
        fp = footprint
        try:
            opp = lo if up_main is hi else hi
            opp_o, _oi = face_loops(opp["face"]["_occ"], origin, u, v)
            if len(opp_o) >= 3 and abs(_area2d(opp_o)) > abs(_area2d(fp)):
                fp = opp_o
        except Exception:                                  # noqa: BLE001
            pass
        loops = [{"role": "OUTSIDE", "pts": fp, "circle": None, "isOuter": True}]
        unused = [p for p in pockets if p["side"] == up_side]
        for cand in list(unused):
            try:
                f_outer, f_inner = face_loops(cand["face"]["_occ"], origin, u, v)
            except Exception:                              # noqa: BLE001
                continue                                   # stays unmatched -> rejected
            if len(f_outer) < 3:
                continue
            isl, bores = _split_floor_wires(shape, f_inner, origin, u, v,
                                            normal, cand["depth"], thickness)
            lp = {"role": "POCKET", "pts": f_outer, "circle": None,
                  "depth": cand["depth"], "through": False}
            if isl:
                lp["islands"] = isl
            loops.append(lp)
            loops.extend(bores)
            unused.remove(cand)
        # Through features live on EVERY fragment of the machined face — the
        # walked one carries only its share, and a groove-split jig lost its
        # screw holes entirely. Holes that pierce the whole board also appear
        # on the footprint face. Gather from all of them, let the solid probe
        # sort bores from blind features, and dedupe (a through hole shows on
        # both faces at the same in-plane position).
        rings = [r for r in inner if len(r) >= 3]
        for rec in coplanar:
            if rec is up_rec:
                continue
            try:
                _o2, in2 = face_loops(rec["_occ"], origin, u, v)
            except Exception:                              # noqa: BLE001
                continue
            rings += [r for r in in2 if len(r) >= 3]
        try:
            _o3, in3 = face_loops(opp_main["face"]["_occ"], origin, u, v)
            rings += [r for r in in3 if len(r) >= 3]
        except Exception:                                  # noqa: BLE001
            pass
        seen_rings = []
        uniq = []
        for r in rings:
            cx = sum(q[0] for q in r) / len(r)
            cy = sum(q[1] for q in r) / len(r)
            a = abs(_area2d(r))
            if any(abs(cx - sx) < 0.5 and abs(cy - sy) < 0.5 and
                   abs(a - sa) < 0.05 * max(sa, 1.0) for sx, sy, sa in seen_rings):
                continue
            seen_rings.append((cx, cy, a))
            uniq.append(r)
        _skip, up_bores = _split_floor_wires(shape, uniq,
                                             origin, u, v, normal, 0.0, thickness)
        loops.extend(up_bores)
        loops.extend(_angled_surface_loops(recs, origin, u, v, normal))
        # A half-lap bar is rebated at BOTH ends on OPPOSITE faces, so EVERY
        # main face is missing an end and "the biggest face" is still short —
        # the profile cut along it severed the lap (and its bolt holes) clean
        # off the part. Every column of material tops out at the machined
        # plane, at a recess floor, or at an angled surface, so the union of
        # those regions IS the footprint. Only adopt it when it genuinely
        # grows the outline: parts whose biggest face already covers the part
        # keep their exact ring, vertex for vertex.
        cover = [fp]
        for rec in coplanar:
            try:
                o2, _i2 = face_loops(rec["_occ"], origin, u, v)
                if len(o2) >= 3:
                    cover.append(o2)
            except Exception:                              # noqa: BLE001
                pass
        cover += [lp["pts"] for lp in loops[1:]
                  if lp["role"] in ("POCKET", "SURFACE") and len(lp["pts"]) >= 3]
        merged = _union_rings(cover)
        if merged and _area2d(merged) > _area2d(fp) + 1.0:
            loops[0]["pts"] = merged
        _attach_surface_maps(shape, loops, origin, u, v, normal, thickness)
        return _finish_loops(loops, pockets, up_side, flip, two_sided,
                             thickness, fragmented, len(unused), far_pockets)

    if proud_up:
        return _floors_mode()

    loops = [{"role": "OUTSIDE", "pts": outer, "circle": None, "isOuter": True}]

    # Inner loops on the machined face are holes, cut-outs, or pocket mouths.
    # A pocket is identified by its opening area matching a detected floor —
    # the same test the engine uses, so the two agree on what is a pocket.
    unused = [p for p in pockets if p["side"] == up_side]
    for ring in inner:
        if len(ring) < 3:
            continue
        a = _area2d(ring)
        # Match the mouth against the floor's GROSS area — see _floor_footprint.
        cand, best = None, None
        for p in unused:
            try:
                fa, centre = _floor_footprint(p, origin, u, v)
            except Exception:                              # noqa: BLE001
                fa, centre = p["area"], None
            # The floor must sit directly UNDER this mouth. By area alone a
            # O12 thru hole (113 mm2) claimed a O11.2 counterbore's floor
            # (99 mm2) across the part — the 50 mm2 tolerance swallowed the
            # difference — so two thru holes became phantom 6 mm pockets and
            # two real counterbores went undetected on the same board.
            if centre is not None and not _point_in_ring(ring, centre):
                continue
            d = abs(fa - a)
            if d <= max(50.0, 0.08 * max(fa, 1.0)) and (best is None or d < best):
                best, cand = d, p
        if cand is not None:
            lp = {"role": "POCKET", "pts": ring, "circle": None,
                  "depth": cand["depth"], "through": False}
            # The floor's inner wires are either bosses to machine around or bores
            # that carry on down and must be cut. The solid decides which.
            _, floor_inner = face_loops(cand["face"]["_occ"], origin, u, v)
            isl, bores = _split_floor_wires(shape, floor_inner, origin, u, v,
                                            normal, cand["depth"], thickness)
            if isl:
                lp["islands"] = isl
            loops.append(lp)
            loops.extend(bores)
            unused.remove(cand)
            continue
        # An unmatched ring with MATERIAL below it is not a through cut. When
        # the part carries freeform faces, it is the boundary of a sculpted
        # region (a dished seat, a formed relief) — before 3D surfacing existed
        # this fell through to INSIDE and the sculpt was CUT OUT of the part as
        # a slug. Probe the solid: through features are empty all the way down.
        if _has_freeform(shape, normal) and len(ring) >= 3:
            cx = sum(q[0] for q in ring) / len(ring)
            cy = sum(q[1] for q in ring) / len(ring)
            if any(_solid_below(shape, origin, u, v, normal, (cx, cy), drop)
                   for drop in (thickness * 0.5, max(0.5, thickness - 0.8))):
                loops.append({"role": "SURFACE", "pts": ring, "circle": None,
                              "isOuter": False})
                continue
        circ = _circle(ring)
        if circ:
            loops.append({"role": "DRILL", "pts": ring, "circle": circ, "isOuter": False})
        else:
            loops.append({"role": "INSIDE", "pts": ring, "circle": None, "isOuter": False})

    # Matching left floors stranded: a fragmented face hides openings on
    # fragments never walked, and an open-sided recess (a rebate running out
    # through the part edge) has NO closed mouth at all — its opening is a
    # notch in the outer wire. Either way the floors themselves say exactly
    # what to machine; rebuild from them.
    if unused:
        return _floors_mode()

    loops.extend(_angled_surface_loops(recs, origin, u, v, normal))
    _attach_surface_maps(shape, loops, origin, u, v, normal, thickness)
    return _finish_loops(loops, pockets, up_side, flip, two_sided,
                         thickness, fragmented, len(unused), far_pockets)


def _finish_loops(loops, pockets, up_side, flip, two_sided,
                  thickness, fragmented, unmatched, far_pockets=None):
    """Shift so the part's bounding box starts at (0, 0), matching what the
    exporter's flat outline returns. The two sources are interchangeable to
    everything downstream, so they must not differ in where the origin sits —
    islands and pocket mouths have to move with the profile or they end up
    machined somewhere else entirely. `far_pockets` (the other side's regions,
    already in this frame) get the same shift, or the mould builder would put
    its floors somewhere else than the part."""
    allpts = [p for lp in loops for p in lp["pts"]]
    mnx = min(p[0] for p in allpts)
    mny = min(p[1] for p in allpts)
    for lp in loops:
        lp["pts"] = [(x - mnx, y - mny) for x, y in lp["pts"]]
        if lp.get("circle"):
            cx, cy, r = lp["circle"]
            lp["circle"] = (cx - mnx, cy - mny, r)
        if lp.get("islands"):
            lp["islands"] = [[(x - mnx, y - mny) for x, y in isl]
                             for isl in lp["islands"]]
        if lp.get("grid"):                       # a SURFACE heightfield rides along
            lp["grid"] = dict(lp["grid"], ox=lp["grid"]["ox"] - mnx,
                              oy=lp["grid"]["oy"] - mny)
    for fp in far_pockets or []:
        fp["pts"] = [(x - mnx, y - mny) for x, y in fp["pts"]]
        fp["inner"] = [[(x - mnx, y - mny) for x, y in w] for w in fp["inner"]]

    return {"loops": loops, "pockets": pockets, "up_side": up_side, "flip": flip,
            "two_sided": two_sided, "thickness": thickness,
            "fragmented": fragmented, "unmatched_pockets": unmatched,
            "far_pockets": far_pockets or []}
