"""
Render a part thumbnail from its solid, with no Onshape API call.

The review page's icons used to come from Onshape's shaded-view endpoint — one
server-side render per part, which is both a network round trip and a chunk of
the annual API allowance. Given the geometry is already in hand when a pack is
built from a STEP file, the same picture can be drawn locally.

This uses OpenCascade's hidden-line removal to project the solid to 2-D, then
draws the visible edges with Pillow. That yields a clean line drawing rather
than a shaded render — which for picking a part off a fabrication list is
arguably easier to read, and it costs no GPU, no vtk and no network.
"""

from __future__ import annotations

import base64
import io
import math

# Looking down (1, 1, 1) gives the standard isometric-ish view Onshape's own
# thumbnails use, so the review page still reads the way people are used to.
_VIEW_DIR = (1.0, 1.0, 1.0)
_CHORD_TOL = 0.15          # mm; curve sampling for the projected edges
_MARGIN_PX = 6


def projected_polylines(shape, view_dir=None) -> "list[list[tuple[float, float]]]":
    """Visible edges of `shape` from `view_dir`, as 2-D polylines in mm.

    Public because the turned-part drawing needs the same projection the
    thumbnails use, just from a side elevation rather than an isometric.
    """
    return _projected_polylines(shape, view_dir or _VIEW_DIR)


def _projected_polylines(shape, view_dir=_VIEW_DIR) -> "list[list[tuple[float, float]]]":
    """Visible edges of `shape` seen from `view_dir`, as 2-D polylines."""
    from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
    from OCP.HLRAlgo import HLRAlgo_Projector
    from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(HLRAlgo_Projector(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*view_dir))))
    algo.Update()
    algo.Hide()

    to_shape = HLRBRep_HLRToShape(algo)
    lines: list[list[tuple[float, float]]] = []
    for compound in (to_shape.VCompound(), to_shape.OutLineVCompound()):
        if compound is None:
            continue
        exp = TopExp_Explorer(compound, TopAbs_ShapeEnum.TopAbs_EDGE)
        while exp.More():
            edge = TopoDS.Edge_s(exp.Current())
            try:
                curve = BRepAdaptor_Curve(edge)
                sampler = GCPnts_QuasiUniformDeflection(curve, _CHORD_TOL)
                if sampler.IsDone() and sampler.NbPoints() >= 2:
                    lines.append([
                        (sampler.Value(i).X(), sampler.Value(i).Y())
                        for i in range(1, sampler.NbPoints() + 1)
                    ])
            except Exception:
                pass
            exp.Next()
    return lines


# ---------------------------------------------------------------------------
# Shaded rendering
# ---------------------------------------------------------------------------
# A line drawing is ambiguous on the very parts that most need telling apart —
# handed brackets, plates with a step in them, a tube against a solid bar all
# reduce to a similar tangle of lines. A shaded image reads as the solid it is,
# which is the whole job of the icon on the review page and the reference.
#
# Rendered with a painter's-algorithm rasteriser over the tessellation: no GPU,
# no vtk, no display server, so it behaves the same on Render as it does here.
# Triangles are drawn far-to-near, which is exact for a convex body and wrong
# only where two faces of a concave one interpenetrate on screen — invisible at
# icon size, and far cheaper than a per-pixel depth buffer in Python.

_MESH_DEFLECTION = 0.35     # mm; chord tolerance for the display mesh
_SUPERSAMPLE = 3            # render this many times larger, then downscale
_BASE_RGB = (176, 186, 200)  # steel
_LIGHT = (0.35, -0.55, 0.75)  # over the viewer's shoulder, slightly above


def _tessellate(shape, deflection: float = None, max_tris: int = 0,
                angular: float = 0.5) -> list:
    """[(p1, p2, p3), ...] world-space triangles of every face.

    `deflection` defaults to the icon setting. With `max_tris`, the mesh is
    rebuilt coarser if the first attempt overshoots, so one enormous part
    cannot dominate the page weight.
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_Orientation, TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    if deflection is None:
        deflection = _MESH_DEFLECTION
    BRepMesh_IncrementalMesh(shape, deflection, False, angular, True)

    tris = []
    exp = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        reversed_face = face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        nodes = [tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1)]
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            if reversed_face:          # keep the winding outward-facing
                a, c = c, a
            p = [nodes[a - 1], nodes[b - 1], nodes[c - 1]]
            tris.append(tuple((q.X(), q.Y(), q.Z()) for q in p))
        # `tri` is released with the face; nodes are already copied out.
    if max_tris and len(tris) > max_tris:
        # Too heavy: retess at a deflection scaled by how far over we are.
        factor = (len(tris) / max_tris) ** 0.5
        return _tessellate(shape, deflection=deflection * factor)
    return tris


def _norm(v):
    m = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / m, v[1] / m, v[2] / m)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def render_shaded(shape, size_px: int = 220, deflection: float = None,
                  angular: float = 0.5) -> "str | None":
    """Base64 PNG of one solid as a shaded 3-D image, or None if it can't be
    drawn (no Pillow, no tessellation, degenerate geometry).

    The icon default meshes at 0.35mm / 0.5rad, which is light for a page of
    thumbnails but shows a cylinder as a dozen flat facets — a hexagon at
    drawing size. Callers rendering ONE part large (the turned-part drawings)
    pass a finer deflection and angular tolerance instead.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None
    try:
        tris = _tessellate(shape, deflection=deflection, angular=angular)
    except Exception:
        return None
    if not tris:
        return None

    # View basis: looking along -view_dir, world Z up.
    d = _norm((-_VIEW_DIR[0], -_VIEW_DIR[1], -_VIEW_DIR[2]))
    right = _norm(_cross(d, (0.0, 0.0, 1.0)))
    if not any(right):
        right = (1.0, 0.0, 0.0)
    up = _norm(_cross(right, d))
    light = _norm(_LIGHT)

    faces = []
    for p1, p2, p3 in tris:
        n = _cross((p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]),
                   (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]))
        if not any(n):
            continue
        n = _norm(n)
        if _dot(n, d) > 0.0:
            continue                     # facing away from the viewer
        uv = [(_dot(p, right), _dot(p, up)) for p in (p1, p2, p3)]
        depth = (_dot(p1, d) + _dot(p2, d) + _dot(p3, d)) / 3.0
        faces.append((depth, uv, abs(_dot(n, light))))
    if not faces:
        return None

    xs = [u for _dp, uv, _s in faces for u, _v in uv]
    ys = [v for _dp, uv, _s in faces for _u, v in uv]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    if w <= 0 and h <= 0:
        return None

    big = size_px * _SUPERSAMPLE
    margin = _MARGIN_PX * _SUPERSAMPLE
    usable = big - 2 * margin
    scale = usable / max(w, h, 1e-9)
    off_x = margin + (usable - w * scale) / 2
    off_y = margin + (usable - h * scale) / 2
    x0, y1 = min(xs), max(ys)

    img = Image.new("RGB", (big, big), "white")
    draw = ImageDraw.Draw(img)
    # Farthest first, so nearer triangles paint over them.
    faces.sort(key=lambda f: -f[0])
    for _depth, uv, lam in faces:
        shade = 0.30 + 0.70 * lam          # ambient + diffuse
        colour = tuple(min(255, int(c * shade)) for c in _BASE_RGB)
        draw.polygon([(off_x + (u - x0) * scale, off_y + (y1 - v) * scale)
                      for u, v in uv], fill=colour, outline=colour)

    img = img.resize((size_px, size_px), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def mesh_payload(shape, max_tris: int = 900) -> "dict | None":
    """A compact triangle mesh for the review page's interactive 3-D view.

    Embedded in the page for every part, so it has to be small: positions are
    quantised to 16-bit over the part's own bounding box and normals to 8-bit,
    which is far finer than anything visible when the point is to recognise
    which part you are holding. Roughly 27 bytes a triangle before base64.

    Returns {"s": scale, "o": origin, "n": tri_count, "p": pos_b64, "d": nrm_b64}
    or None when there is nothing to draw.
    """
    import array
    import struct                                    # noqa: F401  (docs the layout)

    try:
        # Coarser than the icon: this is for turning a part over, not measuring
        # it, and every triangle is paid for in page weight.
        tris = _tessellate(shape, deflection=None, max_tris=max_tris)
    except Exception:
        return None
    if not tris:
        return None

    pts = [c for t in tris for p in t for c in p]
    minx = min(pts[0::3]); maxx = max(pts[0::3])
    miny = min(pts[1::3]); maxy = max(pts[1::3])
    minz = min(pts[2::3]); maxz = max(pts[2::3])
    span = max(maxx - minx, maxy - miny, maxz - minz, 1e-6)
    scale = span / 32000.0                # 0..32000 fits inside a signed int16

    pos = array.array("h")
    nrm = array.array("b")
    for p1, p2, p3 in tris:
        n = _cross((p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]),
                   (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]))
        if not any(n):
            continue
        n = _norm(n)
        qn = [max(-127, min(127, int(round(c * 127)))) for c in n]
        for p in (p1, p2, p3):
            pos.append(min(32767, int(round((p[0] - minx) / scale))))
            pos.append(min(32767, int(round((p[1] - miny) / scale))))
            pos.append(min(32767, int(round((p[2] - minz) / scale))))
            nrm.extend(qn)

    if not len(pos):
        return None
    return {
        "s": scale,
        "n": len(pos) // 9,
        "p": base64.b64encode(pos.tobytes()).decode("ascii"),
        "d": base64.b64encode(nrm.tobytes()).decode("ascii"),
    }


def render_thumbnail(shape, size_px: int = 220) -> "str | None":
    """Base64 PNG (no data: prefix) of one solid, matching what the review page
    expects from the Onshape shaded-view endpoint. None if it can't be drawn.

    Shaded, falling back to the hidden-line drawing if tessellation fails.
    """
    shaded = render_shaded(shape, size_px)
    if shaded:
        return shaded

    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None

    try:
        lines = _projected_polylines(shape)
    except Exception:
        return None
    pts = [p for line in lines for p in line]
    if len(pts) < 2:
        return None

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    if w <= 0 and h <= 0:
        return None

    usable = size_px - 2 * _MARGIN_PX
    scale = usable / max(w, h, 1e-9)
    # Centre the drawing, and flip Y — image rows run downward.
    off_x = _MARGIN_PX + (usable - w * scale) / 2
    off_y = _MARGIN_PX + (usable - h * scale) / 2
    x0, y1 = min(xs), max(ys)

    img = Image.new("RGB", (size_px, size_px), "white")
    draw = ImageDraw.Draw(img)
    for line in lines:
        pixels = [(off_x + (x - x0) * scale, off_y + (y1 - y) * scale)
                  for x, y in line]
        if len(pixels) >= 2:
            draw.line(pixels, fill=(31, 41, 55), width=1)

    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode("ascii")
