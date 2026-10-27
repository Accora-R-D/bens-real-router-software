"""Extract flat 2D profiles of ~18mm sheet parts from the Swing Arm Assembly,
one layered DXF per unique part, ready for nesting + CAM.

Method: bodydetails -> largest planar face (normal = thickness axis) ->
walk each loop's coedges -> project the analytic B-rep edges onto the face
plane -> emit a millimetre DXF (R12).

Layers:
  OUTSIDE  outer loop  (profile cut-out)
  DRILL    circular inner loops (holes -> emitted as true CIRCLE)
  INSIDE   non-circular inner loops (pockets / internal cut-outs)
"""
import sys, math
from pathlib import Path
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass
# The Onshape client is gone. Everything in this file that is pure geometry --
# detect_pockets, classify_sheet, the vector maths, the DXF writer -- is used by
# the wood path through backend/services/wood_geometry.py, which feeds it OCC
# faces. The functions that FETCH from Onshape (sheet_thickness, part_loops,
# part_features) belong to the metal/die path, which has not been converted; they
# fail here with an explanation instead of a NameError on a client that is not
# there.
class _NoOnshape:
    def __getattr__(self, _name):
        raise NotImplementedError(
            "This function still reads geometry from Onshape and was not "
            "converted to STEP. The wood path uses "
            "backend.services.wood_geometry.part_geometry(); the metal/die path "
            "still needs the same treatment.")


client = _NoOnshape()
DID = WID = EID = ""


def classify_sheet(dims, faces, volume_mm3=None):
    """Pure (no network) flat-sheet detection from already-fetched geometry.

    `dims`       — the part's three bbox extents in mm, SORTED ascending.
    `faces`      — the body's faces list (from /bodydetails: bodies[i]["faces"]).
    `volume_mm3` — solid volume in mm^3 (from /massproperties), or None if absent.

    Returns (thickness_mm, is_platelike). thickness = perpendicular gap between
    the dominant pair of PARALLEL planar faces (top/bottom of a plate), found via
    the same normal *line* (|dot|>0.98) — this works even when the part is modelled
    at an angle, where the bounding-box minimum is NOT the thickness. is_platelike
    requires that face pair to fill the footprint and agree with volume/area, which
    rejects chunky/bracket parts that merely have a thin feature.
    """
    footprint = dims[1]*dims[2] or 1.0
    planar = [(f["area"]*1e6, f["surface"]["normal"], f["surface"]["origin"])
              for f in faces if f.get("surface", {}).get("type") == "PLANE"]
    if not planar:
        return dims[0], False
    big_area = max(a for a, _, _ in planar)
    sep, dom = None, 0.0
    for i in range(len(planar)):
        ai, ni, oi = planar[i]
        for j in range(i+1, len(planar)):
            aj, nj, oj = planar[j]
            d = ni["x"]*nj["x"] + ni["y"]*nj["y"] + ni["z"]*nj["z"]
            if abs(d) > 0.98:                                   # same normal line
                s = abs((oj["x"]-oi["x"])*ni["x"] + (oj["y"]-oi["y"])*ni["y"]
                        + (oj["z"]-oi["z"])*ni["z"]) * 1000
                sc = min(ai, aj)
                if s > 0.1 and sc > dom:
                    dom, sep = sc, s
    if sep is None:
        return dims[0], False
    voa = (volume_mm3 / big_area) if volume_mm3 else None        # mm^3 / mm^2
    # A uniform-thickness flat plate satisfies volume == projected_area * thickness,
    # i.e. voa (volume / largest planar face) == the face-pair separation. That holds
    # even for L-shaped / diagonal / large-cutout outlines that don't fill their
    # bounding rectangle, so it is the primary test. The footprint-fill ratio wrongly
    # rejected such plates (e.g. an 18mm part whose outline fills only 35% of its
    # bbox) and is now only a fallback for when mass properties are unavailable.
    if voa is not None:
        # Relative tolerance: a true plate has voa == sep almost exactly; an
        # absolute 1.5mm slop wrongly accepted non-uniform thin parts (e.g. a
        # ~1mm part with voa 2.5 vs sep 1.2). Scale with thickness, floor 0.3mm.
        platelike = abs(voa - sep) <= max(0.3, 0.10 * sep)
    else:
        platelike = (dom / footprint) >= 0.45
    return round(sep, 2), platelike


def sheet_thickness(doc, mv, elem, pid):
    """Network wrapper around `classify_sheet`: fetch this one part's bbox,
    faces and volume, then classify. Used by the CLI drivers and as the
    per-part fallback; `list_parts` batches the same data per element and
    calls `classify_sheet` directly (far fewer round-trips)."""
    base = f"/parts/d/{doc}/m/{mv}/e/{elem}/partid/{pid}"
    bb = client.get(base + "/boundingboxes")
    dims = sorted([(bb["highX"]-bb["lowX"])*1000, (bb["highY"]-bb["lowY"])*1000,
                   (bb["highZ"]-bb["lowZ"])*1000])
    faces = client.get(base + "/bodydetails")["bodies"][0]["faces"]
    vol = None
    try:
        mp = client.get(base + "/massproperties")
        vol = mp["bodies"][pid]["volume"][0] * 1e9               # m^3 -> mm^3
    except Exception:
        pass
    return classify_sheet(dims, faces, vol)

MM = 1000.0                      # Onshape SI metres -> mm
THICK_MIN, THICK_MAX = 17.0, 19.0
ARC_CHORD_TOL = 0.02             # mm: max chord deviation when tessellating arcs
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

# ---------- vector helpers (work in metres, convert at emit) ----------
def v3(d):
    return (d["x"], d["y"], d["z"])
def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def dot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a):
    m = math.sqrt(dot(a, a)) or 1.0
    return (a[0]/m, a[1]/m, a[2]/m)

def make_basis(n):
    """Right-handed (u, v) spanning the plane with normal n; u x v = n."""
    n = norm(n)
    seed = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = norm(cross(seed, n))
    v = cross(n, u)              # already unit, right-handed
    return u, v

def to2d(p, origin, u, v):
    d = sub(p, origin)
    return (dot(d, u)*MM, dot(d, v)*MM)   # mm

# ---------- DXF R12 writer ----------
class Dxf:
    def __init__(self):
        self.e = []
        self.layers = set()
    def line(self, layer, a, b):
        self.layers.add(layer)
        self.e += ["0","LINE","8",layer,
                   "10",f"{a[0]:.5f}","20",f"{a[1]:.5f}","30","0.0",
                   "11",f"{b[0]:.5f}","21",f"{b[1]:.5f}","31","0.0"]
    def circle(self, layer, c, r):
        self.layers.add(layer)
        self.e += ["0","CIRCLE","8",layer,
                   "10",f"{c[0]:.5f}","20",f"{c[1]:.5f}","30","0.0",
                   "40",f"{r:.5f}"]
    def dumps(self):
        tbl = ["0","SECTION","2","TABLES","0","TABLE","2","LAYER","70",str(len(self.layers))]
        for i, lyr in enumerate(sorted(self.layers)):
            tbl += ["0","LAYER","2",lyr,"70","0","62",str((i % 7)+1),"6","CONTINUOUS"]
        tbl += ["0","ENDTAB","0","ENDSEC"]
        hdr = ["0","SECTION","2","HEADER","9","$INSUNITS","70","4","0","ENDSEC"]  # 4 = mm
        ent = ["0","SECTION","2","ENTITIES"] + self.e + ["0","ENDSEC"]
        return "\n".join(hdr + tbl + ent + ["0","EOF",""])

# ---------- arc tessellation in the 2D plane ----------
def tess_arc(c2, r2, s2, e2, m2):
    """Polyline points from start s2 to end e2 on circle (c2,r2), passing through mid m2."""
    a0 = math.atan2(s2[1]-c2[1], s2[0]-c2[0])
    a1 = math.atan2(e2[1]-c2[1], e2[0]-c2[0])
    am = math.atan2(m2[1]-c2[1], m2[0]-c2[0])
    # choose sweep direction (ccw) that passes through am
    def ccw_span(a, b):
        d = (b - a) % (2*math.pi)
        return d
    span_ccw = ccw_span(a0, a1)
    # does ccw path a0->a1 contain am?
    if ccw_span(a0, am) <= span_ccw + 1e-9:
        sweep = span_ccw
    else:
        sweep = span_ccw - 2*math.pi          # go clockwise
    n = max(2, int(abs(sweep) / (2*math.acos(max(1-ARC_CHORD_TOL/max(r2,1e-6),-1))) ) + 1)
    pts = []
    for i in range(n+1):
        a = a0 + sweep*i/n
        pts.append((c2[0]+r2*math.cos(a), c2[1]+r2*math.sin(a)))
    return pts

# ---------- per-part extraction ----------
def chain_segments(segs, tol=1e-3):
    """Connect edge segments into one ordered ring by matching coincident
    endpoints (handles arbitrary coedge order/orientation)."""
    segs = [list(s) for s in segs if len(s) >= 2]
    if not segs:
        return []
    def close(a, b):
        return abs(a[0]-b[0]) < tol and abs(a[1]-b[1]) < tol
    ring = segs.pop(0)
    while segs:
        tail = ring[-1]
        best_i, best_rev, best_d = None, False, None
        for i, s in enumerate(segs):
            for rev, end in ((False, s[0]), (True, s[-1])):
                d = (end[0]-tail[0])**2 + (end[1]-tail[1])**2
                if best_d is None or d < best_d:
                    best_d, best_i, best_rev = d, i, rev
        s = segs.pop(best_i)
        if best_rev:
            s = s[::-1]
        ring.extend(s[1:] if close(s[0], tail) else s)
    return ring


def loop_edges(loop, edge_by_id):
    out = []
    for co in loop["coedges"]:
        e = edge_by_id.get(co["edgeId"])
        if e:
            out.append((e, co.get("orientation", True)))
    return out

def is_circle_loop(edges):
    """All arcs sharing one centre+radius -> (center3d, radius). Else None."""
    cen = rad = None
    for e, _ in edges:
        cv = e.get("curve") or {}
        if cv.get("type") != "CIRCLE":
            return None
        o = v3(cv["origin"]); r = cv["radius"]
        if cen is None:
            cen, rad = o, r
        elif abs(r-rad) > 1e-6 or dot(sub(o,cen),sub(o,cen)) > 1e-9:
            return None
    return (cen, rad) if cen else None

def part_loops(name, doc, mv, elem, pid):
    """Return classified closed polylines (mm) for a part's flat face.

    Each loop: {role: OUTSIDE|INSIDE|DRILL, pts: [(x,y),...] closed,
                circle: (cx,cy,r) or None}.
    Arcs are tessellated into pts; full circles also keep analytic `circle`.
    """
    bd = client.get(f"/parts/d/{doc}/m/{mv}/e/{elem}/partid/{pid}/bodydetails")
    body = bd["bodies"][0]
    edge_by_id = {e["id"]: e for e in body["edges"]}
    planar = [f for f in body["faces"] if f.get("surface", {}).get("type") == "PLANE"]
    face = max(planar, key=lambda f: f.get("area", 0))
    origin = v3(face["surface"]["origin"])
    u, vv = make_basis(v3(face["surface"]["normal"]))

    loops = []
    for loop in face["loops"]:
        edges = loop_edges(loop, edge_by_id)
        outer = loop.get("isOuter", False)
        circ = is_circle_loop(edges)
        # build each edge as an ordered point segment (start->end), tessellating arcs
        segs = []
        for e, _ in edges:
            g = e["geometry"]
            s = to2d(v3(g["startPoint"]), origin, u, vv)
            t = to2d(v3(g["endPoint"]),   origin, u, vv)
            if "Arc" in g["btType"]:
                cv = e["curve"]
                c2 = to2d(v3(cv["origin"]), origin, u, vv); r2 = cv["radius"]*MM
                m2 = to2d(v3(g["midPoint"]), origin, u, vv)
                segs.append(tess_arc(c2, r2, s, t, m2))
            else:
                segs.append([s, t])
        pts = chain_segments(segs)
        if circ and not outer:
            c2 = to2d(circ[0], origin, u, vv)
            role, circle = "DRILL", (c2[0], c2[1], circ[1]*MM)
        else:
            role, circle = ("OUTSIDE" if outer else "INSIDE"), None
        loops.append({"role": role, "pts": pts, "circle": circle})
    return loops


# ---------- pocket detection + orientation ----------
def _outward_normal(face):
    """True outward normal of a bodydetails face. REST surface.normal canonicalises
    both faces of a parallel pair to the SAME direction (can't tell a top pocket
    from a bottom one); face["orientation"] is the recovery flag — outward =
    surface.normal when orientation is True, else negated. Verified on Part B
    (bottom main -> -z, top main -> +z, pocket floor -> -z)."""
    n = v3(face["surface"]["normal"])
    return n if face.get("orientation", True) else (-n[0], -n[1], -n[2])


def detect_pockets(planar, thick_hint=None):
    """Detect blind pockets from a single part's PLANE faces (bodydetails — already
    scoped to the partId, so multi-body source studios don't bleed in).

    Returns (mains, pockets, thickness):
      mains = {"hi": face, "lo": face}  — the two full plate faces (hi = larger Z).
      pockets = [{depth, area, side('TOP'|'BOT'), oz, face}]  — one per blind floor.
    Each floor opens through the main face its OUTWARD normal points at (sign-
    invariant); depth = gap to that main. A real pocket is strictly inside the
    plate (0 < depth < thickness), which rejects walls/slivers and impossible
    depths. Through-holes have no floor so they never appear."""
    feats = []
    for f in planar:
        feats.append({"area": f["area"]*1e6, "n": _outward_normal(f),
                      "o": v3(f["surface"]["origin"]), "face": f})
    if len(feats) < 2:
        return None, [], thick_hint
    big = max(feats, key=lambda f: f["area"])
    axis = big["n"]
    for f in feats:
        f["par"] = dot(f["n"], axis); f["off"] = dot(f["o"], axis) * MM   # mm along axis
    flat = [f for f in feats if abs(f["par"]) > 0.9]                      # parallel to the plate
    if len(flat) < 2:
        return None, [], thick_hint
    mains = sorted(flat, key=lambda f: -f["area"])[:2]
    thickness = abs(mains[0]["off"] - mains[1]["off"])
    # Order the two plate faces along the THICKNESS AXIS. World-Z only distinguishes
    # them when the plate lies flat; a part standing on edge (thickness along X/Y)
    # leaves both mains at the same Z, which collapses hi==lo (Python max/min both
    # return the first on a tie) and silently breaks every TOP/BOT decision below.
    # Use Z when it actually separates them (the proven flat-plate path), else fall
    # back to the position along the thickness axis.
    if abs(mains[0]["o"][2] - mains[1]["o"][2]) * MM > 0.1:
        hi = max(mains, key=lambda m: m["o"][2]); lo = min(mains, key=lambda m: m["o"][2])
    else:
        hi = max(mains, key=lambda m: m["off"]); lo = min(mains, key=lambda m: m["off"])

    # The axis is the biggest face's outward normal, which can point either way
    # through the plate — and hi/lo were just chosen by HEIGHT, so their order
    # along the axis flips with its sign. Point the axis from lo to hi before
    # the proud search below, or "above hi" lands in `below` and a boss standing
    # on the TOP face is grafted onto the BOTTOM main: on an 18mm plate with a
    # 2mm boss that read thickness = 2 and the boss top as a plate face. Never
    # fires when the axis already agrees with the height order.
    if hi["off"] < lo["off"]:
        axis = tuple(-c for c in axis)
        for f in feats:
            f["off"] = -f["off"]

    # PROUD features (RF-weld-die topology): coaxial planar faces OUTSIDE the
    # area-chosen main pair — e.g. 2mm weld bars standing 3mm proud of the relief
    # plane. Their tips are tiny, so the area vote picks back<->relief and reads the
    # bars as unmachinable. The stock must span the OUTERMOST planes: re-anchor that
    # side's main to the outermost proud plane; the former main then falls strictly
    # inside the band and registers as a plain pocket floor below (which is exactly
    # how it is machined — face everything down except the ribs). Never fires on a
    # normal plate (its mains ARE the outermost planes).
    proud = False
    lo_off, hi_off = min(lo["off"], hi["off"]), max(lo["off"], hi["off"])
    below = [f for f in flat if f["off"] < lo_off - 0.3 and f["area"] > 5.0]
    above = [f for f in flat if f["off"] > hi_off + 0.3 and f["area"] > 5.0]
    if below and sum(f["area"] for f in below) > 50.0:
        lo = min(below, key=lambda f: f["off"]); proud = True
    if above and sum(f["area"] for f in above) > 50.0:
        hi = max(above, key=lambda f: f["off"]); proud = True
    if proud:
        mains = [hi, lo]
        thickness = abs(hi["off"] - lo["off"])
    pockets = []
    for f in flat:
        if f in mains:
            continue
        opening = max(mains, key=lambda m: dot(f["n"], m["n"]))           # main it opens through
        depth = abs(f["off"] - opening["off"])
        if depth <= 0.3 or depth >= thickness - 0.3:                      # must be strictly inside
            continue
        pockets.append({"depth": round(depth, 2), "area": round(f["area"], 1),
                        "side": "TOP" if opening is hi else "BOT",
                        "oz": f["o"][2]*MM, "off": f["off"], "face": f["face"], "opening": opening})
    return {"hi": hi, "lo": lo, "axis": axis, "proud": proud}, pockets, round(thickness, 2)


def _area2d(pts):
    a = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        a += x0*y1 - x1*y0
    return abs(a) / 2.0


def _face_loops(face, origin, u, vv, edge_by_id):
    """Project one face's loops into the (origin,u,v) plane — same machinery as
    part_loops, reusable for any chosen face (the up face, a pocket floor, ...)."""
    out = []
    for loop in face["loops"]:
        edges = loop_edges(loop, edge_by_id)
        outer = loop.get("isOuter", False)
        circ = is_circle_loop(edges)
        segs = []
        for e, _ in edges:
            g = e["geometry"]
            s = to2d(v3(g["startPoint"]), origin, u, vv)
            t = to2d(v3(g["endPoint"]), origin, u, vv)
            if "Arc" in g["btType"]:
                cv = e["curve"]
                c2 = to2d(v3(cv["origin"]), origin, u, vv); r2 = cv["radius"]*MM
                m2 = to2d(v3(g["midPoint"]), origin, u, vv)
                segs.append(tess_arc(c2, r2, s, t, m2))
            else:
                segs.append([s, t])
        pts = chain_segments(segs)
        if circ and not outer:
            c2 = to2d(circ[0], origin, u, vv)
            out.append({"role": "DRILL", "pts": pts, "circle": (c2[0], c2[1], circ[1]*MM), "isOuter": False})
        else:
            out.append({"role": "OUTSIDE" if outer else "INSIDE", "pts": pts, "circle": None, "isOuter": outer})
    return out


def _project_face_outer(face, origin, u, vv, edge_by_id):
    """Project a face's OUTER loop into (origin,u,v) — the loop's true outline in mm.
    Works for any face (a plate face, or a pocket floor whose outline is the pocket
    shape), independent of how the rest of that side is split into faces."""
    for lp in _face_loops(face, origin, u, vv, edge_by_id):
        if lp.get("isOuter"):
            return lp["pts"]
    return None
def _through_cylinder_loops(faces, origin, u, vv, axis, hi, lo, seen, tol=0.4):
    """Return analytic circles for concave cylinders that cross the full stock band.

    A through-hole opening on a stepped face is not necessarily one closed planar
    loop: every ledge can split its upper rim into separate arcs. The old extractor
    only inspected planar mouth loops, so those holes disappeared whenever the CAM
    selected the stepped/pocketed side. Bodydetails still exposes the continuous
    concave cylindrical wall, including a bounding box spanning both stock faces.

    Restricting this fallback to concave cylinders parallel to the thickness axis and
    spanning both main-face offsets excludes exterior rounds/fillets and blind holes.
    `seen` shares the planar-loop de-duplication key used by part_features().
    """
    stock_lo, stock_hi = sorted((hi["off"], lo["off"]))
    out = []
    for face in faces:
        surf = face.get("surface") or {}
        if surf.get("type") != "CYLINDER" or face.get("orientation", True):
            continue                                      # convex exterior cylinder / fillet
        cyl_axis = v3(surf.get("axis", (0, 0, 0)))
        if abs(dot(cyl_axis, axis)) < 0.999:
            continue                                      # side bore, not a sheet through-hole
        box = face.get("box") or {}
        if not box.get("minCorner") or not box.get("maxCorner"):
            continue
        mn, mx = v3(box["minCorner"]), v3(box["maxCorner"])
        # Exact projection range of an axis-aligned bounding box onto `axis`.
        p0 = tuple(mn[i] if axis[i] >= 0 else mx[i] for i in range(3))
        p1 = tuple(mx[i] if axis[i] >= 0 else mn[i] for i in range(3))
        cyl_lo, cyl_hi = sorted((dot(p0, axis) * MM, dot(p1, axis) * MM))
        if cyl_lo > stock_lo + tol or cyl_hi < stock_hi - tol:
            continue                                      # capped before a stock face -> blind
        radius = float(surf.get("radius") or 0.0) * MM
        if radius <= 0:
            continue
        cx, cy = to2d(v3(surf["origin"]), origin, u, vv)
        key = (round(cx, 1), round(cy, 1), round(radius, 1))
        if key in seen:
            continue
        seen.add(key)
        # Match the extractor's chord tolerance so the analytic circle offsets and
        # arc-fits cleanly in the downstream CAM pipeline.
        arg = max(-1.0, min(1.0, 1.0 - ARC_CHORD_TOL / radius))
        n = max(24, int(math.ceil(math.pi / math.acos(arg))))
        pts = [(cx + radius * math.cos(2 * math.pi * i / n),
                cy + radius * math.sin(2 * math.pi * i / n)) for i in range(n)]
        out.append({"role": "DRILL", "pts": pts, "circle": (cx, cy, radius)})
    return out




def _drop_features_outside_outline(loops, name=""):
    """Discard inner features that don't fit inside the part's own outline.

    A hole or cut-out is by definition inside the part. A loop that isn't is a
    misread: most often a large-radius curved EDGE fitted as if it were a full
    circle, which then gets cut as a giant arc — the toolpath wanders off the
    sheet and the safety gate fails, so it must never reach the nester.

    Bbox containment (not point-in-polygon) is deliberate: it only removes
    features that cannot belong to the part under any interpretation, so a
    legitimate hole near a concave edge is never thrown away.
    """
    outs = [lp for lp in loops if lp["role"] == "OUTSIDE" and lp.get("pts")]
    if not outs:
        return loops, []
    pts = max(outs, key=lambda lp: abs(_area2d(lp["pts"])))["pts"]
    ox0 = min(p[0] for p in pts); ox1 = max(p[0] for p in pts)
    oy0 = min(p[1] for p in pts); oy1 = max(p[1] for p in pts)
    tol = 0.5                                   # tessellation slack (mm)

    kept, dropped = [], []
    for lp in loops:
        if lp["role"] == "OUTSIDE" or not lp.get("pts"):
            kept.append(lp)
            continue
        xs = [p[0] for p in lp["pts"]]; ys = [p[1] for p in lp["pts"]]
        if (min(xs) < ox0 - tol or max(xs) > ox1 + tol
                or min(ys) < oy0 - tol or max(ys) > oy1 + tol):
            dropped.append(lp)
        else:
            kept.append(lp)
    if dropped:
        for lp in dropped:
            c = lp.get("circle")
            what = (f"{lp['role']} r={c[2]:.1f}mm at ({c[0]:.1f},{c[1]:.1f})" if c
                    else f"{lp['role']} loop")
            print(f"  [extract] {name}: ignoring {what} — outside the part outline "
                  f"x[{ox0:.1f},{ox1:.1f}] y[{oy0:.1f},{oy1:.1f}] (misread edge, not a feature)")
    return kept, dropped


def part_features(name, doc, mv, elem, pid):
    """Profile + pockets + orientation for a sheet part, machined pocket-side-up.

    Returns {loops, pockets, up_side, flip, two_sided, label}:
      loops    — OUTSIDE / INSIDE / DRILL / POCKET (POCKET carries `depth`), all in
                 the machined-face 2D frame.
      up_side  — 'TOP'|'BOT' main face chosen as up.
      flip     — True if the machined face is the lower one (nester mirrors).
      two_sided— pockets on both faces (needs a second setup; only up-side cut).

    Two extraction paths share one detector. The proven flat-plate path (thickness
    along world-Z, machined side a single full-footprint face) walks that face's
    loops. The robust path (thickness along X/Y, or a machined side fragmented into
    several coplanar faces by its own recesses) rebuilds the profile from whichever
    main face has the fullest outline, pockets from their floor faces, and holes +
    cut-outs from the real loops on the machined-face plane — so fillets/rounded
    corners are never mistaken for holes, and far-side features (e.g. back-face
    countersinks) are ignored because their mouths live on the other plane."""
    bd = client.get(f"/parts/d/{doc}/m/{mv}/e/{elem}/partid/{pid}/bodydetails")
    body = bd["bodies"][0]
    edge_by_id = {e["id"]: e for e in body["edges"]}
    faces = body["faces"]
    planar = [f for f in faces if f.get("surface", {}).get("type") == "PLANE"]
    mains, pockets, thickness = detect_pockets(planar)

    # engraving label: Onshape Part Number when set, else the name (instance suffix
    # stripped), upper-cased to the marking charset.
    try:
        pn = next((p.get("partNumber") for p in client.get(f"/parts/d/{doc}/m/{mv}/e/{elem}")
                   if p.get("partId") == pid), None)
    except Exception:
        pn = None
    label = ((pn or "").strip() or name.split("<")[0].strip()).upper()

    if mains is None:           # degenerate (not a clean plate) — fall back to flat profile
        return {"loops": part_loops(name, doc, mv, elem, pid), "pockets": [],
                "up_side": "TOP", "flip": False, "two_sided": False, "label": label,
                "thickness": thickness}

    axis = mains["axis"]; hi = mains["hi"]; lo = mains["lo"]

    # old-style side choice, preserved verbatim for the proven flat-plate path
    sides = set(p["side"] for p in pockets)
    two_sided = len(sides) > 1
    if "BOT" in sides and not two_sided:
        up_side, flip = "BOT", True
    else:
        up_side, flip = "TOP", False           # top-only, none, or two-sided (cut top first)
    up_main = hi if up_side == "TOP" else lo

    # The proven path is only valid for a flat (Z-thickness) plate whose machined
    # face is one full-footprint region. Count the faces coplanar with the machined
    # side: more than one means the recesses have split it, so walking its loops
    # would miss the openings. Either condition routes to the robust path. Proud-rib
    # parts (mains re-anchored to a rib-tip plane) ALWAYS take the robust path — the
    # tip plane's loops carry none of the pocket/margin/hole structure.
    coplanar = [f for f in planar
                if abs(dot(v3(f["surface"]["origin"]), axis)*MM - up_main["off"]) < 0.3
                and abs(dot(_outward_normal(f), axis)) > 0.9]
    if abs(axis[2]) >= 0.9 and len(coplanar) == 1 and not mains.get("proud"):
        up_face = up_main["face"]
        origin = v3(up_face["surface"]["origin"])
        u, vv = make_basis(_outward_normal(up_face))   # OUTWARD normal -> spindle-up frame
        raw = _face_loops(up_face, origin, u, vv, edge_by_id)
        # an inner loop whose area matches a detected pocket IS a pocket opening (not
        # a through cut) — tag it POCKET + depth; leave true holes as-is.
        unused = [p for p in pockets if p["side"] == up_side]
        loops = []
        for lp in raw:
            if lp["role"] == "INSIDE" and unused:
                a = _area2d(lp["pts"])
                cand = min(unused, key=lambda p: abs(p["area"] - a))
                if abs(cand["area"] - a) <= max(50.0, 0.08*cand["area"]):
                    # islands = the pocket FLOOR face's inner loops (bosses standing up in
                    # the recess) projected to the machined frame — kept so the clearing
                    # goes AROUND them instead of machining them away.
                    isl = [l["pts"] for l in _face_loops(cand["face"], origin, u, vv, edge_by_id)
                           if not l.get("isOuter") and len(l["pts"]) >= 3]
                    lp = {"role": "POCKET", "pts": lp["pts"], "circle": None,
                          "depth": cand["depth"]}
                    if isl:
                        lp["islands"] = isl
                    unused.remove(cand)
            loops.append({k: lp[k] for k in ("role", "pts", "circle", "islands") if k in lp}
                         | ({"depth": lp["depth"]} if lp.get("depth") is not None else {}))
        loops, _ = _drop_features_outside_outline(loops, name)
        return {"loops": loops, "pockets": pockets, "up_side": up_side,
                "flip": flip, "two_sided": two_sided, "label": label,
                "thickness": thickness}

    # ---------- robust path: non-Z thickness and/or fragmented machined face ----------
    # machine the side that the most pocket area opens through (else larger footprint)
    area_hi = sum(p["area"] for p in pockets if p["opening"] is hi)
    area_lo = sum(p["area"] for p in pockets if p["opening"] is lo)
    if area_hi or area_lo:
        up_main = hi if area_hi >= area_lo else lo
    else:
        up_main = max((hi, lo), key=lambda m: m["area"])
    up_side = "TOP" if up_main is hi else "BOT"
    two_sided = bool(area_hi) and bool(area_lo)
    flip = up_main is lo

    origin = up_main["o"]
    u, vv = make_basis(up_main["n"])               # OUTWARD normal of the machined face
    loops = []

    # profile: outer loop of whichever main projects to the fuller footprint (the
    # machined side may be fragmented, so its own outline can be a sub-region only)
    best = None
    for m in (up_main, lo if up_main is hi else hi):
        pts = _project_face_outer(m["face"], origin, u, vv, edge_by_id)
        if pts and len(pts) >= 3:
            a = _area2d(pts)
            if best is None or a > best[0]:
                best = (a, pts)
    if best:
        loops.append({"role": "OUTSIDE", "pts": best[1], "circle": None})

    # pockets: each floor opening through the machined face, projected to the up frame.
    # Keep the floor faces too — a hole drilled INTO a pocket (e.g. a screw clearance
    # hole through the pocket floor) opens there, not on the main plane.
    floor_faces = []
    for p in pockets:
        if p["opening"] is not up_main:
            continue
        fl = _face_loops(p["face"], origin, u, vv, edge_by_id)
        pts = next((l["pts"] for l in fl if l.get("isOuter")), None)
        if pts and len(pts) >= 3:
            # islands = non-circular inner loops of the floor (bosses standing up in the
            # recess); machine AROUND them. Circular inner loops are real holes -> left
            # for collect() to drill, not treated as keep-outs.
            isl = [l["pts"] for l in fl
                   if (not l.get("isOuter")) and l["role"] == "INSIDE" and len(l["pts"]) >= 3]
            d = {"role": "POCKET", "pts": pts, "circle": None, "depth": p["depth"]}
            if isl:
                d["islands"] = isl
            loops.append(d)
            floor_faces.append(p["face"])

    # holes + through cut-outs, read from the real loops on the MACHINED-FACE plane AND
    # the pocket floors (both are exposed once the part is set up pocket-side-up). The
    # recesses may have split the machined side into several coplanar faces, so gather
    # from all of them. Reading circular hole-MOUTHS — not cylinder faces — keeps
    # FILLETS / rounded corners out (a fillet never closes into a circle in the plane);
    # reading only the up-side planes ignores far-side features such as the cones of a
    # back-face countersink (their mouths live on the OTHER plane) while still drilling
    # the clearance hole that breaks through to this side.
    up_coplanar = [f for f in planar
                   if abs(dot(v3(f["surface"]["origin"]), axis)*MM - up_main["off"]) < 0.4
                   and abs(dot(_outward_normal(f), axis)) > 0.9]
    seen = set()

    def collect(face, with_cutouts):
        for lp in _face_loops(face, origin, u, vv, edge_by_id):
            if lp.get("isOuter"):
                continue
            a = _area2d(lp["pts"])
            if any(P["role"] == "POCKET" and abs(_area2d(P["pts"]) - a) <= max(50.0, 0.08*a)
                   for P in loops):
                continue                                   # a pocket mouth, already handled
            if lp["role"] == "DRILL":
                cx, cy, r = lp["circle"]
                key = (round(cx, 1), round(cy, 1), round(r, 1))
                if key in seen:
                    continue
                seen.add(key)
                loops.append({"role": "DRILL", "pts": lp["pts"], "circle": lp["circle"]})
            elif with_cutouts and lp["role"] == "INSIDE":   # non-circular through cut-out / slot
                loops.append({"role": "INSIDE", "pts": lp["pts"], "circle": None})

    for f in up_coplanar:
        collect(f, with_cutouts=True)
    for f in floor_faces:
        collect(f, with_cutouts=False)                      # holes through pocket floors
    # A stepped face can split a through-hole mouth into many open arcs, so no single
    # planar face contains the complete circle. Recover only proven full-thickness,
    # concave cylinders; blind back-side bores remain intentionally excluded.
    loops.extend(_through_cylinder_loops(faces, origin, u, vv, axis, hi, lo, seen))

    loops, _ = _drop_features_outside_outline(loops, name)
    return {"loops": loops, "pockets": pockets, "up_side": up_side,
            "flip": flip, "two_sided": two_sided, "label": label,
            "thickness": thickness}


def extract_part(name, doc, mv, elem, pid):
    loops = part_loops(name, doc, mv, elem, pid)
    dxf = Dxf()
    stats = {"OUTSIDE": 0, "DRILL": 0, "INSIDE": 0}
    for lp in loops:
        stats[lp["role"]] += 1
        if lp["circle"]:
            cx, cy, r = lp["circle"]
            dxf.circle("DRILL", (cx, cy), r)
        else:
            for a, b in zip(lp["pts"], lp["pts"][1:]):
                dxf.line(lp["role"], a, b)
    allx = [p[0] for lp in loops for p in lp["pts"]]
    ally = [p[1] for lp in loops for p in lp["pts"]]
    ext = (max(allx)-min(allx), max(ally)-min(ally)) if allx else (0, 0)
    return dxf, stats, ext

def safe(name):
    return "".join(c if c.isalnum() or c in " -_" else "_" for c in name).strip()


# The Onshape CLI driver (main()) was removed with the input layer.
