"""CAM + Syntec post for 2.5D sheet parts.

Takes the classified loops from extract.part_loops, offsets each contour by the
cutter radius (tool-CENTRE path, matching the proven Vectric output), builds the
multi-pass depth strategy, and emits G-code in the captured Syntec dialect
(see SYNTEC_POST_SPEC.md). One .NC per part for now (nesting comes next).
"""
import sys, math
from pathlib import Path
import pyclipper
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)
SCALE = 1000.0   # pyclipper integer scaling (micron precision)

# ---- machine / material profile ----
# MDF on a 9kW router with a Ø6mm 2-flute carbide spiral, run at 18k rpm.
# feed = rpm * flutes * chipload;  plunge = rpm * flutes * plunge_chipload
# DOC ~= 1xD (full-width slot in MDF, rigid machine) -> 18mm in 3 passes
CUTTER_DIA = 6.0
CUTTER_FLUTES = 2
SPINDLE_RPM = 24000      # optimum for Ø6mm carbide in MDF: Vc = pi*6*24000 ~= 450 m/min
CHIPLOAD = 0.146                      # mm/tooth, cutting (6mm cutter, MDF)
PLUNGE_CHIPLOAD = 0.052               # mm/tooth, plunging
STEPOVER = CUTTER_DIA * 0.5           # 50% stepover for pocket area-clearing (3mm)
R = CUTTER_DIA / 2.0                  # primary cutter radius (nesting gaps, sim)
MAT = dict(cut_feed=round(SPINDLE_RPM*CUTTER_FLUTES*CHIPLOAD, -2),        # ~7000
           plunge_feed=round(SPINDLE_RPM*CUTTER_FLUTES*PLUNGE_CHIPLOAD, -2),  # ~2500
           internal_feed_factor=0.9,   # internal contours/pockets fed slower than the profile
           inside_feed=0.0, pocket_feed=0.0,   # absolute overrides, mm/min; 0 = the factor
           max_step=6.2, final_z=-0.5, clearance_over=6.0, spindle=SPINDLE_RPM,
           drill_rpm=12000, drill_feed=600.0,   # holes plunge-drilled slower (anti-scorch)
           chamfer_rpm=12000, chamfer_feed=3000.0)   # chamfer pass slower (big bit, high rim speed)
# machine tool table: T04=Ø6mm, T07=Ø4mm. Narrow slots auto-fall back T04->T07.
TOOLS = {
    4: dict(dia=6.0, feed=MAT["cut_feed"], plunge=MAT["plunge_feed"], step=MAT["max_step"]),
    7: dict(dia=4.0, feed=round(SPINDLE_RPM*CUTTER_FLUTES*0.10, -2), plunge=1800.0, step=4.0),
}
DEFAULT_TOOL = 4
TOOL_PREF = [4, 7]                     # try Ø6 first, then Ø4 for tight features
RAMP_LEN = 30.0           # mm of ramped lead-in descent into each pass (no dead plunge)
DRILL_PECK = 4.0          # mm peck increment when plunge-drilling holes (~0.66xD, MDF chip clearing)
ARC_TOL = 0.02            # mm tolerance for fitting G02/G03 arcs to the toolpath
SMALL_PART_MAXDIM = 200.0   # parts whose largest footprint dim < this need tabs (vacuum insufficient)
MARGIN = 6.0                # offset of part from program origin (matches proven ~6mm)
# holding tabs (only added to small parts' OUTSIDE profile)
TAB_REMAIN = 2.0            # mm of material left under a tab (board bottom = z0) — easy snap
TAB_WIDTH = 8.0            # mm tab length along the contour
TAB_SPACING = 250.0        # one tab per ~this much perimeter
TAB_MIN = 3
DRILL_TOL = 0.8            # hole radius within this of cutter radius -> plunge-drill
# Circular holes at or below this diameter are area-cleared instead of profiled.
# This prevents a small loose core/slug being released into the router enclosure.
SMALL_HOLE_CLEAR_MAX_DIA = 40.0
ENGRAVE_DEPTH = 0.6        # mm: depth for the reference text height; scales DOWN for smaller text
ENGRAVE_REF_H = 25.0       # text cap-height (mm) ENGRAVE_DEPTH is tuned for (V-bit groove width ~ depth)
ENGRAVE_MIN_DEPTH = 0.15   # never engrave shallower than this (stays legible)
ENGRAVE_SAFE = 2.0         # low hop above stock between engrave strokes (not a full retract to clearance)
# Extra plunge below the NOMINAL top, on every engrave stroke whatever its depth. A
# board is never quite its nominal thickness (a "15" comes 14.6) and the tool
# length carries its own tenth or two, so a 0.15-0.6 mm groove cut to the nominal
# surface can miss the wood entirely — a 15 mm sheet came off the bed with no
# marking at all, the V-bit just above it. One millimetre covers both.
ENGRAVE_BITE = 1.0
ENGRAVE_TOOL = None        # tool number of the V-bit/engraver (set by the web config); None -> narrowest
# edge break (chamfer) — set by the web config (_apply_config)
EDGE_BREAK = False         # add a chamfer edge-break pass
EDGE_BREAK_SIZE = 2.0      # chamfer width (mm)
EDGE_BREAK_ANGLE = 90.0    # included angle of the vee/chamfer bit (deg); depth = width/tan(angle/2)
EDGE_BREAK_SCOPE = "profile"   # "profile" = outer edge only | "all" = + holes & pockets
CHAMFER_TOOL = None        # auto chamfer-kind seat (fallback); tool_map["chamfer"] overrides

# Finishing pass on the PROFILES (outer profile + internal cut-outs). The roughing
# passes are offset a further FINISH_ALLOWANCE away from the wall, leaving a skin,
# and one more lap at the true offset takes it off. The finishing lap is cutting a
# thin skin inside an already-open channel, so it is barely loaded — which is the
# point: the wall is cut by a tool that is not deflecting, in one continuous lap,
# so it carries neither the deflection error of the roughing passes nor their step
# marks. Pocket clearing and floors are untouched.
FINISH_PASS = False        # add the finishing lap
FINISH_ALLOWANCE = 0.5     # radial skin the roughing leaves on the wall (mm)
FINISH_STEPPED = False     # False = one full-depth lap; True = step it down like roughing

# 3D surfacing (SURFACE loops carry a heightfield; see wood_geometry). The ball
# nose finishes; the widest flat cutting tool roughs first. Config-by-globals
# like everything else here.
BALL_TOOL = None           # loaded ball-nose seat; tool_map["surface"] overrides
SURFACE_SCALLOP = 0.10     # mm cusp height target for the ball finish stepover
SURFACE_LEAVE = 0.5        # mm stock the roughing leaves for the finish pass
SURFACE_TERRACE_STEP = 1.0 # mm steps when a 3D job is roughed with no ball nose
SURFACE_STEEP_DEG = 35.0   # 3D jobs: faces steeper than this get a cross ball pass
SKIP_PROFILE = False       # pockets-only: machine pockets/holes/cut-outs but NEVER the
                           # outer profile (part stays attached to a pre-cut blank).
                           # The outline is still used to position the part — just not cut.
CLEAR_POCKETS = True       # 2.5D clearing: area-clear every recess so the waste leaves as
                           # chips. Off, a recess is only PROFILED at its depth and the
                           # middle stays in — quicker, but on a through pocket it frees a
                           # slug, and on a blind one it leaves a raised island of stock.


def engrave_depth_for(text_h):
    """Engrave depth proportional to text size: ENGRAVE_DEPTH at ENGRAVE_REF_H, scaled
    down for smaller lettering (so small text isn't cut too deep / wide), floored at
    ENGRAVE_MIN_DEPTH. A V-bit's groove WIDTH grows with depth, so this keeps the line
    weight proportional to the character size."""
    if not text_h or text_h <= 0:
        return ENGRAVE_DEPTH
    return max(ENGRAVE_MIN_DEPTH, min(ENGRAVE_DEPTH, ENGRAVE_DEPTH * text_h / ENGRAVE_REF_H))


def signed_area(pts):
    a = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        a += x0*y1 - x1*y0
    return a/2.0

def to_ccw(pts):
    return pts if signed_area(pts) > 0 else pts[::-1]

def offset(pts, delta):
    """Offset a closed polygon (mm) by delta mm (+ grows). Returns list of paths."""
    path = [(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in to_ccw(pts)]
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(path, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    sol = pco.Execute(delta*SCALE)
    return [[(x/SCALE, y/SCALE) for x, y in p] for p in sol]


def poly_union(polys):
    """Boolean union of closed polygons (mm) -> list of outer-boundary rings (mm).
    Used to merge overlapping pockets into one connected recess so a stepped pocket
    is area-cleared as a whole, not as separate islands. Holes are dropped (pocket
    floors don't enclose un-pocketed islands in practice)."""
    polys = [to_ccw(p) for p in polys if len(p) >= 3]
    if not polys:
        return []
    pc = pyclipper.Pyclipper()
    for p in polys:
        pc.AddPath([(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in p],
                   pyclipper.PT_SUBJECT, True)
    sol = pc.Execute(pyclipper.CT_UNION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    return [[(x/SCALE, y/SCALE) for x, y in p] for p in sol if pyclipper.Orientation(p)]


def _point_in(pt, poly):
    """True if pt (mm) is inside polygon poly (mm)."""
    p = (int(round(pt[0]*SCALE)), int(round(pt[1]*SCALE)))
    ip = [(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in poly]
    return pyclipper.PointInPolygon(p, ip) != 0


def _centroid(pts):
    return (sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts))


def _seg_dist(p, a, b):
    """Distance from point p to segment ab (mm)."""
    px, py = p; ax, ay = a; bx, by = b
    dx, dy = bx-ax, by-ay
    L2 = dx*dx + dy*dy
    if L2 <= 1e-12:
        return math.hypot(px-ax, py-ay)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy)/L2))
    return math.hypot(px-(ax+t*dx), py-(ay+t*dy))


def _swept_segment_inside(a, b, allowed, radius, area_tol=0.50):
    """Bound the local relief made by a round cutter moving from a to b.

    This is the exact 2-D swept disc, not a centreline-only check.  It is used before
    extending a narrow pocket into an already-cleared hole. Tangent CAD features can
    require a tiny dogbone-like junction relief; anything above 0.50 mm2 is rejected,
    preventing a nearby hole or bad classification from causing a gross overcut.
    """
    if not allowed or math.dist(a, b) <= 1e-9:
        return False
    pco = pyclipper.PyclipperOffset()
    pco.AddPath([(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in (a, b)],
                pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    swept = pco.Execute(radius*SCALE)
    if not swept:
        return False
    pc = pyclipper.Pyclipper()
    pc.AddPaths(swept, pyclipper.PT_SUBJECT, True)
    for poly in allowed:
        pc.AddPath([(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in poly],
                   pyclipper.PT_CLIP, True)
    outside = pc.Execute(pyclipper.CT_DIFFERENCE,
                         pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    outside_area = sum(abs(pyclipper.Area(p)) for p in outside) / (SCALE*SCALE)
    return outside_area <= area_tol


def pocket_void_overruns(path, region, cleared_voids, radius, join_tol=0.25):
    """Add safe slot-end excursions into holes that have already been area-cleared.

    Eroding a slot and a tangent hole separately leaves a crescent at their junction.
    For every touching cleared hole, extend the pocket path until the cutter is wholly
    inside the hole, then return. The exact swept disc is checked against (pocket UNION
    hole), with only a tightly capped junction relief allowed, so a nearby but
    non-opening hole can never cause a material overcut into the part.
    """
    if len(path) < 2 or not cleared_voids:
        return path
    inserts = {}
    n = len(region)
    for void in cleared_voids:
        circle = void.get("circle")
        if not circle:
            continue
        cx, cy, hole_r = circle
        if hole_r <= radius + 0.05:
            continue
        gap = min(_seg_dist((cx, cy), region[i], region[(i+1) % n]) for i in range(n)) - hole_r
        if gap > join_tol:
            continue
        k = min(range(len(path)), key=lambda i: (path[i][0]-cx)**2 + (path[i][1]-cy)**2)
        px, py = path[k]
        d = math.hypot(px-cx, py-cy)
        if d <= 1e-9:
            continue
        safe_r = max(0.0, hole_r - radius - 0.10)
        target = (cx + (px-cx)*safe_r/d, cy + (py-cy)*safe_r/d)
        allowed = poly_union([region, void["pts"]])
        if _swept_segment_inside((px, py), target, allowed, radius):
            inserts.setdefault(k, []).append(target)
    if not inserts:
        return path
    out = []
    for k, pt in enumerate(path):
        out.append(pt)
        for target in inserts.get(k, []):
            out.extend((target, pt))
    return out


def _on_polyline(p, poly, tol):
    return min(_seg_dist(p, poly[i], poly[(i+1) % len(poly)]) for i in range(len(poly))) <= tol


def extend_to_profile(region, profile, ext, tol=0.8):
    """Where a pocket-region edge lies ON the outer profile (the pocket runs out to a
    square corner against the part edge), bulge the region OUT past the profile by
    `ext` so the round cutter overruns into the waste and forms that corner square,
    instead of leaving a tool-radius fillet right at the part edge.

    SAFETY (real CAM keeps a stock model + rest-machines only the waste; this is the
    2D version): the overrun only ever goes into the WASTE side of the profile, never
    across the boundary into the keeper. For each abutting edge we step a hair off it
    both ways and keep the side that lands OUTSIDE the part; if neither side is clearly
    outside (not a real open boundary, or it abuts the keeper) the overrun is SKIPPED —
    a small fillet is better than cutting the part. Inset pockets are untouched."""
    if not profile or len(region) < 3:
        return region
    eps = max(tol + 0.5, 1.0)
    quads = []
    n = len(region)
    for i in range(n):
        a = region[i]; b = region[(i+1) % n]
        if not (_on_polyline(a, profile, tol) and _on_polyline(b, profile, tol)):
            continue
        mx, my = (a[0]+b[0])/2, (a[1]+b[1])/2
        dx, dy = b[0]-a[0], b[1]-a[1]; L = math.hypot(dx, dy)
        if L < 1e-6:
            continue
        nx, ny = -dy/L, dx/L
        out_pos = not _point_in((mx+nx*eps, my+ny*eps), profile)   # +n side is waste?
        out_neg = not _point_in((mx-nx*eps, my-ny*eps), profile)   # -n side is waste?
        if out_neg and not out_pos:
            nx, ny = -nx, -ny
        elif not (out_pos and not out_neg):
            continue                                              # ambiguous / into keeper -> skip
        quads.append([a, b, (b[0]+nx*ext, b[1]+ny*ext), (a[0]+nx*ext, a[1]+ny*ext)])
    if not quads:
        return region
    u = poly_union([region] + quads)
    return max(u, key=lambda p: abs(signed_area(p))) if u else region


def depth_passes(top, final=None, step=None):
    if final is None:
        final = MAT["final_z"]
    if step is None:
        step = MAT["max_step"]
    n = max(1, math.ceil((top - final) / step))
    dz = (top - final) / n
    return [top - dz*(i+1) for i in range(n)]


def pocket_rings(boundary, radius=R, stepover=STEPOVER):
    """Concentric area-clearing rings for a pocket: offset the boundary inward
    by radius, +stepover, ... until it vanishes. Returns rings outer->inner."""
    rings = []
    d = radius
    while True:
        sols = [s for s in offset(boundary, -d) if len(s) >= 3]
        if not sols:
            break
        rings.extend(sols)
        d += stepover
        if d > 1e4:
            break
    return rings


def pocket_island_rings(boundary, islands, radius=R, stepover=STEPOVER):
    """Concentric tool-CENTRE rings that clear a pocket while leaving `islands`
    (bosses standing up in the recess) UNTOUCHED. The tool centre is kept `radius`
    inside the boundary AND `radius` outside every island, then that channel is
    cleared by repeated inward offsets. Returns a flat list of closed loops (mm),
    outermost first, or [] if the channel is everywhere too narrow for the tool.
    Each loop is traced as its own ramped pass (see Prog.island_pocket) so the tool
    never hops across an island at depth."""
    inner = [s for s in offset(boundary, -radius) if len(s) >= 3]
    if not inner:
        return []
    keepout = []
    for isl in islands:
        keepout += [s for s in offset(isl, radius) if len(s) >= 3]

    def _i(p):
        return [(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in p]

    if keepout:
        pc = pyclipper.Pyclipper()
        for p in inner:
            pc.AddPath(_i(p), pyclipper.PT_SUBJECT, True)
        for p in keepout:
            pc.AddPath(_i(p), pyclipper.PT_CLIP, True)
        base = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    else:
        base = [_i(p) for p in inner]
    base = [p for p in base if len(p) >= 3]
    if not base:
        return []

    rings, d, guard = [], 0.0, 0
    while guard < 10000:
        guard += 1
        if d == 0.0:
            sol = base
        else:
            pco = pyclipper.PyclipperOffset()
            pco.AddPaths(base, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
            sol = pco.Execute(-d*SCALE)
        sol = [p for p in sol if len(p) >= 3]
        if not sol:
            break
        for p in sol:
            rings.append([(x/SCALE, y/SCALE) for x, y in p])
        d += stepover
    return rings


DOGBONE = True                 # auto corner overcuts so mating parts seat in slots
DOGBONE_MIN_TURN = math.radians(20)   # only relieve corners sharper than this
# A tessellated arc turns a LITTLE at every vertex, but on a fine chord tolerance a
# small radius turns more per vertex than DOGBONE_MIN_TURN — a 0.05mm chord on R3
# steps ~21° each time. A bare per-vertex turn test therefore reads every point of an
# internal fillet under ~3.3mm radius as a sharp corner and dogbones the whole fillet
# away as a string of overcuts: a modelled radius came out of the machine as a row of
# spikes. Two independent guards separate curve from corner.
#
# 1. CHORD. turn = chord/R and chord = 2*sqrt(2*R*tol), so a vertex only breaches the
#    20 degree threshold when R <~ 3.3mm (at the extractors' tol = 0.05mm, see
#    wood_geometry.CHORD_TOL_MM / step_ingest_service.CHORD_TOL_MM), and such an arc's
#    chords are <~1.15mm. A vertex whose BOTH neighbouring segments are shorter than
#    ARC_CHORD_MAX is therefore tessellation, not an edge. NB this bound follows the
#    extractor tolerance: coarsen that and this must be re-derived.
# 2. CIRCLE. For arcs sampled finely enough to have long chords, P[i±2] still lie on
#    the circle through P[i-1..i+1]; the neighbours of a true corner do not.
#
# A rectangle is the case that rules out the tempting third test ("a real corner is an
# isolated turn"): every one of its vertices turns 90 degrees, so isolation finds none.
ARC_CHORD_MAX = 1.5
ARC_FIT_TOL = 0.15


def _on_local_arc(P, i, tol=ARC_FIT_TOL):
    """True if P[i] is a point along a tessellated curve rather than a real corner."""
    n = len(P)
    if n < 5:
        return False                                   # too few points to tell
    a, c, b = P[(i-1) % n], P[i], P[(i+1) % n]
    (ax, ay), (bx, by), (cx, cy) = a, c, b
    d = 2.0 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    if abs(d) < 1e-12:
        return False                                   # collinear — no corner here anyway
    sa, sb, sc_ = ax*ax+ay*ay, bx*bx+by*by, cx*cx+cy*cy
    ux = (sa*(by-cy) + sb*(cy-ay) + sc_*(ay-by)) / d
    uy = (sa*(cx-bx) + sb*(ax-cx) + sc_*(bx-ax)) / d
    r = math.hypot(ax-ux, ay-uy)
    for j in (i-2, i+2):
        q = P[j % n]
        if abs(math.hypot(q[0]-ux, q[1]-uy) - r) > tol:
            return False
    return True


def relief_apexes(pts, role, min_turn=DOGBONE_MIN_TURN):
    """Internal (concave-to-the-cut) sharp corners that the round cutter would
    leave a fillet on. Returns the true corner apex points (tool-centre dips
    here = a radius-R dogbone)."""
    p = []
    for q in pts:                                  # drop consecutive duplicates
        if not p or abs(q[0]-p[-1][0]) > 1e-6 or abs(q[1]-p[-1][1]) > 1e-6:
            p.append(q)
    if len(p) > 1 and abs(p[0][0]-p[-1][0]) < 1e-6 and abs(p[0][1]-p[-1][1]) < 1e-6:
        p.pop()
    n = len(p)
    if n < 3:
        return []
    area = sum(p[i][0]*p[(i+1) % n][1] - p[(i+1) % n][0]*p[i][1] for i in range(n))
    P = p if area > 0 else p[::-1]                  # CCW: interior on the left
    out = []
    for i in range(n):
        a, c, b = P[(i-1) % n], P[i], P[(i+1) % n]
        d1 = (c[0]-a[0], c[1]-a[1]); d2 = (b[0]-c[0], b[1]-c[1])
        l1 = math.hypot(*d1); l2 = math.hypot(*d2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        d1 = (d1[0]/l1, d1[1]/l1); d2 = (d2[0]/l2, d2[1]/l2)
        cross = d1[0]*d2[1] - d1[1]*d2[0]
        turn = abs(math.atan2(cross, d1[0]*d2[0]+d1[1]*d2[1]))
        if turn < min_turn:
            continue
        # a point along a modelled radius, not a corner to relieve
        if l1 < ARC_CHORD_MAX and l2 < ARC_CHORD_MAX:
            continue
        if _on_local_arc(P, i):
            continue
        # cut region: interior for INSIDE/POCKET (convex=left turn), exterior for
        # OUTSIDE (reflex=right turn) -> those corners get filleted -> relieve them.
        if role in ("INSIDE", "POCKET") and cross > 0:
            out.append(c)
        elif role == "OUTSIDE" and cross < 0:
            out.append(c)
    return out


def add_dogbones(op, apexes):
    """Insert a dip to each corner apex (and back) into the offset toolpath."""
    if not apexes:
        return op
    ins = {}
    for A in apexes:
        bi = min(range(len(op)), key=lambda k: (op[k][0]-A[0])**2 + (op[k][1]-A[1])**2)
        ins.setdefault(bi, []).append(A)
    res = []
    for k, pt in enumerate(op):
        res.append(pt)
        for A in ins.get(k, []):
            res.append(A); res.append(pt)          # dip to true corner, return
    return res


def slot_centreline(pts, tool_r, width_tol=0.25):
    """Tool-centre pass for a groove the SAME width as the cutter, or None.

    Jigs are routinely designed with grooves exactly one cutter wide (a 4mm
    slat groove for a Ø4 bit) — un-ringable by area clearing, but cut every
    day as a single centreline pass. Accepted only when the region is a
    straight slot (principal-axis rectangle check) whose width matches the
    tool within `width_tol`: a wider tool would oversize the groove and a
    narrower one undersize it, and both are wrong parts, not fallbacks."""
    n = len(pts)
    if n < 4:
        return None
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - cx) ** 2 for p in pts)
    syy = sum((p[1] - cy) ** 2 for p in pts)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in pts)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    axv = (math.cos(ang), math.sin(ang))
    ts = [(p[0] - cx) * axv[0] + (p[1] - cy) * axv[1] for p in pts]
    ws = [-(p[0] - cx) * axv[1] + (p[1] - cy) * axv[0] for p in pts]
    t0, t1 = min(ts), max(ts)
    width = max(ws) - min(ws)
    length = t1 - t0
    if length <= width:                       # not a slot
        return None
    if abs(width - 2 * tool_r) > width_tol:
        return None
    # a straight slot fills its oriented box (round ends cost a few percent);
    # an L or a curve does not — refuse those rather than cut a wrong line
    if abs(signed_area(pts)) < 0.75 * width * length:
        return None
    wmid = (max(ws) + min(ws)) / 2
    a_t, b_t = t0 + tool_r, t1 - tool_r
    if b_t <= a_t:
        a_t = b_t = (t0 + t1) / 2             # slot barely longer than the tool
    def at(t):
        return (cx + axv[0] * t - axv[1] * wmid, cy + axv[1] * t + axv[0] * wmid)
    return [at(a_t), at(b_t)]


def spiral_from_rings(rings):
    """Link concentric rings (outer->inner from pocket_rings) into ONE continuous
    inner->outer path: start at the innermost ring, walk it, hop to the nearest
    point of the next ring out, and so on. Single entry, continuous motion;
    finishes on the outer wall."""
    spiral = []
    for ring in reversed(rings):                    # inner -> outer
        if not spiral:
            spiral.extend(ring); spiral.append(ring[0])      # close innermost
            continue
        last = spiral[-1]
        bi = min(range(len(ring)), key=lambda k: (ring[k][0]-last[0])**2 + (ring[k][1]-last[1])**2)
        rot = ring[bi:] + ring[:bi]
        spiral.extend(rot); spiral.append(rot[0])            # close each ring
    return spiral


def _circle3(a, b, c):
    """Circle through 3 points -> (cx, cy, r) or None if ~collinear."""
    ax, ay = a; bx, by = b; cx, cy = c
    d = 2*(ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
    if abs(d) < 1e-9:
        return None
    a2 = ax*ax+ay*ay; b2 = bx*bx+by*by; c2 = cx*cx+cy*cy
    ux = (a2*(by-cy) + b2*(cy-ay) + c2*(ay-by)) / d
    uy = (a2*(cx-bx) + b2*(ax-cx) + c2*(bx-ax)) / d
    return (ux, uy, math.hypot(ax-ux, ay-uy))


def fit_arc(pts, i, tol=ARC_TOL, max_r=2000.0, max_chord=4.0):
    """Greedily fit the longest circular arc to pts starting at index i.
    Returns (j, cx, cy, signed_sweep) or None (then caller emits a line).
    Refuses to span long chords so straight edges (whose vertices may happen to
    be concyclic, e.g. a rectangle's corners) are never mistaken for arcs."""
    n = len(pts)
    if i+2 >= n or math.dist(pts[i], pts[i+1]) > max_chord:
        return None
    best = None
    j = i+2
    while j < n:
        if math.dist(pts[j-1], pts[j]) > max_chord:    # long chord -> not a sampled arc
            break
        circ = _circle3(pts[i], pts[(i+j)//2], pts[j])
        if not circ:
            break
        cx, cy, r = circ
        if r > max_r or r < 1e-6:
            break
        # all points within tol of the circle?
        if any(abs(math.hypot(pts[k][0]-cx, pts[k][1]-cy) - r) > tol for k in range(i, j+1)):
            break
        # monotonic sweep, no reversal, not a near-full circle
        ang = [math.atan2(pts[k][1]-cy, pts[k][0]-cx) for k in range(i, j+1)]
        sweep = 0.0; sign = 0; ok = True
        for t in range(1, len(ang)):
            da = ang[t]-ang[t-1]
            while da > math.pi: da -= 2*math.pi
            while da < -math.pi: da += 2*math.pi
            if abs(da) < 1e-9:
                continue
            s = 1 if da > 0 else -1
            if sign == 0: sign = s
            elif s != sign:
                ok = False; break
            sweep += da
        if not ok or abs(sweep) > math.radians(345):
            break
        best = (j, cx, cy, sweep)
        j += 1
    return best


class Prog:
    """Syntec G-code emitter with N+10 numbering."""
    def __init__(self):
        self.lines, self.n = [], 0
        self.cut_feed = MAT["cut_feed"]
        self.plunge_feed = MAT["plunge_feed"]
    def add(self, s):
        self.lines.append(f"N{self.n}{s}")
        self.n += 10
    def _set_tool(self, tool):
        self.cut_feed = TOOLS[tool]["feed"]
        self.plunge_feed = TOOLS[tool]["plunge"]
    def header(self, tool=DEFAULT_TOOL):
        self._set_tool(tool)
        for s in ("G40G17G80G49", "G00 G53 Z0.0", f"T{tool}M06", "G90G54",
                  "G90G71", f"G43 H{tool}", f"S{MAT['spindle']}M3"):
            self.add(s)
    def toolchange(self, tool, clear):
        """Mid-program tool change: retract, swap tool, re-enable comp + spindle."""
        self._set_tool(tool)
        self.add(f"G00Z{clear:.3f}")
        self.add("G00 G53 Z0.0")
        self.add(f"T{tool}M06")
        self.add(f"G43 H{tool}")
        self.add(f"S{MAT['spindle']}M3")
    def spindle(self, rpm):
        """Change spindle speed mid-program (S..M3). Used to drop the rpm for
        plunge-drilling holes, then restore it. The rapid moves to/from the hole
        give the spindle time to settle, so no explicit dwell is needed."""
        self.add(f"S{int(round(rpm))}M3")
    def footer(self, clear):
        self.add(f"G00Z{clear:.3f}")
        self.add("G28G91Z0"); self.add("M5"); self.add("M30")

    def partcut(self, line, clear, depths, top):
        """Straight through-cut that parts the nested strip off from the scrap.
        Flat zig-zag passes (plunge at each turn) so the whole line cuts through."""
        if len(line) < 2:
            return
        x0, y0 = line[0]
        self.add(f"G00X{x0:.3f}Y{y0:.3f}")
        self.add(f"G00Z{clear:.3f}")
        self.add(f"G00Z{top+1.0:.3f}")
        fwd = True
        for z in depths:
            seq = line if fwd else line[::-1]
            self.add(f"G00X{seq[0][0]:.3f}Y{seq[0][1]:.3f}")
            self.add(f"G01Z{z:.3f}F{self.plunge_feed:.1f}")
            for x, y in seq[1:]:
                self.add(f"G01X{x:.3f}Y{y:.3f}F{self.cut_feed:.1f}")
            fwd = not fwd
        self.add(f"G00Z{clear:.3f}")

    def engrave(self, stroke, clear, top, depth=ENGRAVE_DEPTH):
        """Trace one open stroke at a shallow fixed depth (part-number marking).
        Plunge in, follow the polyline, retract — single pass."""
        if len(stroke) < 2:
            return
        z = top - depth - ENGRAVE_BITE
        x0, y0 = stroke[0]
        self.add(f"G00X{x0:.3f}Y{y0:.3f}")
        self.add(f"G00Z{top+1.0:.3f}")
        self.add(f"G01Z{z:.3f}F{self.plunge_feed:.1f}")
        first = True
        for x, y in stroke[1:]:
            f = f"F{self.cut_feed:.1f}" if first else ""
            self.add(f"G01X{x:.3f}Y{y:.3f}{f}"); first = False
        self.add(f"G00Z{clear:.3f}")
    def engrave_group(self, strokes, clear, top):
        """Engrave a whole label's strokes efficiently. `strokes` = [(polyline, depth)].
        Nearest-neighbour ordering (each stroke may be traced either way), and between
        strokes the tool hops to a LOW safe-Z (top+ENGRAVE_SAFE) instead of retracting
        all the way to the clearance plane — far fewer/shorter lift-ups than one
        plunge-trace-fullretract per stroke. One rapid-in at the start, one full retract
        at the end (the next op / footer always re-retracts to clearance first)."""
        items = [(s, d) for s, d in strokes if len(s) >= 2]
        if not items:
            return
        safe = top + ENGRAVE_SAFE
        rem = list(range(len(items))); order = []
        px, py = items[0][0][0]
        while rem:                                   # greedy nearest-neighbour, allow reversal
            bi, brev, bd = None, False, None
            for i in rem:
                s = items[i][0]
                ds = (s[0][0]-px)**2 + (s[0][1]-py)**2
                de = (s[-1][0]-px)**2 + (s[-1][1]-py)**2
                d = min(ds, de)
                if bd is None or d < bd:
                    bd, bi, brev = d, i, de < ds
            order.append((bi, brev)); rem.remove(bi)
            s = items[bi][0]; px, py = (s[0] if brev else s[-1])
        self.add(f"G00Z{clear:.3f}")
        for i, rev in order:
            s, depth = items[i]
            path = s[::-1] if rev else s
            z = top - depth - ENGRAVE_BITE; sx, sy = path[0]
            self.add(f"G00X{sx:.3f}Y{sy:.3f}")
            self.add(f"G00Z{safe:.3f}")
            self.add(f"G01Z{z:.3f}F{self.plunge_feed:.1f}")
            first = True
            for x, y in path[1:]:
                f = f"F{self.cut_feed:.1f}" if first else ""
                self.add(f"G01X{x:.3f}Y{y:.3f}{f}"); first = False
            self.add(f"G00Z{safe:.3f}")              # low hop, not full clearance
        self.add(f"G00Z{clear:.3f}")                 # single full retract at the end
    def profile(self, path, clear, depths, top, tab_z=None, n_tabs=0,
                tab_w=TAB_WIDTH, ramp_len=RAMP_LEN):
        """Trace a closed contour through all depth passes with a RAMPED lead-in
        and reliable holding tabs. Tab window boundaries are inserted as explicit
        points so a full-width tab forms regardless of vertex spacing; Z is
        clamped up to tab_z inside each window. A final-pass overlap re-cuts the
        ramped start so the edge still goes fully through."""
        p = path if (path[0] == path[-1]) else path + [path[0]]
        cum = [0.0]
        for a, b in zip(p, p[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        L = cum[-1]
        if L <= 0:
            return
        rl = min(ramp_len, 0.4*L)
        tabs = tab_z is not None and n_tabs > 0
        centers = [L*(k+0.5)/n_tabs for k in range(n_tabs)] if tabs else []
        half = tab_w/2

        def in_tab(s):
            for c in centers:
                d = abs((s % L)-c); d = min(d, L-d)
                if d <= half + 1e-6:
                    return True
            return False
        def pt_at(s):
            s = max(0.0, min(L, s))
            for i in range(1, len(p)):
                if cum[i] >= s:
                    seg = cum[i]-cum[i-1]; t = 0 if seg <= 0 else (s-cum[i-1])/seg
                    return (p[i-1][0]+(p[i][0]-p[i-1][0])*t, p[i-1][1]+(p[i][1]-p[i-1][1])*t)
            return p[-1]

        # arc-length samples = original vertices + tab window boundaries + ramp end
        svals = set(cum); svals.add(rl)
        for c in centers:
            svals.add((c-half) % L); svals.add((c+half) % L)
        aug = [(s, pt_at(s), in_tab(s)) for s in sorted(svals) if 0 <= s <= L]
        n = len(aug)

        def zbase_at(z, zprev, s):
            return z if (rl <= 0 or s >= rl) else zprev + (z-zprev)*(s/rl)

        def gen_loop(z, zprev, smax=None):
            """One lap as (x,y,z) targets; square tabs (vertical lift/hold/drop) when z<tab_z."""
            out = []; tabpass = tabs and z < tab_z
            for i in range(n):
                s, xy, it = aug[i]
                if s <= 1e-9 or (smax is not None and s > smax + 1e-9):
                    continue
                base = zbase_at(z, zprev, s)
                if tabpass and it:
                    prev_it = aug[i-1][2]; nxt_it = aug[i+1][2] if i+1 < n else False
                    if not prev_it: out.append((xy[0], xy[1], base))
                    out.append((xy[0], xy[1], tab_z))
                    if not nxt_it: out.append((xy[0], xy[1], base))
                else:
                    out.append((xy[0], xy[1], base))
            return out

        # build the full ordered cutting path (start point + every pass + final overlap)
        x0, y0 = p[0]
        seq = [(x0, y0, top + 1.0)]
        last = len(depths) - 1
        for di, z in enumerate(depths):
            zprev = top + 1.0 if di == 0 else depths[di-1]
            seq += gen_loop(z, zprev)
            if di == last and rl > 0:
                seq += gen_loop(z, z, smax=rl)        # clear the ramped start at full depth

        self.add(f"G00X{x0:.3f}Y{y0:.3f}")
        self.add(f"G00Z{clear:.3f}")
        self.add(f"G00Z{top+1.0:.3f}")
        self._emit_cuts(seq)
        self.add(f"G00Z{clear:.3f}")

    def _emit_cuts(self, seq):
        """Emit a cutting path, fitting G02/G03 arcs over constant-Z runs and
        emitting Z-changing moves (ramp / tab lift) as G01 lines with Z."""
        n = len(seq); first = True; i = 1
        while i < n:
            zc = seq[i][2]
            if abs(zc - seq[i-1][2]) <= 1e-6:            # constant-Z run -> arc-fit
                j = i
                while j+1 < n and abs(seq[j+1][2]-zc) <= 1e-6:
                    j += 1
                first = self._emit_run(seq, i-1, j, first)
                i = j + 1
            else:                                        # Z changes -> line carrying Z
                x, y, z = seq[i]
                px, py, pz = seq[i-1]
                if MAT.get("limit_descent") and z < pz - 1e-9:
                    # Metal: never go DOWN faster than the plunge feed. A ramp that
                    # steepens with depth (each spiral level ramps from above the
                    # stock) or a vertical drop after a tab ran at the full cutting
                    # feed — a plunge 2-3x the plunge rate into aluminium. The feed
                    # along the move is cut so its vertical part is the plunge feed;
                    # the next cutting move restates the normal feed.
                    L = math.sqrt((x-px)**2 + (y-py)**2 + (z-pz)**2)
                    vf = min(self.cut_feed, self.plunge_feed * L / (pz - z))
                    self.add(f"G01X{x:.3f}Y{y:.3f}Z{z:.3f}F{vf:.1f}")
                    first = True; i += 1
                    continue
                f = f"F{self.cut_feed:.1f}" if first else ""
                self.add(f"G01X{x:.3f}Y{y:.3f}Z{z:.3f}{f}")
                first = False; i += 1

    def _emit_run(self, seq, a, b, first):
        """Arc-fit the constant-Z polyline seq[a..b] (tool already at seq[a]).
        Z is modal (set by the preceding move), so run moves omit Z."""
        cf = self.cut_feed
        pts = [(seq[k][0], seq[k][1]) for k in range(a, b+1)]
        m = len(pts); i = 0
        while i < m-1:
            ff = f"F{cf:.1f}" if first else ""
            arc = fit_arc(pts, i)
            if arc:
                j, cx, cy, sw = arc
                g = "G03" if sw > 0 else "G02"
                ex, ey = pts[j]; I = cx-pts[i][0]; J = cy-pts[i][1]
                self.add(f"{g}X{ex:.3f}Y{ey:.3f}I{I:.3f}J{J:.3f}{ff}")
                i = j
            else:
                ex, ey = pts[i+1]
                self.add(f"G01X{ex:.3f}Y{ey:.3f}{ff}")
                i += 1
            first = False
        return first
    def peck(self, center, clear, depths, top):
        """Plunge-drill a hole with G83-style pecking (no canned cycle):
        rapid to just above the last cut, feed one DRILL_PECK increment, full
        retract to clear chips, repeat down to final depth."""
        x, y = center
        final = depths[-1]
        self.add(f"G00X{x:.3f}Y{y:.3f}")
        self.add(f"G00Z{clear:.3f}")
        last_top = top + 1.0          # safe Z just above the uncut/previous depth
        z = top
        peck = float(MAT.get("drill_peck") or DRILL_PECK)    # per material (metal pecks shallower)
        while z > final + 1e-6:
            z = max(z - peck, final)
            self.add(f"G00Z{last_top:.3f}")                       # rapid down near previous cut
            self.add(f"G01Z{z:.3f}F{self.plunge_feed:.1f}")       # feed one peck
            self.add(f"G00Z{clear:.3f}")                          # full retract -> clear chips
            last_top = z + 1.0
    def spiral_pocket(self, pts, clear, depths, top, ramp_len=8.0):
        """Continuous-spiral pocket clearing: one ramped entry per depth level,
        then trace the whole spiral (inner->outer) without retracting between
        rings. Arc-fit handles the curved rings."""
        if len(pts) < 2:
            return
        sx, sy = pts[0]
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        rl = min(ramp_len, 0.5*cum[-1]) if cum[-1] > 0 else 0.0
        for z in depths:
            seq = [(sx, sy, top + 1.0)]
            for k in range(1, len(pts)):
                s = cum[k]
                zz = z if (rl <= 0 or s >= rl) else (top+1.0) + (z-(top+1.0))*(s/rl)
                seq.append((pts[k][0], pts[k][1], zz))
            # Re-cut the ramped lead-in at full pass depth (as pocket_batch and
            # island_pocket already do). Without it the first ramp_len mm of the
            # path never reaches this pass's depth — on a small hole that span is
            # most of the wall circle, so an angular wedge was left standing from
            # the surface down and holes came out visibly out of round. The point
            # PAST the ramp end is included so the re-cut overlaps the already-
            # full-depth path rather than stopping a fraction of a chord short.
            if rl > 0:
                for k in range(1, len(pts)):
                    seq.append((pts[k][0], pts[k][1], z))
                    if cum[k] > rl + 1e-9:
                        break
            self.add(f"G00X{sx:.3f}Y{sy:.3f}")
            self.add(f"G00Z{clear:.3f}")
            self.add(f"G00Z{top+1.0:.3f}")
            self._emit_cuts(seq)
            self.add(f"G00Z{clear:.3f}")
    def pocket_batch(self, paths, clear, depths, layer_top, stock_top, ramp_len=8.0):
        """Machine a family of pockets level-by-level as one CAM operation.

        Paths are ordered by nearest next entry at every level.  XY repositioning is
        always above the original stock (never through an uncleared neighbouring
        pocket); entry descends only to the already-cleared top of this depth band.
        """
        paths = [p for p in paths if len(p) >= 2]
        if not paths:
            return
        travel_z = stock_top + 2.0
        self.add(f"G00Z{clear:.3f}")
        for z in depths:
            remaining = list(paths)
            current = None
            while remaining:
                if current is None:
                    pick = 0
                else:
                    pick = min(range(len(remaining)), key=lambda i:
                               (remaining[i][0][0]-current[0])**2 +
                               (remaining[i][0][1]-current[1])**2)
                pts = remaining.pop(pick)
                sx, sy = pts[0]
                cum = [0.0]
                for a, b in zip(pts, pts[1:]):
                    cum.append(cum[-1] + math.dist(a, b))
                if cum[-1] <= 0:
                    continue
                rl = min(ramp_len, 0.5*cum[-1])
                entry_z = layer_top + 1.0
                seq = [(sx, sy, entry_z)]
                for k in range(1, len(pts)):
                    s = cum[k]
                    zz = z if (rl <= 0 or s >= rl) else entry_z + (z-entry_z)*(s/rl)
                    seq.append((pts[k][0], pts[k][1], zz))
                if rl > 0:                     # recut the ramped lead-in at full depth,
                    for k in range(1, len(pts)):     # overlapping one point past it
                        seq.append((pts[k][0], pts[k][1], z))
                        if cum[k] > rl + 1e-9:
                            break
                self.add(f"G00Z{travel_z:.3f}")
                self.add(f"G00X{sx:.3f}Y{sy:.3f}")
                if entry_z < stock_top - 1e-6:
                    # A deeper band's entry sits below the stock top. The band
                    # above IS cleared, but the gate only accepts a rapid below
                    # stock into a column it has seen cut — so descend by rapid
                    # to just above the stock and FEED the rest. Air time at
                    # plunge feed, in exchange for a provably safe entry.
                    self.add(f"G00Z{stock_top + 1.0:.3f}")
                    self.add(f"G01Z{entry_z:.3f}F{self.plunge_feed:.1f}")
                else:
                    self.add(f"G00Z{entry_z:.3f}")
                self._emit_cuts(seq)
                current = (pts[-1][0], pts[-1][1])
            self.add(f"G00Z{travel_z:.3f}")
        self.add(f"G00Z{clear:.3f}")
    def island_pocket(self, loops, clear, depths, top, ramp_len=8.0):
        """Clear a pocket that has islands: at every depth, trace each closed ring as
        its OWN ramped pass, hopping between rings at a low safe-Z above the stock
        (never a cutting move across an island). Level-by-level (all rings at one depth
        before stepping down) keeps the cut shallow and the bosses fully supported."""
        loops = [(lp if lp[0] == lp[-1] else lp + [lp[0]]) for lp in loops if len(lp) >= 2]
        if not loops:
            return
        safe = top + 2.0
        self.add(f"G00Z{clear:.3f}")
        for z in depths:
            for pts in loops:
                cum = [0.0]
                for a, b in zip(pts, pts[1:]):
                    cum.append(cum[-1] + math.dist(a, b))
                total = cum[-1]
                if total <= 0:
                    continue
                rl = min(ramp_len, 0.5*total)
                sx, sy = pts[0]
                seq = [(sx, sy, safe)]
                for k in range(1, len(pts)):
                    s = cum[k]
                    zz = z if (rl <= 0 or s >= rl) else safe + (z-safe)*(s/rl)
                    seq.append((pts[k][0], pts[k][1], zz))
                if rl > 0:                     # recut the ramped lead-in at full depth,
                    for k in range(1, len(pts)):     # overlapping one point past it
                        seq.append((pts[k][0], pts[k][1], z))
                        if cum[k] > rl + 1e-9:
                            break
                self.add(f"G00X{sx:.3f}Y{sy:.3f}")
                self.add(f"G00Z{safe:.3f}")
                self._emit_cuts(seq)
                self.add(f"G00Z{safe:.3f}")                  # low hop, not a full retract
        self.add(f"G00Z{clear:.3f}")
    def surface(self, paths, clear, top):
        """3D raster passes ((x,y,z) polylines from the drop-cutter field):
        rapid above the stock, over, feed down to the lane start at plunge
        feed, then straight 3-axis feed moves. No arc fitting — Z varies."""
        self.add(f"G00Z{clear:.3f}")
        for p in paths:
            if len(p) < 2:
                continue
            x0, y0, z0 = p[0]
            self.add(f"G00Z{top + 1.0:.3f}")     # hop just above the stock
            self.add(f"G00X{x0:.3f}Y{y0:.3f}")
            self.add(f"G01Z{z0:.3f}F{self.plunge_feed:.1f}")
            first = True
            for x, y, z in p[1:]:
                f = f"F{self.cut_feed:.1f}" if first else ""
                self.add(f"G01X{x:.3f}Y{y:.3f}Z{z:.3f}{f}")
                first = False
        self.add(f"G00Z{clear:.3f}")

    def dumps(self):
        return "\n".join(self.lines) + "\n"


# ---- corner reliefs -------------------------------------------------------------
# The profile tool used to be the widest one loaded, full stop, and an outline is
# offset by its radius whatever is on it — so a corner relief modelled at R2 was
# simply swallowed by a Ø6 offset and never appeared in the toolpath: the part came
# off the machine with R3 fillets where the reliefs were drawn. The dogbone code
# cannot help; it relieves SHARP corners only and deliberately steps over modelled
# radii.
#
# The answer is the same one the cut-outs already use: the widest loaded tool THAT
# FITS. For an outline "fits" means it reproduces every modelled radius — what the
# tool leaves uncut (the closing of the outline by its radius, minus the outline) is
# below noise. A part with R2 reliefs is then profiled by the Ø4 in one toolpath,
# layer by layer, corners included, instead of the Ø6 doing the profile and the Ø4
# coming back to drill out every corner separately (tried: it worked, and it took
# far longer than the profile itself). A relief no loaded tool can make is reported
# with the fillet the part will get, not silently rounded.
RELIEF_MIN_AREA = 0.3    # mm² of unreachable material per contour that counts
# A relief drawn for the Ø4 is drawn Ø4: 4.000 wide with an R2 end. Offsetting that
# outline by exactly 2.000 collapses the notch to nothing — the offset engine drops
# a zero-width spike — so the Ø4 "could not reach" a feature made for it, and the
# fit test handed the profile back to the Ø6 and its R3 fillet. Ten microns off the
# radius keeps a hair of width in the notch and the tool centre runs down its middle,
# which is what a cutter does in a slot its own size. It is only used on a contour
# that NEEDS it; every other contour is offset by the exact radius as before.
RELIEF_EPS = 0.01
# The outline reaches the CAM as points: every modelled arc is sampled to a 0.05 mm
# chord sag (wood_geometry.CHORD_TOL_MM). A cutter smaller than an arc cannot quite
# reach the sampled polygon's corners, so every sample point of every relief leaves
# a sliver of "uncut" material a few hundredths thick. Harmless one by one, but the
# fit test SUMMED them: a 980 mm rail with 56 R3.2 reliefs came to 7 mm2 of slivers
# with the Ø6 and 3 mm2 with the Ø4, both over RELIEF_MIN_AREA, so reliefs drawn
# with room to spare for either cutter were reported "tighter than any loaded tool"
# (2026-09-29, 'test frame wood'). Short parts with few reliefs scraped under the
# threshold, which is why only the long ones complained. The leftover is now OPENED
# by this much before it is measured: anything thinner than twice the sampling
# tolerance is the sampling, not the model. A relief genuinely too tight leaves a
# crescent tenths of a millimetre thick or more, which survives the opening intact.
RELIEF_SLIVER = 0.05


def _to_int(rings):
    return [[(int(round(x * SCALE)), int(round(y * SCALE))) for x, y in r] for r in rings if len(r) >= 3]


def _from_int(paths):
    return [[(x / SCALE, y / SCALE) for x, y in p] for p in paths]


def _offset_rings(rings, delta):
    """Offset a SET of rings together (mm) — rings that merge or split are handled."""
    rings = [to_ccw(r) for r in rings if len(r) >= 3]
    if not rings:
        return []
    pco = pyclipper.PyclipperOffset()
    pco.AddPaths(_to_int(rings), pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
    return _from_int(pco.Execute(delta * SCALE))


def _rings_diff(a, b):
    """Region a minus region b, both as lists of rings (mm) -> outer rings (mm)."""
    a = [to_ccw(r) for r in a if len(r) >= 3]
    b = [to_ccw(r) for r in b if len(r) >= 3]
    if not a:
        return []
    if not b:
        return a
    pc = pyclipper.Pyclipper()
    pc.AddPaths(_to_int(a), pyclipper.PT_SUBJECT, True)
    pc.AddPaths(_to_int(b), pyclipper.PT_CLIP, True)
    sol = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    return _from_int([p for p in sol if pyclipper.Orientation(p)])


def _rings_inter(a, b):
    """Region a intersected with region b, both as lists of rings (mm)."""
    a = [to_ccw(r) for r in a if len(r) >= 3]
    b = [to_ccw(r) for r in b if len(r) >= 3]
    if not a or not b:
        return []
    pc = pyclipper.Pyclipper()
    pc.AddPaths(_to_int(a), pyclipper.PT_SUBJECT, True)
    pc.AddPaths(_to_int(b), pyclipper.PT_CLIP, True)
    sol = pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    return _from_int([p for p in sol if pyclipper.Orientation(p)])


def grow_into_waste(region, waste, ext):
    """A pocket region widened ONLY into adjoining waste — the footprint of holes
    and cut-outs that are removed anyway — by up to `ext`, or None if nothing abuts.

    A groove interrupted by a through-hole and a cut-out leaves a stub of groove
    floor between them: on a mesh jig, 1.17 mm of a 4.5 mm groove, eight times.
    Too narrow for any cutter on its own, so it was skipped — and the bar lying in
    that groove would then sit on a 1.17 mm wall standing 6 mm proud. Grown into
    the hole on one side and the cut-out on the other it is a region the cutter
    fits, and the extra it sweeps is material the hole and cut-out remove regardless.
    The growth never reaches solid part: only the intersection with the waste
    footprint is added, so the swept disc stays inside pocket ∪ waste."""
    if not waste or len(region) < 3:
        return None
    halo = _offset_rings([region], ext)
    gained = _rings_inter(halo, waste)
    if _rings_area(gained) < 1e-3:
        return None
    # A tessellated hole touches the stub along a chord, not a shared edge, so a
    # plain union leaves a hairline gap and two pieces; closing by 50 microns
    # heals it. Anything still in more than one piece does not really abut.
    grown = _offset_rings(_offset_rings([region] + gained, 0.05), -0.05)
    return grown[0] if len(grown) == 1 else None


def _rings_area(rings):
    return sum(abs(signed_area(r)) for r in rings if len(r) >= 3)


def _leftover(pts, r, role, apexes=()):
    """What a radius-r cutter following this contour leaves uncut, as rings (mm).

    OUTSIDE: the material that stays is the closing of the part by r; the leftover
    is that minus the part. INSIDE: the hole actually cut is the opening of the
    hole by r; the leftover is the modelled hole minus that. Sharp corners are the
    dogbone's business (or the operator turned dogbones off on purpose), so a disc
    of r around each sharp apex is excluded — only MODELLED radii count here.
    """
    if role == "OUTSIDE":
        got = _offset_rings(_offset_rings([pts], +r), -r)
        left = _rings_diff(got, [pts])
    else:
        got = _offset_rings(_offset_rings([pts], -r), +r)
        left = _rings_diff([pts], got)
    if apexes and left:
        discs = [[(ax + 1.05 * r * math.cos(2 * math.pi * k / 48),
                   ay + 1.05 * r * math.sin(2 * math.pi * k / 48)) for k in range(48)]
                 for ax, ay in apexes]
        left = _rings_diff(left, discs)
    return left


def fit_radius(pts, role, dia):
    """The offset radius at which a cutter of `dia` following this contour leaves
    every modelled radius as drawn, or None if it cannot. Sharp corners are
    excepted — they are the dogbone's business. The exact radius is tried first;
    a hair less (RELIEF_EPS) only rescues a feature drawn exactly one cutter
    wide, which the exact offset collapses."""
    apexes = relief_apexes(pts, role)
    for r in (dia / 2.0, dia / 2.0 - RELIEF_EPS):
        if _rings_area(_real_leftover(_leftover(pts, r, role, apexes))) < RELIEF_MIN_AREA:
            return r
    return None


def _real_leftover(left):
    """The leftover with sampling slivers removed (see RELIEF_SLIVER): an opening —
    shrink by RELIEF_SLIVER, grow back — so only material thicker than twice that
    remains, at its true size."""
    if not left:
        return left
    return _offset_rings(_offset_rings(left, -RELIEF_SLIVER), +RELIEF_SLIVER)


def reproduces_outline(pts, role, dia):
    return fit_radius(pts, role, dia) is not None


def feature_limits(loops):
    """Largest tool diameter (mm) that fits each feature type across all instances —
    drives the per-feature 'does this tool fit?' badge. None = feature absent."""
    def maxfit(pts, ring):
        ok = (lambda d: bool(pocket_rings(pts, d/2))) if ring else (lambda d: bool(offset(pts, -d/2)))
        if not ok(0.6):
            return 0.0
        lo, hi = 0.6, 30.0
        for _ in range(13):
            mid = (lo+hi)/2
            if ok(mid): lo = mid
            else: hi = mid
        return round(lo, 1)
    cut, pkt, holes = [], [], []
    for lp in loops:
        if lp["role"] == "INSIDE": cut.append(maxfit(lp["pts"], False))
        elif lp["role"] == "POCKET": pkt.append(maxfit(lp["pts"], True))
        elif lp.get("circle"): holes.append(round(lp["circle"][2]*2, 1))
    return {"profile": 99.0, "engrave": 99.0,
            "cutout": min(cut) if cut else None,
            "pocket": min(pkt) if pkt else None,
            "hole": min(holes) if holes else None}


def finish_allowance_for(pts, delta_sign, radius):
    """The radial skin the roughing passes should leave on this contour, in mm.

    Zero unless a finishing pass is switched on — and zero for any contour too
    narrow to rough at the wider offset, because leaving a skin there would mean
    the roughing pass vanishes and the "finishing" lap becomes the only pass,
    cutting full width at full depth. A feature that tight is simply cut to size
    in one go, which is what it got before finishing existed.
    """
    if not FINISH_PASS or FINISH_ALLOWANCE <= 0:
        return 0.0
    if delta_sign > 0:                       # outer profile: offsetting outward always fits
        return float(FINISH_ALLOWANCE)
    return float(FINISH_ALLOWANCE) if offset(pts, -(radius + FINISH_ALLOWANCE)) else 0.0


def compute_jobs(name, loops, base=0.0, tool_map=None):
    """Offset toolpaths for one part, bbox min translated to (base, base).

    `tool_map` = {feature: seat_no} explicit per-feature tool choices
    (profile / cutout / pocket / drill / engrave); a feature not in the map (or
    None) is Auto = widest loaded tool that fits. An assigned tool that doesn't fit
    a feature falls back to the widest that does, with a warning.
    Returns (jobs, (w, h)); jobs = [(role, path, pocket_depth, tool)] cut-ordered."""
    tm = tool_map or {}
    allpts = [pt for lp in loops for pt in lp["pts"]]
    minx = min(p[0] for p in allpts); miny = min(p[1] for p in allpts)
    maxx = max(p[0] for p in allpts); maxy = max(p[1] for p in allpts)
    def tr(pts): return [(x-minx+base, y-miny+base) for x, y in pts]

    def pick(feature, fit):
        """(tnum, fell_back) — assigned tool first if it fits, else widest-fits."""
        a = tm.get(feature)
        if a in TOOLS and a not in TOOL_PREF:
            # a V-bit or ball nose assigned to a cutting feature: Ø0 "fits" every
            # contour, and would be sent through 18 mm of board. Refuse it.
            print(f"    !! {feature}: T{a:02d} is not a cutting tool — assignment ignored")
            a = None
        order = ([a] if a in TOOLS else []) + [t for t in TOOL_PREF if t != a and t in TOOLS]
        for tnum in order:
            if fit(TOOLS[tnum]["dia"]):
                return tnum, (a in TOOLS and tnum != a)
        return None, False

    jobs = []
    cleared_voids = []
    for lp in loops:
        pts = tr(lp["pts"])
        role = lp["role"]
        if role == "OUTSIDE":
            if SKIP_PROFILE:                               # pockets-only: don't cut the outline
                continue                                   # (still used for nesting/footprint)
            assigned = tm.get("profile") if tm.get("profile") in TOOLS else None
            ptool, fell = pick("profile", lambda d: reproduces_outline(pts, "OUTSIDE", d))
            if ptool is None:
                # nothing loaded makes these reliefs: cut with the usual tool and say
                # what the corners will actually be
                ptool = assigned or DEFAULT_TOOL
                print(f"    !! {name}: modelled corner relief tighter than any loaded tool "
                      f"— it will come out as an R{TOOLS[ptool]['dia'] / 2:g} fillet")
            elif fell:
                print(f"    ! profile on {name}: chosen tool too wide for its corner reliefs "
                      f"— using Ø{TOOLS[ptool]['dia']:.0f}mm (T{ptool:02d})")
            elif assigned is None and ptool != DEFAULT_TOOL:
                print(f"    corner relief on {name}: profile cut with Ø{TOOLS[ptool]['dia']:.0f}mm "
                      f"(T{ptool:02d}) so the modelled radii are made")
            apex = relief_apexes(pts, "OUTSIDE") if DOGBONE else []
            pr = fit_radius(pts, "OUTSIDE", TOOLS[ptool]["dia"]) or TOOLS[ptool]["dia"]/2
            allow = finish_allowance_for(pts, +1, pr)
            for op in offset(pts, +pr + allow):
                jobs.append(("OUTSIDE", add_dogbones(op, apex), None, ptool))
            if allow:
                for op in offset(pts, +pr):
                    jobs.append(("OUTSIDE_FIN", add_dogbones(op, apex), None, ptool))

        elif role == "POCKET":
            if CLEAR_POCKETS:
                continue                                   # gathered + stepped after the loop
            # Clearing off: cut the recess's boundary at its own depth and leave the
            # middle standing. Offset inward like any cut-out so the finished wall
            # lands on the modelled line, and stop at the pocket depth rather than
            # the through-cut floor — an INSIDE job with a depth is exactly that.
            apex = relief_apexes(pts, "POCKET") if DOGBONE else []
            tnum, _fell = pick("cutout", lambda d: bool(offset(pts, -d/2)))
            if tnum is None:
                # a groove exactly one cutter wide: a single centreline pass
                slot_t = next((t for t in TOOL_PREF if t in TOOLS
                               and slot_centreline(pts, TOOLS[t]["dia"]/2)), None)
                if slot_t is not None:
                    jobs.append(("INSIDE", slot_centreline(pts, TOOLS[slot_t]["dia"]/2),
                                 lp.get("depth"), slot_t))
                else:
                    print(f"    !! pocket on {name} too narrow for any loaded tool — SKIPPED")
            else:
                pr = TOOLS[tnum]["dia"]/2
                allow = finish_allowance_for(pts, -1, pr)
                for op in offset(pts, -pr - allow):
                    jobs.append(("INSIDE", add_dogbones(op, apex), lp.get("depth"), tnum))
                if allow:
                    for op in offset(pts, -pr):
                        jobs.append(("INSIDE_FIN", add_dogbones(op, apex), lp.get("depth"), tnum))

        elif role == "ENGRAVE":
            e = tm.get("engrave")
            tnum = e if e in TOOLS else (ENGRAVE_TOOL if ENGRAVE_TOOL in TOOLS else TOOL_PREF[-1])
            jobs.append(("ENGRAVE", pts, engrave_depth_for(lp.get("text_h")), tnum))

        elif role == "SURFACE":
            continue                              # gathered below (needs two tools)

        elif lp["circle"]:                                # a hole
            r = lp["circle"][2]
            cen = (lp["circle"][0]-minx+base, lp["circle"][1]-miny+base)
            d = tm.get("drill")
            if d in TOOLS and abs(r - TOOLS[d]["dia"]/2) <= DRILL_TOL:
                jobs.append(("PLUNGE", [cen], None, d))
            elif any(abs(r - TOOLS[t]["dia"]/2) <= DRILL_TOL for t in TOOL_PREF):
                tnum = next(t for t in TOOL_PREF if abs(r - TOOLS[t]["dia"]/2) <= DRILL_TOL)
                jobs.append(("PLUNGE", [cen], None, tnum))
            else:
                tnum, _ = pick("drill", lambda dd: r > dd/2 + 0.05 and bool(offset(pts, -dd/2)))
                if tnum is not None:
                    tool_dia = TOOLS[tnum]["dia"]
                    if 2*r <= SMALL_HOLE_CLEAR_MAX_DIA:
                        rings = pocket_rings(pts, radius=tool_dia/2, stepover=tool_dia*0.5)
                        if rings:
                            jobs.append(("HOLE_CLEAR", spiral_from_rings(rings), None, tnum))
                            cleared_voids.append({"pts": pts,
                                                  "circle": (cen[0], cen[1], r)})
                    else:
                        for op in offset(pts, -tool_dia/2):
                            jobs.append(("DRILL", op, None, tnum))
                else:
                    print(f"    !! hole r={r:.1f}mm on {name} too small for any tool — SKIPPED")

        else:                                             # INSIDE cut-out / slot
            apex = relief_apexes(pts, role) if DOGBONE else []
            tnum, fell = pick("cutout", lambda d: bool(offset(pts, -d/2))
                              and reproduces_outline(pts, "INSIDE", d))
            if tnum is None:
                # goes in but rounds the corners: still the right thing to cut,
                # and the shortfall is said out loud
                tnum, fell = pick("cutout", lambda d: bool(offset(pts, -d/2)))
                if tnum is not None:
                    print(f"    !! cut-out on {name}: modelled corner relief tighter than any "
                          f"loaded tool — it will come out as an R{TOOLS[tnum]['dia'] / 2:g} fillet")
            if tnum is not None:
                pr = fit_radius(pts, "INSIDE", TOOLS[tnum]["dia"]) or TOOLS[tnum]["dia"]/2
                allow = finish_allowance_for(pts, -1, pr)
                for op in offset(pts, -pr - allow):
                    jobs.append((role, add_dogbones(op, apex), None, tnum))
                if allow:
                    for op in offset(pts, -pr):
                        jobs.append(("INSIDE_FIN", add_dogbones(op, apex), None, tnum))
                if fell:
                    print(f"    ! slot on {name}: chosen tool too wide — using Ø{TOOLS[tnum]['dia']:.0f}mm (T0{tnum})")
            else:
                print(f"    !! slot on {name} too narrow for any loaded tool — SKIPPED")

    # POCKETS — global Z-level batching.  Every compatible recess is cut at one level
    # before the family steps down, rather than treating 82 slots as 82 unrelated
    # operations.  A pocket that opens into an already area-cleared hole gets a proven
    # swept-disc excursion into that void, closing the otherwise-unreachable crescent.
    _pocket_loops = [lp for lp in loops if lp["role"] == "POCKET"] if CLEAR_POCKETS else []
    island_lps = [lp for lp in _pocket_loops if lp.get("islands")]
    pocket_lps = [(tr(lp["pts"]), lp.get("depth", 6.0)) for lp in _pocket_loops if not lp.get("islands")]
    prof_pts = next((tr(lp["pts"]) for lp in loops if lp["role"] == "OUTSIDE"), None)
    if pocket_lps:
        # A tool "handles" a recess by area-clearing it OR by cutting it as a
        # tool-width centreline slot — jig grooves are exactly one cutter wide.
        tnum, fell = pick("pocket", lambda dia:
                          all(pocket_rings(r, radius=dia/2, stepover=dia*0.5)
                              or slot_centreline(r, dia/2)
                              for r, _ in pocket_lps))
        if tnum is None:
            loaded = [t for t in TOOL_PREF if t in TOOLS]
            tnum = loaded[-1] if loaded else None
        if tnum is None:
            print(f"    !! pocket on {name}: no cutting tool loaded — SKIPPED")
        else:
            rt = TOOLS[tnum]["dia"] / 2
            step = TOOLS[tnum]["dia"] * 0.5
            # pull the tool-width grooves out of the batch: each is one
            # centreline slot pass at its own depth
            slotted = []
            batched = []
            for r, d in pocket_lps:
                if pocket_rings(r, radius=rt, stepover=step):
                    batched.append((r, d))
                    continue
                line = slot_centreline(r, rt)
                if line:
                    slotted.append((line, d))
                else:
                    batched.append((r, d))     # counted as nskip by the bands
            for line, d in slotted:
                jobs.append(("INSIDE", line, d, tnum))
            if slotted:
                print(f"    i {len(slotted)} groove(s) on {name} cut as "
                      f"Ø{2*rt:g}mm centreline slots")
            pocket_lps = batched
            # Waste the cutter may run on into when a remnant is too narrow:
            # holes already area-cleared, and cut-outs (their inside is scrap).
            waste = [v["pts"] for v in cleared_voids] + \
                    [tr(lp["pts"]) for lp in loops if lp["role"] == "INSIDE"]
            nskip = 0
            nrun_on = 0
            prev = 0.0
            for depth in sorted({round(d, 3) for _, d in pocket_lps}):
                active_raw = [r for r, d in pocket_lps if d >= depth - 1e-6]
                # Preserve stepped/overlapping recess behaviour inside the global batch.
                active = poly_union(active_raw) or active_raw
                paths = []
                for raw_region in active:
                    region = extend_to_profile(raw_region, prof_pts, ext=2*rt)
                    rings = pocket_rings(region, radius=rt, stepover=step)
                    if not rings:
                        grown = grow_into_waste(region, waste, 2*rt + 0.5)
                        rings = pocket_rings(grown, radius=rt, stepover=step) if grown else []
                        if rings:
                            region = grown
                            nrun_on += 1
                    if not rings:
                        nskip += 1
                        continue
                    path = spiral_from_rings(rings)
                    path = pocket_void_overruns(path, region, cleared_voids, rt)
                    paths.append(path)
                if paths:
                    jobs.append(("POCKET_BATCH", paths, (prev, depth), tnum))
                prev = depth
            if nrun_on:
                print(f"    i {nrun_on} narrow groove remnant(s) on {name} between through-features: "
                      f"cleared by running the Ø{2*rt:g}mm on into the hole / cut-out beside them")
            if nskip:
                print(f"    !! {nskip} pocket region(s) on {name} too narrow for Ø{2*rt:.0f}mm tool — "
                      f"SKIPPED (load or assign a cutter matching the groove width)")
            if fell:
                print(f"    ! pocket on {name}: chosen tool too wide — using Ø{2*rt:.0f}mm (T0{tnum})")

    # ISLAND POCKETS — recesses with a boss left standing. Each is handled on its own
    # (not merged with other pockets): clear the channel between wall and island, leaving
    # the island. Emits a POCKET_ISL job carrying a list of closed rings (Prog.island_pocket
    # traces them level-by-level with safe hops, never cutting across a boss).
    for lp in island_lps:
        boundary = tr(lp["pts"]); isls = [tr(i) for i in lp["islands"]]
        depth = lp.get("depth", 6.0)
        tnum, fell = pick("pocket", lambda dia: bool(pocket_island_rings(boundary, isls, radius=dia/2)))
        if tnum is None:
            loaded = [t for t in TOOL_PREF if t in TOOLS]
            tnum = loaded[-1] if loaded else None
        if tnum is None:
            print(f"    !! island pocket on {name}: no cutting tool loaded — SKIPPED")
            continue
        rt = TOOLS[tnum]["dia"] / 2
        rings = pocket_island_rings(extend_to_profile(boundary, prof_pts, ext=2*rt), isls, radius=rt)
        if rings:
            jobs.append(("POCKET_ISL", rings, (0.0, depth), tnum))
            print(f"    i island pocket on {name}: {len(isls)} island(s) left standing, "
                  f"cleared with Ø{2*rt:.0f}mm (T0{tnum}), {len(rings)} ring(s)")
            if fell:
                print(f"    ! island pocket on {name}: chosen tool too wide — using Ø{2*rt:.0f}mm (T0{tnum})")
        else:
            print(f"    !! island pocket on {name}: channel too narrow for any loaded tool — SKIPPED")

    # EDGE BREAK — chamfer the edges with the selected vee/chamfer cutter. `size` is the
    # chamfer WIDTH; the cut DEPTH = width / tan(half-angle), so a steeper bit just goes
    # deeper for the same width. Outer profile always; holes & pockets too when
    # scope == "all". Edges are TRACED centred on the true edge line (the flank cuts a
    # `size` chamfer on the part side). Holes are too narrow to trace, so they are
    # COUNTERSUNK with the tool point plunged until the cone reaches hole_radius + size.
    # 3D SURFACING — each SURFACE loop carries its own heightfield. Rough with
    # the widest flat cutting tool (depth-stepped, ramped entries, stock left
    # for the finish), then raster-finish with the ball nose. Both paths come
    # from the drop-cutter field, which is gouge-safe by construction.
    for lp in loops:
        if lp["role"] != "SURFACE":
            continue
        zmap, grid = lp.get("zmap"), lp.get("grid")
        stop = lp.get("top")
        if zmap is None or grid is None or stop is None:
            print(f"    !! sculpted region on {name} has no heightfield — SKIPPED")
            continue
        ball = tm.get("surface") if tm.get("surface") in TOOLS else BALL_TOOL
        no_ball = ball not in TOOLS or TOOLS[ball]["dia"] <= 0
        rough_t = TOOL_PREF[0] if TOOL_PREF and TOOL_PREF[0] in TOOLS else None
        if no_ball and not (lp.get("rough_without_ball") and rough_t is not None):
            print(f"    !! sculpted region on {name} but no ball-nose tool loaded — "
                  f"SKIPPED (set a seat's type to 'ballnose' in Tools & feeds)")
            continue
        import surfacing as SF
        g = dict(grid, ox=grid["ox"] - minx + base, oy=grid["oy"] - miny + base)
        smask = lp.get("mask")
        if no_ball:
            # A 3D job with no ball nose fitted: rough to the surface itself in
            # fine steps with the flat cutter and stop — a terraced shape at
            # size, with only the ball finish missing.
            # Full-depth layers first, then the fine steps only where those
            # layers left material — on the slopes — not over the whole area.
            import numpy as np
            flat = SF.Cutter("flat", TOOLS[rough_t]["dia"])
            coarse = float(MAT["max_step"])
            fine = min(coarse, SURFACE_TERRACE_STEP)
            rpaths, tgt, _ri = SF.raster_rough(zmap, g, flat, stock_top=float(stop),
                                               stepdown=coarse, stock_to_leave=0.0,
                                               mask=smask)
            levels, zl = [], float(stop)
            while zl > np.nanmin(tgt) + 1e-9:
                zl = max(float(np.nanmin(tgt)), zl - coarse)
                levels.append(zl)
            # Fine raster only where the face is gentle: on anything steeper the
            # waterlines below sit closer together than the cutter is wide and
            # leave nothing, and lanes there are hundreds of short hops.
            gy_, gx_ = np.gradient(np.where(np.isfinite(tgt), tgt, float(stop)), g["res"])
            gentle = np.hypot(gx_, gy_) < math.tan(math.radians(SURFACE_STEEP_DEG))
            upper = float(stop)
            for lower in levels:
                band = (np.isfinite(tgt) & (tgt > lower + 1e-6) & (tgt < upper - 1e-6)
                        & gentle)
                if smask is not None:
                    band &= smask
                if band.any():
                    fp, _t, _fi = SF.raster_rough(zmap, g, flat, stock_top=upper,
                                                  stepdown=fine, stock_to_leave=0.0,
                                                  mask=band)
                    rpaths += fp
                upper = lower
            # Raster lanes are 0.4 x dia apart, so on a steep face each layer
            # stops a lane short of it — up to 7 mm of height on a 70-degree
            # side. A waterline at every fine level follows the face exactly:
            # the contour of the flat cutter's drop field at that height is
            # where its edge just touches the surface.
            # The roughing field models the cutter as a square (safe, quick), which
            # holds it ~0.4 x radius off diagonal faces; the waterlines use the
            # round cutter, half a cell oversize so sampling cannot let it gouge.
            import contourpy
            round_flat = SF.Cutter("bull", TOOLS[rough_t]["dia"] + g["res"], corner=1e-6)
            wfield = np.minimum(SF.drop_field(zmap, round_flat, g["res"]), float(stop))
            gx = g["ox"] + np.arange(tgt.shape[1]) * g["res"]
            gy = g["oy"] + np.arange(tgt.shape[0]) * g["res"]
            cgen = contourpy.contour_generator(gx, gy, np.where(np.isfinite(wfield), wfield, float(stop)))
            floor = float(np.nanmin(tgt))
            # A contour point is interpolated between cells and the field is not
            # linear there, so on a steep face it can sit a cell too close. Each
            # point is checked against the real disc of the cutter on the
            # height map, and any that would touch above the level is dropped.
            Rr = TOOLS[rough_t]["dia"] / 2.0
            K = int(math.ceil(Rr / g["res"])) + 1
            ko = np.arange(-K, K + 2)
            zsafe = np.where(np.isfinite(zmap), zmap, -np.inf)

            def disc_max(line):
                bi = np.floor((line[:, 0] - g["ox"]) / g["res"]).astype(int)
                bj = np.floor((line[:, 1] - g["oy"]) / g["res"]).astype(int)
                ci = bi[:, None, None] + ko[None, None, :]
                cj = bj[:, None, None] + ko[None, :, None]
                cx = g["ox"] + ci * g["res"] - line[:, 0][:, None, None]
                cy = g["oy"] + cj * g["res"] - line[:, 1][:, None, None]
                inside = cx * cx + cy * cy <= Rr * Rr
                zz = zsafe[np.clip(cj, 0, zsafe.shape[0] - 1), np.clip(ci, 0, zsafe.shape[1] - 1)]
                return np.where(inside, zz, -np.inf).max(axis=(1, 2))

            zl = float(stop) - fine
            while zl > floor + 1e-6:
                for line in cgen.lines(zl):
                    if smask is not None:
                        ii = np.clip(np.round((line[:, 0] - g["ox"]) / g["res"]).astype(int), 0, tgt.shape[1] - 1)
                        jj = np.clip(np.round((line[:, 1] - g["oy"]) / g["res"]).astype(int), 0, tgt.shape[0] - 1)
                        keep = smask[jj, ii]
                    else:
                        keep = np.ones(len(line), dtype=bool)
                    keep &= disc_max(line) <= zl + 0.02
                    for s, e in SF._runs(keep):
                        if e - s >= 2:
                            rpaths.append([(float(x), float(y), zl) for x, y in line[s:e]])
                zl -= fine
            if rpaths:
                jobs.append(("SURFACE", rpaths, None, rough_t))
            print(f"    ! 3D part {name}: no ball nose loaded — ROUGHED ONLY with "
                  f"Ø{TOOLS[rough_t]['dia']:g}mm, slopes in {fine:g}mm steps so they "
                  f"come out terraced (load the ball nose for the finish)")
            continue
        if rough_t is not None:
            flat = SF.Cutter("flat", TOOLS[rough_t]["dia"])
            rpaths, _tgt, rinfo = SF.raster_rough(
                zmap, g, flat, stock_top=float(stop),
                stepdown=float(MAT["max_step"]), stock_to_leave=SURFACE_LEAVE,
                mask=smask)
            if rpaths:
                jobs.append(("SURFACE", rpaths, None, rough_t))
        bcut = SF.Cutter("ball", TOOLS[ball]["dia"])
        if lp.get("steep_cross"):
            # 3D jobs: lanes exactly on grid rows (between rows Z is read from
            # the nearest one, which on a 70-degree side digs in ~0.3 mm), and a
            # second pass across the first on steep faces — lanes running along
            # a steep side leave ridges a stepover tall up its height.
            import numpy as np
            res = g["res"]
            so = max(res, math.floor(SF.stepover_for_scallop(bcut.R, SURFACE_SCALLOP) / res) * res)
            fpaths, _drop, finfo = SF.raster_finish(zmap, g, bcut, stepover=so, mask=smask)
            gy_, gx_ = np.gradient(zmap, res)
            steep = np.hypot(gx_, gy_) > math.tan(math.radians(SURFACE_STEEP_DEG))
            steep = SF._max_filter_square(np.where(steep, 1.0, -np.inf),
                                          int(math.ceil(bcut.R / res))) > 0
            if smask is not None:
                steep &= smask
            if steep.any():
                xpaths, _d, _xi = SF.raster_finish(zmap, g, bcut, stepover=so, along="y",
                                                   mask=steep)
                fpaths += xpaths
        else:
            fpaths, _drop, finfo = SF.raster_finish(zmap, g, bcut, scallop=SURFACE_SCALLOP,
                                                    mask=smask)
        if fpaths:
            jobs.append(("SURFACE", fpaths, None, ball))
            print(f"    i sculpted region on {name}: roughed with "
                  f"Ø{TOOLS[rough_t]['dia']:g}mm, ball finish Ø{TOOLS[ball]['dia']:g} "
                  f"(T{ball:02d}) at {finfo['stepover_mm']}mm stepover, "
                  f"~{finfo['scallop_mm']}mm cusp, {finfo['lanes']} lanes")

    chamfer_tool = tm.get("chamfer") if tm.get("chamfer") in TOOLS else CHAMFER_TOOL
    if EDGE_BREAK and EDGE_BREAK_SIZE > 0 and chamfer_tool not in TOOLS:
        print(f"    !! edge break on but no chamfer or vee tool available — "
              f"edge break on {name} SKIPPED")
    if EDGE_BREAK and chamfer_tool in TOOLS and EDGE_BREAK_SIZE > 0:
        cd = float(EDGE_BREAK_SIZE)
        half = math.radians(max(1.0, min(179.0, EDGE_BREAK_ANGLE)) / 2.0)
        tan_h = math.tan(half) or 1e-6
        trace_depth = round(cd / tan_h, 3)
        allp = EDGE_BREAK_SCOPE == "all"
        for lp in loops:
            role, pts = lp["role"], tr(lp["pts"])
            if role == "OUTSIDE":
                if not SKIP_PROFILE:                       # no profile cut -> no edge to chamfer
                    jobs.append(("CHAMFER", pts, trace_depth, chamfer_tool))
            elif not allp:
                continue
            elif role == "POCKET" or (role == "INSIDE" and not lp.get("circle")):
                jobs.append(("CHAMFER", pts, trace_depth, chamfer_tool))
            elif lp.get("circle"):                          # hole -> countersink with the point
                cx, cy, r = lp["circle"]
                cen = (cx - minx + base, cy - miny + base)
                jobs.append(("CSINK", [cen], round((r + cd) / tan_h, 3), chamfer_tool))
        # Relieve the outer profile to just below the chamfer BEFORE the chamfer
        # pass: the vee then only kisses the part's top edge instead of ploughing
        # a full-width groove through solid stock, and the profile tool comes back
        # afterwards to finish the cut — so the part stays held by the remaining
        # thickness right through the chamfer and only comes free at the very end.
        pre_depth = round(trace_depth + 1.0, 3)
        # Take the relief from the FINISHED profile, not the roughed one. The vee
        # traces the modelled edge, so if it ran against a wall still carrying its
        # finishing skin the finishing lap would afterwards cut the chamfer off.
        pre_src = "OUTSIDE_FIN" if any(j[0] == "OUTSIDE_FIN" for j in jobs) else "OUTSIDE"
        for role, path, _pd, ptool in list(jobs):
            if role == pre_src:
                jobs.append(("OUTSIDE_PRE", path, pre_depth, ptool))

    # Area-cleared holes run before hole-aware pocket batches; then remaining holes,
    # cut-outs, engraving, profile relief, edge break and the finishing profile.
    # The part remains held until the final OUTSIDE pass.
    order = {"SURFACE": -1, "HOLE_CLEAR": 0, "POCKET_BATCH": 1, "POCKET": 1,
             "POCKET_ISL": 1, "PLUNGE": 2, "DRILL": 3, "INSIDE": 4, "INSIDE_FIN": 4.5,
             "ENGRAVE": 5, "OUTSIDE_PRE": 6, "CHAMFER": 7, "CSINK": 7, "OUTSIDE": 8,
             "OUTSIDE_FIN": 9}
    jobs.sort(key=lambda j: order[j[0]])
    return jobs, (maxx-minx, maxy-miny)


def tab_count(perim):
    return max(TAB_MIN, round(perim / TAB_SPACING))

def _emit_one(prog, role, path, pd, tab, clear, top, tool):
    if role == "ENGRAVE":                       # single shallow pass — no depth stepping
        prog.engrave(path, clear, top, pd if pd is not None else ENGRAVE_DEPTH)
        return
    if role == "POCKET":
        # pd is a (top_depth, bottom_depth) z-band measured from the stock top; this
        # layer cuts only its own slab, ramping from the level above (already cleared).
        z0, z1 = pd if isinstance(pd, tuple) else (0.0, pd if pd is not None else 0.0)
        layer_top = top - z0
        depths = depth_passes(layer_top, top - z1, TOOLS[tool]["step"])
        prog.spiral_pocket(path, clear, depths, layer_top)
        return
    if role == "POCKET_BATCH":
        z0, z1 = pd if isinstance(pd, tuple) else (0.0, pd if pd is not None else 0.0)
        layer_top = top - z0
        depths = depth_passes(layer_top, top - z1, TOOLS[tool]["step"])
        prog.pocket_batch(path, clear, depths, layer_top, top)
        return
    if role == "POCKET_ISL":                    # pocket with island(s): path = list of rings
        z0, z1 = pd if isinstance(pd, tuple) else (0.0, pd if pd is not None else 0.0)
        layer_top = top - z0
        depths = depth_passes(layer_top, top - z1, TOOLS[tool]["step"])
        prog.island_pocket(path, clear, depths, layer_top)
        return
    if role == "HOLE_CLEAR":                   # full-depth area clear; leaves no loose core
        depths = depth_passes(top, MAT["final_z"], TOOLS[tool]["step"])
        prog.spiral_pocket(path, clear, depths, top)
        return
    if role == "CHAMFER":                       # trace the edge in one pass at chamfer depth
        prog.profile(path, clear, [top - pd], top)
        return
    if role == "SURFACE":                       # 3D raster: path = (x,y,z) polylines
        prog.surface(path, clear, top)
        return
    if role == "OUTSIDE_PRE":
        # Partial-depth profile relief ahead of the chamfer pass. Clamped so it can
        # never sever the part: at least 2mm of board is left holding it (final_z is
        # ~0.5 below the board bottom), whatever chamfer size the form asked for.
        bottom = max(top - pd, MAT["final_z"] + 2.5)
        if bottom >= top - 1e-6:
            return                              # nothing useful to relieve
        prog.profile(path, clear, depth_passes(top, bottom, TOOLS[tool]["step"]), top)
        return
    if role == "CSINK":                         # countersink a hole: plunge the point to depth
        prog.peck(path[0], clear, depth_passes(top, top - pd, TOOLS[tool]["step"]), top)
        return
    bottom = (top - pd) if pd is not None else MAT["final_z"]
    if role in ("OUTSIDE_FIN", "INSIDE_FIN") and not FINISH_STEPPED:
        # One continuous lap. The skin is thin and the channel beside it is already
        # open, so the cutter is barely loaded at full depth — and a single lap is
        # what leaves a wall with no step marks, which is the point of the pass.
        depths = [bottom]
    else:
        depths = depth_passes(top, bottom, TOOLS[tool]["step"])
    if role == "PLUNGE":
        prog.peck(path[0], clear, depths, top)
    elif role in ("OUTSIDE", "OUTSIDE_FIN") and tab:
        perim = sum(math.dist(a, b) for a, b in zip(path, path[1:] + path[:1]))
        prog.profile(path, clear, depths, top, tab_z=TAB_REMAIN, n_tabs=tab_count(perim))
    else:
        prog.profile(path, clear, depths, top)


def emit_sheet(prog, jobs, clear, top, part_cuts=None, part_tool=None):
    """Emit a whole program from jobs = [(role, path, pocket_depth, tool, tab)],
    grouping by tool with a tool change between groups (Ø6 first, then Ø4). Engrave
    strokes in a tool group are emitted together (optimised). `part_cuts` (sheet-coord
    lines) are cut LAST — after every part — to part the nested strip off from the
    scrap, on `part_tool` (a loaded CUTTING seat; engrave/V-bit seats are rejected)
    or, if not given/invalid, the widest loaded cutting tool."""
    # PHASE-ordered emission. Centre-cleared holes precede pocket batches because those
    # batches may deliberately use the proven voids for slot-end relief. Each phase
    # groups by tool, keeping tool changes bounded while preserving the safe dependency.
    phase_roles = [("SURFACE",), ("HOLE_CLEAR",),
                   ("POCKET_BATCH", "POCKET", "POCKET_ISL"),
                   ("PLUNGE", "DRILL"), ("INSIDE",), ("INSIDE_FIN",), ("ENGRAVE",),
                   ("OUTSIDE_PRE",), ("CHAMFER", "CSINK"), ("OUTSIDE",),
                   ("OUTSIDE_FIN",)]
    norm_rpm = MAT["spindle"]
    drill_rpm = MAT.get("drill_rpm") or norm_rpm
    drill_feed = MAT.get("drill_feed") or 0.0
    chamfer_rpm = MAT.get("chamfer_rpm") or norm_rpm
    chamfer_feed = MAT.get("chamfer_feed") or 0.0

    # tools needed, in first-use (phase) order — header takes the first
    tools, seen = [], set()
    for roles in phase_roles:
        for j in jobs:
            if j[0] in roles and j[3] not in seen:
                seen.add(j[3]); tools.append(j[3])
    if not tools:
        tools = [DEFAULT_TOOL]
    prog.header(tools[0]); cur = tools[0]

    for roles in phase_roles:
        pjobs = [j for j in jobs if j[0] in roles]
        if not pjobs:
            continue
        # holes and the chamfer pass run slower than the profile/pockets (anti-scorch):
        # pick this phase's spindle/feed override, applied then restored per tool group.
        ph_factor, ph_cut = 1.0, 0.0
        if roles[0] == "PLUNGE":
            ph_rpm, ph_feed = drill_rpm, drill_feed
        elif roles[0] == "CHAMFER":
            ph_rpm, ph_feed = chamfer_rpm, chamfer_feed
        else:
            ph_rpm, ph_feed = norm_rpm, 0.0
            # Internal features run slower than the outer profile: a concave cut
            # engages more of the cutter than a convex one, and small internal
            # contours flex it — full profile feed translates into slight errors
            # on holes and pockets while the outside comes out fine. The outer
            # profile and engraving keep full feed. A manual per-category feed
            # (mm/min) wins over the automatic factor when set.
            # NB the override goes through ph_cut, NOT ph_feed: ph_feed is the
            # drill/chamfer mechanism and sets the PLUNGE feed too, which would
            # let a cut-feed override RAISE the plunge rate.
            if roles[0] in ("INSIDE", "INSIDE_FIN"):
                ph_cut = float(MAT.get("inside_feed") or 0.0)
                if ph_cut <= 0:
                    ph_factor = float(MAT.get("internal_feed_factor") or 1.0)
            elif roles[0] in ("HOLE_CLEAR", "POCKET_BATCH"):
                ph_cut = float(MAT.get("pocket_feed") or 0.0)
                if ph_cut <= 0:
                    ph_factor = float(MAT.get("internal_feed_factor") or 1.0)
        for tnum in sorted({j[3] for j in pjobs}, key=lambda t: TOOL_PREF.index(t) if t in TOOL_PREF else 99):
            if tnum != cur:
                prog.toolchange(tnum, clear); cur = tnum
            grp = [j for j in pjobs if j[3] == tnum]
            slow = ph_rpm != norm_rpm or ph_feed or ph_cut > 0 or ph_factor != 1.0
            if slow:
                if ph_rpm != norm_rpm:
                    prog.spindle(ph_rpm)
                spf, scf = prog.plunge_feed, prog.cut_feed
                if ph_feed:
                    prog.plunge_feed = prog.cut_feed = ph_feed
                elif ph_cut > 0:
                    prog.cut_feed = round(ph_cut, -1)
                elif ph_factor != 1.0:
                    # Scale only the translating feed; plunges and ramps already
                    # run at the (conservative) plunge feed.
                    prog.cut_feed = round(scf * ph_factor, -1)
            eng = []
            for role, path, pd, tool, tab in grp:
                if role == "ENGRAVE":
                    eng.append((path, pd if pd is not None else ENGRAVE_DEPTH))
                else:
                    _emit_one(prog, role, path, pd, tab, clear, top, tool)
            if eng:                                   # whole label as one optimised group
                prog.engrave_group(eng, clear, top)
            if slow:
                prog.plunge_feed, prog.cut_feed = spf, scf
                if ph_rpm != norm_rpm:
                    prog.spindle(norm_rpm)
    # Parting cuts run on a real CUTTING tool — the requested seat if it's a loaded
    # cutting tool (engrave/V-bit seats aren't in TOOL_PREF, so they're rejected),
    # else the widest loaded cutting tool. Tool-change to it first if needed.
    cuts = part_cuts or []
    if cuts:
        cutting = [t for t in TOOL_PREF if t in TOOLS]
        ptool = part_tool if part_tool in cutting else (cutting[0] if cutting else DEFAULT_TOOL)
        if cur != ptool:
            prog.toolchange(ptool, clear); cur = ptool
            if ptool not in tools:
                tools.append(ptool)
        step = TOOLS[ptool]["step"] if ptool in TOOLS else MAT["max_step"]
        for line in cuts:                 # scrap-separation cuts, last
            prog.partcut(line, clear, depth_passes(top, step=step), top)
    prog.footer(clear)
    return tools


def tab_count(perim):
    return max(TAB_MIN, round(perim / TAB_SPACING))


def tab_layout(path, tab_w=TAB_WIDTH):
    """Centre points of the holding tabs along a closed OUTSIDE path — the SAME
    arc-length centres profile() cuts — for previewing/confirming tab positions."""
    p = path if (path and path[0] == path[-1]) else path + [path[0]]
    cum = [0.0]
    for a, b in zip(p, p[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    L = cum[-1]
    if L <= 0:
        return []
    n = tab_count(L)
    centers = [L*(k+0.5)/n for k in range(n)]

    def pt_at(s):
        s = max(0.0, min(L, s))
        for i in range(1, len(p)):
            if cum[i] >= s:
                seg = cum[i]-cum[i-1]; t = 0 if seg <= 0 else (s-cum[i-1])/seg
                return (p[i-1][0]+(p[i][0]-p[i-1][0])*t, p[i-1][1]+(p[i][1]-p[i-1][1])*t)
        return p[-1]
    return [pt_at(c) for c in centers]


def gen_part(name, loops, thickness):
    """Single-part Syntec program (part placed at +MARGIN from origin)."""
    jobs, (w, h) = compute_jobs(name, loops, base=MARGIN)
    top = thickness
    clear = top + MAT["clearance_over"]
    do_tabs = max(w, h) < SMALL_PART_MAXDIM
    flat = [(r, p, pd, t, do_tabs and r == "OUTSIDE") for r, p, pd, t in jobs]
    prog = Prog()
    emit_sheet(prog, flat, clear, top)
    return prog.dumps(), jobs, depth_passes(top), do_tabs


# NOTE: this module's Onshape CLI driver (main()) was removed when the engine
# moved into the fab exporter. Geometry now arrives already extracted, from the
# exporter's STEP reader. Nothing else in this file touched Onshape.
