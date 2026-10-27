"""Nest the 18mm parts onto 2440x1220 sheet(s) and emit one Syntec program per
sheet (sheet-corner origin), matching the proven multi-part .NC files.

Polygon bottom-left nester: places actual part shapes (not bounding boxes) with
0/90/180/270 rotation, interlocking concave/diagonal parts, while enforcing a
real minimum gap between part outlines (collision tested on shapes buffered by
MIN_GAP/2). Multi-sheet: parts that don't fit open a new sheet.
"""
import sys, math, json
from pathlib import Path
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass
import pyclipper
import engrave
from cam_syntec import (compute_jobs, depth_passes, emit_sheet, Prog, MAT, TOOLS,
                        SMALL_PART_MAXDIM, R, offset, SCALE, OUT, tab_layout, TAB_WIDTH,
                        feature_limits, poly_union, relief_apexes, _point_in)

# Portrait, matching the bed: 1220 across (X), 2440 up (Y).
SHEET_W, SHEET_H = 1220.0, 2440.0
# The router's bed, in MACHINE coordinates: a portrait rectangle 1220 across (X) by
# 2440 up (Y), origin at the bottom-left corner, tool changer along the top edge. A
# full 2440x1220 sheet therefore only fits laid across it, which is what swap_xy
# does. A smaller sheet is squared into the X0/Y0 corner, so the bed is a fixed
# frame the preview always draws and the sheet sits inside it.
BED_W, BED_H = 1220.0, 2440.0
MIN_GAP = 12.0       # minimum gap between part outlines (>= cutter dia + clearance)
# Pack the nest hard into the X0/Y0 corner. The placer seats each part's keepout
# (= outline grown by profile-radius + MIN_GAP/2) against EDGE, and the OUTSIDE
# toolpath runs one profile-radius beyond the outline — those radii cancel, so the
# toolpath's low corner = EDGE + MIN_GAP/2. CORNER_MARGIN keeps it a hair off dead-
# zero (a profile/dogbone can dip ~0.1mm past the outline; landing on exactly 0 then
# trips the gate's "leaves sheet" bounds check with a tiny negative). 1mm is still
# in the corner; real clamp/edge clearance is taken up by the work offset (G54).
CORNER_MARGIN = 1.0
EDGE = CORNER_MARGIN - MIN_GAP / 2.0
STEP = 12.0          # nesting search grid (mm); smaller = tighter but slower
PREVIEW_TOL = 0.4    # mm: decimate plan-view polylines (preview only; NC unaffected — kept fine so curves render faithfully)
TABS_MODE = "auto"   # "auto" (small parts only) | "on" (all parts) | "off" (none)


# ---------- geometry ----------
def rot(pts, k):
    k %= 4
    if k == 0: return [(x, y) for x, y in pts]
    if k == 1: return [(-y, x) for x, y in pts]
    if k == 2: return [(-x, -y) for x, y in pts]
    return [(y, -x) for x, y in pts]

def bbox(pts):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

def sc(p): return [(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in p]

def intersects(a, b):
    return _intersects_sc(sc(a), sc(b))

def _intersects_sc(a_sc, b_sc):
    """Polygon overlap test on pre-scaled integer paths (avoids re-scaling the
    fixed placed polygons on every grid cell)."""
    pc = pyclipper.Pyclipper()
    pc.AddPath(a_sc, pyclipper.PT_SUBJECT, True)
    pc.AddPath(b_sc, pyclipper.PT_CLIP, True)
    sol = pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    return any(abs(pyclipper.Area(s)) > 1.0 for s in sol)


def _poly_area(pts):
    """Absolute polygon area (shoelace), mm^2 — the part's TRUE footprint, not its
    bounding box, so sheet utilisation is honest and matches the true-shape nest."""
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(pts, pts[1:] + pts[:1]))) / 2.0


def place_on_sheet(keepout, placed, innerW, innerH, rotations=(0, 1, 2, 3), edge=None):
    """Bottom-left placement of `keepout` among `placed` polys. Returns
    (k, dx, dy, transformed_keepout, kbbox) or None. `rotations` limits the allowed
    90° steps (e.g. (0, 2) keeps grain direction fixed for grain-locked sheet stock).
    `edge` is the keep-out origin offset (defaults to the module EDGE); the caller
    derives it from the requested sheet border — see nest_and_emit.

    Fast path: at each grid cell we only build the candidate polygon + run the
    (expensive) clipper intersection when a placed part's bbox actually overlaps;
    the vast empty region is cleared by cheap bbox maths alone. True-shape (polygon)
    collision lets concave/diagonal parts interlock. Among all rotations + leftmost-
    lowest fits, the placement with the smallest right edge (then lowest) wins, so the
    nest fills the full width and grows along the length as little as possible."""
    edge = EDGE if edge is None else edge
    placed_sc = [(sc(pk), pb) for pk, pb in placed]    # scale fixed polys once
    best = None
    for k in rotations:
        rk = rot(keepout, k)
        rminx, rminy, rmaxx, rmaxy = bbox(rk)
        w, h = rmaxx-rminx, rmaxy-rminy
        if w > innerW or h > innerH:
            continue
        # ROW-MAJOR: scan Y (up the sheet) outer, X (across it) inner, so a row fills
        # ACROSS the width from the origin before the nest steps up the length. The
        # sheet is portrait — 1220 across, 2440 up — so this fills the short dimension
        # first and grows along the long one, leaving the remainder as one clean strip
        # at the far end that a single parting cut can free.
        found = None
        py = 0.0
        while py <= innerH - h and not found:
            px = 0.0
            while px <= innerW - w:
                cb0, cb1, cb2, cb3 = edge+px, edge+py, edge+px+w, edge+py+h
                overlappers = [(pk_sc, pb) for pk_sc, pb in placed_sc
                               if not (cb0 > pb[2] or cb2 < pb[0] or cb1 > pb[3] or cb3 < pb[1])]
                if not overlappers:
                    dx = edge + px - rminx; dy = edge + py - rminy
                    found = (k, dx, dy, [(x+dx, y+dy) for x, y in rk], (cb0, cb1, cb2, cb3))
                    break
                dx = edge + px - rminx; dy = edge + py - rminy
                cand_sc = sc([(x+dx, y+dy) for x, y in rk])
                hit = [pb for pk_sc, pb in overlappers if _intersects_sc(cand_sc, pk_sc)]
                if not hit:
                    found = (k, dx, dy, [(x+dx, y+dy) for x, y in rk], (cb0, cb1, cb2, cb3))
                    break
                # skip along past the right edge of the furthest part we actually hit
                jump_px = max(pb[2] for pb in hit) - edge
                px = max(px + STEP, math.ceil(jump_px / STEP) * STEP)
            py += STEP
        # Minimise the LENGTH used (top edge, cb[3]) then fill from the left (cb[0]),
        # across all rotations — so parts pack across the FULL width before the nest
        # grows up the sheet. (No corner-compaction: pulling parts to the corner
        # clusters them and wastes the width.)
        if found and (best is None or (found[4][3], found[4][0]) < (best[4][3], best[4][0])):
            best = found
    return best


def prepare_part(name, loops, meta=None):
    """Translate a part's loops to origin and build its jobs + keep-out polygon.
    `loops` = list of {role, pts, circle[, depth]} (extract.part_features shape).
    `meta` = {flip, two_sided, up_side} orientation info (extract.part_features).
    Returns a part dict ready for nest_and_emit."""
    meta = meta or {}
    allpts = [pt for lp in loops for pt in lp["pts"]]
    mnx = min(p[0] for p in allpts); mny = min(p[1] for p in allpts)
    for lp in loops:
        lp["pts"] = [(x-mnx, y-mny) for x, y in lp["pts"]]
        if lp.get("circle"):
            cx, cy, r = lp["circle"]; lp["circle"] = (cx-mnx, cy-mny, r)
        if lp.get("islands"):     # islands MUST move to the same frame as pts, or the
            lp["islands"] = [[(x-mnx, y-mny) for x, y in isl] for isl in lp["islands"]]
        if lp.get("grid"):        # a SURFACE heightfield moves with its region
            lp["grid"] = dict(lp["grid"], ox=lp["grid"]["ox"]-mnx, oy=lp["grid"]["oy"]-mny)
    # pocket outlines (for the plan view) — same frame as the jobs (origin-translated)
    # `through` separates a recess with a floor left in it (a counterbore, a blind
    # pocket — needs the part the right way up) from one that breaks out the far
    # side. The preview colours the two apart.
    pocket_outlines = [{"pts": lp["pts"], "depth": lp.get("depth"),
                        "islands": lp.get("islands") or [],
                        "through": bool(lp.get("through"))}
                       for lp in loops if lp["role"] == "POCKET"]
    # The part's TRUE inner geometry, kept for the plan view. The jobs carry tool-CENTRE
    # paths — an inner feature's toolpath sits one cutter radius INSIDE it, so drawing the
    # jobs draws every hole undersize and collapses any radius tighter than the cutter to
    # a point. These are the loops as modelled, so the preview shows the part rather than
    # the machine's route around it. A DRILL loop keeps its fitted circle, so a hole is
    # drawn at its real diameter instead of a fixed dot.
    inner_outlines = [lp["pts"] for lp in loops if lp["role"] == "INSIDE"]
    surface_regions = [lp["pts"] for lp in loops if lp["role"] == "SURFACE"]
    hole_circles = [tuple(lp["circle"]) for lp in loops
                    if lp["role"] == "DRILL" and lp.get("circle")]
    # part-number engraving: place the label in a clear area, add open ENGRAVE strokes
    engrave_strokes = []
    if meta.get("engrave") and meta.get("label"):
        engrave_strokes = engrave.place_label(loops, meta["label"],
                                              max_h=meta.get("engrave_height", 25.0),
                                              offset=tuple(meta.get("engrave_offset") or (0.0, 0.0))) or []
        # measure the placed label's text height (its thin dimension, robust to
        # horizontal/vertical placement) so the engrave DEPTH scales with text size.
        allp = [pt for st in engrave_strokes for pt in st]
        text_h = min(max(p[0] for p in allp) - min(p[0] for p in allp),
                     max(p[1] for p in allp) - min(p[1] for p in allp)) if allp else 0.0
        for st in engrave_strokes:
            loops.append({"role": "ENGRAVE", "pts": st, "circle": None, "text_h": text_h})
    jobs, (w, h) = compute_jobs(name, loops, base=0.0, tool_map=meta.get("tool_map"))
    limits = feature_limits(loops)
    outer = max((lp["pts"] for lp in loops if lp["role"] == "OUTSIDE"),
                key=lambda p: abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(p, p[1:]+p[:1]))))
    # Keep-out gap derives from the ACTUAL profile tool used for the OUTSIDE cut,
    # read back from the computed jobs so it always matches what compute_jobs picked.
    # The tool-CENTRE path sits one radius outside the outline, so wider cutters
    # reserve more room (never welding neighbours together); Ø6 — the usual profile
    # tool — reproduces the previous spacing exactly.
    out_tools = [t for (role, _p, _pd, t) in jobs if role == "OUTSIDE" and t in TOOLS]
    ptool_r = (TOOLS[out_tools[0]]["dia"] / 2.0) if out_tools else R
    keep = max(offset(outer, ptool_r + MIN_GAP/2), key=lambda p: len(p))   # buffered keep-out
    maxdim = max(w, h)
    # Outline in job-local coords (compute_jobs shifts the part min-corner to 0). Kept so
    # the swap-shift extent and the plan preview have the part edge even in pockets-only
    # mode, where the OUTSIDE profile is never emitted as a job.
    _mnx = min(p[0] for lp in loops for p in lp["pts"])
    _mny = min(p[1] for lp in loops for p in lp["pts"])
    footprint = [(x - _mnx, y - _mny) for x, y in outer]
    return {"name": name, "jobs": jobs, "keepout": keep, "area": w*h,
            "poly_area": _poly_area(outer), "footprint": footprint,
            "maxdim": maxdim, "tab": maxdim < SMALL_PART_MAXDIM,
            "pocket_outlines": pocket_outlines,
            "surface_regions": surface_regions,
            "inner_outlines": inner_outlines, "hole_circles": hole_circles,
            "engrave_strokes": engrave_strokes, "engrave_text": meta.get("label", ""),
            "feature_limits": limits,
            "flip": meta.get("flip", False), "two_sided": meta.get("two_sided", False),
            "up_side": meta.get("up_side", "TOP")}


# Below this full-thickness contact fraction the flipped part cannot be trusted
# to sit flat on the spoilboard (picture a 5mm boss carrying a 300mm board), so
# the mould floor is sculpted to the part's inverse instead of cut through.
MOULD_SUPPORT_MIN = 0.25
MOULD_VOID_CLEARANCE = 1.0    # mm grown around non-locating voids in a sculpted floor


def _poly_clip(op, subject_polys, clip_polys):
    """Boolean of ring sets -> list of rings (pyclipper orientation: outers
    CCW, holes CW). Subject/clip rings keep their given orientation, so a
    polygon-with-holes participates correctly."""
    pc = pyclipper.Pyclipper()
    for p in subject_polys:
        if len(p) >= 3:
            pc.AddPath(sc(p), pyclipper.PT_SUBJECT, True)
    for p in clip_polys:
        if len(p) >= 3:
            pc.AddPath(sc(p), pyclipper.PT_CLIP, True)
    try:
        sol = pc.Execute(op, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
    except pyclipper.ClipperException:
        return []
    return [[(x / SCALE, y / SCALE) for x, y in p] for p in sol if len(p) >= 3]


def _poly_diff(subject_polys, clip_polys):
    return _poly_clip(pyclipper.CT_DIFFERENCE, subject_polys, clip_polys)


def _rings_offset(rings, delta):
    """Offset a ring SET (outers grow, holes shrink for delta > 0)."""
    po = pyclipper.PyclipperOffset()
    for p in rings:
        if len(p) >= 3:
            po.AddPath(sc(p), pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    sol = po.Execute(delta * SCALE)
    return [[(x / SCALE, y / SCALE) for x, y in p] for p in sol if len(p) >= 3]


def prepare_mould(name, back_loops, meta=None, far_pockets=None, thickness=None):
    """The flip-jig for a part with 2.5D features on BOTH faces.

    `back_loops` is the part read from its FAR face, in that face's own outward
    frame — i.e. the mirrored view the machine sees once the part is flipped
    over. `far_pockets` are the NEAR side's recess regions in the same frame:
    they say where the flipped part is thinner than full stock, i.e. where it
    would NOT touch the spoilboard through a plain cavity.

    Two mould styles, chosen by how much of the part actually reaches full
    thickness:

      plain  — the far-face outline as a snug through-cavity; the part rests
        on the spoilboard on its full-thickness face. Fast, and right for any
        part that sits flat (contact fraction >= MOULD_SUPPORT_MIN).
      inverse — when the contact patch is too small to locate the part in Z
        (a tiny boss carrying a wide board), the mould is machined as the
        part's 2.5D INVERSE instead: everywhere the part is t thick the mould
        floor is milled t deep, so every region of the flipped part is
        supported at its own level; the full-thickness regions (grown by a
        non-locating clearance) go through to the spoilboard. XY location
        still comes from the outline wall — kerf-matched, with a through
        drill-relief at each convex corner so sharp corners seat. Slower, so
        only used when needed.

    Pass 2 is the same either way: only the far face's recesses, run as a
    separate program after the operator drops the flipped part in. All jobs
    live on one nest part, so wherever the nest (or a drag) puts the mould,
    everything lands together."""
    meta = meta or {}
    allpts = [pt for lp in back_loops for pt in lp["pts"]]
    mnx = min(p[0] for p in allpts); mny = min(p[1] for p in allpts)
    loops = []
    for lp in back_loops:
        d = dict(lp)
        d["pts"] = [(x - mnx, y - mny) for x, y in lp["pts"]]
        if lp.get("islands"):
            d["islands"] = [[(x - mnx, y - mny) for x, y in isl]
                            for isl in lp["islands"]]
        if lp.get("grid"):     # a far-side SURFACE heightfield rides along
            d["grid"] = dict(lp["grid"], ox=lp["grid"]["ox"] - mnx,
                             oy=lp["grid"]["oy"] - mny)
        loops.append(d)
    outline = next(lp["pts"] for lp in loops if lp["role"] == "OUTSIDE")
    far = []
    for fp in far_pockets or []:
        far.append({"pts": [(x - mnx, y - mny) for x, y in fp["pts"]],
                    "depth": fp["depth"],
                    "inner": [[(x - mnx, y - mny) for x, y in w]
                              for w in fp.get("inner") or []]})

    # Which style? The full-thickness contact = footprint minus the near side's
    # recess regions. Plenty of contact -> plain; a sliver -> sculpted inverse.
    a_out = abs(_poly_area(outline))
    contact = 1.0
    if far and thickness and a_out > 1e-6:
        far_union = poly_union([f["pts"] for f in far]) or [f["pts"] for f in far]
        a_far = sum(abs(_poly_area(p)) for p in far_union)
        contact = max(0.0, (a_out - a_far) / a_out)
    inverse = bool(far and thickness) and contact < MOULD_SUPPORT_MIN

    if not inverse:
        # plain: the outline as a snug through-cavity (an INSIDE cutout by role)
        cavity_jobs, (w, h) = compute_jobs(
            name, [{"role": "INSIDE", "pts": outline, "circle": None}],
            tool_map=meta.get("tool_map"))
        mould_pockets = []
    else:
        T = float(thickness)
        through = T - MAT["final_z"]
        mould_loops = []
        # supported floors: where the part is (T - depth) thick, mill the mould
        # depth (T - depth) so the floor carries that region at its own level
        for f in far:
            mould_loops.append({"role": "POCKET", "pts": f["pts"], "circle": None,
                                "depth": round(T - f["depth"], 3)})
        # through voids: the full-thickness regions (they rest on the spoilboard),
        # grown by a non-locating clearance and clipped back to the outline so
        # the growth never eats the locating wall. Holes surviving in the result
        # are floor regions inside a void — kept as islands so the clearing
        # goes around them.
        full = _poly_diff([outline], [f["pts"] for f in far])
        rings = _poly_clip(pyclipper.CT_INTERSECTION,
                           _rings_offset(full, MOULD_VOID_CLEARANCE), [outline])
        outers = [r for r in rings if pyclipper.Orientation(sc(r))]
        holes = [r for r in rings if not pyclipper.Orientation(sc(r))]
        for r in outers:
            isl = [h for h in holes if _point_in(h[0], r)]
            lp = {"role": "POCKET", "pts": r, "circle": None, "depth": through}
            if isl:
                lp["islands"] = isl
            mould_loops.append(lp)
        # the near side's own islands/bores are full thickness (or empty) inside
        # a floored region — voided with the same clearance either way
        for f in far:
            for wire in f["inner"]:
                for g in offset(wire, +MOULD_VOID_CLEARANCE):
                    if len(g) >= 3:
                        mould_loops.append({"role": "POCKET", "pts": g,
                                            "circle": None, "depth": through})
        # sharp convex corners of the part meet cutter-radius fillets in the
        # cavity wall — a small through relief pocket at each apex (the dogbone,
        # as a pocket so it rides the proven ramped clearing) lets them seat
        rr = R + 0.5
        for ax, ay in relief_apexes(outline, "INSIDE"):
            ring = [(ax + rr * math.cos(t), ay + rr * math.sin(t))
                    for t in (k * 2 * math.pi / 24 for k in range(24))]
            mould_loops.append({"role": "POCKET", "pts": ring, "circle": None,
                                "depth": through})
        # deliberately NO OUTSIDE loop: the outline wall is formed by the pocket
        # clearing itself (kerf-matched, cutter inside the line) and must never
        # be profile-cut or edge-extended — it is the locating surface, and a
        # through profile ring would sever the sculpted floor from the sheet.
        cavity_jobs, _wh0 = compute_jobs(name, mould_loops,
                                         tool_map=meta.get("tool_map"))
        cavity_jobs = [j for j in cavity_jobs
                       if j[0] in ("POCKET_BATCH", "POCKET_ISL", "POCKET")
                       or (j[0] == "INSIDE" and j[2] is not None)]
        xs = [p[0] for p in outline]; ys = [p[1] for p in outline]
        w, h = max(xs) - min(xs), max(ys) - min(ys)
        mould_pockets = [{"pts": lp["pts"], "depth": lp.get("depth"),
                          "islands": lp.get("islands") or [], "through": False}
                         for lp in mould_loops if lp["role"] == "POCKET"]

    # pass 2: the far face's recesses only. compute_jobs emits cleared recesses
    # as POCKET_BATCH/POCKET_ISL and (clearing off) profiled ones as INSIDE
    # with a depth; anything else (the outline, through cuts) is not pass-2 work.
    all_jobs, _wh = compute_jobs(name, loops, tool_map=meta.get("tool_map"))
    pass2 = [j for j in all_jobs
             if j[0] in ("POCKET_BATCH", "POCKET_ISL", "POCKET", "SURFACE")
             or (j[0] == "INSIDE" and j[2] is not None)]

    # plan view: the sculpted floors (inverse mode) plus the pass-2 features —
    # both drawn in the mould colour, each with its depth on hover
    pocket_outlines = mould_pockets + \
        [{"pts": lp["pts"], "depth": lp.get("depth"),
          "islands": lp.get("islands") or [], "through": False}
         for lp in loops if lp["role"] == "POCKET"]
    out_tools = [t for (role, _p, _pd, t) in cavity_jobs
                 if role == "INSIDE" and t in TOOLS]
    ptool_r = (TOOLS[out_tools[0]]["dia"] / 2.0) if out_tools else R
    keep = max(offset(outline, ptool_r + MIN_GAP/2), key=lambda p: len(p))
    return {"name": name, "jobs": cavity_jobs, "pass2_jobs": pass2,
            "mould": True, "mould_mode": "inverse" if inverse else "plain",
            "mould_contact": round(contact, 3),
            "keepout": keep, "area": w*h,
            "poly_area": _poly_area(outline), "footprint": outline,
            "maxdim": max(w, h), "tab": False,
            "pocket_outlines": pocket_outlines,
            "inner_outlines": [], "hole_circles": [],
            "engrave_strokes": [], "engrave_text": "",
            "feature_limits": feature_limits(loops),
            "flip": False, "two_sided": False, "up_side": "TOP"}


def _rectpoly(b):
    """A blocked/used rectangle {x,y,w,h} (sheet coords) -> closed polygon."""
    x, y, w, h = b["x"], b["y"], b["w"], b["h"]
    return [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]


def _decim(pts, tol=PREVIEW_TOL):
    """Min-distance decimation for the PLAN PREVIEW only (the dense ~2k-pt cutting
    polylines are overkill on screen and bloat the JSON). NC paths are untouched."""
    if len(pts) <= 3:
        return [(round(x, 2), round(y, 2)) for x, y in pts]
    t2 = tol*tol
    out = [pts[0]]
    for p in pts[1:-1]:
        dx = p[0]-out[-1][0]; dy = p[1]-out[-1][1]
        if dx*dx + dy*dy >= t2:
            out.append(p)
    out.append(pts[-1])
    return [(round(x, 2), round(y, 2)) for x, y in out]


def _wants_tab(part, tabs_mode):
    if tabs_mode == "on":
        return True
    if tabs_mode == "off":
        return False
    return part["maxdim"] < SMALL_PART_MAXDIM      # "auto"


def nest_and_emit(parts, thickness, prefix, sheet_w=SHEET_W, sheet_h=SHEET_H,
                  blocked=None, tabs_mode=None, backplot=True,
                  auto_part=False, part_cuts=None, grain_locked=False, swap_xy=False,
                  part_tool=None, border_mm=None, placements_in=None):
    """Nest prepared parts onto sheet(s) and write one Syntec program + manifest
    per sheet (named <prefix>.NC / <prefix>_n.NC), plus a layout preview.

    `blocked` = list of {x,y,w,h} rectangles (sheet coords) marking already-used
    areas of the (remnant) sheet; parts are kept clear of them. Applies to the
    first sheet only (overflow opens fresh full sheets).
    `tabs_mode` = "auto"|"on"|"off" (defaults to module TABS_MODE).
    `auto_part` = add a straight parting cut ACROSS the full width just past the
    last part, freeing the machined strip from the clean remainder of the sheet.
    The nest grows up the sheet, so the cut is horizontal.
    `part_cuts` = manual parting-cut lines [[(x1,y1),(x2,y2)], ...] (sheet coords, sheet 1).
    `backplot` = write the matplotlib preview PNG (off for the web path — it only
    needs the layout JSON, and the plot is the bulk of the runtime).
    `border_mm` = clear band to leave between the outermost TOOLPATH and the sheet
    edge (defaults to CORNER_MARGIN). The placer seats keep-outs, which stand one
    profile-radius + MIN_GAP/2 proud of the outline while the toolpath runs one
    profile-radius out — so seating keep-outs at `border - MIN_GAP/2` puts the
    toolpath exactly `border` from the edge. See the CORNER_MARGIN note above.
    `placements_in` = operator-dragged positions, {uid: {"x","y","rot"}} in sheet
    mm, where uid is "<part name>#<copy index>" — a part with qty 2 puts two
    identically-named copies on the sheet, so the NAME alone cannot say which one
    was dragged. A pinned copy is placed there instead of being auto-placed;
    anything not pinned still nests around the pinned copies. Positions are
    checked by the caller AND re-checked here — a pin that would overlap a
    neighbour, or fall outside the border, is refused and the copy is auto-placed
    instead, with the refusal reported in the layout."""
    tabs_mode = tabs_mode or TABS_MODE
    blocked = blocked or []
    manual_cuts = part_cuts or []
    pins = placements_in or {}
    block_polys = [(_rectpoly(b), bbox(_rectpoly(b))) for b in blocked]
    parts = sorted(parts, key=lambda p: p["area"], reverse=True)   # big first
    rotations = (0, 2) if grain_locked else (0, 1, 2, 3)   # grain-locked stock keeps long axis fixed
    # The border is what the operator measures with a rule: sheet edge to PART edge.
    # A keep-out stands one profile radius + MIN_GAP/2 proud of the part it wraps, so
    # seating keep-outs at (border - R - MIN_GAP/2) lands the part outline exactly on
    # the border. The CUT then runs one radius further out, and still has to stay on
    # the sheet — so the border is never allowed to squeeze the toolpath closer to the
    # edge than CORNER_MARGIN. Passing nothing reproduces the old 1mm behaviour
    # exactly: border 1 -> eff 4 -> edge -5.
    border = CORNER_MARGIN if border_mm is None else max(0.0, float(border_mm))
    eff = max(border, CORNER_MARGIN + R)
    edge = eff - R - MIN_GAP / 2.0
    innerW, innerH = sheet_w - 2*edge, sheet_h - 2*edge
    # A part with qty 2 appears in `parts` twice — as the SAME dict object, so neither
    # its name nor its identity distinguishes the copies. Number them, and let the
    # layout and any pin refer to a copy by that uid.
    seen = {}
    uids = []
    for p in parts:
        n = seen.get(p["name"], 0); seen[p["name"]] = n + 1
        uids.append(f"{p['name']}#{n}")
    sheets = [[]]; placedpolys = [list(block_polys)]   # seed sheet 1 with used areas
    sheet_uids = [[]]
    pin_refused = []
    # Pinned copies are seated FIRST, on sheet 1, so the auto-nest fills around them
    # rather than the other way round.
    pinned_ok = set()
    for part, uid in zip(parts, uids):
        pin = pins.get(uid)
        if pin is None:
            continue
        k = int(pin.get("rot", 0)) // 90 % 4
        if grain_locked and k % 2:
            pin_refused.append(f"{part['name']}: 90° placement refused (grain-locked sheet)")
            continue
        rk = rot(part["keepout"], k)
        # The pin names where the part's OUTLINE min-corner goes — that is the edge the
        # operator dragged and can see. The keep-out stands one profile radius + half
        # the min gap proud of it, so anchoring on the keep-out instead would drop the
        # part that margin (9mm for a Ø6 profile tool) away from where it was dropped.
        fp = rot(part.get("footprint") or part["keepout"], k)
        fminx, fminy, _fmaxx, _fmaxy = bbox(fp)
        dx = float(pin.get("x", 0.0)) - fminx; dy = float(pin.get("y", 0.0)) - fminy
        cand = [(x+dx, y+dy) for x, y in rk]
        cb = bbox(cand)
        if cb[0] < edge - 1e-6 or cb[1] < edge - 1e-6 or \
           cb[2] > sheet_w - edge + 1e-6 or cb[3] > sheet_h - edge + 1e-6:
            pin_refused.append(f"{part['name']}: dragged outside the {border:g}mm border")
            continue
        if any(_intersects_sc(sc(cand), sc(pk)) for pk, _pb in placedpolys[0]):
            pin_refused.append(f"{part['name']}: dragged into another part")
            continue
        sheets[0].append((part, k, dx, dy)); placedpolys[0].append((cand, cb))
        sheet_uids[0].append(uid); pinned_ok.add(uid)
    for part, uid in zip(parts, uids):
        if uid in pinned_ok:
            continue
        done = False
        for si in range(len(sheets)):
            res = place_on_sheet(part["keepout"], placedpolys[si], innerW, innerH,
                                 rotations, edge=edge)
            if res:
                k, dx, dy, cand, cb = res
                sheets[si].append((part, k, dx, dy)); placedpolys[si].append((cand, cb))
                sheet_uids[si].append(uid)
                done = True; break
        if not done:
            res = place_on_sheet(part["keepout"], [], innerW, innerH, rotations, edge=edge)
            if not res:
                print(f"  !! {part['name']} too big for sheet, skipped"); continue
            sheets.append([]); placedpolys.append([]); sheet_uids.append([])
            k, dx, dy, cand, cb = res
            sheets[-1].append((part, k, dx, dy)); placedpolys[-1].append((cand, cb))
            sheet_uids[-1].append(uid)

    clear = thickness + MAT["clearance_over"]
    npasses = len(depth_passes(thickness))
    sheet_area = sheet_w * sheet_h

    col = {"OUTSIDE": "#00836A", "INSIDE": "#E08000", "DRILL": "#C00000",
           "POCKET": "#8030c0", "POCKET_BATCH": "#8030c0", "POCKET_ISL": "#8030c0",
           "HOLE_CLEAR": "#C00000", "PLUNGE": "#C00000",
           "OUTSIDE_PRE": "#00836A", "CHAMFER": "#a08000", "CSINK": "#a08000",
           "SURFACE": "#b0a000"}
    if backplot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, axes = plt.subplots(len(sheets), 1, figsize=(16, 8*len(sheets)))
        if len(sheets) == 1: axes = [axes]
    out_files = []

    for si, placements in enumerate(sheets):
        used = 0.0; tabset = set(); all_jobs = []; sheet_maxy = 0.0
        # Machine frame (swap_xy): the 90° emit-rotation maps design-x to machine-Y as
        # (sheet_w - x), so a nest shorter than the sheet would leave a gap at machine
        # Y=0. Slide the whole nest to the design RIGHT edge first, so after the rotation
        # it sits hard in the machine X0/Y0 corner. Pure translation -> no mirroring, and
        # the layout transposes the same coords so the preview matches the .NC.
        if swap_xy and placements:
            xs = [dx + x for part, k, dx, dy in placements
                  for j in part["jobs"] if j[0] == "OUTSIDE"
                  for x, _ in rot(j[1], k)]
            if not xs:   # pockets-only: no profile job -> use the (uncut) part footprint extent
                xs = [dx + x for part, k, dx, dy in placements
                      for x, _ in rot(part.get("footprint") or [], k)]
            if xs:
                shift = sheet_w - max(xs) - (eff - R)
                placements = [(part, k, dx + shift, dy) for part, k, dx, dy in placements]
        manifest = {"sheet": si+1, "sheet_w": sheet_w, "sheet_h": sheet_h,
                    "thickness_mm": round(thickness, 2), "cutter_dia_mm": 2*R,
                    "spindle_rpm": MAT["spindle"], "cut_feed": MAT["cut_feed"],
                    "plunge_feed": MAT["plunge_feed"], "passes": npasses,
                    "tabs_mode": tabs_mode, "parts": []}
        # plan-view layout for the UI: sheet, used areas, and every placed part's
        # transformed outline / cutouts / pockets / drill points + tab centres (mm).
        layout = {"sheet_w": sheet_w, "sheet_h": sheet_h, "edge": edge,
                  # `border` is the drawn line — sheet edge to part edge. `cut_border`
                  # is where the toolpath actually runs, one profile radius outside it.
                  "border": eff, "cut_border": round(eff - R, 2),
                  "bed_w": BED_W, "bed_h": BED_H,
                  "tabs_mode": tabs_mode,
                  "blocked": [dict(b) for b in (blocked if si == 0 else [])],
                  "pin_refused": pin_refused if si == 0 else [],
                  "parts": []}
        if backplot:
            ax = axes[si]
            ax.add_patch(plt.Rectangle((0, 0), sheet_w, sheet_h, fill=False, ec="#333", lw=1.5))
            for b in (blocked if si == 0 else []):
                ax.add_patch(plt.Rectangle((b["x"], b["y"]), b["w"], b["h"],
                                           facecolor="#bbb", edgecolor="#777", hatch="//", alpha=0.5))
        all_jobs2 = []          # flip-jig second pass: far-face recesses only
        for _pi, (part, k, dx, dy) in enumerate(placements):
            used += part.get("poly_area", part["area"])   # true footprint, not bbox
            want_tab = _wants_tab(part, tabs_mode)
            if want_tab: tabset.add(part["name"])
            lp = {"name": part["name"], "uid": sheet_uids[si][_pi],
                  "rotation_deg": k*90, "tabbed": want_tab,
                  "flip": part.get("flip", False), "two_sided": part.get("two_sided", False),
                  "up_side": part.get("up_side", "TOP"), "engrave_text": part.get("engrave_text", ""),
                  "outline": [], "cutouts": [], "pockets": [], "drills": [], "tabs": [],
                  "engrave": [], "toolpath": [],
                  "origin": [round(dx, 2), round(dy, 2)]}
            if part.get("mould"):
                lp["mould"] = True
            # The mould's pass-2 jobs ride the SAME placement transform as its
            # pass-1 cavity, so wherever this mould landed (or was dragged),
            # the flipped part's recesses are cut exactly on it.
            for role, path, pd, tool in part.get("pass2_jobs") or []:
                if role == "SURFACE":       # (x,y,z) polylines: rotate xy, keep z
                    tp2 = [[(rx+dx, ry+dy, q[2]) for (rx, ry), q in
                            zip(rot([(q[0], q[1]) for q in loop], k), loop)]
                           for loop in path]
                elif role in ("POCKET_ISL", "POCKET_BATCH"):
                    tp2 = [[(x+dx, y+dy) for x, y in rot(loop, k)] for loop in path]
                else:
                    tp2 = [(x+dx, y+dy) for x, y in rot(path, k)]
                all_jobs2.append((role, tp2, pd, tool, False))
            for role, path, pd, tool in part["jobs"]:
                if role == "SURFACE":       # (x,y,z) polylines: rotate xy, keep z
                    tp = [[(rx+dx, ry+dy, q[2]) for (rx, ry), q in
                           zip(rot([(q[0], q[1]) for q in loop], k), loop)]
                          for loop in path]
                elif role in ("POCKET_ISL", "POCKET_BATCH"):
                    tp = [[(x+dx, y+dy) for x, y in rot(loop, k)] for loop in path]
                else:
                    tp = [(x+dx, y+dy) for x, y in rot(path, k)]
                # Both profile laps are tabbed. The roughing lap already severs the
                # part from the sheet — it only leaves a skin on the WALL, not under
                # the part — so tabbing just the finishing lap would free every part
                # during roughing. prog.profile spaces tabs by fraction of perimeter,
                # so the two concentric laps put them in the same places.
                tab = want_tab and role in ("OUTSIDE", "OUTSIDE_FIN")
                all_jobs.append((role, tp, pd, tool, tab))
                # The jobs are tool-CENTRE paths. They are kept as a separate overlay so
                # the operator can still see the machine's actual route, but they are no
                # longer what draws the part — see the true geometry assembled below.
                if role in ("OUTSIDE", "OUTSIDE_FIN"):
                    lp["toolpath"].append(_decim(tp))
                    sheet_maxy = max(sheet_maxy, max(y for _x, y in tp))
                    if want_tab:
                        lp["tabs"] = [(round(x, 2), round(y, 2)) for x, y in tab_layout(tp)]
                elif role in ("INSIDE", "INSIDE_FIN", "POCKET"):
                    lp["toolpath"].append(_decim(tp))
                elif role == "ENGRAVE": lp["engrave"].append(_decim(tp))
                if backplot:
                    if role in ("POCKET_ISL", "POCKET_BATCH", "SURFACE"):
                        for loop in tp:
                            xs = [p[0] for p in loop] + [loop[0][0]]
                            ys = [p[1] for p in loop] + [loop[0][1]]
                            ax.plot(xs, ys, color=col[role], lw=0.7)
                        continue
                    xs = [p[0] for p in tp] + [tp[0][0]]; ys = [p[1] for p in tp] + [tp[0][1]]
                    if role == "PLUNGE":
                        ax.plot(tp[0][0], tp[0][1], "x", color=col[role], ms=6)
                    else:
                        ax.plot(xs, ys, color=col[role], lw=0.7)
            # ---- the part as MODELLED (what the preview draws) ----------------
            # Profile, cut-outs and holes all come from the loops, never from the jobs,
            # so every feature is drawn at its true size in one consistent space. Drawing
            # the OUTSIDE job here inflated every external radius by the cutter radius,
            # shrank every internal one by it, and collapsed anything tighter than the
            # cutter to a sharp corner — while pockets (already true geometry) were drawn
            # correctly alongside, so the same part was shown in two different spaces.
            if part.get("footprint"):
                lp["outline"] = _decim([(x + dx, y + dy) for x, y in rot(part["footprint"], k)])
            for ring in part.get("surface_regions", []):
                lp.setdefault("surface", []).append(
                    _decim([(x + dx, y + dy) for x, y in rot(ring, k)]))
            for ring in part.get("inner_outlines", []):
                lp["cutouts"].append(_decim([(x + dx, y + dy) for x, y in rot(ring, k)]))
            for cx, cy, r in part.get("hole_circles", []):
                rx, ry = rot([(cx, cy)], k)[0]
                lp["drills"].append((round(rx + dx, 2), round(ry + dy, 2), round(r, 2)))
            # pockets as shaded boundaries (not the spiral toolpath) + depth, for the plan
            for po in part.get("pocket_outlines", []):
                tpp = [(x+dx, y+dy) for x, y in rot(po["pts"], k)]
                isls = [_decim([(x+dx, y+dy) for x, y in rot(i, k)]) for i in (po.get("islands") or [])]
                lp["pockets"].append({"outline": _decim(tpp), "depth": po.get("depth"),
                                      "islands": isls, "through": bool(po.get("through"))})
            layout["parts"].append(lp)
            roles = [j[0] for j in part["jobs"]]
            manifest["parts"].append({
                "name": part["name"], "rotation_deg": k*90,
                "profiles": roles.count("OUTSIDE"), "inner_cutouts": roles.count("INSIDE"),
                "pockets": len(part.get("pocket_outlines", [])),
                "drilled_holes": sum(roles.count(r) for r in ("PLUNGE", "DRILL", "HOLE_CLEAR")),
                "tabs": want_tab})
        # parting cuts: manual lines (sheet 1) + an auto trim just past the last part
        cuts = [[(float(a), float(b)) for a, b in ln] for ln in (manual_cuts if si == 0 else [])]
        if auto_part and sheet_maxy > 0:
            yc = min(round(sheet_maxy + R + 6, 1), sheet_h - 2)
            cuts.append([(0.0, yc), (float(sheet_w), yc)])
        layout["part_cuts"] = [[(round(a, 1), round(b, 1)) for a, b in ln] for ln in cuts]
        if backplot:
            for ln in cuts:
                ax.plot([pp[0] for pp in ln], [pp[1] for pp in ln], color="#c00", lw=1.2, ls="--")
        # swap_xy: lay the nest into the machine box (X=1220 short, Y=2440 long) via a
        # 90 ROTATION (x,y)->(y, sheet_w-x) — NOT a transpose. A transpose reflects the
        # parts (engraving/features come out MIRRORED, on the wrong side); a rotation
        # preserves handedness so parts/text are correct, just reoriented. The arc-fitter
        # runs on the rotated points, so G02/G03 + I/J stay right. The PREVIEW (layout)
        # stays in design coords; PlanView applies the same rotation for display.
        def _rot(x, y): return (y, sheet_w - x)
        emit_jobs, emit_cuts = all_jobs, cuts
        if swap_xy:
            def _xf(role, path):
                if role == "SURFACE":
                    return [[(*_rot(q[0], q[1]), q[2]) for q in loop] for loop in path]
                if role in ("POCKET_ISL", "POCKET_BATCH"):
                    return [[_rot(x, y) for x, y in loop] for loop in path]
                return [_rot(x, y) for x, y in path]
            emit_jobs = [(role, _xf(role, path), pd, tool, tab)
                         for role, path, pd, tool, tab in all_jobs]
            emit_cuts = [[_rot(x, y) for x, y in ln] for ln in cuts]
        prog = Prog()
        tools_used = emit_sheet(prog, emit_jobs, clear, thickness, part_cuts=emit_cuts, part_tool=part_tool)
        manifest["swap_xy"] = swap_xy
        manifest["tools"] = [f"T0{t} Ø{TOOLS[t]['dia']:.0f}mm" for t in tools_used]
        util = used / sheet_area * 100
        layout["util_pct"] = round(util, 1)
        manifest["util_pct"] = round(util, 1)
        fl_agg = {}                                  # tightest feature limit across the sheet's parts
        for part, *_ in placements:
            for kf, v in (part.get("feature_limits") or {}).items():
                if v is not None:
                    fl_agg[kf] = min(fl_agg.get(kf, v), v)
        layout["feature_limits"] = fl_agg
        layout["two_pass"] = bool(all_jobs2)
        manifest["layout"] = layout
        fn = OUT / (f"{prefix}_{si+1}.NC" if len(sheets) > 1 else f"{prefix}.NC")
        fn.write_text(prog.dumps())
        fn.with_suffix(".json").write_text(json.dumps(manifest, indent=2))
        out_files.append(fn)
        # Flip-jig SECOND PASS: its own program, run after the operator drops the
        # flipped part(s) into the mould cavity/cavities. Only the far face's
        # recesses — everything through was cut in pass 1 — with the same stock
        # top (the part sits through the cavity on the spoilboard, flush with
        # the sheet). No parting cuts, no tabs: the sheet must not move.
        if not all_jobs2:
            # A re-generate must never resurrect a previous run's second pass.
            stale = fn.with_name(fn.stem + "_SIDE2.NC")
            stale.unlink(missing_ok=True)
            stale.with_suffix(".json").unlink(missing_ok=True)
        else:
            emit_jobs2 = all_jobs2
            if swap_xy:
                emit_jobs2 = [(role, _xf(role, path), pd, tool, tab)
                              for role, path, pd, tool, tab in all_jobs2]
            prog2 = Prog()
            tools2 = emit_sheet(prog2, emit_jobs2, clear, thickness)
            man2 = dict(manifest)
            man2.pop("layout", None)
            man2["side2"] = True
            man2["tools"] = [f"T0{t} Ø{TOOLS[t]['dia']:.0f}mm" for t in tools2]
            man2["parts"] = [
                {"name": part["name"], "rotation_deg": k*90, "profiles": 0,
                 "inner_cutouts": 0,
                 "pockets": len(part.get("pocket_outlines") or []),
                 "drilled_holes": 0, "tabs": False}
                for part, k, _dx, _dy in placements if part.get("mould")]
            fn2 = fn.with_name(fn.stem + "_SIDE2.NC")
            fn2.write_text(prog2.dumps())
            fn2.with_suffix(".json").write_text(json.dumps(man2, indent=2))
            print(f"  sheet {si+1}: flip-jig second pass -> {fn2.name}")
        print(f"  sheet {si+1}: {len(placements)} parts, util {util:.1f}%  -> {fn.name}"
              + (f"   tabs: {sorted(tabset)}" if tabset else ""))
        if backplot:
            ax.set_aspect("equal"); ax.set_xlim(-30, sheet_w+30); ax.set_ylim(-30, sheet_h+30)
            ax.set_title(f"Sheet {si+1}/{len(sheets)} — {len(placements)} parts — util {util:.1f}%")
            ax.grid(True, alpha=0.2)

    if backplot:
        png = OUT / f"_{prefix}_preview.png"; fig.tight_layout(); fig.savefig(png, dpi=110)
        print("backplot ->", png)
    return out_files


# NOTE: this module's Onshape CLI driver (main()) was removed when the engine
# moved into the fab exporter. Geometry now arrives already extracted, from the
# exporter's STEP reader. Nothing else in this file touched Onshape.
