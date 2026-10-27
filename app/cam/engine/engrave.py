"""Single-line (stick) text engraving for part marking.

Converts a part-number string into open stroke polylines using a built-in
single-line font (no fill needed — ideal for a V-bit), and places the label in
the largest clear rectangle inside the part outline (clear of holes / pockets /
cut-outs) so it's always visible on the finished piece and never in a cut-out.

Public API:
  text_strokes(s, x, y, h)        -> [stroke,...] positioned at (x,y), cap-height h mm
  place_label(loops, text, ...)   -> [stroke,...] auto-placed in the part's clear area
                                     (in the part's own 2D frame), or None if no room
"""
import math
import pyclipper

SCALE = 1000.0

# ---- single-line font: glyphs on a 0..5 x, 0..7 y cell (baseline y=0, cap y=7) ----
F = {
    " ": [],
    "-": [[(0, 3.5), (4, 3.5)]],
    ".": [[(1.7, 0), (2.3, 0)]],
    "/": [[(0, 0), (4, 7)]],
    "0": [[(1, 0), (3, 0), (4, 1), (4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0)]],
    "1": [[(1, 5), (2, 7), (2, 0)], [(0, 0), (4, 0)]],
    "2": [[(0, 6), (1, 7), (3, 7), (4, 6), (4, 5), (0, 1), (0, 0), (4, 0)]],
    "3": [[(0, 7), (4, 7), (2, 4)], [(2, 4), (4, 3), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "4": [[(3, 0), (3, 7), (0, 2.5), (4, 2.5)]],
    "5": [[(4, 7), (0, 7), (0, 4), (3, 4), (4, 3), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "6": [[(4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0), (3, 0), (4, 1), (4, 3), (3, 4), (0, 4)]],
    "7": [[(0, 7), (4, 7), (1, 0)]],
    "8": [[(1, 4), (0, 5), (0, 6), (1, 7), (3, 7), (4, 6), (4, 5), (3, 4), (1, 4),
           (0, 3), (0, 1), (1, 0), (3, 0), (4, 1), (4, 3), (3, 4)]],
    "9": [[(0, 1), (1, 0), (3, 0), (4, 1), (4, 6), (3, 7), (1, 7), (0, 6), (0, 4), (1, 3), (4, 3)]],
    "A": [[(0, 0), (2, 7), (4, 0)], [(0.7, 2.5), (3.3, 2.5)]],
    "B": [[(0, 0), (0, 7), (3, 7), (4, 6), (4, 5), (3, 4), (0, 4)], [(3, 4), (4, 3), (4, 1), (3, 0), (0, 0)]],
    "C": [[(4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0), (3, 0), (4, 1)]],
    "D": [[(0, 0), (0, 7), (3, 7), (4, 6), (4, 1), (3, 0), (0, 0)]],
    "E": [[(4, 7), (0, 7), (0, 0), (4, 0)], [(0, 3.5), (3, 3.5)]],
    "F": [[(4, 7), (0, 7), (0, 0)], [(0, 3.5), (3, 3.5)]],
    "G": [[(4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0), (3, 0), (4, 1), (4, 3), (2, 3)]],
    "H": [[(0, 0), (0, 7)], [(4, 0), (4, 7)], [(0, 3.5), (4, 3.5)]],
    "I": [[(1, 7), (3, 7)], [(2, 7), (2, 0)], [(1, 0), (3, 0)]],
    "J": [[(4, 7), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "K": [[(0, 0), (0, 7)], [(4, 7), (0, 3.5), (4, 0)]],
    "L": [[(0, 7), (0, 0), (4, 0)]],
    "M": [[(0, 0), (0, 7), (2, 4), (4, 7), (4, 0)]],
    "N": [[(0, 0), (0, 7), (4, 0), (4, 7)]],
    "O": [[(1, 0), (3, 0), (4, 1), (4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0)]],
    "P": [[(0, 0), (0, 7), (3, 7), (4, 6), (4, 5), (3, 4), (0, 4)]],
    "Q": [[(1, 0), (3, 0), (4, 1), (4, 6), (3, 7), (1, 7), (0, 6), (0, 1), (1, 0)], [(2.5, 2), (4, 0)]],
    "R": [[(0, 0), (0, 7), (3, 7), (4, 6), (4, 5), (3, 4), (0, 4)], [(2, 4), (4, 0)]],
    "S": [[(4, 6), (3, 7), (1, 7), (0, 6), (0, 5), (1, 4), (3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "T": [[(0, 7), (4, 7)], [(2, 7), (2, 0)]],
    "U": [[(0, 7), (0, 1), (1, 0), (3, 0), (4, 1), (4, 7)]],
    "V": [[(0, 7), (2, 0), (4, 7)]],
    "W": [[(0, 7), (1, 0), (2, 4), (3, 0), (4, 7)]],
    "X": [[(0, 0), (4, 7)], [(0, 7), (4, 0)]],
    "Y": [[(0, 7), (2, 3.5), (4, 7)], [(2, 3.5), (2, 0)]],
    "Z": [[(0, 7), (4, 7), (0, 0), (4, 0)]],
}
ADV = {" ": 4.0, ".": 2.5, "I": 3.0}      # per-char advance (font units); default 5
DEF_ADV = 5.0
CAP = 7.0


def _adv(ch):
    return ADV.get(ch.upper(), DEF_ADV)


def text_width(s, h):
    sc = h / CAP
    return sum(_adv(c) for c in s) * sc


def text_strokes(s, x, y, h):
    """Open stroke polylines (mm) for string s, baseline at y, left edge at x,
    cap-height h mm."""
    sc = h / CAP
    penx = x
    out = []
    for ch in s:
        g = F.get(ch.upper())
        if g:
            for st in g:
                out.append([(penx + px*sc, y + py*sc) for px, py in st])
        penx += _adv(ch) * sc
    return out


# ---- placement: largest clear rectangle inside the outline, clear of features ----
def _offset(poly, delta):
    path = [(int(round(x*SCALE)), int(round(y*SCALE))) for x, y in poly]
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(path, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    return [[(x/SCALE, y/SCALE) for x, y in p] for p in pco.Execute(delta*SCALE)]


def _pip(x, y, poly):
    c = False; n = len(poly); j = n-1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj-xi)*(y-yi)/(yj-yi+1e-12) + xi):
            c = not c
        j = i
    return c


def _largest_rect(grid):
    """Largest all-True axis-aligned rectangle (histogram method).
    Returns (r0, c0, wcells, hcells)."""
    gh = len(grid); gw = len(grid[0]) if gh else 0
    hist = [0]*gw; best = (0, 0, 0, 0, 0)   # area, r0, c0, w, h
    for r in range(gh):
        for c in range(gw):
            hist[c] = hist[c] + 1 if grid[r][c] else 0
        stack = []; c = 0
        while c <= gw:
            cur = hist[c] if c < gw else 0
            start = c
            while stack and stack[-1][1] > cur:
                sc, sh = stack.pop()
                area = sh * (c - sc)
                if area > best[0]:
                    best = (area, r - sh + 1, sc, c - sc, sh)
                start = sc
            stack.append((start, cur)); c += 1
    return best[1], best[2], best[3], best[4]


def clear_rect(loops, margin=5.0, res=1.5):
    """The largest clear rectangle inside the part's OUTSIDE loop, eroded by
    `margin` and clear of every INSIDE/POCKET (and DRILL circle) inflated by
    `margin`. Returns (x, y, w, h) mm, or None."""
    outs = [lp["pts"] for lp in loops if lp["role"] == "OUTSIDE"]
    if not outs:
        return None
    outline = max(outs, key=lambda p: abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(p, p[1:]+p[:1]))))
    inner = _offset(outline, -margin)
    inner = max(inner, key=len) if inner else None
    if not inner:
        return None
    blockers = []
    for lp in loops:
        if lp["role"] in ("INSIDE", "POCKET"):
            inf = _offset(lp["pts"], margin)
            if inf:
                blockers.append(max(inf, key=len))
        if lp.get("circle"):
            cx, cy, r = lp["circle"]; blockers.append(("circ", cx, cy, r + margin))

    xs = [p[0] for p in inner]; ys = [p[1] for p in inner]
    minx, maxx = min(xs), max(xs); miny, maxy = min(ys), max(ys)
    gw = max(1, int((maxx-minx)/res)); gh = max(1, int((maxy-miny)/res))
    grid = [[False]*gw for _ in range(gh)]
    for r in range(gh):
        cy = miny + (r+0.5)*res
        for c in range(gw):
            cx = minx + (c+0.5)*res
            if not _pip(cx, cy, inner):
                continue
            ok = True
            for b in blockers:
                if isinstance(b, tuple) and b[0] == "circ":
                    if (cx-b[1])**2 + (cy-b[2])**2 <= b[3]**2:
                        ok = False; break
                elif _pip(cx, cy, b):
                    ok = False; break
            grid[r][c] = ok
    r0, c0, wc, hc = _largest_rect(grid)
    if wc < 2 or hc < 2:
        return None
    return (minx + c0*res, miny + r0*res, wc*res, hc*res)


def dominant_angle(poly):
    """Length-weighted predominant edge orientation of a polygon, folded into
    (-45°, +45°] (radians). Axis-aligned parts give ~0; an angled part gives its
    tilt — so the engraved text can run parallel to the part's main straight edges.
    Folding by 4x makes perpendicular edges (a rectangle's two edge families) agree."""
    sx = sy = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 1e-9:
            continue
        a = math.atan2(dy, dx)
        sx += L * math.cos(4 * a); sy += L * math.sin(4 * a)
    if sx == 0 and sy == 0:
        return 0.0
    return math.atan2(sy, sx) / 4.0


def _rotate_loops(loops, ang, cx, cy):
    """Copy of loops with every point (and circle centre) rotated by `ang` about (cx,cy)."""
    c, s = math.cos(ang), math.sin(ang)
    def rp(x, y):
        dx, dy = x - cx, y - cy
        return (cx + dx*c - dy*s, cy + dx*s + dy*c)
    out = []
    for lp in loops:
        nl = dict(lp)
        nl["pts"] = [rp(x, y) for x, y in lp["pts"]]
        if lp.get("circle"):
            ccx, ccy, r = lp["circle"]; nx, ny = rp(ccx, ccy); nl["circle"] = (nx, ny, r)
        out.append(nl)
    return out


def _place_axis(loops, text, max_h, min_h, margin, pad):
    """Axis-aligned placement: largest clear rect, best of horizontal / 90°-rotated."""
    rect = clear_rect(loops, margin=margin)
    if not rect:
        return None
    x, y, w, h = rect
    cx, cy = x + w/2, y + h/2

    def fit_height(box_w, box_h):
        th = min(max_h, box_h - 2*pad)
        if th <= 0:
            return 0.0
        tw = text_width(text, th)
        if tw > box_w - 2*pad:
            th *= (box_w - 2*pad) / tw
        return th

    th_h = fit_height(w, h)     # horizontal: text width along w
    th_v = fit_height(h, w)     # vertical: text width along h (rotated 90°)
    if max(th_h, th_v) < min_h:
        return None
    if th_h >= th_v:
        tw = text_width(text, th_h)
        return text_strokes(text, cx - tw/2, cy - th_h/2, th_h)
    # vertical: centred horizontal strokes rotated 90° CCW about the centre
    tw = text_width(text, th_v)
    base = text_strokes(text, cx - tw/2, cy - th_v/2, th_v)
    return [[(cx - (py - cy), cy + (px - cx)) for px, py in st] for st in base]


def place_label(loops, text, max_h=25.0, min_h=3.0, margin=4.0, pad=1.0, offset=(0.0, 0.0)):
    """Auto-place `text` in the part's largest clear rectangle, ALIGNED to the part's
    predominant straight edges (so it reads square to the part, not the sheet axes).
    `offset` (ox, oy) nudges the placed label in the part's own 2D frame — the operator
    can drag it in the plan view, and the nudge re-applies on every re-nest. Returns
    stroke polylines (mm, part frame) or None if there's no room for legible text."""
    outs = [lp["pts"] for lp in loops if lp["role"] == "OUTSIDE"]
    if not outs:
        return None
    outline = max(outs, key=lambda p: abs(sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(p, p[1:]+p[:1]))))
    th = dominant_angle(outline)
    cx = sum(p[0] for p in outline) / len(outline)
    cy = sum(p[1] for p in outline) / len(outline)
    aligned = abs(th) > math.radians(0.5)
    work = _rotate_loops(loops, -th, cx, cy) if aligned else loops
    strokes = _place_axis(work, text, max_h, min_h, margin, pad)
    if strokes is None:
        return None
    if aligned:                # rotate the placed text back so it sits on the part's edges
        c, s = math.cos(th), math.sin(th)
        strokes = [[(cx + (px-cx)*c - (py-cy)*s, cy + (px-cx)*s + (py-cy)*c) for px, py in st] for st in strokes]
    ox, oy = (offset or (0.0, 0.0))
    if ox or oy:
        strokes = [[(px + ox, py + oy) for px, py in st] for st in strokes]
    return strokes


def label_text(name, part_number=None):
    """Engraving string: Part Number when set, else the name with the instance
    suffix '<n>' stripped, upper-cased to the marking charset."""
    s = (part_number or "").strip()
    if not s:
        s = name.split("<")[0].strip()
    return s.upper()
