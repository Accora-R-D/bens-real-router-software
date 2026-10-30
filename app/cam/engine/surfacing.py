"""3-axis surfacing toolpaths from a Z-map heightfield (Phase 2a — see SURFACING_PLAN.md).

This is the drop-cutter FINISHING core. It consumes the Z-map + machining frame
produced by surface_extract and emits ordered (x,y,z) toolpath polylines in the
machining frame (mm), ready for the Syntec post (Phase 3) to turn into G-code.

Why a Z-map drop-cutter (not OCL yet): it is exact for a ball nose, runs on plain
numpy (no Python-3.11/OCL dependency, so it is testable today), parallelises on the
22-core box, and is the agreed validation oracle for OCL. OCL's mesh Waterline will
slot in behind the same `*_finish(...) -> [polyline]` interface for steep walls.

Core idea — the DROP-CUTTER FIELD. For a cutter centred at (x,y) with its tip at
height Ztip, a point on the cutter at planar offset d from the axis sits prof(d)
above the tip. No-gouge requires Ztip + prof(d) >= surface everywhere under the
tool, so the tool rests at Ztip = max over the footprint of (surface - prof(d)).
Computing that for every cell at once is a grayscale dilation of the Z-map by the
(negated) cutter profile — one vectorised pass. Any scan pattern then just reads
this field along its route.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

import numpy as np


# ----------------------------- cutter model -----------------------------
@dataclass
class Cutter:
    kind: str               # 'ball' | 'flat' | 'bull'
    dia: float              # mm
    corner: float = 0.0     # mm — bull-nose corner radius (ignored for ball/flat)
    length: float = 25.0    # mm

    @property
    def R(self):
        return self.dia / 2.0


def scallop_for_stepover(R, s):
    """Cusp height left between adjacent ball passes at planar stepover s."""
    return R - math.sqrt(max(R * R - (s / 2.0) ** 2, 0.0))


def stepover_for_scallop(R, h):
    """Planar stepover that yields a target scallop height h (h clamped to R)."""
    h = min(max(h, 0.0), R)
    return 2.0 * math.sqrt(max(2 * R * h - h * h, 0.0))


def cutter_kernel(cutter: Cutter, res):
    """Footprint offsets (di, dj in cells) + prof (mm above the tip) for the cutter.
    ball:  prof = R - sqrt(R^2 - d^2);  flat: 0 inside R;  bull: flat to (R-r) then
    a corner-radius arc."""
    R = cutter.R
    rad = int(math.ceil(R / res))
    ii = np.arange(-rad, rad + 1)
    di, dj = np.meshgrid(ii, ii, indexing="ij")
    d = np.hypot(di, dj) * res
    inside = d <= R + 1e-9
    if cutter.kind == "flat":
        prof = np.zeros_like(d)
    elif cutter.kind == "ball":
        prof = R - np.sqrt(np.maximum(R * R - d * d, 0.0))
    elif cutter.kind == "bull":
        r = max(min(cutter.corner, R), 1e-6)
        flatR = R - r
        prof = np.where(d <= flatR, 0.0,
                        r - np.sqrt(np.maximum(r * r - (d - flatR) ** 2, 0.0)))
    else:
        raise ValueError(f"unknown cutter kind {cutter.kind!r}")
    return di[inside].ravel(), dj[inside].ravel(), prof[inside].ravel().astype(np.float64)


def _shift(S, di, dj, fill=-np.inf):
    """Return A with A[i,j] = S[i+di, j+dj], out-of-bounds filled with `fill`."""
    H, W = S.shape
    out = np.full_like(S, fill)
    rs = slice(max(0, di), min(H, H + di)); rd = slice(max(0, -di), min(H, H - di))
    cs = slice(max(0, dj), min(W, W + dj)); cd = slice(max(0, -dj), min(W, W - dj))
    out[rd, cd] = S[rs, cs]
    return out


def _max_filter_square(S, rad):
    """Fast conservative max filter for flat-tool roughing.

    A square circumscribes the real circular cutter footprint, so this may leave a
    little extra roughing stock near diagonal walls but can never create a gouge.
    Two separable 1-D passes reduce work from O(rad^2*N) to O(rad*N), which matters
    for full-size furniture formers with several million Z-map cells.
    """
    if rad <= 0:
        return S.copy()
    h, w = S.shape
    tmp = np.full_like(S, -np.inf)
    for d in range(-rad, rad + 1):
        dst = slice(max(0, -d), min(w, w - d))
        src = slice(max(0, d), min(w, w + d))
        np.maximum(tmp[:, dst], S[:, src], out=tmp[:, dst])
    out = np.full_like(S, -np.inf)
    for d in range(-rad, rad + 1):
        dst = slice(max(0, -d), min(h, h - d))
        src = slice(max(0, d), min(h, h + d))
        np.maximum(out[dst, :], tmp[src, :], out=out[dst, :])
    return out


def drop_field(zmap, cutter: Cutter, res):
    """The tip-Z the cutter rests at for EVERY centre cell (grayscale dilation of
    the Z-map by the negated cutter profile). NaN where the whole footprint is void.
    No-gouge is guaranteed by construction (Ztip = max(surface - prof)).

    Slice-based in-place max over the kernel offsets — no per-offset full-array
    allocation, so it scales to fine grids without scipy."""
    S = np.where(np.isfinite(zmap), zmap, -np.inf).astype(np.float64)
    H, W = S.shape
    if cutter.kind == "flat":
        out = _max_filter_square(S, int(math.ceil(cutter.R / res)))
        out[~np.isfinite(out)] = np.nan
        return out

    out = np.full(S.shape, -np.inf)
    di, dj, prof = cutter_kernel(cutter, res)
    for a, b, p in zip(di.tolist(), dj.tolist(), prof.tolist()):
        rd = slice(max(0, -a), min(H, H - a)); rs = slice(max(0, a), min(H, H + a))
        cd = slice(max(0, -b), min(W, W - b)); cs = slice(max(0, b), min(W, W + b))
        np.maximum(out[rd, cd], S[rs, cs] - p, out=out[rd, cd])
    out[~np.isfinite(out)] = np.nan
    return out


# ----------------------------- finishing strategies -----------------------------
def _runs(mask):
    """Yield (start, stop) index ranges of contiguous True in a 1-D bool array."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return
    splits = np.where(np.diff(idx) > 1)[0] + 1
    for grp in np.split(idx, splits):
        yield grp[0], grp[-1] + 1


def raster_finish(zmap, grid, cutter: Cutter, stepover=None, scallop=None,
                  along="x", min_pts=2, mask=None):
    """Parallel/raster finishing. Lanes run along `along` ('x' or 'y'), stepped by
    `stepover` (or derived from a target `scallop`). Returns
    (paths, Zdrop, info) where paths = list of (x,y,z) polylines (mm, machining
    frame), alternating direction (zig-zag), split at voids. info has the
    resolved stepover + resulting scallop + point count."""
    res = grid["res"]
    if stepover is None:
        stepover = stepover_for_scallop(cutter.R, scallop if scallop is not None else 0.05)
    Zdrop = drop_field(zmap, cutter, res)

    # work in a lane-major orientation: rows = lanes (step axis), cols = travel.
    # Lane positions are placed at EXACT `stepover` spacing (mm) — not snapped to the
    # grid — so the cusp pitch is accurate regardless of grid resolution. Z is read
    # from the nearest grid row.
    if along == "auto":
        # Travel along the longer dimension, minimising lane count and retracts.
        along = "x" if grid["nx"] >= grid["ny"] else "y"
    if along == "x":
        Z = Zdrop; ox, oy = grid["ox"], grid["oy"]; nrows = grid["ny"]
        M = mask
        def coord(col, ystep, z): return (ox + col * res, oy + ystep, z)
    elif along == "y":
        Z = Zdrop.T; ox, oy = grid["ox"], grid["oy"]; nrows = grid["nx"]
        M = mask.T if mask is not None else None
        def coord(col, ystep, z): return (ox + ystep, oy + col * res, z)
    else:
        raise ValueError("along must be 'x' or 'y'")

    length = (nrows - 1) * res
    paths, npts, li, nlanes = [], 0, 0, 0
    while li * stepover <= length + 1e-9:
        ystep = li * stepover
        row = min(nrows - 1, int(round(ystep / res)))
        z = Z[row, :]; nlanes += 1
        ok = np.isfinite(z)
        if M is not None:
            ok &= M[row, :]                   # lanes cut only the masked region
        for s, e in _runs(ok):
            seg = [coord(c, ystep, float(z[c])) for c in range(s, e)]
            if li % 2 == 1:
                seg.reverse()
            if len(seg) >= min_pts:
                paths.append(seg); npts += len(seg)
        li += 1
    info = {"stepover_mm": round(stepover, 4),
            "scallop_mm": round(scallop_for_stepover(cutter.R, stepover), 4),
            "lanes": nlanes, "paths": len(paths), "points": npts, "along": along,
            "cutter": {"kind": cutter.kind, "dia": cutter.dia, "corner": cutter.corner}}
    return paths, Zdrop, info


def raster_rough(zmap, grid, cutter: Cutter, *, stock_top, stepdown=3.0,
                 stepover=None, stock_to_leave=1.0, along="auto", ramp_angle_deg=5.0,
                 mask=None):
    """Layered projected roughing with ramp entry and no plunge into raw stock."""
    if cutter.kind not in ("flat", "bull"):
        raise ValueError("surface roughing requires a flat or bull-nose cutter")
    if stepdown <= 0 or stock_to_leave < 0:
        raise ValueError("stepdown must be > 0 and stock-to-leave must be >= 0")
    res = float(grid["res"])
    stepover = float(stepover if stepover is not None else cutter.dia * 0.4)
    if stepover <= 0 or stepover > cutter.dia:
        raise ValueError("roughing stepover must be > 0 and no greater than the cutter diameter")
    target = drop_field(zmap, cutter, res) + float(stock_to_leave)
    stock_top = float(stock_top)
    # Stock-to-leave can push cells that already sit AT the stock top above it
    # (the STEP path pins a margin band around a sculpted region to stock top).
    # There is no stock above the stock — clip, so those cells simply never cut.
    np.minimum(target, stock_top, out=target)
    finite = target[np.isfinite(target)]
    if finite.size == 0:
        raise ValueError("surface contains no machinable heightfield cells")

    if along == "auto":
        along = "x" if grid["nx"] >= grid["ny"] else "y"
    if along == "x":
        Z = target; travel_o, step_o = grid["ox"], grid["oy"]
        M = mask
        def point(c, lane, z): return (travel_o + c * res, step_o + lane, z)
    elif along == "y":
        Z = target.T; travel_o, step_o = grid["oy"], grid["ox"]
        M = mask.T if mask is not None else None
        def point(c, lane, z): return (step_o + lane, travel_o + c * res, z)
    else:
        raise ValueError("along must be 'auto', 'x' or 'y'")

    floor = float(finite.min())
    levels = []
    z = stock_top
    while z > floor + 1e-9:
        z = max(floor, z - float(stepdown))
        levels.append(z)
    lane_extent = (Z.shape[0] - 1) * res
    paths, points, linked = [], 0, 0
    prev = stock_top
    tan_ramp = math.tan(math.radians(max(1.0, min(15.0, ramp_angle_deg))))
    link_max = 2.0 * cutter.dia

    def link_ok(level, p, q):
        """Can the cutter go straight from cell p to cell q at `level` without
        gouging? The cell nearest every point of the line must let it sit at
        that height — the rule every lane is cut by, and the field already
        holds the cutter clear of the part by its radius (more, on diagonals:
        its kernel is the square round the cutter)."""
        n = max(2, int(math.ceil(math.dist(p, q) / (0.5 * res))) + 1)
        for k in range(n):
            t = k / (n - 1)
            r = int(round(p[0] + (q[0] - p[0]) * t))
            c = int(round(p[1] + (q[1] - p[1]) * t))
            if not (0 <= r < Z.shape[0] and 0 <= c < Z.shape[1]):
                return False
            zc = Z[r, c]
            if not np.isfinite(zc) or zc > level + 1e-9:
                return False
        return True

    for level in levels:
        # Every run on this level first, then chained: from the lowest run left,
        # step to the nearest run in a lane either side whenever the cutter can
        # get there at this level, so each island is one continuous zigzag with
        # one ramp. A raster that lifted and ramped for every run (the Mould:
        # 217 paths, each zigzagging down 6.2 mm at 5 deg, often over a 5 mm
        # run) spent more of its travel ramping than cutting.
        runs = []
        li = 0
        while li * stepover <= lane_extent + 1e-9:
            lane = li * stepover
            row = min(Z.shape[0] - 1, int(round(lane / res)))
            lane_ok = np.isfinite(Z[row]) & (Z[row] <= level + 1e-9)
            if M is not None:
                lane_ok &= M[row]             # rough only the masked region
            for s, e in _runs(lane_ok):
                if e - s >= 2:
                    runs.append((li, row, lane, s, e - 1))
            li += 1
        left = list(range(len(runs)))
        while left:
            k = left.pop(0)
            li, row, lane, s, e = runs[k]
            cols = list(range(s, e + 1))
            if len(paths) % 2:
                cols.reverse()
            a, b = cols[0], cols[-1]
            run_len = max(res, abs(b - a) * res)
            ramp_len = max(0.0, (prev - level) / tan_ramp)
            traversals = max(1, int(math.ceil(ramp_len / run_len)))
            path = []
            for t in range(traversals + 1):
                c = a if t % 2 == 0 else b
                frac = min(1.0, (t * run_len) / max(ramp_len, 1e-9))
                path.append(point(c, lane, prev + (level - prev) * frac))
            if path[-1][2] > level + 1e-9:
                c = b if path[-1][0:2] == point(a, lane, level)[0:2] else a
                path.append(point(c, lane, level))
            start_at_a = path[-1][0:2] == point(a, lane, level)[0:2]
            clear_cols = cols if start_at_a else list(reversed(cols))
            path.extend(point(c, lane, level) for c in clear_cols[1:])
            end = (row, clear_cols[-1])
            cur = li
            while True:
                best = None
                for j in left:
                    lj, rj, lanej, sj, ej = runs[j]
                    if abs(lj - cur) != 1:
                        continue
                    near, far = (sj, ej) if abs(sj - end[1]) <= abs(ej - end[1]) else (ej, sj)
                    d = math.dist((end[0] * res, end[1] * res), (rj * res, near * res))
                    if d <= link_max and (best is None or d < best[0]):
                        best = (d, j, near, far)
                if best is None or not link_ok(level, end, (runs[best[1]][1], best[2])):
                    break
                _d, j, near, far = best
                left.remove(j)
                lj, rj, lanej, _s, _e = runs[j]
                step = 1 if far >= near else -1
                path.extend(point(c, lanej, level) for c in range(near, far + step, step))
                linked += 1
                end, cur = (rj, far), lj
            paths.append(path); points += len(path)
        prev = level
    info = {"stepdown_mm": float(stepdown), "stepover_mm": round(stepover, 4),
            "stock_to_leave_mm": float(stock_to_leave), "levels": len(levels),
            "paths": len(paths), "points": points, "ramp_paths": len(paths),
            "linked_runs": linked,
            "along": along, "stock_top_mm": stock_top,
            "cutter": {"kind": cutter.kind, "dia": cutter.dia, "corner": cutter.corner}}
    return paths, target, info


def flat_area_mask(zmap, grid, frame, tol_deg=3.0):
    """Cells that are near-horizontal (good for a FLAT endmill, which leaves a
    crisper bottom than a ball). Gradient of the Z-map < tan(tol). Returns a bool
    grid aligned with zmap. (Hook for flat-tool finishing of the floor.)"""
    res = grid["res"]
    gy, gx = np.gradient(np.where(np.isfinite(zmap), zmap, np.nan), res)
    slope = np.hypot(gx, gy)
    return np.isfinite(slope) & (slope <= math.tan(math.radians(tol_deg)))


def estimate_nc_bytes(paths, bytes_per_block=26):
    """Rough Syntec .NC size: one G01 block per point (~26 bytes incl. N-number)."""
    return sum(len(p) for p in paths) * bytes_per_block
