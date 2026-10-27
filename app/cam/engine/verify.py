"""Safety gate for generated Syntec programs. Parses the .NC and runs rule
checks that must PASS before the file is allowed onto the USB stick.

Catches the real 2.5D-router failure modes:
  - rapid (G00) moving through material (Z below stock top while not clear)
  - cut depth gouging too far below the board (spoilboard damage)
  - a cut that never reaches through the board (part stays attached)
  - toolpath leaving the sheet bounds (crash into clamps/frame)
  - missing feed rate on a cutting move
"""
import sys, re, math, json
from pathlib import Path
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

TOL = 0.01

def _arc_pts(x, y, nx, ny, i, j, ccw, step=1.0):
    cx, cy = x+i, y+j
    r = math.hypot(x-cx, y-cy)
    a0 = math.atan2(y-cy, x-cx); a1 = math.atan2(ny-cy, nx-cx)
    sweep = (a1-a0) % (2*math.pi) if ccw else -((a0-a1) % (2*math.pi))
    if abs(sweep) < 1e-9:
        sweep = 2*math.pi if ccw else -2*math.pi
    nseg = max(2, int(abs(sweep)*r/step))
    return [(cx+r*math.cos(a0+sweep*k/nseg), cy+r*math.sin(a0+sweep*k/nseg)) for k in range(1, nseg+1)]


def parse(text):
    """Yield motion records: dict(g, x, y, z, f). Arcs (G02/G03) expanded to steps."""
    g = 0; x = y = z = 0.0; f = 0.0
    num = re.compile(r"([GXYZFIJ])(-?\d+\.?\d*)")
    for raw in text.splitlines():
        line = re.sub(r"^N\d+", "", raw.split(";")[0])
        if re.search(r"G53|G28", line):
            continue
        toks = dict((m.group(1), float(m.group(2))) for m in num.finditer(line))
        if not toks:
            continue
        if "G" in toks and toks["G"] in (0, 1, 2, 3):
            g = int(toks["G"])
        if "F" in toks: f = toks["F"]
        nx = toks.get("X", x); ny = toks.get("Y", y); nz = toks.get("Z", z)
        if g in (2, 3) and ("X" in toks or "Y" in toks):
            for qx, qy in _arc_pts(x, y, nx, ny, toks.get("I", 0.0), toks.get("J", 0.0), g == 3):
                yield {"g": g, "x": qx, "y": qy, "z": nz, "f": f, "src": raw.strip()}
            x, y, z = nx, ny, nz
            continue
        moved = any(k in toks for k in "XYZ")
        x, y, z = nx, ny, nz
        if moved:
            yield {"g": g, "x": x, "y": y, "z": z, "f": f, "src": raw.strip()}


def gouge_violations(recs, zmap, grid, cutter, tol_gouge=0.05):
    """Surface-mode gouge rule. `drop_field(zmap,cutter,res)` is the lowest tip-Z the
    cutter can sit at WITHOUT gouging (Ztip = max over the footprint of surface-prof);
    a finishing pass follows that field. A cutting point emitted BELOW it (minus tol)
    digs into the model = a gouge. Reuses the existing gouge-free oracle (no OCL).

    recs x/y/z must be in the SAME machining frame as `zmap` (grid = {res,ox,oy,...};
    zmap indexed [y_row, x_col], matching surfacing.raster_finish). Returns a list of
    human-readable violation strings (empty = clean)."""
    import surfacing
    import numpy as np
    res, ox, oy = grid["res"], grid["ox"], grid["oy"]
    Zdrop = surfacing.drop_field(zmap, cutter, res)
    ny, nx = Zdrop.shape
    out = []
    for r in recs:
        if r["g"] not in (1, 2, 3):
            continue
        j = int(round((r["x"] - ox) / res))   # column along x
        i = int(round((r["y"] - oy) / res))   # row along y
        if i < 0 or i >= ny or j < 0 or j >= nx:
            continue
        floor = Zdrop[i, j]
        if not np.isfinite(floor):
            continue
        if r["z"] < floor - tol_gouge:
            out.append(f"GOUGE at X{r['x']:.2f}Y{r['y']:.2f}: Z{r['z']:.3f} below "
                       f"gouge-free Z{float(floor):.3f}: {r.get('src','')}")
    return out


def check(path, stock_top, sheet_w, sheet_h, final_z_expected, clearance, expect_through=True,
          mode="router", zmap=None, grid=None, cutter=None, tol_gouge=0.05):
    """Mode-aware safety gate. mode='router' (default) is the FROZEN, byte-validated
    2.5D rule-set — identical verdicts to before. mode='metal' relaxes the cut-through
    requirement (blind pockets/plates) and treats sheet_w/h as the stock-box bounds.
    mode='surface' drops the cut-through rule entirely (finishing leaves material) and
    ADDS a drop-cutter gouge check when a zmap+grid+cutter are supplied."""
    text = Path(path).read_text()
    recs = list(parse(text))
    fails, warns = [], []
    if mode in ("metal", "surface"):
        expect_through = False   # blind pockets / 3D finishing never reach through

    min_cut_z = math.inf
    xs, ys = [], []
    clear = True       # tool starts retracted at machine home
    prev = None
    cutfloor = {}      # (round x, round y) -> deepest Z already cut there
    def key(r): return (round(r["x"]), round(r["y"]))
    for r in recs:
        moved_xy = prev is not None and (abs(prev["x"]-r["x"]) > TOL or abs(prev["y"]-r["y"]) > TOL)
        moved_z = prev is not None and abs(prev["z"]-r["z"]) > TOL
        if r["x"] or r["y"]:
            xs.append(r["x"]); ys.append(r["y"])
        cutting = r["g"] in (1, 2, 3)

        if r["g"] == 0:
            if moved_z and r["z"] >= stock_top - TOL:        # retract above stock => clear
                clear = True
            if moved_xy and not clear:                       # lateral rapid while plunged
                fails.append(f"RAPID through material at Z{r['z']:.2f}: {r['src']}")
            # rapid descending below stock is only safe into an already-cut hole:
            # flag only if it goes below the depth already cut at this XY (into solid).
            if moved_z and r["z"] < stock_top - TOL and r["z"] < cutfloor.get(key(r), math.inf) - TOL:
                fails.append(f"RAPID plunge into uncut material at Z{r['z']:.2f}: {r['src']}")
        if cutting:
            if r["z"] < stock_top - TOL:
                clear = False
            cutfloor[key(r)] = min(cutfloor.get(key(r), math.inf), r["z"])
            if r["z"] < min_cut_z:
                min_cut_z = r["z"]
            if r["f"] <= 0:
                fails.append(f"cutting move with no feed: {r['src']}")
            if r["z"] < final_z_expected - 1.0:
                fails.append(f"cut too deep Z{r['z']:.2f} (expected ~{final_z_expected}): {r['src']}")
        prev = r

    # did we cut through? deepest cut should reach <= ~0 (through the board). Skipped in
    # pockets-only mode (expect_through=False): blind pockets intentionally never reach
    # the board, so the part is meant to stay attached — flag it as a WARN, not a FAIL.
    if mode == "surface":
        pass   # 3D finishing intentionally leaves material — "cut through" is not a rule
    elif min_cut_z > 0.0 + TOL:
        if expect_through:
            fails.append(f"deepest cut Z{min_cut_z:.2f} > 0 — parts not cut through")
        else:
            warns.append(f"pockets-only: deepest cut Z{min_cut_z:.2f} — outer profile not cut, "
                         f"parts stay attached to the sheet")
    # sheet / stock-box bounds (skip when caller passes no bounds, e.g. a free surface op)
    if xs and sheet_w is not None and sheet_h is not None:
        if min(xs) < -TOL or max(xs) > sheet_w + TOL or min(ys) < -TOL or max(ys) > sheet_h + TOL:
            fails.append(f"toolpath leaves sheet: X[{min(xs):.1f},{max(xs):.1f}] "
                         f"Y[{min(ys):.1f},{max(ys):.1f}] vs {sheet_w}x{sheet_h}")
    # structural sanity
    if "M30" not in text:
        fails.append("no M30 program end")
    if not re.search(r"S\d+M3", text):
        warns.append("no spindle speed (S..M3) set")
    # surface gouge rule (only when a target surface field is supplied)
    if mode == "surface" and zmap is not None and grid is not None and cutter is not None:
        fails += gouge_violations(recs, zmap, grid, cutter, tol_gouge)

    n_rapid_clear = sum(1 for r in recs if r["g"] == 0 and abs(r["z"]-clearance) < 0.1)
    print(f"  {Path(path).name}: {len(recs)} moves, {n_rapid_clear} retracts, "
          f"deepest cut Z{min_cut_z:.2f}, X[{min(xs):.0f},{max(xs):.0f}] Y[{min(ys):.0f},{max(ys):.0f}]")
    for w in warns:
        print(f"    WARN  {w}")
    for fl in fails:
        print(f"    FAIL  {fl}")
    ok = not fails
    print(f"    => {'PASS' if ok else 'FAIL'}")
    return ok, fails, warns


if __name__ == "__main__":
    OUT = Path(__file__).parent / "out"
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT / "SHEET_18mm.NC"
    mf = target.with_suffix(".json")              # read stock/sheet from the manifest
    m = json.loads(mf.read_text()) if mf.exists() else {}
    stock = m.get("thickness_mm", 18.0)
    sw, sh = m.get("sheet_w", 2440.0), m.get("sheet_h", 1220.0)
    if m.get("swap_xy"):                  # NC emitted with X<->Y swapped -> swap the bounds
        sw, sh = sh, sw
    ok, _fails, _warns = check(target, stock_top=stock, sheet_w=sw, sheet_h=sh,
               final_z_expected=-0.5, clearance=stock + 6.0)
    sys.exit(0 if ok else 1)
