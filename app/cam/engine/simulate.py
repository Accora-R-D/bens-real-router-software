"""Job sheet + graphical toolpath simulation for a generated Syntec program.

Produces, for a given .NC (default out/SHEET_18mm.NC):
  - JOB_SHEET_<n>.md   operator job sheet (material, tool, parts, time, notes)
  - _toolpath_<n>.png  static top-view: cuts coloured by depth, rapids dashed
  - _simulation_<n>.gif animated tool motion (cut vs rapid, in program order)

Run:  python simulate.py [path-to.NC]
"""
import sys, re, math, json
from pathlib import Path
from datetime import date
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.animation as animation

RAPID_RATE = 12000.0   # assumed rapid traverse mm/min (for time estimate only)
# The router does not cut at the programmed feed through corners and short
# moves: it slows for each change of direction and speeds up again. Fitted to
# the machine (2026-10-28): Base Plate (1), 12 mm, timed at 45 min against a
# 31 min estimate that ignored this; the same setting puts the Oct-26 53-part
# sheet at ~1.5 h, which matches "an hour or two and still going". Corners are
# taken the way grbl-style planners take them (junction deviation), arcs as
# their chords.
MACHINE_ACCEL = 380.0          # mm/s^2
JUNCTION_DEV = 0.05            # mm
OUT = Path(__file__).parent / "out"


def _arc_points(x, y, nx, ny, i, j, ccw, step=0.5):
    """Tessellate a G02/G03 arc (centre = start+I/J, incremental) into points."""
    cx, cy = x+i, y+j
    r = math.hypot(x-cx, y-cy)
    a0 = math.atan2(y-cy, x-cx); a1 = math.atan2(ny-cy, nx-cx)
    sweep = (a1-a0) % (2*math.pi) if ccw else -((a0-a1) % (2*math.pi))
    if abs(sweep) < 1e-9:
        sweep = 2*math.pi if ccw else -2*math.pi      # full circle
    n = max(2, int(abs(sweep)*r/step))
    return [(cx+r*math.cos(a0+sweep*k/n), cy+r*math.sin(a0+sweep*k/n)) for k in range(1, n+1)]


def parse_segments(text):
    """Return motion segments: dict(kind, x0,y0,z0, x1,y1,z1, feed).
    kind in {rapid, cut, plunge}. G02/G03 arcs are tessellated into cut segments."""
    num = re.compile(r"([GXYZFIJ])(-?\d+\.?\d*)")
    g = 0; x = y = z = 0.0; f = 0.0
    segs = []
    started = False
    for raw in text.splitlines():
        line = re.sub(r"^N\d+", "", raw.split(";")[0])
        if re.search(r"G53|G28", line):          # skip homing / machine moves
            continue
        toks = {m.group(1): float(m.group(2)) for m in num.finditer(line)}
        if "G" in toks and toks["G"] in (0, 1, 2, 3):
            g = int(toks["G"])
        if "F" in toks: f = toks["F"]
        nx, ny, nz = toks.get("X", x), toks.get("Y", y), toks.get("Z", z)
        if started and g in (2, 3) and ("X" in toks or "Y" in toks):
            pts = _arc_points(x, y, nx, ny, toks.get("I", 0.0), toks.get("J", 0.0), g == 3)
            px, py = x, y
            for qx, qy in pts:
                segs.append({"kind": "cut", "x0": px, "y0": py, "z0": z,
                             "x1": qx, "y1": qy, "z1": nz, "f": f})
                px, py = qx, qy
            x, y, z = nx, ny, nz
            continue
        moved = (nx, ny, nz) != (x, y, z)
        if moved and started:
            dxy = math.hypot(nx-x, ny-y)
            if g == 0:
                kind = "rapid"
            elif dxy < 1e-6 and abs(nz-z) > 1e-6:
                kind = "plunge"
            else:
                kind = "cut"
            segs.append({"kind": kind, "x0": x, "y0": y, "z0": z,
                         "x1": nx, "y1": ny, "z1": nz, "f": f})
        x, y, z = nx, ny, nz
        started = True
    return segs


TOOL_CHANGE_MIN = 20.0 / 60.0   # per M06: carousel swap + spindle back to speed


def _planned_minutes(segs, feeds, accel=None, dev=None):
    """Time for the moves with the machine's acceleration: each move ramps up
    towards its feed and down again for the next corner, the corner speed set
    by how sharp it is (junction deviation). A rapid starts and ends at rest."""
    a = MACHINE_ACCEL if accel is None else accel
    dv = JUNCTION_DEV if dev is None else dev
    n = len(segs)
    if not n:
        return 0.0
    L, U, vmax = [], [], []
    for s, f in zip(segs, feeds):
        dx, dy, dz = s["x1"] - s["x0"], s["y1"] - s["y0"], s["z1"] - s["z0"]
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        L.append(d)
        U.append((dx / d, dy / d, dz / d) if d > 1e-12 else (0.0, 0.0, 0.0))
        vmax.append(max(f, 1e-6) / 60.0)                  # mm/s
    vj = [0.0] * (n + 1)                                 # speed entering move i
    for i in range(1, n):
        if segs[i]["kind"] == "rapid" or segs[i - 1]["kind"] == "rapid":
            continue
        u0, u1 = U[i - 1], U[i]
        c = -(u0[0] * u1[0] + u0[1] * u1[1] + u0[2] * u1[2])
        c = max(-1.0, min(1.0, c))
        if c < -0.999999:                                  # straight on
            v = 1e9
        else:
            h = math.sqrt(0.5 * (1.0 - c))
            v = math.sqrt(a * dv * h / (1.0 - h)) if h < 0.999999 else 0.0
        vj[i] = min(v, vmax[i - 1], vmax[i])
    for i in range(n - 1, -1, -1):                         # can it stop in time?
        vj[i] = min(vj[i], math.sqrt(vj[i + 1] ** 2 + 2 * a * L[i]))
    for i in range(n):                                     # can it get up to speed?
        vj[i + 1] = min(vj[i + 1], math.sqrt(vj[i] ** 2 + 2 * a * L[i]))
    t = 0.0
    for i in range(n):
        v0, v1, vm, d = vj[i], vj[i + 1], vmax[i], L[i]
        if d <= 0:
            continue
        da = (vm * vm - v0 * v0) / (2 * a)
        dd = (vm * vm - v1 * v1) / (2 * a)
        if da + dd <= d:                                   # reaches the feed
            t += (vm - v0) / a + (vm - v1) / a + (d - da - dd) / vm
        else:                                              # never does
            vp = math.sqrt(max((2 * a * d + v0 * v0 + v1 * v1) / 2, 0.0))
            t += max(vp - v0, 0) / a + max(vp - v1, 0) / a
    return t / 60.0


def stats(segs, tool_changes: int = 0):
    """Distances and an estimated cycle time for a parsed program.

    The time used to be every cutting move at the program's FIRST feed, rapids
    in XY only: pockets, drilling, engraving and ramps at their own feeds were
    all costed at the profile feed, and the hundreds of vertical retracts and
    approaches cost nothing. Now each move is timed at its own feed over its
    true 3-D length, rapids include their Z travel, and each tool change adds
    TOOL_CHANGE_MIN. Since 2026-10-28 the machine's acceleration is in it too
    (see MACHINE_ACCEL): a 31 min estimate had run 45 min on the machine."""
    def L(s): return math.hypot(s["x1"]-s["x0"], s["y1"]-s["y0"])
    def L3(s): return math.sqrt((s["x1"]-s["x0"])**2 + (s["y1"]-s["y0"])**2 + (s["z1"]-s["z0"])**2)
    cut = sum(L(s) for s in segs if s["kind"] == "cut")
    rapid = sum(L(s) for s in segs if s["kind"] == "rapid")
    plunge = sum(abs(s["z1"]-s["z0"]) for s in segs if s["kind"] == "plunge")
    cut_f = next((s["f"] for s in segs if s["kind"] == "cut" and s["f"]), 3672.0)
    pl_f = next((s["f"] for s in segs if s["kind"] == "plunge" and s["f"]), 350.0)
    feeds = [RAPID_RATE if s["kind"] == "rapid"
             else (s["f"] or (pl_f if s["kind"] == "plunge" else cut_f)) for s in segs]
    t = _planned_minutes(segs, feeds)
    t += max(0, int(tool_changes)) * TOOL_CHANGE_MIN
    xs = [s["x1"] for s in segs] + [s["x0"] for s in segs]
    ys = [s["y1"] for s in segs] + [s["y0"] for s in segs]
    return dict(cut=cut, rapid=rapid, plunge=plunge, minutes=t,
                deepest=min(s["z1"] for s in segs),
                bbox=(min(xs), min(ys), max(xs), max(ys)),
                n=len(segs))


def jobsheet_md(nc_name, manifest, st):
    """Operator job sheet (Markdown string) for one program. Leads with the two
    numbers the operator wants first — cycle time + safety gate — then a
    before-cycle-start checklist (T-table diameters, work origin/datum, stock),
    setup, parts, run estimate and notes. All values come from the manifest, so
    they stay correct (no hardcoded tool/spindle)."""
    m = manifest or {}
    bx0, by0, bx1, by1 = st["bbox"]
    gate = m.get("safety_gate", "?")
    reasons = m.get("safety_gate_reasons") or []
    gwarns = m.get("safety_gate_warnings") or []
    L = []
    L.append(f"# CNC Router Job Sheet — Sheet {m.get('sheet', 1)}")
    L.append("")
    L.append(f"**Program:** `{nc_name}`  |  **Generated:** {date.today().isoformat()}  |  **Controller:** Syntec")
    L.append("")
    L.append(f"**Est. cycle time: {st['minutes']:.1f} min**  ·  **Safety gate: {gate}**")
    if gate != "PASS" and reasons:
        L.append("")
        L.append("> ⚠ **DO NOT RUN — safety gate FAILED:**")
        for r in reasons:
            L.append(f"> - {r}")
    if gwarns:
        L.append("")
        for w in gwarns:
            L.append(f"> ⚠ {w}")
    L.append("")
    L.append("## ✅ Before cycle start")
    L.append("")
    L.append("- [ ] **Set the machine tool table (T-table) so each T-number is the diameter below**")
    L.append("      *(tool-centre G-code — a wrong T-table silently mis-sizes every part):*")
    for t in m.get("tools", []):
        L.append(f"    - [ ] {t}")
    if m.get("tool_overrides"):
        L.append("- [ ] **TOOL OVERRIDE in force** — these seats were FORCED on the form, not chosen "
                 "automatically. Check each one is the bit you meant:")
        for feat, seat in m["tool_overrides"].items():
            L.append(f"    - [ ] {feat}: {seat}")
    L.append("- [ ] **Work origin (G54): sheet FRONT-LEFT corner; Z0 = board bottom / spoilboard top.**")
    if m.get("frame"):
        fx0, fy0, fx1, fy1 = m["frame"]["rect"]
        L.append(f"- [ ] **Watch the opening frame:** before cutting, the tool traces the nest "
                 f"outline X[{fx0:.0f}–{fx1:.0f}] Y[{fy0:.0f}–{fy1:.0f}] at Z{m['frame']['z']:g} "
                 f"(the rapid height). FEED HOLD if it leaves the stock or comes near a clamp.")
    L.append(f"- [ ] Stock {m.get('sheet_w',2440):.0f} × {m.get('sheet_h',1220):.0f} mm × "
             f"{m.get('thickness_mm','?')} mm — parts lie within X[{bx0:.0f}–{bx1:.0f}] Y[{by0:.0f}–{by1:.0f}] mm.")
    L.append("- [ ] Vacuum on / hold-down confirmed (tab-less parts cut fully free on the last pass).")
    L.append("")
    L.append("## Setup")
    L.append("")
    L.append("| Item | Value |")
    L.append("|---|---|")
    L.append(f"| Tools (seat → Ø) | {', '.join(m.get('tools', [])) or '?'} |")
    L.append(f"| Material thickness | {m.get('thickness_mm','?')} mm |")
    L.append(f"| Spindle | {m.get('spindle_rpm','?')} rpm |")
    L.append(f"| Cut feed | {m.get('cut_feed',0):.0f} mm/min |")
    L.append(f"| Plunge feed | {m.get('plunge_feed',0):.0f} mm/min |")
    L.append(f"| Depth passes | {m.get('passes','?')} |")
    L.append("")
    L.append("## Parts on this sheet")
    L.append("")
    parts = m.get("parts", [])
    if parts:
        L.append("| # | Part | Rot | Profiles | Inner cut-outs | Drilled holes | Tabs |")
        L.append("|---|---|---|---|---|---|---|")
        for i, p in enumerate(parts, 1):
            L.append(f"| {i} | {p['name']} | {p['rotation_deg']}° | {p['profiles']} | "
                     f"{p['inner_cutouts']} | {p['drilled_holes']} | {'YES' if p['tabs'] else '—'} |")
    L.append("")
    L.append("## Run estimate")
    L.append("")
    L.append("| Metric | Value |")
    L.append("|---|---|")
    L.append(f"| Cutting distance | {st['cut']/1000:.1f} m |")
    L.append(f"| Rapid distance | {st['rapid']/1000:.1f} m |")
    L.append(f"| Total plunge | {st['plunge']:.0f} mm |")
    L.append(f"| Deepest Z | {st['deepest']:.2f} mm |")
    L.append(f"| **Est. cycle time** | **{st['minutes']:.1f} min** (cut+plunge+rapid @ {RAPID_RATE:.0f}mm/min) |")
    L.append("")
    L.append("## Operator notes")
    L.append("")
    tabbed = [p["name"] for p in parts if p.get("tabs")]
    drilled = [p["name"] for p in parts if p.get("drilled_holes")]
    L.append(f"- **Tabs:** {', '.join(tabbed) if tabbed else 'none'} — small parts left on holding tabs; "
             "snap/trim free after the cut.")
    L.append(f"- **Drilled holes:** {', '.join(drilled) if drilled else 'none'} — plunge-drilled with the "
             "router bit (holes ≤ cutter Ø).")
    L.append("- **Workholding:** vacuum bed assumed for tab-less parts. Check vacuum before cycle start.")
    return "\n".join(L)


def write_jobsheet(nc, manifest, st, idx):
    fn = OUT / f"JOB_SHEET_{idx}.md"
    fn.write_text(jobsheet_md(nc.name, manifest, st), encoding="utf-8")
    return fn


def plot_static(segs, st, manifest, idx):
    sw = (manifest or {}).get("sheet_w", 2440); sh = (manifest or {}).get("sheet_h", 1220)
    fig, ax = plt.subplots(figsize=(16, 8))
    ax.add_patch(plt.Rectangle((0, 0), sw, sh, fill=False, ec="#333", lw=1.5))
    # rapids
    rseg = [[(s["x0"], s["y0"]), (s["x1"], s["y1"])] for s in segs if s["kind"] == "rapid"]
    ax.add_collection(LineCollection(rseg, colors="#cc0000", linewidths=0.4,
                                     linestyles="dashed", alpha=0.5))
    # cuts coloured by depth
    cseg = [[(s["x0"], s["y0"]), (s["x1"], s["y1"])] for s in segs if s["kind"] == "cut"]
    cz = [s["z1"] for s in segs if s["kind"] == "cut"]
    if cseg:
        lc = LineCollection(cseg, array=__import__("numpy").array(cz), cmap="viridis_r", linewidths=0.7)
        ax.add_collection(lc); fig.colorbar(lc, ax=ax, label="cut depth Z (mm)", shrink=0.6)
    # plunge / drill points
    for s in segs:
        if s["kind"] == "plunge":
            ax.plot(s["x1"], s["y1"], ".", color="#0040ff", ms=2)
    ax.set_aspect("equal"); ax.set_xlim(-30, sw+30); ax.set_ylim(-30, sh+30)
    ax.set_title(f"Toolpaths sheet {idx} — green=cut (by depth), red dashed=rapid, blue=plunge — "
                 f"~{st['minutes']:.0f} min")
    ax.grid(True, alpha=0.2)
    fn = OUT / f"_toolpath_{idx}.png"; fig.tight_layout(); fig.savefig(fn, dpi=110); plt.close(fig)
    return fn


def animate(segs, manifest, idx, nframes=220):
    sw = (manifest or {}).get("sheet_w", 2440); sh = (manifest or {}).get("sheet_h", 1220)
    fig, ax = plt.subplots(figsize=(14, 7))
    ax.add_patch(plt.Rectangle((0, 0), sw, sh, fill=False, ec="#333", lw=1.5))
    ax.set_aspect("equal"); ax.set_xlim(-30, sw+30); ax.set_ylim(-30, sh+30)
    ax.grid(True, alpha=0.2); ax.set_title(f"Toolpath simulation — sheet {idx}")
    cut_lc = LineCollection([], colors="#00836A", linewidths=0.8); ax.add_collection(cut_lc)
    rap_lc = LineCollection([], colors="#cc0000", linewidths=0.4, linestyles="dashed", alpha=0.5)
    ax.add_collection(rap_lc)
    tool, = ax.plot([], [], "o", color="#ff8000", ms=7)
    n = len(segs); chunk = max(1, math.ceil(n / nframes))

    def update(fr):
        upto = min(n, (fr+1)*chunk)
        sub = segs[:upto]
        cut_lc.set_segments([[(s["x0"], s["y0"]), (s["x1"], s["y1"])]
                             for s in sub if s["kind"] != "rapid"])
        rap_lc.set_segments([[(s["x0"], s["y0"]), (s["x1"], s["y1"])]
                             for s in sub if s["kind"] == "rapid"])
        if upto:
            tool.set_data([sub[-1]["x1"]], [sub[-1]["y1"]])
        return cut_lc, rap_lc, tool

    frames = math.ceil(n / chunk)
    anim = animation.FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    fn = OUT / f"_simulation_{idx}.gif"
    anim.save(fn, writer=animation.PillowWriter(fps=18))
    plt.close(fig)
    return fn


def run(nc):
    text = nc.read_text()
    segs = parse_segments(text)
    st = stats(segs)
    mf_path = nc.with_suffix(".json")
    manifest = json.loads(mf_path.read_text()) if mf_path.exists() else {}
    idx = manifest.get("sheet", 1)
    js = write_jobsheet(nc, manifest, st, idx)
    tp = plot_static(segs, st, manifest, idx)
    print(f"  {nc.name}: {st['n']} moves, cut {st['cut']/1000:.1f}m, rapid {st['rapid']/1000:.1f}m, "
          f"~{st['minutes']:.1f} min")
    print(f"  job sheet -> {js.name}")
    print(f"  toolpath  -> {tp.name}")
    print("  rendering simulation gif ...")
    gif = animate(segs, manifest, idx)
    print(f"  simulation -> {gif.name}")


if __name__ == "__main__":
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT / "SHEET_18mm.NC"
    run(target)
