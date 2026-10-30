"""Engine API: turn extracted parts + Material + ToolTable into a Syntec program.

Reuses the proven pipeline in ../engine/ (nest -> multi-tool post -> verify).
Per-request config is applied by mutating the engine modules' config
(MAT/TOOLS/...) under a lock, so behaviour is identical to the CLI runs.

The extraction half is gone: in the fab exporter the geometry arrives already
extracted from the assembly STEP, so the entry point is generate_from_parts().
"""
from __future__ import annotations
import sys, io, re, json, copy, hashlib, threading, contextlib
from pathlib import Path
from dataclasses import dataclass, field
from dataclasses import replace as _dc_replace

from .models import Material, ToolTable

# --- make the proven pipeline importable -------------------------------------
# Monorepo layout: the engine lives at <repo>/engine/ (sibling of router_cam/).
# Honour ROUTER_ENGINE_DIR only when it actually points at the engine; otherwise
# fall back to the in-repo engine/. This survives a stale env var — e.g. an old
# dashboard ROUTER_ENGINE_DIR=/app/engine_core left over from before the monorepo,
# which would otherwise put a dead path on sys.path and break `import osclient`.
import os
_DEFAULT_ENGINE = Path(__file__).resolve().parent.parent / "engine"
_env_dir = os.getenv("ROUTER_ENGINE_DIR")
if _env_dir and (Path(_env_dir) / "osclient.py").exists():
    ENGINE_DIR = Path(_env_dir)
else:
    ENGINE_DIR = _DEFAULT_ENGINE
if str(ENGINE_DIR) not in sys.path:
    sys.path.insert(0, str(ENGINE_DIR))

# NOTE: `extract` is NOT imported here. It is the Onshape B-rep reader, and in
# the exporter the geometry arrives already extracted from the STEP, so the wood
# path never touches it. The metal/die path still calls it and imports it
# lazily at its own call sites; keeping it off the module import means the CAM
# engine loads in an environment with no Onshape client at all.
import cam_syntec as cs                # noqa: E402
import nest                            # noqa: E402
import verify                          # noqa: E402
import simulate                        # noqa: E402
import viewer3d                        # noqa: E402

_LOCK = threading.Lock()
URL_RE = re.compile(r"/documents/(?:d/)?([0-9a-f]+)/w/([0-9a-f]+)/e/([0-9a-f]+)")

# Cache of extracted part profiles so re-nesting (new params / remnant) skips the
# Onshape round-trip. Keyed by a stable job_id derived from url+thickness+exclude.
_PARTS_CACHE: dict = {}
_CACHE_ORDER: list = []


def _cache_put(job_id, payload):
    _PARTS_CACHE[job_id] = payload
    _CACHE_ORDER.append(job_id)
    while len(_CACHE_ORDER) > 32:
        _PARTS_CACHE.pop(_CACHE_ORDER.pop(0), None)


def parse_url(url: str):
    m = URL_RE.search(url)
    if not m:
        raise ValueError("Not a recognised Onshape document URL")
    return m.group(1), m.group(2), m.group(3)


@dataclass
class Result:
    nc: str
    manifest: dict
    warnings: list
    viewer: dict
    layout: list = field(default_factory=list)   # per-sheet plan-view geometry (first program)
    job_id: str = ""                             # reuse cached parts for re-nest
    diagnostics: list = field(default_factory=list)   # per-part extraction/placement outcome
    programs: list = field(default_factory=list)      # one entry per sheet thickness


# The Onshape document scan and B-rep extraction that used to live here
# (_assembly_part_counts / _elem_geometry / list_parts / _extract_parts)
# were removed with the Onshape input layer. The exporter reads the
# assembly STEP and hands parts straight to generate_from_parts().


# The features an operator may force a seat for, as the job sheet names them.
# "chamfer" and "parting" ride in the same tool_map but have their own controls.
TOOL_FEATURE_LABELS = {"profile": "outer profiles", "cutout": "internal cut-outs",
                       "pocket": "pockets", "drill": "holes", "engrave": "engraving",
                       "rough": "3D roughing", "surface": "3D finishing (ball nose)"}


def _apply_config(material: Material, tools: ToolTable):
    cs.MAT.clear(); cs.MAT.update(material.mat_dict())
    tdict, pref = tools.engine_tools(material)
    cs.TOOLS.clear(); cs.TOOLS.update(tdict)
    cs.TOOL_PREF[:] = pref
    cs.DEFAULT_TOOL = pref[0] if pref else 4
    cs.TAB_REMAIN = material.tab_remain
    cs.TAB_WIDTH = material.tab_width
    # engraving uses the V-bit seat (kind == "vee") — engrave_seats() includes a Ø0
    # tip-diameter vee that the cutting-tool filter would drop. None -> fall back to narrowest.
    cs.ENGRAVE_TOOL = next((s.seat_no for s in tools.engrave_seats() if s.kind == "vee"), None)
    cs.DOGBONE = getattr(material, "dogbone", True)   # corner overcuts on/off
    cs.SKIP_PROFILE = getattr(material, "pockets_only", False)   # pockets-only: don't cut the outline
    cs.CLEAR_POCKETS = getattr(material, "clear_pockets_25d", True)   # area-clear recesses, or just profile them
    # edge break (chamfer): the loaded 'chamfer' seat + size/scope from the material
    cs.CHAMFER_TOOL = tools.chamfer_seat()
    cs.BALL_TOOL = tools.ball_seat()          # 3D surface finishing seat (or None)
    cs.EDGE_BREAK = getattr(material, "edge_break", False)
    cs.EDGE_BREAK_SIZE = getattr(material, "edge_break_size", 2.0)
    cs.EDGE_BREAK_ANGLE = getattr(material, "edge_break_angle", 90.0)
    cs.EDGE_BREAK_SCOPE = getattr(material, "edge_break_scope", "profile")
    # Finishing lap on the profiles (see cam_syntec.FINISH_PASS)
    cs.FINISH_PASS = getattr(material, "finish_pass", False)
    cs.FINISH_ALLOWANCE = getattr(material, "finish_allowance", 0.5)
    cs.FINISH_STEPPED = getattr(material, "finish_stepped", False)


# All cam_syntec module globals that _apply_config mutates. snapshot/restore make an
# operation hermetic: a later op never inherits a stale value a previous op set. This
# is the prerequisite for the cam_core operation-tree orchestrator (and for any future
# move to an instance-safe engine — gated behind a byte-identical proof). The lock and
# single-worker model stay; this only makes config application symmetric.
_CONFIG_ATTRS = ["MAT", "TOOLS", "TOOL_PREF", "DEFAULT_TOOL", "TAB_REMAIN", "TAB_WIDTH",
                 "ENGRAVE_TOOL", "DOGBONE", "SKIP_PROFILE", "CLEAR_POCKETS", "CHAMFER_TOOL",
                 "EDGE_BREAK", "EDGE_BREAK_SIZE", "EDGE_BREAK_ANGLE", "EDGE_BREAK_SCOPE",
                 "BALL_TOOL", "FINISH_PASS", "FINISH_ALLOWANCE", "FINISH_STEPPED"]


def snapshot_config() -> dict:
    """Deep-copy every cam_syntec config global so it can be restored verbatim."""
    return {a: copy.deepcopy(getattr(cs, a)) for a in _CONFIG_ATTRS if hasattr(cs, a)}


def restore_config(snap: dict):
    """Restore globals from a snapshot. dicts/lists are mutated IN PLACE (clear/update,
    slice-assign) so references the engine holds stay valid; scalars are reassigned."""
    for a, v in snap.items():
        cur = getattr(cs, a, None)
        if isinstance(cur, dict):
            cur.clear(); cur.update(v)
        elif isinstance(cur, list):
            cur[:] = v
        else:
            setattr(cs, a, copy.deepcopy(v))


@contextlib.contextmanager
def configured(material: Material, tools: ToolTable):
    """Apply per-op config under the engine lock, restoring the prior config on exit so
    each operation is hermetic. Used by the cam_core orchestrator for non-nest ops; the
    proven generate()/regenerate() path manages its own config inside _nest_one."""
    with _LOCK:
        snap = snapshot_config()
        try:
            _apply_config(material, tools)
            yield
        finally:
            restore_config(snap)


def _viewer_payload(nc_path: Path, top: float, tools: "ToolTable | None" = None):
    segs = simulate.parse_segments(nc_path.read_text())
    pts = viewer3d.decimate(segs)
    # Each seat's cutter as the viewer carves it: a ball nose leaves a round
    # bottom and a V-bit a 90 deg groove; everything else is a flat disc. The
    # viewer used to carve every move with the profile cutter's flat disc, so a
    # ball-nose finish came out as a flat-bottomed staircase.
    shapes = {}
    for s in (tools.seats if tools else []):
        if s.loaded:
            kind = ("ball" if s.kind == "ballnose"
                    else "vee" if s.kind in ("vee", "chamfer") else "flat")
            shapes[str(s.seat_no)] = {"R": round(max(s.diameter_mm, 0.0) / 2, 3), "kind": kind}
    cutpts = [(s["x0"], s["y0"]) for s in segs if s["kind"] != "rapid"] + \
             [(s["x1"], s["y1"]) for s in segs if s["kind"] != "rapid"]
    if not cutpts:
        return {}
    import math
    pad = cs.R + 8
    minx = min(p[0] for p in cutpts)-pad; maxx = max(p[0] for p in cutpts)+pad
    miny = min(p[1] for p in cutpts)-pad; maxy = max(p[1] for p in cutpts)+pad
    # Adaptive height-field resolution: refine a small job (a single pocket) to a crisp
    # grid, but bound the cell count so a full-sheet nest stays light in the browser.
    extx = max(1.0, maxx-minx); exty = max(1.0, maxy-miny)
    # The grid itself never crosses the wire — only its dimensions do; the browser
    # builds and carves the height field locally. So resolution costs client CPU
    # and memory, not payload: 1M cells is ~1M vertices and a 4MB height texture,
    # comfortably within WebGL2 on the workshop machines, and visibly crisper on
    # small features than the old 300k.
    # 2M cells at 0.2 mm (a 280 mm square at full detail) since the viewer
    # builds its index buffer as one typed array and shades per pixel.
    MAX_CELLS = 2_000_000
    res = round(max(0.2, math.sqrt(extx*exty / MAX_CELLS)), 3)
    return {"grid": {"gw": int(math.ceil((maxx-minx)/res)), "gh": int(math.ceil((maxy-miny)/res)),
                     "res": res, "ox": round(minx, 2), "oy": round(miny, 2), "top": round(top, 2)},
            "path": {"px": [p[0] for p in pts], "py": [p[1] for p in pts],
                     "pz": [p[2] for p in pts], "cut": [p[3] for p in pts],
                     "tool": [p[4] for p in pts]},
            "tools": shapes,
            # 3D jobs: each part's model and toolpaths, placed (nest._scene_part)
            "scene": _scene(nc_path),
            "R": cs.R}


def _scene(nc_path: Path):
    import json
    f = nc_path.with_suffix(".scene.json")
    try:
        return json.loads(f.read_text()) if f.exists() else []
    except (OSError, ValueError):
        return []


def _frame_lines(nc_text, clear, sheet_w, sheet_h, dia_by_seat):
    """The opening frame for one program: G00 moves at the clearance height round
    the rectangle the cutter will touch — every cutting move's centre, grown by
    the widest cutter the program loads, kept on the sheet so the safety gate's
    bounds rule still holds. None when the program cuts nothing."""
    import re
    xs, ys = [], []
    for r in verify.parse(nc_text):
        if r["g"] in (1, 2, 3):
            xs.append(r["x"]); ys.append(r["y"])
    if not xs:
        return None
    seats = [int(t) for t in re.findall(r"T(\d+)M0?6", nc_text)]
    rad = max((dia_by_seat.get(t, 0.0) for t in seats), default=0.0) / 2.0
    x0 = max(0.0, min(xs) - rad); x1 = min(sheet_w, max(xs) + rad)
    y0 = max(0.0, min(ys) - rad); y1 = min(sheet_h, max(ys) + rad)
    lines = [f"G00Z{clear:.3f}",
             f"G00X{x0:.3f}Y{y0:.3f}",
             f"G00X{x1:.3f}Y{y0:.3f}",
             f"G00X{x1:.3f}Y{y1:.3f}",
             f"G00X{x0:.3f}Y{y1:.3f}",
             f"G00X{x0:.3f}Y{y0:.3f}"]
    return lines, (round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1))


def add_frame(nc_path, clear, sheet_w, sheet_h, dia_by_seat):
    """Insert the opening frame into a written program, straight after the header
    (tool selected, length offset on, spindle started) and before the first cut,
    and renumber. Returns the frame rectangle, or None if nothing was added.

    Rapids, not feed moves: the frame is a look, not a cut. It runs at the height
    every other rapid in the program uses, so if the frame clears the clamps the
    job's rapids do too — and the operator still has the rapid-override knob to
    slow it down and watch the corners."""
    import re
    text = Path(nc_path).read_text()
    got = _frame_lines(text, clear, sheet_w, sheet_h, dia_by_seat)
    if not got:
        return None
    frame, rect = got
    body = [re.sub(r"^N\d+", "", ln) for ln in text.splitlines() if ln.strip()]
    # end of the header: the first spindle start (S...M3)
    at = next((i for i, ln in enumerate(body) if re.fullmatch(r"S\d+M0?3", ln.strip())), None)
    if at is None:
        return None
    body = body[:at + 1] + frame + body[at + 1:]
    Path(nc_path).write_text("\n".join(f"N{10 * k}{ln}" for k, ln in enumerate(body)) + "\n")
    return rect


def _nest_one(parts_loops, material: Material, tools: ToolTable,
              sheet_w, sheet_h, blocked, prefix, sets=1, auto_part=False, part_cuts=None,
              tool_map=None, engrave_pos=None, swap_xy=False,
              border_mm=None, placements_in=None) -> dict:
    """Nest one thickness group into a program payload. Each part is replicated by
    its instance qty × `sets` (whole-assembly multiplier). material.thickness_mm is
    this group's thickness."""
    buf = io.StringIO()
    with _LOCK, contextlib.redirect_stdout(buf):
        _apply_config(material, tools)
        if getattr(material, "pockets_only", False):
            print("    ! pockets-only: outer profile NOT cut — parts stay attached to the sheet "
                  "(pockets / holes / cut-outs / engraving still machined)")
        engrave_on = getattr(material, "engrave", False)
        tm = tool_map or {}
        epos = engrave_pos or {}
        eng_assigned = tm.get("engrave")
        eng_tool = eng_assigned if eng_assigned in cs.TOOLS else cs.ENGRAVE_TOOL
        is_vee = {s.seat_no for s in tools.engrave_seats() if s.kind == "vee"}
        if engrave_on and eng_tool is None:
            print(f"    ! engraving on but no V-bit (kind 'vee') loaded and none assigned — using the "
                  f"narrowest tool (Ø{cs.TOOLS[cs.TOOL_PREF[-1]]['dia']:.0f}mm); load/assign a vee for crisp text")
        elif engrave_on and eng_tool is not None and eng_tool not in is_vee:
            print(f"    ! engraving with T{eng_tool:02d} (not a V-bit) — text may be wide/chunky")
        prepared = []
        machine_3d = getattr(material, "machine_3d_features", True)
        if not machine_3d:
            print("    ! 3D features off: counterbores and blind pockets are NOT "
                  "machined — only features that go through the board are cut")
        if not getattr(material, "clear_pockets_25d", True):
            print("    ! 2.5D clearing off: recesses are profiled at depth, not cleared — "
                  "the waste in the middle of each one is left in")
        for name, loops, meta in parts_loops:
            m = dict(meta)
            m["engrave"] = engrave_on and not meta.get("no_engrave")
            m["engrave_height"] = getattr(material, "engrave_height", 25.0)
            m["tool_map"] = tm
            m["engrave_offset"] = epos.get(name) or [0.0, 0.0]
            part_loops = copy.deepcopy(loops)
            if not machine_3d:
                # Drop the recesses that keep a floor; anything that breaks through
                # is a cut-out by another name and still gets cut.
                part_loops = [lp for lp in part_loops
                              if lp.get("role") not in ("POCKET", "SURFACE")
                              or lp.get("through")]
            pp = nest.prepare_part(name, part_loops, m)
            n = max(1, int(meta.get("qty", 1))) * max(1, int(sets))
            prepared.extend([pp] * n)               # qty × sets copies onto the sheet(s)
            npk = len(meta.get("pockets") or [])
            back = m.get("backside_loops")
            if back and machine_3d and not getattr(material, "pockets_only", False):
                # Two-sided 2.5D: nest an inverse mould (the far-face outline as
                # a snug through-cavity) alongside the parts. Pass 1 cuts it with
                # everything else; the operator flips the finished part into it
                # and runs the separate SIDE 2 program, which cuts only the far
                # face's recesses. One mould per copy.
                mould = nest.prepare_mould(f"{name} mould", copy.deepcopy(back), m,
                                           far_pockets=copy.deepcopy(m.get("backside_far")),
                                           thickness=material.thickness_mm)
                if not mould["pass2_jobs"]:
                    # Every far-side recess was refused (too narrow for the loaded
                    # tools) — a mould with nothing to cut in it is wasted board
                    # and a misleading instruction to flip.
                    print(f"    !! {name}: none of the far-side recesses can be machined "
                          f"with the loaded tools — no mould cut; those features need "
                          f"a smaller cutter or a hand drill")
                    mould = None
            else:
                mould = None
            if mould:
                prepared.extend([mould] * n)
                if mould.get("mould_mode") == "inverse":
                    print(f"    ! {name}: pockets on BOTH faces, and the full-thickness "
                          f"contact patch is too small to sit flat on "
                          f"({mould['mould_contact']*100:.0f}% of the footprint or less) — "
                          f"the mould is milled as the part's 2.5D INVERSE so every region "
                          f"is supported at its own level (slower than a plain cavity). "
                          f"When pass 1 finishes, remove the part (no plug — this mould is "
                          f"all chips), drop it in UPSIDE DOWN, and run the SIDE 2 program "
                          f"without moving the sheet")
                else:
                    print(f"    ! {name}: pockets on BOTH faces — pass 1 cuts the {meta['up_side']} "
                          f"side and an inverse mould; when it finishes, remove the part and the "
                          f"mould plug, drop the part in UPSIDE DOWN (it should be a snug fit), "
                          f"and run the SIDE 2 program without moving the sheet. If the outline "
                          f"is symmetric, check the already-cut features line up with the plan "
                          f"before running side 2 — a symmetric part seats more than one way")
            elif meta.get("two_sided"):
                print(f"    !! {name}: pockets on BOTH faces — only the {meta['up_side']} side is "
                      f"machined; a second (flipped) setup is needed for the other side")
            elif meta.get("flip"):
                # Recesses only on the modelled underside: the program is simply
                # generated with that face up, outline mirrored to match. A board
                # machined from one side is the same part whichever way it went
                # on the bed, so this is information, not a warning — the
                # warnings are for the two-sided cases above, where a second
                # setup really is needed.
                print(f"    i {name}: recesses are on the modelled underside — the program "
                      f"machines them with that face up, which is the same finished part")
        if not prepared:
            raise ValueError(f"No parts at {material.thickness_mm}mm found.")
        files = nest.nest_and_emit(prepared, thickness=material.thickness_mm, prefix=prefix,
                                   sheet_w=sheet_w, sheet_h=sheet_h, blocked=blocked or [],
                                   tabs_mode=getattr(material, "tabs_mode", "auto"), backplot=False,
                                   auto_part=auto_part, part_cuts=part_cuts or [],
                                   grain_locked=getattr(material, "grain_locked", False),
                                   swap_xy=swap_xy, part_tool=tm.get("parting"),
                                   border_mm=border_mm, placements_in=placements_in)

    # The opening frame goes in before anything reads the programs back — the
    # gate, the time estimate and the viewer all see the file as it will run.
    vw, vh = (sheet_h, sheet_w) if swap_xy else (sheet_w, sheet_h)   # NC bounds follow the swap
    clear_z = material.thickness_mm + material.clearance_over
    dia_by_seat = {s.seat_no: s.diameter_mm for s in tools.seats}
    frames = {}
    if getattr(material, "frame_nest", True):
        for f in list(files) + [f.with_name(f.stem + "_SIDE2.NC") for f in files]:
            if f.exists():
                frames[f.name] = add_frame(f, clear_z, vw, vh, dia_by_seat)
    manifests = [json.loads(f.with_suffix(".json").read_text()) for f in files]
    nc_path = files[0]
    if frames.get(nc_path.name):
        manifests[0]["frame"] = {"rect": frames[nc_path.name], "z": round(clear_z, 2)}
    manifest = manifests[0]
    log = buf.getvalue()
    warnings = [ln.strip() for ln in log.splitlines() if "!" in ln or "SKIP" in ln.upper()]
    # An overridden thickness is the one setting where the program deliberately
    # disagrees with the models it came from, so it leads the warnings rather than
    # sitting quietly in a form the operator filled in ten minutes ago.
    overridden = float(getattr(material, "thickness_override", 0.0) or 0.0) > 0
    if overridden:
        modelled = sorted({float(m.get("thickness") or 0.0) for _n, _l, m in parts_loops})
        says = ", ".join(f"{t:g}mm" for t in modelled if t) or "no stated thickness"
        warnings.insert(0, f"! STOCK THICKNESS OVERRIDDEN: cutting every part from a "
                           f"{material.thickness_mm:g}mm board. The models say {says}.")
    # Forced tools are the other deliberate departure from Auto — a long-flute bit
    # in a spare seat for holes the standard one cannot reach through, say. Named
    # up front for the same reason as the thickness, and carried on the manifest
    # so the job sheet states which seats were forced after the form is gone.
    forced = {k: int(v) for k, v in (tool_map or {}).items()
              if v and k in TOOL_FEATURE_LABELS}
    if forced:
        def _seat(t):
            return (f"T{t:02d} (Ø{cs.TOOLS[t]['dia']:g}mm)" if t in cs.TOOLS
                    else f"T{t:02d} (NOT LOADED)")
        desc = ", ".join(f"{TOOL_FEATURE_LABELS[k]} -> {_seat(t)}" for k, t in forced.items())
        warnings.insert(1 if overridden else 0,
                        f"! TOOL OVERRIDE: {desc}. Auto tool choice is off for these "
                        f"features; a forced tool that does not fit a feature is swapped "
                        f"back and noted below.")
        manifest["tool_overrides"] = {TOOL_FEATURE_LABELS[k]: _seat(t) for k, t in forced.items()}
    # Safety gate: aggregate PASS/FAIL AND the specific reasons across all sheets, so a
    # FAIL tells the operator WHY (the reasons were computed and discarded before).
    gate_ok = True; gate_fails = []; gate_warns = []
    for f in files:
        okf, failsf, warnsf = verify.check(
            f, stock_top=material.thickness_mm, sheet_w=vw, sheet_h=vh,
            final_z_expected=material.final_z,
            clearance=material.thickness_mm + material.clearance_over,
            expect_through=not getattr(material, "pockets_only", False))
        gate_ok = gate_ok and okf
        gate_fails += failsf; gate_warns += warnsf
    manifest["safety_gate"] = "PASS" if gate_ok else "FAIL"
    manifest["safety_gate_reasons"] = gate_fails
    manifest["safety_gate_warnings"] = gate_warns
    # Feedrate-aware cycle-time estimate (also computed before, never surfaced).
    nc_text = nc_path.read_text()
    st = simulate.stats(simulate.parse_segments(nc_text), tool_changes=nc_text.count("M06"))
    manifest["est_minutes"] = round(st["minutes"], 1)
    job_sheet = simulate.jobsheet_md(nc_path.name, manifest, st)
    layout = [m["layout"] for m in manifests if "layout" in m]

    # Flip-jig SECOND PASS program(s), written alongside by nest_and_emit when a
    # two-sided part put a mould on the sheet. Gated separately: pass 2 cuts only
    # recesses, so it must NOT be required to cut through — but every other rule
    # (depth, bounds, rapids, clearance) applies unchanged.
    side2_files = [f2 for f in files
                   for f2 in [f.with_name(f.stem + "_SIDE2.NC")] if f2.exists()]
    side2 = None
    if side2_files:
        ok2 = True; fails2 = []; warns2 = []
        for f in side2_files:
            okf, failsf, warnsf = verify.check(
                f, stock_top=material.thickness_mm, sheet_w=vw, sheet_h=vh,
                final_z_expected=material.final_z,
                clearance=material.thickness_mm + material.clearance_over,
                expect_through=False)
            ok2 = ok2 and okf
            fails2 += failsf; warns2 += warnsf
        man2 = json.loads(side2_files[0].with_suffix(".json").read_text())
        if frames.get(side2_files[0].name):
            man2["frame"] = {"rect": frames[side2_files[0].name], "z": round(clear_z, 2)}
        man2["safety_gate"] = "PASS" if ok2 else "FAIL"
        man2["safety_gate_reasons"] = fails2
        man2["safety_gate_warnings"] = warns2
        nc2 = side2_files[0].read_text()
        st2 = simulate.stats(simulate.parse_segments(nc2), tool_changes=nc2.count("M06"))
        man2["est_minutes"] = round(st2["minutes"], 1)
        side2 = {"thickness": material.thickness_mm, "side2": True,
                 # the flip pass is cut from the same board, so it carries the same
                 # override marking through to its own filename
                 "overridden": overridden,
                 "sheets": len(side2_files), "nc": nc2, "manifest": man2,
                 "warnings": ["! SIDE 2: run only after pass 1 — remove the part "
                              "and the mould plug, drop the part in upside down, "
                              "and do NOT move the sheet"],
                 "layout": [],
                 "viewer": _viewer_payload(side2_files[0], material.thickness_mm, tools),
                 "nc_files": [{"name": f.name, "nc": f.read_text()}
                              for f in side2_files],
                 "job_sheet": simulate.jobsheet_md(side2_files[0].name, man2, st2)}

    # Carried on the manifest as well as the payload: the manifest is what the job
    # sheet and the saved .json keep, so the board a program was posted for survives
    # after the browser tab that set it has gone.
    manifest["stock_thickness_mm"] = material.thickness_mm
    manifest["thickness_overridden"] = overridden
    if getattr(material, "finish_pass", False):
        manifest["finish_allowance_mm"] = material.finish_allowance
        manifest["finish_stepped"] = bool(getattr(material, "finish_stepped", False))
    return {"thickness": material.thickness_mm, "sheets": len(files),
            "overridden": overridden,
            "nc": nc_text, "manifest": manifest, "warnings": warnings,
            "layout": layout, "viewer": _viewer_payload(nc_path, material.thickness_mm, tools),
            "nc_files": [{"name": f.name, "nc": f.read_text()} for f in files],
            "job_sheet": job_sheet, "side2": side2}


def _annotate_placement(res: Result):
    """Mark which successfully-extracted parts actually landed on a sheet (across
    all programs) so the diagnostics explain every part's final fate."""
    placed = {pp["name"] for prog in res.programs for sheet in prog["layout"]
              for pp in sheet.get("parts", [])}
    for d in res.diagnostics:
        if d.get("status") == "ok":
            d["placed"] = d["name"] in placed
            if not d["placed"]:
                d["reason"] = "extracted but did not fit on the sheet"


def _for_thickness(val, T):
    """Parting cuts / used-areas may be a per-thickness dict {thickness: items} (keyed
    by rounded mm, JSON string keys) so they DON'T bleed across thickness programs,
    or a plain list (single-thickness / legacy) applied as-is. Returns items for T."""
    if isinstance(val, dict):
        return val.get(str(T)) or val.get(T) or val.get(str(int(round(float(T))))) or []
    return val or []


def _nest_programs(parts, material, tools, sheet_w, sheet_h, blocked, prefix, sets,
                   auto_part=False, part_cuts=None, tool_map=None, engrave_pos=None, swap_xy=False,
                   border_mm=None, placements=None):
    """Group cached parts by thickness and nest one program per thickness. Parting
    cuts, used-areas and dragged placements are resolved PER thickness so a cut (or a
    pin) made on one thickness's plan never lands on another."""
    # Normally each part is cut at the thickness read from its own solid, one program
    # per thickness — an assembly in 12 and 18mm ply comes out as two programs, which
    # is what the workshop needs.
    #
    # The override says the opposite: there is ONE board on the bed and it is this
    # thick, whatever the models say. So it collapses the groups rather than merely
    # renaming them — leaving them split would post two programs claiming the same
    # stock, and the operator would cut the sheet twice. Feature depths are measured
    # down from the stock top and are unchanged; what moves is where the top is, how
    # far the through-cut has to go, and the stock top the safety gate checks against.
    override = float(getattr(material, "thickness_override", 0.0) or 0.0)
    by_t = {}
    for name, loops, meta in parts:
        T = override if override > 0 else meta.get("thickness", material.thickness_mm)
        by_t.setdefault(T, []).append((name, loops, meta))
    programs = []
    for T in sorted(by_t):
        matT = _dc_replace(material, thickness_mm=T)
        ap = auto_part.get(str(T), False) if isinstance(auto_part, dict) else bool(auto_part)
        prog = _nest_one(by_t[T], matT, tools, sheet_w, sheet_h, _for_thickness(blocked, T),
                         f"{prefix}_{T:g}mm", sets, ap, _for_thickness(part_cuts, T),
                         tool_map, engrave_pos, swap_xy=swap_xy, border_mm=border_mm,
                         placements_in=_for_thickness(placements, T) or {})
        side2 = prog.pop("side2", None)
        programs.append(prog)
        if side2:                    # the flip-jig pass, right after its pass 1
            programs.append(side2)
    return programs


def _assemble(programs, diag, jid):
    p0 = programs[0]
    res = Result(nc=p0["nc"], manifest=p0["manifest"], warnings=p0["warnings"],
                 viewer=p0["viewer"], layout=p0["layout"], job_id=jid,
                 diagnostics=diag, programs=programs)
    _annotate_placement(res)
    return res


def generate(url: str, material: Material, tools: ToolTable, *a, **k) -> Result:
    """Removed with the Onshape input layer. Use `generate_from_parts`."""
    raise NotImplementedError(
        "Onshape URL input was removed when the engine moved into the fab "
        "exporter. Geometry now arrives already extracted from the assembly "
        "STEP — call generate_from_parts(parts, diag, job_id, ...).")


def generate_from_parts(parts, diag, job_id: str, material: Material, tools: ToolTable,
                        sheet_w=2440.0, sheet_h=1220.0, prefix="JOB", blocked=None, sets=1,
                        auto_part=False, part_cuts=None, tool_map=None, engrave_pos=None,
                        swap_xy=False, border_mm=None, placements=None, exclude=None) -> Result:
    """Nest and post parts that were extracted upstream.

    This is what `generate()` was, minus the Onshape half: the exporter's STEP
    reader has already produced `parts` as [(name, loops, meta), ...], so the
    pipeline picks up at nesting. One program per thickness; `sets` cuts N
    complete assemblies (qty x sets per part); `material` is the speeds/feeds
    profile applied to every thickness.

    `exclude` = part names the operator chose not to cut. They are dropped before
    nesting and recorded in the diagnostics as a deliberate choice, so a missing
    part on the sheet never reads as a silent extraction failure.

    The parts are cached under `job_id` exactly as before, so `regenerate()`
    still re-nests with new feeds, tools or a remnant without re-reading the
    STEP. The FULL part list is cached, not the filtered one, so unticking a part
    and re-nesting brings it back without re-reading the STEP.
    """
    if not parts:
        raise ValueError("No parts to nest.")
    _cache_put(job_id, {"parts": parts, "diag": diag or []})
    diag = diag or []
    skip = set(exclude or ())
    if skip:
        parts = [p for p in parts if p[0] not in skip]
        for d in diag:
            if d.get("name") in skip:
                d["status"] = "skipped"
                d["reason"] = "excluded — not selected for cutting"
        if not parts:
            raise ValueError("Every part was excluded — nothing left to cut.")
    programs = _nest_programs(parts, material, tools, sheet_w, sheet_h, blocked, prefix,
                              sets, auto_part, part_cuts, tool_map, engrave_pos,
                              swap_xy=swap_xy, border_mm=border_mm, placements=placements)
    return _assemble(programs, diag, job_id)


def regenerate(job_id: str, material: Material, tools: ToolTable,
               sheet_w=2440.0, sheet_h=1220.0, blocked=None, prefix="JOB", sets=1,
               auto_part=False, part_cuts=None, tool_map=None, engrave_pos=None, swap_xy=False) -> Result:
    """Re-nest cached parts with new params / tool table / remnant / sets / parting
    cuts / per-feature tools / engrave positions — no Onshape round-trip. Raises
    KeyError if expired."""
    entry = _PARTS_CACHE.get(job_id)
    if not entry:
        raise KeyError("Job expired — run Generate again to re-extract parts.")
    programs = _nest_programs(entry["parts"], material, tools, sheet_w, sheet_h, blocked, prefix, sets,
                              auto_part, part_cuts, tool_map, engrave_pos, swap_xy=swap_xy)
    return _assemble(programs, entry.get("diag", []), job_id)
