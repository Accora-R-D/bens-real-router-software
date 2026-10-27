"""Router CAM: turn the wood parts of a fabrication pack into Syntec G-code.

This is the second half of the wood handoff. `wood_cam_service` maps the
exporter's STEP outlines onto the CAM engine's part tuples; here that mapping is
driven by a job, so the review page can hand a finished pack straight to the
CAM window without the operator re-uploading anything.

Flow
----
1. The operator clicks Download Pack on the review page.
2. The page POSTs its decisions to `/cam/wood/{job_id}`. If nothing came back
   as wood, the download proceeds exactly as it always did and no CAM window
   opens -- that is the whole trigger.
3. If there are wood parts, the page opens `/cam?job=...`, which loads the
   parts by job id and nests them.

The STEP is re-read rather than the parts being stashed at pack time: the job
already keeps `step_path` (the re-run path needs it, and the file is
deliberately not deleted), so there is one source of truth for the geometry
instead of two copies that can drift. The parse is cached on the job because
reading a large assembly is not cheap.
"""

from __future__ import annotations

import os
import time
import re
import sys
import tempfile
import threading
import uuid
from pathlib import Path

from fastapi import APIRouter, File, HTTPException, Request, UploadFile
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse

from backend.services import wood_cam_service as wood
from backend.services.memory_service import container_mb, give_back, limit_mb, rss_mb
from starlette.concurrency import run_in_threadpool

router = APIRouter(prefix="/cam", tags=["cam"])

_APP = Path(__file__).resolve().parents[2]
_FRONTEND = _APP / "frontend"
_CAM = _APP / "cam"

# The CAM engine's modules import each other by bare name, the way they did when
# engine/ was its own top-level directory, so both it and its parent go on the
# path. Done once, at import, rather than per request.
for _p in (str(_CAM), str(_CAM / "engine")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Every job the router holds, by id. In the fab exporter this was the exporter's
# own job table, shared so a pack's wood could be handed across in-process; the
# router now runs as its own service and keeps its own.
_JOBS: dict = {}

# The engine applies per-request config by mutating module globals under its own
# lock, which is why the service runs single-worker. Generation is serialised
# here as well so two browser windows cannot interleave a nest.
_GEN_LOCK = threading.Lock()


def _job(job_id: str) -> dict:
    job = _JOBS.get(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Unknown job")
    job["_touched"] = time.monotonic()
    return job


# ---------------------------------------------------------------- memory
# Every job used to keep its parsed STEP (OpenCascade shapes, tens of MB) and its
# 3-D simulation payload for the life of the process, and nothing ever removed
# them. Each upload + Nest left ~47 MB behind; the Render instance has 512 MB,
# so a few rounds of nesting killed the server mid-request and the page showed
# the proxy's HTML error page ("Unexpected token '<' ... is not valid JSON",
# 2026-10-12). Only the most recently used jobs now keep the heavy parts; an
# older one keeps its NC (small, still downloadable) and says it must be
# re-uploaded if anyone goes back to it.
_HEAVY_KEYS = ("_step_parts", "cam_viewer")
_KEEP_HEAVY = 2          # the job in hand plus one other (e.g. a pack's CAM handoff)


def _release_old_jobs(current_id: str) -> int:
    holders = sorted(((j.get("_touched", 0.0), jid) for jid, j in list(_JOBS.items())
                      if jid != current_id and any(k in j for k in _HEAVY_KEYS)),
                     reverse=True)
    released = 0
    for _t, jid in holders[_KEEP_HEAVY - 1:]:
        j = _JOBS.get(jid)
        if j is None:
            continue
        for k in _HEAVY_KEYS:
            j.pop(k, None)
        j["_released"] = True
        released += 1
    if released:
        give_back()
    return released


_RELEASED_MSG = ("This job was cleared from the server's memory to make room for "
                 "newer ones. Drop the STEP files in again to carry on.")


def _step_parts(job_id: str):
    """Every part in the job's STEP, parsed once and remembered."""
    job = _job(job_id)
    cached = job.get("_step_parts")
    if cached is not None:
        return cached
    path = job.get("step_path")
    if job.get("standalone"):
        # An upload has nothing on disk to re-read from: its parts went when
        # the job was released.
        raise HTTPException(status_code=410, detail=_RELEASED_MSG)
    _release_old_jobs(job_id)
    if not path or not Path(path).exists():
        raise HTTPException(
            status_code=409,
            detail="This pack has no STEP on file. The CAM handoff only works "
                   "for packs built from an uploaded assembly STEP.")
    try:
        from backend.services import step_ingest_service as ingest
        parts = ingest.read_step(str(path))
    except ImportError:
        raise HTTPException(status_code=503,
                            detail="OpenCascade is not installed on the server.")
    except Exception as exc:
        raise HTTPException(status_code=500,
                            detail=f"Could not re-read the STEP: {exc}")
    job["_step_parts"] = parts
    return parts


# ---------------------------------------------------------------- standalone
# The CAM window is opened from the exporter home page and fed STEP files by
# hand. Two kinds arrive:
#
#   * one solid per file — the pack's "Wood Files" folder. Those STEPs carry no
#     product name (the pack writes the bare solid), so the NAME and QUANTITY
#     ride on the filename the pack wrote them with ("18mm_2off_Part_61.step");
#     a file named without them counts as 1 off.
#   * a whole assembly exported from Onshape. Its STEP names every part and
#     places every occurrence, so the names and quantities come from INSIDE the
#     file — "Seat Board" x2, "Back Board" x1 — and the filename only names the
#     job. Before this every part was called "<assembly> (1)", "(2)", ... and
#     that is what was engraved on it.
#
# Thickness comes from the geometry regardless.

_QTY_RE = re.compile(r"(?:^|[_\s-])(\d+)\s*off(?=$|[_\s-])", re.I)
_THK_PREFIX_RE = re.compile(r"^\s*\d+(?:\.\d+)?\s*mm[_\s-]*", re.I)
_MAX_UPLOAD_MB = 80


def _parse_upload_name(stem: str) -> tuple[str, int]:
    """('Part_61', 2) from '18mm_2off_Part_61' — and (stem, 1) when the name
    carries no quantity."""
    qty = 1
    m = _QTY_RE.search(stem)
    if m:
        qty = max(1, min(500, int(m.group(1))))
        stem = (stem[:m.start()] + " " + stem[m.end():])
    stem = _THK_PREFIX_RE.sub("", stem)
    name = stem.strip(" _-") or "part"
    return name, qty


def upload_title(filenames: "list[str]") -> str:
    """The job's name, from what was dropped: one file's stem, or what several
    files' stems share. It leads every program's filename so a folder of .NC
    files sorts by job, not by "WOOD_18mm" forty times over."""
    stems = [Path(f).stem.strip() for f in filenames if f]
    stems = [s for s in stems if s]
    if not stems:
        return ""
    if len(stems) == 1:
        return stems[0]
    common = os.path.commonprefix(stems).rstrip(" _-0123456789.")
    # a pack's Wood Files share only their thickness ("18mm_") — no name in that
    common = _THK_PREFIX_RE.sub("", common).strip(" _-")
    return common if len(common) >= 3 else ""


def safe_prefix(title, fallback: str = "WOOD") -> str:
    """A program-file prefix from a job title: letters, digits and single
    underscores, at most 40 long, never empty."""
    s = re.sub(r"[^A-Za-z0-9]+", "_", str(title or "")).strip("_")
    s = re.sub(r"_{2,}", "_", s)
    if len(s) > 40:                      # cut at a word, not mid-token
        s = s[:40]
        s = s.rsplit("_", 1)[0] if "_" in s else s
    return s.strip("_") or fallback


def ingest_uploads(files: "list[tuple[str, bytes]]") -> dict:
    """Read uploaded STEP files into a standalone CAM job. Returns the job dict
    to store; raises ValueError with an operator-readable reason."""
    from backend.services import step_ingest_service as ingest
    parts, errors = [], []
    seen = set()
    for fname, data in files:
        stem = Path(fname).stem
        if not re.search(r"\.(step|stp)$", fname, re.I):
            errors.append(f"{fname}: not a STEP file")
            continue
        if len(data) > _MAX_UPLOAD_MB * 1024 * 1024:
            errors.append(f"{fname}: over {_MAX_UPLOAD_MB}MB")
            continue
        name, qty = _parse_upload_name(stem)
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as tf:
                tf.write(data)
                tmp = tf.name
            read = ingest.read_step(tmp)
        except Exception as exc:                        # noqa: BLE001
            errors.append(f"{fname}: could not read ({str(exc)[:80]})")
            continue
        finally:
            # The parsed shapes are all that is kept; the copy on disk was
            # never deleted and the server's temp folder filled with them.
            if tmp:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
        if not read:
            errors.append(f"{fname}: no solid found")
            continue
        for k, part in enumerate(read):
            part.source = stem                   # which file — for assembly initials
            if len(read) == 1:
                # One part: the filename is its name and quantity (see above).
                part.name = name
                part.qty = qty
            else:
                # An assembly: the STEP's own part names and occurrence counts.
                # The reader has already folded identical bodies into one part
                # with its quantity; a "2off" on the filename means two of the
                # whole assembly, so it multiplies. A body the STEP left
                # unnamed gets the file's name and a number, as before.
                own = (part.name or "").strip()
                part.name = own if own and not own.startswith("unnamed") \
                    else f"{name} ({k + 1})"
                part.qty = max(1, int(part.qty or 1)) * qty
                part.merged_qty = {k: int(v) * qty for k, v in (part.merged_qty or {}).items()}
            while part.name in seen:
                part.name += " *"
            seen.add(part.name)
            parts.append(part)
    if not parts:
        raise ValueError("; ".join(errors) or "no usable parts in the upload")
    return {"wood_names": sorted(p.name for p in parts),
            "_step_parts": parts, "standalone": True,
            "upload_errors": errors,
            "title": upload_title([f for f, _d in files])}


@router.post("/upload")
async def cam_upload(files: "list[UploadFile]" = File(...)) -> JSONResponse:
    """Start a standalone CAM job from dropped STEP files."""
    payload = []
    for f in files:
        payload.append((f.filename or "part.step", await f.read()))
    try:
        # Reading a STEP is seconds of CPU. On the event loop it froze the whole
        # single-worker server — every page, poll and other user — until done.
        job = await run_in_threadpool(ingest_uploads, payload)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    job_id = "camup-" + uuid.uuid4().hex[:10]
    job["_touched"] = time.monotonic()
    _JOBS[job_id] = job
    _release_old_jobs(job_id)
    out = await run_in_threadpool(wood.summary, job["_step_parts"],
                                  names=set(job["wood_names"]))
    await run_in_threadpool(give_back)       # the STEP reader's scratch memory
    out.update({"job_id": job_id, "warning": wood.missing_pocket_warning(),
                "upload_errors": job.get("upload_errors") or []})
    return JSONResponse(out)


@router.post("/wood/{job_id}")
async def cam_wood(job_id: str, request: Request) -> JSONResponse:
    """Which parts of this pack are wood, given the operator's decisions.

    Called by Download Pack before it downloads. A zero count is the normal,
    expected answer for a steel-only assembly and is not an error -- the caller
    just carries on downloading.
    """
    decisions = await request.json()
    names = wood.wood_names(decisions)
    job = _job(job_id)
    job["wood_names"] = sorted(names)
    if not names:
        return JSONResponse({"count": 0, "pieces": 0, "thicknesses_mm": [],
                             "names": []})
    # _step_parts reads the whole assembly STEP the first time (seconds of CPU,
    # ~15 s on the server for a 7 MB assembly): on a worker thread with the rest,
    # not inline as an argument — that ran it on the event loop and froze the
    # server, polls included, while the pack's wood went to the router.
    summary = await run_in_threadpool(lambda: wood.summary(_step_parts(job_id), names=names))
    summary["warning"] = wood.missing_pocket_warning()
    summary["title"] = job.get("title") or ""
    return JSONResponse(summary)


@router.get("/wood/{job_id}")
async def cam_wood_get(job_id: str) -> JSONResponse:
    """The stored wood selection, for the CAM window to load on open."""
    job = _job(job_id)
    names = set(job.get("wood_names") or [])
    if not names:
        return JSONResponse({"count": 0, "pieces": 0, "thicknesses_mm": [],
                             "names": []})
    # _step_parts reads the whole assembly STEP the first time (seconds of CPU,
    # ~15 s on the server for a 7 MB assembly): on a worker thread with the rest,
    # not inline as an argument — that ran it on the event loop and froze the
    # server, polls included, while the pack's wood went to the router.
    summary = await run_in_threadpool(lambda: wood.summary(_step_parts(job_id), names=names))
    summary["warning"] = wood.missing_pocket_warning()
    summary["title"] = job.get("title") or ""
    return JSONResponse(summary)


@router.get("/memory")
async def cam_memory() -> JSONResponse:
    """How much memory the server is using, and what is holding it — open this
    on the live app when the router misbehaves (the instance has 512 MB)."""
    jobs = list(_JOBS.values())
    return JSONResponse({
        "rss_mb": rss_mb(),
        "container_mb": container_mb(),       # web server + export subprocess
        "limit_mb": limit_mb(),
        "mmgt_opt": os.environ.get("MMGT_OPT", ""),
        "jobs": len(jobs),
        "jobs_holding_geometry": sum(1 for j in jobs if any(k in j for k in _HEAVY_KEYS)),
        "keep_geometry_for": _KEEP_HEAVY,
    })


@router.get("/defaults")
async def cam_defaults() -> JSONResponse:
    """Stock materials and the 10-seat tool table.

    Served rather than hardcoded in the page so the speeds/feeds the operator
    sees are the ones the engine will actually apply.
    """
    from dataclasses import asdict
    from router_cam.models import DEFAULT_MATERIALS, DEFAULT_TOOL_TABLE
    return JSONResponse({
        "materials": [asdict(m) for m in DEFAULT_MATERIALS],
        "seats": [asdict(s) for s in DEFAULT_TOOL_TABLE.seats],
    })


_DOWNLOAD_COUNTER_RE = re.compile(r"\s*\(\d+\)\s*$")


def assembly_initials(sources) -> dict:
    """{source title: short initials} — "Foot End Complete" -> "FEC". Titles
    whose initials clash ("Head_End lift" / "Head_End leg" are both HEL) get
    more letters of their last word until they differ: HELI / HELE. Counters
    a browser adds to a re-download (" (2)") and size/qty prefixes are ignored."""
    def words(src):
        stem = _DOWNLOAD_COUNTER_RE.sub("", str(src or ""))
        stem = _THK_PREFIX_RE.sub("", stem)
        stem = re.sub(r"(?:^|[_\s-])\d+\s*off(?=$|[_\s-])", " ", stem, flags=re.I)
        return [w for w in re.split(r"[^A-Za-z0-9]+", stem) if w]

    uniq = [s for s in dict.fromkeys(s for s in sources if s)]
    ws = {s: words(s) for s in uniq}

    def make(s, k):
        w = ws[s]
        if not w:
            return ""
        head = "".join(x if x.isdigit() else x[0] for x in w[:-1])
        last = w[-1] if w[-1].isdigit() else w[-1][:k]      # numbers stay whole
        return (head + last).upper()

    out = {s: make(s, 1) for s in uniq}
    for k in range(2, 12):
        clash = {}
        for s, a in out.items():
            clash.setdefault(a, []).append(s)
        dup = [ss for ss in clash.values() if len(ss) > 1]
        if not dup:
            break
        for group in dup:
            for s in group:
                out[s] = make(s, k)
    return out


def _split_identical(parts, step_parts, skip=(), qtys=None):
    """[(name, loops, meta)] with every merged part split back into its original
    STEP names (see cam_generate). Loops are deep-copied per entry: nesting
    moves them in place, and two entries sharing one would both move it."""
    import copy
    by_name = {p.name: p for p in step_parts}
    taken = {name for name, _l, _m in parts}

    def edited(name):
        # The page sends every part's quantity, pre-filled with the reader's
        # count; only one the operator CHANGED is an edit.
        if name not in (qtys or {}):
            return False
        try:
            return int(qtys[name]) != int(getattr(by_name.get(name), "qty", 0) or 0)
        except (TypeError, ValueError):
            return False

    out = []
    for name, loops, meta in parts:
        merged = dict(getattr(by_name.get(name), "merged_qty", None) or {})
        if not merged or name in skip or edited(name):
            out.append((name, loops, meta))
            continue
        own = int(meta.get("qty") or 1) - sum(merged.values())
        if own >= 1:
            first = dict(meta, qty=own, label=name)
            out.append((name, loops, first))
        for other, q in merged.items():
            label = other
            while other in taken:
                other += " *"
            taken.add(other)
            m2 = copy.deepcopy(meta)
            m2.update(qty=int(q), label=label)
            out.append((other, copy.deepcopy(loops), m2))
    return out


@router.post("/generate/{job_id}")
async def cam_generate(job_id: str, request: Request) -> JSONResponse:
    """Nest the job's wood parts and post them to Syntec G-code."""
    body = await request.json()
    job = _job(job_id)
    names = set(job.get("wood_names") or [])
    if not names:
        raise HTTPException(status_code=409, detail="No wood parts on this job.")
    # Make room before the heavy part, not after it: the peak is what kills.
    _release_old_jobs(job_id)

    from router_cam.engine import generate_from_parts
    from router_cam.models import (DEFAULT_TOOL_TABLE, Material, clamp_material,
                                   tooltable_from_list, DEFAULT_MATERIALS)

    # The heavy part of generate — reading every part's geometry, then nesting,
    # posting and simulating — runs on a worker thread, not the event loop: the
    # server stays able to answer the page (and everyone else) meanwhile.
    parts, diag = await run_in_threadpool(wood.cam_parts_and_diag,
                                          _step_parts(job_id), names=names)
    if not parts:
        raise HTTPException(status_code=422,
                            detail="No wood part yielded a usable outline.")

    # Editable quantities from the parts list — the filename (or the assembly)
    # said one thing, the operator may want another for this run.
    qtys = body.get("quantities") or {}
    if isinstance(qtys, dict):
        for name, loops, meta in parts:
            if name in qtys:
                try:
                    meta["qty"] = max(1, min(500, int(qtys[name])))
                except (TypeError, ValueError):
                    pass

    # "Name identical parts separately": the reader folds identical bodies into
    # one part (one file, N off), which is right for cutting but engraves every
    # copy with the first one's name. Ticked, each folded-in body comes back as
    # its own entry under its own STEP name and count, so each is engraved with
    # its own name. Unticked parts stay out, and a part whose quantity was edited
    # on the Parts tab is left whole: the operator's number for it stands.
    # Remember each part's assembly before any split, so split entries carry it.
    by_src = {p.name: (getattr(p, "source", "") or job.get("title") or "")
              for p in _step_parts(job_id)}
    for name, _loops, meta in parts:
        meta["_source"] = by_src.get(name, "")

    if (body.get("material") or {}).get("unique_names"):
        parts = _split_identical(parts, _step_parts(job_id),
                                 skip=set(body.get("exclude") or ()),
                                 qtys=qtys if isinstance(qtys, dict) else {})

    # "Add assembly initials to engraved names": parts read from several files
    # can share a name (X1 in the head-end lift AND the foot-end leg), and the
    # duplicates were told apart only by " *" suffixes in upload order — which
    # nobody can map back to an assembly once the parts are off the sheet.
    # Ticked, each engraving reads "<name> <initials of its file>", e.g.
    # "X1 FEC"; the " *" markers are dropped since the initials do that job.
    if (body.get("material") or {}).get("engrave_assembly"):
        acr = assembly_initials([m.get("_source", "") for _n, _l, m in parts])
        for _name, _loops, meta in parts:
            a = acr.get(meta.get("_source", ""), "")
            base = re.sub(r"(\s*\*)+$", "", str(meta.get("label") or _name)).strip()
            meta["label"] = f"{base} {a}".strip()

    # Every depth-bearing number in here arrives from a browser form. Clamp before
    # anything sees it, and keep the notes: a value that had to be moved is reported
    # back rather than quietly applied, so the operator is never shown one depth and
    # given another. See models._LIMITS for why this is not merely defensive.
    mat_in = body.get("material")
    if mat_in:
        try:
            raw = Material(**{k: v for k, v in mat_in.items()
                              if k in Material.__dataclass_fields__})
        except (TypeError, ValueError) as exc:
            raise HTTPException(status_code=422, detail=f"Bad material: {exc}")
        material, clamp_notes = clamp_material(raw)
    else:
        material, clamp_notes = DEFAULT_MATERIALS[0], []
    tools_in = body.get("tools")
    tools = tooltable_from_list(tools_in) if tools_in else DEFAULT_TOOL_TABLE

    # Portrait by default, matching the bed: 1220 across, 2440 up.
    sheet_w = float(body.get("sheet_w") or 1220.0)
    sheet_h = float(body.get("sheet_h") or 2440.0)
    # Border: the clear band between the outermost toolpath and the sheet edge. Capped
    # at a quarter of the short side so a fat border can never shrink the usable area
    # to nothing (which would read as "nothing fits" rather than "border too big").
    try:
        border_mm = float(body.get("border_mm", 20.0))
    except (TypeError, ValueError):
        border_mm = 20.0
    border_mm = min(max(border_mm, 0.0), min(sheet_w, sheet_h) / 4.0)

    # Each part keeps its OWN thickness. The engine groups by thickness and cuts
    # one program per group, applying the chosen material's feeds and speeds at
    # that group's thickness — so an assembly in 12 mm and 18 mm ply comes out as
    # two programs, which is what the workshop needs. Forcing every part onto the
    # material's thickness (as this did) silently cut everything from one board.
    def _generate():
        with _GEN_LOCK:
            return generate_from_parts(
                parts, diag, f"cam-{job_id}", material, tools,
                sheet_w=sheet_w, sheet_h=sheet_h,
                # Programs lead with the job's name — the assembly that was
                # dropped in, or the STEP the pack was built from — so a folder
                # of .NC files sorts by job. The page sends it; the job carries
                # it anyway in case an older page does not.
                prefix=safe_prefix(body.get("prefix") or job.get("title")),
                sets=int(body.get("sets") or 1),
                auto_part=bool(body.get("auto_part")),
                part_cuts=body.get("part_cuts") or None,
                tool_map=body.get("tool_map") or None,
                swap_xy=bool(body.get("swap_xy")),
                border_mm=border_mm,
                placements=body.get("placements") or None,
                exclude=body.get("exclude") or None)

    try:
        res = await run_in_threadpool(_generate)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    finally:
        # Nesting, posting and simulating leave ~150 MB of freed scratch
        # behind; hand it back before the next request needs it.
        await run_in_threadpool(give_back)

    # One NC per program, keyed so each can be downloaded on its own. Usually one
    # per thickness; a two-sided (flip-jig) job adds a "-side2" program that is
    # run after the operator flips the part into its mould.
    def _pkey(p):
        return f"{p['thickness']:g}" + ("-side2" if p.get("side2") else "")
    job["cam_nc"] = {_pkey(p): p["nc"] for p in (res.programs or [])}
    # The gate verdict is kept server-side too. The download endpoint consults THIS,
    # not a flag the browser sends, so a failed program cannot be fetched by a stale
    # tab or a hand-made URL without saying so explicitly.
    job["cam_gate"] = {_pkey(p): (p.get("manifest") or {}).get("safety_gate", "?")
                       for p in (res.programs or [])}
    # The 3D viewer payload is megabytes of height-field and toolpath. It is kept here
    # and fetched per thickness by the simulation tab, so the generate response — which
    # the operator waits on — stays small.
    job["cam_viewer"] = {_pkey(p): p.get("viewer") or {}
                         for p in (res.programs or [])}
    # Whether each program was posted against an overridden board thickness. Kept
    # server-side for the same reason the gate verdict is: the download names the
    # file from THIS, not from anything the browser sends back.
    job["cam_override"] = {_pkey(p): bool(p.get("overridden"))
                           for p in (res.programs or [])}

    return JSONResponse({
        "job_id": res.job_id,
        "diagnostics": res.diagnostics,
        "clamped": clamp_notes,
        "border_mm": border_mm,
        "programs": [{
            "thickness": p["thickness"],
            "key": _pkey(p),
            "side2": bool(p.get("side2")),
            "label": f"{p['thickness']:g}mm" + (" — side 2" if p.get("side2") else ""),
            "nc_lines": len(p["nc"].splitlines()),
            "manifest": {k: v for k, v in (p["manifest"] or {}).items()
                         if k != "layout"},
            "warnings": list(p.get("warnings") or []),
            "layout": p.get("layout") or [],
            "sheets": p.get("sheets"),
            "job_sheet": p.get("job_sheet") or "",
            "has_viewer": bool(p.get("viewer")),
        } for p in (res.programs or [])],
    })


@router.get("/viewer/{job_id}")
async def cam_viewer(job_id: str, thickness: str = "") -> JSONResponse:
    """The material-removal payload for one thickness, for the simulation tab.

    Served separately from /generate because it is large (a height-field grid plus
    the decimated toolpath) and is only wanted when the operator opens the tab.
    """
    job = _job(job_id)
    viewers = job.get("cam_viewer") or {}
    if not viewers:
        if job.get("_released"):
            raise HTTPException(status_code=410, detail=_RELEASED_MSG)
        raise HTTPException(status_code=404, detail="Nothing generated yet.")
    key = thickness or next(iter(viewers))
    if key not in viewers:
        raise HTTPException(status_code=404, detail=f"No program for {key}mm on this job.")
    if not viewers[key]:
        raise HTTPException(status_code=409,
                            detail="This program produced no cutting moves to simulate.")
    return JSONResponse(viewers[key])


@router.get("/nc/{job_id}")
async def cam_nc(job_id: str, thickness: str = "",
                 acknowledge_gate_failure: bool = False) -> PlainTextResponse:
    """One thickness's program, as a download.

    A program whose safety gate FAILED is not handed out on a plain request. The
    gate is what stands between the operator and a cutter driven too deep or off
    the sheet, and it used to be advisory here: the NC was written unconditionally
    and only the UI showed the verdict, so a stale tab or a typed URL could pull a
    failing program with nothing said. Fetching one now takes a deliberate,
    explicit acknowledgement.
    """
    job = _job(job_id)
    programs = job.get("cam_nc") or {}
    if not programs:
        raise HTTPException(status_code=404, detail="Nothing generated yet.")
    key = thickness or next(iter(programs))
    if key not in programs:
        raise HTTPException(status_code=404,
                            detail=f"No program for {key}mm on this job.")
    gate = (job.get("cam_gate") or {}).get(key, "?")
    if gate != "PASS" and not acknowledge_gate_failure:
        raise HTTPException(
            status_code=409,
            detail=f"The {key}mm program did not pass the safety gate ({gate}). "
                   f"Do not send it to the machine. Fix the reported reasons and "
                   f"re-generate; download it only if you know why you need it.")
    # The thickness is in the filename because that is the one piece of the job that
    # travels to the machine on its own — and OVERRIDE is in it when the board was
    # not the one the models describe, so the operator reads it off the USB stick
    # rather than having to remember a checkbox from the browser.
    mark = "_OVERRIDE" if (job.get("cam_override") or {}).get(key) else ""
    # Led by the job's name — the assembly dropped in, or the STEP the pack was
    # built from — so the programs on the USB stick sort by job, not as forty
    # files all called WOOD_18mm.
    lead = safe_prefix(job.get("title"))
    fname = (f"{lead}_{key[:-6]}mm{mark}_SIDE2_{job_id[:8]}.NC" if key.endswith("-side2")
             else f"{lead}_{key}mm{mark}_{job_id[:8]}.NC")
    return PlainTextResponse(
        programs[key],
        headers={"Content-Disposition": f'attachment; filename="{fname}"'})


@router.get("")
@router.get("/")
async def cam_page() -> FileResponse:
    """The CAM window itself."""
    page = _FRONTEND / "cam.html"
    if not page.exists():
        raise HTTPException(status_code=500, detail="frontend/cam.html not found")
    # Never cached, like the exporter's own page: after a deploy a cached copy
    # hid new options until the operator knew to press Ctrl+F5 (2026-10-26).
    return FileResponse(str(page), media_type="text/html",
                        headers={"Cache-Control": "no-store, max-age=0"})
