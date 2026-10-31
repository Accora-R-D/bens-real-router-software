"""Hand wood parts from the fabrication pack to the router CAM engine.

Why this exists
---------------
The exporter already reads an assembly STEP and pulls a flat outline for every
plate it finds. The router CAM engine already turns flat outlines into nested
Syntec G-code. They were built against different input shapes, so this module is
the seam: it takes the parts the operator classified (or overrode) as `wood` on
the review page and re-expresses them in the exact tuple shape
`router_cam.engine` expects.

Nothing downstream is adapted to us. The 2.5D path is byte-validated against
proven Syntec files, so the rule is that the CAM engine is never modified to
accommodate the exporter -- the records are made to fit it instead.

The shape being produced
------------------------
`router_cam.engine._extract_parts()` returns `[(name, loops, meta), ...]` and
everything after it -- nesting, the post, the safety gate, the simulator -- is
pure geometry. So that tuple is the whole contract:

    loops  [{role: OUTSIDE|INSIDE|DRILL, pts: [(x, y), ...], circle, isOuter}]
    meta   {flip, two_sided, up_side, pockets, label, qty, thickness}

Two sources of geometry
-----------------------
Preferred is `wood_geometry.part_geometry`, which reads the SOLID and so brings
pockets, their depths and their islands with it, plus which face to machine.

The fallback is the exporter's flat outline, which sees one planar face and so
carries through-features only. It is used when the solid cannot be read
confidently -- a machined face split into coplanar fragments by its own
recesses, a shape that is not a clean plate, or no OpenCascade at all. That is a
real difference in what gets cut, so it is recorded per part
(`pocket_source`, and a `reason`) and surfaced rather than left to be discovered
at the machine.
"""

from __future__ import annotations

import math

WOOD = "wood"

# A loop is a drill hole if every sampled point sits on one circle to within
# this. The outline sampler works to CHORD_TOL_MM (0.05), so the sagitta of a
# chord already costs most of that; 0.12 accepts real holes without letting a
# rounded-corner slot through as a circle.
CIRCLE_TOL_MM = 0.12

# Below this a "circle" is more likely a sampling artefact than a hole.
MIN_HOLE_R_MM = 0.6


def wood_names(decisions) -> set:
    """Names the operator ended up with as wood, however they got there.

    `final_process` is already the resolved value -- the review page sets it to
    the override where there is one and the prediction otherwise -- so a part
    recognised as wood and a part corrected to wood are the same thing here,
    which is what was asked for.
    """
    out = set()
    for d in decisions or []:
        if (d.get("final_process") or "").strip().lower() == WOOD:
            name = (d.get("name") or "").strip()
            if name:
                out.add(name)
    return out


def _signed_area(pts) -> float:
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return a / 2.0


def _open(pts) -> list:
    """Drop a repeated closing vertex.

    The exporter closes its loops by repeating the first point, which is what a
    DXF polyline wants. The CAM offsetter treats the list as implicitly closed,
    so leaving it in hands pyclipper a zero-length final segment.
    """
    out = [(float(x), float(y)) for x, y in pts]
    while len(out) > 1 and math.hypot(out[0][0] - out[-1][0],
                                      out[0][1] - out[-1][1]) < 1e-9:
        out.pop()
    return out


def _circle(pts):
    """(cx, cy, r) if these points lie on one circle, else None.

    Algebraic (Kasa) fit: exact for a true circle, and cheap. Solving for the
    centre rather than assuming the centroid matters because the outline is
    sampled by deflection, so the points are not evenly spaced around the hole
    and the centroid sits off-centre on a partial arc.
    """
    if len(pts) < 6:
        return None
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = szx = szy = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sz += z
        szx += z * x
        szy += z * y
    # normal equations for  z + D x + E y + F = 0
    a11, a12, a13 = sxx, sxy, sx
    a21, a22, a23 = sxy, syy, sy
    a31, a32, a33 = sx, sy, float(n)
    b1, b2, b3 = -szx, -szy, -sz
    det = (a11 * (a22 * a33 - a23 * a32)
           - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31))
    if abs(det) < 1e-9:
        return None
    D = (b1 * (a22 * a33 - a23 * a32) - a12 * (b2 * a33 - a23 * b3)
         + a13 * (b2 * a32 - a22 * b3)) / det
    E = (a11 * (b2 * a33 - a23 * b3) - b1 * (a21 * a33 - a23 * a31)
         + a13 * (a21 * b3 - b2 * a31)) / det
    F = (a11 * (a22 * b3 - b2 * a32) - a12 * (a21 * b3 - b2 * a31)
         + b1 * (a21 * a32 - a22 * a31)) / det
    cx, cy = -D / 2.0, -E / 2.0
    r2 = cx * cx + cy * cy - F
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    if r < MIN_HOLE_R_MM:
        return None
    for x, y in pts:
        if abs(math.hypot(x - cx, y - cy) - r) > CIRCLE_TOL_MM:
            return None
    return (cx, cy, r)


def cam_loops(edges) -> list:
    """Exporter outline loops -> CAM role-tagged loops.

    The exporter returns the outer boundary and every hole in one list and lets
    the caller work out which is which, so that is done here: largest enclosed
    area is the profile, everything else is interior. Interior loops that are
    circles become DRILL (the post then drills or helical-bores them rather than
    tracing a polygon); the rest become INSIDE cut-outs.
    """
    rings = [_open(lp) for lp in edges or []]
    rings = [r for r in rings if len(r) >= 3]
    if not rings:
        return []
    areas = [abs(_signed_area(r)) for r in rings]
    outer_i = max(range(len(rings)), key=lambda i: areas[i])

    loops = []
    for i, ring in enumerate(rings):
        if i == outer_i:
            loops.append({"role": "OUTSIDE", "pts": ring, "circle": None,
                          "isOuter": True})
            continue
        circ = _circle(ring)
        if circ:
            loops.append({"role": "DRILL", "pts": ring, "circle": circ,
                          "isOuter": False})
        else:
            loops.append({"role": "INSIDE", "pts": ring, "circle": None,
                          "isOuter": False})
    return loops


def cam_parts_and_diag(step_parts, decisions=None, names=None, mode="flat"):
    """`(parts, diag)` — the CAM tuples, and a row for EVERY wood part.

    The diagnostics matter as much as the parts. The CAM engine fills in a
    `placed` flag afterwards, so a part that extracted cleanly but did not fit
    on the sheet shows up as "did not fit" instead of quietly not being cut.
    Without a row per part there is nothing for it to annotate, and a missing
    part becomes invisible.

    `mode` "3d" reads every part as a shaped heightfield (`wood_3d`) instead
    of a plate; a flat job never goes near that code.
    """
    want = names if names is not None else wood_names(decisions)
    out, diag = [], []
    for p in step_parts:
        if want is not None and p.name not in want:
            continue
        thickness = float(getattr(p, "thickness_mm", 0.0) or 0.0)
        qty = int(getattr(p, "qty", 1) or 1)
        row = {"name": p.name, "thickness_mm": thickness, "is_sheet": True,
               "qty": qty, "status": "pending", "reason": ""}
        diag.append(row)

        if mode == "3d":
            from .wood_3d import read_geometry_3d, looks_3d, plate_model
            shape = getattr(p, "shape", None)
            if shape is not None and not looks_3d(shape):
                # A plain plate in a 3D job (stays and rockers dropped in with a
                # shaped part): the flat reader lays it on its face and cuts its
                # profile, pockets, cut-outs and holes — not a heightfield roughed
                # layer by layer and ball-finished all over.
                loops, geom, why = _read_geometry(p)
                if geom and geom.get("frame"):
                    try:
                        outer = next(lp for lp in loops if lp["role"] == "OUTSIDE")
                        outer["model"] = plate_model(shape, geom)
                    except Exception:                      # noqa: BLE001 - view only
                        pass
                    why = "; ".join(w for w in ("a flat plate — cut as a board", why) if w)
                else:          # the plate read failed: surface it rather than outline it
                    loops, geom, why = read_geometry_3d(p)
            else:
                loops, geom, why = read_geometry_3d(p)
        else:
            loops, geom, why = _read_geometry(p)
        if not any(lp["role"] == "OUTSIDE" for lp in loops):
            row["status"] = "failed"
            row["reason"] = why or "no outer profile could be read"
            continue

        pockets = [lp for lp in loops if lp["role"] == "POCKET"]
        row.update(status="ok",
                   profiles=sum(1 for lp in loops if lp["role"] == "OUTSIDE"),
                   cutouts=sum(1 for lp in loops if lp["role"] == "INSIDE"),
                   holes=sum(1 for lp in loops if lp["role"] == "DRILL"),
                   pockets=len(pockets),
                   pocket_source="solid" if geom else "flat outline")
        if geom is None or mode == "3d":
            row["reason"] = why or ""

        if geom:
            thickness = float(geom.get("thickness") or thickness)
            row["thickness_mm"] = thickness
        meta = {
            "flip": bool(geom["flip"]) if geom else False,
            "two_sided": bool(geom["two_sided"]) if geom else False,
            "up_side": geom["up_side"] if geom else "TOP",
            "pockets": geom["pockets"] if geom else [],
            "label": (getattr(p, "part_number", "") or p.name),
            "qty": qty,
            "thickness": thickness,
        }
        if mode == "3d":
            # The label would be placed by the footprint and could land on a
            # slope, engraving air or half-letters. Not engraved for now.
            meta["no_engrave"] = True
        # Two-sided 2.5D parts: read the part AGAIN from its other face. That
        # read comes back in the far face's own outward frame — the mirrored
        # view the machine sees once the part is flipped — and drives the
        # flip-jig second pass: pass 1 also cuts an inverse mould (the far-face
        # outline as a snug through-cavity) somewhere on the sheet; the
        # operator flips the finished part into it and runs pass 2, which cuts
        # only the far face's recesses. Through-features are already cut in
        # pass 1, so only OUTSIDE (the mould shape) and POCKETs are kept here.
        if geom and geom.get("two_sided"):
            back_side = "BOT" if geom["up_side"] == "TOP" else "TOP"
            try:
                back = wood_geometry_module().part_geometry(
                    getattr(p, "shape", None), p.name, force_side=back_side)
                if back.get("unmatched_pockets"):
                    raise wood_geometry_module().GeometryError(
                        f"{back['unmatched_pockets']} far-side recess(es) had "
                        "no matching opening")
                back_loops = [lp for lp in back["loops"]
                              if lp["role"] in ("OUTSIDE", "POCKET", "SURFACE")]
                if len(back_loops) > 1:        # an outline AND something to cut
                    meta["backside_loops"] = back_loops
                    # The NEAR side's regions in the same mirrored frame — they
                    # say where the flipped part is thinner than full stock, so
                    # the mould builder can sculpt a supporting floor when the
                    # full-thickness contact patch is too small to sit on.
                    meta["backside_far"] = back.get("far_pockets") or []
                    row["reason"] = ("pockets on both faces — pass 1 cuts the "
                                     "face-up side plus an inverse mould; flip "
                                     "the part into the mould and run pass 2")
                else:
                    row["reason"] = ("pockets on both faces but the far side "
                                     "read empty — only the up side is cut")
            except Exception as exc:                       # noqa: BLE001
                row["reason"] = ("pockets on both faces — far side could not "
                                 f"be read ({str(exc)[:80]}); only the up "
                                 "side is cut, the rest needs a second setup")
        out.append((p.name, loops, meta))
    return out, diag


def wood_geometry_module():
    from . import wood_geometry
    return wood_geometry


def _read_geometry(part):
    """(loops, geom_or_None, why) — solid first, flat outline as the fallback.

    The solid is preferred because it is the only source that carries pockets.
    Falling back is not a failure, but it IS a different cut, so the reason
    comes back with it rather than being swallowed.
    """
    try:
        from . import wood_geometry
        geom = wood_geometry.part_geometry(getattr(part, "shape", None), part.name)
        # Judge the extraction by its OUTCOME, not by a proxy. `fragmented` counts
        # faces coplanar with the machined side, and the top of an island in a
        # pocket is coplanar with it by definition — so rejecting on that flag
        # threw away precisely the parts islands exist for. What actually matters
        # is whether every recess found on the machined side was matched to an
        # opening: if any went unmatched, a pocket would be missing from the cut.
        if geom.get("unmatched_pockets"):
            raise wood_geometry.GeometryError(
                f"{geom['unmatched_pockets']} recess(es) had no matching opening "
                "on the machined face — cut as through-features only")
        return geom["loops"], geom, ""
    except ImportError:
        why = "OpenCascade not available — through-features only"
    except Exception as exc:
        why = f"{type(exc).__name__}: {exc}"[:160]
    # The through-features-only fallback needs an OUTLINE — but wood parts
    # never get one at ingest (edges are extracted for laser parts only), so
    # this fallback used to return nothing and the whole job died with "no
    # wood part yielded a usable outline". Extract it from the solid now, the
    # same way a review override to flat does.
    if not getattr(part, "edges", None) and getattr(part, "shape", None) is not None:
        try:
            from backend.services.step_ingest_service import ensure_flat_outline
            err = ensure_flat_outline(part)
            if err:
                why = f"{why}; outline fallback: {err}"[:200]
        except Exception as exc:                            # noqa: BLE001
            why = f"{why}; outline fallback failed: {str(exc)[:60]}"[:200]
    return cam_loops(getattr(part, "edges", None)), None, why


def cam_parts(step_parts, decisions=None, names=None) -> list:
    """Just the CAM tuples. See `cam_parts_and_diag` for the reportable version."""
    return cam_parts_and_diag(step_parts, decisions, names)[0]


def summary(step_parts, decisions=None, names=None) -> dict:
    """What the UI needs in order to decide whether to open the CAM window."""
    want = names if names is not None else wood_names(decisions)
    parts = ([p for p in step_parts if p.name in want]
             if want is not None else list(step_parts))
    thicknesses = sorted({round(float(getattr(p, "thickness_mm", 0) or 0), 1)
                          for p in parts if getattr(p, "thickness_mm", 0)})
    return {
        "count": len(parts),
        "pieces": sum(int(getattr(p, "qty", 1) or 1) for p in parts),
        "thicknesses_mm": thicknesses,
        "names": [p.name for p in parts],
        # Shown (and editable) next to each part in the CAM window's list.
        "quantities": {p.name: int(getattr(p, "qty", 1) or 1) for p in parts},
    }


def missing_pocket_warning() -> str:
    """Kept for callers that want a standing caveat; the per-part `reason` in the
    diagnostics is the accurate account, because whether pockets were read is now
    decided part by part rather than being true of the whole job."""
    return ("Pockets are read from the solid. Any part that could not be read that "
            "way is cut as through-features only and is flagged in the placement "
            "table — check it before cutting.")
