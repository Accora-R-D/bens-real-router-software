"""3D-surfacing geometry intake: Onshape part -> triangle mesh (mm) + top-down
Z-map heightfield. This is the PARALLEL geometry path to extract.py (which only
yields a flat 2D profile of the largest planar face) and the first stage of the
3-axis surfacing strategy. See SURFACING_PLAN.md.

Two representations come out of one Onshape `tessellatedfaces` fetch:
  * mesh  — (M,3,3) triangle array in mm, + per-facet normals. Feeds OCL's
            STLSurf in Phase 2 (and a binary STL is written for that).
  * zmap  — a regular numpy heightfield (top surface seen from +machining-axis).
            Used for stock/rest-machining state, material-removal sim, and the
            three.js preview.

INTEGRATION CONTRACT (so the hosted app can vendor + drive this like the 2.5D
path): no desktop-only assumptions in the core; the Onshape client is injected
via osclient (the app ships an env-based osclient in engine_core/); per-request
config is module-globals (MESH_CHORD_TOL_MM / MESH_ANGLE_TOL / ZMAP_RES_MM /
MACHINE_AXIS), mutated under engine._LOCK exactly like cam_syntec.MAT/TOOLS;
and `surface_features(name, doc, mv, elem, pid, ...)` mirrors
extract.part_features so it drops into engine._extract_parts' _pmap loop. All
returns are plain data (numpy arrays + dicts) — JSON-friendly except the arrays.
"""
from __future__ import annotations
import sys, struct, math
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# The Onshape client only serves fetch_mesh(); everything else in here is plain
# numpy. The STEP path builds its mesh from OCC and never touches Onshape, so
# the import must not be a precondition for using the Z-map machinery.
try:
    from osclient import client, DID, WID, EID   # reuse the existing Onshape client + keys
except Exception:                                # noqa: BLE001
    client = DID = WID = EID = None

MM = 1000.0                       # Onshape SI metres -> mm
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

# ---- per-request config (mutable; the app mutates these under engine._LOCK) ----
MESH_CHORD_TOL_MM = 0.05          # max chord deviation of the tessellation (mm)
MESH_ANGLE_TOL = 0.1              # max facet angle deviation (radians)
ZMAP_RES_MM = 0.5                 # Z-map grid pitch (mm)
MACHINE_AXIS = "z"                # default tool/up axis: a signed world axis
                                  # (z,-z,x,-x,y,-y), "auto" (max-reachable, see
                                  # orientation_report), or an explicit [x,y,z] vector

# Signed world-axis directions a 3-axis tool can approach from.
AXES = {"x": (1, 0, 0), "-x": (-1, 0, 0),
        "y": (0, 1, 0), "-y": (0, -1, 0),
        "z": (0, 0, 1), "-z": (0, 0, -1)}


# ============================ 1. fetch the mesh ============================
def fetch_mesh(doc, mv, elem, pid, chord_tol_mm=None, angle_tol=None):
    """GET tessellatedfaces for one part -> mesh dict.

    Returns {"tris": (M,3,3) float32 mm, "fnormals": (M,3) float32,
             "nbodies", "nfaces", "nfacets"}. Coordinates are the part's own
            Part-Studio frame (correct for surfacing a single part), metres->mm.
    """
    chord = (chord_tol_mm if chord_tol_mm is not None else MESH_CHORD_TOL_MM) / MM
    ang = angle_tol if angle_tol is not None else MESH_ANGLE_TOL
    path = f"/parts/d/{doc}/m/{mv}/e/{elem}/partid/{pid}/tessellatedfaces"
    resp = client.get(path, params={
        "chordTolerance": chord, "angleTolerance": ang,
        "outputFacetNormals": "true", "outputVertexNormals": "false",
    })
    bodies = resp.get("bodies", []) if isinstance(resp, dict) else resp
    tris, fnormals = [], []
    nfaces = 0
    for body in bodies:
        for face in body.get("faces", []):
            nfaces += 1
            for facet in face.get("facets", []):
                vs = facet.get("vertices", [])
                idx = facet.get("indices") or []
                n = facet.get("normal", {})
                fn = (n.get("x", 0.0), n.get("y", 0.0), n.get("z", 1.0))
                pts = [(v["x"], v["y"], v["z"]) for v in vs]
                # indices empty => each facet's `vertices` IS one triangle (3 pts);
                # if present, they index `vertices` in triples (general triangulation).
                triples = ([pts[i:i+3] for i in range(0, len(pts), 3)] if not idx
                           else [[pts[idx[k]], pts[idx[k+1]], pts[idx[k+2]]]
                                 for k in range(0, len(idx), 3)])
                for tri in triples:
                    if len(tri) == 3:
                        tris.append(tri)
                        fnormals.append(fn)
    if not tris:
        raise ValueError("tessellatedfaces returned no facets")
    tris = np.asarray(tris, dtype=np.float64) * MM            # -> mm
    fnormals = np.asarray(fnormals, dtype=np.float64)
    return {"tris": tris.astype(np.float32), "fnormals": fnormals.astype(np.float32),
            "nbodies": len(bodies), "nfaces": nfaces, "nfacets": int(tris.shape[0])}


# ============================ 2. machining frame ============================
def _basis(m):
    """Right-handed (u, v) spanning the plane normal to unit vector m."""
    m = np.asarray(m, float); m = m / (np.linalg.norm(m) or 1.0)
    seed = np.array([1.0, 0, 0]) if abs(m[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(seed, m); u /= (np.linalg.norm(u) or 1.0)
    v = np.cross(m, u)
    return u, v, m


def _tri_areas(mesh):
    t = mesh["tris"].astype(float)
    return 0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1)


def _facing(mesh, m, eps_up=0.02, eps_un=1e-3, base_cos=0.98):
    """Split the surface, by area fraction, relative to a tool approaching from +m:
      up        = DIRECTLY SURFACEABLE — normal faces the tool (n·m > eps_up). A
                  3-axis spindle fixed along m can cut these as a surface.
      bed       = the ~flat face resting on the bed (n·m < -base_cos, ≈antiparallel).
                  Expected and NOT machined — not a problem, just the datum.
      overhang  = GENUINE UNDERCUT — down-facing but not the bed (-base_cos ≤ n·m <
                  -eps_un). Unreachable from +m; if significant, the part needs a flip.
      (the rest are ~vertical walls, n·m≈0: only grazed by the tool flank.)
    Choosing the setup = MAXIMISE `up`; significant `overhang` for the best m => 2 setups.
    NB a horizontal floor scores ~0 from a side axis: it's perpendicular to the tool
    there, not surfaceable — only a top-down axis can cut it."""
    m = np.asarray(m, float); m = m / (np.linalg.norm(m) or 1.0)
    area = _tri_areas(mesh)
    nm = mesh["fnormals"].astype(float) @ m
    tot = float(area.sum()) or 1.0
    up = float(area[nm > eps_up].sum()) / tot
    bed = float(area[nm < -base_cos].sum()) / tot
    overhang = float(area[(nm < -eps_un) & (nm >= -base_cos)].sum()) / tot
    return up, overhang, bed


def surfaceable_area(mesh, m):
    """Fraction (0..1) a 3-axis tool from +m can directly surface (see `_facing`)."""
    return _facing(mesh, m)[0]


def _dominant_axis(mesh):
    """Unsigned normal line of the largest coplanar-area facet cluster (a part's
    flat base/back). Extra candidate so parts modelled at an angle still resolve.
    Vectorised (no per-facet Python loop) for big meshes."""
    n = mesh["fnormals"].astype(float)
    area = _tri_areas(mesh)
    keys = np.round(np.abs(n), 2)                        # merge +/- directions
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros(len(uniq)); np.add.at(sums, inv, area)
    v = uniq[int(np.argmax(sums))]
    return v / (np.linalg.norm(v) or 1.0)


def orientation_report(mesh):
    """Rank machining directions by reachable (non-undercut) area so the tool —
    and the app UI — can choose the setup orientation for ANY part, not just z-up.
    `recommended` maximises what one 3-axis setup reaches; `needs_multi_setup` /
    `ambiguous` flag parts that want a flip or a 2nd setup."""
    def row(lbl, vec):
        up, over, bed = _facing(mesh, vec)
        return {"axis": lbl, "vector": [round(float(c), 4) for c in vec],
                "surfaceable_pct": round(100 * up, 1),
                "overhang_pct": round(100 * over, 1), "bed_pct": round(100 * bed, 1)}

    cands, seen = [], set()
    for lbl, vec in AXES.items():
        cands.append(row(lbl, vec)); seen.add(tuple(np.round(vec, 2)))
    da = _dominant_axis(mesh)                            # the dominant flat (handles angled parts)
    for lbl, v in (("flat+", da), ("flat-", -da)):       # add only if not already a world axis
        key = tuple(np.round(v, 2))
        if key not in seen:
            cands.append(row(lbl, v)); seen.add(key)
    cands.sort(key=lambda c: -c["surfaceable_pct"])
    best, second = cands[0], cands[1]
    return {"candidates": cands,
            "recommended": best["axis"], "recommended_vector": best["vector"],
            "recommended_surfaceable_pct": best["surfaceable_pct"],
            "recommended_overhang_pct": best["overhang_pct"],
            "ambiguous": (best["surfaceable_pct"] - second["surfaceable_pct"]) < 5.0,
            "needs_multi_setup": best["overhang_pct"] > 5.0}


def _resolve_axis(mesh, axis):
    """axis -> unit machining vector m. Accepts a 3-vector, a signed world-axis
    string (z/-z/x/-x/y/-y), or 'auto' (= orientation_report's recommendation)."""
    if isinstance(axis, (list, tuple, np.ndarray)) and len(axis) == 3:
        return np.asarray(axis, float)
    if axis == "auto":
        return np.asarray(orientation_report(mesh)["recommended_vector"], float)
    key = str(axis).lower()
    if key in AXES:
        return np.asarray(AXES[key], float)
    raise ValueError(f"unknown axis {axis!r}; use z/-z/x/-x/y/-y, 'auto', or [x,y,z]")


def choose_frame(mesh, axis=None):
    """Build the machining frame {O, u, v, m}: m = tool/up axis (cut from +m),
    O on the lowest plane along m (so heights >= 0), (u,v) span the XY bed."""
    m = _resolve_axis(mesh, axis if axis is not None else MACHINE_AXIS)
    u, v, m = _basis(m)
    pts = mesh["tris"].reshape(-1, 3).astype(float)
    O = pts[np.argmin(pts @ m)]                          # a point on the base plane
    return {"O": O, "u": u, "v": v, "m": m}


def _to_frame(pts, frame):
    """World mm points (...,3) -> frame coords (x along u, y along v, z along m)."""
    d = pts - frame["O"]
    return np.stack([d @ frame["u"], d @ frame["v"], d @ frame["m"]], axis=-1)


# ============================ 3. build the Z-map ============================
def build_zmap(mesh, frame, res_mm=None, window=None):
    """Rasterise the mesh's UPPER envelope (what a 3-axis tool sees from +m) into
    a heightfield. zmap[j,i] = surface height at (ox+i*res, oy+j*res), in mm above
    the base plane. Cells with no material are NaN. Near-vertical facets (which
    project to a line) are skipped — their top rim is carried by adjacent top
    facets (good enough for Phase 1; OCL handles steep walls exactly in Phase 2).
    """
    res = res_mm if res_mm is not None else ZMAP_RES_MM
    tf = _to_frame(mesh["tris"].astype(float), frame)        # (M,3,3) frame coords
    xs, ys = tf[..., 0], tf[..., 1]
    if window is not None:                                   # explicit (x0,x1,y0,y1) frame window
        x0, x1, y0, y1 = window
        ox, oy = float(x0), float(y0)
        nx = int(math.ceil((x1 - x0) / res)) + 1
        ny = int(math.ceil((y1 - y0) / res)) + 1
    else:                                                    # auto-extent to the mesh
        ox, oy = float(xs.min()), float(ys.min())
        nx = int(math.ceil((xs.max() - ox) / res)) + 1
        ny = int(math.ceil((ys.max() - oy) / res)) + 1
    zmap = np.full((ny, nx), -np.inf, dtype=np.float64)

    for tri in tf:
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = tri
        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-9:                                # vertical -> skip
            continue
        i0 = max(0, int(math.floor((min(x0, x1, x2) - ox) / res)))
        i1 = min(nx - 1, int(math.ceil((max(x0, x1, x2) - ox) / res)))
        j0 = max(0, int(math.floor((min(y0, y1, y2) - oy) / res)))
        j1 = min(ny - 1, int(math.ceil((max(y0, y1, y2) - oy) / res)))
        if i1 < i0 or j1 < j0:
            continue
        gx = ox + np.arange(i0, i1 + 1) * res
        gy = oy + np.arange(j0, j1 + 1) * res
        GX, GY = np.meshgrid(gx, gy)
        a = ((y1 - y2) * (GX - x2) + (x2 - x1) * (GY - y2)) / denom
        b = ((y2 - y0) * (GX - x2) + (x0 - x2) * (GY - y2)) / denom
        c = 1.0 - a - b
        inside = (a >= -1e-9) & (b >= -1e-9) & (c >= -1e-9)
        z = a * z0 + b * z1 + c * z2
        block = zmap[j0:j1 + 1, i0:i1 + 1]
        np.maximum(block, np.where(inside, z, -np.inf), out=block)

    grid = {"ox": round(ox, 4), "oy": round(oy, 4), "res": res, "nx": nx, "ny": ny}
    zmap[~np.isfinite(zmap)] = np.nan                        # no-material -> NaN
    return zmap, grid


# ============================ 4. public entry ============================
def surface_features(name, doc, mv, elem, pid, axis=None, res_mm=None,
                     chord_tol_mm=None, keep_mesh=True):
    """Mirror of extract.part_features for the surfacing path. Returns
    {name, mesh?, zmap, grid, frame, meta}. `mesh` omitted if keep_mesh=False."""
    mesh = fetch_mesh(doc, mv, elem, pid, chord_tol_mm=chord_tol_mm)
    report = orientation_report(mesh)                    # rank machining directions
    sel = axis if axis is not None else MACHINE_AXIS
    if sel == "auto":
        used_label, vec = f"auto({report['recommended']})", report["recommended_vector"]
    else:
        used_label = sel if isinstance(sel, str) else "custom"
        vec = [round(float(c), 6) for c in _resolve_axis(mesh, sel)]
    frame = choose_frame(mesh, vec)
    zmap, grid = build_zmap(mesh, frame, res_mm)
    finite = zmap[np.isfinite(zmap)]
    tf = _to_frame(mesh["tris"].reshape(-1, 3).astype(float), frame)
    up, over, bed = _facing(mesh, frame["m"])
    surf, overhang, bed_pct = round(100 * up, 1), round(100 * over, 1), round(100 * bed, 1)
    warnings = []
    if surf < report["recommended_surfaceable_pct"] - 1.0:
        warnings.append(f"from {used_label} only {surf}% of the surface is directly surfaceable; the "
                        f"best single setup is '{report['recommended']}' ({report['recommended_surfaceable_pct']}%)")
    if overhang > 5.0:
        warnings.append(f"{overhang}% of the surface is a genuine undercut from {used_label} (down-facing, "
                        f"not the bed) — that area needs a flip / 2nd setup")
    meta = {
        "name": name, "ref": [doc, mv, elem, pid],
        "facets": mesh["nfacets"], "faces": mesh["nfaces"], "bodies": mesh["nbodies"],
        "extent_mm": [round(float(np.ptp(tf[:, 0])), 2), round(float(np.ptp(tf[:, 1])), 2)],
        "z_min_mm": round(float(finite.min()), 3) if finite.size else None,
        "z_max_mm": round(float(finite.max()), 3) if finite.size else None,
        "z_range_mm": round(float(np.ptp(finite)), 3) if finite.size else 0.0,
        "grid": grid,
        "frame": {k: [round(float(c), 6) for c in np.atleast_1d(frame[k])]
                  for k in ("O", "u", "v", "m")},
        "orientation": {**report, "axis_used": used_label,
                        "axis_used_vector": [round(float(c), 4) for c in frame["m"]],
                        "surfaceable_pct": surf, "overhang_pct": overhang, "bed_pct": bed_pct,
                        "warnings": warnings},
    }
    out = {"name": name, "zmap": zmap, "grid": grid, "frame": frame, "meta": meta}
    if keep_mesh:
        out["mesh"] = mesh
    return out


# ============================ 5. verify / artifacts ============================
def write_binary_stl(mesh, path):
    """Binary STL (mm) for OCL's STLSurf (Phase 2) and external viewers."""
    tris, nrm = mesh["tris"], mesh["fnormals"]
    M = int(tris.shape[0])
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", M))
        for k in range(M):
            f.write(struct.pack("<3f", *nrm[k]))
            for vtx in tris[k]:
                f.write(struct.pack("<3f", float(vtx[0]), float(vtx[1]), float(vtx[2])))
            f.write(struct.pack("<H", 0))
    return path


def save_zmap_png(zmap, grid, path, title=""):
    """Height-shaded preview of the Z-map (Phase 1 visual proof)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ox, oy, res, nx, ny = grid["ox"], grid["oy"], grid["res"], grid["nx"], grid["ny"]
    fig, ax = plt.subplots(figsize=(8, 8 * ny / max(nx, 1)))
    im = ax.imshow(zmap, origin="lower", aspect="equal", cmap="viridis",
                   extent=[ox, ox + nx * res, oy, oy + ny * res])
    ax.set_title(title or "Z-map (surface height, mm)")
    ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)")
    fig.colorbar(im, ax=ax, shrink=0.8, label="height above base (mm)")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)
    return path


def _first_parts(limit=1):
    asm = client.get(f"/assemblies/d/{DID}/w/{WID}/e/{EID}",
                     params={"includeMateFeatures": "false"})
    seen, out = set(), []
    for i in asm["rootAssembly"]["instances"]:
        if i.get("type") != "Part":
            continue
        key = (i["documentId"], i["documentMicroversion"], i["elementId"], i["partId"])
        if key in seen:
            continue
        seen.add(key)
        out.append((i["name"], *key))
        if len(out) >= limit:
            break
    return out


def main():
    """Smoke test: extract a mesh + Z-map for the first part(s) of the known doc,
    write STL + Z-map PNG to out/, and print stats. Point this at a real mould
    part studio later by editing DID/WID/EID in osclient or extending main()."""
    parts = _first_parts(limit=int(sys.argv[1]) if len(sys.argv) > 1 else 2)
    for name, doc, mv, elem, pid in parts:
        try:
            feat = surface_features(name, doc, mv, elem, pid, axis=MACHINE_AXIS)
        except Exception as e:
            print(f"  {name:28} FAILED: {type(e).__name__}: {e}")
            continue
        m = feat["meta"]; g = m["grid"]
        stem = "".join(c if c.isalnum() or c in " -_" else "_" for c in name).strip()
        stl = write_binary_stl(feat["mesh"], OUT / f"{stem}.stl")
        png = save_zmap_png(feat["zmap"], g, OUT / f"{stem}_zmap.png",
                            title=f"{name}  Z-map  (axis={MACHINE_AXIS})")
        filled = int(np.isfinite(feat["zmap"]).sum())
        print(f"  {name:28} facets={m['facets']:6d}  "
              f"extent={m['extent_mm'][0]:7.1f}x{m['extent_mm'][1]:7.1f}mm  "
              f"Zrange={m['z_range_mm']:6.2f}mm  grid={g['nx']}x{g['ny']}@{g['res']}mm "
              f"({filled} filled)  -> {stl.name}, {png.name}")


if __name__ == "__main__":
    main()
