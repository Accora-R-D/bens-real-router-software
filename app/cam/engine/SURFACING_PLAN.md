# Adding 3D Surfacing to the Onshape → Syntec CNC Router Pipeline

**Status:** Plan (2026-06-01). Grounds: deep read of the live engine + research on surfacing
algorithms, OSS CAM libraries, Onshape 3D export, and Syntec posting.

---

## 1. Goal

Extend the proven 2.5D pipeline (`cnc-router-pipeline` + hosted `cnc-cam-app`) to also
produce **true 3-axis surfacing** programs: roughing + ball-nose finishing of curved /
non-planar faces, output as Syntec G-code that passes the safety gate, simulates, and ships
through the same web app.

**Decided (2026-06-01):**
- **Scope = full 3D parts / moulds / formers** (deep cavities, steep walls). So **roughing is
  first-class** (not optional), and **steep-wall waterline finishing + rest machining** are in
  scope from the start — not just shallow relief.
- **Engine = OpenCAMLib (OCL) as the core kernel** ("most powerful + efficient"), wrapped by a
  permissive (MIT) mesh/viz layer. See §4.3.

## 2. The one non-negotiable principle

**3D is a *parallel* strategy. The 2.5D path is never mutated.**

The current output is byte-validated against 8 reference `.NC` files. 3D surfacing breaks
every foundational 2.5D assumption (planar-face-only extraction, 2D paths with Z synthesised
at emit time, flat-end cutter, fixed-Z depth passes, "cut through to z≤0"). So we add a new
`SURFACE` role + new `Prog` methods + new extractor + a UI toggle **defaulting OFF**, and
leave the profile/pocket/drill/engrave code paths untouched. Folding 3D into the existing
branches would risk regressing known-good output — explicitly out of bounds.

## 3. Why the current pipeline can't do it today (the hard assumptions)

| Component | Baked-in 2.5D assumption | Where |
|---|---|---|
| `extract.py` | Picks the **largest PLANE face**, projects edges onto it, drops Z (`MM=1000`, DXF z hardcoded `0.0`). Non-planar faces are invisible. | `extract.py:22-65, 91-93`; planar filter `:199,333` |
| `cam_syntec.py` | A `job` is `(role, path, pocket_depth, tool)`; **`path` is `(x,y)` only** — Z never lives in geometry. | `:481, 497-552` |
| `cam_syntec.py` | `depth_passes()` returns a flat ladder of **scalar Z levels**; multi-pass = same XY path re-cut at each level. | `:76-83` |
| `cam_syntec.py` | `_emit_run` arc-fits assuming **constant Z** (I/J only, no K, Z modal). Any Z change kills arc compression. | `:387-407` |
| `cam_syntec.py` | Tool = flat cylinder radius `R`; no ball/nose geometry. Used in offset, nest gap, viewer. | `:26-39, 67-73` |
| `verify.py` | **`min_cut_z > 0` ⇒ FAIL "parts not cut through"** — fails *every* finishing-only pass. | `:96-98` |
| `verify.py` | Single `final_z_expected` plane; flat-bottom tool; 1mm XY quantised cutfloor; arc Z flattened. | `:67, 92-93, 20-28` |
| `material_sim.py` / `viewer3d.py` | Removal stamps **one flat-disc Z** per segment at `min(z0,z1)`. | `material_sim.py:46-69`; `viewer3d.py:126-145` |
| `cam_syntec.py` `main()` | Only accepts ~18mm flat sheet parts (`17≤dmin≤19`); 3D parts silently dropped. | `:645-646` |

## 4. Key technical decisions (recommended)

1. **Part representation = triangle mesh for toolpathing, Z-map for everything else.** Because
   the scope is full moulds/formers with **steep walls**, the toolpath kernel (OCL) consumes the
   **STL triangle mesh** directly (`STLSurf`) — a pure Z-map gets stair-stepped on near-vertical
   faces and can't drive waterline well. Keep a **Z-map / heightfield** alongside it for what it's
   genuinely best at: **stock/rest-machining state** (rough→finish difference), **material-removal
   simulation**, and the three.js preview — both `material_sim.py` and `viewer3d.py` **already are**
   height-fields, so that reuse stands. B-rep is not needed (tessellate and work from the mesh).

2. **Geometry source = Onshape `tessellatedfaces`** (JSON, synchronous, vertices + indices +
   per-facet & per-vertex normals in one call). No translation job, no file parsing; the
   existing `osclient.get` JSON helper works as-is. Coordinates are **metres → ×1000 for mm**
   (the codebase already does this). Set `chordTolerance`≈1e-5–5e-5 m and `outputVertexNormals=true`.
   `/parts/.../partid/{pid}/tessellatedfaces` scopes to one part. Pin to a microversion `m` for
   reproducible/cacheable meshes. (glTF `.glb` is the fallback if we want a cacheable binary +
   reuse in the three.js preview; STL `units=millimeter` if we'd rather use `numpy-stl`.)

3. **Toolpath engine = OpenCAMLib (OCL) core, permissive wrapper.**
   - **OCL is the kernel.** It implements precisely what full moulds/formers need, gouge-free,
     over an STL mesh with Cyl/Ball/Bull/Cone cutters: `BatchDropCutter`/`PathDropCutter`/
     `AdaptivePathDropCutter` (parallel + projected finishing), `PushCutter`/`BatchPushCutter`
     → `Waterline`/`AdaptiveWaterline` (steep-wall + Z-level), all in fast C++ with OpenMP. This
     is the hard math (multi-cutter contact, waterline loop ordering) we do **not** hand-roll.
   - **Licensing: OCL is LGPL-2.1 (since Aug 2018, NOT GPL)** — safe to dynamically link into the
     closed hosted product via its pip wheel; **no AGPL/SaaS disclosure trigger.** Keep it
     unmodified + replaceable + attributed.
   - **Version pin:** prebuilt wheels stop at **Python 3.11**, so pin the **engine container and
     the surfacing dev env to 3.11** (or build OCL from source on 3.12+: VS Build Tools + CMake +
     Boost.Python). This is the main infra cost of the choice — the Render Dockerfile python stage
     must be 3.11.
   - **Permissive wrapper (MIT/BSD):** `trimesh` for STL I/O + the `tessellatedfaces`→`STLSurf`
     bridge, a numpy **Z-map** for stock/rest-machining + sim, `pyvista`/VTK for any custom
     sampling — all embed freely, parallelise on the 22-core box. A tiny numpy drop-cutter is kept
     only as a **validation oracle** against OCL `BatchDropCutter`, not as the engine.
   - **Roughing / adaptive:** OCL has no 2.5D *adaptive* clearing. If projected-raster + Z-level
     roughing (below) prove too slow on thick stock, lift **FreeCAD `Adaptive2d` (LGPL-2.1)** behind
     our own binding. **Avoid PyCAM / DerpCAM (GPL-3.0, viral)** entirely.

4. **Post = the existing Syntec post + a surfacing wrapper.** Syntec is Fanuc-dialect ISO, so
   N+10 / tool-centre / no-G41-G42 / G01-G02-G03 already map. Add:
   - Wrap the dense surfacing G01 stream in **`G05 P10000` … `G05 P0`** (or `G05.1 Q1 E0.01`)
     for high-speed look-ahead smoothing; force **G64** continuous-cut; **never G61/G09** in a
     surfacing pass (exact-stop = stutter at every point).
   - **Tool-centre output with the ball radius offset along the surface normal in CAM** (we keep
     no cutter comp; CAM owns all gouge avoidance). Stamp the assumed ball radius into the header
     comment; re-post on tool change.
   - CAM chord/linearisation tolerance ~**0.005–0.01 mm** (finish), tie controller `E` to it.
     Arc-fit only genuinely circular regions. Watch block count (no hard cap; bounded by RAM
     ~>6 MB free and ~1000–2000 blocks/s) — decimate within tolerance; DNC-stream the largest jobs.

5. **Tooling = add a ball-nose kind.** `TOOLS`/`Seat` gain `kind='ballnose'`, `nose_radius_mm`,
   `corner_radius_mm`. Feeds use the **effective diameter** `Deff = 2·√(DOC·(D−DOC))` so RPM rises
   to hold Vc at shallow DOC. Finishing stepover **5–12% of dia** via scallop `h = R−√(R²−(s/2)²)`.
   **Never straight full-Z plunge a ball nose** (zero speed at centre) — ramp/helix/arc lead-in.
   Carbide mandatory in MDF.

## 5. Phased roadmap

Each phase is independently verifiable (mirrors how the 2.5D pipeline was built: prove in sim
before the next layer).

### Phase 0 — Decisions & setup
- ✅ Scope + engine decided (see top of doc / §8).
- ✅ **Render hosting prepared + verified (2026-06-01).** OCL runs on Render with no source
  build: the `opencamlib-2023.1.11-cp311-cp311-manylinux_2_17_x86_64` wheel resolves for the exact
  Render target and is **self-contained** (bundles its own `libgomp`/OpenMP — no apt packages). The
  full dep set (opencamlib + trimesh + numpy 1.26 + matplotlib + pyclipper) resolves with no
  conflicts for cp311/linux, and the wheel contains every Phase-2 symbol (BatchDropCutter/
  PathDropCutter/AdaptivePathDropCutter/Waterline/AdaptiveWaterline + Cyl/Ball/Bull cutters).
  Changes (all in `cnc-cam-app/`, in files clean of the in-flight engrave WIP):
  `Dockerfile` + `backend/Dockerfile` → `python:3.11-slim`; `backend/requirements.txt` +=
  `opencamlib==2023.1.11`, `trimesh==4.*`; vendored `engine_core/surface_extract.py`; new
  `engine_core/surface_selftest.py` (runtime proof: imports ocl + runs a BatchDropCutter, run it in
  the Render shell). **Deploy pending** a scoped commit + push (see below) — not auto-pushed because
  master carries unrelated uncommitted engrave WIP and a push deploys the live team tool.
- TODO: register a ball-nose seat in the tool table (`TOOLS` / `Seat` / Supabase `cam_tool_seats`).

### Phase 1 — Geometry intake → mesh (+ Z-map)  *(new `surface_extract.py`)* — ✅ DONE 2026-06-01
- **Built + verified** `surface_extract.py` (numpy + matplotlib only — zero new deps; default
  Python is 3.14 here, so OCL/trimesh come in Phase 2 on a 3.11 pin).
- `fetch_mesh()` GETs `tessellatedfaces` via the existing `osclient` and ×1000 to mm. **Verified
  live response shape:** top-level dict → `bodies[].faces[].facets[]`; each facet has `vertices`
  (3 pts, metres), per-facet `normal`, per-vertex `normals`, and an (often empty) `indices` — when
  empty each facet's `vertices` IS one triangle. Mesh = `(M,3,3)` tri array + per-facet normals.
- `choose_frame()` picks the machining/up axis — world `x|y|z` (default `z`), `auto` (largest
  coplanar-area flat-face direction, for a mould's base), or an explicit `[x,y,z]` vector — and a
  base origin so heights ≥ 0. `build_zmap()` rasterises the mesh's **upper envelope** into a numpy
  heightfield (vertical facets skipped — refined by OCL in Phase 2). `write_binary_stl()` emits the
  STL for OCL; `save_zmap_png()` is the visual check.
- **Verified:** Arm Pin → Z-map is the rounded top of a cylinder (peak on centreline, falls to
  edges); STL byte-exact (84 + 50·M); `auto`-axis picks the dominant flat. API
  `surface_features(name, doc, mv, elem, pid, …)` mirrors `extract.part_features`; config is module
  globals (`MESH_CHORD_TOL_MM`/`MESH_ANGLE_TOL`/`ZMAP_RES_MM`/`MACHINE_AXIS`) — drops straight into
  the app's `engine._extract_parts` + `_apply_config` pattern when integrated (Phase 6).
- ✅ **Orientation handling** (don't assume z-up): `choose_frame` takes a signed world axis
  (`z/-z/x/-x/y/-y`), `auto`, or a vector; `orientation_report` ranks directions by
  **directly-surfaceable area** (n·m > 0), separating the **bed face** (datum, not machined) from
  **genuine overhangs** (→ flip), and emits `meta["orientation"]` (recommended axis, %-surfaceable,
  overhang/bed %, `needs_multi_setup`, warnings) for a per-part app dropdown. Validated on the real
  former (recommends `z`, 0.9% overhang) and on a deliberately-wrong `-z` (warns + flags flip).
- ✅ **Validated on a real former** (Onshape `f548…/…/3cec…`, 875×975 mm, 45 mm relief, 282,720
  facets): rim+fillet+floor+42 bosses resolve correctly; STL 14.1 MB byte-exact; saved for offline dev.
- **Still TODO in this phase (deferred, not blocking):** optionally co-register to the 2.5D
  up-face frame when a part is *also* a plate; relax `cam_syntec.main()`’s 17–19 mm sheet gate.

### Phase 2 — Finishing  *(new `surfacing.py`)* — **core value** — 2a ✅ DONE 2026-06-01
- ✅ **Phase 2a — drop-cutter finishing core, in numpy (runs today, no OCL/3.11 dependency).**
  `Cutter` (ball/flat/bull) + scallop math; `drop_field` = the drop-cutter as a grayscale dilation
  of the Z-map by the negated cutter profile (`Ztip = max(surface − prof)`, **gouge-free by
  construction**); `raster_finish` → (x,y,z) parallel/zig-zag toolpaths; `flat_area_mask` (flat-tool
  floor hook); `estimate_nc_bytes`. **Validated** (`test_surfacing.py`): analytic dome → apex tip
  = Rs exactly, no-gouge slack ≈ −1.8e-15; scallop round-trips; the **real former** (ball Ø6 @
  0.05 mm scallop → 1.09 mm stepover) = 977 passes, **846 k points, Z 5.57–51 mm, ~22 MB NC** —
  a concrete confirmation of the file-size risk (→ flat-area floor finishing + waterline-only-on-
  steep-walls + G05/decimation matter).
- ⏳ **Phase 2b — OCL Waterline** for steep walls (rim sides, boss walls) slots in behind the same
  `*_finish(...) → [polyline]` interface; needs a Python-3.11 sandbox (this box has only 3.12/3.14).
- Below is the original OCL-first framing (the numpy core above is the verified first increment +
  OCL oracle):
- Wrap OCL: build `STLSurf`, set a `BallCutter` (+ `BullCutter`/`CylCutter` as needed),
  `setSampling()` from the scallop target. Scallop helpers `h = R−√(R²−(s/2)²)` / inverse
  `s = 2·√(2Rh−h²)` to convert a target finish → stepover (watch the radius-vs-diameter off-by-2).
- **Parallel finishing** via `PathDropCutter`/`AdaptivePathDropCutter` over raster lanes for
  shallow regions; **waterline finishing** via `Waterline`/`AdaptiveWaterline` for **steep walls**
  (split by a slope-angle threshold ~30–45°). Optional **pencil** pass for concave valleys.
  Flat-area detector (normal≈+Z) → flat endmill in 2.5D for crisp bottoms.
- Link OCL CL-points into ordered `(x,y,z)` polylines with ramp/helix lead-ins (OCL gives points,
  not linked toolpaths — we own linking + lead-ins + retracts).
- **Verify:** the tiny numpy drop-cutter as an **oracle** vs OCL `BatchDropCutter` on the same STL;
  plot paths over the Z-map.

### Phase 3 — Post integration  *(edit `cam_syntec.py`)*
- New role `'SURFACE'`; **embed Z in the path points** (do NOT widen the job tuple — that breaks
  4 unpack sites at `:587, :629, :662`). Add `'SURFACE'` to the cut-order dict at `:550` (else
  `KeyError` on sort).
- `compute_jobs:497-552` gains a `SURFACE` branch producing `(role, path_xyz, None, ballnose_tool)`.
- New `Prog.surface_finish()` (and later `surface_rough()`) that **bypass `depth_passes`** and feed
  the `(x,y,z)` seq straight into `_emit_cuts:369-385` (its else-branch already emits `G01 X Y Z`).
  **Skip `_emit_run` arc-fit** for surface (would flatten Z); tighten the `|Δz|<1e-6` constant-Z
  detector which is too coarse. Route SURFACE in `_emit_one:558-572` / `emit_sheet:575-593`, skipping
  tab/drill logic.
- Add G05/G64 wrapper + ball-radius surface-normal offset helper next to `offset():67-73`;
  ramp/helix lead-in (no dead plunge); header stamps assumed ball radius.
- **Verify:** emit a real `.NC` for a test relief; eyeball blocks; confirm `G05 P10000`/`G05 P0`.

### Phase 4 — Safety gate + sim/viz  *(edit `verify.py`, `material_sim.py`, `viewer3d.py`)*
- ✅ **Ball-nose material-removal SIM done early** (`surf_sim.py`, new — the legacy flat-disc
  `material_sim.py`/`viewer3d.py` can't model a ball nose). `simulate_cut` carves a fine stock
  Z-map (scatter-MIN of the tip envelope along the path); `save_render` = relief-shaded backplot;
  `save_cross_section` shows the cusps; `build_zmap(window=)` enables high-res crops. Validated on
  the former: clean ~0.05 mm floor scallops at 1.09 mm pitch, and a steep-wall section showing the
  raster ball **lagging on near-vertical faces** (→ confirms the waterline need). Full-part + crop
  sims run in ~30 s each on 3.14.
- ✅ **Interactive 3D viewer done** (`surf_sim.save_viewer`): standalone three.js HTML of the as-cut
  surface — lit height-field, orbit/zoom/pan, Z-exaggeration slider (pops shallow scallops),
  wireframe + reset. Writes `out/Part 1_view_overview.html` (full part) + `out/Part 1_view_crop.html`
  (high-res). Reuses the vendored `out/lib/` three.js. Render-verified (clean, no console errors).
- ✅ **Animated carving viewer done** (`surf_anim.py` → `out/Part 1_anim.html`): WATCH the ball-nose
  tool sweep the lanes and carve the stock block down to the form (play/pause/speed/scrub). Ball-nose
  stamp + lit tool at the cut front; coarse (1.5 mm grid, 2.5 mm stepover, 142 k pts) for smooth
  playback. Render-verified mid-carve via the Preview MCP.
- Still TODO: mode-aware `verify.py` (gouge vs target Z-map, cut-through relax).
- **Make `verify.check` mode-aware** (thread a `mode`/`tool_tip_radius`/`surface_grid` param like
  `final_z_expected` is passed today, `:130`):
  - Relax **cut-through** (`:96-98`) and **cut-too-deep single-plane** (`:92-93`) for finishing-only.
  - Add the **gouge check** (currently a FALSE-PASS — the most dangerous gap): compare commanded Z
    against the target-surface Z-map (key against a *finer-than-1mm* grid, not the lossy `round()`
    cutfloor at `:67`). Add a **max-Z-rate / plunge-feed** check on steep descents.
  - `_arc_pts:20-28` should **interpolate Z** across helical lead-ins instead of stamping the end Z.
- Sim/viz reuse (both are already height-fields): swap the flat-disc stamp for a **ball-nose tip
  profile** `z_floor(dx,dy) = tip_z + (R−√(R²−d²))` (`material_sim.stamp:46-57`, `viewer3d.stamp:126-134`),
  **lerp Z along the segment** instead of `min(z0,z1)` (`:62`, `:145`), raise grid resolution
  (`RES 1.0`/`RES3D 1.5` → ~0.1–0.3 mm), and stop `viewer3d.decimate():18-33` dropping Z detail.
  Watch memory (inline HTML payload, gw·gh mesh).
- **Verify:** safety gate PASSES on a finishing program; viewer shows scallops.

### Phase 5 — Roughing + rest machining + multi-tool (first-class for moulds/formers)
- **Z-level roughing** with a large **flat endmill** + stock allowance: at each stepdown plane,
  clear cells above the plane (reuse the spiral-pocket clearing pattern) — OCL `Waterline` per
  level gives the boundary, pocket-clear inside it.
- **Projected raster roughing** as the cheap even-stock option: drop-cutter with a positive
  (leave-stock) Z offset so it follows the contour and leaves uniform stock for finishing.
- **Rest machining (multi-tool):** store the post-rough / post-finish **Z-map**, diff against a
  smaller-tool drop-cutter pass, and cut only the leftover in corners/valleys the big tool missed.
  This is the standard mould workflow: big rougher → ball finisher → small ball/pencil for detail.
- **If thick-stock cycle time bites:** add LGPL FreeCAD `Adaptive2d` constant-engagement clearing
  (isolated behind our own binding). Lower priority for forgiving MDF/plastic, real for hardwood/
  tooling board.
- **Workholding:** moulds/formers are blocks, **not vacuum-bed sheets** — no tabs, screw/clamp
  fixturing, and a defined stock origin/top. Single part per program (no flat nesting).

### Phase 6 — Web app exposure  *(edit `cnc-cam-app`)*
- UI: ball-nose tool **kind + nose-radius column** in the 10-seat table; a **Surfacing panel**
  (strategy / stepover-or-scallop / stock allowance / finish feed) behind a toggle **default OFF**.
- API: add a `surfacing` block to `GenerateReq`/`RenestReq` (`app.py:69-95`); thread through
  `engine.generate` + `_apply_config` (set `cs.SURFACING` globals under `_LOCK` exactly like
  `cs.MAT/cs.TOOLS`, and **clear them every call** so state can't leak into a later 2.5D job).
- A surfaced part is a **single 3D part**, not a flat-nested sheet — give it its own flow (skip
  multi-part nesting). Reuse the three.js viewer with the ball-nose stamp.
- **Verify:** live Onshape → surface program → gate PASS → preview, end-to-end on Render.

## 6. Risks & mitigations

- **Regressing the proven 2.5D output** → strict parallel `SURFACE` path; 2.5D code untouched;
  re-run the 8 reference-file byte checks in CI.
- **Safety-gate false FAIL blocks the whole job** (one Result bundles all programs) → mode-aware
  gate is a Phase-4 gate, not optional.
- **Gouge FALSE-PASS** (no surface model today) → the gouge check is the single most important new
  safety rule; without it 3D output is unsafe to ship.
- **File-size / block-count blow-up** → tolerance-based decimation, arc-fit prismatic regions, G05
  smoothing, DNC stream for huge jobs. Confirm the controller's real RAM/USB limit on the 60W-E.
- **OCL Python-version pin** (wheels ≤3.11, library dormant since 2023) → pin 3.11; LGPL lets us
  fork/rebuild if ever needed; keep it a *separable* dependency. Add OCL + any LGPL bits to a
  third-party-notices/attribution page.
- **Concurrency leakage** via the module-global `cs.*` mutation pattern → clear all new globals in
  `_apply_config` every request.
- **Extraction is the dominant cost/schedule risk** (there is no non-planar extractor at all today);
  the CAM emitter changes are comparatively small.

## 7. Licensing summary (for the hosted commercial tool)

- **OpenCAMLib — LGPL-2.1**: ✅ usable. Dynamic-link via the pip wheel (the binding is a runtime
  extension); keep it unmodified/replaceable; attribute it. No SaaS disclosure trigger (that's AGPL).
- **FreeCAD `Adaptive2d` — LGPL-2.1**: ✅ only if you ever need 2.5D adaptive clearing; isolate the
  source behind your own binding, don't depend on the FreeCAD app at runtime.
- **trimesh / pyvista (MIT), VTK (BSD)**: ✅ permissive — use for mesh prep, grids, viz.
- **PyCAM / DerpCAM — GPL-3.0**: ❌ viral; reference for algorithms only, never link.

## 8. Open decisions

**Resolved 2026-06-01:** scope = full 3D parts/moulds/formers (roughing + steep-wall waterline in
scope); engine = OCL core (Python 3.11 pin).

**Still need your input:**
1. **Tooling** — which ball-nose diameters/radii are in the changer, plus a large flat rougher and
   a small ball/pencil for detail? The smallest **concave radius** in your parts must be ≥ the
   finishing ball radius or that feature is left uncut (3-axis hard limit).
2. **Single- vs two-sided** — do your moulds/formers have features on more than one face (needing a
   flip + re-fixture, two programs)? 3-axis can't reach undercuts; confirm parts are single-valued
   from each setup direction.
3. **Stock & fixturing** — stock material (MDF? tooling board? hardwood?), block size convention,
   and how it's held (screws/clamps/fixture) so the post can set a correct stock-top origin and
   safe-Z, since the vacuum-bed/no-tab sheet assumptions don't apply.

## 9. New / changed files at a glance

- **New:** `surface_extract.py` (mesh → Z-map), `surfacing.py` (drop-cutter + strategies),
  tests/`test_surface.py`.
- **Edited:** `extract.py` (mesh fetch + relax gate), `cam_syntec.py` (`SURFACE` role, `Prog.surface_*`,
  G05/G64 wrap, normal-offset, tool model), `verify.py` (mode-aware + gouge + Z-rate), `material_sim.py`
  + `viewer3d.py` (ball-nose stamp, Z-lerp, finer grid), and in `cnc-cam-app`: `models.py`, `engine.py`,
  `backend/app.py`, the Next.js tool table + a surfacing panel.
