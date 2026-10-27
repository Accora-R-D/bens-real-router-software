"""
Read a browser-exported assembly STEP and produce the same part records the
fabrication pipeline already consumes — with no Onshape API calls at all.

Why this exists
---------------
Onshape meters API calls against an ANNUAL allowance (HTTP 402 "API limit
exceeded" once it's gone), and a pack of a ~90-part assembly costs on the order
of two thousand calls, most of it polling.  Exporting the assembly by hand from
the Onshape browser UI costs nothing — Onshape's own client is exempt — so the
whole pipeline can run off one downloaded file.

What it replaces
----------------
Only the input layer.  `read_step()` emits records shaped exactly like the ones
`introspect_bodies_batch` builds from FeatureScript:

    {"name", "qty", "thickness_mm", "_bbox_w_mm", "_bbox_h_mm", "_edges", ...}

so the classifier, nester, DXF writer, review page and fabrication reference all
run downstream unchanged.

What it cannot do
-----------------
A STEP carries geometry, names and assembly structure — not Onshape's native
sheet-metal flag, frame-profile attribute, material or part numbers.  The
classifier already has geometric fallbacks for the first two; material and part
numbers have to come from a BOM CSV exported alongside.

Needs the OpenCascade kernel (`OCP`).  Imported lazily so this module can be
imported in environments that don't have it.
"""

from __future__ import annotations

import collections
import math
import re
from dataclasses import dataclass, field


# Sampling deflection when turning arcs and splines into polylines.  0.05 mm is
# well inside laser tolerance and matches the fidelity of the FeatureScript
# edge sampling this replaces.
CHORD_TOL_MM = 0.05

# Two planes closer than this are the same plane; further apart than this can't
# be a sheet thickness.
# A profile whose edges do not meet within this is not trustworthy: the gap
# would be bridged by a straight line across the part.
OUTLINE_GAP_TOL_MM = 0.05

# Boards the workshop buys. Sheet steel is 2/3/4/5/6/8/10 — never 12 — so a
# 12mm plate is board, per the workshop's own stock list.
_WOOD_GAUGE_MM = (12.0, 15.0, 18.0)
_MIN_WALL_MM = 0.2
_MAX_WALL_MM = 60.0


def _snap_stock_dim(v: float) -> float:
    """A measured section size, quoted the way stock is sold.

    Box and round section come in whole millimetres, occasionally a half; a
    measurement of 20.144 is a 20 with a modelling seam in it (an unfolded
    frame's two segments left their walls 0.14 mm out of plane), not a size
    that exists — and it went straight into a filename as 20.1x20.1. The
    tolerance is deliberately tight so genuine odd sizes survive: 48.3 pipe
    is 0.3 from 48 and stays 48.3.
    """
    for step, tol in ((1.0, 0.2), (0.5, 0.1)):
        snapped = round(v / step) * step
        if abs(snapped - v) <= tol:
            return round(snapped, 1)
    return round(v, 1)


def _snap_wall(v: float) -> float:
    """Wall thickness, quoted in the halves it is sold in (1.5, 2, 2.5 ...)."""
    snapped = round(v * 2) / 2
    return snapped if abs(snapped - v) <= 0.15 else round(v, 1)


# Everything the workshop cuts from steel. Turned and printed parts are left
# blank deliberately — their weight is not called out on the reference.
_STEEL_PROCESSES = {"flat", "folded", "tube_chs", "tube_shs",
                    "tube_mitered", "rod"}

# Processes that produce no per-part file.
#
# bought_in: purchased, not made. It belongs on the cut list and the reference
# but must not be exported as geometry — exporting them was sending the workshop
# STEPs of nuts and bolts.
#
# wood: the router does not want a file per part. Its deliverable is the CAM
# program for each thickness of board, produced by the CAM window from the same
# assembly STEP, and a per-part STEP alongside it is just something else to keep
# in step. Wood still appears on the cut list and the reference.
# Wood is no longer file-less: each wood part ships as its own STEP in the
# pack's "Wood Files" folder, and the operator feeds those into the router CAM
# by hand — the CAM window stopped auto-opening from Download Pack.
NO_FILE_PROCESSES = {"bought_in", "skip"}
MILD_STEEL_DENSITY_KG_M3 = 7850.0


class OutlineError(RuntimeError):
    """The flat profile could not be extracted safely."""


@dataclass
class StepPart:
    name: str
    qty: int
    process: str                       # flat | rod | tube_chs | tube_shs | folded | turned | unknown
    length_mm: float
    width_mm: float
    thickness_mm: float
    volume_mm3: float
    detail: str = ""
    depth_mm: float = 0.0              # smallest bounding-box extent
    section_kind: str = ""             # CHS | SHS | RHS, for tube naming
    section_w: float = 0.0             # OD for CHS, larger face for SHS/RHS
    section_h: float = 0.0             # smaller face; == w for SHS
    edges: list = field(default_factory=list)      # flat outline loops, 2-D, mm
    edges_primitive: list = field(default_factory=list)  # native lines/arcs
    shape: object = None                            # TopoDS_Shape, for re-export
    as_modelled: object = None      # pre-rewrite shape of a frame, for the icon
    signature: tuple = ()                           # geometry fingerprint, for de-duping
    chirality: int = 0                              # +1 / -1 handed, 0 has a mirror plane
    part_number: str = ""                           # assigned by the pack builder
    merged_from: list = field(default_factory=list)  # names folded into this one
    merged_qty: dict = field(default_factory=dict)   # folded-in name -> its own count
    source: str = ""                                 # the file/assembly it was read from
    sub_assemblies: list = field(default_factory=list)  # weldments it appears in
    sub_qty: dict = field(default_factory=dict)      # weldment -> pieces used there

    @property
    def material(self) -> str:
        return "Mild steel" if self.process in _STEEL_PROCESSES else ""

    @property
    def unit_mass_kg(self) -> float:
        """Mass of ONE piece, or 0 where material isn't called out."""
        if self.process not in _STEEL_PROCESSES:
            return 0.0
        return self.volume_mm3 * 1e-9 * MILD_STEEL_DENSITY_KG_M3

    @property
    def total_mass_kg(self) -> float:
        return self.unit_mass_kg * self.qty

    def as_flat_record(self) -> dict:
        """The dict shape the nester and DXF writer already expect."""
        xs = [p[0] for loop in self.edges for p in loop]
        ys = [p[1] for loop in self.edges for p in loop]
        return {
            "name": self.name,
            "part_number": self.part_number,
            "qty": self.qty,
            "quantity": self.qty,
            "thickness_mm": self.thickness_mm,
            "_bbox_w_mm": (max(xs) - min(xs)) if xs else 0.0,
            "_bbox_h_mm": (max(ys) - min(ys)) if ys else 0.0,
            "_edges": self.edges,
            "_edges_primitive": self.edges_primitive,
            "_hole_labels": [],
        }


def _ocp():
    """Import the kernel on demand, with a message that says what to install."""
    try:
        import OCP  # noqa: F401
    except ImportError as exc:      # pragma: no cover - environment dependent
        raise RuntimeError(
            "Reading STEP files needs the OpenCascade kernel. "
            "Install it with:  pip install cadquery-ocp"
        ) from exc
    return True


# ---------------------------------------------------------------------------
# Assembly tree
# ---------------------------------------------------------------------------

def _read_document(path: str):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString

    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if not reader.ReadFile(path):
        raise RuntimeError(f"could not read STEP file: {path}")
    reader.Transfer(doc)
    return doc


def _label_name(label) -> str:
    from OCP.TDataStd import TDataStd_Name
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def _solids_of(shape) -> list:
    """Every solid inside a shape. A weldment or composite part arrives as one
    label holding several solids, and measuring the compound as a single body
    would classify the whole weldment as one part instead of the tube, plate and
    bracket it is actually made from."""
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopoDS import TopoDS

    out = []
    exp = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def _clean_name(name: str) -> str:
    """Onshape marks instances with a trailing "<2>". The name is the same part
    either way, and the marker only makes identical parts look distinct."""
    return re.sub(r"\s*<\d+>\s*$", "", (name or "").strip())


def _child_suffix(i: int) -> str:
    """_a, _b, ... _z, _aa — matching how the Onshape route names the children
    of a composite part."""
    letters = ""
    i += 1
    while i:
        i, r = divmod(i - 1, 26)
        letters = chr(97 + r) + letters
    return f"_{letters}"


def _collect_parts(doc) -> "dict[str, dict]":
    """Walk the assembly and count how many times each distinct part occurs.

    Quantity comes from the number of occurrences in the tree, which is what the
    BOM walk used to provide. Sub-assembly membership is recorded so a pack can
    be split per weldment, and multi-solid leaves are broken into their
    individual bodies.
    """
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence, TDF_Label, TDF_Tool
    from OCP.TCollection import TCollection_AsciiString

    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)

    # Keyed by the product label, NOT the name.  Two different products can
    # share a name — a board and its counterbored twin were both exported as
    # "Wooden Swing Arm Board" — and keying by name folded the second into the
    # first's quantity and threw its geometry away, so one of the two boards
    # was machined to the wrong drawing.  Same-name parts that really are
    # identical still end up as one row: they are separate records here and
    # _merge_duplicates folds them by geometry.
    found: dict[str, dict] = {}
    loose: list[dict] = []      # unnamed bodies — composite-part constituents

    def add(name: str, shape, path: tuple, key: "str | None" = None):
        rec = found.setdefault(key or name,
                               {"name": name, "shape": shape, "qty": 0,
                                "subs": [], "sub_qty": collections.Counter()})
        rec["qty"] += 1
        rec["sub_qty"][path[-1] if path else ""] += 1
        # Which weldment(s) this part turns up in — the immediate parent is the
        # useful one; the root assembly is not a sub-assembly.
        sub = path[-1] if path else ""
        if sub and sub not in rec["subs"]:
            rec["subs"].append(sub)

    def walk(label, path: tuple = (), is_root: bool = False, world=None):
        # `world` accumulates the placements down the tree.  Two bodies can only
        # be tested for contact once both are in the same (world) frame; each
        # label on its own carries only its position relative to its parent.
        from OCP.TopLoc import TopLoc_Location
        from OCP.XCAFDoc import XCAFDoc_ShapeTool
        if world is None:
            world = TopLoc_Location()
        if not is_root:
            world = world * XCAFDoc_ShapeTool.GetLocation_s(label)

        # Resolve the reference BEFORE asking whether it is an assembly.  A
        # component label is only a pointer: IsAssembly is False on it even when
        # the shape it points at is a whole sub-assembly.  Testing the pointer
        # collapsed every sub-assembly into a single "part" holding all its
        # solids — one 273-solid leaf for the main frame — which threw away
        # every real part name inside it and made composite parts, hardware
        # names and the sub-assembly split all impossible to recover.
        target = label
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            target = ref

        if tool.IsAssembly_s(target):
            name = _clean_name(_label_name(target))
            # The outermost assembly names the pack, not a sub-assembly, so it
            # is not added to the path — otherwise every part would look like it
            # belonged to a sub-assembly and splitting the pack would be moot.
            child_path = path if (is_root or not name) else path + (name,)
            kids = TDF_LabelSequence()
            tool.GetComponents_s(target, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), child_path, world=world)
            return

        name = _clean_name(_label_name(target)) or ""
        shape = tool.GetShape_s(target)
        if _is_unnamed_body(name):
            # Onshape's STEP export has no concept of a composite part, so it
            # writes each constituent BODY as its own unnamed top-level product
            # ("id34").  Left alone, every segment of a mitred frame ships as
            # its own tube.  Held back here and regrouped by contact below;
            # bodies Onshape did name are never touched.
            loose.append({"shape": shape.Moved(world), "path": path,
                          "id": name})
            return
        # A leaf holding several solids is one part made of several bodies.
        # It is exported as a single part — what the workshop orders.
        entry = TCollection_AsciiString()
        TDF_Tool.Entry_s(target, entry)
        add(name or "unnamed", shape, path, key=entry.ToCString())

    for i in range(1, roots.Length() + 1):
        # Drop the outermost assembly from the path: it names the pack.
        walk(roots.Value(i), (), is_root=True)

    for name, shape, path in _regroup_composites(loose):
        add(name, shape, path)

    # Re-key by name for the callers, suffixing clashes: the first product
    # under a name keeps it, later distinct products become "Name (2)"...
    taken: collections.Counter = collections.Counter()
    out: dict[str, dict] = {}
    for rec in found.values():
        name = rec.pop("name")
        taken[name] += 1
        out[name if taken[name] == 1 else f"{name} ({taken[name]})"] = rec
    return out


def _is_unnamed_body(name: str) -> bool:
    """True for Onshape's placeholder body names ("id34").

    These are the constituent bodies of a composite part: Onshape writes no
    name for them because STEP has nowhere to put the composite itself.  It is
    the only marker in the file that says "this is part of something bigger",
    which is what makes regrouping safe — a part the user actually named is
    never a candidate.
    """
    return bool(re.fullmatch(r"id\d+", (name or "").strip()))


def _regroup_composites(loose: list) -> list:
    """Cluster touching unnamed bodies back into composite parts.

    Yields (name, shape, path).  Bodies that touch — a mitred frame's segments
    meet face to face at each mitre — belong to one composite part and are
    returned as a single compound, so the pack ships one file for the frame
    rather than one per segment.  Only bodies in the same sub-assembly are ever
    joined, and a body touching nothing comes back on its own.
    """
    if not loose:
        return []
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder

    for b in loose:
        b["box"] = _bbox(b["shape"])

    # Union-find over "touching, and in the same sub-assembly".
    parent = list(range(len(loose)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(loose)):
        for j in range(i + 1, len(loose)):
            if loose[i]["path"] != loose[j]["path"]:
                continue
            if find(i) == find(j):
                continue
            # Bounding boxes first — the exact distance is far too slow to run
            # on every pair, and boxes that miss cannot be touching.
            if not _boxes_touch(loose[i]["box"], loose[j]["box"]):
                continue
            if _shapes_touch(loose[i]["shape"], loose[j]["shape"]):
                parent[find(i)] = find(j)

    clusters: dict = {}
    for i in range(len(loose)):
        clusters.setdefault(find(i), []).append(loose[i])

    out = []
    n = 0
    for members in clusters.values():
        path = members[0]["path"]
        if len(members) == 1:
            # Nothing touching it, so it is an ordinary part that simply has no
            # name.  Keep the id so it can still be told apart on the review page.
            out.append((f"Part {members[0]['id']}", members[0]["shape"], path))
            continue
        n += 1
        builder = BRep_Builder()
        comp = TopoDS_Compound()
        builder.MakeCompound(comp)
        for m in members:
            builder.Add(comp, m["shape"])
        out.append((f"Composite part {n}", comp, path))
    return out


def _bbox(shape) -> tuple:
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, True)
    return box.Get()


def _boxes_touch(a: tuple, b: tuple, tol: float = 0.25) -> bool:
    """Do two bounding boxes overlap or come within `tol` on every axis?"""
    return all(a[i] - tol <= b[i + 3] and b[i] - tol <= a[i + 3]
               for i in range(3))


def _shapes_touch(a, b, tol: float = 0.1) -> bool:
    """True when two solids actually meet — mitred segments share a face, so
    the distance between them is zero."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    try:
        d = BRepExtrema_DistShapeShape(a, b)
        return bool(d.IsDone()) and d.Value() <= tol
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------

def _faces(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopoDS import TopoDS
    exp = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        yield TopoDS.Face_s(exp.Current())
        exp.Next()


def _face_area(face) -> float:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    return props.Mass()


def _measure(shape) -> dict:
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, True)
    xm, ym, zm, xM, yM, zM = box.Get()
    extents = sorted([xM - xm, yM - ym, zM - zm])

    vol = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vol)

    kinds = collections.Counter()
    planes = []       # (unit normal, signed offset, area, face)
    radii = []
    cyls = []         # {dir, loc, radius, area} — axis line matters for tube
    total_area = 0.0
    for face in _faces(shape):
        surf = BRepAdaptor_Surface(face)
        stype = surf.GetType()
        area = _face_area(face)
        total_area += area
        if stype == GeomAbs_SurfaceType.GeomAbs_Plane:
            kinds["plane"] += 1
            pl = surf.Plane()
            d = pl.Axis().Direction()
            n = (d.X(), d.Y(), d.Z())
            # Canonical sign so opposing faces of one wall group together.
            if (n[0] + n[1] + n[2]) < 0:
                n = (-n[0], -n[1], -n[2])
            loc = pl.Location()
            offset = loc.X() * n[0] + loc.Y() * n[1] + loc.Z() * n[2]
            planes.append((n, offset, area, face))
        elif stype == GeomAbs_SurfaceType.GeomAbs_Cylinder:
            kinds["cylinder"] += 1
            cyl = surf.Cylinder()
            radii.append(cyl.Radius())
            ax = cyl.Axis()
            d, loc = ax.Direction(), ax.Location()
            cyls.append({"dir": (d.X(), d.Y(), d.Z()),
                         "loc": (loc.X(), loc.Y(), loc.Z()),
                         "radius": cyl.Radius(), "area": area})
        elif stype == GeomAbs_SurfaceType.GeomAbs_Cone:
            kinds["cone"] += 1
        elif stype == GeomAbs_SurfaceType.GeomAbs_Torus:
            kinds["torus"] += 1
        else:
            kinds["other"] += 1

    return {"extents": extents, "volume": vol.Mass(), "kinds": kinds,
            "planes": planes, "radii": radii, "cyls": cyls,
            "total_area": total_area}


def _wall_thickness(planes) -> "tuple[float | None, object, tuple | None]":
    """Smallest plausible gap between parallel planes, preferring the pair with
    the most face area — that's the sheet's two faces rather than some rib.

    Returns (thickness, largest_face_of_that_pair, normal_of_that_pair).
    """
    groups: dict[tuple, list] = collections.defaultdict(list)
    for n, offset, area, face in planes:
        groups[tuple(round(c, 5) for c in n)].append((offset, area, face))

    best = (None, None, 0.0, None)      # thickness, face, area, normal
    for n, entries in groups.items():
        entries.sort(key=lambda e: e[0])
        # A small plane BETWEEN the sheet's two faces breaks a consecutive-only
        # scan: a counterbore floor sits mid-thickness, so an 18mm board with a
        # 6mm counterbore paired face-to-floor and read 12mm — and 12 is a
        # board gauge, so the wrong number even looked right. Scan the group's
        # dominant planes as well; a minor floor cannot mask the sheet.
        biggest = max(a for _d, a, _f in entries)
        major = [e for e in entries if e[1] >= 0.2 * biggest]
        seqs = [entries] if len(major) == len(entries) else [entries, major]
        for seq in seqs:
            for (d0, a0, f0), (d1, a1, f1) in zip(seq, seq[1:]):
                gap = d1 - d0
                if not (_MIN_WALL_MM < gap < _MAX_WALL_MM):
                    continue
                if (a0 + a1) > best[2]:
                    best = (gap, f0 if a0 >= a1 else f1, a0 + a1, n)
    return best[0], best[1], best[3]


def ensure_flat_outline(part) -> str:
    """Extract the flat outline for a part that BECAME flat after ingest.

    Edges are read from the solid exactly once, at ingest, and only for parts
    classified flat there. A review override that reclassified a part to flat
    therefore arrived at the exporter with no edges, and the dispatch requires
    `flat and edges` — so the part fell through to the catch-all and shipped as
    a STEP, and the nest never saw it. The operator asked for a DXF and got
    neither the DXF nor the nesting.

    Returns "" on success, or the reason it could not be outlined. Mirrors the
    ingest path's own extraction: same plane-pair search, same outline walk,
    same true-size correction from the traced edges.
    """
    if part.edges:
        return ""
    if part.shape is None:
        return "no solid on this part"
    try:
        m = _measure(part.shape)
    except Exception as exc:                               # noqa: BLE001
        return f"could not measure the solid: {str(exc)[:80]}"
    thickness, widest_face, _normal = _wall_thickness(m["planes"])
    if widest_face is None:
        # No parallel pair within the wall bounds — take the largest planar
        # face outright; the operator has said this is a plate.
        planar = sorted(m["planes"], key=lambda e: e[2], reverse=True)
        if not planar:
            return "no planar face to outline"
        widest_face = planar[0][3]
    try:
        edges, edge_prims = _face_outline_2d(widest_face)
    except Exception as exc:                               # noqa: BLE001
        return f"profile failed: {str(exc)[:80]}"
    if not edges:
        return "outline came back empty"
    part.edges = edges
    part.edges_primitive = edge_prims
    if thickness:
        part.thickness_mm = thickness
    # The traced outline is the true size; the world bounding box of an angled
    # part is inflated by its own tilt.
    xs = [q[0] for lp in edges for q in lp]
    ys = [q[1] for lp in edges for q in lp]
    if xs and ys:
        a, b = max(xs) - min(xs), max(ys) - min(ys)
        part.length_mm, part.width_mm = (a, b) if a >= b else (b, a)
    return ""


def _is_bought_in(name: str, description: str = "") -> bool:
    """Hardware — screws, nuts, washers, rivets, bearings — is bought, not made.

    Reuses the pack's own name test rather than a second list, so both routes
    agree on what counts as hardware.
    """
    import sys
    from pathlib import Path as _P
    scripts = _P(__file__).resolve().parents[2] / "scripts"
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    try:
        from introspect_bodies_batch import is_hardware_name
    except Exception:
        return False
    try:
        hit, _token = is_hardware_name(name or "", description or "")
        return bool(hit)
    except Exception:
        return False


_FREEFORM_MIN_FACES = 4        # B-spline/other faces before a part reads as moulded
_BEND_RADIUS_TOL = 0.25        # inner and outer bend radii differ by t within this


def _bend_pairs(m: dict, thickness: float) -> int:
    """How many sheet bends the cylinders make: coaxial pairs whose radii
    differ by the sheet thickness (the inside and outside of one fold).
    Holes, bosses and fillets are single radii on their axes and do not count."""
    n = 0
    for g in _coaxial_groups(m["cyls"]):
        radii = sorted({round(r, 2) for r in g["radii"]})
        if any(abs((b - a) - thickness) <= _BEND_RADIUS_TOL
               for a in radii for b in radii if b > a):
            n += 1
    return n


def _classify(m: dict, thickness: "float | None") -> "tuple[str, str]":
    """Everything that is not tube (see _analyse_tube, which is the only
    judge of that) or hardware (see _is_bought_in). Order matters, and each
    step below is placed where a real part needed it."""
    T, W, L = m["extents"]
    kinds = m["kinds"]
    ncyl = kinds["cylinder"]
    radii = sorted({round(r, 2) for r in m["radii"]})

    # Solid round bar first: one cylinder and nothing but its two end faces.
    # Length is not the test — a Ø8 pin 15 long is a saw cut from Ø8 bar just as
    # a 900 long one is, and the operator filed exactly that as "rod" after it
    # shipped as a turned part. What makes a part TURNED is a feature the lathe
    # has to make (a step, a bore, a chamfer); a featureless cylinder has none.
    # Shorter than half its diameter it is a disc, and the plate rule below
    # decides whether the laser cuts it.
    plain = (len(radii) == 1 and ncyl >= 1 and kinds["plane"] <= 2
             and sum(kinds.values()) == ncyl + kinds["plane"])
    dom = m.get("dominant_cyl")
    if plain and dom:
        # measured ALONG the bar, not the longest box side: a disc's longest
        # extent is its diameter
        if dom["length"] >= 0.5 * dom["od"]:
            return "rod", f"D{dom['od']:.0f} x L{dom['length']:.0f}"

    # A body of revolution is turned, and it has to be decided BEFORE the plate
    # test below: a Ø20 x 36 pin has two parallel end faces 36 apart, and its
    # extent along their normal is 36, so it satisfied every clause of "plate"
    # and shipped as a 36 mm plate 20 x 20. What a turned part has that a plate
    # does not is one coaxial cylinder family carrying most of its surface,
    # with the solid no wider about that axis than the cylinder itself.
    if dom and dom["frac"] >= 0.6 and dom["cross"][0] <= dom["od"] + 1.0:
        # ... and mostly solid about that axis. A 2 mm sheet rolled into a half
        # cylinder is also one coaxial family no wider than its radius, but it
        # fills a few percent of its envelope where even a heavily bored boss
        # fills a quarter or more.
        envelope = math.pi * (dom["od"] / 2) ** 2 * max(dom["length"], 1e-6)
        # ... and longer than half its diameter. A Ø20 disc 9 thick is a
        # laser-cut plate part, and was already being cut as one.
        if m["volume"] >= 0.2 * envelope and dom["length"] >= 0.5 * dom["od"]:
            return "turned", f"D{dom['od']:.0f} x L{dom['length']:.0f}"

    # Compare the thickness against the extent along the plate's OWN normal,
    # falling back to the world box only when that could not be measured.
    across = m.get("along_normal")
    if across is None:
        across = T
    # A plate is thinner than its own face. The thickness check alone passes a
    # block whose two biggest faces happen to be parallel.
    if thickness and abs(across - thickness) < 0.3 and thickness <= W + 1e-6:
        # Board gauges: sheet steel stops at 12mm, and the workshop's boards are
        # 15 or 18 ("generally they will be 18mm or 15mm thick, mostly flat").
        # A plate on those gauges is wood for the router, not a laser profile —
        # every board used to arrive as `flat` and be hand-overridden on the
        # review page, every run. A 15/18mm metal tooling plate is the known
        # false positive; the review page is where it gets flipped back, and an
        # override to flat now re-extracts the outline so the DXF still ships.
        if any(abs(thickness - g) <= 1.0 for g in _WOOD_GAUGE_MM):
            return "wood", (f"{thickness:.1f}mm board, {L:.0f} x {W:.0f} — "
                            "board gauge (sheet steel stops at 12mm)")
        return "flat", f"{thickness:.1f}mm plate, {L:.0f} x {W:.0f}"
    # There used to be two tube rules here — "two radii and long" and "a
    # thickness, long, six planes". They had no idea whether the cylinders
    # were the body or a bush pressed into it, and called a fabricated 6 mm
    # lifting link "OD30 x 3 wall" because it carried a Ø30 boss. Tube is
    # _analyse_tube's decision alone; if it said no, it is not tube.
    if thickness and T > thickness * 2.5 and ncyl >= 2:
        # "Bend faces" used to mean any cylinder at all. A moulded 8 mm block
        # 50 x 40 with fillets, bosses and fourteen free-form faces was "8.0mm
        # sheet, 21 bend faces" — a folded bracket — and the operator filed it
        # as bought-in on three packs running. A bend in sheet is a concentric
        # pair of cylinders a thickness apart; a part with none of those and a
        # crop of free-form faces is a moulding or a casting, not sheet metal.
        bends = _bend_pairs(m, thickness)
        nfree = kinds.get("other", 0)
        if bends == 0 and nfree >= _FREEFORM_MIN_FACES:
            return "bought_in", (f"{nfree} free-form faces and no sheet bends — "
                                 "a moulded or machined item, not fabricated")
        return "folded", f"{thickness:.1f}mm sheet, {ncyl} bend faces"
    if ncyl >= 2 and len(radii) >= 2 and L < 4 * W:
        return "turned", f"D{2 * radii[-1]:.0f} x L{L:.0f}"
    return "unknown", f"planes={kinds['plane']} cyl={ncyl}"


# ---------------------------------------------------------------------------
# Flat-pattern outline
# ---------------------------------------------------------------------------

def _face_outline_2d(face) -> tuple:
    """Every wire of a planar face as a closed 2-D polyline, in the face's own
    plane, normalised so the bounding box starts at (0, 0).

    The outer boundary and any holes come back in one list — which is what the
    nester and DXF writer expect, and they sort out which is which by area.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopoDS import TopoDS
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection
    from OCP.BRep import BRep_Tool

    plane = BRepAdaptor_Surface(face).Plane()
    pos = plane.Position()
    org, xd, yd = pos.Location(), pos.XDirection(), pos.YDirection()

    def to_2d(p):
        dx, dy, dz = p.X() - org.X(), p.Y() - org.Y(), p.Z() - org.Z()
        return (dx * xd.X() + dy * xd.Y() + dz * xd.Z(),
                dx * yd.X() + dy * yd.Y() + dz * yd.Z())

    def edge_primitive(edge, seq) -> "dict | list":
        """The edge as a native DXF entity where it is one.

        A hole sampled as a polyline is a polygon — 13 segments on a 3 mm hole,
        which is what makes small circles look faceted. A circular edge is
        emitted as a true arc instead, so the laser cuts a circle rather than
        a many-sided approximation. Anything else stays a polyline.
        """
        from OCP.GeomAbs import GeomAbs_CurveType
        curve = BRepAdaptor_Curve(edge)
        try:
            ctype = curve.GetType()
            if ctype == GeomAbs_CurveType.GeomAbs_Line and len(seq) >= 2:
                return {"kind": "line", "start": list(seq[0]),
                        "end": list(seq[-1])}
            if ctype == GeomAbs_CurveType.GeomAbs_Circle:
                circ = curve.Circle()
                centre = to_2d(circ.Location())
                radius = circ.Radius()
                u0, u1 = curve.FirstParameter(), curve.LastParameter()
                full = abs((u1 - u0) - 2 * math.pi) < 1e-6
                if full:
                    # ezdxf arcs need a sweep, and a full circle has none, so
                    # it goes out as two halves.
                    quarters = [(u0, u0 + math.pi), (u0 + math.pi, u1)]
                else:
                    quarters = [(u0, u1)]
                out = []
                for a, b in quarters:
                    out.append({
                        "kind": "arc", "center": list(centre),
                        "radius_mm": radius,
                        "start": list(to_2d(curve.Value(a))),
                        "end": list(to_2d(curve.Value(b))),
                        "mid": list(to_2d(curve.Value((a + b) / 2.0))),
                    })
                return out
        except Exception:
            pass
        return {"kind": "polyline", "points": [list(p) for p in seq]}

    def sample_edge(edge) -> list:
        """Points along one edge. An edge that cannot be tessellated falls back
        to its endpoints rather than being dropped: a missing edge leaves a hole
        in the loop that the closing step then bridges with a straight line
        straight across the part, which is a profile the laser will happily cut
        and the workshop will not spot until the part is wrong."""
        curve = BRepAdaptor_Curve(edge)
        try:
            sampler = GCPnts_QuasiUniformDeflection(curve, CHORD_TOL_MM)
            if sampler.IsDone() and sampler.NbPoints() >= 2:
                return [to_2d(sampler.Value(i))
                        for i in range(1, sampler.NbPoints() + 1)]
        except Exception:
            pass
        try:                              # straight-line fallback
            return [to_2d(curve.Value(curve.FirstParameter())),
                    to_2d(curve.Value(curve.LastParameter()))]
        except Exception:
            return []

    loops: list[list] = []
    prims: list = []
    gaps: list[float] = []
    wexp = TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_WIRE)
    while wexp.More():
        wire = TopoDS.Wire_s(wexp.Current())
        pts: list = []
        # Pass the face: on a wire that runs over a seam, the explorer needs it
        # to order and orient the edges correctly.
        we = BRepTools_WireExplorer(wire, face)
        while we.More():
            edge = we.Current()
            if BRep_Tool.Degenerated_s(edge):
                we.Next()
                continue
            seq = sample_edge(edge)
            if seq:
                # Each edge is sampled along its OWN parameter direction, so
                # some arrive back-to-front. The explorer already knows which
                # vertex the wire enters this edge by — anchor to that instead
                # of guessing from proximity, which picked the wrong end on
                # short edges and left the loop 0.8 mm open.
                start = to_2d(BRep_Tool.Pnt_s(we.CurrentVertex()))
                if (math.hypot(seq[-1][0] - start[0], seq[-1][1] - start[1])
                        < math.hypot(seq[0][0] - start[0], seq[0][1] - start[1])):
                    seq.reverse()
                prim = edge_primitive(edge, seq)
                prims.extend([prim] if isinstance(prim, dict) else prim)
                if not pts:
                    pts = seq
                else:
                    gap = math.hypot(pts[-1][0] - seq[0][0],
                                     pts[-1][1] - seq[0][1])
                    if gap > OUTLINE_GAP_TOL_MM:
                        gaps.append(gap)
                    pts.extend(seq[1:])
            we.Next()
        if len(pts) >= 3:
            close_gap = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
            if close_gap > OUTLINE_GAP_TOL_MM:
                gaps.append(close_gap)
            if close_gap > 1e-9:
                pts.append(pts[0])
            loops.append(pts)
        wexp.Next()

    if gaps:
        # Loud, not silent: a bridged gap means the drawn profile is not the
        # part. Better to reject the outline than to cut the wrong shape.
        raise OutlineError(
            f"profile does not close: {len(gaps)} gap(s), largest "
            f"{max(gaps):.2f} mm")

    if not loops:
        return [], []
    minx = min(p[0] for lp in loops for p in lp)
    miny = min(p[1] for lp in loops for p in lp)
    shifted = [[[p[0] - minx, p[1] - miny] for p in lp] for lp in loops]
    return shifted, _shift_primitives(prims, -minx, -miny)


def _shift_primitives(prims: list, dx: float, dy: float) -> list:
    """Move primitives by the same amount the loops were, so the two describe
    the same profile — the sheet writer refuses to use them otherwise."""
    out = []
    for p in prims:
        q = dict(p)
        for key in ("start", "end", "mid", "center"):
            if key in q:
                q[key] = [q[key][0] + dx, q[key][1] + dy]
        if "points" in q:
            q["points"] = [[x + dx, y + dy] for x, y in q["points"]]
        out.append(q)
    return out


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def _coaxial_groups(cyls: list) -> list:
    """Cylindrical faces grouped by the axis they share.

    On round stock the bore and the skin share the tube's axis. On box section
    the four corner radii also share a DIRECTION with the axis but sit off it,
    so grouping on the full axis line — direction and position — keeps them
    apart, which is what stopped a rounded-corner box being read as round.
    """
    groups: list[dict] = []
    for c in cyls:
        for g in groups:
            same_dir = abs(abs(
                c["dir"][0] * g["dir"][0] + c["dir"][1] * g["dir"][1]
                + c["dir"][2] * g["dir"][2]) - 1.0) < 1e-6
            if not same_dir:
                continue
            d = [c["loc"][i] - g["loc"][i] for i in range(3)]
            along = sum(d[i] * g["dir"][i] for i in range(3))
            perp = math.sqrt(max(sum(d[i] * d[i] for i in range(3))
                                 - along * along, 0.0))
            if perp < 1e-4:                     # same axis LINE, not just parallel
                g["radii"].append(c["radius"])
                g["area"] += c["area"]
                r = round(c["radius"], 3)
                g["by_r"][r] = g["by_r"].get(r, 0.0) + c["area"]
                break
        else:
            groups.append({"dir": c["dir"], "loc": c["loc"],
                           "radii": [c["radius"]], "area": c["area"],
                           # area per radius: tells a bore that runs the full
                           # length from a shallow counterbore of the same radius
                           "by_r": {round(c["radius"], 3): c["area"]}})
    return groups


def _dominant_cylinder(m: dict, shape) -> "dict | None":
    """The coaxial cylinder family carrying most of the surface, with the
    solid's extents about that axis — what a body of revolution looks like.
    None when there is no cylindrical face at all."""
    groups = sorted(_coaxial_groups(m["cyls"]), key=lambda g: -g["area"])
    if not groups or not m["total_area"]:
        return None
    g = groups[0]
    radii = sorted({round(r, 3) for r in g["radii"]})
    try:
        length, big, small = _section_extents(shape, g["dir"])
    except Exception:                                      # noqa: BLE001
        return None
    return {"frac": g["area"] / m["total_area"], "od": 2 * radii[-1],
            "radii": radii, "length": length, "cross": (big, small)}


def _section_extents(shape, axis, x_ref=None) -> "tuple[float, float, float]":
    """Bounding box of `shape` in a frame whose Z is `axis`, as
    (length_along_axis, larger_cross_dim, smaller_cross_dim).

    Measured about the tube's own axis rather than the world axes: a tube lying
    at an angle in the assembly has a world bounding box inflated by the
    diagonal, which is how a 40x40 box came out quoted as 44.5.

    `x_ref` fixes the rotation ABOUT that axis. Without it the section is
    measured at whatever angle the frame happened to land on, which inflates it
    just as badly — a 50x30 read as 53.5x51. For box section it should be the
    normal of one of the flat walls.
    """
    from OCP.gp import gp_Trsf, gp_Ax3, gp_Pnt, gp_Dir
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    z = gp_Dir(axis[0], axis[1], axis[2])
    if x_ref is not None:
        ax3 = gp_Ax3(gp_Pnt(0, 0, 0), z, gp_Dir(x_ref[0], x_ref[1], x_ref[2]))
    else:
        ax3 = gp_Ax3(gp_Pnt(0, 0, 0), z)
    trsf = gp_Trsf()
    trsf.SetTransformation(ax3)
    aligned = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    box = Bnd_Box()
    BRepBndLib.Add_s(aligned, box, True)
    xm, ym, zm, xM, yM, zM = box.Get()
    cross = sorted([xM - xm, yM - ym])
    return (zM - zm, cross[1], cross[0])


def _wall_from_side_planes(planes, axis) -> "float | None":
    """Wall thickness of box section: the SMALLEST gap between parallel side
    walls, not the largest-area pair.

    The bore faces of a box tube are its biggest, so picking by area returned
    the cavity width — a 40x40x2 reported a 36 mm wall.
    """
    groups: dict[tuple, list] = collections.defaultdict(list)
    for n, offset, area, _face in planes:
        along = abs(n[0] * axis[0] + n[1] * axis[1] + n[2] * axis[2])
        if along > 1e-3:                 # not a side wall; it's an end cut
            continue
        groups[tuple(round(c, 4) for c in n)].append(offset)
    # The wall is the step in from the OUTSIDE face on each side. Taking the
    # smallest gap anywhere let an internal feature — a rib, a slot, a small
    # step — stand in for it and report a 0.5 mm wall on much thicker stock.
    gaps = []
    for offsets in groups.values():
        offsets = sorted(set(round(o, 4) for o in offsets))
        if len(offsets) < 2:
            continue
        for gap in (offsets[1] - offsets[0], offsets[-1] - offsets[-2]):
            if 0.5 <= gap <= 8:
                gaps.append(gap)
    if not gaps:
        return None
    gaps.sort()
    return gaps[len(gaps) // 2]          # median: robust to one odd face


def _axis_from_planes(planes) -> "tuple | None":
    """The extrusion axis of a prism that has no cylindrical faces to read it
    from: the direction every side wall is parallel to.

    Scored by face area so end cuts and small features cannot outvote the
    walls, which is what a sharp-cornered box section needs.

    Every pair of face normals proposes an axis (their cross product); the axis
    whose perpendicular faces carry the most area wins, the first pair in
    (i, j) order on a tie. That used to be a triple loop in Python — pairs x
    faces — and on a 480-face router jig it took 5.8 s of every upload (more
    on the server). Most pairs propose the very same axis, and a score depends
    only on the axis, so each distinct axis is scored once, as array maths.
    The arithmetic is kept bit-for-bit the loop's — including Python 3.12's
    compensated sum() — so every tie still breaks exactly as it did.
    """
    import numpy as np
    if len(planes) < 2:
        return None
    N = np.array([n for n, _o, _a, _f in planes], dtype=float)
    A = np.array([a for _n, _o, a, _f in planes], dtype=float)
    i, j = np.triu_indices(len(N), k=1)            # row-major: (0,1), (0,2), ...
    n1, n2 = N[i], N[j]
    ax = np.stack([n1[:, 1] * n2[:, 2] - n1[:, 2] * n2[:, 1],
                   n1[:, 2] * n2[:, 0] - n1[:, 0] * n2[:, 2],
                   n1[:, 0] * n2[:, 1] - n1[:, 1] * n2[:, 0]], axis=1)
    mag = np.sqrt(_pysum_rows([ax[:, 0] * ax[:, 0], ax[:, 1] * ax[:, 1],
                               ax[:, 2] * ax[:, 2]]))
    keep = mag >= 0.3                               # nearly parallel: no axis from these
    if not keep.any():
        return None
    ax = ax[keep] / mag[keep][:, None]
    # Distinct axes, each with the index of the first pair that proposed it.
    uniq, first = np.unique(ax, axis=0, return_index=True)
    order = np.argsort(first)
    uniq, first = uniq[order], first[order]
    terms = []
    for k in range(len(N)):
        dot = (uniq[:, 0] * N[k, 0] + uniq[:, 1] * N[k, 1]) + uniq[:, 2] * N[k, 2]
        terms.append(np.where(np.abs(dot) < 1e-3, A[k], 0.0))
    scores = _pysum_rows(terms)
    k = int(np.argmax(scores))                      # first pair order, as the loop kept
    if not scores[k] > 0.0:
        return None
    return tuple(float(c) for c in ax[first[k]])


def _pysum_rows(columns):
    """Element-wise sum of a sequence of equal-length arrays, rounded exactly as
    Python 3.12's built-in sum() rounds a sequence of floats (Neumaier
    compensated summation). A zero term changes nothing, as skipping it would."""
    import numpy as np
    f = np.zeros_like(columns[0], dtype=float)
    c = np.zeros_like(f)
    for x in columns:
        t = f + x
        c += np.where(np.abs(f) >= np.abs(x), (f - t) + x, (x - t) + f)
        f = t
    return np.where((c != 0.0) & np.isfinite(c), f + c, f)


def _analyse_tube(m: dict, shape, force: bool = False) -> "dict | None":
    """Decide whether this is round or box stock, and measure the section
    about its own axis. None when it is not tube at all.

    Tube is a SECTION, not a length. A 37 mm offcut of 50x50 SHS is still
    50x50 SHS, and the old "at least three sections long" guard sent every
    short stub off to be called a folded bracket (the corner radii read as
    bends) or a turned boss (a hole read as a bore) — five parts on one
    frame assembly, every one hand-corrected. What makes a body tube is that
    its section is a CLOSED hollow with one consistent thin wall, so that is
    what is tested here, and length is not.

    `force` keeps its meaning: the operator has already said this is tube and
    only the size is in question, so the shape tests are skipped.

    A section that measures as tube but does not run straight — its envelope
    about the axis is bigger than the section — comes back with "bent": True.
    It is real tube, but not a straight length the tube laser can cut, and
    the caller flags it rather than posting it as stock.
    """
    cyls = m["cyls"]
    total_area = m["total_area"] or 1.0
    groups = sorted(_coaxial_groups(cyls), key=lambda g: -g["area"])

    # Round stock: one axis carrying the bore and the skin, dominating the
    # surface. Holes elsewhere have their own axes and cannot contribute.
    for g in groups:
        radii = sorted({round(r, 3) for r in g["radii"]})
        if len(radii) < 2 or g["area"] < 0.4 * total_area:
            continue
        wall = radii[-1] - radii[-2]
        if not (0.5 <= wall <= 8):
            continue
        od = 2 * radii[-1]
        length, env_big, env_small = _section_extents(shape, g["dir"])
        if not force:
            # Tube stock is a skin and ONE bore. A second, smaller radius on
            # the same axis is a counterbore or a stepped hole, and a wall a
            # quarter of the diameter is bar with a hole in it: both are
            # turned parts, however long they are. A Ø20 boss with a stepped
            # Ø14/Ø10 bore and a Ø20 pin with a Ø10 hole both read as "OD20
            # tube" before this.
            # Only radii that carry real area count as steps: a 55 mm² chamfer
            # at the mouth of a Ø24x4 tube's bore is not a counterbore.
            by_r = g.get("by_r", {})
            significant = [r for r in radii if by_r.get(r, 0.0) >= 0.05 * g["area"]]
            if len(significant) > 2 or wall > 0.2 * od:
                continue
            # And the bore must run the whole length. A shallow counterbore
            # in a solid boss shows the same two radii as a tube; only the
            # face area tells them apart.
            bore_area = g.get("by_r", {}).get(radii[-2], 0.0)
            if bore_area < 0.7 * (2 * math.pi * radii[-2] * length):
                continue
        if not force and length < 0.5 * od:
            # A ring thinner than half its diameter — a Ø18 x 6 spacer — is a
            # laser-cut disc from plate, which is how the workshop makes it.
            continue
        # Round stock is NOT snapped: standard pipe is 26.9, 33.7, 42.4, 48.3 OD.
        sec = {"kind": "CHS", "w": round(od, 1), "h": round(od, 1),
               "wall": _snap_wall(wall), "length": length, "axis": g["dir"]}
        # A straight tube's envelope IS its section. Bigger means it is bent,
        # or something is joined to it, and neither is straight stock.
        if not force and max(env_big, env_small) > od + 1.0:
            sec["bent"] = True
            sec["envelope"] = (round(env_big, 1), round(env_small, 1))
        return sec

    # Box stock: the corner radii run along the tube, so their shared direction
    # is the axis. Plenty of box section has SHARP corners and therefore no
    # cylindrical faces at all, so fall back to the walls themselves — the axis
    # is the one direction every side wall's normal is perpendicular to.
    # The walls decide the axis: it is the one direction every side wall's
    # normal is perpendicular to. On real tube the corner radii agree with it —
    # but a cross-drilled hole is a cylinder family too, and on sharp-cornered
    # box it is the ONLY one, so taking the biggest family's axis measured a
    # 50x50 stub across its Ø13 hole instead of along the tube.
    axis = _axis_from_planes(m["planes"])
    if axis is None and groups:
        axis = groups[0]["dir"]
    if axis is None:
        return None

    side_planes = [(n, a) for n, _o, a, _f in m["planes"]
                   if abs(n[0] * axis[0] + n[1] * axis[1]
                          + n[2] * axis[2]) < 1e-3]
    if len(side_planes) < 4:
        return None
    x_ref = max(side_planes, key=lambda t: t[1])[0]

    length, env_big, env_small = _section_extents(shape, axis, x_ref)
    big, small = env_big, env_small

    wall = _wall_from_side_planes(m["planes"], axis)
    if not wall:
        return None

    # The SECTION is measured from the side walls themselves, not the bounding
    # box. The box is inflated by anything that is not plain tube — a mitred
    # L-frame measured about one leg's axis reads the OTHER leg's length as a
    # cross dimension (a 20x20x2 frame shipped named 90.4x20x2), and tabs or
    # angled end cuts do the same in miniature. The outermost parallel-wall
    # pair per normal family is the stock size however the ends are cut.
    spans = []          # (span across the family, its normal, total area, walls)
    fam: dict = {}
    for n, offset, area, _f in m["planes"]:
        along = abs(n[0] * axis[0] + n[1] * axis[1] + n[2] * axis[2])
        if along > 1e-3:
            continue
        key = tuple(round(c, 4) for c in n)
        fam.setdefault(key, {"offsets": [], "area": 0.0})
        fam[key]["offsets"].append(offset)
        fam[key]["area"] += area
    for key, g in fam.items():
        offs = sorted(set(round(o, 3) for o in g["offsets"]))
        # A real tube wall is an OUTER face with a BORE face one wall-thickness
        # behind it; anything without such a partner — another leg's end cap,
        # a boss face — is not a wall and must not stretch the section. This is
        # what kept the L-frame honest: its second leg's end cap sat 90mm out
        # in this family and turned a 20-wide section into a 100.
        paired = set()
        for i, a in enumerate(offs):
            for b in offs[i + 1:]:
                if 0.5 <= b - a <= 8.0:
                    paired.add(a); paired.add(b)
        # Count the WALLS in the family: adjacent faces one wall-thickness
        # apart, the same thickness as the rest of the section. This is what
        # tells a closed box from an open channel. A 4 mm U-channel has two
        # flanges in one family but a single web in the other — plus a 0.5 mm
        # lip that the looser pairing above accepted as a second "wall", which
        # is how it shipped as RHS 45x21x4, a size that does not exist.
        walls = sum(1 for a, b in zip(offs, offs[1:])
                    if abs((b - a) - wall) <= 0.25 * wall + 0.05)
        if len(paired) >= 2:
            span = max(paired) - min(paired)
            if span > 1.0:
                spans.append((span, key, g["area"], walls))
    closed = False
    if spans:
        spans.sort(key=lambda t: -t[2])              # biggest walls first
        w1 = spans[0]
        perp = [sp for sp in spans[1:]
                if abs(sum(a * b for a, b in zip(sp[1], w1[1]))) < 0.2]
        if perp:
            big, small = max(w1[0], perp[0][0]), min(w1[0], perp[0][0])
            closed = w1[3] >= 2 and perp[0][3] >= 2
    if not force and not closed:
        return None
    if not force and small - 2 * wall < 1.0:
        # Two "walls" with nothing between them is a solid bar with grooves in
        # it, not a hollow section — an "RHS 18.5x6 x 3 wall" cannot exist.
        return None
    if not force and m["kinds"].get("torus"):
        # Rolled tube has no toroidal faces; a sheet box FOLDED closed does, at
        # every corner where two bends meet. A 3 mm folded enclosure measured
        # as a closed hollow with two walls per family — which it is — and
        # would have shipped as "RHS 88x69", a size nobody rolls.
        return None
    kind = "SHS" if abs(big - small) < 0.5 else "RHS"
    sec = {"kind": kind, "w": _snap_stock_dim(big), "h": _snap_stock_dim(small),
           "wall": _snap_wall(wall), "length": length, "axis": axis}
    if not force and (env_big > big + 2.0 or env_small > small + 2.0):
        if m["kinds"]["plane"] > 14:
            # A tube with one bend has four walls a leg, two ends and a cut
            # face or two. A body that reads as a closed section, is bigger
            # than it, AND carries twenty-eight planes is a folded 4 mm part
            # whose return flanges paired up as walls — it went from "folded"
            # to "not straight stock" on the strength of that pairing.
            return None
        sec["bent"] = True
        sec["envelope"] = (round(env_big, 1), round(env_small, 1))
    return sec


def _signature(m: dict, thickness: "float | None") -> tuple:
    """A fingerprint that is equal for two copies of the same part.

    Volume, surface area, the sorted bounding box and the face-type census
    together are enough to separate genuinely different parts, and are
    invariant to however the assembly happens to have placed each instance.
    """
    kinds = m["kinds"]
    return (
        round(m["volume"], 3),
        round(sum(a for _n, _d, a, _f in m["planes"]), 3),
        tuple(round(e, 3) for e in m["extents"]),
        tuple(sorted(kinds.items())),
        tuple(sorted(round(r, 3) for r in m["radii"])),
        round(thickness, 3) if thickness else None,
    )


_MIRROR_RE = re.compile(r"mirror", re.I)
_CHIRAL_TOL = 1e-12       # a mirror-symmetric body scores 0 to rounding (~1e-17);
                          # a handed one 1e-8 and up on the normalised moments


def _chirality(shape) -> int:
    """+1 or -1 for a handed body, 0 for one with a mirror plane (see _chirality_score)."""
    total = _chirality_score(shape)
    if abs(total) < _CHIRAL_TOL:
        return 0
    return 1 if total > 0 else -1


def _chirality_score(shape) -> float:
    """The signed handedness of a body: positive or negative for a handed one,
    zero (to rounding) for one with a mirror plane.

    Every term of the fingerprint — volume, area, box, face census, radii — is
    the same for a part and its mirror image, so a left and a right pivot tube
    merged into "2 off" of one of them and the workshop was a hand short. What
    tells them apart is a pseudo-scalar: a quantity a rotation leaves alone and
    a reflection negates. Take the face centroids v_i about the area centroid
    (weights w_i = face area) and a scalar s_i per face; the three moment
    vectors

        m_k = Σ_i w_i s_i^k v_i        k = 1, 2, 3

    rotate with the body, so det[m_1 m_2 m_3] is unchanged by any rotation and
    negated by any reflection. A body with a mirror plane is its own mirror
    image, so its determinant is zero to rounding; a handed body's is not, and
    its mirror image scores exactly the negative. Two scalars are tried — the
    centroid radius and the face area — and the stronger answer taken, so a
    part whose radii happen to line up is still read. (An earlier version
    summed det(v_i, v_j, v_k) over triples; by Cauchy–Binet every such sum is a
    3x3 determinant of moments, and with the centred first moment as a column
    it vanishes identically. This form is that determinant done properly.)

    A flat plate and its "mirror" both score 0 — a handed 2D outline is the
    same plate turned over, which is right.
    """
    import numpy as np
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopoDS import TopoDS
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    pts, wts = [], []
    exp = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(TopoDS.Face_s(exp.Current()), props)
        c = props.CentreOfMass()
        pts.append((c.X(), c.Y(), c.Z()))
        wts.append(abs(props.Mass()))       # a reversed face reports a negative area
        exp.Next()
    if len(pts) < 4:
        return 0.0
    v = np.asarray(pts, float)
    w = np.asarray(wts, float)
    w = w / w.sum()
    c = (w[:, None] * v).sum(axis=0)
    v = v - c
    scale = np.sqrt((w * (v ** 2).sum(axis=1)).sum()) or 1.0
    v = v / scale
    def pseudo(scalar):
        m = np.stack([((w * scalar ** k)[:, None] * v).sum(axis=0) for k in (1, 2, 3)], axis=1)
        return float(np.linalg.det(m))
    radius = np.linalg.norm(v, axis=1)
    area = w / w.max()
    total = max((pseudo(radius), pseudo(area)), key=abs)
    return total if np.isfinite(total) else 0.0


def _may_merge(a: "StepPart", b: "StepPart") -> bool:
    """Two parts with the same fingerprint are still not interchangeable if one
    is the handed version of the other — a left and a right bracket have
    identical volume, area and bounding box, and cutting two of one would be
    wrong. The handedness measured from the geometry decides (see _chirality);
    the "-Mirrored" naming convention is kept as a second opinion."""
    if a.chirality != b.chirality:
        return False
    return bool(_MIRROR_RE.search(a.name)) == bool(_MIRROR_RE.search(b.name))


def _merge_duplicates(parts: "list[StepPart]") -> "list[StepPart]":
    """Fold identical parts into one entry with the quantities added up.

    An assembly names every instance separately, so nine identical spacers
    arrive as nine one-off parts and would be cut as nine separate files. The
    workshop wants one file marked 9 off.
    """
    out: list[StepPart] = []
    for part in parts:
        for kept in out:
            if (kept.signature and kept.signature == part.signature
                    and kept.process == part.process
                    and _may_merge(kept, part)):
                kept.qty += part.qty
                kept.merged_from.append(part.name)
                kept.merged_qty[part.name] = kept.merged_qty.get(part.name, 0) + part.qty
                for sub in part.sub_assemblies:
                    if sub not in kept.sub_assemblies:
                        kept.sub_assemblies.append(sub)
                for sub, n in (part.sub_qty or {}).items():
                    kept.sub_qty[sub] = kept.sub_qty.get(sub, 0) + n
                break
        else:
            out.append(part)
    return out


def read_step(path: str) -> "list[StepPart]":
    """Every distinct part in the assembly, measured and classified, with flat
    outlines extracted for anything that is a plate."""
    _ocp()
    doc = _read_document(path)
    parts: list[StepPart] = []

    for name, rec in _collect_parts(doc).items():
        shape = rec["shape"]
        try:
            m = _measure(shape)
        except Exception as exc:
            parts.append(StepPart(name=name, qty=rec["qty"], process="error",
                                  length_mm=0, width_mm=0, thickness_mm=0,
                                  volume_mm3=0, detail=str(exc)[:120]))
            continue

        thickness, widest_face, plate_normal = _wall_thickness(m["planes"])
        m["wall_gap"] = thickness
        # Measure the plate about its OWN normal. On the world bounding box a
        # plate lying at an angle looks far thicker than it is, so the flat test
        # failed and it fell through to the catch-all, arriving as a STEP.
        if plate_normal is not None:
            try:
                m["along_normal"] = _section_extents(shape, plate_normal)[0]
            except Exception:
                m["along_normal"] = None
        else:
            m["along_normal"] = None
        m["dominant_cyl"] = _dominant_cylinder(m, shape)
        tube = _analyse_tube(m, shape)
        process, detail = _classify(m, thickness)
        if tube and tube.get("bent"):
            # Real tube section, but not a straight length: bent, or with
            # features that reach beyond the section. Posting it as stock would
            # hand the tube laser a file it cannot cut, so it goes to the review
            # page with the section it IS and the envelope it isn't.
            eb, es = tube["envelope"]
            size = (f"CHS OD{tube['w']:g}" if tube["kind"] == "CHS"
                    else f"{tube['kind']} {tube['w']:g}x{tube['h']:g}")
            process = "unknown"
            detail = (f"{size} x {tube['wall']:g} wall, but envelope {eb:g} x {es:g}: "
                      "bent, or carrying features beyond the section — not a straight "
                      "length (no tube bender: re-model as a mitred frame, or review)")
            tube = None
        if tube:
            process = "tube_chs" if tube["kind"] == "CHS" else "tube_shs"
            detail = (f"{tube['kind']} "
                      + (f"OD{tube['w']:g}" if tube["kind"] == "CHS"
                         else f"{tube['w']:g}x{tube['h']:g}")
                      + f" x {tube['wall']:g} wall, L{tube['length']:.0f}")
        # A composite made of several tube segments is a mitred frame. Measured
        # as one body it looks like whatever its overall envelope resembles —
        # a closed frame reads as a 4 mm folded sheet, an L-shaped one as turned
        # bar — and would ship a bogus DXF or drawing. Flagged as unclassified
        # instead, with the section and segment count on the row, so it lands in
        # front of the operator on the review page rather than being quietly got
        # wrong. It cannot be auto-unfolded here (see module docstring).
        frame = _mitred_frame(shape)
        if frame:
            # Its own process, so the review page can offer the two frame
            # rewrites on exactly the parts they apply to.
            process, detail = "tube_mitered", frame
            if tube is None:
                # The chain as a whole is not a straight tube (that is the
                # point), so take the stock size from one straight leg — but
                # keep the composite's own envelope as its length.
                leg = frame_section(shape)
                if leg:
                    tube = dict(leg, length=m["extents"][2])

        # Hardware is bought in whatever its geometry looks like — a washer is a
        # flat disc and a bolt is a rod, and neither should be manufactured.
        if _is_bought_in(name):
            process, detail = "bought_in", "hardware (bought in, by name)"
        T, W, L = m["extents"]

        # `thickness` is the gap between the widest parallel plane pair, which
        # is the sheet thickness on a plate but meaningless on round stock —
        # there it latches onto the end cuts and reported a 25x1.5 CHS as 4mm
        # wall. Give each process the number that actually means something.
        radii = sorted({round(r, 3) for r in m["radii"]})
        kind, sec_w, sec_h = "", 0.0, 0.0
        if tube:
            kind, sec_w, sec_h = tube["kind"], tube["w"], tube["h"]
            L = tube["length"]
        if tube:
            wall = tube["wall"]
        elif process in ("flat", "folded"):
            wall = thickness or T
        else:                       # rod, turned: no wall, quote the section
            wall = T

        edges, edge_prims = [], []
        if process == "flat" and widest_face is not None:
            try:
                edges, edge_prims = _face_outline_2d(widest_face)
            except Exception as exc:
                # Flag it rather than silently falling back to a STEP, which is
                # what hid bad profiles before.
                process = "outline_failed"
                detail += f" | PROFILE FAILED: {str(exc)[:90]}"

        # For a plate the flat outline IS the true size; the world bounding box
        # of an angled part is inflated by its own tilt and would put a wrong
        # size on the cut list and the reference.
        if edges:
            xs = [q[0] for lp in edges for q in lp]
            ys = [q[1] for lp in edges for q in lp]
            if xs and ys:
                a, b = max(xs) - min(xs), max(ys) - min(ys)
                L, W = (a, b) if a >= b else (b, a)
        elif tube:
            L, W = tube["length"], tube["w"]

        parts.append(StepPart(
            name=name, qty=rec["qty"], process=process,
            length_mm=L, width_mm=W, thickness_mm=wall,
            volume_mm3=m["volume"], detail=detail, edges=edges, shape=shape,
            edges_primitive=edge_prims,
            sub_assemblies=list(rec.get("subs") or []),
            sub_qty=dict(rec.get("sub_qty") or {}),
            depth_mm=T, section_kind=kind, section_w=sec_w,
            section_h=sec_h, signature=_signature(m, thickness),
            chirality=_chirality(shape),
        ))

    return _merge_duplicates(parts)


def _mitred_frame(shape) -> str:
    """Describe a composite that is a chain of tube segments, or "".

    Two shapes turn up. A MITRED frame is straight segments cut at an angle and
    welded, and its blank is one straight length, so the developed length is
    worth quoting: it is the sum of the segments measured through their centres,
    because a mitre makes the outer wall longer than the inner one and the
    bounding box would overstate the cut.

    A BENT frame has formed corners instead (toroidal faces). No developed
    length is quoted for one: the workshop has no tube bender, so it has to be
    re-modelled as a mitred frame before it can be made, and a part-measured
    length would be a number someone could order stock against.
    """
    solids = _solids_of(shape)
    if len(solids) < 2:
        return ""

    straights, bends = [], 0
    for solid in solids:
        try:
            m = _measure(solid)
            m["wall_gap"] = _wall_thickness(m["planes"])[0]
        except Exception:
            return ""
        if m["kinds"].get("torus"):
            bends += 1              # a formed corner, not a straight length
            continue
        sec = _analyse_tube(m, solid, force=True)
        if not sec:
            return ""              # not tube at all, so not a frame
        straights.append((sec, m))

    if len(straights) < 2:
        return ""
    first = straights[0][0]
    for sec, _m in straights[1:]:
        if (sec["kind"] != first["kind"]
                or abs(sec["w"] - first["w"]) > 0.5
                or abs(sec["h"] - first["h"]) > 0.5
                or abs(sec["wall"] - first["wall"]) > 0.3):
            return ""              # an assortment welded together, not a frame

    size = (f"CHS OD{first['w']:g}" if first["kind"] == "CHS"
            else f"{first['kind']} {first['w']:g}x{first['h']:g}")
    stock = f"{len(straights)} x {size} x {first['wall']:g} wall"
    if bends:
        return (f"bent frame: {stock} + {bends} formed bend(s) - NO TUBE BENDER, "
                f"re-model as a mitred frame")
    total = sum(_centre_length(m, sec["axis"]) or sec["length"]
                for sec, m in straights)
    return (f"mitred frame: {stock}, developed length {total:.0f} mm "
            f"- NEEDS UNFOLDING")


def _centre_length(m: dict, axis) -> float:
    """Length of one mitred segment along its centreline.

    The two end cuts are planes, so the axial position of each at the section
    centroid is what bounds the blank; the corner where the bounding box reaches
    furthest belongs to the long side of a mitre, not the centre.
    """
    ends = []
    for normal, _offset, area, face in m["planes"]:
        if abs(_dot3(normal, axis)) < 0.2:      # a wall, not an end cut
            continue
        ends.append((_face_centre(face), area))
    if len(ends) < 2:
        return 0.0
    ends.sort(key=lambda e: -e[1])              # the two biggest are the cuts
    a, b = ends[0][0], ends[1][0]
    return abs(_dot3((b[0] - a[0], b[1] - a[1], b[2] - a[2]), axis))


def _face_centre(face) -> tuple:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    c = props.CentreOfMass()
    return (c.X(), c.Y(), c.Z())


def _dot3(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def frame_section(shape) -> "dict | None":
    """The stock section of a frame, measured from one straight segment.

    Needed to name the file when the operator has chosen NOT to unfold a
    mitred frame: the chain as a whole has no single axis to measure, but every
    leg is cut from the same stock.
    """
    _ocp()
    for solid in _solids_of(shape):
        try:
            m = _measure(solid)
            if m["kinds"].get("torus"):
                continue                    # a formed bend, not a straight leg
            m["wall_gap"] = _wall_thickness(m["planes"])[0]
            sec = _analyse_tube(m, solid, force=True)
            if sec:
                return sec
        except Exception:
            continue
    return None


def tube_section(part) -> "dict | None":
    """Measure the section of a part already known to be tube.

    An operator override says "this is tube" but carries no section, and a
    short offcut is rejected by the length guards, so both used to end up named
    UNK. Measured with those guards off, since the question is only how big the
    section is, not whether it is tube.
    """
    _ocp()
    try:
        m = _measure(part.shape)
        m["wall_gap"] = _wall_thickness(m["planes"])[0]
        return _analyse_tube(m, part.shape, force=True)
    except Exception:
        return None


def flat_records(parts: "list[StepPart]") -> list:
    """Just the plates, in the dict form the nester consumes."""
    return [p.as_flat_record() for p in parts
            if p.process == "flat" and p.edges]


class _quiet_stdout:
    """Silence OpenCascade's writer.

    It prints a dozen lines of transfer statistics per file straight from C++,
    which would bury the run log and the progress display in the web UI. Python
    can't intercept that, so the file descriptor itself is redirected.
    """

    def __enter__(self):
        import os
        self._saved = os.dup(1)
        self._null = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self._null, 1)
        return self

    def __exit__(self, *exc):
        import os
        os.dup2(self._saved, 1)
        os.close(self._null)
        os.close(self._saved)
        return False


def write_solid_step(shape, path: str) -> None:
    """Write one solid out as its own STEP file.

    This is all a folded part needs: the workshop unfolds it in the Bystronic
    software downstream, so the exporter only has to recognise it and hand over
    the solid. Same for tube, rod and turned stock.
    """
    _ocp()
    from OCP.STEPControl import STEPControl_Writer, STEPControl_StepModelType
    from OCP.IFSelect import IFSelect_ReturnStatus

    with _quiet_stdout():
        writer = STEPControl_Writer()
        writer.Transfer(shape, STEPControl_StepModelType.STEPControl_AsIs)
        status = writer.Write(path)
    if status != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise RuntimeError(f"failed writing STEP: {path}")


def stock_description(part: "StepPart") -> str:
    """Short stock call-out for filenames and the cut list, matching the
    convention the API-driven pack already uses (`25x1.5mm_CHS`, `D4`, `4mm`)."""
    if part.process == "flat":
        return f"{part.thickness_mm:.0f}mm"
    if part.process == "rod":
        return f"D{part.width_mm:.0f}"
    if part.process in ("tube_chs", "tube_shs"):
        # detail already carries the section; keep it filename-safe
        return part.detail.split(",")[0].replace(" ", "").replace("x", "x")
    return part.process
