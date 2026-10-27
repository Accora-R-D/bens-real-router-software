"""Config models for the router CAM engine — what the web UI lets the team edit.

Material: feeds/speeds/depths per material+thickness (replaces the hardcoded MAT).
Seat/ToolTable: the 10-seat tool changer — which cutter is loaded in each seat
(replaces the hardcoded TOOLS). The engine selects the widest loaded tool that
fits each feature and posts that seat's T-number.
"""
from __future__ import annotations
from dataclasses import dataclass, field, asdict


def _feed(v: float) -> float:
    """A feed rate, mm/min, to two significant figures. It was rounded to the
    nearest 100, which is invisible at MDF feeds (every one of them lies in
    1000-9999 mm/min, where the two agree exactly) but coarse for metal: the
    aluminium preset's 480 mm/min came out as 500 and its 160 mm/min plunge as
    200 — faster than the reviewed numbers, the wrong way to be wrong."""
    import math
    if v <= 0:
        return 0.0
    return float(round(v, 1 - int(math.floor(math.log10(v)))))


@dataclass
class Material:
    name: str = "18mm MDF"
    thickness_mm: float = 18.0
    chipload: float = 0.146          # mm/tooth, cutting
    plunge_chipload: float = 0.052   # mm/tooth, plunging
    spindle_rpm: int = 24000
    max_step: float = 6.2            # max depth of cut per pass (mm)
    final_z: float = -0.5            # cut-through depth into spoilboard
    # Rapid plane above the NOMINAL stock top. Was 6: on a sheet that is bowed
    # or sitting on a chip the cutter skimmed the surface between parts. 15 mm
    # clears a badly bowed board and costs a few seconds of Z travel a sheet.
    clearance_over: float = 15.0
    tab_remain: float = 2.0          # holding-tab height (mm)
    tab_width: float = 8.0           # holding-tab length (mm)
    # Off by default: a holding tab has to be cut off and sanded back, so it is
    # opt-in per job rather than something that quietly appears on small parts.
    tabs_mode: str = "off"           # "auto" (small parts) | "on" (all) | "off" (none)
    grain_locked: bool = False       # sheet has a grain (ply): nest only at 0/180°, never 90/270°
    engrave: bool = False            # engrave the part number on each part
    engrave_height: float = 25.0     # max text cap-height (mm); auto-shrinks to fit
    # Identical parts are folded into one (one file, N off) and every copy is
    # engraved with one name. Ticked, each keeps its own STEP name instead.
    unique_names: bool = False
    # Engrave "<name> <assembly initials>" — e.g. "X1 FEC" — so parts read from
    # several assemblies that share a name can be told apart off the sheet.
    engrave_assembly: bool = False
    dogbone: bool = True             # overcut sharp inner corners so mating parts seat
    # Pockets-only: cut the pockets (and holes/cut-outs/engraving) but NOT the part's
    # outer profile — for machining features into a pre-cut blank/plate that must stay
    # attached to the sheet. The outline is still used to position the part; it's just
    # never cut. The safety gate's "parts not cut through" rule is relaxed accordingly.
    pockets_only: bool = False
    # 2.5D clearing: area-clear every recess so its waste leaves as chips. Off, a
    # recess is only profiled at its depth and the middle is left in — quicker, but
    # a through pocket then frees a loose slug the cutter can pick up. On either
    # way the part still cuts THROUGH, so unlike pockets_only this does not relax
    # the safety gate's cut-through rule.
    clear_pockets_25d: bool = True
    # 3D features: machine the recesses that have a floor left in them — counterbores,
    # blind pockets. These are the features that need the part the right way up. Off,
    # only what goes through the board is cut and the panel comes out flat.
    machine_3d_features: bool = True
    # Plunge-drilled holes scorch at full router rpm — a router bit's centre has ~zero
    # surface speed, so it rubs and burns. Drill holes ONLY at this slower spindle +
    # feed (everything else keeps the normal speeds/feeds above). 0 rpm = don't change.
    # Internal features — cut-outs, pockets, cleared holes — are fed slower than
    # the outer profile. A concave cut engages more of the cutter and small
    # features flex it, so full profile feed leaves slight errors on internal
    # contours while the outside comes out fine. 0.9 = 10% slower.
    internal_feed_factor: float = 0.9
    # Manual overrides for those categories, mm/min. 0 = automatic (the factor
    # above applied to the profile feed). Set, they take precedence.
    inside_feed: float = 0.0         # internal profiles: cut-outs, slots
    pocket_feed: float = 0.0         # pockets and area-cleared holes
    drill_rpm: int = 12000           # spindle rpm for plunge-drilled holes (0 = same as spindle_rpm)
    drill_feed: float = 600.0        # mm/min plunge feed for holes (0 = same as plunge_feed)
    # Edge break (chamfer) — runs a 45° chamfer cutter along the part edges after the
    # pockets/holes and before the profile cut. size = chamfer width (= depth at 45°).
    edge_break: bool = False         # add an edge-break pass (needs a loaded chamfer/vee tool)
    edge_break_size: float = 2.0     # chamfer width in mm
    edge_break_angle: float = 90.0   # included angle of the vee/chamfer bit (deg); 90 = 45° flank
    edge_break_scope: str = "profile"  # "profile" = outer edge only | "all" = + holes & pockets
    chamfer_rpm: int = 12000         # spindle rpm for the chamfer pass (big bit -> high rim speed scorches)
    chamfer_feed: float = 3000.0     # mm/min feed for the chamfer pass (0 = the tool's normal feed)
    # Stock thickness override, mm. 0 = off: each part is cut at the thickness read
    # from its own solid, one program per thickness, which is right whenever the model
    # matches the board. Set, it says "the board on the bed is THIS thick" and every
    # part is cut from it as one program — for stock that is not the modelled gauge,
    # or a model whose thickness read wrong. It moves the through-cut depth and the
    # gate's stock top together, so the gate still checks against the real board.
    thickness_override: float = 0.0
    # Finishing pass. Off, every contour is cut straight to size as it steps down.
    # On, the depth passes stop `finish_allowance` short of the wall and one final
    # pass takes that skin off, so the wall is cut by a tool that is barely loaded
    # and carries no step marks. Profiles only (outer profile and internal cut-outs)
    # — pocket floors and area clearing are unchanged.
    finish_pass: bool = False
    finish_allowance: float = 0.5    # radial stock left on the wall by the roughing passes (mm)
    # The finishing pass is one full-depth lap by default: it is cutting a thin skin
    # in an already-open channel, so it is barely loaded, and a single lap is what
    # leaves an unstepped wall. Stepped down instead for deep boards or hard stock.
    finish_stepped: bool = False
    # Opening frame: before anything is cut, the tool traces the outline of the
    # whole nest at the clearance height — the same height every rapid uses — so
    # the operator can see the origin is where the stock is (and that the rapid
    # plane clears the clamps along it) before the first plunge.
    frame_nest: bool = True
    # Peck increment for plunge-drilled holes, mm. 4 mm (~0.66xD) clears MDF dust;
    # metal chips pack, so a metal preset pecks shallower.
    drill_peck: float = 4.0
    # Cap every descending feed move so its vertical speed never exceeds the plunge
    # feed. Off for MDF (its programs are unchanged); on for metal, where a steep
    # ramp or a drop after a tab at full cutting feed is a plunge the cutter will
    # not survive for long.
    limit_descent: bool = False

    def mat_dict(self):
        """As the engine's MAT dict."""
        return dict(cut_feed=_feed(self.spindle_rpm * 2 * self.chipload),
                    plunge_feed=_feed(self.spindle_rpm * 2 * self.plunge_chipload),
                    max_step=self.max_step, final_z=self.final_z,
                    clearance_over=self.clearance_over, spindle=self.spindle_rpm,
                    internal_feed_factor=self.internal_feed_factor,
                    inside_feed=self.inside_feed, pocket_feed=self.pocket_feed,
                    drill_rpm=self.drill_rpm or self.spindle_rpm,
                    drill_feed=self.drill_feed,
                    drill_peck=self.drill_peck,
                    limit_descent=self.limit_descent,
                    chamfer_rpm=self.chamfer_rpm or self.spindle_rpm,
                    chamfer_feed=self.chamfer_feed)


@dataclass
class Seat:
    seat_no: int                     # 1..10 -> the machine T-number
    diameter_mm: float = 6.0
    flutes: int = 2
    kind: str = "upcut"              # upcut / downcut / compression / vee / chamfer
    loaded: bool = True
    note: str = ""


# seat kinds never picked as a general cutting tool: vee/chamfer shape edges,
# and a ball nose leaves a round-bottomed kerf — right for 3D surfacing, wrong
# for profiles, pockets and drilling.
_NON_CUTTING = ("vee", "chamfer", "ballnose")


@dataclass
class ToolTable:
    seats: list[Seat] = field(default_factory=list)

    def loaded_seats(self) -> list[Seat]:
        """Cutting tools — loaded seats with a real diameter (profile/cutout/pocket/
        drill). A V-bit's tip diameter is ~0, and a chamfer/V seat shapes edges, not
        stock — both are deliberately NOT cutting tools (see engrave_seats / chamfer_seat)."""
        return [s for s in self.seats if s.loaded and s.diameter_mm > 0 and s.kind not in _NON_CUTTING]

    def engrave_seats(self) -> list[Seat]:
        """Tools eligible to engrave — every loaded seat regardless of diameter, so a
        V-bit entered with Ø0 (its tip) is still available for part marking."""
        return [s for s in self.seats if s.loaded]

    def ball_seat(self):
        """The loaded ball-nose seat (3D surface finishing), or None."""
        return next((s.seat_no for s in self.seats
                     if s.loaded and s.kind == "ballnose"), None)

    def chamfer_seat(self):
        """The seat for the edge-break pass when none is assigned, or None.

        A chamfer-kind seat first, else a loaded V-bit — the UI's Auto option
        reads "chamfer or vee bit", but this only accepted kind == "chamfer",
        so with the usual vee in T05 and the dropdown on Auto the edge break
        silently emitted nothing and parts came off the machine square-edged."""
        for kind in ("chamfer", "vee"):
            seat = next((s.seat_no for s in self.seats
                         if s.loaded and s.kind == kind), None)
            if seat is not None:
                return seat
        return None

    def engine_tools(self, material: Material):
        """Build the engine's TOOLS dict + tool-preference order (widest first).
        Per-tool feeds derive from the material's chip load; DOC capped at 1xD.
        Cutting tools populate the preference order; engrave-only seats (e.g. a Ø0
        V-bit) are registered in TOOLS — so an assigned/auto engrave tool resolves —
        but kept OUT of the cutting preference so they are never picked to cut."""
        tools, pref = {}, []
        for s in sorted(self.loaded_seats(), key=lambda x: -x.diameter_mm):
            tools[s.seat_no] = dict(
                dia=s.diameter_mm,
                feed=_feed(material.spindle_rpm * s.flutes * material.chipload),
                plunge=_feed(material.spindle_rpm * s.flutes * material.plunge_chipload),
                step=min(material.max_step, s.diameter_mm),
            )
            pref.append(s.seat_no)
        for s in self.engrave_seats():
            if s.seat_no not in tools:                       # engrave-only (e.g. Ø0 vee)
                tools[s.seat_no] = dict(
                    dia=max(s.diameter_mm, 0.0),
                    feed=_feed(material.spindle_rpm * s.flutes * material.chipload),
                    plunge=_feed(material.spindle_rpm * s.flutes * material.plunge_chipload),
                    step=material.max_step,                  # non-zero; engrave is single shallow pass
                )
        return tools, pref


DEFAULT_MATERIALS = [
    Material("18mm MDF", 18.0),
    Material("15mm MDF", 15.0),          # same feeds, speeds and DOC as 18mm
    Material("12mm MDF", 12.0, max_step=6.2),
    Material("6mm MDF", 6.0, max_step=3.0),
    # Aluminium — reviewed 2026-10-15 for a first physical run, and set
    # conservatively on purpose: slower is fine, a broken or welded cutter is not.
    # Written for a Ø6 (or Ø4) 2-flute carbide cutter MADE FOR ALUMINIUM (uncoated
    # or ZrN, polished flutes, flute length >= 14 mm for 12 mm plate) — not the
    # MDF bits; see the job notes. One chip load applies to every seat, so it is
    # picked for the Ø4, the more fragile of the two.
    #   spindle 8000 rpm  -> Vc ~150 m/min on Ø6, ~100 on Ø4: well inside carbide-in-
    #                        aluminium practice and cool-running on a router spindle
    #   chipload 0.03     -> feed 8000 x 2 x 0.03 = 480 mm/min (was 0.05 / 800). Above
    #                        the ~0.01 mm where a cutter rubs and welds aluminium,
    #                        below the 0.04-0.05 that suits the Ø6 on a rigid machine
    #   internal x0.9     -> pockets / cut-outs 430 mm/min
    #   max_step 0.5      -> axial DOC 0.08D (Ø6) / 0.13D (Ø4); a full-width slot at
    #                        that depth is ~10 N of cutting force — trivial deflection.
    #                        The real risk in a 12 mm slot is chips packing and being
    #                        re-cut: blow them out (air blast / mist) the whole time
    #   plunge chip 0.01  -> 160 mm/min for any straight plunge (pockets and profiles
    #                        ramp in: profile lead-in 30 mm long, ~1 deg; pockets 8 mm,
    #                        ~3.6 deg at 0.5 mm per level)
    #   drill 8000 / 150, peck 1.0 mm -> an end mill plunging aluminium cannot clear
    #                        chips; shallow pecks with a full retract each time
    #   edge break 8000 rpm / 300 mm/min if ticked (the MDF 12000 / 3000 would be
    #                        ten times too fast in aluminium)
    #   tabs ON, 0.8 mm   -> a part cut free on the last pass can be caught by the
    #                        cutter and thrown; vacuum hold on small aluminium parts
    #                        is weak. 0.8 mm tabs snap/cut off with a file
    #   final_z -0.2      -> just through, barely touching the spoilboard
    #   limit_descent     -> no feed move goes down faster than the 160 mm/min
    #                        plunge: steep spiral-entry ramps (they steepen with
    #                        depth, to ~59 deg at the bottom of 12 mm) and the drop
    #                        after each tab otherwise plunged at 430-480 mm/min
    Material("12mm Aluminium", 12.0, chipload=0.03, plunge_chipload=0.01,
             spindle_rpm=8000, max_step=0.5, final_z=-0.2,
             drill_rpm=8000, drill_feed=150.0, drill_peck=1.0,
             chamfer_rpm=8000, chamfer_feed=300.0,
             tabs_mode="on", tab_remain=0.8, limit_descent=True),
]

# The machine's actual loading. A V-bit is entered at Ø0 because that is its tip
# diameter, and that is also what keeps it out of the cutting-tool preference while
# leaving it available to engrave — see ToolTable.loaded_seats / engrave_seats.
DEFAULT_TOOL_TABLE = ToolTable([
    # The ball nose lives in T01 but is not fitted for everyday work: it ships
    # UNTICKED so a plain job never reaches for it, and the operator ticks it
    # on when a surfacing job actually has it in the seat.
    Seat(1, 6.0, 2, "ballnose", False, "Ø6 ball nose — 3D surfacing"),
    Seat(4, 6.0, 2, "upcut", True, "Ø6 general"),
    Seat(5, 0.0, 2, "vee", True, "V-bit — engraving"),
    Seat(7, 4.0, 2, "upcut", True, "Ø4 — narrow slots & grooves"),
] + [Seat(n, 0.0, 2, "upcut", False, "") for n in (2, 3, 6, 8, 9, 10)])


# ---------------------------------------------------------------- safety clamps
# Every Material field arrives from the browser, and two of them (final_z, max_step)
# set the depth of cut. The safety gate is handed the SAME final_z that generated the
# toolpath, so it agrees with whatever it is told: ask for final_z = -40 and the post
# cuts to -40 and the gate approves it. These bounds are the only thing standing
# between a typo (or a stale form) and a cutter driven through the bed, so they are
# applied server-side, on the way in, and are deliberately not overridable.
#
# They are limits, not preferences: every shipped DEFAULT_MATERIALS value sits well
# inside them, so clamping never changes a normal job.
_LIMITS = {
    # field:            (min,     max)
    "thickness_mm":     (0.1,    100.0),
    "chipload":         (0.001,    1.0),
    "plunge_chipload":  (0.001,    1.0),
    "spindle_rpm":      (1000,   24000),
    "max_step":         (0.05,    10.0),   # depth of cut per pass
    "final_z":          (-6.0,     0.0),   # through-cut floor: at most 6mm into spoilboard
    "clearance_over":   (1.0,     50.0),   # rapid plane must be ABOVE the stock
    "tab_remain":       (0.0,     25.0),
    "tab_width":        (1.0,    100.0),
    "engrave_height":   (1.0,    200.0),
    "internal_feed_factor": (0.2,   1.0),   # never a speed-up; 0.2 floor keeps it a cut, not a rub
    "inside_feed":      (0.0,  20000.0),
    "pocket_feed":      (0.0,  20000.0),
    "drill_rpm":        (0,      24000),
    "drill_feed":       (0.0,   20000.0),
    "drill_peck":       (0.2,      10.0),   # a zero peck would never reach depth
    "edge_break_size":  (0.1,     10.0),
    # tan(angle/2) sets the chamfer depth (size / tan). An angle near zero makes that
    # unbounded, which is an independent depth path the gate only catches by luck.
    "edge_break_angle": (15.0,   179.0),
    "chamfer_rpm":      (0,      24000),
    "chamfer_feed":     (0.0,   20000.0),
    # 0 = off, so the floor is 0 rather than a minimum board. A nonsense small value
    # is caught below instead, where it can be told apart from "off".
    "thickness_override": (0.0,  100.0),
    # A skin thicker than this is not a finishing cut, it is a second roughing pass,
    # and it would let the roughing passes leave more than the finisher can take.
    "finish_allowance": (0.0,     5.0),
}


def clamp_material(m: Material) -> tuple[Material, list[str]]:
    """Force every numeric field inside its safe band. Returns the material and a
    note for each value that had to be moved, so the operator is told rather than
    silently overruled."""
    notes = []
    for fld, (lo, hi) in _LIMITS.items():
        try:
            v = getattr(m, fld)
            if v is None:
                continue
            v = float(v)
        except (TypeError, ValueError):
            setattr(m, fld, Material.__dataclass_fields__[fld].default)
            notes.append(f"{fld}: not a number, reset to default")
            continue
        if v != v:                                   # NaN compares false with everything
            setattr(m, fld, Material.__dataclass_fields__[fld].default)
            notes.append(f"{fld}: not a number, reset to default")
            continue
        c = min(max(v, lo), hi)
        if c != v:
            setattr(m, fld, int(c) if isinstance(getattr(m, fld), int) else c)
            notes.append(f"{fld}: {v:g} clamped to {c:g}")
    # A pass deeper than the board itself is one full-depth plunge in disguise.
    if m.max_step > m.thickness_mm:
        notes.append(f"max_step: {m.max_step:g} clamped to the {m.thickness_mm:g}mm board")
        m.max_step = m.thickness_mm
    # A tab taller than the board never severs the part; the gate would fail the job
    # as "not cut through" after the fact, which is a slower way to learn it.
    if m.tab_remain >= m.thickness_mm:
        notes.append(f"tab_remain: {m.tab_remain:g} clamped below the {m.thickness_mm:g}mm board")
        m.tab_remain = max(0.0, m.thickness_mm - 0.5)
    # 0 means "off"; a fraction of a millimetre means a mistyped board, and it would
    # otherwise be taken literally and cut every part out of nothing.
    if 0.0 < m.thickness_override < 1.0:
        notes.append(f"thickness_override: {m.thickness_override:g} is not a board — "
                     f"raised to 1mm")
        m.thickness_override = 1.0
    # Asking to leave more skin than the cutter's own depth of cut per pass means the
    # finishing lap is the heavier of the two, which is backwards.
    if m.finish_pass and m.finish_allowance > m.max_step:
        notes.append(f"finish_allowance: {m.finish_allowance:g} clamped to the "
                     f"{m.max_step:g}mm depth of cut")
        m.finish_allowance = m.max_step
    return m, notes


def material_from_dict(d: dict) -> Material:
    m = Material(**{k: d[k] for k in d if k in Material.__dataclass_fields__})
    return clamp_material(m)[0]

def tooltable_from_list(rows: list[dict]) -> ToolTable:
    """Seats as sent by the browser, with the numbers made safe. A negative or absurd
    diameter would otherwise flow straight into the polygon offsetting and the feed
    maths — an inside-out toolpath is not something the gate is equipped to notice."""
    seats = []
    for r in rows:
        s = Seat(**{k: r[k] for k in r if k in Seat.__dataclass_fields__})
        try:
            s.diameter_mm = min(max(float(s.diameter_mm or 0.0), 0.0), 50.0)
        except (TypeError, ValueError):
            s.diameter_mm = 0.0
        try:
            s.flutes = min(max(int(s.flutes or 1), 1), 8)
        except (TypeError, ValueError):
            s.flutes = 2
        seats.append(s)
    return ToolTable(seats)

def material_to_dict(m: Material) -> dict:
    return asdict(m)
