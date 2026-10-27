# Syntec Post Spec — captured from proven Vectric output

Reverse-engineered from 8 known-good `.NC` files that already run on the machine
(Sams Plastic Trunk 4/5/6mm; DXF (Wood) 18/20mm; Router 18mm Swing Arm/wood;
CM6 side arm 15mm). This is the contract the generator must reproduce.

## Header (byte-identical in every file)
```
N0G40G17G80G49        ; cancel comp / XY plane / cancel canned cycle / cancel tool-length
N10G00 G53 Z0.0       ; rapid, machine coords, retract Z to machine home
N20T7M06              ; tool 7, tool change
N30G90G54             ; absolute, work offset G54 (origin = sheet corner)
N40G90G71             ; absolute, G71 = METRIC units
N50G43 H7             ; tool-length comp on, height offset H7
N60S18000M3           ; spindle 18000 rpm CW
```

## Footer (every file)
```
N..G00Z<clearance>    ; retract to clearance plane
N..G28G91Z0           ; return Z to home (incremental)
N..M5                 ; spindle off
N..M30                ; program end
```

## Line numbering
`N0`, `N10`, `N20`, … increment by 10. No decimals on N.

## Geometry / motion
- `G00 X.. Y..`  rapid to profile start (XY)
- `G00 Z<clear>` rapid down to clearance plane
- `G01 Z<depth> F350.0`  plunge at **F350** to pass depth
- contour at cut feed: **F3672 (wood)** / **F3500 (plastic)**
- `G01` lines, `G02` CW arc, `G03` CCW arc
- arcs: **I/J are incremental** (centre relative to arc start); X/Y are absolute
- **Tool-CENTRE coordinates** — no G41/G42 in body (G40 active). The CAM
  pre-offsets the contour by the tool radius (outside for cut-outs). Our
  generator must do the radius offset itself.
- **2 mm radius lead-in / lead-out arc** tangent onto each contour.
- coordinates to 3 decimals, mm.

## Depth strategy (multi-pass, ramp straight down per pass)
- clearance plane Z = **stock_top + 5 mm**  (6→11, 5→10, 4→9, 15→20, 18→23, 20→25)
- final pass Z = **−0.2 mm (wood) / −0.3 mm (plastic)**  (cuts just through into spoilboard)
- stepdown per pass:
  - wood  ≈ **2.02 mm**  (18 mm → 9 passes: 15.978,13.956,…,−0.2)
  - plastic ≈ **1.575 mm** (6 mm → 4 passes: 4.425,2.85,1.275,−0.3)
- each profile is fully machined (all depth passes) **before** retract + move to next.

## Tooling / fixturing
- single tool **T7 / H7**, **S18000**, throughout (no tool changes observed)
- **NO TABS** — parts cut fully free on the final pass ⇒ assumes **vacuum bed**
  (or the operator accepts free parts). Pipeline default should match unless told otherwise.
- **NO drill canned cycles** (no G81/G83) — holes are cut as circular profiles
  with the same router bit, not drilled.

## Nesting / origin
- multi-part files place several profiles in one program, origin at the
  **sheet corner (G54)**; e.g. `18mm Swing Arm.NC` = 3 nested profiles up to
  X≈748 Y≈533. Confirms: nest → one program → sheet-corner origin.

## UNKNOWNS to confirm before generating
1. **Tool diameter** (router bit Ø) — needed for the radius offset. Not derivable
   from the header; infer from a measured part or ask. (Lead-in arc is 2 mm and is
   independent of tool radius.)
2. **Tabs / vacuum** — confirm the no-tab convention is intentional (vacuum bed).
3. **Cut feed by material** — F3672 wood / F3500 plastic confirmed; extend table
   as new materials appear.
4. **File extension / naming** the Syntec expects on the USB stick (`.NC` uppercase observed).
