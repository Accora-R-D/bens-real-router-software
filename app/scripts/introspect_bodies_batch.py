"""is_hardware_name: tells bought-in hardware ("Hex socket head cap screw
M8x1.25 x 35", "100mm Castor") from parts that are made, by name.

Extracted, unchanged, from the fab exporter's scripts/introspect_bodies_batch.py
(Accora-R-D/real-accora-fab-exporter), which also talks to the Onshape API.
The STEP reader (backend/services/step_ingest_service.py) imports it by this
module name, so the router keeps the exporter's hardware detection without
carrying any Onshape code. Keep it in step with the exporter's copy.
"""
from __future__ import annotations

import re


# Last-word-is-hardware test.  "Custom Washer" matches; "Spring Mounting Tab"
# does not, because the last token is "Tab".  M-thread prefix like "M3" / "M4"
# at the start of the name is also accepted as a hardware signal.
_HARDWARE_NOUNS = {
    "nut", "locknut", "lock-nut", "bolt", "screw", "rivet", "washer",
    "spring", "bearing", "bushing", "bush", "circlip", "stud",
    "insert", "standoff", "grub",
    # Note: `pin`, `spacer`, `dowel`, `shim` are NOT in this set — they
    # are commonly the names of CUSTOM machined / formed parts (e.g.
    # "Bracket Pin", "Nylon Spacer").  They only count as bought-in
    # via the dimensioned-bought-in regex (e.g. "6mm Steel Spacer",
    # "M6 Dowel Pin").
    # Bought-in extras observed in Accora bedroom-furniture BOMs.
    # `cap` / `castor` / etc. only fire AFTER a size token like
    # "20x20mm" — covered by _DIMENSIONED_BOUGHT_IN_RE.
    "knob", "clip", "castor", "caster", "wheel", "handle", "plug",
    "gasket", "seal", "o-ring", "oring", "sleeve",
}


# Positional / variant suffixes that carry no part-class meaning.  Strip from
# the tail of a name before checking the last token.  "Extension Spring-Mirrored"
# becomes "Extension Spring" and resolves to hardware.  "Bracket-LH" loses LH
# but still ends in "Bracket", which isn't hardware, so won't match.
_MODIFIER_TOKENS = {
    "mirrored", "mirror", "reflected",
    "rh", "lh", "left", "right",
    "front", "rear", "back", "top", "bottom", "upper", "lower",
    "inner", "outer", "external", "internal",
    "copy", "ref", "alt",
    # Country / region suffixes (e.g. "PRESTO/CB2-CABLE CLIP-UK")
    "uk", "us", "eu", "ca", "au", "nz", "fr", "de", "jp",
    # Standards designators. A washer is named by its form, not its shape:
    # "Steel Washer Form A" used to leave "a" as the last token and so read as
    # a fabricated part.
    "form", "grade", "type", "class", "din", "iso", "bs",
}


# Single letters are grade/form designators in a fastener name ("Form A",
# "grade B"), never the part class.
# The nouns a fabricated part takes when it is made to carry a bought-in
# unit. A name ending in one of these is the bracket, not the motor.
_FABRICATED_FOR_NOUNS = {"bracket", "mount", "mounting", "plate", "tab", "arm",
                         "clip", "cover", "guard", "support", "base", "lug",
                         "boss", "spacer", "adapter", "adaptor", "link", "lever"}


# Digits and capitals only, at least 8 long with both kinds present, and
# optionally one hyphenated suffix. Pure model-number territory — EXCEPT
# Accora's own drawing numbers, which look just like it: "2NBF0156-01234",
# "2NBF0758-0042_00". A short letter block followed by 3–4 digits and a
# hyphen is a drawing series, not a vendor, and stays with geometry (see the
# P167 note on _VENDOR_CODE_RES below).
_CATALOGUE_CODE_RE = re.compile(
    r"(?!\d?[A-Z]{1,4}\d{3,4}(?:-|_|$))"
    r"(?=.*\d)(?=.*[A-Z])[A-Z0-9]{8,}(?:-[A-Z0-9]+)?", re.IGNORECASE)


_GRADE_LETTER_RE = re.compile(r"^[a-z]$")


_VERSION_TOKEN_RE = re.compile(r"^v\d+$|^\d+(?:\.\d+)?$|^#\d+$")


_M_THREAD_RE = re.compile(r"^\s*M\d{1,2}\b", re.IGNORECASE)


# Match an M-thread spec ANYWHERE in the name — many Accora fastener
# names embed it mid-string ("Hex socket head cap screw M8x1.25 x 35").
_M_THREAD_ANYWHERE_RE = re.compile(r"\bM\d{1,2}(?:x\d+(?:\.\d+)?)?\b", re.IGNORECASE)


# Self-tapping screw thread spec: ST4.2 x 9.5, ST3.5x12, etc.
_ST_THREAD_RE = re.compile(r"\bST\s*\d+(?:\.\d+)?\s*[x×]\s*\d+(?:\.\d+)?\b", re.IGNORECASE)


# Fastener-class word embedded anywhere in the name — when combined
# with an M-thread or ST-thread spec it's almost certainly a screw/nut/bolt.
_FASTENER_WORD_RE = re.compile(
    r"\b(screw|nut|bolt|rivet|washer|stud|grub|dowel|self.tapping|"
    r"countersunk|cap.head|button.head|hex.head|socket.head|machine.screw)\b",
    re.IGNORECASE
)


# Compound-pin fasteners — "Clevis Pin", "Split Pin", "Roll Pin",
# "Cotter Pin", "Dowel Pin", "Spring Pin", "Locating Pin".  Bare "Pin"
# alone is too ambiguous (could be a custom Accora rod), but any of
# these compound forms is clearly a standard imported fastener.
_COMPOUND_PIN_RE = re.compile(
    r"\b(clevis|split|roll|cotter|dowel|spring|spirol|locating)\s+pin\b",
    re.IGNORECASE
)


# A drive unit — motor, actuator, gas strut — is bought whole, and so is
# anything named as one of its pieces ("Main Lift Motor body", "... rod",
# "KA30_3D_STANDARD_MOTOR_AND_STAND"). What is NOT bought is the part made
# to hold it: a "Motor Mount Bracket" ends in a fabrication noun.
_DRIVE_UNIT_RE = re.compile(
    # lookarounds, not \b: an underscore is a word character, so \b never
    # fires inside KA30_3D_STANDARD_MOTOR_AND_STAND
    r"(?<![A-Za-z])(motor|actuator|gearbox|gas\s*spring|gas\s*strut)(?![A-Za-z])",
    re.IGNORECASE)


# "20x20mm Cap", "30x20mm Grey Cap", "100mm Castor" etc — a dimension
# prefix turns ambiguous nouns like "Cap" into a clear bought-in signal.
_DIMENSIONED_BOUGHT_IN_RE = re.compile(
    r"\b\d+(?:x\d+)?\s*mm\b.*\b(cap|plug|knob|wheel|caster|castor|spacer|dowel|grommet|bumper)\b",
    re.IGNORECASE
)


def is_hardware_name(name: str, desc: str = "") -> tuple[bool, str]:
    """Return (is_hardware, matched_token).

    Detection in order of confidence:
      1. M-thread spec anywhere ("M8x1.25") + a fastener-class word
         ("screw", "nut", "bolt", "self-tapping", ...) → very confident
         fastener.  Catches "Hex socket head cap screw M8x1.25 x 35"
         and "Hex thin nut grade A & B M12".
      2. ST-thread (self-tapping) spec → fastener.
      3. Strict M-thread prefix at start → fastener.
      4. Dimensioned bought-in pattern: "20x20mm Cap", "100mm Castor",
         "30x20mm Grey Cap" — ambiguous nouns become clear with a
         dimension prefix.
      5. Last meaningful token is a hardware noun (legacy strict path).
      6. Hardware noun anywhere in the name (looser fallback for
         multi-word names like "Spring Pin Latch Assembly").
    """
    for text in (name or "", desc or ""):
        text = text.strip()
        if not text:
            continue
        # 1. M-thread + fastener word combo.
        m_thread = _M_THREAD_ANYWHERE_RE.search(text)
        if m_thread and _FASTENER_WORD_RE.search(text):
            return (True, m_thread.group(0).strip())
        # 2. Self-tapping screw spec.
        st_match = _ST_THREAD_RE.search(text)
        if st_match and _FASTENER_WORD_RE.search(text):
            return (True, st_match.group(0).strip())
        # 3. M-thread prefix at start (legacy).
        m = _M_THREAD_RE.match(text)
        if m:
            return (True, m.group(0).strip())
        # 4. Dimensioned bought-in pattern.
        dim_match = _DIMENSIONED_BOUGHT_IN_RE.search(text)
        if dim_match:
            return (True, dim_match.group(0).strip())
        # 4b. Compound-pin fastener (Clevis Pin, Split Pin, etc.).
        pin_match = _COMPOUND_PIN_RE.search(text)
        if pin_match:
            return (True, pin_match.group(0).strip())
        # 4c. A drive unit, or one of its pieces. "Main Lift Motor body" shipped
        # as RHS 6x2.5 with a 6 mm wall and its rod as CHS OD28 — a linear
        # actuator, sized as steel stock. The word may sit anywhere in the name
        # — but a name that ENDS in a fabrication noun ("Motor Mount Bracket")
        # is the part made to hold the unit, and stays fabricated.
        drive = _DRIVE_UNIT_RE.search(text)
        if drive:
            words = [w for w in re.split(r"[\s\-_/()]+", text.lower()) if w]
            while words and (words[-1] in _MODIFIER_TOKENS
                             or _VERSION_TOKEN_RE.fullmatch(words[-1])):
                words.pop()
            if not (words and words[-1] in _FABRICATED_FOR_NOUNS):
                return (True, drive.group(1))
        # 4d. A bare catalogue code: a run of letters and digits with no
        # vocabulary in it at all ("5946UAP100R36-32S30") names a vendor's
        # model, and every sub-body of a castor or a motor imported under it
        # is bought with it. Before this each of those bodies was measured as
        # if it were stock — "CHS 7x7", "RHS 5.4x1.2" — and shipped for making.
        if _CATALOGUE_CODE_RE.fullmatch(text.split("(")[0].strip()):
            return (True, text.split("(")[0].strip())
        # 5. Last-token hardware noun (after stripping modifiers / version).
        tokens = re.split(r"[\s\-_/]+", text.lower())
        tokens = [t for t in tokens if t]
        while tokens and (
            tokens[-1] in _MODIFIER_TOKENS
            or _VERSION_TOKEN_RE.fullmatch(tokens[-1])
            or _GRADE_LETTER_RE.fullmatch(tokens[-1])
            or tokens[-1] == "x"   # "M8x1.25 x 12" leaves trailing 'x'
        ):
            tokens.pop()
        if tokens and tokens[-1] in _HARDWARE_NOUNS:
            return (True, tokens[-1])
        # 6. Hardware noun anywhere — only fires for unambiguous nouns
        # that don't double as descriptors of larger assemblies.
        # These name a fastener class outright, wherever they appear: no
        # fabricated part is called a washer or a locknut.  Deliberately does
        # NOT include pin / spacer / bush / bearing, which are routinely the
        # names of turned parts made in-house.
        unambiguous = {"screw", "bolt", "nut", "locknut", "rivet", "washer",
                       "circlip", "grub", "stud", "dowel", "grommet",
                       "cotter", "setscrew", "nyloc"}
        for tok in tokens:
            if tok in unambiguous:
                return (True, tok)
    return (False, "")
