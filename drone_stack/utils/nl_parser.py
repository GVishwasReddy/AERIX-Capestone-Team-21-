"""Natural-language command parser.

Turns free text like *"go up vertically for 5m and stay"* or
*"take off to 5m then move forward 10 meters and turn right 90 degrees"* into a
list of structured :class:`Intent` objects that the navigation node executes.

It is fully rule-based and offline - no external API, no network, deterministic
and unit-tested - so it can never stall or crash the flight loop. Anything it
cannot understand becomes an ``unknown`` intent (reported back to the user)
rather than a silent failure.

Grammar coverage (what makes it robust to real phrasing):
  * spelled-out numbers - "climb five meters", "turn ninety degrees",
    "go up a hundred feet", "move a couple of metres", "descend half a meter"
  * politeness / filler - "could you please take off to about 5m", "just hover"
  * vague magnitudes - "go up a bit", "move forward a lot", "turn slightly left"
  * negation - "don't land", "do not disarm", "no need to take off" (the clause
    is dropped rather than executed)
  * connectives - commas, "then", "and then", "after that", "next",
    "followed by", "->"
  * fuzzy typo tolerance - "tkae off", "lnad", "swithc to loyter", "disrm"

Intent actions and params:
    takeoff        {altitude}
    land           {}
    rtl            {}
    arm / disarm   {}
    hold           {}
    resume         {}
    start_mission  {}
    emergency      {}
    move           {dx, dy, dz}         # metres, body frame (fwd, left, up)
    set_altitude   {altitude}           # absolute, metres above home
    set_speed      {speed}              # m/s
    yaw            {angle_deg, direction}  # direction: +1 = right/CW, -1 = left/CCW
    set_mode       {mode}               # ArduPilot flight-mode name
    unknown        {text}
"""
from __future__ import annotations

import difflib
import re
from dataclasses import dataclass, field

# Every keyword the parser reacts to. Free-text tokens are fuzzy-matched against
# this vocabulary so typos ("tkae off", "lnad", "swithc to loyter", "disrm")
# still map to the right command. Fully offline + deterministic (difflib).
_VOCAB: tuple[str, ...] = tuple(sorted({
    # actions
    "takeoff", "take", "off", "launch", "lift", "land", "touchdown", "touch",
    "return", "home", "come", "back", "resume", "continue", "proceed", "carry",
    "start", "begin", "mission", "arm", "disarm", "emergency", "abort",
    "mayday", "kill", "motor", "motors", "hold", "stop", "halt", "stay", "hover",
    "wait", "freeze", "brake", "pause",  # note: "run" deliberately excluded so
    # "trun" fuzzy-corrects to "turn" rather than "run" (mission-run uses a
    # literal substring check, unaffected by the fuzzy vocabulary).
    # directions
    "forward", "ahead", "front", "straight", "backward", "backwards", "reverse",
    "behind", "left", "port", "right", "starboard", "up", "ascend", "climb",
    "rise", "higher", "raise", "down", "descend", "lower", "drop", "sink",
    "decrease",
    # rotation / speed / measure
    "turn", "rotate", "yaw", "spin", "clockwise", "counterclockwise",
    "anticlockwise", "around", "speed", "faster", "slower", "altitude",
    "meters", "metres", "meter", "metre", "degrees", "degree",
    # flight modes + switching
    "stabilize", "stabilise", "loiter", "guided", "auto", "acro", "circle",
    "orbit", "poshold", "althold", "drift", "sport", "flip", "follow", "throw",
    "zigzag", "position", "smart", "mode", "switch", "change", "enter",
    "engage", "activate",
    # connectors
    "then", "and",
}))

# Short/ambiguous tokens we never auto-correct (they are valid as-is). Includes
# the vague-magnitude and number-scale words so difflib never mangles them.
_KEEP = {"to", "at", "up", "by", "go", "of", "in", "on", "the", "a", "an",
         "m", "cm", "mm", "ft", "km", "rtl", "cw", "ccw", "deg", "now",
         "around", "bit", "tad", "lot", "way", "far", "big", "hair", "tiny",
         "half", "few", "couple", "dozen", "several", "quarter", "hundred",
         "thousand",
         # connective words - protected so fuzzy-correct can't mangle them
         # (e.g. "after" -> "faster") and break clause splitting.
         "after", "that", "next", "followed"}


# --- spelled-out numbers ---------------------------------------------------
# "climb five meters" / "turn ninety degrees" / "go up a hundred feet" all need
# the words turned into digits *before* the numeric regex runs. Done here so the
# rest of the parser keeps working on ordinary numbers.
_WORD_UNITS: dict[str, float] = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
    "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
    "seventeen": 17, "eighteen": 18, "nineteen": 19, "twenty": 20, "thirty": 30,
    "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80,
    "ninety": 90,
}
_WORD_SCALES: dict[str, float] = {"hundred": 100, "thousand": 1000}
# Loose quantity words people actually say.
_WORD_SMALL: dict[str, float] = {
    "couple": 2, "few": 3, "several": 4, "dozen": 12, "half": 0.5,
    "quarter": 0.25,
}


def _fmt_num(val: float) -> str:
    return str(int(val)) if val == int(val) else str(val)


def _combine_run(run: list[tuple[str, float]]) -> float:
    """Fold a run of number tokens ("one hundred and twenty") into a value."""
    total = 0.0
    current = 0.0
    for kind, v in run:
        if kind == "scale":
            if v == 100:
                current = (current or 1) * 100
            else:  # thousand
                total += (current or 1) * 1000
                current = 0.0
        else:
            current += v
    return total + current


def _words_to_numbers(text: str) -> str:
    """Replace spelled-out number phrases with their digit equivalents."""
    toks = text.split()
    out: list[str] = []
    i, n = 0, len(toks)
    while i < n:
        t = toks[i]
        starts = (
            t in _WORD_UNITS or t in _WORD_SMALL or t in _WORD_SCALES
            or (t in ("a", "an") and i + 1 < n
                and (toks[i + 1] in _WORD_SCALES or toks[i + 1] in _WORD_SMALL))
        )
        if not starts:
            out.append(t)
            i += 1
            continue
        run: list[tuple[str, float]] = []
        while i < n:
            t = toks[i]
            if t in ("a", "an"):
                nxt = toks[i + 1] if i + 1 < n else ""
                if nxt in _WORD_SCALES:      # "a hundred" -> 1 x 100
                    run.append(("unit", 1))
                    i += 1
                    continue
                if nxt in _WORD_SMALL:       # "a couple" -> just the article
                    i += 1
                    continue
                break
            if t == "and":  # only the "and" *inside* a number ("hundred and ten")
                nxt = toks[i + 1] if i + 1 < n else ""
                if run and (nxt in _WORD_UNITS or nxt in _WORD_SCALES
                            or nxt in _WORD_SMALL):
                    i += 1
                    continue
                break
            if t in _WORD_UNITS:
                run.append(("unit", _WORD_UNITS[t]))
            elif t in _WORD_SCALES:
                run.append(("scale", _WORD_SCALES[t]))
            elif t in _WORD_SMALL:
                run.append(("unit", _WORD_SMALL[t]))
            else:
                break
            i += 1
        out.append(_fmt_num(_combine_run(run)) if run else t)
    return " ".join(out)


# --- filler / politeness ---------------------------------------------------
# Dropped up front so "could you please take off to about 5m" reads the same as
# "take off to 5m". Never includes direction/number/unit words.
_FILLER = {"please", "kindly", "could", "would", "can", "you", "just",
           "about", "approximately", "roughly", "approx", "like", "maybe",
           "somewhere", "some"}


def _strip_fillers(text: str) -> str:
    return " ".join(t for t in text.split() if t not in _FILLER)


# --- vague magnitudes ------------------------------------------------------
def _vague(clause: str) -> str | None:
    """Return 'small' / 'large' when a clause gives a fuzzy amount, else None."""
    if _has(clause, "slightly", "little", "bit", "tad", "touch", "nudge",
            "hair", "tiny", "small", "gently"):
        return "small"
    if _has(clause, "lot", "far", "way", "much", "significantly", "large",
            "big", "loads"):
        return "large"
    return None


def _vague_dist(clause: str, default: float, small: float, large: float) -> float:
    v = _vague(clause)
    return small if v == "small" else large if v == "large" else default


# --- negation --------------------------------------------------------------
_NEG = re.compile(r"\b(don't|dont|do not|never|no need|not|cancel|nevermind|"
                  r"never mind)\b")


def _is_negated(clause: str) -> bool:
    return _NEG.search(clause) is not None


_NUMBER = re.compile(r"-?\d+(?:\.\d+)?")
# Clause separators: commas/semicolons plus the common English connectives.
_SPLIT = re.compile(
    r"\s*(?:,|;|->|\band then\b|\bthen\b|\band\b|\bafter that\b|\bafterwards\b|"
    r"\bnext\b|\bfollowed by\b|&)\s*",
    re.IGNORECASE,
)

# ArduPilot copter flight modes reachable by name from plain English. Longest
# keys are matched first so "smart rtl" wins over "rtl", "alt hold" over "hold".
_MODES: dict[str, str] = {
    "smart rtl": "SMART_RTL", "smartrtl": "SMART_RTL",
    "alt hold": "ALT_HOLD", "althold": "ALT_HOLD", "altitude hold": "ALT_HOLD",
    "pos hold": "POSHOLD", "poshold": "POSHOLD", "position hold": "POSHOLD",
    "guided nogps": "GUIDED_NOGPS",
    "stabilize": "STABILIZE", "stabilise": "STABILIZE",
    "loiter": "LOITER", "guided": "GUIDED", "auto": "AUTO", "acro": "ACRO",
    "circle": "CIRCLE", "orbit": "CIRCLE", "drift": "DRIFT", "sport": "SPORT",
    "flip": "FLIP", "follow": "FOLLOW", "throw": "THROW", "zigzag": "ZIGZAG",
    "poshold mode": "POSHOLD",
}
_MODE_SWITCH = ("mode", "switch", "change", "enter", "engage", "activate", "go to")


def _match_mode(clause: str) -> str | None:
    """Return an ArduPilot mode name if *clause* asks to switch into one.

    Fires when the clause names a mode together with a switch cue ("mode",
    "switch to X", "enter X"), or when the clause is essentially just the mode
    name (e.g. the user types "loiter" or "guided").
    """
    stripped = clause.strip()
    has_cue = any(c in clause for c in _MODE_SWITCH)
    for key in sorted(_MODES, key=len, reverse=True):
        if re.search(rf"\b{re.escape(key)}\b", clause):
            if has_cue or stripped in (key, f"{key} mode"):
                return _MODES[key]
    return None


@dataclass
class Intent:
    action: str
    params: dict = field(default_factory=dict)
    source: str = ""

    def describe(self) -> str:
        if self.params:
            inner = ", ".join(f"{k}={v}" for k, v in self.params.items())
            return f"{self.action}({inner})"
        return self.action


def _autocorrect(text: str) -> str:
    """Map each misspelled word to its closest known keyword (typo tolerance)."""
    out: list[str] = []
    for tok in text.split():
        if (len(tok) < 3 or tok in _KEEP or tok in _VOCAB
                or any(ch.isdigit() for ch in tok)):
            out.append(tok)
            continue
        match = difflib.get_close_matches(tok, _VOCAB, n=1, cutoff=0.72)
        out.append(match[0] if match else tok)
    return " ".join(out)


def _to_meters(value: float, unit: str | None) -> float:
    unit = (unit or "").lower()
    if unit in ("cm", "centimeter", "centimetre", "centimeters", "centimetres"):
        return value / 100.0
    if unit in ("mm", "millimeter", "millimetre"):
        return value / 1000.0
    if unit in ("ft", "feet", "foot"):
        return value * 0.3048
    if unit in ("km", "kilometer", "kilometre"):
        return value * 1000.0
    return value  # metres / unspecified


def _first_measure(text: str) -> tuple[float | None, str | None]:
    """Return (value, unit) of the first number in *text*, if any."""
    match = _NUMBER.search(text)
    if not match:
        return None, None
    value = float(match.group())
    tail = text[match.end():].lstrip()
    unit_match = re.match(r"[a-zA-Z°]+", tail)
    unit = unit_match.group() if unit_match else None
    return value, unit


def _has(text: str, *words: str) -> bool:
    return any(re.search(rf"\b{re.escape(w)}\b", text) for w in words)


def parse(text: str) -> list[Intent]:
    """Parse *text* into a list of intents (may be empty for blank input)."""
    if not text or not text.strip():
        return []
    normalized = _words_to_numbers(_strip_fillers(text.strip().lower()))
    corrected = _autocorrect(normalized)
    clauses = [c.strip() for c in _SPLIT.split(corrected) if c.strip()]
    intents: list[Intent] = []
    for clause in clauses:
        # A negated clause ("don't land", "do not disarm") is dropped rather
        # than executed - never do the opposite of what the pilot asked.
        if _is_negated(clause):
            continue
        intent = _parse_clause(clause)
        if intent is not None:
            intents.append(intent)
    return _postprocess(intents)


def _parse_clause(clause: str) -> Intent | None:  # noqa: C901 - a keyword dispatch
    value, unit = _first_measure(clause)
    degrees = "deg" in clause or "°" in clause

    # --- safety / high priority --------------------------------------------
    if _has(clause, "emergency", "abort", "mayday", "kill"):
        if _has(clause, "kill") and _has(clause, "motor", "motors"):
            return Intent("disarm", source=clause)
        return Intent("emergency", source=clause)
    if _has(clause, "disarm"):
        return Intent("disarm", source=clause)

    # --- discrete flight actions -------------------------------------------
    if _has(clause, "takeoff", "launch") or ("take" in clause and "off" in clause) \
            or ("lift" in clause and "off" in clause):
        alt = value if value is not None else 5.0
        return Intent("takeoff", {"altitude": _to_meters(alt, unit)}, clause)
    if _has(clause, "land", "touchdown") or ("touch" in clause and "down" in clause):
        return Intent("land", source=clause)
    if _has(clause, "rtl", "home") or ("return" in clause) or ("come" in clause and "back" in clause):
        return Intent("rtl", source=clause)
    if _has(clause, "resume", "continue", "proceed", "carry"):
        return Intent("resume", source=clause)
    if ("start" in clause or "begin" in clause or "run" in clause) and "mission" in clause:
        return Intent("start_mission", source=clause)
    if _has(clause, "arm"):
        return Intent("arm", source=clause)

    # --- flight-mode switch (loiter / guided / stabilize / circle / ...) ----
    mode = _match_mode(clause)
    if mode is not None:
        return Intent("set_mode", {"mode": mode}, clause)

    # --- rotation ----------------------------------------------------------
    if _has(clause, "turn", "rotate", "yaw", "spin"):
        if value is not None:
            angle = value
        elif _has(clause, "around"):      # "turn around" -> half turn
            angle = 180.0
        else:
            angle = _vague_dist(clause, default=90.0, small=30.0, large=180.0)
        direction = -1 if _has(clause, "left", "ccw", "counterclockwise", "anticlockwise") else 1
        return Intent("yaw", {"angle_deg": angle, "direction": direction}, clause)

    # --- speed -------------------------------------------------------------
    if "speed" in clause or "faster" in clause or "slower" in clause:
        if value is not None:
            return Intent("set_speed", {"speed": value}, clause)

    # --- vertical ----------------------------------------------------------
    up = _has(clause, "up", "ascend", "climb", "rise", "higher", "raise")
    down = _has(clause, "down", "descend", "lower", "drop", "sink", "decrease")
    absolute = _has(clause, "to", "at") or "altitude" in clause
    if up or down:
        if value is not None:
            dist = _to_meters(value, unit)
        else:
            dist = _vague_dist(clause, default=2.0, small=1.0, large=8.0)
        if absolute and value is not None:
            return Intent("set_altitude", {"altitude": dist}, clause)
        signed = dist if up else -dist
        return Intent("move", {"dx": 0.0, "dy": 0.0, "dz": signed}, clause)
    if "altitude" in clause and value is not None:
        return Intent("set_altitude", {"altitude": _to_meters(value, unit)}, clause)

    # --- horizontal translation --------------------------------------------
    forward = _has(clause, "forward", "ahead", "front", "straight")
    backward = _has(clause, "back", "backward", "backwards", "reverse", "behind")
    left = _has(clause, "left", "port")
    right = _has(clause, "right", "starboard")
    if forward or backward or left or right:
        if value is not None:
            dist = _to_meters(value, unit)
        else:
            dist = _vague_dist(clause, default=1.0, small=0.5, large=8.0)
        dx = dist if forward else (-dist if backward else 0.0)
        dy = dist if left else (-dist if right else 0.0)
        return Intent("move", {"dx": dx, "dy": dy, "dz": 0.0}, clause)

    # --- hold / stop (checked late so "stay" after a move is a no-op) -------
    if _has(clause, "hold", "stop", "halt", "stay", "hover", "wait", "freeze", "brake", "pause"):
        return Intent("hold", source=clause)

    return Intent("unknown", {"text": clause}, clause)


def _postprocess(intents: list[Intent]) -> list[Intent]:
    """Drop a trailing 'hold' when it merely confirms a preceding motion.

    e.g. "go up 5m and stay" -> the drone already holds at the target, so the
    'stay' should not brake and cancel the climb.
    """
    motion = {"move", "takeoff", "set_altitude", "yaw", "set_speed"}
    if len(intents) >= 2 and any(i.action in motion for i in intents):
        intents = [
            i for idx, i in enumerate(intents)
            if not (i.action == "hold" and idx == len(intents) - 1)
        ]
    return intents
