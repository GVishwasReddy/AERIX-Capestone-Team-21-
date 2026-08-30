"""Regression tests for the 2026-08-25 stuck-in-TAKEOFF flight.

The aircraft climbed to 3 m and hovered over the launch point until a human
brought it down. It never navigated, because ``NavigationNode._do_takeoff``
reads ``_fused.alt_rel_m`` and that value was 0.0 half the time.

Three MAVLink messages map onto one ``Altitude``, and ``FusionNode`` replaces
its altitude slot outright instead of merging. VFR_HUD carries climb rate and
AMSL but no relative altitude, so the ``Altitude`` built from it published
``relative_m=0.0`` - the dataclass default - at 10 Hz, interleaved with the real
value from GLOBAL_POSITION_INT at the same rate. The takeoff gate resets its
settle deadline on any sample below the threshold, so it could never open.
"""
from __future__ import annotations

from types import SimpleNamespace

from drone_stack.interfaces.mavlink_interface import RealMavlink
from drone_stack.msg import Altitude


def _iface():
    return RealMavlink({"connection": "udpin:127.0.0.1:1", "baud": 57600})


def _gpi(relative_m: float, amsl_m: float = 909.0):
    msg = SimpleNamespace(
        relative_alt=int(relative_m * 1000), alt=int(amsl_m * 1000),
        vx=0, vy=0, vz=0, hdg=0,
    )
    msg.get_type = lambda: "GLOBAL_POSITION_INT"
    return msg


def _vfr(amsl_m: float = 909.0, climb: float = 0.2):
    msg = SimpleNamespace(alt=amsl_m, climb=climb, groundspeed=0.0,
                          airspeed=0.0, heading=0, throttle=50)
    msg.get_type = lambda: "VFR_HUD"
    return msg


def _altitudes(messages):
    return [m for m in messages if isinstance(m, Altitude)]


def test_vfr_hud_does_not_erase_the_relative_altitude():
    """VFR_HUD must carry the last known relative altitude, not 0.0.

    This is the whole bug: every Altitude replaces the fusion node's single
    slot, so a VFR_HUD-derived Altitude reporting 0.0 is indistinguishable from
    the aircraft being on the ground.
    """
    iface = _iface()
    iface._parse(_gpi(3.0))

    alts = _altitudes(iface._parse(_vfr()))

    assert alts, "VFR_HUD stopped publishing an Altitude entirely"
    assert alts[0].relative_m == 3.0, (
        "VFR_HUD published relative_m=%r, erasing the real altitude - the "
        "takeoff gate can never open" % alts[0].relative_m
    )
    assert alts[0].climb_ms == 0.2, "climb rate was lost"


def test_the_takeoff_gate_survives_interleaved_telemetry():
    """Replay the real 10 Hz GLOBAL_POSITION_INT / VFR_HUD interleave.

    Both arrive at 10 Hz on this airframe, so roughly every other altitude
    sample came from VFR_HUD. Not one of them may read 0.0 while the aircraft
    is at 3 m.
    """
    iface = _iface()
    seen = []
    for _ in range(10):
        seen += _altitudes(iface._parse(_gpi(3.0)))
        seen += _altitudes(iface._parse(_vfr()))

    assert len(seen) == 20
    assert all(a.relative_m == 3.0 for a in seen), (
        "relative altitude dropped to %r on some samples"
        % sorted({a.relative_m for a in seen})
    )


def test_a_genuine_zero_is_still_reported():
    """The fix must not pin the altitude - on the ground it must read 0."""
    iface = _iface()
    iface._parse(_gpi(3.0))
    iface._parse(_gpi(0.0))

    alts = _altitudes(iface._parse(_vfr()))
    assert alts[0].relative_m == 0.0
