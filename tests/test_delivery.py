"""Firebase delivery path: order -> waypoints -> hover -> return, plus the
altitude ceiling and pilot-override rules that bound the whole thing.

These are the behaviours that are expensive to discover on real hardware, so
they are pinned here: an order must never fly higher than the configured
ceiling, the hover must last its full duration, and a mission must come home by
itself once the hold expires.
"""
from __future__ import annotations

import json
import time

import pytest

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.firebase_interface import (
    FileOrders,
    NullOrders,
    build_order_source,
)
from drone_stack.msg import (
    ArmedStatus,
    MissionState,
    LinkQuality,
    FcMessage,
    DeliveryPhase,
    FlightMode,
    FusedState,
    GpsFix,
    MissionPhase,
)
from drone_stack.nodes.firebase_delivery_node import FirebaseDeliveryNode
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.srv.services import ServiceRequest
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import haversine_m

HOME_LAT, HOME_LON = 12.9017, 77.6540


def _stack(**overrides):
    """A navigator with a GPS fix at HOME, ready to be given a delivery."""
    config = Config.load()
    if overrides:
        raw = config.raw          # deep copy; safe to mutate
        raw.setdefault("delivery", {}).update(overrides)
        config = Config(raw)
    bus = MessageBus()
    services = ServiceRegistry()
    nav = NavigationNode(bus, config, services)
    nav._on_gps(GpsFix(fix_type=3, satellites=12, lat=HOME_LAT, lon=HOME_LON))
    return bus, services, nav, config


def _target(north_m: float) -> tuple[float, float]:
    return HOME_LAT + north_m / 111320.0, HOME_LON


# -- waypoint expansion ------------------------------------------------------
def test_single_coordinate_expands_into_multiple_waypoints():
    _, services, nav, _ = _stack(leg_length_m=25.0)
    lat, lon = _target(100.0)
    resp = services.call("set_delivery_target", lat=lat, lon=lon, hover_s=60.0)
    assert resp.success
    assert resp.data["waypoints"] == 4          # 100 m / 25 m legs
    assert abs(resp.data["distance_m"] - 100.0) < 2.0
    plan = resp.data["plan"]
    # The last waypoint is the drop point and is the only one that holds.
    assert plan[-1]["kind"] == "hover"
    assert plan[-1]["hold_s"] == 60.0
    assert all(p["hold_s"] == 0.0 for p in plan[:-1])
    assert haversine_m(plan[-1]["lat"], plan[-1]["lon"], lat, lon) < 0.5


def test_expansion_is_capped_by_max_legs():
    _, services, nav, _ = _stack(leg_length_m=1.0, max_legs=6)
    lat, lon = _target(100.0)
    resp = services.call("set_delivery_target", lat=lat, lon=lon)
    assert resp.data["waypoints"] == 6


def test_delivery_needs_a_home_position():
    config = Config.load()
    services = ServiceRegistry()
    NavigationNode(MessageBus(), config, services)   # no GPS fix published
    resp = services.call("set_delivery_target", lat=HOME_LAT, lon=HOME_LON)
    assert not resp.success
    assert "home" in resp.message.lower()


# -- altitude ceiling --------------------------------------------------------
def test_every_waypoint_respects_the_altitude_ceiling():
    _, services, nav, _ = _stack()
    ceiling = nav._alt_ceiling
    lat, lon = _target(80.0)
    resp = services.call("set_delivery_target", lat=lat, lon=lon, alt_m=ceiling + 50)
    assert all(p["alt_m"] <= ceiling for p in resp.data["plan"])


def test_commanded_altitude_is_clamped_even_when_asked_directly():
    bus, services, nav, _ = _stack()
    ceiling = nav._alt_ceiling
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    services.call("goto_gps", lat=HOME_LAT, lon=HOME_LON, alt=ceiling + 100)
    alts = [c.params["alt"] for c in sent if "alt" in c.params]
    assert alts and all(a <= ceiling for a in alts)


# -- hover then return -------------------------------------------------------
def _fly_to_last_waypoint(nav) -> None:
    """Put the aircraft on the final waypoint with the mission running."""
    nav._armed = True
    nav._phase = MissionPhase.NAVIGATE
    nav._current_wp = nav._mission.count - 1
    wp = nav._mission.waypoints[-1]
    nav._fused = FusedState(x=wp.x_m, y=wp.y_m, alt_rel_m=wp.alt_m)


def test_hover_holds_for_its_full_duration_then_returns_home():
    bus, services, nav, _ = _stack()
    lat, lon = _target(30.0)
    services.call("set_delivery_target", lat=lat, lon=lon, hover_s=60.0)
    _fly_to_last_waypoint(nav)

    nav._do_navigate()
    assert nav._phase == MissionPhase.HOVER
    assert 59.0 < nav.hover_remaining_s <= 60.0

    # Still holding most of a minute later.
    nav._hover_until = time.monotonic() + 30.0
    nav._do_hover()
    assert nav._phase == MissionPhase.HOVER

    # Hold expires -> the last waypoint is consumed -> the mission returns
    # home *on that same tick*. There is no NAVIGATE hop through GUIDED: the
    # drop point is the end of the route, and asking the FC for GUIDED only to
    # ask for SMART_RTL 100 ms later adds a mode change that can be refused
    # while the aircraft is holding station on a finite battery.
    nav._hover_until = time.monotonic() - 0.01
    nav._do_hover()
    assert nav._phase == MissionPhase.RTL


def test_hover_holds_in_guided_and_asserts_the_position_target():
    """The drop-point hold must be flown by a mode the sticks cannot drive.

    GUIDED holds our setpoint; the hold is entered by re-asserting that
    setpoint rather than by a mode change.
    """
    bus, services, nav, _ = _stack(hover_mode="GUIDED")
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    lat, lon = _target(30.0)
    services.call("set_delivery_target", lat=lat, lon=lon, hover_s=5.0)
    _fly_to_last_waypoint(nav)
    nav._do_navigate()

    assert nav._phase == MissionPhase.HOVER
    gotos = [c for c in sent if c.command == "goto"]
    assert gotos, "the hold must re-assert the GUIDED position target"
    assert gotos[-1].params["lat"] == lat
    modes = [c.params.get("mode") for c in sent if c.command == "set_mode"]
    assert not (set(modes) & NavigationNode._STICK_ALTITUDE_MODES), (
        f"the hold entered a throttle-stick-driven mode: {modes}"
    )


@pytest.mark.parametrize("unsafe", ["POSHOLD", "LOITER", "ALT_HOLD", "STABILIZE"])
def test_a_stick_driven_hover_mode_is_refused(unsafe):
    """Config cannot re-arm the crash.

    POSHOLD/LOITER/ALT_HOLD hold ALTITUDE from the pilot's throttle stick.
    On an autonomous delivery that stick rests at RC3_MIN, so entering one
    commands a full-rate descent - it broke this airframe's landing gear on
    two logged flights. The navigator must substitute GUIDED, whatever the
    config says.
    """
    _, services, nav, _ = _stack(hover_mode=unsafe)
    assert nav._hover_mode == "GUIDED"


def test_a_sagging_hover_is_caught_and_the_hold_re_asserted():
    """The backstop: if the aircraft sinks during the hold, arrest it.

    This is the shape of the real crash - a hold that quietly became a
    2.4 m/s descent - so it must not merely be reported, it must be acted on.
    """
    bus, services, nav, _ = _stack(hover_mode="GUIDED")
    lat, lon = _target(30.0)
    services.call("set_delivery_target", lat=lat, lon=lon, hover_s=5.0)
    _fly_to_last_waypoint(nav)
    nav._do_navigate()
    assert nav._phase == MissionPhase.HOVER

    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._mode = "POSHOLD"                     # FC is not where we asked it to be
    wp = nav._hover_wp
    nav._fused = FusedState(x=wp.x_m, y=wp.y_m, alt_rel_m=wp.alt_m - 2.0)  # sinking
    nav._do_hover()

    assert any(
        c.command == "set_mode" and c.params.get("mode") == "GUIDED" for c in sent
    ), "a sagging hold must be pulled back into GUIDED"
    assert any(c.command == "goto" for c in sent), "and the target re-asserted"
    assert "SAG" in nav._status_message.upper()

    # Re-asserted once, not sprayed at 10 Hz.
    sent.clear()
    nav._do_hover()
    assert not sent, "the hold recovery must not re-fire every tick"


def test_return_prefers_smart_rtl_and_falls_back():
    bus, services, nav, _ = _stack(return_mode="SMART_RTL")
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._armed = True
    nav._enter_rtl()
    assert nav._phase == MissionPhase.RTL
    assert any(c.params.get("mode") == "SMART_RTL" for c in sent if c.command == "set_mode")

    # The FC never reports SMART_RTL -> we must not sit there waiting forever.
    nav._rtl_requested_at = time.monotonic() - 5.0
    nav._mode = "LOITER"
    nav._do_rtl()
    assert any(c.params.get("mode") == "RTL" for c in sent if c.command == "set_mode")


# -- command topics are never latched -----------------------------------------
def test_the_bus_refuses_to_latch_command_topics():
    """Structural guard: the rule lives in the bus, not at each subscribe site.

    Opting out per-subscriber only works while every future author remembers
    to. Making the bus refuse means a new command topic is safe by being
    listed in Topics.EVENT_TOPICS, and a forgotten flag cannot reintroduce the
    fault.
    """
    from drone_stack.msg import NavCommand

    bus = MessageBus()
    bus.publish(Topics.MAVLINK_CMD, NavCommand(command="arm", params={}))
    bus.publish(Topics.MISSION_CMD, "start")
    assert bus.latest(Topics.MAVLINK_CMD) is None
    assert bus.latest(Topics.MISSION_CMD) is None

    # even a subscriber that explicitly asks for the latched value gets nothing
    got: list = []
    bus.subscribe(Topics.MAVLINK_CMD, got.append, deliver_latched=True)
    assert got == [], f"replayed a stale command: {got}"

    # ...while live commands still flow, and state topics still latch
    bus.publish(Topics.MAVLINK_CMD, NavCommand(command="brake", params={}))
    assert [c.command for c in got] == ["brake"]
    bus.publish(Topics.MISSION_STATE, MissionState())
    assert bus.latest(Topics.MISSION_STATE) is not None


def test_every_command_topic_is_registered_as_an_event():
    """A topic whose name says 'command' must not be latchable."""
    for name in dir(Topics):
        if name.endswith("_CMD"):
            assert getattr(Topics, name) in Topics.EVENT_TOPICS, (
                f"Topics.{name} carries an instruction but is not in EVENT_TOPICS"
            )


# -- the status line does not overstate what will be flown --------------------
def test_a_clamped_altitude_is_reported_not_silently_substituted():
    """Every route that takes an altitude says so when the ceiling bites."""
    _, _, nav, _ = _stack()
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, yaw=0.0, valid=True)
    nav._armed = True
    ceil = nav._alt_ceiling

    _, msg = nav._manual_takeoff(50.0)
    assert "50.0" in msg and f"{ceil:.1f} m" in msg, msg

    _, msg = nav._manual_goto_relative(0.0, 0.0, 0.0, absolute_alt=80.0)
    assert "80.0" in msg and "ceiling" in msg, msg

    # a relative climb is reported in the delta actually flown, not the ask
    _, msg = nav._manual_goto_relative(0.0, 0.0, 999.0)
    assert "999" in msg and "ceiling" in msg, msg
    assert "up 999" not in msg, f"still claims to fly the requested climb: {msg}"


def test_an_altitude_within_the_ceiling_gets_no_nagging_note():
    """The note must mean something; it cannot appear on every command."""
    _, _, nav, _ = _stack()
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=1.0, yaw=0.0, valid=True)
    nav._armed = True
    for _, msg in (
        nav._manual_takeoff(nav._alt_ceiling),
        nav._manual_goto_relative(5.0, 0.0, 0.0),
        nav._manual_goto_relative(0.0, 0.0, 0.5),
    ):
        assert "ceiling" not in msg, f"nagged about a legal altitude: {msg}"


# -- stale command replay -----------------------------------------------------
def test_a_recreated_mavlink_node_does_not_replay_the_last_command():
    """The supervisor watchdog recreating a dead node must not re-arm.

    The bus latches every topic and replays it to new subscribers. MavlinkNode
    subscribes in __init__ and queues whatever arrives straight out to the
    autopilot, so before the deliver_latched=False fix, a watchdog recreation
    mid-flight re-issued the last command - potentially 'arm' or 'takeoff'.
    """
    from drone_stack.nodes.mavlink_node import MavlinkNode
    from drone_stack.msg import NavCommand

    bus = MessageBus()
    cfg = Config.load()

    class _Iface:
        connected = True
        def connect(self): return True
        def close(self): pass
        def link_quality(self): return LinkQuality(connected=True)

    bus.publish(Topics.MAVLINK_CMD, NavCommand(command="arm", params={}))
    node = MavlinkNode(bus, cfg, _Iface())
    assert len(node._cmd_queue) == 0, (
        f"replayed a stale command on construction: {list(node._cmd_queue)}"
    )

    # A command published *after* construction must still get through.
    bus.publish(Topics.MAVLINK_CMD, NavCommand(command="brake", params={}))
    assert [c.command for c in node._cmd_queue] == ["brake"]


# -- home: the point everything is measured from ------------------------------
def _nav_no_home():
    config = Config.load()
    bus, services = MessageBus(), ServiceRegistry()
    return bus, services, NavigationNode(bus, config, services)


def test_home_ignores_a_fix_that_fails_the_configured_minimums():
    """min_satellites / min_gps_fix_type were config nobody read.

    Home anchored on the first 3D fix of any quality. Indoors on five
    satellites that can be hundreds of metres out, and geofence distance,
    delivery radius and every ENU->lat/lon conversion inherit the error.
    """
    _, _, nav = _nav_no_home()
    assert nav._min_sats >= 6 and nav._min_fix_type >= 3

    nav._on_gps(GpsFix(fix_type=2, satellites=12, lat=HOME_LAT, lon=HOME_LON))
    assert nav._home is None, "latched on a 2D fix"

    nav._on_gps(GpsFix(fix_type=3, satellites=nav._min_sats - 1,
                       lat=HOME_LAT, lon=HOME_LON))
    assert nav._home is None, "latched below min_satellites"

    nav._on_gps(GpsFix(fix_type=3, satellites=nav._min_sats,
                       lat=HOME_LAT, lon=HOME_LON))
    assert nav._home == (HOME_LAT, HOME_LON)


def test_home_follows_the_aircraft_until_it_arms():
    """On the ground, the current fix *is* the launch point.

    A home latched on the session's first marginal fix used to persist
    unchanged after the aircraft was carried outside to a real one.
    """
    _, _, nav = _nav_no_home()
    nav._on_gps(GpsFix(fix_type=3, satellites=7, lat=HOME_LAT, lon=HOME_LON))
    assert nav._home == (HOME_LAT, HOME_LON)

    moved_lat = HOME_LAT + 400.0 / 111320.0        # carried 400 m away
    nav._on_gps(GpsFix(fix_type=3, satellites=14, lat=moved_lat, lon=HOME_LON))
    assert nav._home == (moved_lat, HOME_LON), "home did not follow to the pad"


def test_home_freezes_the_moment_the_aircraft_arms():
    """Latching after takeoff would put home wherever we were flying."""
    _, _, nav = _nav_no_home()
    nav._on_gps(GpsFix(fix_type=3, satellites=12, lat=HOME_LAT, lon=HOME_LON))
    nav._on_armed(ArmedStatus(armed=True))
    assert nav._has_flown

    airborne = HOME_LAT + 200.0 / 111320.0
    nav._on_gps(GpsFix(fix_type=3, satellites=14, lat=airborne, lon=HOME_LON))
    assert nav._home == (HOME_LAT, HOME_LON), "home moved while flying"

    # and it stays frozen after landing/disarm, for the rest of the power cycle
    nav._on_armed(ArmedStatus(armed=False))
    nav._on_gps(GpsFix(fix_type=3, satellites=14, lat=airborne, lon=HOME_LON))
    assert nav._home == (HOME_LAT, HOME_LON), "home moved after disarming"


def test_the_navigator_is_the_only_authority_on_home():
    """The delivery node measures its radius from the navigator's home."""
    bus, services, nav, config = _stack()
    node = FirebaseDeliveryNode(bus, config, services)
    assert node._home is None

    nav._publish_state()
    assert node._home == (HOME_LAT, HOME_LON), (
        "delivery node did not take home from the navigator"
    )


# -- the 3 m altitude hardlock -----------------------------------------------
def test_no_path_can_command_an_altitude_above_the_ceiling():
    """Every route to an altitude, including the raw send, ends up clamped."""
    bus, services, nav, _ = _stack()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=2.0, yaw=0.0, valid=True)
    nav._armed = True

    nav._manual_takeoff(99.0)
    nav._manual_goto_relative(0.0, 0.0, 500.0)
    services.call("goto_gps", lat=nav._home[0] + 1e-4, lon=nav._home[1], alt=1234.0)
    services.call("set_delivery_target", lat=nav._home[0] + 2e-4,
                  lon=nav._home[1], alt_m=999.0, hover_s=60)
    nav._send("takeoff", altitude=float("inf"))
    nav._send("goto", lat=1.0, lon=1.0, alt=float("nan"))

    alts = [c.params.get("alt", c.params.get("altitude"))
            for c in sent if "alt" in c.params or "altitude" in c.params]
    assert alts, "expected altitude-bearing commands"
    for a in alts:
        assert 0.5 <= float(a) <= nav._alt_ceiling, f"escaped the ceiling: {a}"


def test_the_hardlock_stays_armed_when_auto_failsafes_are_disabled():
    """The bench-testing switch must not disarm the ceiling.

    config/real.yaml has carried failsafes_enabled: false for bench work. If
    the ceiling check sat behind that switch, the one limit that protects the
    airframe would be off exactly when someone forgot to flip it back.
    """
    for enabled in (True, False):
        bus, services, nav, _ = _stack()
        nav._safety = dict(nav._safety)
        nav._safety["failsafes_enabled"] = enabled
        nav._phase = MissionPhase.NAVIGATE
        nav._armed = True               # the ceiling is gated on armed
        over = nav._alt_ceiling + nav._alt_margin + 0.01
        nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=over, yaw=0.0, valid=True)
        assert nav._check_failsafe() == "max_altitude", (
            f"ceiling not enforced with failsafes_enabled={enabled}"
        )


def test_the_hardlock_brakes_rather_than_commanding_a_descent():
    """A bad altitude estimate must not be answered by flying down into it."""
    bus, services, nav, _ = _stack()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._phase = MissionPhase.NAVIGATE
    nav._armed = True                   # the ceiling is gated on armed
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=9.0, yaw=0.0, valid=True)
    nav.step()
    assert nav._phase == MissionPhase.HOLD
    assert [c.command for c in sent] == ["brake"]
    assert "HARDLOCK" in nav._status_message


def test_the_hardlock_never_fights_the_pilot():
    """Manual control outranks the ceiling. The human is flying; stand down."""
    bus, services, nav, _ = _stack()
    sent: list = []
    nav._phase = MissionPhase.NAVIGATE
    nav._set_mode("GUIDED")
    nav._on_mode(FlightMode(mode_name="ALT_HOLD"))
    nav._check_pilot_override()
    nav._mode_mismatch_since -= 5.0
    assert nav._check_pilot_override() is True

    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    # The bus replays the latched message to a new subscriber, so the set_mode
    # from the setup above lands here immediately. Drop it: we are measuring
    # what step() emits from now on, not what was already on the bus.
    sent.clear()
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=40.0, yaw=0.0, valid=True)
    for _ in range(40):
        nav.step()
    assert sent == [], f"commanded the aircraft while the pilot flew: {sent}"


def test_a_ceiling_breach_below_the_margin_is_not_a_nuisance_trip():
    """Baro noise around a 2 m hover must not brake the aircraft."""
    bus, services, nav, _ = _stack()
    nav._phase = MissionPhase.NAVIGATE
    for alt in (1.0, 2.0, nav._alt_ceiling + nav._alt_margin - 0.01):
        nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=alt, yaw=0.0, valid=True)
        assert nav._check_failsafe() != "max_altitude", f"nuisance trip at {alt} m"


# -- arming refusals ---------------------------------------------------------
def test_arming_retries_are_paced_not_fired_every_step():
    """The autopilot refusing to arm must not turn into a command flood.

    The original code re-sent set_mode+arm on every step (~10 Hz), producing a
    dozen "PreArm: GPS glitching" refusals a second on the link.
    """
    bus, services, nav, _ = _stack()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._armed = False
    nav._phase = MissionPhase.ARMING
    nav._arm_retry_s = 10.0          # nothing should retry within the test

    for _ in range(20):
        nav._do_arming()

    arms = [c for c in sent if c.command == "arm"]
    assert len(arms) == 1, f"expected one arm attempt, got {len(arms)}"


def test_arming_gives_up_and_reports_the_autopilots_own_reason():
    bus, services, nav, _ = _stack()
    nav._armed = False
    nav._phase = MissionPhase.ARMING
    nav._on_fc_message(FcMessage(text="PreArm: GPS glitching", severity=3,
                                 is_prearm=True))
    nav._do_arming()
    assert "GPS glitching" in nav._status_message

    nav._arm_started_at = time.monotonic() - 999.0    # past the timeout
    nav._do_arming()
    assert nav._phase == MissionPhase.IDLE
    assert "GPS glitching" in nav._status_message
    assert "GPS glitching" in services.call("mission_status").data["arm_refusal"]


def test_a_stale_refusal_is_not_reported_as_the_live_reason():
    """A prearm message from a minute ago says nothing about now."""
    _, _, nav, _ = _stack()
    nav._on_fc_message(FcMessage(text="PreArm: GPS glitching", is_prearm=True))
    nav._fc_refusal_at = time.monotonic() - 60.0
    assert nav._fc_refusal_reason() == ""


def test_a_delivery_that_never_armed_aborts_instead_of_showing_enroute():
    """An aircraft still on the pad must not read as EN ROUTE on the panel."""
    _, services, nav, node = _delivery_stack(auto_accept=False)
    lat, lon = _target(40.0)
    services.call("delivery_inject", lat=lat, lon=lon, order_id="m1")
    services.call("delivery_accept")
    assert node._state.phase == DeliveryPhase.ACCEPTED

    # The autopilot refuses; the navigator gives up and falls back to IDLE.
    nav._on_fc_message(FcMessage(text="PreArm: GPS glitching", is_prearm=True))
    nav._phase = MissionPhase.IDLE
    node._track_active()

    assert node._state.phase == DeliveryPhase.ABORTED
    assert "GPS glitching" in node._state.message
    assert "GPS glitching" in node._state.last_error


def test_arming_shows_progress_rather_than_silence():
    _, services, nav, node = _delivery_stack(auto_accept=False)
    lat, lon = _target(40.0)
    services.call("delivery_inject", lat=lat, lon=lon, order_id="m1")
    services.call("delivery_accept")
    nav._phase = MissionPhase.ARMING
    nav._on_fc_message(FcMessage(text="PreArm: Need 3D Fix", is_prearm=True))
    node._track_active()
    assert node._state.phase == DeliveryPhase.ACCEPTED
    assert "Need 3D Fix" in node._state.message


# -- pilot authority ---------------------------------------------------------
def test_transmitter_mode_change_stands_the_navigator_down():
    bus, services, nav, _ = _stack()
    nav._override_grace = 0.0
    nav._set_mode("GUIDED")
    nav._on_mode(FlightMode(mode_name="LAND"))    # pilot flipped the switch
    # The first mismatch only starts the grace timer - a mode change we asked
    # for takes a moment to be reported back, and that must not read as a
    # takeover.
    assert nav._check_pilot_override() is False
    assert nav._check_pilot_override() is True
    assert nav._pilot_override is True
    assert nav._phase == MissionPhase.MANUAL
    # ...and only an operator action gives control back.
    assert services.call("resume").success
    assert nav._pilot_override is False


def test_our_own_mode_changes_are_not_an_override():
    _, _, nav, _ = _stack()
    nav._override_grace = 0.0
    nav._set_mode("POSHOLD")
    nav._on_mode(FlightMode(mode_name="POSHOLD"))
    assert nav._check_pilot_override() is False


# -- home latching -----------------------------------------------------------
def test_home_is_latched_on_the_ground_and_never_moves():
    config = Config.load()
    nav = NavigationNode(MessageBus(), config, ServiceRegistry())
    nav._on_gps(GpsFix(fix_type=2, lat=1.0, lon=1.0))       # 2D fix: not good enough
    assert nav._home is None
    nav._on_gps(GpsFix(fix_type=3, satellites=9, lat=HOME_LAT, lon=HOME_LON))
    assert nav._home == (HOME_LAT, HOME_LON)
    nav._on_armed(ArmedStatus(armed=True))
    nav._on_gps(GpsFix(fix_type=3, satellites=9, lat=50.0, lon=8.0))
    assert nav._home == (HOME_LAT, HOME_LON)                # airborne: unchanged


# -- order source ------------------------------------------------------------
def test_file_order_source_reads_the_flutter_field_names(tmp_path):
    path = tmp_path / "order.json"
    path.write_text(
        '{"orderId": "abc", "recipientId": "r1", '
        '"targetLat": 12.5, "targetLng": 77.5, "status": "DISPATCHED"}',
        encoding="utf-8",
    )
    src = FileOrders({"file_path": str(path)})
    src.connect()
    orders = src.poll()
    assert len(orders) == 1
    assert orders[0].order_id == "abc"
    assert orders[0].target_lat == 12.5
    assert orders[0].target_lon == 77.5      # targetLng -> target_lon


def test_disabled_delivery_builds_a_null_source():
    assert isinstance(build_order_source({"enabled": False}), NullOrders)


# -- the node ----------------------------------------------------------------
def _delivery_stack(**overrides):
    overrides.setdefault("source", "none")
    bus, services, nav, config = _stack(**overrides)
    node = FirebaseDeliveryNode(bus, config, services)
    node._home = (HOME_LAT, HOME_LON)
    return bus, services, nav, node


def _file_stack(tmp_path, orders, **overrides):
    """A delivery node fed by a JSON file holding several orders."""
    path = tmp_path / "orders.json"
    path.write_text(json.dumps(orders), encoding="utf-8")
    overrides.setdefault("source", "file")
    overrides["file_path"] = str(path)
    overrides.setdefault("max_delivery_radius_m", 5000.0)
    bus, services, nav, config = _stack(**overrides)
    node = FirebaseDeliveryNode(bus, config, services)
    node._home = (HOME_LAT, HOME_LON)
    node._source.connect()
    node._poll_orders()
    node._refresh_listing()
    node._publish_state()
    return bus, services, nav, node


def _app_order(oid, north_m, status="DISPATCHED", who="gvish"):
    lat, lon = _target(north_m)
    return {"orderId": oid, "recipientId": who, "targetLat": lat, "targetLng": lon,
            "status": status, "createdAt": "2026-08-20T15:00:00"}


def test_every_order_from_the_app_reaches_the_dashboard(tmp_path):
    """The panel must show the whole queue, not just the one being flown.

    "I placed an order and nothing appeared" is the failure this guards: an
    order the drone will not fly still has to be visible, with a reason.
    """
    bus, _, _, node = _file_stack(tmp_path, [
        _app_order("A1", 40.0),
        _app_order("A2", 90.0, who="kavya"),
    ])
    state = bus.latest(Topics.DELIVERY_STATE)
    ids = [o["order_id"] for o in state.orders]
    assert ids == ["A1", "A2"]
    assert [o["recipient_id"] for o in state.orders] == ["gvish", "kavya"]
    assert all(o["dispatchable"] for o in state.orders)
    assert abs(state.orders[0]["distance_m"] - 40.0) < 2.0


def test_an_unflyable_order_is_shown_with_its_reason(tmp_path):
    """Out of range is a fact about the order, not a reason to hide it."""
    bus, _, _, node = _file_stack(
        tmp_path, [_app_order("FAR", 4000.0)], max_delivery_radius_m=100.0
    )
    row = bus.latest(Topics.DELIVERY_STATE).orders[0]
    assert row["order_id"] == "FAR"
    assert row["dispatchable"] is False
    assert "limit" in row["blocked_reason"]


def test_operator_can_pick_which_queued_order_to_fly(tmp_path):
    bus, services, nav, node = _file_stack(
        tmp_path, [_app_order("A1", 40.0), _app_order("A2", 90.0)]
    )
    # The oldest is offered by default...
    assert node._pending.order_id == "A1"

    resp = services.call("delivery_select", order_id="A2")
    assert resp.success
    assert node._pending.order_id == "A2"
    assert nav._phase == MissionPhase.IDLE          # selecting never launches

    # ...and the choice survives the next poll rather than snapping back.
    node._poll_orders()
    assert node._pending.order_id == "A2"

    node._publish_state()
    assert bus.latest(Topics.DELIVERY_STATE).selected_order_id == "A2"

    # Only then does accepting commit the aircraft, to the chosen order.
    assert services.call("delivery_accept").success
    assert node._active.order_id == "A2"


def test_selecting_an_unknown_order_is_refused(tmp_path):
    _, services, _, node = _file_stack(tmp_path, [_app_order("A1", 40.0)])
    resp = services.call("delivery_select", order_id="does-not-exist")
    assert not resp.success
    assert node._pending.order_id == "A1"           # unchanged


def test_refresh_reports_what_it_found(tmp_path):
    _, services, _, _ = _file_stack(
        tmp_path, [_app_order("A1", 40.0), _app_order("A2", 90.0)]
    )
    resp = services.call("delivery_refresh")
    assert resp.success
    assert resp.data["orders"] == 2


def test_a_pending_order_already_knows_its_distance(tmp_path):
    """The operator decides on ACCEPT with the distance in front of them, so it
    cannot wait until after acceptance to be filled in."""
    bus, _, _, _ = _file_stack(tmp_path, [_app_order("A1", 40.0)])
    state = bus.latest(Topics.DELIVERY_STATE)
    assert state.phase == DeliveryPhase.PENDING
    assert abs(state.distance_m - 40.0) < 2.0


def test_delivery_services_are_registered():
    _, services, _, _ = _delivery_stack()
    for name in ("delivery_accept", "delivery_abort", "delivery_status",
                 "delivery_inject", "delivery_set_auto", "delivery_select",
                 "delivery_refresh"):
        assert services.has(name)


def test_injected_order_waits_for_an_operator_by_default():
    _, services, nav, node = _delivery_stack(auto_accept=False)
    lat, lon = _target(40.0)
    resp = services.call("delivery_inject", lat=lat, lon=lon)
    assert resp.success
    assert node._state.phase == DeliveryPhase.PENDING
    assert nav._phase == MissionPhase.IDLE          # nothing has been armed

    accepted = services.call("delivery_accept")
    assert accepted.success
    assert node._state.phase == DeliveryPhase.ACCEPTED
    # start_mission only latches the request; the navigator commits to the air
    # on its own step, which is what keeps arming off a caller's thread.
    assert nav._start_requested is True
    nav._maybe_auto_start()
    assert nav._phase == MissionPhase.ARMING
    assert nav._mission.name == f"delivery-{node._active.order_id}"


def test_an_injected_order_is_not_withdrawn_by_an_empty_poll():
    """A manual/test order lives only in the node, never in the query.

    The poll loop withdraws a pending order when it stops appearing upstream
    (cancelled in the app). An injected order never appears there at all, so
    that rule must not apply to it - otherwise the test path evaporates a
    couple of seconds after the order is placed and nobody can accept it.
    """
    _, services, _, node = _delivery_stack(auto_accept=False)
    lat, lon = _target(40.0)
    services.call("delivery_inject", lat=lat, lon=lon, order_id="manual-1")
    node._poll_orders()                      # the "none" source returns nothing
    node._poll_orders()
    assert node._state.phase == DeliveryPhase.PENDING
    assert node._pending is not None
    assert services.call("delivery_accept").success


def test_order_beyond_the_delivery_radius_is_refused():
    _, services, nav, node = _delivery_stack(max_delivery_radius_m=50.0)
    lat, lon = _target(500.0)
    resp = services.call("delivery_inject", lat=lat, lon=lon)
    assert not resp.success
    assert node._state.phase == DeliveryPhase.REJECTED
    assert nav._phase == MissionPhase.IDLE


def test_null_island_order_is_refused():
    _, services, _, node = _delivery_stack()
    assert not services.call("delivery_inject", lat=0.0, lon=0.0).success
    assert node._state.phase == DeliveryPhase.REJECTED


def test_remaining_distance_is_measured_from_where_the_drone_is():
    """The dashboard's progress bar reads `remaining_m`, so it has to shrink.

    Measuring home-to-target instead would leave it pinned at the full route
    length for the entire flight.
    """
    _, services, nav, node = _delivery_stack(auto_accept=False)
    # Scale to whatever the delivery radius is configured to: this test is
    # about the arithmetic, not the limit, and hardcoding a distance means it
    # breaks the next time the operating envelope is retuned.
    full = node._max_radius * 0.9
    part = full * 0.75
    lat, lon = _target(full)
    services.call("delivery_inject", lat=lat, lon=lon, order_id="m1")
    services.call("delivery_accept")
    nav._maybe_auto_start()

    node._on_gps(GpsFix(fix_type=3, satellites=12, lat=HOME_LAT, lon=HOME_LON))
    assert abs(node._remaining_m(node._active, returning=False) - full) < 2.0

    # ...three quarters of the way there
    node._on_gps(GpsFix(fix_type=3, satellites=12, lat=_target(part)[0], lon=lon))
    assert abs(node._remaining_m(node._active, returning=False) - (full - part)) < 2.0
    # ...and on the way back it counts down to home, not to the drop point.
    assert abs(node._remaining_m(node._active, returning=True) - part) < 2.0
    # home itself never moves with the aircraft
    assert node._home == (HOME_LAT, HOME_LON)


def test_idle_message_follows_the_link_so_the_panel_self_heals():
    """The dashboard must stop saying "no Firebase key" once a key is added,
    without anyone restarting the stack."""
    bus, services, _, node = _delivery_stack()
    node._source.link = "no-credentials"
    node._publish_state()
    assert "no Firebase key" in bus.latest(Topics.DELIVERY_STATE).message

    node._source.link = "online"           # key dropped in, node reconnected
    node._publish_state()
    assert bus.latest(Topics.DELIVERY_STATE).message == "waiting for orders"


def test_a_recovered_source_clears_its_old_error():
    """Installing the Firebase key must wipe the "no key" error off the panel,
    not leave the operator chasing a problem that is already fixed."""
    bus, _, _, node = _delivery_stack()
    node._source.link = "no-credentials"
    node._source.last_error = "no service-account key"
    node._publish_state()
    assert bus.latest(Topics.DELIVERY_STATE).last_error

    node._source.link = "online"
    node._source.last_error = ""
    node._publish_state()
    assert bus.latest(Topics.DELIVERY_STATE).last_error == ""


def test_delivery_state_is_published_for_the_gcs():
    bus, services, _, node = _delivery_stack()
    lat, lon = _target(40.0)
    services.call("delivery_inject", lat=lat, lon=lon)
    node._publish_state()
    state = bus.latest(Topics.DELIVERY_STATE)
    assert state is not None
    assert state.phase == DeliveryPhase.PENDING
    assert state.target_lat == lat


# -- the default delivery profile: exact point, 15 s hold, SMART_RTL ---------
#
# One order from the Flutter app must produce exactly one behaviour, and it
# must be the one the operator was promised. These pin the three halves of
# that promise so a config edit or a refactor cannot quietly change it.


def test_the_shipped_default_holds_for_fifteen_seconds():
    """An order that names no hold time gets 15 s, not a minute."""
    _, services, nav, config = _stack()
    assert float(config.section("delivery")["hover_seconds"]) == 15.0
    lat, lon = _target(30.0)
    plan = services.call("set_delivery_target", lat=lat, lon=lon)
    assert plan.success
    assert plan.data["hover_s"] == 15.0
    assert nav._mission.waypoints[-1].hold_s == 15.0


def test_the_order_document_cannot_choose_its_own_hold_or_height():
    """The customer supplies a point on a map. Nothing else.

    Height and loiter time are ours: a buggy or hostile client that writes
    hoverSeconds: 3600 must not be able to park the aircraft over an address
    until the battery gives out.
    """
    from drone_stack.interfaces.firebase_interface import _order_from_doc

    order = _order_from_doc(
        "doc1",
        {
            "orderId": "H43J5MwzxMfAjqb46Ssq",
            "targetLat": HOME_LAT + 0.0002,
            "targetLng": HOME_LON,
            "hoverSeconds": 3600.0,
            "hoverAltM": 120.0,
        },
        {"hover_seconds": 15.0, "hover_alt_m": 3.0},
    )
    assert order is not None
    assert order.hover_seconds == 15.0
    assert order.hover_alt_m == 3.0


def test_the_last_waypoint_is_the_customers_exact_coordinate():
    """The goto carries the order's own lat/lon, not a derived value.

    enu_to_geodetic is an exact inverse of geodetic_to_enu at delivery ranges,
    so the previous round trip did land on the right coordinate - but only for
    as long as that stays true of both functions. This pins the guarantee to
    the number the customer actually sent instead of to that invariant.
    """
    bus, services, nav, _ = _stack()
    lat, lon = _target(30.0)
    services.call("set_delivery_target", lat=lat, lon=lon)

    assert nav._mission.waypoints[-1].lat == lat
    assert nav._mission.waypoints[-1].lon == lon

    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._armed = True
    nav._phase = MissionPhase.NAVIGATE
    nav._current_wp = nav._mission.count - 1
    nav._fused = FusedState(x=0.0, y=0.0, alt_rel_m=3.0)   # still far out
    nav._do_navigate()

    gotos = [c for c in sent if c.command == "goto"]
    assert gotos, "no goto was issued for the final waypoint"
    assert gotos[-1].params["lat"] == lat
    assert gotos[-1].params["lon"] == lon


def test_the_drop_point_hold_returns_home_without_asking_for_guided():
    """Hold expires -> SMART_RTL, with no GUIDED in between.

    GUIDED is the mode the FC is most likely to refuse (EKF/GPS quality). A
    refusal while holding station over the customer is how an aircraft ends up
    on a dead battery, so the last hold hands straight to SMART_RTL.
    """
    bus, services, nav, _ = _stack(return_mode="SMART_RTL", hover_mode="GUIDED")
    lat, lon = _target(30.0)
    services.call("set_delivery_target", lat=lat, lon=lon)
    _fly_to_last_waypoint(nav)
    nav._do_navigate()
    assert nav._phase == MissionPhase.HOVER

    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    nav._hover_until = time.monotonic() - 0.01
    nav._do_hover()

    assert nav._phase == MissionPhase.RTL
    modes = [
        c.params.get("mode") for c in sent if c.command == "set_mode"
    ]
    assert "SMART_RTL" in modes
    assert "GUIDED" not in modes, (
        "the drop-point hold detoured through GUIDED on its way home"
    )


def test_an_intermediate_hold_still_resumes_the_mission():
    """Only the *last* hold goes home; a mid-route hold carries on."""
    bus, services, nav, _ = _stack(hover_mode="GUIDED")
    lat, lon = _target(60.0)
    services.call("set_delivery_target", lat=lat, lon=lon)
    assert nav._mission.count >= 2, "need a multi-leg route for this test"

    nav._armed = True
    nav._phase = MissionPhase.HOVER
    nav._current_wp = 0                       # not the final waypoint
    nav._hover_until = time.monotonic() - 0.01
    nav._do_hover()

    assert nav._phase == MissionPhase.NAVIGATE
    assert nav._current_wp == 1


# -- an order that arrives before the GPS does -------------------------------
#
# A Pi that has just booted has no fix for the first minute or two. An order
# landing in that window is a perfectly good delivery, and _reject() remembers
# an order id for the life of the process - so rejecting it there would strand
# the customer with no way back short of restarting the service.


def _one_order(tmp_path, **overrides):
    """One order, and a node that has NOT polled yet.

    Unlike _file_stack this leaves home unset and the first poll unrun, so the
    test can decide what the world looked like when the order first arrived.
    """
    path = tmp_path / "orders.json"
    path.write_text(
        json.dumps([{
            "orderId": "H43J5MwzxMfAjqb46Ssq",
            "targetLat": HOME_LAT + 0.0002,
            "targetLng": HOME_LON,
            "status": "DISPATCHED",
        }]),
        encoding="utf-8",
    )
    overrides.setdefault("source", "file")
    overrides["file_path"] = str(path)
    overrides.setdefault("max_delivery_radius_m", 5000.0)
    bus, services, nav, config = _stack(**overrides)
    node = FirebaseDeliveryNode(bus, config, services)
    node._source.connect()
    return bus, services, nav, node


def test_an_order_that_beats_the_gps_fix_is_held_not_rejected(tmp_path):
    bus, services, nav, node = _one_order(tmp_path)
    assert node._home is None               # cold boot: no fix yet
    node._poll_orders()

    assert node._state.phase != DeliveryPhase.REJECTED, (
        "a good order was rejected for a condition that clears by itself"
    )
    assert node._pending is not None
    assert node._deferred is True
    assert "H43J5MwzxMfAjqb46Ssq" not in node._handled, (
        "the order id was burned - it can never be offered again"
    )
    assert "held" in node._state.message


def test_the_held_order_flies_once_the_fix_arrives(tmp_path):
    bus, services, nav, node = _one_order(tmp_path)
    node._home = None
    node._poll_orders()
    assert node._deferred is True

    node._home = (HOME_LAT, HOME_LON)       # sats acquired
    node._poll_orders()

    assert node._deferred is False
    assert node._state.phase == DeliveryPhase.PENDING
    assert node._pending is not None
    assert node._pending.order_id == "H43J5MwzxMfAjqb46Ssq"


def test_accepting_before_the_fix_does_not_burn_the_order(tmp_path):
    """An impatient ACCEPT & FLY must stay pressable."""
    bus, services, nav, node = _one_order(tmp_path)
    node._home = None
    node._poll_orders()

    r = node._svc_accept(ServiceRequest("delivery_accept"))
    assert r.success is False
    assert "GPS" in r.message
    assert "H43J5MwzxMfAjqb46Ssq" not in node._handled
    assert node._pending is not None, "the offer was withdrawn by a failed accept"


def test_an_out_of_range_order_is_still_rejected_outright(tmp_path):
    """The retryable path must not soften a real refusal."""
    bus, services, nav, node = _file_stack(
        tmp_path,
        [{
            "orderId": "far",
            "targetLat": HOME_LAT + 0.5,     # ~55 km
            "targetLng": HOME_LON,
            "status": "DISPATCHED",
        }],
        max_delivery_radius_m=60.0,
    )
    assert node._state.phase == DeliveryPhase.REJECTED
    assert "far" in node._handled
    assert node._deferred is False


# -- the ceiling is enforced at the wire, not just in the navigator ----------
#
# A plan written into the FC's own mission slot is flown by AUTO with this
# process out of the loop entirely, so NavigationNode._clamp_alt cannot cover
# it. RealMavlink.upload_mission has to hold the line itself.


def test_the_link_carries_the_ceiling_from_the_safety_section():
    from drone_stack.launch.builders import _with_ceiling

    config = Config.load()
    section = _with_ceiling(config)
    assert section["max_altitude_m"] == config.get("safety.max_altitude_m")
    assert "connection" in section, "the mavlink section was replaced, not extended"
    assert "max_altitude_m" not in config.section("mavlink"), (
        "_with_ceiling mutated the shared config"
    )


def test_an_uploaded_mission_cannot_exceed_the_ceiling():
    from drone_stack.interfaces.mavlink_interface import RealMavlink

    link = RealMavlink({"connection": "udp:127.0.0.1:1", "max_altitude_m": 3.0})
    assert link._clamp_mission_alt(50.0, "takeoff altitude") == 3.0
    assert link._clamp_mission_alt(3.0, "waypoint 1") == 3.0
    assert link._clamp_mission_alt(2.5, "waypoint 2") == 2.5


def test_a_link_with_no_ceiling_configured_defaults_to_two_metres():
    """A missing key must not mean 'no limit'."""
    from drone_stack.interfaces.mavlink_interface import RealMavlink

    link = RealMavlink({"connection": "udp:127.0.0.1:1"})
    assert link._alt_ceiling == 2.0
    assert link._clamp_mission_alt(120.0, "takeoff altitude") == 2.0
