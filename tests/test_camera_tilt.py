"""Automatic camera tilt on the AUX6 servo during a delivery.

Requirement (2026-09-23): while flying autonomously to the waypoint set in the
app, point the camera DOWN about 3 m before reaching it; after the handshake,
when the aircraft goes to RTL, point it back UP. Then DOWN again while landing,
and forward (UP) once on the ground.

The servo angles live in ``aux2_servo`` - the same block that feeds the GCS
DOWN/UP buttons - and are pinned here rather than read from the shipped
profile, so re-tuning the mechanism cannot silently change what these tests
assert (see test_smooth_avoidance.py for the same rule applied to geometry).
"""
from __future__ import annotations

import copy
import time

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import ArmedStatus, FusedState, MissionPhase, Waypoint
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config

_SERVO = {
    "out_channel": 14, "min_us": 500, "max_us": 2500, "deg_span": 180.0,
    "down_deg": 165.0, "up_deg": 90.0,
    "auto_tilt": True, "down_before_drop_m": 3.0,
}
DOWN_US = 2333      # 500 + 165 * 2000/180 = 2333.3
UP_US = 1500        # 500 +  90 * 2000/180

# Drop point 20 m north of home. The route leg before it has no hold.
DROP = Waypoint(seq=1, x_m=0.0, y_m=20.0, alt_m=2.0, hold_s=20.0,
                radius_m=1.5, kind="hover")
LEG = Waypoint(seq=0, x_m=0.0, y_m=10.0, alt_m=2.0, kind="nav")


def _nav(**servo):
    raw = copy.deepcopy(Config.load().raw)
    raw["aux2_servo"] = {**_SERVO, **servo}
    nav = raw.setdefault("navigation", {})
    nav["avoidance_rtl_router"] = "pi"
    bus = MessageBus()
    sent: list = []
    bus.subscribe(Topics.MAVLINK_CMD, sent.append)
    node = NavigationNode(bus, Config(raw), ServiceRegistry())
    node._armed = True
    node._has_flown = True
    node._home = (12.9, 77.6)
    node._mode = "GUIDED"
    node._phase = MissionPhase.NAVIGATE
    node._mission.waypoints = [copy.copy(LEG), copy.copy(DROP)]
    node._current_wp = 1
    sent.clear()
    return node, sent


def _at(node, y):
    """Place the aircraft on the track, ``y`` metres north of home."""
    node._fused = FusedState(x=0.0, y=y, alt_rel_m=2.0, yaw=0.0, valid=True)


def _servo(sent):
    return [(c.params["channel"], c.params["pwm"])
            for c in sent if c.command == "set_servo"]


class TestCameraDownOnApproach:
    def test_nothing_outside_the_trigger_distance(self):
        node, sent = _nav()
        _at(node, 16.5)                 # 3.5 m out
        node._do_navigate()
        assert _servo(sent) == []

    def test_down_at_three_metres_from_the_drop_point(self):
        node, sent = _nav()
        _at(node, 17.0)                 # exactly 3.0 m out
        node._do_navigate()
        assert _servo(sent) == [(14, DOWN_US)]

    def test_down_is_sent_once_not_every_tick(self):
        node, sent = _nav()
        for y in (17.0, 17.4, 17.8, 18.2):
            _at(node, y)
            node._do_navigate()
        assert _servo(sent) == [(14, DOWN_US)]

    def test_a_route_leg_without_a_hold_never_tilts(self):
        node, sent = _nav()
        node._current_wp = 0
        _at(node, 9.5)                  # 0.5 m from the intermediate leg
        node._do_navigate()
        assert _servo(sent) == []

    def test_the_pi_flown_home_leg_never_tilts(self):
        node, sent = _nav()
        node._mission.waypoints.append(
            Waypoint(seq=2, x_m=0.0, y_m=0.0, hold_s=5.0, kind="rtl"))
        node._current_wp = 2
        _at(node, 1.0)
        node._do_navigate()
        assert _servo(sent) == []

    def test_a_drop_point_reached_in_one_tick_still_gets_the_camera(self):
        # Arrival and the trigger on the same tick: the tilt must come first,
        # or the hold starts with the camera still facing forward.
        node, sent = _nav()
        _at(node, 19.5)
        node._do_navigate()
        assert node._phase == MissionPhase.HOVER
        assert _servo(sent) == [(14, DOWN_US)]

    def test_disabled_by_config(self):
        node, sent = _nav(auto_tilt=False)
        _at(node, 18.0)
        node._do_navigate()
        assert _servo(sent) == []

    def test_the_angle_maps_exactly_as_the_gcs_buttons_map_it(self):
        # app.js aux2DegToUs spreads deg_span over min_us..max_us, so a
        # narrowed envelope RESCALES the angle rather than clipping it:
        # 500 + 165 * 1500/180 = 1875. The automatic DOWN must land where the
        # DOWN button lands, or the two disagree about where "down" is.
        node, sent = _nav(max_us=2000)
        _at(node, 18.0)
        node._do_navigate()
        assert _servo(sent) == [(14, 1875)]

    def test_the_command_is_clamped_to_the_configured_envelope(self):
        # This command goes straight onto MAVLINK_CMD and never passes through
        # GcsHub's per-channel clamp, so the node must apply it itself. Only an
        # angle configured beyond deg_span can reach it.
        node, sent = _nav(down_deg=200.0)
        _at(node, 18.0)
        node._do_navigate()
        assert _servo(sent) == [(14, 2500)]

    def test_a_new_mission_re_arms_the_trigger(self):
        node, sent = _nav()
        _at(node, 18.0)
        node._do_navigate()
        node._begin_mission()
        node._phase = MissionPhase.NAVIGATE
        node._current_wp = 1
        node._do_navigate()
        assert _servo(sent) == [(14, DOWN_US), (14, DOWN_US)]


class TestCameraUpOnReturn:
    def _held_over_drop(self):
        node, sent = _nav()
        _at(node, 19.5)
        node._do_navigate()             # camera DOWN, hold starts
        assert node._phase == MissionPhase.HOVER
        return node, sent

    def test_handshake_then_rtl_raises_the_camera(self):
        node, sent = self._held_over_drop()
        # The handshake confirmed long enough ago that the grace has run out.
        node._ble_delivered_at = time.monotonic() - 60.0
        node._do_hover()
        assert node._phase in (MissionPhase.RTL, MissionPhase.NAVIGATE)
        assert _servo(sent) == [(14, DOWN_US), (14, UP_US)]

    def test_hold_timeout_rtl_raises_the_camera(self):
        node, sent = self._held_over_drop()
        node._hover_until = time.monotonic() - 1.0
        node._do_hover()
        assert _servo(sent)[-1] == (14, UP_US)

    def test_camera_is_raised_before_the_about_face_not_after(self):
        # _enter_rtl returns early while the 180 deg turn runs; the UP must
        # already have gone out by then.
        node, sent = self._held_over_drop()
        node._enter_rtl(turn_first=True)
        assert _servo(sent)[-1] == (14, UP_US)

    def test_arriving_home_does_not_command_it_again(self):
        node, sent = self._held_over_drop()
        node._enter_rtl(turn_first=True)
        node._pi_return_flown = True    # re-entry at home after the Pi's leg
        node._enter_rtl(turn_first=True)
        assert _servo(sent).count((14, UP_US)) == 1

    def test_a_failsafe_rtl_mid_approach_raises_the_camera(self):
        # The pilot needs the forward view most on an emergency return.
        node, sent = _nav()
        _at(node, 18.0)
        node._do_navigate()             # DOWN, still short of the drop point
        node._enter_rtl()               # failsafe / operator: turn_first=False
        assert _servo(sent) == [(14, DOWN_US), (14, UP_US)]

    def test_a_return_that_never_tilted_still_makes_the_position_known(self):
        node, sent = _nav()
        node._enter_rtl()               # abort 20 m out, camera never moved
        assert _servo(sent) == [(14, UP_US)]

    def test_disabled_by_config_the_return_leaves_the_servo_alone(self):
        node, sent = _nav(auto_tilt=False)
        node._enter_rtl()
        assert _servo(sent) == []


class TestCameraOnLanding:
    def _returning(self):
        node, sent = _nav()
        node._cam_tilt = "up"           # as left by the return
        return node, sent

    def test_the_navigators_land_points_the_camera_down(self):
        node, sent = self._returning()
        node._enter_land()
        node._camera_tilt_for_landing()
        assert _servo(sent) == [(14, DOWN_US)]

    def test_a_land_the_navigator_did_not_command_still_counts(self):
        # Pilot's RC LAND switch: the node has stood down into MANUAL, but
        # the camera follows the aircraft, not the navigator.
        node, sent = self._returning()
        node._phase = MissionPhase.MANUAL
        node._mode = "LAND"
        node._camera_tilt_for_landing()
        assert _servo(sent) == [(14, DOWN_US)]

    def test_down_is_sent_once_through_the_descent(self):
        node, sent = self._returning()
        node._mode = "LAND"
        for _ in range(30):
            node._camera_tilt_for_landing()
        assert _servo(sent) == [(14, DOWN_US)]

    def test_touchdown_turns_the_camera_forward(self):
        node, sent = self._returning()
        node._mode = "LAND"
        node._camera_tilt_for_landing()
        node._on_armed(ArmedStatus(armed=False))    # LAND disarms on touchdown
        node._camera_tilt_for_landing()
        assert _servo(sent) == [(14, DOWN_US), (14, UP_US)]

    def test_the_fc_staying_in_land_after_disarm_does_not_re_tilt(self):
        # ArduPilot stays in LAND on the ground. Without the ARMED condition
        # the next tick would drive the camera straight back down.
        node, sent = self._returning()
        node._mode = "LAND"
        node._camera_tilt_for_landing()
        node._on_armed(ArmedStatus(armed=False))
        for _ in range(30):
            node._camera_tilt_for_landing()
        assert _servo(sent) == [(14, DOWN_US), (14, UP_US)]

    def test_power_on_disarmed_moves_nothing(self):
        # No arm -> disarm edge, no LAND: opening the GCS on the bench must
        # not move a mechanism.
        node, sent = _nav()
        node._armed = False
        node._on_armed(ArmedStatus(armed=False))
        node._mode = "LAND"
        node._camera_tilt_for_landing()
        assert _servo(sent) == []

    def test_it_is_wired_into_step_even_under_pilot_override(self):
        node, sent = self._returning()
        node._check_pilot_override = lambda: True
        node._annunciate_while_manual = lambda: None
        node._mode = "LAND"
        node.step()
        assert _servo(sent) == [(14, DOWN_US)]
