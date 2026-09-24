"""The simulator's published heading must mean what the stack thinks it means.

SimWorld works internally in the MATHS convention - yaw measured anticlockwise
from east - because its motion model, ``raycast`` and ``mock_lidar`` are all
written that way and agree with each other. The rest of the stack works in the
COMPASS convention: ``body_to_enu``/``enu_to_body`` put yaw = 0 at north and
count clockwise, and ``FusionNode`` feeds ``Attitude.yaw`` straight into
``fused.yaw``, which every bearing decision then rotates by.

Until 2026-09-19 SimWorld published its internal angle raw. Measured that day:
a simulated aircraft flying due north reported ``Attitude.yaw = 90 deg``. Every
bearing in simulation - the yaw gate, the pre-RTL turn, the about-face, obstacle
body-frame bearings, the GCS heading arrow - was mirrored about the 45 deg line.

The whole 601-test suite passed throughout, because the simulator was
self-consistent: sim obstacles landed exactly where sim yaw said they would.
Only a comparison against the STACK's own rotation helper catches it, which is
what these tests do - they assert against ``body_to_enu``, not against numbers
copied out of the simulator.

This is the same shape as the POSHOLD hover in CLAUDE.md 11: a simulator that
agrees with itself and not with the aircraft. It is worth a dedicated file.
"""
from __future__ import annotations

import math
import time

import pytest

from drone_stack.msg import NavCommand
from drone_stack.nodes.obstacle_tracker import body_to_enu
from drone_stack.sim.world import SimWorld, flip_yaw_frame
from drone_stack.utils.config import Config


def _world(**sim_overrides) -> SimWorld:
    config = Config.load()
    if sim_overrides:
        raw = config.raw
        raw.setdefault("sim", {}).update(sim_overrides)
        config = Config(raw)
    world = SimWorld(config)
    world.state.armed = True
    world.state.z = 3.0
    return world


def _published(world) -> tuple[float, float]:
    """(Attitude.yaw in rad, GpsFix.course_deg) as the stack receives them."""
    yaw = course = None
    for msg in world.get_messages():
        name = type(msg).__name__
        if name == "Attitude":
            yaw = msg.yaw
        elif name == "GpsFix":
            course = msg.course_deg
    assert yaw is not None and course is not None
    return yaw, course


def _fly_to(world, east: float, north: float, seconds: float = 4.0):
    world._target = (east, north, 3.0)
    for _ in range(int(seconds / 0.1)):
        world._integrate(0.1)
    return world


class TestTheHeadingIsACompassHeading:
    """0 = north, counting clockwise, as every consumer assumes."""

    @pytest.mark.parametrize(
        "east, north, expected_deg",
        [
            (0.0, 50.0, 0.0),        # due north
            (50.0, 0.0, 90.0),       # due east
            (0.0, -50.0, 180.0),     # due south
            (-50.0, 0.0, 270.0),     # due west
            (50.0, 50.0, 45.0),      # north-east
        ],
    )
    def test_flying_a_cardinal_direction_reports_its_bearing(
        self, east, north, expected_deg
    ):
        world = _fly_to(_world(), east, north)
        yaw_rad, course_deg = _published(world)

        assert math.degrees(yaw_rad) % 360.0 == pytest.approx(
            expected_deg, abs=1.0), (
            f"flying east={east} north={north} and reporting "
            f"{math.degrees(yaw_rad) % 360.0:.0f} deg, not {expected_deg:.0f}")
        assert course_deg % 360.0 == pytest.approx(expected_deg, abs=1.0)

    def test_north_east_is_the_one_bearing_the_bug_got_right(self):
        """compass = 90 - maths is a REFLECTION about 45 deg, so 45 is its
        fixed point. A test that only checked north-east would have passed
        against the broken simulator - hence the cardinal cases above."""
        world = _fly_to(_world(), 50.0, 50.0)
        yaw_rad, _ = _published(world)

        assert math.degrees(yaw_rad) % 360.0 == pytest.approx(45.0, abs=1.0)
        assert flip_yaw_frame(math.radians(45.0)) == pytest.approx(
            math.radians(45.0))


class TestThePublishedYawAgreesWithTheStacksOwnRotation:
    """The invariant that matters, stated without a magic number in sight:
    rotate 'straight ahead' by the published yaw and you get the direction the
    aircraft is actually travelling."""

    @pytest.mark.parametrize(
        "east, north",
        [(0.0, 50.0), (50.0, 0.0), (0.0, -50.0), (-50.0, 0.0), (30.0, -40.0)],
    )
    def test_nose_forward_rotated_by_yaw_is_the_direction_of_travel(
        self, east, north
    ):
        world = _fly_to(_world(), east, north)
        yaw_rad, _ = _published(world)
        s = world.state

        nose_e, nose_n = body_to_enu(1.0, 0.0, yaw_rad)
        speed = math.hypot(s.vx, s.vy)
        assert speed > 0.5, "not moving; the comparison would be meaningless"

        # Dot product of two unit vectors: 1.0 when they point the same way.
        alignment = (nose_e * s.vx + nose_n * s.vy) / speed
        assert alignment == pytest.approx(1.0, abs=0.02), (
            f"the nose ({math.degrees(yaw_rad) % 360.0:.0f} deg) and the "
            f"velocity (east {s.vx:+.2f}, north {s.vy:+.2f}) disagree - "
            "alignment %.3f" % alignment)


class TestTheYawCommandTurnsTheRightWay:
    """MAV_CMD_CONDITION_YAW direction +1 is clockwise, which must RAISE the
    published compass heading. The internal angle moves the other way; that is
    the point of the conversion, and it is easy to break by 'simplifying'."""

    def test_a_clockwise_turn_increases_the_published_heading(self):
        world = _fly_to(_world(), 0.0, 50.0)          # settle facing north
        before, _ = _published(world)

        world.command(NavCommand(
            command="yaw", params={"angle": 90.0, "direction": 1}))
        after, _ = _published(world)

        turned = math.degrees(after - before) % 360.0
        assert turned == pytest.approx(90.0, abs=1.0), (
            f"a +90 deg clockwise command moved the heading {turned:.0f} deg")

    def test_an_anticlockwise_turn_decreases_it(self):
        world = _fly_to(_world(), 0.0, 50.0)
        before, _ = _published(world)

        world.command(NavCommand(
            command="yaw", params={"angle": 90.0, "direction": -1}))
        after, _ = _published(world)

        turned = math.degrees(after - before) % 360.0
        assert turned == pytest.approx(270.0, abs=1.0)

    def test_an_about_face_lands_on_the_opposite_bearing(self):
        """The pre-RTL turn this simulator has to be able to exercise."""
        world = _fly_to(_world(), 0.0, 50.0)          # north
        world.command(NavCommand(
            command="yaw", params={"angle": 180.0, "direction": 1}))
        after, _ = _published(world)

        assert math.degrees(after) % 360.0 == pytest.approx(180.0, abs=1.0)


class TestTheConfiguredStartHeading:
    """⚠ The two conversions here CANCEL. ``__init__`` converts the configured
    compass heading in, ``get_messages`` converts back out, so a round trip
    holds even if both ends are wrong - deleting ``flip_yaw_frame`` entirely
    leaves every assertion below except the internal one passing. Checked by
    patching the conversion to identity on 2026-09-19; only the state.yaw
    assertion noticed. Keep it, and keep it white-box: it is the one place that
    pins WHERE the frame changes rather than merely that it comes back."""

    def test_start_heading_deg_is_a_compass_heading(self):
        """An operator writing start_heading_deg: 90 means facing east."""
        world = _world(start_heading_deg=90.0)
        yaw_rad, course_deg = _published(world)

        assert math.degrees(yaw_rad) % 360.0 == pytest.approx(90.0, abs=0.5)
        assert course_deg % 360.0 == pytest.approx(90.0, abs=0.5)

    def test_it_is_stored_internally_in_the_maths_frame(self):
        """north in (compass 0) must become east-relative 90 in the state, or
        raycast and mock_lidar - which never go through the conversion - are
        reading a heading the rest of the simulator does not share."""
        world = _world(start_heading_deg=0.0)

        assert math.degrees(world.state.yaw) % 360.0 == pytest.approx(
            90.0, abs=0.5), (
            "state.yaw is not in the maths frame; the conversion at __init__ "
            "is missing or has been made symmetric with get_messages")

    def test_a_configured_heading_points_the_nose_where_it_says(self):
        """The end-to-end meaning: face east, fly body-forward, go east."""
        world = _world(start_heading_deg=90.0)
        world.command(NavCommand(
            command="velocity", params={"vx": 2.0, "vy": 0.0, "vz": 0.0}))
        for _ in range(20):
            world._cmd_vel_expiry = time.monotonic() + 0.5
            world._integrate(0.05)

        assert world.state.x > 1.0, "did not travel east"
        assert abs(world.state.y) < abs(world.state.x) * 0.2

    def test_the_default_start_heading_is_north(self):
        world = _world()
        yaw_rad, _ = _published(world)

        assert math.degrees(yaw_rad) % 360.0 == pytest.approx(0.0, abs=0.5)


class TestTheConversionItself:
    def test_it_is_its_own_inverse(self):
        for deg in (0.0, 17.0, 45.0, 90.0, 179.0, -120.0):
            rad = math.radians(deg)
            there_and_back = flip_yaw_frame(flip_yaw_frame(rad))
            assert there_and_back == pytest.approx(
                math.atan2(math.sin(rad), math.cos(rad)), abs=1e-9)

    def test_it_reflects_rather_than_rotates(self):
        """A rotation preserves the ORDER of two bearings; a reflection swaps
        it. Pinning this is what stops the conversion being 'simplified' into
        an offset, which would look right at 45 deg and be wrong everywhere."""
        a, b = flip_yaw_frame(math.radians(10.0)), flip_yaw_frame(
            math.radians(20.0))
        assert a > b, "10 deg and 20 deg kept their order - that is a rotation"
