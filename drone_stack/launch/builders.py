"""Factories that pick real vs mock hardware interfaces from configuration.

This is the single place where ``mode: sim|real`` is turned into concrete
objects. Keeping it here means nodes never import simulation code, and the mock
implementations never leak into the hardware path.
"""
from __future__ import annotations

from drone_stack.interfaces.lidar_interface import LidarInterface, RealLidar
from drone_stack.interfaces.mavlink_interface import MavlinkInterface, RealMavlink
from drone_stack.utils.config import Config


def build_world(config: Config):
    """Return a fresh :class:`~drone_stack.sim.world.SimWorld` (sim mode only)."""
    from drone_stack.sim import SimWorld

    return SimWorld(config)


def build_mavlink_interface(config: Config, world=None) -> MavlinkInterface:
    if config.mode == "sim":
        from drone_stack.sim import MockMavlink

        if world is None:
            world = build_world(config)
        return MockMavlink(world, config.section("mavlink"))
    return RealMavlink(_with_ceiling(config))


def _with_ceiling(config: Config) -> dict:
    """The mavlink section plus the altitude ceiling the link must enforce.

    The ceiling lives under ``safety``, but RealMavlink.upload_mission needs it
    to clamp a plan on its way into the FC's mission slot - that plan is flown
    by AUTO with the navigator out of the loop, so its clamp does not cover it.
    Copied rather than aliased so nothing here mutates the shared config.
    """
    section = dict(config.section("mavlink"))
    section.setdefault("max_altitude_m", config.get("safety.max_altitude_m", 2.0))
    return section


def build_lidar_interface(config: Config, world=None) -> LidarInterface:
    if config.mode == "sim":
        from drone_stack.sim import MockLidar

        if world is None:
            world = build_world(config)
        return MockLidar(world, config.section("lidar"))
    return RealLidar(config.section("lidar"))
