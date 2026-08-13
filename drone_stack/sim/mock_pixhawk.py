"""MockMavlink - a simulated Pixhawk (Phase 7).

Implements the same :class:`~drone_stack.interfaces.mavlink_interface.MavlinkInterface`
contract as the real link, but is driven by a :class:`~drone_stack.sim.world.SimWorld`.
It also simulates GPS, battery, IMU and attitude, so the whole stack runs with no
hardware attached.
"""
from __future__ import annotations

from typing import Any

from drone_stack.interfaces.mavlink_interface import MavlinkInterface
from drone_stack.msg import LinkQuality, Message, NavCommand
from drone_stack.sim.world import SimWorld
from drone_stack.utils.logging_setup import get_logger


class MockMavlink(MavlinkInterface):
    """Simulated autopilot link backed by a shared SimWorld."""

    def __init__(self, world: SimWorld, config: dict[str, Any] | None = None) -> None:
        self.log = get_logger("sim.pixhawk")
        self._world = world
        self._connected = False
        self._packets = 0
        self._conn_str = (config or {}).get("connection", "sim://pixhawk")

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        self._connected = True
        self.log.info("mock Pixhawk connected (simulation)")
        return True

    def close(self) -> None:
        self._connected = False

    def receive(self) -> list[Message]:
        if not self._connected:
            return []
        self._world.step()
        messages = self._world.get_messages()
        self._packets += len(messages)
        return messages

    def send_command(self, command: NavCommand) -> bool:
        if not self._connected:
            return False
        self._world.command(command)
        return True

    def link_quality(self) -> LinkQuality:
        return LinkQuality(
            connected=self._connected,
            packets_received=self._packets,
            drop_rate_pct=0.0,
            last_heartbeat_age_s=0.0,
            connection_string=self._conn_str,
        )
