"""MavlinkNode - Phase 2.

Owns a :class:`~drone_stack.interfaces.mavlink_interface.MavlinkInterface`
(real or mock), auto-connects with retry, publishes every telemetry stream to
the bus, and forwards :class:`~drone_stack.msg.NavCommand` messages received on
``/cmd/mavlink`` to the autopilot.
"""
from __future__ import annotations

import threading
from collections import deque

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.mavlink_interface import MavlinkInterface
from drone_stack.msg import (
    Altitude,
    ArmedStatus,
    Attitude,
    Battery,
    FlightMode,
    GpsFix,
    Heartbeat,
    Imu,
    LinkQuality,
    NavCommand,
    RcChannels,
    SystemStatus,
    Velocity,
)
from drone_stack.utils.config import Config
from drone_stack.utils.node import NodeBase


class MavlinkNode(NodeBase):
    """Publishes MAVLink telemetry and relays commands to the autopilot."""

    #: Map each telemetry dataclass to the topic it publishes on.
    _TYPE_TOPIC = {
        Heartbeat: Topics.HEARTBEAT,
        GpsFix: Topics.GPS,
        Attitude: Topics.ATTITUDE,
        Imu: Topics.IMU,
        Battery: Topics.BATTERY,
        Altitude: Topics.ALTITUDE,
        Velocity: Topics.VELOCITY,
        FlightMode: Topics.FLIGHT_MODE,
        ArmedStatus: Topics.ARMED,
        SystemStatus: Topics.SYS_STATUS,
        RcChannels: Topics.RC,
    }

    def __init__(
        self, bus: MessageBus, config: Config, interface: MavlinkInterface
    ) -> None:
        section = config.section("mavlink")
        super().__init__("mavlink", bus, config, rate_hz=section.get("rate_hz", 50))
        self.iface = interface
        self._reconnect_interval = float(section.get("reconnect_interval_s", 2.0))
        self._conn_str = section.get("connection", "")
        self._cmd_queue: deque[NavCommand] = deque()
        self._cmd_lock = threading.Lock()
        # RC-driven payload release. Transmitter channel 9 HIGH -> Release,
        # default LOW -> Lock. Edge-triggered so it never fights the GCS buttons
        # unless the switch actually moves. Lock/Release must match app.js.
        pl = config.section("payload")
        self._payload_out_ch = int(pl.get("out_channel", 9))   # FC output (AUX1)
        self._payload_rc_ch = int(pl.get("rc_channel", 9))     # transmitter input
        self._payload_lock_us = int(pl.get("lock_us", 1100))
        self._payload_release_us = int(pl.get("release_us", 1410))
        self._payload_released: bool | None = None
        self.subscribe(Topics.MAVLINK_CMD, self._on_command)

    def _on_command(self, msg) -> None:
        if isinstance(msg, NavCommand):
            with self._cmd_lock:
                self._cmd_queue.append(msg)

    def on_start(self) -> None:
        self._ensure_connected()

    def _ensure_connected(self) -> bool:
        if self.iface.connected:
            return True
        while not self.stopping:
            if self.iface.connect():
                self.log.info("MAVLink link established")
                self.publish(Topics.LINK, self.iface.link_quality())
                return True
            self.publish(
                Topics.LINK,
                LinkQuality(connected=False, connection_string=self._conn_str),
            )
            self.log.warning(
                "link down - retrying in %.1fs", self._reconnect_interval
            )
            self.sleep(self._reconnect_interval)
        return False

    def step(self) -> None:
        if not self.iface.connected:
            self.publish(
                Topics.LINK,
                LinkQuality(connected=False, connection_string=self._conn_str),
            )
            self._ensure_connected()
            return

        for message in self.iface.receive():
            topic = self._TYPE_TOPIC.get(type(message))
            if topic is not None:
                self.publish(topic, message)
            if isinstance(message, RcChannels):
                self._check_payload_rc(message)

        self._flush_commands()
        self.publish(Topics.LINK, self.iface.link_quality())

    def _check_payload_rc(self, rc: RcChannels) -> None:
        """Drive the payload servo from transmitter RC channel 9.

        Switch HIGH (>1700us) -> Release, default LOW (<1300us) -> Lock, with a
        mid-stick dead-band so switch chatter can't oscillate the drop. Sent only
        on a state change (edge), and safely locks on the first clear LOW read.
        Runs on the node's step thread, same as command flushing - no races.
        """
        ch = self._payload_rc_ch
        if len(rc.channels) < ch:
            return
        us = rc.channels[ch - 1]
        if not us:  # 0 => channel absent / no RC signal
            return
        if us > 1700:
            want_release = True
        elif us < 1300:
            want_release = False
        else:
            return  # dead-band around mid-stick
        if want_release == self._payload_released:
            return
        self._payload_released = want_release
        pwm = self._payload_release_us if want_release else self._payload_lock_us
        self.log.info(
            "RC ch%d=%dus -> payload %s (%dus)",
            ch, us, "RELEASE" if want_release else "LOCK", pwm,
        )
        self.iface.send_command(
            NavCommand("set_servo", {"channel": self._payload_out_ch, "pwm": pwm})
        )

    def _flush_commands(self) -> None:
        while True:
            with self._cmd_lock:
                if not self._cmd_queue:
                    return
                command = self._cmd_queue.popleft()
            accepted = self.iface.send_command(command)
            self.log.info(
                "command '%s' %s",
                command.command,
                "sent" if accepted else "REJECTED",
            )

    def on_stop(self) -> None:
        try:
            self.iface.close()
        except Exception:  # noqa: BLE001
            self.log.exception("error closing MAVLink interface")
