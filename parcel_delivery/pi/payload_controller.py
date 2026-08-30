"""Payload release interface.

The actual release mechanism is hardware-specific (servo/gripper on a GPIO pin,
or a Pixhawk servo output). This module keeps that behind a clean async
interface so the state machine never needs to know which it is.
"""
from __future__ import annotations

import asyncio
import logging

logger = logging.getLogger("payload_controller")


class PayloadController:
    def __init__(self, flight_controller=None, release_duration_s: float = 2.0) -> None:
        # flight_controller is optional: when present, a successful release
        # signals delivery confirmation back to the flight controller.
        self.flight_controller = flight_controller
        self.release_duration_s = release_duration_s

    async def precision_locate(self) -> tuple[float, float] | None:
        """Camera-based precision landing / marker detection.

        TODO(hardware): run the marker detector on the downward camera and
        return a (north_m, east_m) offset from the current position to the
        detected drop marker, or None if no marker was found. Until that is
        implemented we return None and the caller drops at the hover position.
        """
        logger.info("precision_locate: not implemented, using hover position")
        return None

    async def release_payload(self, location: tuple[float, float] | None = None) -> bool:
        """Actuate the release mechanism. Returns True on success.

        TODO(hardware): drive the servo/gripper. On this airframe the payload
        servo is on a Pixhawk AUX output, so the real implementation will send
        a MAV_CMD_DO_SET_SERVO rather than toggling a Pi GPIO pin.
        """
        if location is not None:
            logger.info("releasing payload at offset %s", location)
        else:
            logger.info("releasing payload at hover position")

        # Simulated actuation time for the stub.
        await asyncio.sleep(self.release_duration_s)

        logger.info("payload released")
        if self.flight_controller is not None:
            self.flight_controller.confirm_delivery()
        return True
