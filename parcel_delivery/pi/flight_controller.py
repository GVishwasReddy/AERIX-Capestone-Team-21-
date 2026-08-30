"""MAVSDK-based flight control: pre-arm checks, takeoff, offboard goto,
hover, RTL — plus a supervisory failsafe task that can abort at any point.

Safety notes:
  * Every mission runs with a failsafe supervisor task alive alongside it.
    Battery, geofence and hover-timeout breaches raise FlightAbort into the
    mission, which forces an RTL.
  * This module never depends on the Pixhawk trusting the Pi: ArduPilot's own
    RC/GCS failsafes remain the last line of defence if this process dies.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionGlobalYaw

from exceptions import FlightAbort, PreArmCheckFailed
from mission_validator import haversine_distance_m

logger = logging.getLogger("flight_controller")

__all__ = ["FlightController", "FlightAbort", "PreArmCheckFailed"]


class FlightController:
    def __init__(
        self,
        connection: str,
        home_lat: float,
        home_lon: float,
        geofence_radius_m: float,
        arrival_tolerance_m: float,
        battery_failsafe_pct: float,
        default_hover_altitude_m: float,
        telemetry_interval_s: float = 1.5,
    ) -> None:
        self.connection = connection
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.geofence_radius_m = geofence_radius_m
        self.arrival_tolerance_m = arrival_tolerance_m
        self.battery_failsafe_pct = battery_failsafe_pct
        self.default_hover_altitude_m = default_hover_altitude_m
        self.telemetry_interval_s = telemetry_interval_s

        self.drone = System()
        self._connected = False
        self._delivery_confirmed = asyncio.Event()
        self._abort_reason: str | None = None
        self._failsafe_task: asyncio.Task | None = None

    # ------------------------------------------------------------------ setup

    async def connect(self) -> None:
        logger.info("connecting to vehicle at %s", self.connection)
        await self.drone.connect(system_address=self.connection)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("vehicle connected")
                self._connected = True
                break

    async def preflight_checks(self) -> None:
        """Abort before arming if GPS/EKF/battery are not healthy."""
        async for health in self.drone.telemetry.health():
            problems = []
            if not health.is_global_position_ok:
                problems.append("no global position estimate")
            if not health.is_home_position_ok:
                problems.append("home position not set")
            if not health.is_gyrometer_calibration_ok:
                problems.append("gyro not calibrated")
            if not health.is_accelerometer_calibration_ok:
                problems.append("accelerometer not calibrated")
            if problems:
                raise PreArmCheckFailed("; ".join(problems))
            break

        async for battery in self.drone.telemetry.battery():
            pct = battery.remaining_percent * 100.0
            if pct < self.battery_failsafe_pct:
                raise PreArmCheckFailed(
                    f"battery {pct:.0f}% below failsafe threshold {self.battery_failsafe_pct:.0f}%"
                )
            break

        logger.info("preflight checks passed")

    # ------------------------------------------------------------- mission ops

    async def arm_and_takeoff(self, altitude_m: float) -> None:
        await self.preflight_checks()
        self._delivery_confirmed.clear()
        self._abort_reason = None

        logger.info("arming")
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(altitude_m)
        logger.info("taking off to %.1fm", altitude_m)
        await self.drone.action.takeoff()

        # Wait until we are within 0.5m of the requested takeoff altitude.
        async for position in self.drone.telemetry.position():
            self._raise_if_aborted()
            if position.relative_altitude_m >= altitude_m - 0.5:
                break
            await asyncio.sleep(0)

        self._failsafe_task = asyncio.create_task(self._failsafe_supervisor())
        logger.info("takeoff complete")

    async def fly_to(self, lat: float, lon: float, altitude_m: float) -> None:
        """Fly to the target in offboard mode and stop on arrival.

        On arrival we simply stop sending setpoints — offboard holds the last
        commanded position, which is the hover.
        """
        setpoint = PositionGlobalYaw(
            lat_deg=lat,
            lon_deg=lon,
            alt_m=altitude_m,
            yaw_deg=0.0,
            altitude_type=PositionGlobalYaw.AltitudeType.REL_HOME,
        )
        await self.drone.offboard.set_position_global(setpoint)

        try:
            await self.drone.offboard.start()
        except OffboardError as exc:
            raise FlightAbort(f"offboard start failed: {exc._result.result}") from exc

        logger.info("enroute to %.6f, %.6f at %.1fm", lat, lon, altitude_m)

        async for position in self.drone.telemetry.position():
            self._raise_if_aborted()
            distance = haversine_distance_m(
                position.latitude_deg, position.longitude_deg, lat, lon
            )
            if distance <= self.arrival_tolerance_m:
                logger.info("arrived (%.1fm from target), holding position", distance)
                return
            # Keep the setpoint fresh so offboard does not time out.
            await self.drone.offboard.set_position_global(setpoint)
            await asyncio.sleep(0.2)

    async def await_delivery_confirmation(self, timeout_s: float) -> bool:
        """Block until the payload module signals completion, or timeout.

        Returns True if confirmed, False if the hover timeout elapsed first.
        """
        logger.info("awaiting delivery confirmation (timeout %.0fs)", timeout_s)
        try:
            await asyncio.wait_for(self._delivery_confirmed.wait(), timeout=timeout_s)
            logger.info("delivery confirmed")
            return True
        except asyncio.TimeoutError:
            logger.warning("hover timeout elapsed with no confirmation — returning home")
            return False

    def confirm_delivery(self) -> None:
        """Called by the payload controller once the parcel is released."""
        self._delivery_confirmed.set()

    async def return_to_launch(self) -> None:
        try:
            await self.drone.offboard.stop()
        except OffboardError:
            logger.warning("offboard already stopped")

        logger.info("returning to launch")
        await self.drone.action.return_to_launch()

        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                logger.info("landed")
                break

        if self._failsafe_task is not None:
            self._failsafe_task.cancel()
            self._failsafe_task = None

    # ------------------------------------------------------------- telemetry

    async def current_telemetry(self) -> dict[str, Any]:
        telemetry: dict[str, Any] = {}
        async for position in self.drone.telemetry.position():
            telemetry.update(
                lat=position.latitude_deg,
                lon=position.longitude_deg,
                alt_m=position.relative_altitude_m,
            )
            break
        async for battery in self.drone.telemetry.battery():
            telemetry["battery_pct"] = round(battery.remaining_percent * 100.0)
            break
        async for gps_info in self.drone.telemetry.gps_info():
            telemetry["gps_fix"] = str(gps_info.fix_type)
            break
        async for heading in self.drone.telemetry.heading():
            telemetry["heading"] = heading.heading_deg
            break
        return telemetry

    async def stream_telemetry(self, delivery_id: str, firebase_client) -> None:
        """Push a telemetry snapshot to Firebase every telemetry_interval_s."""
        while True:
            try:
                telemetry = await self.current_telemetry()
                await firebase_client.push_telemetry(delivery_id, telemetry)
                logger.debug("telemetry %s", telemetry)
            except asyncio.CancelledError:
                raise
            except Exception:  # noqa: BLE001 - telemetry must never kill a flight
                logger.exception("telemetry push failed")
            await asyncio.sleep(self.telemetry_interval_s)

    # -------------------------------------------------------------- failsafes

    def _raise_if_aborted(self) -> None:
        if self._abort_reason is not None:
            raise FlightAbort(self._abort_reason)

    async def _failsafe_supervisor(self) -> None:
        """Watch battery and geofence continuously, regardless of mission phase."""
        try:
            while True:
                async for battery in self.drone.telemetry.battery():
                    pct = battery.remaining_percent * 100.0
                    if pct < self.battery_failsafe_pct:
                        self._trigger_abort(
                            f"battery {pct:.0f}% below failsafe {self.battery_failsafe_pct:.0f}%"
                        )
                    break

                async for position in self.drone.telemetry.position():
                    distance = haversine_distance_m(
                        position.latitude_deg,
                        position.longitude_deg,
                        self.home_lat,
                        self.home_lon,
                    )
                    if distance > self.geofence_radius_m:
                        self._trigger_abort(
                            f"geofence breach: {distance:.0f}m from home "
                            f"(limit {self.geofence_radius_m:.0f}m)"
                        )
                    break

                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001
            logger.exception("failsafe supervisor error")

    def _trigger_abort(self, reason: str) -> None:
        if self._abort_reason is None:
            logger.error("FAILSAFE: %s", reason)
            self._abort_reason = reason
            # Unblock anything waiting on delivery confirmation.
            self._delivery_confirmed.set()
