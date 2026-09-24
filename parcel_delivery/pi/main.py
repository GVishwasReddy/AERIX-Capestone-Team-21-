"""Entry point: wires Firebase, the flight controller, the lidar bridge and
the state machine together.

Usage:
    python main.py                 # uses .env in this directory
    python main.py --env path.env
    python main.py --no-lidar      # skip the lidar bridge (SITL without hardware)
"""
from __future__ import annotations

import argparse
import asyncio
import logging
from typing import Any

from config import load_config
from firebase_client import FirebaseClient
from flight_controller import FlightController
from lidar_bridge import LidarBridge
from logging_setup import setup_logging
from payload_controller import PayloadController
from state_machine import MissionStateMachine

logger = logging.getLogger("main")


class DeliveryService:
    def __init__(self, config, enable_lidar: bool = True) -> None:
        self.config = config
        self.enable_lidar = enable_lidar

        self.firebase = FirebaseClient(
            credentials_path=config.firebase_credentials_path,
            database_url=config.firebase_db_url,
        )
        self.flight = FlightController(
            connection=config.mavlink_connection,
            home_lat=config.home_lat,
            home_lon=config.home_lon,
            geofence_radius_m=config.geofence_radius_m,
            arrival_tolerance_m=config.arrival_tolerance_m,
            battery_failsafe_pct=config.battery_failsafe_pct,
            default_hover_altitude_m=config.hover_altitude_m,
        )
        self.payload = PayloadController(flight_controller=self.flight)
        self.lidar: LidarBridge | None = None

        # Only one mission at a time — the drone is a single physical vehicle.
        self._mission_lock = asyncio.Lock()

    async def handle_pending_delivery(self, delivery_id: str, doc: dict[str, Any]) -> None:
        if self._mission_lock.locked():
            logger.warning(
                "delivery %s arrived while another mission is in progress — rejecting",
                delivery_id,
            )
            await self.firebase.push_status(
                delivery_id,
                "error",
                error_message="another delivery is already in progress",
            )
            return

        async with self._mission_lock:
            logger.info("starting delivery %s", delivery_id)
            machine = MissionStateMachine(
                delivery_id=delivery_id,
                delivery_doc=doc,
                firebase_client=self.firebase,
                flight_controller=self.flight,
                payload_controller=self.payload,
                home_lat=self.config.home_lat,
                home_lon=self.config.home_lon,
                geofence_radius_m=self.config.geofence_radius_m,
                min_altitude_m=self.config.min_altitude_m,
                max_altitude_m=self.config.max_altitude_m,
                hover_timeout_s=self.config.hover_timeout_s,
            )
            await machine.run()
            logger.info("delivery %s finished in state %s", delivery_id, machine.state)

    async def run(self) -> None:
        await self.flight.connect()

        if self.enable_lidar:
            self.lidar = LidarBridge(
                mavlink_connection=self.config.lidar_mavlink_connection,
                lidar_port=self.config.lidar_port,
                lidar_baud=self.config.lidar_baud,
                fov_enabled=self.config.lidar_fov_enabled,
                fov_deg=self.config.lidar_fov_deg,
            )
            try:
                self.lidar.start()
            except Exception:  # noqa: BLE001 - a missing lidar must not ground the stack
                logger.exception("lidar bridge failed to start — continuing without it")
                self.lidar = None
        else:
            logger.info("lidar bridge disabled by flag")

        loop = asyncio.get_running_loop()
        self.firebase.start_listener(self.handle_pending_delivery, loop)
        logger.info("listening for pending deliveries")

        try:
            while True:
                await asyncio.sleep(3600)
        except asyncio.CancelledError:
            pass
        finally:
            self.firebase.stop_listener()
            if self.lidar is not None:
                self.lidar.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env", default=None, help="path to the .env file")
    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="do not start the lidar bridge (useful for SITL with no hardware)",
    )
    return parser.parse_args()


async def main_async(args: argparse.Namespace) -> None:
    config = load_config(args.env)
    setup_logging(config.log_dir, config.log_level)
    logger.info("configuration loaded, connection=%s", config.mavlink_connection)

    service = DeliveryService(config, enable_lidar=not args.no_lidar)
    await service.run()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("shutting down")


if __name__ == "__main__":
    main()
