"""Bringup - the single entrypoint that launches the whole stack.

Usage::

    python -m drone_stack.launch.bringup --config config/sim.yaml
    python -m drone_stack.launch.bringup --config config/real.yaml

Which nodes start is controlled by the ``enabled`` flags in the config. In
simulation a shared :class:`~drone_stack.sim.world.SimWorld` backs both the mock
Pixhawk and mock LiDAR.
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading

from drone_stack.bus import MessageBus
from drone_stack.launch.builders import (
    build_lidar_interface,
    build_mavlink_interface,
    build_world,
)
from drone_stack.nodes import (
    DiagnosticsNode,
    FirebaseDeliveryNode,
    FusionNode,
    LidarNode,
    MavlinkNode,
    NavigationNode,
    ObstacleNode,
    ProximityNode,
)
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config
from drone_stack.utils.logging_setup import get_logger, setup_logging
from drone_stack.utils.node import Supervisor
from drone_stack.web import WebDashboard


def build_supervisor(
    config: Config,
    bus: MessageBus | None = None,
    services: ServiceRegistry | None = None,
    include_web: bool = True,
) -> Supervisor:
    """Construct the bus, services and all enabled nodes.

    ``bus``/``services`` may be supplied so an external front-end (e.g. the GCS)
    shares the same bus. ``include_web`` disables the built-in Flask dashboard
    node (the GCS provides its own UI).
    """
    log = get_logger("bringup")
    bus = bus if bus is not None else MessageBus()
    services = services if services is not None else ServiceRegistry()
    supervisor = Supervisor(bus)

    world = build_world(config) if config.mode == "sim" else None
    log.info("bringup in '%s' mode", config.mode)

    if config.get("mavlink.enabled", True):
        mav_iface = build_mavlink_interface(config, world)
        supervisor.add(lambda: MavlinkNode(bus, config, mav_iface))

    if config.get("lidar.enabled", True):
        lidar_iface = build_lidar_interface(config, world)
        supervisor.add(lambda: LidarNode(bus, config, lidar_iface, services))

    if config.get("fusion.enabled", True):
        supervisor.add(lambda: FusionNode(bus, config))

    if config.get("obstacles.enabled", True):
        supervisor.add(lambda: ObstacleNode(bus, config))

    if config.get("proximity.enabled", False):
        # Started before the navigator: this is the avoidance that keeps
        # working when the pilot takes the sticks and the navigator stands
        # down. It only feeds the FC's proximity library - it issues no flight
        # commands and never changes mode.
        supervisor.add(lambda: ProximityNode(bus, config))

    if config.get("navigation.enabled", True):
        supervisor.add(lambda: NavigationNode(bus, config, services))

    if config.get("diagnostics.enabled", True):
        supervisor.add(lambda: DiagnosticsNode(bus, config))

    if config.get("delivery.enabled", True):
        # Started after NavigationNode because it drives the aircraft entirely
        # through that node's services (set_delivery_target / start_mission),
        # so the flight-safety rules apply to a Firebase order exactly as they
        # do to anything else. Its order source degrades to "no-credentials"
        # rather than failing, so a Pi without a Firebase key still boots.
        supervisor.add(lambda: FirebaseDeliveryNode(bus, config, services))

    if config.get("novelty.enabled", False):
        # Lazily imported (unlike every node above) so a deployment that
        # never enables the novelty layer pays no import cost for its extra
        # dependencies (pydantic, numpy-backed perception types) - matches
        # the "additive, optional, off by default" design in the project
        # plan. See drone_stack/novelty/delivery_node.py's own docstring.
        from drone_stack.novelty.delivery_node import DeliveryNode

        supervisor.add(lambda: DeliveryNode(bus, config))

    if include_web and config.get("web.enabled", True):
        supervisor.add(lambda: WebDashboard(bus, config, services))

    return supervisor


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the drone_stack.")
    parser.add_argument(
        "--config",
        "-c",
        default=None,
        help="Path to a profile YAML (e.g. config/sim.yaml). "
        "Defaults to the built-in default.yaml (simulation).",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    config = Config.load(profile_path=args.config)
    setup_logging(config)
    log = get_logger("bringup")

    supervisor = build_supervisor(config)

    stop_event = threading.Event()

    def _handle_signal(signum, _frame):
        log.info("received signal %s - shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    supervisor.start()
    log.info("stack running - press Ctrl-C to stop")
    try:
        while not stop_event.wait(0.5):
            pass
    finally:
        supervisor.stop()
    log.info("bye")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
