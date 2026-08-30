"""LidarNode - Phase 3.

Reads full scans from a :class:`~drone_stack.interfaces.lidar_interface.LidarInterface`
(real RPLIDAR or mock), and publishes:

* ``/scan``  - :class:`~drone_stack.msg.LaserScan`
* ``/cloud`` - :class:`~drone_stack.msg.PointCloud` (Cartesian, sensor frame)
* ``/diagnostics/lidar`` - :class:`~drone_stack.msg.Diagnostic`

Automatically reconnects if the device is unplugged.
"""
from __future__ import annotations

import math

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.interfaces.lidar_interface import FovMask, LidarInterface
from drone_stack.msg import Diagnostic, DiagLevel, LaserScan, PointCloud, ScanState
from drone_stack.srv import ServiceRegistry, ServiceRequest, ServiceResponse
from drone_stack.utils.config import Config
from drone_stack.utils.geometry import polar_to_cartesian
from drone_stack.utils.node import NodeBase


class LidarNode(NodeBase):
    """Publishes LaserScan + PointCloud + diagnostics from the RPLIDAR.

    Exposes a scan state machine (STOPPED / SCANNING / PAUSED) via services so
    the GCS Start/Stop/Pause/Resume buttons work. When not SCANNING the device
    stays connected but no scans are published.
    """

    def __init__(
        self,
        bus: MessageBus,
        config: Config,
        interface: LidarInterface,
        services: ServiceRegistry | None = None,
    ) -> None:
        section = config.section("lidar")
        super().__init__("lidar", bus, config, rate_hz=section.get("rate_hz", 10))
        self.iface = interface
        self.services = services or ServiceRegistry()
        self._reconnect_interval = float(section.get("reconnect_interval_s", 2.0))
        self._port = section.get("port", "")
        # Reported to the GCS so the radar can draw the front line and the
        # ignored rear wedge from the same numbers the driver masks with.
        self.fov = FovMask(section)
        self._scan_count = 0
        self._empty_reads = 0
        self._scan_state = ScanState.SCANNING
        self.services.register("scan_start", self._svc_start)
        self.services.register("scan_stop", self._svc_stop)
        self.services.register("scan_pause", self._svc_pause)
        self.services.register("scan_resume", self._svc_resume)

    def _svc_start(self, req: ServiceRequest) -> ServiceResponse:
        self._scan_state = ScanState.SCANNING
        return ServiceResponse(True, "scanning")

    def _svc_stop(self, req: ServiceRequest) -> ServiceResponse:
        self._scan_state = ScanState.STOPPED
        return ServiceResponse(True, "scan stopped")

    def _svc_pause(self, req: ServiceRequest) -> ServiceResponse:
        if self._scan_state == ScanState.SCANNING:
            self._scan_state = ScanState.PAUSED
        return ServiceResponse(True, "scan paused")

    def _svc_resume(self, req: ServiceRequest) -> ServiceResponse:
        if self._scan_state == ScanState.PAUSED:
            self._scan_state = ScanState.SCANNING
        return ServiceResponse(True, "scan resumed")

    def on_start(self) -> None:
        self._ensure_connected()

    def _ensure_connected(self) -> bool:
        if self.iface.connected:
            return True
        while not self.stopping:
            if self.iface.connect():
                self.log.info("RPLIDAR connected")
                return True
            self._publish_diag(
                DiagLevel.ERROR, f"LiDAR not connected ({self._port})"
            )
            self.log.warning(
                "LiDAR unavailable - retrying in %.1fs", self._reconnect_interval
            )
            self.sleep(self._reconnect_interval)
        return False

    def step(self) -> None:
        self.publish(Topics.SCAN_STATE, self._scan_state)
        if self._scan_state != ScanState.SCANNING:
            self._publish_diag(DiagLevel.OK, self._scan_state.value.lower())
            return

        if not self.iface.connected:
            self._publish_diag(DiagLevel.ERROR, "LiDAR disconnected")
            self._ensure_connected()
            return

        scan = self.iface.read_scan()
        if scan is None:
            self._empty_reads += 1
            if not self.iface.connected:
                self._publish_diag(DiagLevel.ERROR, "LiDAR link lost - reconnecting")
                self._ensure_connected()
            else:
                self._publish_diag(DiagLevel.WARN, "empty scan")
            return

        self._scan_count += 1
        self.publish(Topics.SCAN, scan)
        self.publish(Topics.CLOUD, self._to_cloud(scan))
        valid = sum(1 for r in scan.ranges if math.isfinite(r))
        self._publish_diag(
            DiagLevel.OK,
            "scanning",
            beams=scan.count,
            valid_returns=valid,
            scans=self._scan_count,
            fov_deg=self.fov.fov_deg,
            blind_deg=self.fov.blind_deg,
        )

    @staticmethod
    def _to_cloud(scan: LaserScan) -> PointCloud:
        points: list[tuple[float, float, float]] = []
        angle = scan.angle_min
        for rng in scan.ranges:
            if math.isfinite(rng):
                x, y = polar_to_cartesian(rng, angle)
                points.append((x, y, 0.0))
            angle += scan.angle_increment
        return PointCloud(frame_id=scan.frame_id, points=points)

    def _publish_diag(self, level: DiagLevel, message: str, **values) -> None:
        self.publish(
            Topics.DIAG_LIDAR,
            Diagnostic(name="lidar", level=level, message=message, values=values),
        )
