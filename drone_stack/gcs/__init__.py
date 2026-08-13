"""Drone GCS - a FastAPI + WebSocket ground control station.

Reuses the tested :mod:`drone_stack` engine (MessageBus, nodes, sim world,
MAVLink/LiDAR interfaces, obstacle detection, navigation, NL parser) and adds a
real-time web UI on top. See :mod:`drone_stack.gcs.server`.
"""
from drone_stack.gcs.hub import GcsHub

__all__ = ["GcsHub"]
