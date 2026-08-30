"""Hardware interfaces (MAVLink + LiDAR), each with an abstract base.

The concrete *real* implementations live here; the *mock* implementations live
in :mod:`drone_stack.sim`. Both satisfy the same abstract base class, so nodes
are identical in simulation and on real hardware.
"""
from drone_stack.interfaces.firebase_interface import (
    FirebaseInterface,
    FileOrders,
    FirestoreOrders,
    NullOrders,
    build_order_source,
)
from drone_stack.interfaces.lidar_interface import LidarInterface, RealLidar
from drone_stack.interfaces.mavlink_interface import MavlinkInterface, RealMavlink

__all__ = [
    "MavlinkInterface",
    "RealMavlink",
    "LidarInterface",
    "RealLidar",
    "FirebaseInterface",
    "FirestoreOrders",
    "FileOrders",
    "NullOrders",
    "build_order_source",
]
