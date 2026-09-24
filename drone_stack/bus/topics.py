"""Canonical topic names.

Using constants instead of raw strings prevents typos and gives a single place
to see every channel in the system (the equivalent of a ROS topic list).
"""
from __future__ import annotations


class Topics:
    """Namespaced topic-name constants."""

    # --- Telemetry (published by MavlinkNode) -------------------------------
    HEARTBEAT = "/telemetry/heartbeat"
    GPS = "/telemetry/gps"
    ATTITUDE = "/telemetry/attitude"
    IMU = "/telemetry/imu"
    BATTERY = "/telemetry/battery"
    ALTITUDE = "/telemetry/altitude"
    VELOCITY = "/telemetry/velocity"
    FLIGHT_MODE = "/telemetry/mode"
    ARMED = "/telemetry/armed"
    SYS_STATUS = "/telemetry/sys_status"
    RC = "/telemetry/rc"
    LINK = "/telemetry/link"

    # --- LiDAR (published by LidarNode) -------------------------------------
    SCAN = "/scan"
    CLOUD = "/cloud"

    # --- Perception / state -------------------------------------------------
    FUSED_STATE = "/state/fused"
    OBSTACLES = "/obstacles"
    AVOIDANCE = "/avoidance"
    SCAN_STATE = "/scan/state"

    # --- Navigation ---------------------------------------------------------
    MISSION_STATE = "/mission/state"
    MISSION_PLAN = "/mission/plan"        # the loaded Mission (waypoints)
    MISSION_CMD = "/mission/cmd"          # request a mission action
    MAVLINK_CMD = "/cmd/mavlink"          # NavCommand -> MavlinkNode
    MISSION_UPLOAD = "/mission/upload"    # result of an FC mission upload
    FC_MESSAGE = "/telemetry/fc_message"  # autopilot STATUSTEXT (PreArm, EKF...)

    # --- Firebase parcel delivery -------------------------------------------
    DELIVERY_ORDER = "/delivery/order"    # DeliveryOrder pulled from Firebase
    DELIVERY_STATE = "/delivery/state"    # DeliveryState for the GCS panel
    # DeliveryBleResult, bridged from drone_ble_peripheral.py's drop gates
    # (HMAC auth + geofence + DROP HMAC) via GcsHub. An event, not a standing
    # state - see EVENT_TOPICS below - so a node recreated mid-flight cannot
    # replay a previous order's stale "delivered" signal into a new hold.
    DELIVERY_BLE_RESULT = "/delivery/ble_result"
    # BlePhoneSignal: RSSI samples of the recipient's live BLE connection and
    # the phone's own GPS fixes, bridged from the peripheral like the result
    # above. Samples, not state - never latched, so a recreated navigator
    # cannot be handed the previous order's phone.
    BLE_PHONE = "/delivery/ble_phone"

    # --- Vision --------------------------------------------------------------
    PERSON_LOCK = "/vision/person_lock"   # PersonLockState, camera -> navigator
    PHONE_HINT = "/vision/phone_hint"     # PhoneHint, navigator -> camera

    # --- Command topics ------------------------------------------------------
    # A request to *act*, not a state to observe. The bus refuses to latch
    # these (see MessageBus.publish), so a subscriber that attaches after the
    # fact can never be handed a stale command and carry it out. That matters
    # because Supervisor recreates a dead node by calling its constructor
    # again, which re-subscribes: with latching left on, a recreated
    # MavlinkNode replayed the last command it ever saw - an 'arm' or
    # 'takeoff' spinning the props back up with nobody asking.
    #
    # Any new topic carrying an instruction belongs in here.
    EVENT_TOPICS = frozenset({MISSION_CMD, MAVLINK_CMD, DELIVERY_BLE_RESULT, BLE_PHONE})

    # --- Diagnostics --------------------------------------------------------
    DIAGNOSTICS = "/diagnostics"
    DIAG_NODES = "/diagnostics/nodes"
    DIAG_LIDAR = "/diagnostics/lidar"
    DIAG_MAVLINK = "/diagnostics/mavlink"

    @classmethod
    def all(cls) -> list[str]:
        """Return every declared topic name (useful for introspection/tests)."""
        return [
            v for k, v in vars(cls).items()
            if not k.startswith("_") and isinstance(v, str)
        ]
