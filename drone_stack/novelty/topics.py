"""Novelty-layer bus topics.

Kept separate from :class:`drone_stack.bus.topics.Topics` deliberately - the
novelty layer is additive and optional (``novelty.enabled: false`` by
default), so it gets its own namespace instead of growing the core topic
list. Commands still flow back into the core stack through the *existing*
``Topics.MISSION_CMD`` / ``Topics.MAVLINK_CMD`` channels (see
``drone_stack/novelty/delivery_node.py``).
"""
from __future__ import annotations


class NoveltyTopics:
    """Namespaced topic-name constants for the AERIX novelty layer."""

    # --- Perception (published by the camera pipeline, see gcs/cameras.py) --
    PERSON_DETECTIONS = "/novelty/perception/persons"      # list[PersonDetection]
    TERRAIN_MAP = "/novelty/perception/terrain"             # TerrainMap

    # --- Recipient auth (published by ble_handshake + recipient_auth.py) ----
    BLE_AUTH_EVENT = "/novelty/auth/ble_event"               # BleAuthEvent
    AUTH_DECISION = "/novelty/auth/decision"                 # AuthDecision

    # --- Landing zone selection ---------------------------------------------
    ZONE_CANDIDATES = "/novelty/landing/candidates"          # list[ZoneCandidate]

    # --- Descent safety -------------------------------------------------------
    MOTION_ABORT = "/novelty/motion/abort"                   # MotionAbortEvent

    # --- Mission FSM ---------------------------------------------------------
    MISSION_FSM_STATE = "/novelty/fsm/state"                 # FsmStateSnapshot
    MISSION_FSM_EVENT = "/novelty/fsm/event"                 # any Event subtype

    @classmethod
    def all(cls) -> list[str]:
        """Return every declared topic name (introspection/tests)."""
        return [
            v for k, v in vars(cls).items()
            if not k.startswith("_") and isinstance(v, str)
        ]
