"""Exception types shared across the stack.

Kept in their own dependency-free module so the state machine (and its tests)
never need MAVSDK or firebase-admin importable just to catch an error.
"""
from __future__ import annotations


class FlightAbort(Exception):
    """Raised when a failsafe forces the mission to end early."""


class PreArmCheckFailed(Exception):
    """Raised when the vehicle is not fit to fly."""
