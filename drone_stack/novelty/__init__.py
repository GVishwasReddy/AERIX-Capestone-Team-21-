"""AERIX novelty layer - the patentable decision layer above the flight stack.

Everything in this package sits ABOVE ``drone_stack.nodes`` / the MAVLink
interface and only ever talks to it through the same two seams every other
front-end uses:

* ``NavCommand`` published on ``Topics.MISSION_CMD``  -> invokes a
  ``NavigationNode`` service (``hold``, ``rtl``, ``land``, ...).
* ``NavCommand("set_servo", ...)`` published on ``Topics.MAVLINK_CMD``  ->
  drives the payload servo, identical to the existing GCS "PAYLOAD" buttons.

No module in this package imports PX4/ArduPilot parameters, the low-level
MAVLink interface, or the avoidance/RTL logic directly - see
``docs/novelty/*.md`` for the formulas and prior-art notes behind each
module, and ``CLAUDE.md`` / ``docs/ARCHITECTURE.md`` for how this package
attaches to the rest of the stack.
"""
