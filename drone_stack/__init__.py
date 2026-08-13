"""drone_stack - ROS-free autonomous drone software stack.

A lightweight in-process publish/subscribe framework (see :mod:`drone_stack.bus`)
hosts a set of cooperating nodes (:mod:`drone_stack.nodes`) that talk to a
Pixhawk over MAVLink and a Slamtec RPLIDAR over serial, and run identically in
simulation (:mod:`drone_stack.sim`) or against real hardware.
"""

__version__ = "1.0.0"
