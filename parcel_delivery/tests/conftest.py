"""Make the pi/ modules importable from the tests without installing a package."""
import os
import sys

PI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pi")

if PI_DIR not in sys.path:
    sys.path.insert(0, PI_DIR)
