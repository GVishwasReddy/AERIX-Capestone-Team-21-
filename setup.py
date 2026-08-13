"""Thin shim so `python setup.py` and editable installs keep working.

All real metadata lives in ``pyproject.toml`` / ``setup.cfg``.
"""
from setuptools import setup

if __name__ == "__main__":
    setup()
