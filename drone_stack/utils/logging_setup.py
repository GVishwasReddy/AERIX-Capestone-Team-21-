"""Central logging configuration.

``get_logger`` works even before ``setup_logging`` is called (a NullHandler is
attached to the package logger at import), so nodes constructed in unit tests
never emit "no handlers" warnings.
"""
from __future__ import annotations

import logging
import logging.handlers
import os
from typing import Any

_ROOT_NAME = "drone"
_configured = False

# Ensure the package logger always has a handler.
logging.getLogger(_ROOT_NAME).addHandler(logging.NullHandler())


def get_logger(name: str) -> logging.Logger:
    """Return a namespaced logger, e.g. ``drone.mavlink``."""
    if name.startswith(_ROOT_NAME + ".") or name == _ROOT_NAME:
        return logging.getLogger(name)
    return logging.getLogger(f"{_ROOT_NAME}.{name}")


def setup_logging(config: Any = None) -> logging.Logger:
    """Configure console + rotating-file logging from a config section.

    Accepts either a :class:`~drone_stack.utils.config.Config`, a plain dict
    (the ``logging`` section), or ``None`` for defaults. Idempotent.
    """
    global _configured

    section: dict = {}
    if config is not None:
        # Accept a full Config or just the logging dict.
        if hasattr(config, "section"):
            section = config.section("logging")
        elif isinstance(config, dict):
            section = config.get("logging", config)

    level_name = str(section.get("level", "INFO")).upper()
    level = getattr(logging, level_name, logging.INFO)
    log_dir = section.get("dir", "logs")
    console = section.get("console", True)
    max_bytes = int(section.get("max_bytes", 5 * 1024 * 1024))
    backups = int(section.get("backup_count", 3))

    root = logging.getLogger(_ROOT_NAME)
    root.setLevel(level)

    # Remove previously installed handlers (keep it idempotent across reloads),
    # but retain a NullHandler as a safety net.
    for handler in list(root.handlers):
        root.removeHandler(handler)
    root.addHandler(logging.NullHandler())

    fmt = logging.Formatter(
        "%(asctime)s %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    if console:
        stream = logging.StreamHandler()
        stream.setFormatter(fmt)
        stream.setLevel(level)
        root.addHandler(stream)

    try:
        os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(
            os.path.join(log_dir, "drone_stack.log"),
            maxBytes=max_bytes,
            backupCount=backups,
        )
        file_handler.setFormatter(fmt)
        file_handler.setLevel(level)
        root.addHandler(file_handler)
    except OSError as exc:  # e.g. read-only filesystem - keep console logging
        root.warning("could not open log file in %s: %s", log_dir, exc)

    root.propagate = False
    _configured = True
    return root
