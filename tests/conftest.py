"""Shared pytest fixtures."""
from __future__ import annotations

import pytest

from drone_stack.bus import MessageBus
from drone_stack.utils.config import Config


@pytest.fixture
def config() -> Config:
    """The built-in default configuration (simulation mode)."""
    return Config.load()


@pytest.fixture
def bus() -> MessageBus:
    return MessageBus()


@pytest.fixture(autouse=True)
def _isolated_recordings(tmp_path_factory, monkeypatch) -> None:
    """Keep every test away from the aircraft's real recordings/.

    Tests build GcsHub(Config.load()), and its FlightRecorder resolves the
    repo-relative "recordings" dir - on the Pi, the live one. Constructing a
    recorder sweeps that dir and adopts its owed renders, so a pytest run in
    ~/drone_stack on 2026-09-23 deleted the running HD render's temp file and
    the frames of the flight it was rendering. Config.load() honours
    DRONE_<SECTION>_<KEY>, so this reaches every hub a test builds.
    """
    monkeypatch.setenv("DRONE_RECORDING_DIR",
                       str(tmp_path_factory.mktemp("recordings")))
