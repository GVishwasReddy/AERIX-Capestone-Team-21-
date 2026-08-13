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
