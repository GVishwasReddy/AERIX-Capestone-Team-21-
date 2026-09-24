"""No test may touch the aircraft's real recordings/ (see conftest.py)."""
from __future__ import annotations

from pathlib import Path

from drone_stack.utils.config import Config


def test_the_test_suite_never_records_into_the_real_recordings_dir():
    live = (Path(__file__).resolve().parents[1] / "recordings").resolve()
    used = Path(Config.load().section("recording")["dir"]).resolve()
    assert used != live, "a test would sweep and adopt the aircraft's real flights"
