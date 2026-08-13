"""§3 - flight-log evidence for the novelty layer.

Every decision the novelty layer makes is written as one JSON line to a
per-flight file:

    flight_logs/<ISO8601Z>_<git-short-hash>.jsonl

Each line is:
    {event_name, timestamp_utc, mission_state, inputs, computed_values,
     threshold, decision, model_versions}

This is reduction-to-practice evidence for a patent filing, so the schema is
fixed and every field listed above is always present (never omitted, never
renamed) - see docs/novelty/*.md "Test evidence" sections for how each
module's decisions map onto these fields.

Design note - logger failures do NOT abort a flight. This is the one place
in the novelty layer that deliberately does NOT follow "log and escalate to
ABORT_RTL" (see brief §7): failing to *write a log line* is not evidence
that a *decision* is unsafe, and turning a disk-full or read-only-filesystem
condition into a forced abort would make the evidence logger itself a
flight-safety hazard. A write failure is logged via the standard `logging`
module and otherwise swallowed.
"""
from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from drone_stack.novelty.types import to_jsonable
from drone_stack.utils.logging_setup import get_logger

_log = get_logger("novelty.evidence")


def _git_short_hash(repo_dir: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_dir,
            capture_output=True,
            text=True,
            timeout=2.0,
            check=True,
        )
        return out.stdout.strip() or "nogit"
    except Exception as exc:  # noqa: BLE001 - never block a flight on git
        _log.warning("evidence_logger: could not read git hash (%s) - using 'nogit'", exc)
        return "nogit"


def _iso_stamp(t: float | None = None) -> str:
    dt = datetime.fromtimestamp(t if t is not None else time.time(), tz=timezone.utc)
    return dt.strftime("%Y%m%dT%H%M%SZ")


@dataclass
class _DecisionCounts:
    total: int = 0
    by_event: dict[str, int] = field(default_factory=dict)
    abort_reasons: dict[str, int] = field(default_factory=dict)
    first_stamp: float | None = None
    last_stamp: float | None = None


class EvidenceLogger:
    """One instance per flight. Not thread-safe across nodes writing
    concurrently by design - each novelty module logs through the single
    :class:`~drone_stack.novelty.delivery_node.DeliveryNode` instance that
    owns it, matching how every other node owns its own bus subscriptions."""

    def __init__(
        self,
        flight_logs_dir: Path | str,
        repo_dir: Path | str,
        model_versions: dict[str, str] | None = None,
    ) -> None:
        self._dir = Path(flight_logs_dir)
        self._model_versions = dict(model_versions or {})
        self._git_hash = _git_short_hash(Path(repo_dir))
        self._counts = _DecisionCounts()
        self._path: Path | None = None
        self._handle = None
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
            filename = f"{_iso_stamp()}_{self._git_hash}.jsonl"
            self._path = self._dir / filename
            self._handle = self._path.open("a", encoding="utf-8")
            _log.info("evidence log: %s", self._path)
        except OSError as exc:
            _log.warning("evidence_logger: could not open flight log (%s)", exc)

    # -- properties ------------------------------------------------------
    @property
    def path(self) -> Path | None:
        return self._path

    @property
    def git_hash(self) -> str:
        return self._git_hash

    def set_model_versions(self, versions: dict[str, str]) -> None:
        self._model_versions = dict(versions)

    # -- logging -----------------------------------------------------------
    def log_event(
        self,
        event_name: str,
        mission_state: str,
        inputs: dict[str, Any],
        computed_values: dict[str, Any],
        threshold: dict[str, Any] | float | None,
        decision: str,
    ) -> None:
        """Write one decision record. Never raises."""
        record = {
            "event_name": event_name,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "mission_state": mission_state,
            "inputs": to_jsonable(inputs),
            "computed_values": to_jsonable(computed_values),
            "threshold": to_jsonable(threshold),
            "decision": decision,
            "model_versions": dict(self._model_versions),
        }
        self._update_counts(event_name, decision)
        self._write(record)

    def _update_counts(self, event_name: str, decision: str) -> None:
        now = time.time()
        self._counts.total += 1
        self._counts.by_event[event_name] = self._counts.by_event.get(event_name, 0) + 1
        if self._counts.first_stamp is None:
            self._counts.first_stamp = now
        self._counts.last_stamp = now
        if decision.startswith("abort") or "abort" in decision.lower():
            self._counts.abort_reasons[decision] = self._counts.abort_reasons.get(decision, 0) + 1

    def _write(self, record: dict) -> None:
        if self._handle is None:
            return
        try:
            self._handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            self._handle.flush()
        except OSError as exc:
            _log.warning("evidence_logger: write failed (%s)", exc)

    # -- summary ------------------------------------------------------------
    def summary(self) -> dict[str, Any]:
        c = self._counts
        duration_s = (
            round(c.last_stamp - c.first_stamp, 3)
            if c.first_stamp is not None and c.last_stamp is not None
            else 0.0
        )
        return {
            "total_decisions": c.total,
            "decisions_by_event": dict(c.by_event),
            "abort_reasons": dict(c.abort_reasons),
            "duration_s": duration_s,
            "git_hash": self._git_hash,
            "model_versions": dict(self._model_versions),
        }

    def close(self) -> None:
        """Write the per-flight summary line and close the file. Idempotent."""
        if self._handle is None:
            return
        try:
            self._handle.write(
                json.dumps({"event_name": "flight_summary", **self.summary()}, ensure_ascii=False)
                + "\n"
            )
            self._handle.flush()
            self._handle.close()
        except OSError as exc:
            _log.warning("evidence_logger: close failed (%s)", exc)
        finally:
            self._handle = None

    def __enter__(self) -> "EvidenceLogger":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
