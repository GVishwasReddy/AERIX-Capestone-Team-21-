"""novelty/evidence_logger.py - a decision record round-trips exactly through
log_event() -> the JSONL file, filenames carry ISO timestamp + git hash, and
logger failures never raise (§3, §7 "log and escalate, never crash on log I/O
itself" - except the logger's own write path is explicitly exempt, see the
module docstring)."""
from __future__ import annotations

import json
import subprocess

from drone_stack.novelty.evidence_logger import EvidenceLogger


def _git_init(repo_dir) -> str:
    subprocess.run(["git", "init", "-q"], cwd=repo_dir, check=True)
    subprocess.run(["git", "config", "user.email", "test@test"], cwd=repo_dir, check=True)
    subprocess.run(["git", "config", "user.name", "test"], cwd=repo_dir, check=True)
    (repo_dir / "f.txt").write_text("x", encoding="utf-8")
    subprocess.run(["git", "add", "."], cwd=repo_dir, check=True)
    subprocess.run(["git", "commit", "-q", "-m", "init"], cwd=repo_dir, check=True)
    out = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], cwd=repo_dir, capture_output=True, text=True, check=True
    )
    return out.stdout.strip()


def test_filename_carries_iso_timestamp_and_git_hash(tmp_path):
    repo_dir = tmp_path / "repo"
    repo_dir.mkdir()
    git_hash = _git_init(repo_dir)
    logs_dir = tmp_path / "flight_logs"

    logger = EvidenceLogger(flight_logs_dir=logs_dir, repo_dir=repo_dir)
    try:
        assert logger.git_hash == git_hash
        assert logger.path is not None
        assert logger.path.parent == logs_dir
        assert logger.path.name.endswith(f"_{git_hash}.jsonl")
        # ISO8601Z prefix: YYYYMMDDTHHMMSSZ
        stamp = logger.path.name.split("_")[0]
        assert len(stamp) == 16 and stamp.endswith("Z")
    finally:
        logger.close()


def test_no_git_repo_falls_back_to_nogit(tmp_path):
    repo_dir = tmp_path / "not_a_repo"
    repo_dir.mkdir()
    logs_dir = tmp_path / "flight_logs"

    logger = EvidenceLogger(flight_logs_dir=logs_dir, repo_dir=repo_dir)
    try:
        assert logger.git_hash == "nogit"
        assert logger.path.name.endswith("_nogit.jsonl")
    finally:
        logger.close()


def test_log_event_round_trips_through_jsonl(tmp_path):
    repo_dir = tmp_path / "repo"
    repo_dir.mkdir()
    _git_init(repo_dir)
    logs_dir = tmp_path / "flight_logs"

    with EvidenceLogger(flight_logs_dir=logs_dir, repo_dir=repo_dir) as logger:
        logger.set_model_versions({"terrain": "abc123@2026-08-01"})
        logger.log_event(
            event_name="landing_zone_selected",
            mission_state="SEARCHING",
            inputs={"candidate_count": 3},
            computed_values={"score": 0.82},
            threshold={"min_score": 0.35},
            decision="zone_accepted",
        )
        log_path = logger.path

    lines = log_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2  # one decision record + one flight_summary

    record = json.loads(lines[0])
    assert record["event_name"] == "landing_zone_selected"
    assert record["mission_state"] == "SEARCHING"
    assert record["inputs"] == {"candidate_count": 3}
    assert record["computed_values"] == {"score": 0.82}
    assert record["threshold"] == {"min_score": 0.35}
    assert record["decision"] == "zone_accepted"
    assert record["model_versions"] == {"terrain": "abc123@2026-08-01"}
    assert "timestamp_utc" in record

    summary = json.loads(lines[1])
    assert summary["event_name"] == "flight_summary"
    assert summary["total_decisions"] == 1
    assert summary["decisions_by_event"] == {"landing_zone_selected": 1}


def test_abort_decisions_are_counted_in_summary(tmp_path):
    repo_dir = tmp_path / "repo"
    repo_dir.mkdir()
    _git_init(repo_dir)

    with EvidenceLogger(flight_logs_dir=tmp_path / "flight_logs", repo_dir=repo_dir) as logger:
        logger.log_event("motion_check", "DESCENDING", {}, {}, {}, decision="abort_descent")
        logger.log_event("motion_check", "DESCENDING", {}, {}, {}, decision="continue")
        summary = logger.summary()

    assert summary["total_decisions"] == 2
    assert summary["abort_reasons"] == {"abort_descent": 1}


def test_close_is_idempotent(tmp_path):
    repo_dir = tmp_path / "repo"
    repo_dir.mkdir()
    _git_init(repo_dir)

    logger = EvidenceLogger(flight_logs_dir=tmp_path / "flight_logs", repo_dir=repo_dir)
    logger.log_event("e", "STATE", {}, {}, None, decision="d")
    logger.close()
    logger.close()  # must not raise


def test_unwritable_log_dir_does_not_raise(tmp_path):
    """A read-only / unavailable flight_logs_dir must degrade, not crash a
    flight - see module docstring."""
    blocked = tmp_path / "blocked"
    blocked.write_text("not a directory", encoding="utf-8")  # file, not dir

    logger = EvidenceLogger(flight_logs_dir=blocked / "flight_logs", repo_dir=tmp_path)
    assert logger.path is None
    logger.log_event("e", "STATE", {}, {}, None, decision="d")  # must not raise
    logger.close()  # must not raise
