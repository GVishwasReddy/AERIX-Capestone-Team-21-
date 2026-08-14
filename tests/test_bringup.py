"""launch/bringup.py - the novelty layer is additive and off by default
(project plan §9): DeliveryNode must be absent unless novelty.enabled is
explicitly set, and constructible without error when it is."""
from __future__ import annotations

from drone_stack.launch.bringup import build_supervisor
from drone_stack.utils.config import Config


def test_novelty_disabled_by_default_no_delivery_node():
    supervisor = build_supervisor(Config.load(), include_web=False)
    names = [n.node_name for n in supervisor.nodes]
    assert "delivery" not in names
    assert names  # sanity: the rest of the stack still builds


def test_novelty_enabled_adds_delivery_node(tmp_path):
    cfg = Config.load().with_overrides(
        novelty={
            "enabled": True,
            "rate_hz": 5.0,
            "flight_logs_dir": str(tmp_path / "flight_logs"),
            "repo_dir": ".",
        }
    )
    supervisor = build_supervisor(cfg, include_web=False)
    names = [n.node_name for n in supervisor.nodes]
    assert "delivery" in names
