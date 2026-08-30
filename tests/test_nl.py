"""Tests for the natural-language command parser and its execution."""
from __future__ import annotations

from drone_stack.bus import MessageBus
from drone_stack.bus.topics import Topics
from drone_stack.msg import FusedState, MissionPhase, NavCommand
from drone_stack.nodes.navigation_node import NavigationNode
from drone_stack.srv import ServiceRegistry
from drone_stack.utils.config import Config
from drone_stack.utils.nl_parser import parse


# --- parser ----------------------------------------------------------------
def _one(text: str):
    intents = parse(text)
    assert len(intents) == 1, f"{text!r} -> {[i.describe() for i in intents]}"
    return intents[0]


def test_empty_input():
    assert parse("") == []
    assert parse("   ") == []


def test_vertical_relative_and_trailing_stay():
    # "and stay" must not cancel the climb.
    intents = parse("go up vertically for 5m and stay")
    assert len(intents) == 1
    assert intents[0].action == "move"
    assert intents[0].params["dz"] == 5.0


def test_descend():
    i = _one("descend 2m")
    assert i.action == "move" and i.params["dz"] == -2.0


def test_takeoff_with_altitude():
    i = _one("take off to 5m")
    assert i.action == "takeoff" and i.params["altitude"] == 5.0


def test_forward_back_left_right():
    assert _one("move forward 10 meters").params["dx"] == 10.0
    assert _one("go back 4m").params["dx"] == -4.0
    assert _one("go left 3m").params["dy"] == 3.0
    assert _one("strafe right 2m").params["dy"] == -2.0


def test_yaw_direction_and_angle():
    r = _one("turn right 90 degrees")
    assert r.action == "yaw" and r.params["angle_deg"] == 90.0 and r.params["direction"] == 1
    l = _one("rotate left 45 deg")
    assert l.params["direction"] == -1 and l.params["angle_deg"] == 45.0


def test_absolute_altitude():
    i = _one("climb to 10m")
    assert i.action == "set_altitude" and i.params["altitude"] == 10.0


def test_set_speed():
    i = _one("set speed to 4 m/s")
    assert i.action == "set_speed" and i.params["speed"] == 4.0


def test_discrete_actions():
    assert _one("land now").action == "land"
    assert _one("return home").action == "rtl"
    assert _one("rtl").action == "rtl"
    assert _one("emergency stop").action == "emergency"
    assert _one("abort").action == "emergency"
    assert _one("disarm").action == "disarm"
    assert _one("hover").action == "hold"


def test_feet_units_converted():
    i = _one("go up 10 ft")
    assert abs(i.params["dz"] - 3.048) < 1e-6


def test_multi_step_command():
    intents = parse("take off to 5m, move forward 10m and turn right 90 degrees")
    actions = [i.action for i in intents]
    assert actions == ["takeoff", "move", "yaw"]


def test_unknown_is_reported_not_dropped():
    i = _one("make me a sandwich")
    assert i.action == "unknown"


# --- execution through the navigation node ---------------------------------
def _armed_node():
    bus = MessageBus()
    node = NavigationNode(bus, Config.load(), ServiceRegistry())
    node._home = (47.397742, 8.545594)
    node._fused = FusedState(x=0.0, y=0.0, alt_rel_m=5.0, yaw=0.0, valid=True)
    node._armed = True
    sent = []
    bus.subscribe(Topics.MAVLINK_CMD, lambda m: sent.append(m))
    return node, sent


def test_nl_move_up_sets_manual_target():
    node, sent = _armed_node()
    resp = node.services.call("nl_command", text="go up 5m")
    assert resp.success
    assert node._phase == MissionPhase.MANUAL
    assert node._manual_target is not None
    # 5 m current + 5 m up would be 10 m, but the altitude ceiling is absolute:
    # a spoken command is not a way around it.
    assert abs(node._manual_target[2] - node._alt_ceiling) < 1e-6
    gotos = [c for c in sent if isinstance(c, NavCommand) and c.command == "goto"]
    assert gotos, "a goto command should have been issued"


def test_nl_forward_moves_in_heading():
    node, sent = _armed_node()
    node.services.call("nl_command", text="move forward 8m")
    tx, ty, _ = node._manual_target
    assert abs(tx - 8.0) < 1e-6 and abs(ty) < 1e-6       # yaw=0 -> +x east


def test_nl_takeoff_arms_and_takes_off():
    bus = MessageBus()
    node = NavigationNode(bus, Config.load(), ServiceRegistry())
    sent = []
    bus.subscribe(Topics.MAVLINK_CMD, lambda m: sent.append(m))
    resp = node.services.call("nl_command", text="take off to 8m")
    assert resp.success
    commands = [c.command for c in sent if isinstance(c, NavCommand)]
    assert "arm" in commands and "takeoff" in commands
    assert node._phase == MissionPhase.MANUAL


def test_nl_emergency():
    node, _ = _armed_node()
    node.services.call("nl_command", text="emergency")
    assert node._phase == MissionPhase.EMERGENCY


def test_nl_unknown_reports_failure():
    node, _ = _armed_node()
    resp = node.services.call("nl_command", text="make me a sandwich")
    assert not resp.success
    assert "did not understand" in resp.message


def test_nl_move_without_fix_fails_gracefully():
    bus = MessageBus()
    node = NavigationNode(bus, Config.load(), ServiceRegistry())  # no fused/home
    resp = node.services.call("nl_command", text="go forward 5m")
    assert not resp.success
    assert "GPS" in resp.message or "position" in resp.message


# --- stronger grammar: spelled-out numbers, vague amounts, negation --------
def test_spelled_out_numbers():
    assert _one("climb five meters").params["dz"] == 5.0
    assert _one("turn right ninety degrees").params["angle_deg"] == 90.0
    assert abs(_one("go up a hundred feet").params["dz"] - 30.48) < 1e-6
    assert _one("descend half a meter").params["dz"] == -0.5
    assert _one("move a couple of meters forward").params["dx"] == 2.0


def test_filler_and_politeness_ignored():
    assert _one("could you please take off to about eight meters").params["altitude"] == 8.0
    assert _one("just hover").action == "hold"


def test_vague_magnitudes():
    assert _one("go up a bit").params["dz"] == 1.0
    assert _one("move forward a lot").params["dx"] == 8.0
    assert _one("turn slightly left").params == {"angle_deg": 30.0, "direction": -1}


def test_turn_around_is_half_turn():
    assert _one("turn around").params["angle_deg"] == 180.0


def test_negation_drops_clause():
    assert parse("don't land") == []
    assert parse("do not disarm") == []
    assert parse("no need to take off") == []


def test_extra_connectives_split():
    intents = parse("take off to 5m then move forward 10m after that turn around")
    assert [i.action for i in intents] == ["takeoff", "move", "yaw"]
    assert intents[1].params["dx"] == 10.0
    assert intents[2].params["angle_deg"] == 180.0
