"""Tests for the OBSTACLE_DISTANCE construction.

The message-building logic is real and fully exercised here with synthetic
scan data — no lidar hardware and no MAVLink connection required.
"""
import pytest

from lidar_bridge import (
    DEFAULT_FOV_DEG,
    DISTANCE_UNKNOWN,
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    LidarBridge,
    parse_measurement_nodes,
    scan_to_distances,
    sector_keep_mask,
    to_pymavlink_connection,
    wrap_180,
)


def encode_node(is_start: bool, angle_deg: float, distance_m: float) -> bytes:
    """Build a 5-byte legacy measurement node, as the C1 emits them."""
    start = 1 if is_start else 0
    # bit0 = start flag, bit1 = its inverse, bits2-7 = quality
    b0 = start | ((1 - start) << 1) | (0x20 << 2)
    angle_q6 = int(round(angle_deg * 64.0))
    b1 = ((angle_q6 & 0x7F) << 1) | 1  # check bit set
    b2 = (angle_q6 >> 7) & 0xFF
    dist_q2 = int(round(distance_m * 1000.0 * 4.0))
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0 & 0xFF, b1 & 0xFF, b2, b3, b4])

MIN_CM, MAX_CM = 15, 1200


def build(scan, **kwargs):
    return scan_to_distances(scan, MIN_CM, MAX_CM, **kwargs)


class TestArrayShape:
    def test_always_72_sectors(self):
        assert len(build([])) == SECTOR_COUNT
        assert len(build([(0.0, 5.0)])) == SECTOR_COUNT

    def test_empty_scan_is_all_unknown(self):
        assert build([]) == [DISTANCE_UNKNOWN] * SECTOR_COUNT

    def test_sector_width_is_five_degrees(self):
        assert SECTOR_WIDTH_DEG == 5.0


class TestBucketing:
    def test_zero_degrees_lands_in_sector_zero(self):
        distances = build([(0.0, 5.0)])
        assert distances[0] == 500
        assert distances[1] == DISTANCE_UNKNOWN

    def test_angle_maps_to_expected_sector(self):
        # 47 degrees / 5 = sector 9
        distances = build([(47.0, 3.0)])
        assert distances[9] == 300

    def test_last_sector(self):
        distances = build([(359.9, 2.0)])
        assert distances[71] == 200

    def test_360_wraps_to_sector_zero(self):
        distances = build([(360.0, 4.0)])
        assert distances[0] == 400

    def test_negative_angle_wraps(self):
        # -5 degrees == 355 degrees -> sector 71
        distances = build([(-5.0, 4.0)])
        assert distances[71] == 400

    def test_meters_converted_to_centimeters(self):
        assert build([(0.0, 1.0)])[0] == 100
        assert build([(0.0, 7.25)])[0] == 725


class TestClosestWins:
    def test_nearest_reading_in_a_sector_is_kept(self):
        # Both land in sector 0; the closer one must win (conservative).
        distances = build([(1.0, 8.0), (2.0, 3.0), (3.0, 6.0)])
        assert distances[0] == 300

    def test_order_does_not_matter(self):
        forward = build([(1.0, 3.0), (2.0, 8.0)])
        reverse = build([(2.0, 8.0), (1.0, 3.0)])
        assert forward == reverse


class TestFiltering:
    def test_drops_readings_below_min_range(self):
        assert build([(0.0, 0.05)])[0] == DISTANCE_UNKNOWN

    def test_drops_readings_beyond_max_range(self):
        assert build([(0.0, 50.0)])[0] == DISTANCE_UNKNOWN

    @pytest.mark.parametrize("distance", [0.0, -1.0, None])
    def test_drops_invalid_distances(self, distance):
        assert build([(0.0, distance)])[0] == DISTANCE_UNKNOWN

    def test_keeps_readings_exactly_at_limits(self):
        assert build([(0.0, MIN_CM / 100)])[0] == MIN_CM
        assert build([(0.0, MAX_CM / 100)])[0] == MAX_CM


class TestAngleOffset:
    def test_offset_shifts_sector(self):
        # 0 degrees with a +90 degree mounting offset -> sector 18
        distances = build([(0.0, 5.0)], angle_offset_deg=90.0)
        assert distances[18] == 500
        assert distances[0] == DISTANCE_UNKNOWN

    def test_offset_wraps_past_360(self):
        distances = build([(350.0, 5.0)], angle_offset_deg=20.0)
        assert distances[2] == 500


class TestFullScan:
    def test_uniform_wall_fills_every_sector(self):
        scan = [(angle, 4.0) for angle in range(0, 360)]
        distances = build(scan)
        assert all(d == 400 for d in distances)

    def test_realistic_partial_scan(self):
        # Obstacle spanning 30-60 degrees, nothing elsewhere.
        scan = [(float(a), 2.5) for a in range(30, 61)]
        distances = build(scan)
        assert distances[6] == 250    # 30 deg
        assert distances[11] == 250   # 55-60 deg
        assert distances[0] == DISTANCE_UNKNOWN
        assert distances[40] == DISTANCE_UNKNOWN

    def test_all_values_fit_uint16(self):
        scan = [(float(a), 12.0) for a in range(0, 360)]
        assert all(0 <= d <= 65535 for d in build(scan))


class TestConnectionStringTranslation:
    """MAVSDK and pymavlink disagree on connection-string syntax.

    Getting this wrong is silent and nasty: pymavlink treats any device
    containing a colon as UDP, so a MAVSDK serial string turns into a bogus
    UDP connection instead of raising.
    """

    def test_serial_with_baud(self):
        assert to_pymavlink_connection("serial:///dev/ttyACM0:115200") == (
            "/dev/ttyACM0",
            115200,
        )

    def test_serial_by_id_path_with_baud(self):
        device, baud = to_pymavlink_connection(
            "serial:///dev/serial/by-id/usb-ArduPilot_Pixhawk1_1234-if00:921600"
        )
        assert device == "/dev/serial/by-id/usb-ArduPilot_Pixhawk1_1234-if00"
        assert baud == 921600

    def test_serial_without_baud(self):
        assert to_pymavlink_connection("serial:///dev/ttyACM0") == ("/dev/ttyACM0", None)

    def test_udp_listen(self):
        assert to_pymavlink_connection("udp://:14540") == ("udpin:0.0.0.0:14540", None)

    def test_udp_connect(self):
        assert to_pymavlink_connection("udp://127.0.0.1:14550") == (
            "udpout:127.0.0.1:14550",
            None,
        )

    def test_tcp(self):
        assert to_pymavlink_connection("tcp://:5760") == ("tcp:127.0.0.1:5760", None)

    def test_bare_device_path_passes_through(self):
        assert to_pymavlink_connection("/dev/ttyACM0") == ("/dev/ttyACM0", None)

    def test_native_pymavlink_string_passes_through(self):
        assert to_pymavlink_connection("udpin:0.0.0.0:14540") == (
            "udpin:0.0.0.0:14540",
            None,
        )

    def test_serial_never_yields_a_colon_device(self):
        # The actual failure mode: a colon in the device makes pymavlink pick
        # its UDP transport and raise "UDP ports must be specified as host:port".
        device, _ = to_pymavlink_connection("serial:///dev/ttyACM0:115200")
        assert ":" not in device


class TestNodeParser:
    """The C1 is driven over raw serial, so we parse its 5-byte nodes ourselves."""

    def test_parses_a_single_node(self):
        raw = encode_node(True, 90.0, 3.0)
        nodes, leftover = parse_measurement_nodes(raw)
        assert len(nodes) == 1
        is_start, angle, distance = nodes[0]
        assert is_start is True
        assert angle == pytest.approx(90.0, abs=0.05)
        assert distance == pytest.approx(3.0, abs=0.01)
        assert leftover == b""

    def test_parses_multiple_nodes(self):
        raw = b"".join(
            encode_node(i == 0, float(i * 10), 2.0 + i * 0.5) for i in range(6)
        )
        nodes, leftover = parse_measurement_nodes(raw)
        assert len(nodes) == 6
        assert leftover == b""
        assert nodes[0][0] is True
        assert all(n[0] is False for n in nodes[1:])

    def test_returns_partial_trailing_bytes_as_leftover(self):
        raw = encode_node(True, 45.0, 1.5) + b"\x01\x02"
        nodes, leftover = parse_measurement_nodes(raw)
        assert len(nodes) == 1
        assert len(leftover) == 2

    def test_resyncs_when_buffer_starts_mid_packet(self):
        # Two junk bytes that cannot begin a valid node, then a real one.
        raw = b"\x00\x00" + encode_node(True, 180.0, 4.0)
        nodes, _ = parse_measurement_nodes(raw)
        assert len(nodes) == 1
        assert nodes[0][1] == pytest.approx(180.0, abs=0.05)

    def test_empty_buffer(self):
        nodes, leftover = parse_measurement_nodes(b"")
        assert nodes == []
        assert leftover == b""

    def test_parsed_scan_feeds_the_bucketer(self):
        raw = b"".join(
            encode_node(i == 0, float(i * 5), 2.5) for i in range(72)
        )
        nodes, _ = parse_measurement_nodes(raw)
        readings = [(angle, dist) for _s, angle, dist in nodes]
        distances = build(readings)
        assert all(d == 250 for d in distances)


class TestWrap180:
    @pytest.mark.parametrize(
        "raw, expected",
        # Range is [-180, 180), so 180 folds to -180. Immaterial to the mask,
        # which only ever reads abs(), but pin it down rather than leave it open.
        [(0.0, 0.0), (90.0, 90.0), (180.0, -180.0), (181.0, -179.0),
         (270.0, -90.0), (360.0, 0.0), (-90.0, -90.0), (-270.0, 90.0)],
    )
    def test_folds_onto_the_symmetric_range(self, raw, expected):
        assert wrap_180(raw) == pytest.approx(expected)


class TestSectorKeepMask:
    """The rear wedge must never reach the flight controller.

    The C1 spins a full circle, but the rear of that circle is the airframe's
    own tail and whatever the drone is parked next to — stationary returns that
    make ArduPilot brake for the vehicle itself.
    """

    def test_default_window_matches_drone_stack(self):
        # 250 deg, judged on sector centres: 0-24 and 47-71 kept, 25-46 masked.
        keep = sector_keep_mask()
        assert len(keep) == SECTOR_COUNT
        assert sum(keep) == 50
        assert all(keep[i] for i in range(0, 25))
        assert not any(keep[i] for i in range(25, 47))
        assert all(keep[i] for i in range(47, SECTOR_COUNT))

    def test_masked_arc_is_the_rear_110_degrees(self):
        masked = [i for i, ok in enumerate(sector_keep_mask()) if not ok]
        assert len(masked) * SECTOR_WIDTH_DEG == 360.0 - DEFAULT_FOV_DEG

    def test_mask_is_symmetric_about_the_nose(self):
        # Sector i and its mirror (72 - 1 - i) look the same distance off the
        # nose, so a left/right handedness slip would show up here.
        keep = sector_keep_mask()
        for i in range(SECTOR_COUNT):
            assert keep[i] == keep[(SECTOR_COUNT - 1 - i)]

    def test_nose_and_tail(self):
        keep = sector_keep_mask()
        assert keep[0] is True                      # straight ahead
        assert keep[SECTOR_COUNT // 2] is False     # straight behind

    def test_disabled_keeps_the_full_circle(self):
        assert sector_keep_mask(DEFAULT_FOV_DEG, enabled=False) == [True] * SECTOR_COUNT

    def test_360_degrees_is_not_a_mask(self):
        assert sector_keep_mask(360.0) == [True] * SECTOR_COUNT

    def test_narrower_window_keeps_fewer_sectors(self):
        assert sum(sector_keep_mask(180.0)) == 36
        assert sum(sector_keep_mask(90.0)) == 18

    def test_zero_degrees_keeps_nothing(self):
        assert sum(sector_keep_mask(0.0)) == 0

    def test_boundary_sector_is_kept(self):
        # Sector 24's centre is 122.5 deg, inside a 125 deg half-window; 25's
        # is 127.5 and outside. An off-by-one here silently narrows the view.
        keep = sector_keep_mask()
        assert keep[24] is True
        assert keep[25] is False


class TestMaskedBucketing:
    def test_rear_beams_cannot_reach_the_flight_controller(self):
        keep = sector_keep_mask()
        # 180 deg = directly behind, well inside the masked wedge.
        distances = build([(180.0, 1.0)], keep=keep)
        assert distances[36] == DISTANCE_UNKNOWN
        assert distances == [DISTANCE_UNKNOWN] * SECTOR_COUNT

    def test_masked_sector_stays_unknown_not_far(self):
        # UNKNOWN, not a large distance: a fake "nothing for 12 m" reading
        # would be a claim about ground the sensor never scanned.
        keep = sector_keep_mask()
        assert build([(180.0, 0.3)], keep=keep)[36] == DISTANCE_UNKNOWN

    def test_front_beams_are_unaffected(self):
        keep = sector_keep_mask()
        assert build([(0.0, 5.0)], keep=keep)[0] == 500
        assert build([(120.0, 3.0)], keep=keep)[24] == 300
        assert build([(240.0, 3.0)], keep=keep)[48] == 300

    def test_uniform_wall_only_fills_the_scanned_window(self):
        keep = sector_keep_mask()
        scan = [(float(a), 4.0) for a in range(360)]
        distances = build(scan, keep=keep)
        assert sum(1 for d in distances if d == 400) == 50
        assert sum(1 for d in distances if d == DISTANCE_UNKNOWN) == 22

    def test_no_mask_is_the_old_behaviour(self):
        scan = [(float(a), 4.0) for a in range(360)]
        assert all(d == 400 for d in build(scan))
        assert build(scan) == build(scan, keep=None)

    def test_mask_is_applied_after_the_mounting_offset(self):
        # A beam 180 deg off the RAW front is only "behind the aircraft" once
        # the mounting offset has been applied. With a +90 deg offset the beam
        # that ends up behind is the one the lidar calls 90 deg.
        keep = sector_keep_mask()
        assert build([(90.0, 1.0)], angle_offset_deg=90.0, keep=keep) == (
            [DISTANCE_UNKNOWN] * SECTOR_COUNT
        )
        assert build([(180.0, 1.0)], angle_offset_deg=90.0, keep=keep)[54] == 100


class TestBridgeMaskWiring:
    """The mask has to be built into the bridge, not just available to it."""

    def _bridge(self, **kwargs):
        return LidarBridge(
            mavlink_connection="udp://:14540",
            lidar_port="/dev/null",
            **kwargs,
        )

    def test_mask_is_on_by_default(self):
        assert self._bridge()._keep == sector_keep_mask()

    def test_mask_can_be_disabled(self):
        assert self._bridge(fov_enabled=False)._keep == [True] * SECTOR_COUNT

    def test_custom_window_is_honoured(self):
        assert self._bridge(fov_deg=180.0)._keep == sector_keep_mask(180.0)

    def test_mask_is_built_once_not_per_scan(self):
        bridge = self._bridge()
        assert bridge._keep is bridge._keep
