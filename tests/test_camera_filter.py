"""Tests for the live-camera image conditioning chain.

The artefact being fixed is row-correlated interference from the ESCs coupling
into the CSI ribbon in flight: horizontal lines drifting through the picture.
The two things that matter are therefore symmetric, and both are asserted
here - the filter must remove a stripe, and it must NOT touch a horizontal
feature that belongs to the scene. A destriper that eats the horizon is worse
than no destriper at all.

numpy only: no OpenCV, no picamera2, so these run anywhere.
"""
from __future__ import annotations

import numpy as np
import pytest

from drone_stack.gcs.cameras import CameraManager, _encode_jpeg
from drone_stack.gcs.frame_filter import FrameFilter, filter_from_config, rolling_median

H, W = 240, 320


def _scene(seed: int = 7) -> np.ndarray:
    """A frame carrying real structure the filter must preserve: a vertical
    gradient, a hard horizontal edge (the horizon), a vertical pole, noise."""
    rng = np.random.default_rng(seed)
    img = np.zeros((H, W, 3), np.float32)
    img[:, :] = np.linspace(40, 90, H)[:, None, None]
    img[H // 2:] += 60.0                      # hard horizon step
    img[:, 150:170] += 45.0                   # vertical pole
    img += rng.normal(0.0, 2.5, (H, W, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


def _stripe(img: np.ndarray, rows, amp: int) -> np.ndarray:
    out = img.astype(np.int16)
    out[rows] += amp
    return np.clip(out, 0, 255).astype(np.uint8)


# --------------------------------------------------------------- primitives --
def test_rolling_median_follows_a_step_without_overshoot():
    """The reason a median is used and not a mean: a genuine horizontal edge
    must not appear in the residual, or it would be 'corrected' away."""
    signal = np.concatenate([np.full(40, 10.0), np.full(40, 70.0)])
    base = rolling_median(signal, 15)
    assert np.abs(base - signal).max() == pytest.approx(0.0, abs=1e-5)


def test_rolling_median_rejects_a_single_outlier():
    signal = np.full(40, 10.0)
    signal[20] = 200.0
    base = rolling_median(signal, 15)
    assert base[20] == pytest.approx(10.0)


def test_rolling_median_degenerate_windows_are_safe():
    signal = np.arange(5, dtype=np.float32)
    assert np.array_equal(rolling_median(signal, 1), signal)
    assert rolling_median(signal, 999).shape == signal.shape


# ---------------------------------------------------------------- destripe --
def test_clean_frame_passes_through_untouched():
    """No stripe, no correction - not 'a small correction'. A filter that
    always nudges something is a filter that is always slightly wrong."""
    clean = _scene()
    filt = FrameFilter(temporal=False)
    out = filt.apply(clean)
    assert np.array_equal(out, clean)
    assert filt.stats()["rows_corrected"] == 0
    assert filt.stats()["rows_repaired"] == 0


def test_periodic_banding_is_removed():
    clean = _scene()
    rows = np.arange(30, 220, 7)
    dirty = _stripe(clean, rows, 9)
    out = FrameFilter(temporal=False).apply(dirty)

    before = np.abs(dirty.astype(int) - clean.astype(int))[rows].mean()
    after = np.abs(out.astype(int) - clean.astype(int))[rows].mean()
    assert after < before * 0.25, f"banding {before:.2f} -> {after:.2f} DN"


def test_a_single_bright_line_is_removed():
    clean = _scene()
    dirty = _stripe(clean, [77], 9)          # inside destripe_ceiling
    filt = FrameFilter(temporal=False)
    out = filt.apply(dirty)
    assert np.abs(out[77].astype(int) - clean[77].astype(int)).mean() < 2.0
    # ...and its neighbours were left alone, bit-for-bit
    assert np.array_equal(out[70], dirty[70])


@pytest.mark.parametrize("amp, left_max, repaired", [
    (9,  1.5, 0),    # inside destripe_ceiling  -> shrunk away entirely
    (11, 1.5, 0),
    (13, 3.5, 0),    # past the ceiling         -> capped, a little left over
    (15, 0.5, 1),    # past row_repair_floor    -> row rebuilt, nothing left
    (25, 0.5, 1),
])
def test_the_correction_ladder(amp, left_max, repaired):
    """The three regimes, in one place, because they have to stay ordered:
    below destripe_ceiling a line is shrunk away; between the ceiling and
    row_repair_floor it is capped (the price of never gouging a real edge);
    above row_repair_floor the row is rebuilt outright and nothing is left."""
    clean = _scene()
    filt = FrameFilter(temporal=False)
    out = filt.apply(_stripe(clean, [77], amp))
    left = abs(float(out[77].mean()) - float(clean[77].mean()))
    assert left <= left_max, f"amp {amp}: {left:.2f} DN left"
    assert filt.stats()["rows_repaired"] == repaired


def test_the_horizon_survives_destriping():
    """The scene's own hard horizontal edge must come through intact."""
    clean = _scene()
    dirty = _stripe(clean, np.arange(30, 220, 7), 9)
    out = FrameFilter(temporal=False).apply(dirty)

    step_in = float(clean[H // 2:H // 2 + 4].mean() - clean[H // 2 - 4:H // 2].mean())
    step_out = float(out[H // 2:H // 2 + 4].mean() - out[H // 2 - 4:H // 2].mean())
    assert step_out == pytest.approx(step_in, rel=0.10)


def test_vertical_detail_is_untouched():
    """Destriping acts along rows only; a vertical pole must not soften."""
    clean = _scene()
    out = FrameFilter(temporal=False).apply(_stripe(clean, [100], 12))
    edge_in = float(clean[:, 150:170].mean() - clean[:, 130:150].mean())
    edge_out = float(out[:, 150:170].mean() - out[:, 130:150].mean())
    assert edge_out == pytest.approx(edge_in, rel=0.05)


def test_destripe_can_be_disabled_independently_of_repair():
    clean = _scene()
    dirty = _stripe(clean, np.arange(30, 220, 7), 9)
    filt = FrameFilter(temporal=False, destripe=False, row_repair=True)
    out = filt.apply(dirty)
    assert filt.stats()["rows_corrected"] == 0
    assert np.array_equal(out, dirty)


# -------------------------------------------------------------- row repair --
def test_destroyed_rows_are_rebuilt_from_neighbours():
    clean = _scene()
    wrecked = clean.copy()
    wrecked[100] = 250          # blown out
    wrecked[101] = 4            # crushed
    filt = FrameFilter(temporal=False)
    out = filt.apply(wrecked)

    assert filt.stats()["rows_repaired"] == 2
    err = np.abs(out[100:102].astype(int) - clean[100:102].astype(int)).mean()
    assert err < 8.0, f"repaired rows still off by {err:.1f} DN"


def test_row_repair_does_not_fire_on_a_hard_horizontal_edge():
    """A large row residual is not proof of damage - the rows either side of a
    horizon show one too, and repairing those interpolates ACROSS the edge.
    That halved an 89 DN horizon before the neighbour-similarity check existed.
    A destroyed row resembles neither neighbour; an edge row resembles its own
    side. Only the first may be rebuilt."""
    frame = np.zeros((H, W, 3), np.uint8)
    frame[:H // 2] = 30
    frame[H // 2:] = 200                    # 170 DN step, far past row_repair_floor
    filt = FrameFilter(temporal=False)
    out = filt.apply(frame)

    assert filt.stats()["rows_repaired"] == 0, "the horizon was mistaken for damage"
    step = float(out[H // 2:H // 2 + 3].mean() - out[H // 2 - 3:H // 2].mean())
    assert step == pytest.approx(170.0, abs=12.0), f"edge softened to {step:.0f} DN"


def test_a_destroyed_row_next_to_an_edge_is_still_repaired():
    """The guard must not become a blanket exemption."""
    frame = np.zeros((H, W, 3), np.uint8)
    frame[:H // 2] = 30
    frame[H // 2:] = 200
    frame[H // 2 + 6] = 255                 # genuinely destroyed, near the edge
    filt = FrameFilter(temporal=False)
    out = filt.apply(frame)
    assert filt.stats()["rows_repaired"] == 1
    assert abs(int(out[H // 2 + 6].mean()) - 200) < 12


def test_destripe_ceiling_bounds_a_single_row_correction():
    """Caps how far one row can be moved, so a residual the median could not
    follow perfectly cannot gouge the image."""
    clean = _scene()
    dirty = _stripe(clean, [90], 40)
    filt = FrameFilter(temporal=False, destripe_ceiling=5.0, row_repair=False)
    out = filt.apply(dirty)
    moved = float(dirty[90].mean()) - float(out[90].mean())
    assert 0 < moved <= 6.0, f"row moved {moved:.1f} DN despite a 5 DN ceiling"


def test_destripe_ceiling_cannot_be_set_below_the_floor():
    filt = FrameFilter(destripe_floor=4.0, destripe_ceiling=1.0)
    assert filt.destripe_ceiling >= filt.destripe_floor


def test_repair_survives_an_entirely_destroyed_frame():
    """Every row bad -> nothing to interpolate from. Must not raise."""
    filt = FrameFilter(temporal=False)
    frame = np.zeros((32, 32, 3), np.uint8)
    frame[::2] = 255                       # alternating rows, all 'severe'
    out = filt.apply(frame)
    assert out.shape == frame.shape
    assert filt.enabled


# ---------------------------------------------------------------- temporal --
def _settle(filt, base, sigma, rng, n=8):
    for _ in range(n):
        noisy = np.clip(base.astype(np.float32) + rng.normal(0, sigma, base.shape),
                        0, 255).astype(np.uint8)
        filt.apply(noisy)


def test_temporal_denoise_reduces_noise():
    rng = np.random.default_rng(3)
    base = _scene()
    filt = FrameFilter(destripe=False, row_repair=False)
    _settle(filt, base, 6.0, rng)

    noisy = np.clip(base.astype(np.float32) + rng.normal(0, 6.0, base.shape),
                    0, 255).astype(np.uint8)
    out = filt.apply(noisy)
    sigma_in = float(np.std(noisy.astype(int) - base.astype(int)))
    sigma_out = float(np.std(out.astype(int) - base.astype(int)))
    assert sigma_out < sigma_in * 0.75, f"{sigma_in:.2f} -> {sigma_out:.2f} DN"


def test_temporal_does_not_ghost_a_moving_object():
    """The gate is what makes temporal averaging safe on a moving aircraft."""
    rng = np.random.default_rng(3)
    base = _scene()
    filt = FrameFilter(destripe=False, row_repair=False)
    _settle(filt, base, 6.0, rng)

    moved = base.copy()
    moved[:, 100:160] = 230                 # an object arrives
    out = filt.apply(moved)
    ghost = np.abs(out[:, 100:160].astype(int) - moved[:, 100:160].astype(int)).max()
    assert ghost <= 4, f"moving region smeared by {ghost} DN"


def test_temporal_does_not_drift_the_image_dark():
    """Rounding, not flooring, in the fixed-point blend - a floor would bias
    every frame downward and the picture would sink over minutes of flight."""
    filt = FrameFilter(destripe=False, row_repair=False)
    flat = np.full((64, 64, 3), 128, np.uint8)
    for _ in range(200):
        out = filt.apply(flat)
    assert int(out.min()) == 128 and int(out.max()) == 128


def test_disabling_temporal_drops_the_stale_previous_frame():
    filt = FrameFilter()
    filt.apply(_scene())
    filt._temporal_live = False
    filt.apply(_scene())
    assert filt._prev is None


def test_gate_scale_handles_a_frame_not_divisible_by_it():
    filt = FrameFilter(destripe=False, row_repair=False, temporal_gate_scale=7)
    frame = np.full((100, 130, 3), 100, np.uint8)
    filt.apply(frame)
    out = filt.apply(frame)
    assert out.shape == frame.shape


# ----------------------------------------------------------- budget guard --
def test_over_budget_drops_temporal_but_never_destripe():
    filt = FrameFilter(budget_ms=0.0)       # nothing can meet this
    clean = _scene()
    dirty = _stripe(clean, np.arange(30, 220, 7), 9)
    for _ in range(40):
        filt.apply(dirty)

    assert filt.stats()["degraded"] is True
    assert filt.stats()["temporal"] is False
    assert filt.enabled and filt.destripe        # the stage that matters stays
    out = filt.apply(dirty)
    assert filt.stats()["rows_corrected"] > 0
    assert np.abs(out.astype(int) - clean.astype(int))[np.arange(30, 220, 7)].mean() < 3.0


def test_a_generous_budget_keeps_temporal_running():
    filt = FrameFilter(budget_ms=10_000.0)
    for _ in range(40):
        filt.apply(_scene())
    assert filt.stats()["degraded"] is False
    assert filt.stats()["temporal"] is True


# -------------------------------------------------------------- robustness --
def test_mono_frames_are_handled():
    filt = FrameFilter()
    mono = np.full((64, 64), 100, np.uint8)
    mono[20] = 140
    out = filt.apply(mono)
    assert out.shape == mono.shape
    assert abs(int(out[20].mean()) - 100) < 8


def test_odd_inputs_pass_through_rather_than_raising():
    filt = FrameFilter()
    assert filt.apply(None) is None
    tiny = np.zeros((4, 4, 3), np.uint8)
    assert filt.apply(tiny) is tiny                     # too small to profile
    floats = np.zeros((64, 64, 3), np.float32)
    assert filt.apply(floats) is floats                 # wrong dtype
    assert filt.enabled


def test_a_faulty_filter_disables_itself_and_passes_the_frame():
    """A filter bug must never take the camera down with it."""
    filt = FrameFilter()
    filt.destripe_window = "not a number"               # force an internal error
    frame = _scene()
    out = filt.apply(frame)
    assert out is frame
    assert filt.enabled is False
    assert filt.apply(frame) is frame                   # and stays out of the way


def test_disabled_filter_is_a_passthrough():
    filt = FrameFilter(enabled=False)
    frame = _scene()
    assert filt.apply(frame) is frame


# ------------------------------------------------------------------ config --
def test_filter_from_config_maps_every_key():
    filt = filter_from_config({
        "enabled": True, "destripe": False, "destripe_window": 21,
        "destripe_floor": 1.5, "destripe_gain": 0.5, "row_repair": False,
        "destripe_ceiling": 7.5,
        "row_repair_floor": 30.0, "temporal": False, "temporal_alpha": 0.4,
        "temporal_motion": 33.0, "temporal_deadband": 9.0,
        "temporal_gate_scale": 8, "column_step": 2, "budget_ms": 5.0,
    })
    assert (filt.destripe, filt.destripe_window, filt.destripe_floor) == (False, 21, 1.5)
    assert filt.destripe_ceiling == 7.5
    assert (filt.row_repair, filt.row_repair_floor) == (False, 30.0)
    assert (filt.temporal_alpha, filt.temporal_motion) == (0.4, 33.0)
    assert (filt.temporal_deadband, filt.temporal_gate_scale) == (9.0, 8)
    assert (filt.column_step, filt.budget_ms) == (2, 5.0)


def test_filter_from_config_accepts_nothing():
    assert filter_from_config(None).enabled is True


def test_out_of_range_config_is_clamped():
    filt = filter_from_config({"temporal_alpha": 5.0, "temporal_deadband": 99.0,
                               "temporal_motion": 20.0, "temporal_gate_scale": 0})
    assert filt.temporal_alpha <= 0.95          # 1.0 would freeze the picture
    assert filt.temporal_deadband < filt.temporal_motion   # or the gate inverts
    assert filt.temporal_gate_scale >= 1


def test_real_config_wires_the_pi_camera(config):
    """Default policy: filter the CSI camera - it is the one on the ribbon that
    runs past the ESCs. It is also the ONLY camera since the USB C270 was
    removed on 2026-09-11, so cam 0 must no longer exist at all."""
    mgr = CameraManager(bus=None, enabled=True,
                        settings=config.get("cameras", {}))
    assert mgr.get(1)._filter is not None and mgr.get(1)._filter.enabled
    assert mgr.get(0) is None


def test_real_config_rotates_the_pi_camera_180(config):
    """The camera is mounted upside down, so the deployed config must ask for
    the flip - otherwise the whole feed is inverted and nothing says why."""
    mgr = CameraManager(bus=None, enabled=True,
                        settings=config.get("cameras", {}))
    assert mgr.get(1)._rotate_180 is True


def test_camera_manager_still_works_with_no_settings():
    mgr = CameraManager(bus=None, enabled=True)
    assert mgr.get(1) is not None
    assert mgr.get(0) is None


def test_camera_settings_reach_the_devices():
    mgr = CameraManager(bus=None, enabled=True, settings={
        "jpeg_quality": 85,
        "picam": {"width": 640, "height": 360, "fps": 15,
                  "isp_denoise": "high_quality", "ae_mode": "normal",
                  "rotate_180": True},
        "filter": {"enabled": True},
    })
    pi = mgr.get(1)
    assert (pi._req_w, pi._req_h, pi._fps_cap) == (640, 360, 15)
    assert (pi._isp_denoise, pi._ae_mode) == ("high_quality", "normal")
    assert pi._rotate_180 is True
    assert pi._jpeg_quality == 85


# ------------------------------------------------------ frame sequencing ----
class _FakeCv2:
    IMWRITE_JPEG_QUALITY = 1

    def __init__(self, payload):
        self._payload = payload

    def imencode(self, ext, frame, params):
        return (self._payload is not None), self._payload


def test_encode_rejects_a_truncated_jpeg():
    """A truncated JPEG renders as the top of the image over a hard horizontal
    edge and grey below - the exact artefact this work is chasing. Drop it."""
    good = np.frombuffer(b"\xff\xd8\x00\x01\xff\xd9", np.uint8)
    assert _encode_jpeg(_FakeCv2(good), None, 60) == bytes(good)

    truncated = np.frombuffer(b"\xff\xd8\x00\x01\x00\x00", np.uint8)
    assert _encode_jpeg(_FakeCv2(truncated), None, 60) is None
    assert _encode_jpeg(_FakeCv2(np.frombuffer(b"\xff", np.uint8)), None, 60) is None
    assert _encode_jpeg(_FakeCv2(None), None, 60) is None


def test_publish_advances_the_sequence_and_wakes_a_waiter():
    import threading
    cam = CameraManager(bus=None, enabled=True).get(1)
    _, seq0 = cam.jpeg_seq()

    woke = threading.Event()

    def waiter():
        if cam.wait_for_frame(seq0, timeout=5.0):
            woke.set()

    t = threading.Thread(target=waiter, daemon=True)
    t.start()
    cam._publish(b"\xff\xd8\xff\xd9", 4, 2)
    t.join(timeout=5.0)

    payload, seq1 = cam.jpeg_seq()
    assert seq1 == seq0 + 1
    assert payload == b"\xff\xd8\xff\xd9"
    assert woke.is_set(), "a published frame must wake the MJPEG endpoint"


def test_wait_for_frame_returns_false_when_nothing_arrives():
    """A dead camera must not wedge the request forever."""
    cam = CameraManager(bus=None, enabled=True).get(1)
    _, seq = cam.jpeg_seq()
    assert cam.wait_for_frame(seq, timeout=0.15) is False


def test_wait_for_frame_returns_immediately_if_already_behind():
    cam = CameraManager(bus=None, enabled=True).get(1)
    cam._publish(b"\xff\xd8\xff\xd9", 4, 2)
    assert cam.wait_for_frame(-1, timeout=0.0) is True


def test_filter_stats_are_exposed_on_the_camera_tile():
    mgr = CameraManager(bus=None, enabled=True,
                        settings={"filter": {"enabled": True},
                                  "picam": {"filter": True}})
    cam = mgr.get(1)
    cam._condition(_stripe(_scene(), [100], 12))
    info = cam.info()
    assert "filter" in info
    assert info["filter"]["rows_corrected"] >= 1


# ----------------------------------------------------- latency reporting ----
def test_publish_measures_capture_to_published_latency():
    """The number the whole exercise turns on. A camera can hold 30 fps while
    every frame it serves is a second old, so fps alone proves nothing."""
    import time as _time
    cam = CameraManager(bus=None, enabled=True).get(1)
    cam.connected = True

    cam._publish(b"\xff\xd8\xff\xd9", 4, 2, captured_at=_time.monotonic() - 0.250)
    assert cam.info()["latency_ms"] == pytest.approx(250, abs=40)

    # smoothed, so one hiccup does not make the reading jump
    for _ in range(60):
        cam._publish(b"\xff\xd8\xff\xd9", 4, 2, captured_at=_time.monotonic() - 0.020)
    assert cam.info()["latency_ms"] == pytest.approx(20, abs=10)


def test_publish_without_a_stamp_leaves_latency_alone():
    cam = CameraManager(bus=None, enabled=True).get(1)
    cam.connected = True
    cam._publish(b"\xff\xd8\xff\xd9", 4, 2)
    assert cam.info()["latency_ms"] == 0


def test_a_disconnected_camera_reports_no_latency():
    cam = CameraManager(bus=None, enabled=True).get(1)
    cam._latency_ms = 900.0
    assert cam.connected is False
    assert cam.info()["latency_ms"] == 0


# -------------------------------------------------- async frame delivery ----
def test_subscribers_are_woken_by_a_published_frame():
    """The MJPEG endpoint waits on this. Holding a worker thread per open
    connection instead would draw from a pool of ~8 on a Pi 5, so a few browser
    tabs could exhaust it and stall unrelated work."""
    import asyncio

    cam = CameraManager(bus=None, enabled=True).get(1)

    async def scenario():
        event = cam.subscribe()
        try:
            assert not event.is_set()
            # published from a capture thread, exactly as the real one does
            import threading
            threading.Thread(
                target=lambda: cam._publish(b"\xff\xd8\xff\xd9", 4, 2),
                daemon=True).start()
            await asyncio.wait_for(event.wait(), timeout=5.0)
            return True
        finally:
            cam.unsubscribe(event)

    assert asyncio.run(scenario()) is True


def test_unsubscribe_removes_the_waiter():
    import asyncio

    cam = CameraManager(bus=None, enabled=True).get(1)

    async def scenario():
        event = cam.subscribe()
        assert len(cam._async_waiters) == 1
        cam.unsubscribe(event)
        assert len(cam._async_waiters) == 0

    asyncio.run(scenario())
    # publishing after every waiter has gone must not raise
    cam._publish(b"\xff\xd8\xff\xd9", 4, 2)


def test_publish_survives_a_closed_event_loop():
    """A browser tab closing mid-flight must not take the capture thread down."""
    import asyncio

    cam = CameraManager(bus=None, enabled=True).get(1)

    async def scenario():
        return cam.subscribe()

    asyncio.run(scenario())          # loop is closed on return, waiter still registered
    assert len(cam._async_waiters) == 1
    cam._publish(b"\xff\xd8\xff\xd9", 4, 2)     # must not raise
    assert cam.jpeg_seq()[1] >= 1
