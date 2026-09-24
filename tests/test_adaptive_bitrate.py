"""The adaptive-bitrate controller that keeps the video at 30 fps.

Why these exist: the controller decides, in flight, how much picture quality to
give up to keep the frame rate. Getting it wrong is not a cosmetic bug - an
oscillating controller makes the operator's view pulse between sharp and soft
every few seconds, which is worse to fly behind than a steadily soft picture.

See SKILL.md "Video: 720p30 + adaptive bitrate" for the measured behaviour these
pin down.
"""
from __future__ import annotations

import time

from drone_stack.gcs import cameras


# [scale, jpeg_quality], sharpest first - a miniature of the real ladder.
LADDER = [[1.0, 55], [1.0, 44], [1.0, 34], [0.5, 34]]
TARGET = 30.0


def _cam(rung: int = 0):
    cam = cameras._BaseCamera(1, "T", adaptive=True, ladder=LADDER,
                              target_fps=TARGET)
    cam._rung = rung
    return cam


def _evaluate(cam, delivered: float):
    """One controller evaluation at *delivered* fps; returns the new rung."""
    rung, good = cam._choose_rung(cam._rung, delivered, TARGET,
                                  len(LADDER), cam._good_ticks)
    cam._rung = max(0, min(int(rung), len(LADDER) - 1))
    cam._good_ticks = good
    return cam._rung


def test_a_single_bad_evaluation_drops_one_rung_immediately():
    """Falling behind is an emergency: by the time it shows, it is stuttering."""
    cam = _cam(rung=0)
    assert _evaluate(cam, delivered=4.0) == 1


def test_a_rung_is_never_skipped_on_the_way_down():
    """One rung per evaluation, so the softening is legible rather than a jump."""
    cam = _cam(rung=0)
    assert [_evaluate(cam, 2.0) for _ in range(3)] == [1, 2, 3]


def test_it_stops_at_the_last_rung():
    cam = _cam(rung=len(LADDER) - 1)
    assert _evaluate(cam, 1.0) == len(LADDER) - 1


def test_climbing_back_needs_four_consecutive_healthy_evaluations():
    cam = _cam(rung=2)
    cam._probe_floor = 0                      # nothing has failed yet
    assert [_evaluate(cam, 30.0) for _ in range(4)] == [2, 2, 2, 1]


def test_a_merely_adequate_link_never_climbs():
    """90% of target is good enough to hold, not good enough to bet on."""
    cam = _cam(rung=2)
    cam._probe_floor = 0
    assert [_evaluate(cam, 27.0) for _ in range(8)] == [2] * 8


def test_a_scaled_rung_publishes_the_scaled_dimensions():
    """The camera tile must report what was SENT, not what the sensor produced."""
    cv2 = __import__("cv2")
    numpy = __import__("numpy")
    cam = _cam(rung=3)                         # [0.5, 34]
    frame = numpy.zeros((720, 1280, 3), dtype=numpy.uint8)
    jpeg, out = cam._encode_frame(cv2, frame)
    assert jpeg is not None
    assert out.shape[:2] == (360, 640)


def test_a_failed_rung_is_not_re_entered_before_the_cooldown_expires():
    """The anti-oscillation guarantee - the one that matters most.

    Observed live on 2026-09-13 before the cooldown existed: the controller
    walked back up, re-flooded the link, collapsed, and repeated every few
    seconds while the operator watched the picture breathe.
    """
    cam = _cam(rung=0)
    assert _evaluate(cam, delivered=4.0) == 1
    assert cam._probe_floor == 1, "the rung that collapsed was not remembered"

    # However healthy the link now looks, rung 0 stays off-limits - it is the
    # rung that just collapsed. Eight evaluations is 16 s, twice the streak a
    # climb needs, so this fails loudly if the ceiling is not being honoured.
    assert [_evaluate(cam, 30.0) for _ in range(8)] == [1] * 8

    # Once the cooldown expires the ceiling relaxes by exactly one rung. The
    # healthy streak has already been earned during the hold, so the newly
    # unlocked rung is spent on the first evaluation after the release.
    cam._probe_t -= cameras._ADAPT_COOLDOWN_S + 1.0
    assert _evaluate(cam, 30.0) == 0
    assert cam._probe_floor == 0
