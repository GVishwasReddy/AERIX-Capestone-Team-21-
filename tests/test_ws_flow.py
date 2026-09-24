"""GCS WebSocket flow control  (gcs/ws_flow.py, 2026-09-23).

"The longer I am in the GCS the laggier the camera and every button get; a
refresh fixes it." The WebSocket half of that was an unbounded queue: the hub
sent 15 frames/s regardless of the page, and uvicorn's websockets-sansio
protocol never pauses a sender. These tests pin the two properties that keep
a slow page from building a backlog, and the delta that stops re-sending what
the page already has.
"""
from __future__ import annotations

import asyncio
import time

from drone_stack.gcs.ws_flow import FrameGate, PayloadDelta


def _run(coro):
    return asyncio.run(coro)


class TestFrameGate:
    def test_a_page_that_never_acks_is_left_free_running(self):
        # An old cached app.js has no acks. It must behave exactly as before
        # the change, not be wedged by a server upgrade.
        async def go():
            g = FrameGate(window=2, stall_s=5.0)
            t0 = time.monotonic()
            for _ in range(10):
                await g.wait_turn()
                g.next_seq()
            return time.monotonic() - t0
        assert _run(go()) < 0.1

    def test_frames_in_flight_never_exceed_the_window_while_acks_lag(self):
        # The load-bearing property: a page that renders slowly receives FEWER
        # frames, never a growing queue of them.
        async def go():
            g = FrameGate(window=2, stall_s=5.0)
            g.on_ack(0)                       # page has announced it acks
            worst = 0

            async def slow_page():
                while g.seq < 20:
                    await asyncio.sleep(0.02)
                    g.on_ack(g.seq)           # renders whatever is newest

            page = asyncio.create_task(slow_page())
            while g.seq < 20:
                await g.wait_turn()
                g.next_seq()
                worst = max(worst, g.in_flight())
            page.cancel()
            return worst
        assert _run(go()) <= 2

    def test_an_ack_releases_a_waiting_sender_promptly(self):
        async def go():
            g = FrameGate(window=1, stall_s=5.0)
            g.on_ack(0)
            g.next_seq()                      # window now full
            loop = asyncio.get_running_loop()
            loop.call_later(0.05, g.on_ack, 1)
            t0 = time.monotonic()
            await g.wait_turn()
            return time.monotonic() - t0
        assert 0.03 < _run(go()) < 0.5

    def test_an_ack_for_an_unsent_frame_cannot_open_the_window(self):
        g = FrameGate(window=2)
        g.next_seq()
        g.on_ack(999)                         # bogus / future seq
        assert g.acked == 1
        assert g.in_flight() == 0

    def test_acks_never_move_backwards(self):
        g = FrameGate(window=2)
        for _ in range(5):
            g.next_seq()
        g.on_ack(4)
        g.on_ack(2)                           # reordered / stale
        assert g.acked == 4

    def test_garbage_acks_are_ignored(self):
        g = FrameGate(window=2)
        g.next_seq()
        g.on_ack("not-a-number")
        g.on_ack(None)
        assert g.acked == 0 and not g.enabled

    def test_a_page_that_stops_acking_is_bounded_to_one_frame_per_stall(self):
        # A hidden tab pauses requestAnimationFrame, so it stops acking. The
        # server must neither flood it nor wedge the connection forever.
        #
        # TODO(human): drive a FrameGate whose page has acked once and then
        # gone silent, run the sender loop for a fixed wall-clock budget, and
        # assert how many frames it was allowed to send.
        pass


class TestPayloadDelta:
    @staticmethod
    def _con(*ids):
        return [{"id": i, "msg": f"line {i}"} for i in ids]

    def test_console_lines_are_sent_once_each(self):
        d = PayloadDelta()
        first = d.apply({"console": self._con(1, 2, 3)})
        again = d.apply({"console": self._con(1, 2, 3)})
        more = d.apply({"console": self._con(2, 3, 4, 5)})
        assert [e["id"] for e in first["console"]] == [1, 2, 3]
        assert again["console"] == []
        assert [e["id"] for e in more["console"]] == [4, 5]

    def test_an_unchanged_trail_is_not_resent(self):
        d = PayloadDelta()
        trail = [[1.0, 2.0], [1.1, 2.1]]
        assert "trail" in d.apply({"trail": list(trail)})
        assert "trail" not in d.apply({"trail": list(trail)})

    def test_a_full_trail_that_slides_is_resent(self):
        # The hub's trail is a bounded deque: once full its LENGTH stops
        # changing while its contents keep moving. Length alone would freeze
        # the map trail for the rest of the flight.
        d = PayloadDelta()
        d.apply({"trail": [[0, 0], [1, 1], [2, 2]]})
        out = d.apply({"trail": [[1, 1], [2, 2], [3, 3]]})
        assert "trail" in out

    def test_a_cleared_trail_is_resent(self):
        d = PayloadDelta()
        d.apply({"trail": [[0, 0], [1, 1]]})
        assert d.apply({"trail": []})["trail"] == []

    def test_other_keys_pass_through_untouched(self):
        d = PayloadDelta()
        p = {"telemetry": {"alt": 1.5}, "scan": {"ranges": [1, 2]}}
        assert d.apply(dict(p)) == p
