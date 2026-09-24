"""Per-connection flow control for the GCS WebSocket  (added 2026-09-23).

Symptom: "the longer I am in the GCS the laggier the camera and every button
get; a page refresh fixes it". Measured on the Pi, two queues that only ever
grew for the life of a connection - which is exactly why a refresh (a new
connection) cleared them:

1. The WebSocket. uvicorn's websockets-sansio protocol never pauses a sender:
   ``send()`` waits on a ``writable`` Event that is set once and never cleared,
   and ``transport.write`` buffers without limit. ``/ws`` pushed 15 frames a
   second whether or not the browser was keeping up, so a slow tab accumulated
   frames in the Pi's asyncio buffer, and every button's result came back from
   the far end of that queue.

2. The camera. Linux autotunes a socket's send buffer UP (87 KB -> 818 KB ->
   1.3 MB observed, 4 MB ceiling) and never back down. Once full, 661 KB of
   JPEG sat unsent behind a 1.83 Mbit/s link: 2.9 s of latency, and growing
   with session length. Fixed at the listener in ``server.py``
   (``TCP_NOTSENT_LOWAT``), not here.

This module holds the WebSocket half: a credit window, and a thinner that
strips what the browser already has.
"""
from __future__ import annotations

import asyncio
import time


class FrameGate:
    """At most ``window`` telemetry frames un-acknowledged by the browser.

    The page acks each frame's ``seq`` AFTER it has rendered it (from a
    requestAnimationFrame callback), so the window covers the network and the
    browser's own main thread alike. A page too slow to render 15 Hz simply
    receives fewer, always-fresh frames instead of a growing backlog.

    A page that has never acked (an old cached app.js) is left free-running,
    exactly as before, so a stale tab cannot be wedged by a server upgrade.
    """

    def __init__(self, window: int = 2, stall_s: float = 2.0) -> None:
        self.window = max(1, int(window))
        self.stall_s = float(stall_s)
        self.seq = 0
        self.acked = 0
        self.enabled = False
        self._acked_evt = asyncio.Event()

    def next_seq(self) -> int:
        self.seq += 1
        return self.seq

    def in_flight(self) -> int:
        return self.seq - self.acked

    def on_ack(self, seq) -> None:
        try:
            seq = int(seq)
        except (TypeError, ValueError):
            return
        self.enabled = True
        # Never trust an ack for a frame not yet sent; never move backwards.
        seq = min(seq, self.seq)
        if seq > self.acked:
            self.acked = seq
        self._acked_evt.set()

    async def wait_turn(self) -> None:
        """Return when the next frame may be sent.

        A browser that stops acking entirely - a hidden tab, whose rAF is
        paused - is not waited on forever: after ``stall_s`` one frame goes out
        anyway. That bounds a silent page to one frame per ``stall_s`` (it
        cannot back up) and guarantees a connection is never wedged by a lost
        ack; the next real ack restores the full rate.
        """
        if not self.enabled:
            return
        deadline = time.monotonic() + self.stall_s
        while self.in_flight() >= self.window:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self._acked_evt.clear()
            try:
                await asyncio.wait_for(self._acked_evt.wait(), remaining)
            except asyncio.TimeoutError:
                return


class PayloadDelta:
    """Remove from a frame what this connection has already delivered.

    ``console`` was the largest key in every frame (7.3 KB of 17 KB): the same
    ~60 log lines re-sent 15 times a second, which the page then discarded by
    id. ``trail`` can reach 800 points (~30 KB) and only changes when the
    aircraft moves 0.7 m. Both compete with the camera for the same Wi-Fi.

    The page keeps the last trail it was sent, and merges console lines across
    frames it coalesces, so neither is lost by being sent once.
    """

    def __init__(self) -> None:
        self.console_id = 0
        self._trail_sig = None

    def apply(self, payload: dict) -> dict:
        con = payload.get("console")
        if con is not None:
            fresh = [e for e in con if e.get("id", 0) > self.console_id]
            if fresh:
                self.console_id = max(e.get("id", 0) for e in fresh)
            payload["console"] = fresh

        trail = payload.get("trail")
        if trail is not None:
            # The trail is a bounded deque: once full its length stops
            # changing, so the ends are part of the signature too.
            sig = (len(trail),
                   tuple(trail[0]) if trail else None,
                   tuple(trail[-1]) if trail else None)
            if sig == self._trail_sig:
                del payload["trail"]
            else:
                self._trail_sig = sig
        return payload
