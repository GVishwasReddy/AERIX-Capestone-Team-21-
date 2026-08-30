"""Firebase Admin SDK client: listens for pending deliveries, pushes status
and telemetry updates, and reconnects on network loss.
"""
from __future__ import annotations

import asyncio
import datetime
import logging
from typing import Any, Awaitable, Callable

import firebase_admin
from firebase_admin import credentials, db

logger = logging.getLogger("firebase_client")

DeliveryCallback = Callable[[str, dict[str, Any]], Awaitable[None]]


def _now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


class FirebaseClient:
    def __init__(self, credentials_path: str, database_url: str) -> None:
        if not firebase_admin._apps:
            cred = credentials.Certificate(credentials_path)
            firebase_admin.initialize_app(cred, {"databaseURL": database_url})
        self._deliveries_ref = db.reference("/deliveries")
        self._listener = None
        self._seen_ids: set[str] = set()

    async def push_status(
        self,
        delivery_id: str,
        status: str,
        telemetry: dict[str, Any] | None = None,
        error_message: str | None = None,
    ) -> None:
        payload: dict[str, Any] = {
            "status": status,
            "status_updated_at": _now_iso(),
        }
        if telemetry is not None:
            payload["drone_telemetry"] = telemetry
        if error_message is not None:
            payload["error_message"] = error_message
        if status == "delivered":
            payload["delivery_confirmed"] = True

        await asyncio.to_thread(self._deliveries_ref.child(delivery_id).update, payload)

    async def push_telemetry(self, delivery_id: str, telemetry: dict[str, Any]) -> None:
        """Write only the telemetry block — used by the periodic in-flight stream."""
        payload = {
            "drone_telemetry": telemetry,
            "status_updated_at": _now_iso(),
        }
        await asyncio.to_thread(self._deliveries_ref.child(delivery_id).update, payload)

    def start_listener(self, on_pending: DeliveryCallback, loop: asyncio.AbstractEventLoop) -> None:
        """Stream new /deliveries entries; invoke on_pending for any with
        status == 'pending' that we have not already dispatched.
        Reconnects automatically on stream errors (the Admin SDK's Stream
        already retries the underlying connection; we additionally guard
        against dropped generator threads by restarting the listener).
        """

        def _handle(event) -> None:
            data = event.data
            if data is None:
                return
            # event.path == "/" on initial full snapshot, "/<id>" on updates
            if event.path == "/":
                if not isinstance(data, dict):
                    return
                items = data.items()
            else:
                delivery_id = event.path.lstrip("/")
                items = [(delivery_id, data)]

            for delivery_id, doc in items:
                if not isinstance(doc, dict):
                    continue
                if doc.get("status") != "pending":
                    continue
                if delivery_id in self._seen_ids:
                    continue
                self._seen_ids.add(delivery_id)
                asyncio.run_coroutine_threadsafe(on_pending(delivery_id, doc), loop)

        self._start_stream(_handle)

    def _start_stream(self, handler) -> None:
        try:
            self._listener = self._deliveries_ref.listen(handler)
        except Exception:
            logger.exception("firebase listener failed to start, retrying in 5s")
            asyncio.get_event_loop().call_later(5, self._start_stream, handler)

    def stop_listener(self) -> None:
        if self._listener is not None:
            self._listener.close()
            self._listener = None
