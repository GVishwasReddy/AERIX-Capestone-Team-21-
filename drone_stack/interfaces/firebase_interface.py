"""Firebase order source, with the same real/mock split as the other interfaces.

The AERIX Flutter app writes one document per delivery into the Firestore
``orders`` collection: recipient, destination lat/lng, a delivery token and a
``status``. That document is the only place the drone's destination exists, so
this module is the boundary where a customer's tap becomes something the flight
stack can act on.

Three implementations, chosen by ``delivery.source``:

``firestore``
    :class:`FirestoreOrders` - the real thing, via ``firebase-admin``.
``file``
    :class:`FileOrders` - watches a JSON file. This is what simulation and the
    ``scripts/inject_order.py`` test path use, so the entire delivery chain can
    be exercised without credentials or a network.
``none``
    :class:`NullOrders` - the feature is wired in but idle.

Nothing here decides whether to *fly*: an implementation only reports the
orders it can see and accepts status write-backs. Accepting an order, checking
it against the geofence and committing the aircraft are FirebaseDeliveryNode's
job, deliberately kept out of the I/O layer.
"""
from __future__ import annotations

import abc
import json
import os
import threading
import time
from pathlib import Path
from typing import Any

from drone_stack.msg import DeliveryOrder
from drone_stack.utils.logging_setup import get_logger

# Link states surfaced verbatim on the GCS delivery panel.
LINK_DISABLED = "disabled"
LINK_NO_CREDENTIALS = "no-credentials"
LINK_CONNECTING = "connecting"
LINK_ONLINE = "online"
LINK_ERROR = "error"


def _repo_root() -> Path:
    # interfaces/firebase_interface.py -> interfaces -> drone_stack -> repo
    return Path(__file__).resolve().parents[2]


def _resolve(path_str: str) -> Path:
    p = Path(path_str).expanduser()
    return p if p.is_absolute() else (_repo_root() / p)


class FirebaseInterface(abc.ABC):
    """Contract for a source of delivery orders."""

    #: Human-readable connection state, shown on the GCS.
    link: str = LINK_DISABLED
    #: Last error string, shown on the GCS when link == error.
    last_error: str = ""

    @abc.abstractmethod
    def connect(self) -> bool:
        """Open the connection. Must be safe to call repeatedly."""

    @abc.abstractmethod
    def poll(self) -> list[DeliveryOrder]:
        """Return the orders currently awaiting dispatch, oldest first."""

    def list_recent(self, limit: int = 10) -> list[DeliveryOrder]:
        """Recent orders of *any* status, newest first.

        ``poll`` deliberately returns only what is dispatchable. This is the
        wider view the dashboard shows so an order placed in the app is
        visible even when it is already delivered, cancelled, or in a state
        the drone will not act on - "I placed an order and nothing appeared"
        should never be ambiguous. Optional; default is empty.
        """
        return []

    def update_order(self, order_id: str, fields: dict[str, Any]) -> bool:
        """Write progress back onto the order. Optional; default is a no-op."""
        return False

    def close(self) -> None:
        """Release resources. Must be safe to call twice."""


def _order_from_doc(doc_id: str, data: dict[str, Any], defaults: dict[str, Any]) -> DeliveryOrder | None:
    """Map one Firestore/JSON document onto a :class:`DeliveryOrder`.

    Field names follow the Flutter ``OrderModel``. Both ``targetLng`` and
    ``targetLon`` are accepted because the app uses the Google-Maps ``lng``
    spelling while the flight stack uses ``lon`` throughout, and a silent
    mismatch here would look exactly like "the drone ignored my order".
    """
    lat = data.get("targetLat", data.get("target_lat"))
    lon = data.get("targetLng", data.get("targetLon", data.get("target_lon")))
    if lat is None or lon is None:
        return None
    try:
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        return None
    created = data.get("createdAt", data.get("created_at", ""))
    if hasattr(created, "isoformat"):          # Firestore timestamp
        created = created.isoformat()
    return DeliveryOrder(
        order_id=str(data.get("orderId", data.get("order_id", doc_id))),
        recipient_id=str(data.get("recipientId", data.get("recipient_id", ""))),
        target_lat=lat,
        target_lon=lon,
        # Deliberately NOT read from the document. The customer supplies a
        # point on a map and nothing else: height and loiter time are ours,
        # so a buggy or hostile client cannot talk the aircraft into a
        # ceiling or a hold we did not pick. (This is what the DeliveryOrder
        # docstring always claimed; the code used to let the doc win.)
        hover_alt_m=float(defaults.get("hover_alt_m", 2.0)),
        hover_seconds=float(defaults.get("hover_seconds", 15.0)),
        created_at=str(created),
        source=str(defaults.get("source", "firestore")),
        status=str(data.get("status", data.get("state", ""))),
    )


class NullOrders(FirebaseInterface):
    """No order source. The delivery panel shows the feature as disabled."""

    def __init__(self) -> None:
        self.link = LINK_DISABLED
        self.last_error = ""

    def connect(self) -> bool:
        return True

    def poll(self) -> list[DeliveryOrder]:
        return []


class FileOrders(FirebaseInterface):
    """Read orders from a local JSON file.

    Used in simulation and by ``scripts/inject_order.py``. The file holds
    either one order object or a list of them; it is re-read only when its
    mtime changes, so polling is free.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.log = get_logger("delivery.file")
        self._path = _resolve(str(config.get("file_path", "config/pending_order.json")))
        self._defaults = {
            "hover_alt_m": float(config.get("hover_altitude_m", 2.0)),
            "hover_seconds": float(config.get("hover_seconds", 15.0)),
            "source": "file",
        }
        self._mtime = 0.0
        self._cache: list[DeliveryOrder] = []
        self.link = LINK_DISABLED
        self.last_error = ""

    def connect(self) -> bool:
        self.link = LINK_ONLINE
        self.log.info("watching %s for delivery orders", self._path)
        return True

    def poll(self) -> list[DeliveryOrder]:
        try:
            mtime = self._path.stat().st_mtime
        except OSError:
            self._cache = []
            self._mtime = 0.0
            return []
        if mtime == self._mtime:
            return list(self._cache)
        self._mtime = mtime
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            self.link = LINK_ERROR
            self.last_error = f"{self._path.name}: {exc}"
            self.log.warning("could not read %s: %s", self._path, exc)
            return []
        docs = raw if isinstance(raw, list) else [raw]
        orders = []
        for i, doc in enumerate(docs):
            if not isinstance(doc, dict):
                continue
            order = _order_from_doc(f"file-{i}", doc, self._defaults)
            if order is not None:
                orders.append(order)
        self.link = LINK_ONLINE
        self.last_error = ""
        self._cache = orders
        self.log.info("loaded %d order(s) from %s", len(orders), self._path.name)
        return list(orders)

    def list_recent(self, limit: int = 10) -> list[DeliveryOrder]:
        # The file *is* the whole order book in this mode.
        self.poll()
        return list(self._cache)[:limit]

    def update_order(self, order_id: str, fields: dict[str, Any]) -> bool:
        return False


class FirestoreOrders(FirebaseInterface):
    """The real Firestore-backed order source (``firebase-admin``).

    Deliberately polls instead of using ``on_snapshot``. The listener spawns
    its own gRPC threads and reconnects on its own schedule, which makes "did
    the drone see my order?" hard to answer from a log; a poll on the node's
    own loop keeps the whole path synchronous and inspectable, and a delivery
    order is not a 10 Hz signal.

    The query filters on ``status`` only. Adding ``order_by("createdAt")``
    server-side would turn it into a composite query and Firestore would reject
    it until someone created the index by hand - so ordering is done here.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.log = get_logger("delivery.firestore")
        self._cfg = config
        self._collection = str(config.get("collection", "orders"))
        self._status_field = str(config.get("status_field", "status"))
        self._dispatch_status = str(config.get("dispatch_status", "DISPATCHED"))
        self._cred_path = _resolve(
            str(config.get("credentials_path", "config/firebase-service-account.json"))
        )
        self._project_id = str(config.get("project_id", "") or "")
        self._defaults = {
            "hover_alt_m": float(config.get("hover_altitude_m", 2.0)),
            "hover_seconds": float(config.get("hover_seconds", 15.0)),
            "source": "firestore",
        }
        self._db = None
        self._app = None
        self._lock = threading.Lock()
        self._last_complaint = 0.0
        self.link = LINK_DISABLED
        self.last_error = ""

    def _complain(self, message: str, *args: Any) -> None:
        """Log a setup problem at most once a minute.

        poll() retries connect() on every cycle, which is what lets someone
        drop the key in without restarting the stack - but it would also print
        the same error every few seconds and bury the flight log.
        """
        now = time.monotonic()
        if now - self._last_complaint < 60.0:
            self.log.debug(message, *args)
            return
        self._last_complaint = now
        self.log.error(message, *args)

    def connect(self) -> bool:
        with self._lock:
            if self._db is not None:
                return True
            try:
                import firebase_admin
                from firebase_admin import credentials, firestore
            except ImportError as exc:
                self.link = LINK_NO_CREDENTIALS
                self.last_error = (
                    "firebase-admin not installed "
                    "(pip install firebase-admin in the drone_stack venv)"
                )
                self._complain("%s: %s", self.last_error, exc)
                return False

            self.link = LINK_CONNECTING
            try:
                # Prefer an explicit service-account file; fall back to
                # GOOGLE_APPLICATION_CREDENTIALS / metadata-server ADC so a
                # Pi provisioned another way still works.
                if self._cred_path.exists():
                    cred = credentials.Certificate(str(self._cred_path))
                elif os.environ.get("GOOGLE_APPLICATION_CREDENTIALS"):
                    cred = credentials.ApplicationDefault()
                else:
                    self.link = LINK_NO_CREDENTIALS
                    self.last_error = (
                        f"no service-account key at {self._cred_path.name} "
                        "- run scripts/firebase_setup.py"
                    )
                    self._complain(
                        "Firestore disabled: no service-account key at %s. "
                        "An order placed in the app cannot reach the drone "
                        "until one is installed: Firebase console -> Project "
                        "settings -> Service accounts -> Generate new private "
                        "key, then run scripts/firebase_setup.py <key.json>. "
                        "The node keeps retrying, so no restart is needed.",
                        self._cred_path,
                    )
                    return False
                options = {"projectId": self._project_id} if self._project_id else None
                try:
                    self._app = firebase_admin.get_app("aerix-delivery")
                except ValueError:
                    self._app = firebase_admin.initialize_app(
                        cred, options, name="aerix-delivery"
                    )
                self._db = firestore.client(self._app)
            except Exception as exc:  # noqa: BLE001 - report, never crash the node
                self.link = LINK_ERROR
                self.last_error = str(exc)
                self.log.exception("Firestore connect failed")
                self._db = None
                return False
            self.link = LINK_ONLINE
            self.last_error = ""
            self.log.info(
                "Firestore connected, watching '%s' for %s='%s'",
                self._collection, self._status_field, self._dispatch_status,
            )
            return True

    def poll(self) -> list[DeliveryOrder]:
        if self._db is None and not self.connect():
            return []
        try:
            query = self._db.collection(self._collection)
            try:
                from google.cloud.firestore_v1.base_query import FieldFilter

                query = query.where(
                    filter=FieldFilter(
                        self._status_field, "==", self._dispatch_status
                    )
                )
            except ImportError:      # older client: positional form still works
                query = query.where(
                    self._status_field, "==", self._dispatch_status
                )
            snapshot = query.limit(25).get()
        except Exception as exc:  # noqa: BLE001
            self.link = LINK_ERROR
            self.last_error = str(exc)
            self.log.warning("Firestore poll failed: %s", exc)
            return []
        orders: list[DeliveryOrder] = []
        for doc in snapshot:
            order = _order_from_doc(doc.id, doc.to_dict() or {}, self._defaults)
            if order is not None:
                order.doc_id = doc.id
                orders.append(order)
        orders.sort(key=lambda o: o.created_at or "")
        self.link = LINK_ONLINE
        self.last_error = ""
        return orders

    def list_recent(self, limit: int = 10) -> list[DeliveryOrder]:
        """Every recent order, whatever its status.

        Unfiltered on purpose: this is what proves to the operator that the
        app and the drone are looking at the same collection. Ordering is done
        here rather than with order_by() so no composite index is required.
        """
        if self._db is None and not self.connect():
            return []
        try:
            snapshot = self._db.collection(self._collection).limit(50).get()
        except Exception as exc:  # noqa: BLE001
            self.link = LINK_ERROR
            self.last_error = str(exc)
            self.log.warning("Firestore listing failed: %s", exc)
            return []
        orders = []
        for doc in snapshot:
            order = _order_from_doc(doc.id, doc.to_dict() or {}, self._defaults)
            if order is not None:
                order.doc_id = doc.id
                orders.append(order)
        orders.sort(key=lambda o: o.created_at or "", reverse=True)
        return orders[:limit]

    def update_order(self, order_id: str, fields: dict[str, Any]) -> bool:
        """Merge ``fields`` into the order document.

        Written under separate ``drone*`` keys rather than by overwriting
        ``status``: the Flutter app's active-order query looks for
        ``status == 'DISPATCHED'``, so repurposing that field would make the
        customer's own order vanish from their screen the moment it took off.
        """
        if self._db is None or not order_id:
            return False
        try:
            self._db.collection(self._collection).document(order_id).set(
                fields, merge=True
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.log.warning("Firestore write-back failed for %s: %s", order_id, exc)
            self.last_error = str(exc)
            return False

    def close(self) -> None:
        self._db = None
        self.link = LINK_DISABLED


def build_order_source(config: dict[str, Any], mode: str = "real") -> FirebaseInterface:
    """Pick an implementation from ``delivery.source``.

    ``source: firestore`` in simulation is honoured - reading real orders
    without flying them is a legitimate dry run - but an unset source in sim
    defaults to the file watcher so nothing needs credentials to be tested.
    """
    if not config.get("enabled", True):
        return NullOrders()
    source = str(config.get("source", "file" if mode == "sim" else "firestore")).lower()
    if source == "firestore":
        return FirestoreOrders(config)
    if source == "file":
        return FileOrders(config)
    return NullOrders()
