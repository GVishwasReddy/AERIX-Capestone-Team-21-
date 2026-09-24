#!/usr/bin/env python3
"""
Raspberry Pi 5 BLE Peripheral Service for Drone Handshake & Parcel Drop.

Security Features:
  1. HMAC-SHA256 Challenge-Response (Mutual Authentication) using a
     per-order dynamic token pulled from Firestore (see firebase_sync.py),
     not a hardcoded shared secret.
  2. Micro Geofence: phone GPS vs the DRONE's own live GPS (pulled from
     drone_stack's GCS at http://localhost:8090), with autonomous
     "follow-me" relocation (goto_gps) if the drone isn't close enough yet.
  3. Anti-spoofing temporal consistency: a single GPS reading is never
     trusted on its own (easy to spoof/replay/glitch). The drone requires
     several consecutive readings in a row that agree with each other
     (no implausible jumps) within a short rolling window before a
     position is considered verified - see _record_temporal_reading().
  4. Signed GPS payload verification - prevents GPS coordinate tampering.
  5. A RESULT characteristic the phone must read after writing DROP, so a
     successful BLE write is never confused with a successful drop - the Pi
     is the only source of truth for whether the parcel actually released.
  6. Single-use token: the order file is invalidated after a successful
     drop, and a signed local receipt is written for the dual-audit sync
     (see firebase_sync.py push-receipts).

Dependencies:
    pip install -r requirements.txt   (bluez-peripheral)

Usage:
    sudo python3 drone_ble_peripheral.py                  # requires active_order.json
    sudo python3 drone_ble_peripheral.py --dev             # bench testing, no order file
    sudo python3 drone_ble_peripheral.py --order-file /path/to/active_order.json
"""

import os
import sys
import json
import time
import hmac
import hashlib
import asyncio
import argparse
import math
import threading
import urllib.request
import urllib.error
from pathlib import Path

from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.agent import NoIoAgent
from bluez_peripheral.util import get_message_bus

# ══════════════════════════════════════════════════════════════════
#  CONSTANTS & UUIDs (Must match Flutter app)
# ══════════════════════════════════════════════════════════════════

SERVICE_UUID     = "12345678-1234-5678-1234-567812345678"
NONCE_CHAR_UUID  = "12345678-1234-5678-1234-567812345679"  # Read  — Challenge Nonce
AUTH_CHAR_UUID   = "12345678-1234-5678-1234-56781234567a"  # R/W   — Auth HMAC + Mutual ACK
DROP_CHAR_UUID   = "12345678-1234-5678-1234-56781234567b"  # Write — Drop Command
GPS_CHAR_UUID    = "12345678-1234-5678-1234-56781234567c"  # Write — GPS Coordinates
RESULT_CHAR_UUID = "12345678-1234-5678-1234-56781234567d"  # Read  — Outcome of last DROP attempt

# ── GCS (drone_stack) integration ──
GCS_BASE_URL = os.environ.get("GCS_BASE_URL", "http://localhost:8090")
GCS_HTTP_TIMEOUT_S = 2.0

# ── Geofence radius ──
GEOFENCE_MICRO_RADIUS_M = 10.0   # phone vs drone's own live GPS
FOLLOW_ME_COOLDOWN_S = 8.0       # don't spam goto_gps if the phone retries fast

# ── Anti-spoofing temporal consistency ──
# A single GPS reading is cheap to fake (fake-GPS apps, replay, multipath
# glitches). Require several consecutive readings that agree with each
# other before a position is trusted for a geofence decision.
TEMPORAL_READINGS_REQUIRED = 5   # consecutive consistent readings needed
TEMPORAL_WINDOW_S = 12.0         # readings older than this fall out of the window
TEMPORAL_MAX_JUMP_M = 20.0       # bigger gap between consecutive readings = not the same person standing still

# ── RESULT characteristic outcome codes (ASCII, read after writing DROP) ──
RESULT_PENDING        = b"PENDING"
RESULT_OK             = b"OK"
RESULT_AUTH_FAIL      = b"AUTH_FAIL"
RESULT_GEOFENCE_FAIL  = b"GEOFENCE_FAIL"
RESULT_SIGNATURE_FAIL = b"SIGNATURE_FAIL"
# The drone has not locked onto a person at the drop point yet (or the window
# after the lock has closed). The token is NOT spent - retry the DROP.
RESULT_NOT_READY      = b"NOT_READY"

# ── Phone RSSI sampling (locating the recipient among several people) ──
RSSI_SAMPLE_S = 0.25             # controller read period
RSSI_POST_S = 0.5                # batch period to the GCS
RSSI_MAX_FAILS = 12              # ~3 s of no reading = the link is gone
RSSI_MAX_RUN_S = 20 * 60.0

_HERE = Path(__file__).resolve().parent
_RECEIPTS_PENDING_DIR = _HERE / "receipts" / "pending"

# ── Dev-only fallback (bench testing without Firestore/active_order.json) ──
_DEV_SECRET_KEY = b"MY_SUPER_SECRET_KEY"
_DEV_TARGET_LAT = 13.0827
_DEV_TARGET_LNG = 80.2707
_DEV_ORDER_ID = "DEV_BENCH_ORDER"


# ══════════════════════════════════════════════════════════════════
#  HAVERSINE DISTANCE CALCULATION
# ══════════════════════════════════════════════════════════════════

def haversine_distance(lat1, lng1, lat2, lng2):
    """
    Calculate the great-circle distance between two GPS points
    using the Haversine formula. Returns distance in meters.
    """
    R = 6371000  # Earth's radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lng2 - lng1)

    a = (math.sin(d_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


# ══════════════════════════════════════════════════════════════════
#  ACTIVE ORDER LOADING (per-delivery dynamic token + target GPS)
# ══════════════════════════════════════════════════════════════════

class ActiveOrder:
    """The order this flight was dispatched for: dynamic token + target GPS.

    Written by ``firebase_sync.py pull-order`` before takeoff (the "GCS
    flashes token + target coordinates into the Pi's memory" step in the
    design doc). Loaded at startup, then replaced at runtime by _order_watch
    whenever the GCS takes on a new order (2026-09-24; it used to need a
    process restart per delivery, which nobody remembered to do).
    """

    def __init__(self, order_id: str, token: bytes, target_lat: float, target_lng: float):
        self.order_id = order_id
        self.token = token
        self.target_lat = target_lat
        self.target_lng = target_lng

    @classmethod
    def load(cls, path: Path) -> "ActiveOrder":
        data = json.loads(path.read_text(encoding="utf-8"))
        token_hex = data["deliveryToken"]
        return cls(
            order_id=data["orderId"],
            token=bytes.fromhex(token_hex),
            target_lat=float(data["targetLat"]),
            target_lng=float(data["targetLng"]),
        )

    @classmethod
    def dev_fallback(cls) -> "ActiveOrder":
        return cls(_DEV_ORDER_ID, _DEV_SECRET_KEY, _DEV_TARGET_LAT, _DEV_TARGET_LNG)


def _invalidate_order_file(path: Path) -> None:
    """Single-use token: rename the order file so it can't be pulled/reused
    for a second delivery after this one completes."""
    if not path.exists():
        return
    used_path = path.with_name(f"{path.stem}.used_{int(time.time())}{path.suffix}")
    try:
        path.rename(used_path)
        print(f"[ORDER] Invalidated {path.name} -> {used_path.name} (token expired)")
    except OSError as exc:
        print(f"  ⚠️ Could not invalidate order file: {exc}")


# ══════════════════════════════════════════════════════════════════
#  GCS (drone_stack) HTTP HELPERS
# ══════════════════════════════════════════════════════════════════

def gcs_get(path: str) -> dict | None:
    url = f"{GCS_BASE_URL}{path}"
    try:
        with urllib.request.urlopen(url, timeout=GCS_HTTP_TIMEOUT_S) as response:
            return json.loads(response.read().decode())
    except (urllib.error.URLError, TimeoutError, ValueError) as exc:
        print(f"  ❌ [GCS] GET {path} failed: {exc}")
        return None


def gcs_post(path: str, payload: dict) -> dict | None:
    url = f"{GCS_BASE_URL}{path}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=GCS_HTTP_TIMEOUT_S) as response:
            return json.loads(response.read().decode())
    except (urllib.error.URLError, TimeoutError, ValueError) as exc:
        print(f"  ❌ [GCS] POST {path} failed: {exc}")
        return None


def gcs_handshake_open() -> bool:
    """May the parcel be released now? The navigator only opens the window
    once the camera has locked onto a person at the drop point (see
    NavigationNode._handshake_open). Fails CLOSED: no answer is a no."""
    state = gcs_get("/api/state")
    if not isinstance(state, dict):
        return False
    mission = state.get("mission")
    return isinstance(mission, dict) and mission.get("handshake_open") is True


def _post_async(payload: dict) -> None:
    """Fire-and-forget POST, off the BLE/D-Bus event loop."""
    threading.Thread(target=gcs_post, args=("/api/command", payload), daemon=True).start()


class RssiSampler(threading.Thread):
    """Reads the live connection's RSSI and posts it to the GCS in batches.

    The navigator pairs each sample with where the aircraft was when it was
    measured (hence the wall-clock timestamp) and uses them to tell WHICH of
    the people under the drone is holding the phone."""

    def __init__(self, order_id: str, device_path: str) -> None:
        super().__init__(daemon=True)
        self.order_id = order_id
        self.device_path = device_path
        self.stop_evt = threading.Event()

    def run(self) -> None:
        try:
            from rssi_probe import RssiProbe, addr_from_device_path
        except Exception as exc:  # noqa: BLE001
            print(f"  ⚠️ [RSSI] probe unavailable: {exc}")
            return
        where = addr_from_device_path(self.device_path) or (0, None)
        probe = RssiProbe(*where)
        print(f"  📶 [RSSI] sampling hci{where[0]} {where[1] or '(single LE link)'}")
        batch, fails, n = [], 0, 0
        started = last_post = time.monotonic()
        try:
            while not self.stop_evt.is_set() and time.monotonic() - started < RSSI_MAX_RUN_S:
                v = probe.read()
                if v is None:
                    fails += 1
                    if fails >= RSSI_MAX_FAILS:
                        break
                else:
                    fails = 0
                    batch.append([round(time.time(), 3), v])
                    n += 1
                if batch and time.monotonic() - last_post >= RSSI_POST_S:
                    gcs_post("/api/command", {"cmd": "ble_phone_signal", "params": {
                        "order_id": self.order_id, "samples": batch}})
                    batch, last_post = [], time.monotonic()
                self.stop_evt.wait(RSSI_SAMPLE_S)
        finally:
            probe.close()
            print(f"  📶 [RSSI] sampler stopped after {n} samples")


def get_drone_live_gps() -> tuple[float, float] | None:
    """Ask drone_stack for the drone's own current GPS (Gate 3 needs this -
    the doc's "Pi asks the Pixhawk for the drone's OWN Live Satellite GPS").

    The real GCS has no ``/api/telemetry`` route - the one-shot snapshot is
    ``GET /api/state`` (see gcs/server.py), and its GPS fields live nested
    under a ``"position"`` key as ``lat``/``lon``/``fix`` (see
    GcsHub.build_payload), not a flat ``has_fix``/``lat``/``lon``. Without
    this the micro-geofence could never see a live position at all - always
    RESULT_GEOFENCE_FAIL, regardless of how close the phone actually is.
    """
    state = gcs_get("/api/state")
    if state is None:
        return None
    position = state.get("position") or {}
    if not position.get("fix", False):
        print("  ⚠️ [GCS] drone has no 3D GPS fix yet")
        return None
    lat, lon = position.get("lat"), position.get("lon")
    if lat is None or lon is None:
        return None
    return float(lat), float(lon)


def request_follow_me(lat: float, lon: float) -> bool:
    """Command drone_stack to autonomously fly to the phone's GPS (Gate 3
    "Autonomous Relocation / Follow-Me" in the design doc)."""
    result = gcs_post("/api/command", {"cmd": "goto_gps", "params": {"lat": lat, "lon": lon}})
    ok = bool(result and result.get("ok"))
    msg = result.get("message") if result else "no response from GCS"
    print(f"  🚁 [FOLLOW-ME] goto_gps({lat:.7f}, {lon:.7f}) -> {'accepted' if ok else 'REJECTED'}: {msg}")
    return ok


def gcs_set_servo(channel: int, pwm: int) -> bool:
    result = gcs_post("/api/command", {"cmd": "set_servo", "params": {"channel": channel, "pwm": pwm}})
    return bool(result and result.get("ok"))


def gcs_payload_cfg() -> dict:
    """Payload servo channel + lock/release PWM, asked of the GCS.

    This file used to hardcode the channel (9) and both pulse widths
    (1410/1100), which made it a THIRD copy of numbers that config/*.yaml and
    the GCS UI already own. When the servos moved from AUX1/AUX2 to AUX5/AUX6
    on 2026-09-06, this was the copy that would have gone on commanding the
    dead pin - a delivery release that silently does nothing, with the receipt
    still written and the token still burned.

    GcsHub ships the whole `payload` block in every /api/state frame, so read
    it from there. Falls back to the config defaults if the GCS is unreachable;
    a failed lookup must not become a command on the wrong channel.
    """
    state = gcs_get("/api/state")
    pl = (state or {}).get("payload") or {}
    if not pl:
        print("  ⚠️ [GCS] no payload config in /api/state - using defaults")
    return {
        "channel": int(pl.get("channel", 12)),
        "release_us": int(pl.get("release_us", 1410)),
        "lock_us": int(pl.get("lock_us", 1100)),
    }


# ══════════════════════════════════════════════════════════════════
#  SIGNED LOCAL RECEIPT (drone-side half of Dual Auditing)
# ══════════════════════════════════════════════════════════════════

def write_drone_receipt(order: ActiveOrder, lat: float, lng: float) -> None:
    """Drone-side signed 'Delivery Success' receipt, stored offline for
    firebase_sync.py to upload once the drone is back on warehouse Wi-Fi."""
    timestamp = time.time()
    fields = f"{order.order_id}|{timestamp:.3f}|{lat:.8f}|{lng:.8f}"
    signature = hmac.new(order.token, fields.encode("utf-8"), hashlib.sha256).hexdigest()
    receipt = {
        "orderId": order.order_id,
        "source": "drone",
        "timestamp": timestamp,
        "lat": lat,
        "lng": lng,
        "token": order.token.hex(),
        "signature": signature,
    }
    _RECEIPTS_PENDING_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _RECEIPTS_PENDING_DIR / f"{order.order_id}_{int(timestamp)}.json"
    out_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(f"  🧾 [RECEIPT] wrote {out_path.name} (pending sync to Firestore)")


# ══════════════════════════════════════════════════════════════════
#  BLE GATT SERVICE
# ══════════════════════════════════════════════════════════════════

class DroneService(Service):
    def __init__(self, order: ActiveOrder, order_file: Path | None):
        super().__init__(SERVICE_UUID, True)
        self.order = order
        self.order_file = order_file

        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        self.gps_verified = False       # micro geofence passed (phone vs drone live GPS)
        self.received_gps_lat = None
        self.received_gps_lng = None
        self.gps_readings = []          # rolling window of (monotonic_ts, lat, lng) - anti-spoofing
        self.ack_hmac = bytes(20)       # Mutual auth ACK — initially empty
        self.last_result = RESULT_PENDING
        self._last_follow_me_ts = 0.0
        self._delivered = False         # true once this order's token is spent
        self._rssi: RssiSampler | None = None

        print(f"[Pi 5] Order: {order.order_id}")
        print(f"[Pi 5] Generated initial 16-byte Nonce: {self.current_nonce.hex()}")
        print(f"[Pi 5] Order Target: ({order.target_lat}, {order.target_lng})  "
              f"micro<= {GEOFENCE_MICRO_RADIUS_M}m  "
              f"({TEMPORAL_READINGS_REQUIRED} consistent readings required)")

    def swap_order(self, order: ActiveOrder, order_file: Path | None) -> None:
        """Adopt a newly dispatched order without restarting the process.

        Called on the asyncio loop (same thread as every characteristic
        handler), so no handler can observe a half-swapped state. Any session
        in progress was authenticated against the OLD token and is dropped:
        the app must read a fresh nonce, exactly as after a reconnect."""
        self.order = order
        self.order_file = order_file
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        self.gps_verified = False
        self.received_gps_lat = None
        self.received_gps_lng = None
        self.gps_readings = []
        self.ack_hmac = bytes(20)
        self.last_result = RESULT_PENDING
        self._delivered = False
        print(f"\n[ORDER] Hot-swapped to order {order.order_id} "
              f"target ({order.target_lat}, {order.target_lng}) - token refreshed")

    # ──────────────────────────────────────────────────────────────
    #  1. NONCE CHARACTERISTIC (READ ONLY) — Challenge
    # ──────────────────────────────────────────────────────────────
    @characteristic(NONCE_CHAR_UUID, CharacteristicFlags.READ)
    def nonce_characteristic(self, options):
        # Refresh challenge nonce on every read for anti-replay security
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        self.gps_verified = False
        self.gps_readings = []  # new session - anti-spoofing window starts over
        self.ack_hmac = bytes(20)
        self.last_result = RESULT_PENDING
        print(f"\n[BLE Read Nonce] New Challenge Nonce: {self.current_nonce.hex()}")
        return self.current_nonce

    # ──────────────────────────────────────────────────────────────
    #  2. AUTH CHARACTERISTIC (READ + WRITE) — Mutual Authentication
    # ──────────────────────────────────────────────────────────────
    @characteristic(AUTH_CHAR_UUID, CharacteristicFlags.READ | CharacteristicFlags.WRITE)
    def auth_characteristic(self, options):
        # READ: Return the ACK HMAC so the app can verify drone identity
        print(f"[BLE Read Auth] App reading ACK for mutual auth verification")
        return self.ack_hmac

    @auth_characteristic.setter
    def auth_characteristic(self, value, options):
        print(f"[BLE Write Auth] Received HMAC Signature ({len(bytes(value))} bytes): {bytes(value).hex()}")

        if self._delivered:
            print("  ⚠️ AUTH REJECTED: this order's token has already been used.")
            self.is_authenticated = False
            return

        # Calculate expected HMAC-SHA256(Nonce, OrderToken) truncated to 20 bytes
        expected_hmac = hmac.new(self.order.token, self.current_nonce, hashlib.sha256).digest()[:20]

        if hmac.compare_digest(bytes(value), expected_hmac):
            self.is_authenticated = True
            print("  ✅ AUTHENTICATION SUCCESSFUL! Drone unlocked.")

            # ── Generate Mutual Auth ACK ──
            # ACK = HMAC(Nonce + "ACK", OrderToken) truncated to 20 bytes
            ack_payload = self.current_nonce + b"ACK"
            self.ack_hmac = hmac.new(self.order.token, ack_payload, hashlib.sha256).digest()[:20]
            print(f"  🔑 Mutual Auth ACK generated: {self.ack_hmac.hex()}")
            self._start_rssi(options)
        else:
            self.is_authenticated = False
            self.ack_hmac = bytes(20)
            print("  ❌ AUTHENTICATION FAILED! Invalid HMAC signature.")
            print(f"     Expected: {expected_hmac.hex()}")
            print(f"     Received: {bytes(value).hex()}")

    # ──────────────────────────────────────────────────────────────
    #  3. GPS CHARACTERISTIC (WRITE ONLY) — Micro Geofence
    # ──────────────────────────────────────────────────────────────
    @characteristic(GPS_CHAR_UUID, CharacteristicFlags.WRITE)
    def gps_characteristic(self, options):
        return bytes()

    @gps_characteristic.setter
    def gps_characteristic(self, value, options):
        raw = bytes(value)
        print(f"\n[BLE Write GPS] Received GPS payload ({len(raw)} bytes)")

        if not self.is_authenticated:
            print("  ⚠️ GPS REJECTED: Connection is not authenticated!")
            self.last_result = RESULT_AUTH_FAIL
            return

        try:
            # Parse the GPS payload: [gpsString bytes] + [0x00] + [16-byte HMAC]
            separator_index = raw.index(0x00)
            gps_bytes = raw[:separator_index]
            received_signature = raw[separator_index + 1:]

            gps_string = gps_bytes.decode("utf-8")
            print(f"  📍 GPS String: {gps_string}")

            # ── Verify GPS HMAC Signature ──
            # Expected: HMAC(nonce + gpsString, token) truncated to 16 bytes
            gps_hmac_payload = self.current_nonce + gps_bytes
            expected_gps_hmac = hmac.new(self.order.token, gps_hmac_payload, hashlib.sha256).digest()[:16]

            if not hmac.compare_digest(received_signature, expected_gps_hmac):
                print("  ❌ GPS SIGNATURE INVALID! Possible GPS data tampering.")
                self.gps_verified = False
                self.last_result = RESULT_SIGNATURE_FAIL
                return

            print("  🔐 GPS signature verified — data is authentic.")

            # ── Parse lat, lng ──
            parts = gps_string.split(",")
            lat = float(parts[0])
            lng = float(parts[1])
            self.received_gps_lat = lat
            self.received_gps_lng = lng
            # Signed, so it is the recipient's phone: let the navigator use it
            # to find them (the RSSI search starts from here).
            _post_async({"cmd": "ble_phone_fix", "params": {
                "order_id": self.order.order_id, "lat": lat, "lon": lng, "t": time.time()}})

            # ── Anti-spoofing: don't trust a single reading ──
            self._record_temporal_reading(lat, lng)
            if len(self.gps_readings) < TEMPORAL_READINGS_REQUIRED:
                self.gps_verified = False
                self.last_result = RESULT_GEOFENCE_FAIL
                print(f"  ⏳ [Temporal Consistency] {len(self.gps_readings)}/"
                      f"{TEMPORAL_READINGS_REQUIRED} consistent readings so far — "
                      f"need more before trusting this position.")
                return

            self._check_geofences(lat, lng)

        except Exception as e:
            print(f"  ❌ GPS PARSE ERROR: {e}")
            self.gps_verified = False
            self.last_result = RESULT_GEOFENCE_FAIL

    def _record_temporal_reading(self, lat: float, lng: float) -> None:
        """Anti-spoofing temporal consistency: a single GPS reading is cheap
        to fake (fake-GPS apps, a replayed value, a one-off multipath
        glitch). Require several consecutive readings, arriving within a
        short rolling window, that stay close to each other - a real person
        standing still naturally reports nearly the same spot each time; an
        attacker now has to fool the check repeatedly instead of once.

        An implausible jump between consecutive readings (further than a
        person could realistically move between BLE writes) is treated as
        a broken streak: the window restarts from this newest reading
        rather than being averaged in, so a spoofed/glitched outlier can't
        drag a otherwise-consistent streak across the line.
        """
        now = time.monotonic()
        self.gps_readings = [r for r in self.gps_readings if now - r[0] <= TEMPORAL_WINDOW_S]

        if self.gps_readings:
            _, prev_lat, prev_lng = self.gps_readings[-1]
            jump = haversine_distance(prev_lat, prev_lng, lat, lng)
            if jump > TEMPORAL_MAX_JUMP_M:
                print(f"  ⚠️ [Temporal Consistency] implausible jump ({jump:.1f}m since "
                      f"the last reading) — treating as spoofed/glitched, restarting "
                      f"the consistency window.")
                self.gps_readings = []

        self.gps_readings.append((now, lat, lng))

    def _check_geofences(self, phone_lat: float, phone_lng: float) -> None:
        # ── Micro Geofence — phone vs the DRONE's own live GPS ──
        # Only reached once TEMPORAL_READINGS_REQUIRED consistent readings
        # are in hand (see gps_characteristic.setter).
        drone_pos = get_drone_live_gps()
        if drone_pos is None:
            self.gps_verified = False
            self.last_result = RESULT_GEOFENCE_FAIL
            print("  ❌ GEOFENCE UNAVAILABLE — could not reach drone_stack telemetry.")
            print("Handshake Unsuccessful")
            return

        drone_lat, drone_lng = drone_pos
        micro_distance = haversine_distance(phone_lat, phone_lng, drone_lat, drone_lng)
        print(f"  📏 [Micro Geofence] phone vs drone's live position: {micro_distance:.2f}m "
              f"(max {GEOFENCE_MICRO_RADIUS_M}m)")

        if micro_distance <= GEOFENCE_MICRO_RADIUS_M:
            self.gps_verified = True
            self.last_result = RESULT_OK
            print(f"  ✅ GEOFENCE PASSED! Drone is directly over the recipient.")
            print("Handshake Successful")
            return

        # Geofence failed -> Autonomous Relocation ("Follow-Me")
        self.gps_verified = False
        self.last_result = RESULT_GEOFENCE_FAIL
        print("Handshake Unsuccessful")
        now = time.monotonic()
        if now - self._last_follow_me_ts >= FOLLOW_ME_COOLDOWN_S:
            self._last_follow_me_ts = now
            print(f"  🔄 GEOFENCE FAILED ({micro_distance:.1f}m away) — "
                  f"Commanding autonomous relocation...")
            request_follow_me(phone_lat, phone_lng)
        else:
            print(f"  🔄 GEOFENCE still failing ({micro_distance:.1f}m) — relocation already "
                  f"in progress, waiting.")

    # ──────────────────────────────────────────────────────────────
    #  4. DROP CHARACTERISTIC (WRITE ONLY) — Release Parcel
    # ──────────────────────────────────────────────────────────────
    @characteristic(DROP_CHAR_UUID, CharacteristicFlags.WRITE)
    def drop_characteristic(self, options):
        return bytes()

    @drop_characteristic.setter
    def drop_characteristic(self, value, options):
        print(f"\n[BLE Write Drop] Received Drop Command ({len(bytes(value))} bytes): {bytes(value).hex()}")

        # ── Security Gate 1: Must be authenticated ──
        if not self.is_authenticated:
            print("  ⚠️ DROP REJECTED: Connection is not authenticated!")
            self.last_result = RESULT_AUTH_FAIL
            return

        # ── Security Gate 2: Must have passed the micro geofence ──
        if not self.gps_verified:
            print("  ⚠️ DROP REJECTED: Geofence verification not passed!")
            print(f"     Micro geofence: FAIL")
            self.last_result = RESULT_GEOFENCE_FAIL
            return

        # ── Security Gate 4: Verify DROP HMAC signature ──
        # Enhanced DROP HMAC: HMAC(nonce + gpsString + "DROP", token) truncated to 20 bytes
        gps_string = f"{self.received_gps_lat:.8f},{self.received_gps_lng:.8f}"
        gps_bytes = gps_string.encode("utf-8")
        drop_payload = self.current_nonce + gps_bytes + b"DROP"
        expected_drop_hmac = hmac.new(self.order.token, drop_payload, hashlib.sha256).digest()[:20]

        if not hmac.compare_digest(bytes(value), expected_drop_hmac):
            print("  ❌ DROP REJECTED: Invalid Drop HMAC payload.")
            print(f"     Expected: {expected_drop_hmac.hex()}")
            print(f"     Received: {bytes(value).hex()}")
            self.last_result = RESULT_SIGNATURE_FAIL
            return

        # ── Gate 5: the drone must have locked onto a person below ──
        # Checked last so a bad signature is still reported as one. Nothing
        # is spent here: the app retries the same DROP until the navigator
        # opens the window (or the flight gives up and returns home).
        if not gcs_handshake_open():
            print("  ⏳ DROP DEFERRED: no person locked at the drop point yet "
                  "(or the window closed) - token NOT spent, retry.")
            self.last_result = RESULT_NOT_READY
            return

        print("\n" + "=" * 60)
        print("  🪂 ALL SECURITY GATES PASSED!")
        print(f"     ✅ Gate 1: HMAC Authentication   — PASSED")
        print(f"     ✅ Gate 2: Micro Geofence ({GEOFENCE_MICRO_RADIUS_M}m)  — PASSED")
        print(f"     ✅ Gate 3: Drop HMAC Signature   — PASSED")
        print(f"     ✅ Gate 4: Person locked below   — PASSED")
        print()
        print("  🪂 TRIGGERING SERVO/SOLENOID RELEASE...")
        print("=" * 60 + "\n")

        self._delivered = True
        self.last_result = RESULT_OK

        # Tell drone_stack the handshake is done - NavigationNode cuts its
        # drop-point hold short a few seconds after this instead of waiting
        # out the full hover timeout (see gcs/hub.py's
        # _on_ble_delivery_result). Fired here, at the security-gate
        # decision, not after the servo pulse below: the release gate having
        # been verified (auth + geofence + DROP HMAC) is the "delivery
        # complete" signal regardless of payload-servo hardware state.
        gcs_post("/api/command", {
            "cmd": "ble_delivery_result",
            "params": {"order_id": self.order.order_id, "success": True},
        })

        # Run the servo pulse + receipt + token invalidation off the asyncio
        # loop (it includes a multi-second wait for the servo to clear) so it
        # never blocks the BLE/D-Bus event loop.
        threading.Thread(target=self._release_and_finalize, daemon=True).start()

    def _start_rssi(self, options) -> None:
        """One sampler per authenticated connection."""
        path = ""
        try:
            path = str((options or {}).get("device", ""))
        except Exception:  # noqa: BLE001
            pass
        if self._rssi is not None and self._rssi.is_alive():
            if self._rssi.device_path == path:
                return
            self._rssi.stop_evt.set()
        self._rssi = RssiSampler(self.order.order_id, path)
        self._rssi.start()

    def _release_and_finalize(self) -> None:
        """Hardware release + dual-audit receipt + token expiry.

        Runs on a background thread (see drop_characteristic.setter) so the
        multi-second servo settle time never blocks the BLE event loop.
        """
        # Snapshot first: _order_watch may swap self.order for the NEXT
        # dispatch while this thread sleeps, and the receipt + token expiry
        # must stay with the order that was actually delivered.
        order, order_file = self.order, self.order_file
        lat, lng = self.received_gps_lat, self.received_gps_lng
        # Channel and both pulse widths come from the GCS (config is the
        # single source of truth) - never hardcoded here. See gcs_payload_cfg.
        _cfg = gcs_payload_cfg()
        out_ch = _cfg["channel"]
        open_pwm = _cfg["release_us"]
        close_pwm = _cfg["lock_us"]

        if gcs_set_servo(out_ch, open_pwm):
            print(f"[HARDWARE] Servo OPENED (PWM {open_pwm}) -> Parcel Released!")
        else:
            print("  ❌ [HARDWARE ERROR] Failed to send open command.")

        print("[HARDWARE] Waiting 2 seconds before resetting servo...")
        time.sleep(2)

        if gcs_set_servo(out_ch, close_pwm):
            print(f"[HARDWARE] Servo CLOSED (PWM {close_pwm}) -> Ready for next drop.")
        else:
            print("  ❌ [HARDWARE ERROR] Failed to send close/reset command.")

        # Dual auditing: drone-side signed receipt, queued for later upload.
        write_drone_receipt(order, lat, lng)

        # Single-use token: this order's token can never be replayed.
        if order_file is not None:
            _invalidate_order_file(order_file)

        print("[STATE] Delivery complete — order token expired, ready for next flight.\n")

    # ──────────────────────────────────────────────────────────────
    #  5. RESULT CHARACTERISTIC (READ ONLY) — Outcome of last DROP attempt
    # ──────────────────────────────────────────────────────────────
    @characteristic(RESULT_CHAR_UUID, CharacteristicFlags.READ)
    def result_characteristic(self, options):
        print(f"[BLE Read Result] App reading last DROP outcome: {self.last_result.decode()}")
        return self.last_result


# ══════════════════════════════════════════════════════════════════
#  MAIN — Start BLE Server
# ══════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════
#  ORDER HOT-RELOAD
# ══════════════════════════════════════════════════════════════════
#
# The token used to be pulled ONCE, at boot (ble_handshake_start.sh). An order
# dispatched after boot - i.e. every real flight - was then authenticated
# against the previous order's token, and the app got AUTHENTICATION FAILED
# with nothing to say why (2026-09-24 13:40 flight: GCS on one order, BLE on
# the 13:38 one). Now the peripheral follows the order the GCS is flying.

ORDER_WATCH_S = 3.0
ORDER_PULL_RETRY_S = 30.0
_DEFAULT_CRED = "/home/pi/drone_stack/config/firebase-service-account.json"


def _gcs_order_id() -> str | None:
    """The order the GCS currently holds, or None. Deliberately quiet (unlike
    gcs_get): the GCS restarts often and a 3 s poll must not flood the log."""
    try:
        with urllib.request.urlopen(f"{GCS_BASE_URL}/api/state",
                                    timeout=GCS_HTTP_TIMEOUT_S) as response:
            state = json.loads(response.read().decode())
    except (urllib.error.URLError, TimeoutError, ValueError, OSError):
        return None
    delivery = state.get("delivery") if isinstance(state, dict) else None
    oid = delivery.get("order_id") if isinstance(delivery, dict) else None
    return str(oid) if oid else None


def _spent_order_ids(order_file: Path) -> set[str]:
    """Every order whose token was already used for a DROP on this aircraft,
    from the active_order.used_<ts>.json files _invalidate_order_file leaves."""
    spent: set[str] = set()
    for path in order_file.parent.glob(f"{order_file.stem}.used_*{order_file.suffix}"):
        try:
            spent.add(json.loads(path.read_text(encoding="utf-8"))["orderId"])
        except (OSError, ValueError, KeyError):
            continue
    return spent


def _pull_order_sync(cred: str, order_id: str) -> Path | None:
    """firebase_sync.pull_order, off the event loop. Returns the file it wrote."""
    try:
        import firebase_sync
        firebase_sync._init_app(cred)
        if firebase_sync.pull_order(order_id) != 0:
            return None
        return firebase_sync._ACTIVE_ORDER_PATH
    except Exception as exc:  # firebase/network errors must never kill BLE
        print(f"  ❌ [ORDER] pull of {order_id} failed: {exc}")
        return None


async def _order_watch(service: "DroneService", order_file: Path, cred: str) -> None:
    spent = _spent_order_ids(order_file)
    failed_at: dict[str, float] = {}
    print(f"[ORDER] Watching GCS for new orders every {ORDER_WATCH_S:.0f}s "
          f"({len(spent)} spent order id(s) on record)")
    while True:
        await asyncio.sleep(ORDER_WATCH_S)
        if service._delivered:
            spent.add(service.order.order_id)
        oid = await asyncio.to_thread(_gcs_order_id)
        if not oid or oid == service.order.order_id or oid in spent:
            continue
        if time.monotonic() - failed_at.get(oid, -math.inf) < ORDER_PULL_RETRY_S:
            continue
        print(f"\n[ORDER] GCS is on order {oid}, BLE holds {service.order.order_id} "
              f"- pulling the new token")
        path = await asyncio.to_thread(_pull_order_sync, cred, oid)
        try:
            new = ActiveOrder.load(path) if path is not None else None
        except (OSError, ValueError, KeyError) as exc:
            print(f"  ❌ [ORDER] pulled file unreadable: {exc}")
            new = None
        if new is None or new.order_id != oid:
            failed_at[oid] = time.monotonic()
            continue
        service.swap_order(new, path)


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Drone BLE handshake peripheral")
    parser.add_argument(
        "--order-file", type=Path, default=_HERE / "active_order.json",
        help="path to the active_order.json written by firebase_sync.py pull-order",
    )
    parser.add_argument(
        "--max-order-age-s",
        type=float,
        default=float(os.environ.get("AERIX_BLE_MAX_ORDER_AGE_S", 6 * 3600)),
        help="refuse an active_order.json older than this many seconds "
             "(0 disables the check). A deliveryToken is single-use and issued "
             "per flight, so a stale file means guaranteed HMAC failures.",
    )
    parser.add_argument(
        "--cred", default=os.environ.get("AERIX_FIREBASE_CRED", _DEFAULT_CRED),
        help="Firebase service-account JSON used to pull a newly dispatched "
             "order's token at runtime (see _order_watch)",
    )
    parser.add_argument(
        "--dev", action="store_true",
        help="bench-testing fallback: use a hardcoded token/target instead of "
             "requiring active_order.json (INSECURE — do not use for a real delivery)",
    )
    return parser.parse_args(argv)


async def main():
    args = _parse_args()

    # A delivery token is single-use and issued per flight, so an order file is
    # only meaningful for as long as the flight it was pulled for. Age matters
    # because the pull at boot is allowed to fail quietly ("no DISPATCHED order
    # available - this is normal when idle"), and without this check the
    # peripheral would then fall back to whatever active_order.json happened to
    # be left on disk from a previous day and advertise it as "SECURE per-order
    # mode". That is exactly what happened on 2026-08-30: a token pulled on
    # 08-27 was served to the app three days later, and every handshake failed
    # HMAC authentication with no indication of why.
    #
    # A stale file is therefore treated as no file at all.
    stale_age = None
    if args.order_file.exists() and args.max_order_age_s > 0:
        age = time.time() - args.order_file.stat().st_mtime
        if age > args.max_order_age_s:
            stale_age = age

    if stale_age is not None:
        hours = stale_age / 3600.0
        print(f"❌ Order file {args.order_file} is {hours:.1f} h old "
              f"(limit {args.max_order_age_s / 3600.0:.1f} h) - REFUSING it.")
        print("   Its deliveryToken is single-use and almost certainly no longer")
        print("   the one the app holds, so every handshake would fail HMAC auth.")
        print("   Dispatch an order in the app, then restart this service:")
        print("     sudo systemctl restart aerix-ble")
        if not args.dev:
            sys.exit(1)
        print("   --dev given: coming up on the bench token instead so the")
        print("   peripheral still advertises. NOT a real delivery.")

    if args.order_file.exists() and stale_age is None:
        order = ActiveOrder.load(args.order_file)
        order_file = args.order_file
    elif args.dev:
        print("  ⚠️  --dev mode: using hardcoded bench-testing token/target. "
              "This is NOT a per-order dynamic token — do not use for a real delivery.")
        order = ActiveOrder.dev_fallback()
        order_file = None
    else:
        print(f"❌ No active order file at {args.order_file}.")
        print("   Run 'python3 firebase_sync.py pull-order' first (needs a DISPATCHED")
        print("   order in Firestore + GOOGLE_APPLICATION_CREDENTIALS set), or pass --dev")
        print("   for bench testing without Firestore.")
        sys.exit(1)

    bus = await get_message_bus()

    service = DroneService(order, order_file)
    await service.register(bus)

    # BLE Advertisement (matches target UUID scanned by app)
    advertisement = Advertisement(
        localName="Target Drone Detected",
        serviceUUIDs=[SERVICE_UUID],
        appearance=0,
        timeout=0,
    )
    await advertisement.register(bus)

    agent = NoIoAgent()
    await agent.register(bus)

    print("\n" + "=" * 60)
    print(" 🛸 RASPBERRY PI 5 DRONE BLE SERVER STARTED")
    print(f" Service UUID    : {SERVICE_UUID}")
    print(f" Order ID        : {order.order_id}")
    # Token provenance, on the banner rather than buried in the pull step. The
    # failure this prevents is silent: a handshake against a token from an
    # earlier flight looks identical to a wrong app, and the only symptom is
    # "AUTHENTICATION FAILED" with two hashes that mean nothing on their own.
    if order_file is not None:
        token_age = (time.time() - order_file.stat().st_mtime) / 60.0
        print(f" Token           : per-order, pulled {token_age:.0f} min ago")
    else:
        print(" Token           : ⚠️  HARDCODED BENCH TOKEN (--dev) - not a real delivery")
    print(f" Target GPS      : ({order.target_lat}, {order.target_lng})")
    print(f" Micro Geofence  : {GEOFENCE_MICRO_RADIUS_M}m")
    print(f" Temporal Check  : {TEMPORAL_READINGS_REQUIRED} consistent readings "
          f"within {TEMPORAL_WINDOW_S:.0f}s (anti-spoofing)")
    print(f" GCS             : {GCS_BASE_URL}")
    print(" Security Layers : HMAC Mutual Auth + Temporal Consistency + Micro Geofence "
          "+ Signed Payloads + RESULT read-back")
    print(" Status          : Advertising... Waiting for Mobile App connection.")
    print("=" * 60 + "\n")

    # Held in a local: asyncio keeps only weak refs to tasks, so an unreferenced
    # watcher could be garbage-collected mid-flight.
    watch = None
    if Path(args.cred).exists():
        watch = asyncio.create_task(_order_watch(service, args.order_file, args.cred))
    else:
        print(f"  ⚠️  [ORDER] no Firebase credentials at {args.cred} - order hot-reload "
              "OFF; a newly dispatched order needs: sudo systemctl restart aerix-ble")

    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
