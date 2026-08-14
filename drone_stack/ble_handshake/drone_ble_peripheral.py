#!/usr/bin/env python3
"""
Raspberry Pi 5 BLE Peripheral Service for Drone Handshake & Parcel Drop.
Uses HMAC-SHA256 Challenge-Response protocol for secure authorization.

## Re-gated (AERIX novelty layer, project plan §9 "BLE release path")

This service carries NO release power of its own. It used to command the
payload servo directly the moment the DROP HMAC verified; now a successful
DROP write only PUBLISHES a signal - over the same local HTTP bridge the
GCS web UI already uses (``POST /api/command``) - that
``drone_stack.novelty.recipient_auth.DualFactorAuthenticator`` (§2.2) reads
as one of its two required, independent channels. The other channel is the
vision person-detector. Only the mission FSM's own ``RELEASING`` state (see
``drone_stack/novelty/delivery_node.py``) ever actually commands the servo.
A spoofed or wrong-person BLE handshake with no vision corroboration at the
same place and time can no longer release anything on its own.

This process is a separate asyncio/D-Bus event loop from the main GCS
process (``drone_stack/gcs/server.py``, run under uvicorn) - they were
never in the same Python process, so "publish onto the bus" here means an
HTTP POST to the GCS's existing local API, exactly like the old
``trigger_parcel_release`` already did for ``set_servo``. See
``drone_stack/gcs/hub.py``'s own ``_on_ble_auth_event`` docstring for the
receiving side.

Dependencies:
    pip install bluez-peripheral

Usage:
    sudo python3 drone_ble_peripheral.py
"""

import os
import hmac
import hashlib
import asyncio
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.agent import NoIoAgent
from bluez_peripheral.util import get_message_bus

# --- CONSTANTS & UUIDs (Must match Flutter App) ---
SERVICE_UUID   = "12345678-1234-5678-1234-567812345678"
NONCE_CHAR_UUID = "12345678-1234-5678-1234-567812345679"  # Read Challenge
AUTH_CHAR_UUID  = "12345678-1234-5678-1234-56781234567a"  # Write Auth Signature
DROP_CHAR_UUID  = "12345678-1234-5678-1234-56781234567b"  # Write Drop Command
GPS_CHAR_UUID   = "12345678-1234-5678-1234-56781234567c"  # Write phone GPS fix (NEW)

SECRET_KEY = b"MY_SUPER_SECRET_KEY"  # Must match Flutter app _secretKey

# The GCS's own local HTTP API (drone_stack/gcs/server.py), already used by
# the web UI - the only channel this process has back into the drone_stack
# process. Same host/port CLAUDE.md documents for the GCS web UI.
GCS_API_URL = "http://localhost:8090/api/command"


class DroneService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        self.last_phone_gps = None  # (lat, lon), set by gps_characteristic writes
        print(f"[Pi 5] Generated initial 16-byte Nonce: {self.current_nonce.hex()}")

    # --- 1. NONCE CHARACTERISTIC (READ ONLY) ---
    @characteristic(NONCE_CHAR_UUID, CharacteristicFlags.READ)
    def nonce_characteristic(self, options):
        # Refresh challenge nonce on every read for anti-replay security.
        # A refresh invalidates any earlier auth, so also tell the FSM the
        # BLE channel is no longer confirmed - otherwise the last-published
        # "authenticated=True" bus event would sit latched (stale) even
        # after the phone disconnects or re-challenges.
        was_authenticated = self.is_authenticated
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        print(f"\n[BLE Read] App requested challenge. New Nonce: {self.current_nonce.hex()}")
        if was_authenticated:
            self.publish_ble_auth_event(authenticated=False)
        return self.current_nonce

    # --- 2. AUTH CHARACTERISTIC (WRITE ONLY) ---
    @characteristic(AUTH_CHAR_UUID, CharacteristicFlags.WRITE)
    def auth_characteristic(self, options):
        # Getter stub required by decorator; not used for WRITE-only
        return bytes()

    @auth_characteristic.setter
    def auth_characteristic(self, value, options):
        print(f"[BLE Write Auth] Received HMAC Signature: {bytes(value).hex()}")

        # Calculate expected HMAC-SHA256(Nonce, SecretKey) and truncate to 20 bytes
        expected_hmac = hmac.new(SECRET_KEY, self.current_nonce, hashlib.sha256).digest()[:20]

        if hmac.compare_digest(bytes(value), expected_hmac):
            self.is_authenticated = True
            print("  ✅ AUTHENTICATION SUCCESSFUL! Drone unlocked.")
        else:
            self.is_authenticated = False
            print("  ❌ AUTHENTICATION FAILED! Invalid HMAC signature.")

    # --- 3. DROP CHARACTERISTIC (WRITE ONLY) ---
    @characteristic(DROP_CHAR_UUID, CharacteristicFlags.WRITE)
    def drop_characteristic(self, options):
        # Getter stub required by decorator; not used for WRITE-only
        return bytes()

    @drop_characteristic.setter
    def drop_characteristic(self, value, options):
        print(f"[BLE Write Drop] Received Drop Command Signature: {bytes(value).hex()}")

        if not self.is_authenticated:
            print("  ⚠️ DROP REJECTED: Connection is not authenticated!")
            return

        # Calculate expected HMAC-SHA256(Nonce + "DROP", SecretKey) and truncate to 20 bytes
        payload = self.current_nonce + b"DROP"
        expected_drop_hmac = hmac.new(SECRET_KEY, payload, hashlib.sha256).digest()[:20]

        if hmac.compare_digest(bytes(value), expected_drop_hmac):
            print("\n" + "=" * 50)
            print("  ✅ DROP AUTHORIZED! Notifying the mission FSM (BLE channel confirmed)...")
            print("     (release itself is the FSM's decision, not this process's - see")
            print("      the re-gating note at the top of this file)")
            print("=" * 50 + "\n")
            self.publish_ble_auth_event(authenticated=True)
        else:
            print("  ❌ DROP REJECTED: Invalid Drop HMAC payload.")

    # --- 4. GPS CHARACTERISTIC (WRITE ONLY, NEW) ---
    @characteristic(GPS_CHAR_UUID, CharacteristicFlags.WRITE)
    def gps_characteristic(self, options):
        # Getter stub required by decorator; not used for WRITE-only
        return bytes()

    @gps_characteristic.setter
    def gps_characteristic(self, value, options):
        """Phone writes its current GPS fix as two little-endian IEEE-754
        doubles (16 bytes total): [lat: f64, lon: f64]. Feeds the fused BLE
        position estimate in recipient_auth.py (phone GPS preferred over the
        coarser RSSI-range fallback - see docs/novelty/recipient_auth.md
        §3c). Independent of the auth/drop handshake above - the app may
        send GPS updates at any time, authenticated or not."""
        import struct

        raw = bytes(value)
        if len(raw) != 16:
            print(f"  ⚠️ GPS WRITE REJECTED: expected 16 bytes (2x float64), got {len(raw)}")
            return
        lat, lon = struct.unpack("<dd", raw)
        self.last_phone_gps = (lat, lon)
        print(f"[BLE Write GPS] Phone position: {lat:.6f}, {lon:.6f}")

    def publish_ble_auth_event(self, authenticated: bool):
        """Bridge to the GCS process over its existing local HTTP API - see
        the module docstring's "Re-gated" section and
        drone_stack/gcs/hub.py's own _on_ble_auth_event. Never controls
        hardware directly; a network failure here just means the FSM never
        sees this event (falls back to its own vision-only/BLE-timeout
        handling), not a release with no oversight."""
        import urllib.request
        import json

        params = {"authenticated": authenticated}
        if self.last_phone_gps is not None:
            params["phone_gps"] = list(self.last_phone_gps)
        data = json.dumps({"cmd": "ble_auth_event", "params": params}).encode("utf-8")
        req = urllib.request.Request(
            GCS_API_URL, data=data, headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(req, timeout=2.0) as response:
                result = json.loads(response.read().decode())
                print(f"[BLE->GCS] published authenticated={authenticated} -> {result}")
        except Exception as e:
            print(f"  ❌ [BLE->GCS ERROR] Failed to publish auth event: {e}")


async def main():
    bus = await get_message_bus()

    service = DroneService()
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

    print("\n==================================================")
    print(" 🛸 RASPBERRY PI 5 DRONE BLE SERVER STARTED")
    print(f" Service UUID : {SERVICE_UUID}")
    print(f" Secret Key   : {SECRET_KEY.decode()}")
    print(f" GCS bridge   : {GCS_API_URL}")
    print(" Release      : RE-GATED - this process no longer drives the servo;")
    print("                a confirmed drop only notifies the mission FSM")
    print("                (drone_stack/novelty/delivery_node.py), which")
    print("                releases only once vision independently agrees.")
    print(" Status       : Advertising... Waiting for Mobile App connection.")
    print("==================================================\n")

    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
