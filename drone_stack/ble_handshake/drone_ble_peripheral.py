#!/usr/bin/env python3
"""
Raspberry Pi 5 BLE Peripheral Service for Drone Handshake & Parcel Drop.
Uses HMAC-SHA256 Challenge-Response protocol for secure authorization.

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

SECRET_KEY = b"MY_SUPER_SECRET_KEY"  # Must match Flutter app _secretKey


class DroneService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        print(f"[Pi 5] Generated initial 16-byte Nonce: {self.current_nonce.hex()}")

    # --- 1. NONCE CHARACTERISTIC (READ ONLY) ---
    @characteristic(NONCE_CHAR_UUID, CharacteristicFlags.READ)
    def nonce_characteristic(self, options):
        # Refresh challenge nonce on every read for anti-replay security
        self.current_nonce = os.urandom(16)
        self.is_authenticated = False
        print(f"\n[BLE Read] App requested challenge. New Nonce: {self.current_nonce.hex()}")
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
            print("  🪂 DROP AUTHORIZED! TRIGGERING SERVO/SOLENOID RELEASE...")
            print("=" * 50 + "\n")
            self.trigger_parcel_release()
        else:
            print("  ❌ DROP REJECTED: Invalid Drop HMAC payload.")

    def trigger_parcel_release(self):
        """
        Hardware action when parcel drop is authorized.
        Sends a command to the GCS server to release the payload via Pixhawk on channel 9 (AUX 1).
        Opens the release mechanism, waits, then resets it back to the closed position.
        """
        import urllib.request
        import json
        import time
        
        url = "http://localhost:8090/api/command"
        
        # 1. OPEN — move servo to release position
        open_pwm = 1410
        data = json.dumps({"cmd": "set_servo", "params": {"channel": 9, "pwm": open_pwm}}).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        
        try:
            with urllib.request.urlopen(req, timeout=2.0) as response:
                result = json.loads(response.read().decode())
                print(f"[HARDWARE] Servo OPENED (PWM {open_pwm}) -> Parcel Released! API Response: {result}")
        except Exception as e:
            print(f"  ❌ [HARDWARE ERROR] Failed to send open command: {e}")
            return
        
        # 2. Wait for the parcel to clear
        print("[HARDWARE] Waiting 2 seconds before resetting servo...")
        time.sleep(2)
        
        # 3. CLOSE — reset servo back to closed position for next drop
        close_pwm = 1100
        data = json.dumps({"cmd": "set_servo", "params": {"channel": 9, "pwm": close_pwm}}).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        
        try:
            with urllib.request.urlopen(req, timeout=2.0) as response:
                result = json.loads(response.read().decode())
                print(f"[HARDWARE] Servo CLOSED (PWM {close_pwm}) -> Ready for next drop. API Response: {result}")
        except Exception as e:
            print(f"  ❌ [HARDWARE ERROR] Failed to send close/reset command: {e}")


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
    print(" Status       : Advertising... Waiting for Mobile App connection.")
    print("==================================================\n")

    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
