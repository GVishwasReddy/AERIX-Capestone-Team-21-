#!/usr/bin/env python3
"""
Test script for detecting, diagnosing, and testing an external TP-Link Bluetooth USB dongle on Raspberry Pi.
Checks:
 1. USB Bus detection (lsusb / TP-Link IDs like UB400 / UB500 / Realtek / CSR)
 2. Linux HCI Adapter status (hci0, hci1, etc.)
 3. Kernel dmesg logs for Bluetooth firmware / driver initialization
 4. BLE / Classic Bluetooth scanning test on detected adapters
"""

import subprocess
import sys
import re
import time

def run_cmd(cmd):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return res.stdout.strip()
    except Exception as e:
        return str(e)

def print_header(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def step1_check_usb():
    print_header("1. CHECKING USB BUS FOR TP-LINK / EXTERNAL BLUETOOTH DONGLE")
    usb_output = run_cmd("lsusb")
    print("Attached USB Devices:")
    print(usb_output if usb_output else "  (No USB devices found)")
    
    tp_link_keywords = ["2357", "tp-link", "tplink", "realtek", "csr", "bluetooth", "0a12"]
    found_dongles = []
    for line in usb_output.splitlines():
        line_lower = line.lower()
        if any(kw in line_lower for kw in tp_link_keywords) and "root hub" not in line_lower:
            found_dongles.append(line)
            
    if found_dongles:
        print("\n✅ External Bluetooth / Target USB Dongle Detected:")
        for d in found_dongles:
            print(f"   -> {d}")
    else:
        print("\n⚠️ No external TP-Link / USB Bluetooth dongle detected in 'lsusb'.")
        print("   -> Please plug your TP-Link Bluetooth dongle into a USB port on the Pi and run this script again.")

def step2_check_hci_adapters():
    print_header("2. CHECKING HSI ADAPTERS (hciconfig / bluetoothctl)")
    hci_out = run_cmd("hciconfig -a")
    if not hci_out:
        print("⚠️ 'hciconfig' returned no adapters or is unavailable.")
        hci_out = run_cmd("bluetoothctl list")
        print("bluetoothctl list:")
        print(hci_out)
        return []

    print("Available HCI Controllers:")
    print(hci_out)
    
    # Parse HCI adapters
    adapters = re.findall(r"(hci\d+):", hci_out)
    return list(set(adapters))

def step3_check_kernel_logs():
    print_header("3. CHECKING KERNEL LOGS FOR BLUETOOTH DRIVERS & FIRMWARE")
    dmesg_bt = run_cmd("dmesg | grep -iE 'bluetooth|rtl|csr|hci' | tail -n 25")
    if dmesg_bt:
        print(dmesg_bt)
    else:
        print("No recent Bluetooth kernel log entries found.")

def step4_test_scan(adapters):
    print_header("4. TESTING BLUETOOTH SCANNING ON HCI ADAPTERS")
    if not adapters:
        print("❌ No HCI adapters available for testing.")
        return

    for hci in adapters:
        print(f"\n--- Testing Adapter: {hci} ---")
        # Ensure adapter is UP
        up_res = run_cmd(f"sudo hciconfig {hci} up")
        print(f"Bringing {hci} UP... {up_res if up_res else 'OK'}")
        
        info = run_cmd(f"hciconfig {hci}")
        print(f"Adapter Info:\n{info}")
        
        print(f"Performing 5-second BLE scan on {hci}...")
        scan_res = run_cmd(f"sudo timeout 5 lescan_test || sudo timeout 5 hcitool -i {hci} lescan 2>&1")
        if "Set scan parameters failed" in scan_res or "Input/output error" in scan_res:
            print(f"⚠️ BLE Scan Notice on {hci}: {scan_res}")
            print("   -> Trying bluetoothctl scan...")
            btctl_res = run_cmd(f"sudo bluetoothctl --timeout 5 scan on")
            print(btctl_res if btctl_res else "Scan complete.")
        else:
            lines = [l for l in scan_res.splitlines() if "LE Scan" not in l][:10]
            if lines:
                print("Found BLE Devices:")
                for l in lines:
                    print(f"  {l}")
            else:
                print("No BLE devices responded during the 5-second scan window (or scan requires bluetoothctl).")

def main():
    print("=" * 60)
    print("      TP-LINK EXTERNAL BLUETOOTH DONGLE TEST & DIAGNOSTIC")
    print("=" * 60)
    
    step1_check_usb()
    adapters = step2_check_hci_adapters()
    step3_check_kernel_logs()
    step4_test_scan(adapters)
    
    print_header("DIAGNOSTIC SUMMARY & INSTRUCTIONS")
    print("• If you just plugged in your TP-Link USB dongle:")
    print("  1. Run 'lsusb' to verify it appears.")
    print("  2. If it is TP-Link UB500 (RTL8761BU chipset), it may require firmware file:")
    print("     '/lib/firmware/rtl_bt/rtl8761b_fw.bin'")
    print("  3. Run 'sudo python3 test_tp_link_bt.py' to run this diagnostic anytime.")
    print("=" * 60)

if __name__ == "__main__":
    main()
