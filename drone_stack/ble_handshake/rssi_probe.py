"""RSSI of a live BLE connection, read straight off the controller.

BlueZ's D-Bus ``Device1.RSSI`` is only refreshed by discovery, so while the
phone is CONNECTED it is stale or absent. The controller still measures every
packet on the link; HCI_Read_RSSI (OGF 0x05, OCF 0x0005) returns it for a
connection handle. The handle comes from HCIGETCONNLIST.

Needs a raw HCI socket, i.e. root / CAP_NET_RAW - aerix-ble runs as root.
Pure stdlib, Linux only; ``parse_*`` helpers are split out so the byte
layouts can be tested anywhere.

    python3 rssi_probe.py            # print every LE link's RSSI on hci0
    python3 rssi_probe.py 1          # ... on hci1
"""

from __future__ import annotations

import fcntl
import select
import socket
import struct
import sys
import time

HCIGETCONNLIST = 0x800448D4          # _IOR('H', 212, int)
SOL_HCI = 0
HCI_FILTER = 2
HCI_COMMAND_PKT = 0x01
HCI_EVENT_PKT = 0x04
EVT_CMD_COMPLETE = 0x0E
EVT_CMD_STATUS = 0x0F
LE_LINK = 0x80
OP_READ_RSSI = 0x1405                # (OGF 0x05 << 10) | OCF 0x0005

_CONN_INFO = struct.Struct("<H6sBBHI")   # handle, bdaddr, type, out, state, link_mode
_REQ_HDR = struct.Struct("<HH")          # dev_id, conn_num
MAX_CONN = 10


def addr_to_bytes(addr: str) -> bytes:
    """"AA:BB:CC:DD:EE:FF" -> bdaddr_t (little-endian, i.e. reversed)."""
    return bytes(int(p, 16) for p in reversed(addr.split(":")))


def bytes_to_addr(b: bytes) -> str:
    return ":".join(f"{x:02X}" for x in reversed(b))


def addr_from_device_path(path: str) -> tuple[int, str] | None:
    """"/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF" -> (0, "AA:BB:CC:DD:EE:FF")."""
    parts = str(path).strip("/").split("/")
    if len(parts) < 4 or not parts[2].startswith("hci") or not parts[3].startswith("dev_"):
        return None
    try:
        dev_id = int(parts[2][3:])
    except ValueError:
        return None
    addr = parts[3][4:].replace("_", ":")
    return (dev_id, addr) if len(addr) == 17 else None


def build_conn_list_req(dev_id: int, n: int = MAX_CONN) -> bytearray:
    return bytearray(_REQ_HDR.pack(dev_id, n) + b"\0" * (_CONN_INFO.size * n))


def parse_conn_list(buf: bytes) -> list[dict]:
    _, num = _REQ_HDR.unpack_from(buf, 0)
    out = []
    for i in range(num):
        handle, bd, typ, outgoing, state, mode = _CONN_INFO.unpack_from(
            buf, _REQ_HDR.size + i * _CONN_INFO.size)
        out.append({"handle": handle, "addr": bytes_to_addr(bd), "type": typ,
                    "out": outgoing, "state": state, "link_mode": mode})
    return out


def build_read_rssi(handle: int) -> bytes:
    return struct.pack("<BHBH", HCI_COMMAND_PKT, OP_READ_RSSI, 2, handle)


def is_reply(pkt: bytes) -> bool:
    """Is this event the controller answering a Read RSSI? Command Complete
    carries the opcode at [4:6], Command Status at [5:7]."""
    if len(pkt) < 7 or pkt[0] != HCI_EVENT_PKT:
        return False
    if pkt[1] == EVT_CMD_COMPLETE:
        return struct.unpack_from("<H", pkt, 4)[0] == OP_READ_RSSI
    if pkt[1] == EVT_CMD_STATUS:
        return struct.unpack_from("<H", pkt, 5)[0] == OP_READ_RSSI
    return False


def parse_read_rssi(pkt: bytes, handle: int) -> int | None:
    """The RSSI (dBm) from a Command Complete for OUR Read RSSI, else None.

    Layout: [0]=0x04 [1]=0x0E [2]=plen [3]=ncmd [4:6]=opcode [6]=status
    [7:9]=handle [9]=int8 rssi. A Command Status for it means it failed."""
    if len(pkt) < 7 or pkt[0] != HCI_EVENT_PKT:
        return None
    if pkt[1] == EVT_CMD_STATUS and len(pkt) >= 7:
        return None
    if pkt[1] != EVT_CMD_COMPLETE or len(pkt) < 10:
        return None
    opcode, status, h = struct.unpack_from("<HBH", pkt, 4)
    if opcode != OP_READ_RSSI or status != 0 or (h & 0x0FFF) != handle:
        return None
    rssi = struct.unpack_from("<b", pkt, 9)[0]
    return rssi if rssi < 0 else None      # 127 = unavailable; >=0 is not a real LE reading


class RssiProbe:
    """Reads one connection's RSSI on demand. Re-resolves the handle when the
    link drops and comes back (a reconnect gets a new handle)."""

    def __init__(self, dev_id: int = 0, addr: str | None = None) -> None:
        self.dev_id = int(dev_id)
        self.addr = addr.upper() if addr else None
        self._sock: socket.socket | None = None
        self._handle: int | None = None

    def _socket(self) -> socket.socket:
        if self._sock is None:
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
            s.bind((self.dev_id,))
            flt = struct.pack("<IIIH2x", 1 << HCI_EVENT_PKT,
                              (1 << EVT_CMD_COMPLETE) | (1 << EVT_CMD_STATUS), 0,
                              OP_READ_RSSI)
            s.setsockopt(SOL_HCI, HCI_FILTER, flt)
            self._sock = s
        return self._sock

    def links(self) -> list[dict]:
        buf = build_conn_list_req(self.dev_id)
        fcntl.ioctl(self._socket().fileno(), HCIGETCONNLIST, buf, True)
        return [c for c in parse_conn_list(bytes(buf)) if c["type"] == LE_LINK]

    def _resolve(self) -> int | None:
        links = self.links()
        if self.addr:
            for c in links:
                if c["addr"] == self.addr:
                    return c["handle"]
        # The phone may sit behind a resolvable private address BlueZ has
        # already mapped to its identity; with exactly one LE link there is
        # no ambiguity about whose it is.
        return links[0]["handle"] if len(links) == 1 else None

    def read(self, timeout_s: float = 0.3) -> int | None:
        """RSSI in dBm, or None (no link, controller refused, timeout)."""
        try:
            if self._handle is None:
                self._handle = self._resolve()
                if self._handle is None:
                    return None
            s = self._socket()
            s.send(build_read_rssi(self._handle))
            end = time.monotonic() + timeout_s
            while True:
                left = end - time.monotonic()
                if left <= 0 or not select.select([s], [], [], left)[0]:
                    self._handle = None
                    return None
                pkt = s.recv(260)
                if not is_reply(pkt):
                    continue                      # someone else's command
                if pkt[1] == EVT_CMD_COMPLETE:
                    v = parse_read_rssi(pkt, self._handle)
                    if v is None:
                        self._handle = None       # stale handle: resolve again next time
                    return v
                self._handle = None               # Command Status = refused
                return None
        except OSError:
            self.close()
            return None

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock, self._handle = None, None


if __name__ == "__main__":
    probe = RssiProbe(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
    try:
        while True:
            links = probe.links()
            vals = []
            for c in links:
                p = RssiProbe(probe.dev_id, c["addr"])
                vals.append(f"{c['addr']} h={c['handle']} rssi={p.read()}")
                p.close()
            print(time.strftime("%H:%M:%S"), " | ".join(vals) or "no LE links")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
