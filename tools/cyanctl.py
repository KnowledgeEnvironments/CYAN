#!/usr/bin/env python3
"""Operate and inspect a CYAN FPGA target.

The FPGA-native management transport uses a reserved raw-Ethernet frame.  The
host protocol and input encoders are useful before the RTL endpoint exists, so
they live here rather than in a Vivado script.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import zlib


ROOT = Path(__file__).resolve().parents[1]
SYS_CLASS_TTY = Path("/sys/class/tty")
SYS_CLASS_NET = Path("/sys/class/net")

MGMT_MAC = bytes.fromhex("024359414e01")
MGMT_ETHERTYPE = 0x88B5
MGMT_MAGIC = b"CYAN"
MGMT_VERSION = 1
MGMT_HEADER = struct.Struct("!4sBBBBIHHI")
MGMT_HEADER_LEN = MGMT_HEADER.size
ETH_HEADER = struct.Struct("!6s6sH")
ETH_MIN_PAYLOAD = 46

# AF_PACKET is Linux-only.  Keep its Linux ABI value available so the
# protocol/unit-test layer remains importable on macOS, where socket.socket is
# mocked.  Live raw-Ethernet board control remains a Watson/Linux operation.
RAW_PACKET_FAMILY = getattr(socket, "AF_PACKET", 17)

FLAG_RESPONSE = 0x01

OP_STATUS = 0x01
OP_BOARD_STATUS = 0x04
OP_HDMI_LINK_STATUS = 0x05
OP_RESET = 0x02
OP_FAN = 0x03
OP_KBD_BYTES = 0x10
OP_MOUSE_BYTES = 0x11
OP_KBD_TX_EVENTS = 0x20
OP_MOUSE_TX_EVENTS = 0x21
OP_KBD_RX_EVENTS = 0x22
OP_MOUSE_RX_EVENTS = 0x23
OP_VIDEO_STATUS = 0x24
OP_SCC_ACCESS_EVENTS = 0x25
OP_SCC_TRACE_CONTROL = 0x26
OP_VIDEO_SIGNATURES = 0x27
OP_VIDEO_STAGE_SIGNATURES = 0x28
OP_RESOLVER_STATUS = 0x29
OP_RESOLVER_DETAIL_STATUS = 0x2A
OP_LG1_TRACE_ARM = 0x2B
OP_LG1_TRACE_MARK = 0x2C
OP_LG1_TRACE_FREEZE = 0x2D
OP_LG1_TRACE_STATUS = 0x2E
OP_LG1_TRACE_READ = 0x2F
OP_CAPTURE_INFO = 0x30
OP_CAPTURE_READ = 0x31
OP_GFX_RETRACE_STATUS = 0x32
OP_VISUAL_STAGE_STATUS = 0x33
OP_SCAN_SERVICE_STATUS = 0x34
OP_SCAN_LATENCY_STATUS = 0x35

OPCODE_NAMES = {
    OP_STATUS: "status",
    OP_BOARD_STATUS: "board-status",
    OP_HDMI_LINK_STATUS: "hdmi-status",
    OP_RESET: "reset",
    OP_FAN: "fan",
    OP_KBD_BYTES: "kbd-bytes",
    OP_MOUSE_BYTES: "mouse-bytes",
    OP_KBD_TX_EVENTS: "kbd-tx-events",
    OP_MOUSE_TX_EVENTS: "mouse-tx-events",
    OP_KBD_RX_EVENTS: "kbd-rx-events",
    OP_MOUSE_RX_EVENTS: "mouse-rx-events",
    OP_VIDEO_STATUS: "video-status",
    OP_SCC_ACCESS_EVENTS: "scc-access-events",
    OP_SCC_TRACE_CONTROL: "scc-trace-control",
    OP_VIDEO_SIGNATURES: "video-signatures",
    OP_VIDEO_STAGE_SIGNATURES: "video-stage-signatures",
    OP_RESOLVER_STATUS: "resolver-status",
    OP_RESOLVER_DETAIL_STATUS: "resolver-detail-status",
    OP_LG1_TRACE_ARM: "lg1-trace-arm",
    OP_LG1_TRACE_MARK: "lg1-trace-mark",
    OP_LG1_TRACE_FREEZE: "lg1-trace-freeze",
    OP_LG1_TRACE_STATUS: "lg1-trace-status",
    OP_LG1_TRACE_READ: "lg1-trace-read",
    OP_CAPTURE_INFO: "capture-info",
    OP_CAPTURE_READ: "capture-read",
    OP_GFX_RETRACE_STATUS: "gfx-retrace-status",
    OP_VISUAL_STAGE_STATUS: "visual-stage-status",
    OP_SCAN_SERVICE_STATUS: "scan-service-status",
    OP_SCAN_LATENCY_STATUS: "scan-latency-status",
}

# presentation-watch is deliberately observation-only.  Keep the complete
# request (opcode plus payload) allowlist here so a future refactor cannot
# accidentally admit reset, input, fan, SCC, trace-control, or RGB-read
# operations through its persistent management socket.
PRESENTATION_WATCH_ALLOWED_REQUESTS = frozenset({
    (OP_RESOLVER_STATUS, b""),
    (OP_CAPTURE_INFO, b"\x01"),
    (OP_CAPTURE_INFO, b"\x02"),
    (OP_VIDEO_SIGNATURES, b""),
    (OP_VIDEO_STAGE_SIGNATURES, b""),
    (OP_RESOLVER_DETAIL_STATUS, b""),
    (OP_VISUAL_STAGE_STATUS, b""),
})


def presentation_watch_request_allowed(
    opcode: int,
    payload: bytes,
    *,
    allow_rgb_word_reads: bool = False,
) -> bool:
    """Keep passive presentation observers on an explicit request boundary."""
    if (opcode, payload) in PRESENTATION_WATCH_ALLOWED_REQUESTS:
        return True
    return (
        allow_rgb_word_reads
        and opcode == OP_CAPTURE_READ
        and len(payload) == 3
        and payload[2] < 4
    )


def presentation_watch_request_retryable(opcode: int, payload: bytes) -> bool:
    """Retry idempotent observations, but never replay the capture pin."""
    if opcode == OP_CAPTURE_INFO and payload == b"\x01":
        return False
    return presentation_watch_request_allowed(
        opcode, payload, allow_rgb_word_reads=True
    )

ASCII_TO_SGI_SCANCODE = {
    "a": 10, "b": 35, "c": 27, "d": 17, "e": 16, "f": 18,
    "g": 25, "h": 26, "i": 39, "j": 33, "k": 34, "l": 41,
    "m": 43, "n": 36, "o": 40, "p": 47, "q": 9, "r": 23,
    "s": 11, "t": 24, "u": 32, "v": 28, "w": 15, "x": 20,
    "y": 31, "z": 19,
    "1": 7, "2": 13, "3": 14, "4": 21, "5": 22,
    "6": 29, "7": 30, "8": 37, "9": 38, "0": 45,
    "-": 46, "=": 53, "[": 48, "]": 55, "\\": 56,
    ";": 42, "'": 49, ",": 44, ".": 51, "/": 52, "`": 54,
    "\r": 50, "\n": 50, "\t": 8, " ": 82, "\b": 60,
}

SHIFTED_ASCII_TO_BASE = {
    "~": "`", "!": "1", "@": "2", "#": "3", "$": "4",
    "%": "5", "^": "6", "&": "7", "*": "8", "(": "9",
    ")": "0", "_": "-", "+": "=", "{": "[", "}": "]",
    "|": "\\", ":": ";", '"': "'", "<": ",", ">": ".",
    "?": "/",
}

NAMED_SGI_SCANCODE = {
    "escape": 6,
    "tab": 8,
    "enter": 50,
    "return": 50,
    "space": 82,
    "backspace": 60,
    "delete": 61,
    "left": 72,
    "down": 73,
    "up": 80,
    "right": 79,
    "shift": 5,
    "leftshift": 5,
    "rightshift": 4,
    "control": 2,
    "ctrl": 2,
    "leftcontrol": 2,
    "leftctrl": 2,
    "rightcontrol": 85,
    "rightctrl": 85,
    "alt": 83,
    "leftalt": 83,
    "rightalt": 84,
    "capslock": 3,
}

LG1_TRACE_MARKERS = {
    "focus": 0x01,
    "twilight-pre-xwd": 0x02,
    "twilight-active": 0x03,
    "teardown": 0x04,
}
LG1_TRACE_MARKER_NAMES = {
    0x00: "arm",
    **{value: name for name, value in LG1_TRACE_MARKERS.items()},
    0xFF: "freeze",
}


@dataclass(frozen=True)
class MgmtMessage:
    opcode: int
    sequence: int
    payload: bytes = b""
    flags: int = 0
    status: int = 0

    def encode(self) -> bytes:
        if not 0 <= self.opcode <= 0xFF:
            raise ValueError("opcode must fit in one byte")
        if not 0 <= self.sequence <= 0xFFFFFFFF:
            raise ValueError("sequence must fit in four bytes")
        if len(self.payload) > 0xFFFF:
            raise ValueError("payload is too large")
        checksum = zlib.crc32(self.payload) & 0xFFFFFFFF
        return MGMT_HEADER.pack(
            MGMT_MAGIC,
            MGMT_VERSION,
            self.opcode,
            self.flags,
            self.status,
            self.sequence,
            len(self.payload),
            0,
            checksum,
        ) + self.payload

    @classmethod
    def decode(cls, data: bytes) -> "MgmtMessage":
        if len(data) < MGMT_HEADER_LEN:
            raise ValueError("management message is shorter than its header")
        (
            magic,
            version,
            opcode,
            flags,
            status,
            sequence,
            payload_len,
            reserved,
            expected_checksum,
        ) = MGMT_HEADER.unpack_from(data)
        if magic != MGMT_MAGIC:
            raise ValueError("management magic mismatch")
        if version != MGMT_VERSION:
            raise ValueError(f"unsupported management version {version}")
        if reserved != 0:
            raise ValueError("management reserved field is nonzero")
        end = MGMT_HEADER_LEN + payload_len
        if len(data) < end:
            raise ValueError("management payload is truncated")
        payload = data[MGMT_HEADER_LEN:end]
        checksum = zlib.crc32(payload) & 0xFFFFFFFF
        if checksum != expected_checksum:
            raise ValueError("management payload checksum mismatch")
        return cls(opcode, sequence, payload, flags, status)


def parse_mac(text: str) -> bytes:
    pieces = text.strip().split(":")
    if len(pieces) != 6:
        raise ValueError(f"invalid MAC address: {text!r}")
    try:
        value = bytes(int(piece, 16) for piece in pieces)
    except ValueError as exc:
        raise ValueError(f"invalid MAC address: {text!r}") from exc
    if any(len(piece) != 2 for piece in pieces):
        raise ValueError(f"invalid MAC address: {text!r}")
    return value


def format_mac(value: bytes) -> str:
    if len(value) != 6:
        raise ValueError("MAC address must be six bytes")
    return value.hex(":")


def encode_ethernet_frame(source_mac: bytes, message: MgmtMessage) -> bytes:
    payload = message.encode()
    if len(payload) < ETH_MIN_PAYLOAD:
        payload += bytes(ETH_MIN_PAYLOAD - len(payload))
    return ETH_HEADER.pack(MGMT_MAC, source_mac, MGMT_ETHERTYPE) + payload


def decode_ethernet_frame(
    frame: bytes, expected_destination: bytes | None = None
) -> tuple[bytes, bytes, MgmtMessage]:
    if len(frame) < ETH_HEADER.size + MGMT_HEADER_LEN:
        raise ValueError("Ethernet frame is too short")
    destination, source, ethertype = ETH_HEADER.unpack_from(frame)
    if ethertype != MGMT_ETHERTYPE:
        raise ValueError(f"unexpected EtherType 0x{ethertype:04x}")
    if expected_destination is not None and destination != expected_destination:
        raise ValueError(f"unexpected destination MAC {format_mac(destination)}")
    return destination, source, MgmtMessage.decode(frame[ETH_HEADER.size:])


def scancode_for_key(key: str) -> int:
    lowered = key.lower()
    if lowered in NAMED_SGI_SCANCODE:
        return NAMED_SGI_SCANCODE[lowered]
    if len(key) == 1:
        lookup = key.lower() if "A" <= key <= "Z" else key
        if lookup in ASCII_TO_SGI_SCANCODE:
            return ASCII_TO_SGI_SCANCODE[lookup]
    raise ValueError(f"unsupported SGI key {key!r}")


def encode_key(key: str, release: bool = False) -> bytes:
    code = scancode_for_key(key)
    return bytes([code | (0x80 if release else 0)])


def encode_text(text: str) -> bytes:
    encoded = bytearray()
    for character in text:
        shifted = "A" <= character <= "Z" or \
            character in SHIFTED_ASCII_TO_BASE
        physical_key = SHIFTED_ASCII_TO_BASE.get(character, character)
        code = scancode_for_key(physical_key)
        if shifted:
            encoded.append(NAMED_SGI_SCANCODE["leftshift"])
        encoded.extend((code, code | 0x80))
        if shifted:
            encoded.append(NAMED_SGI_SCANCODE["leftshift"] | 0x80)
    return bytes(encoded)


def mouse_sync_byte(buttons: int) -> int:
    if not 0 <= buttons <= 7:
        raise ValueError("mouse button mask must be in the range 0..7")
    lmr = 0x07
    if buttons & 0x1:
        lmr &= ~0x04
    if buttons & 0x2:
        lmr &= ~0x02
    if buttons & 0x4:
        lmr &= ~0x01
    return 0x80 | lmr


def encode_mouse(buttons: int, dx: int, dy: int) -> bytes:
    encoded = bytearray()
    remaining_x = dx
    remaining_y = dy
    while True:
        step_x = min(127, max(-128, remaining_x))
        step_y = min(127, max(-128, remaining_y))
        remaining_x -= step_x
        remaining_y -= step_y
        encoded.extend(
            (
                mouse_sync_byte(buttons),
                step_x & 0xFF,
                step_y & 0xFF,
                0,
                0,
            )
        )
        if remaining_x == 0 and remaining_y == 0:
            return bytes(encoded)


def discover_cp2105_ports(sys_class_tty: Path = SYS_CLASS_TTY) -> list[dict[str, str]]:
    ports: list[dict[str, str]] = []
    for tty_path in sorted(sys_class_tty.glob("ttyUSB*")):
        device_link = tty_path / "device"
        if not device_link.exists():
            continue
        device = device_link.resolve()
        interface_file = device.parent / "bInterfaceNumber"
        driver_link = device / "driver"
        if not interface_file.is_file() or not driver_link.exists():
            continue
        interface = interface_file.read_text().strip().lower()
        driver = driver_link.resolve().name
        if driver != "cp210x" or interface not in {"00", "01"}:
            continue
        role = "system-controller" if interface == "00" else "sgi-scc-b"
        ports.append(
            {
                "device": f"/dev/{tty_path.name}",
                "interface": interface,
                "driver": driver,
                "role": role,
            }
        )
    return ports


def console_owners(device: str) -> list[str]:
    result = subprocess.run(
        ["fuser", device],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return result.stdout.split()


def git_identity() -> dict[str, object]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=ROOT,
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        ).stdout
    )
    return {"commit": commit, "tracked_dirty": dirty}


def status_document() -> dict[str, object]:
    ports = discover_cp2105_ports()
    for port in ports:
        port["owners"] = console_owners(port["device"])
    return {
        "source": git_identity(),
        "serial": ports,
        "management": {
            "destination_mac": format_mac(MGMT_MAC),
            "ethertype": f"0x{MGMT_ETHERTYPE:04x}",
            "rtl_endpoint": "source-implemented-hardware-unproven",
        },
    }


def read_interface_mac(interface: str) -> bytes:
    path = SYS_CLASS_NET / interface / "address"
    if not path.is_file():
        raise ValueError(f"network interface does not exist: {interface}")
    return parse_mac(path.read_text().strip())


def transact_on_socket(
    sock: socket.socket,
    source_mac: bytes,
    request: MgmtMessage,
    timeout: float,
) -> MgmtMessage:
    frame = encode_ethernet_frame(source_mac, request)
    sock.settimeout(timeout)
    sock.send(frame)
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("CYAN management request timed out")
        sock.settimeout(remaining)
        response_frame = sock.recv(65535)
        try:
            destination, source, response = decode_ethernet_frame(
                response_frame, source_mac
            )
        except ValueError:
            continue
        if destination != source_mac or source != MGMT_MAC:
            continue
        if not response.flags & FLAG_RESPONSE:
            continue
        if response.sequence != request.sequence or response.opcode != request.opcode:
            continue
        return response


def transact(interface: str, request: MgmtMessage, timeout: float) -> MgmtMessage:
    source_mac = read_interface_mac(interface)
    with socket.socket(RAW_PACKET_FAMILY, socket.SOCK_RAW, socket.htons(MGMT_ETHERTYPE)) as sock:
        sock.bind((interface, 0))
        return transact_on_socket(sock, source_mac, request, timeout)


def snoop_management_frames(args: argparse.Namespace) -> int:
    """Passively decode CYAN management frames without transmitting."""
    observed = 0
    with socket.socket(
        RAW_PACKET_FAMILY,
        socket.SOCK_RAW,
        socket.htons(MGMT_ETHERTYPE),
    ) as sock:
        sock.bind((args.interface, 0))
        sock.settimeout(args.timeout)
        while observed < args.count:
            try:
                frame = sock.recv(65535)
            except socket.timeout:
                break
            try:
                destination, source, message = decode_ethernet_frame(frame)
            except ValueError:
                continue
            print(json.dumps({
                "destination": format_mac(destination),
                "source": format_mac(source),
                "opcode": message.opcode,
                "opcode_name": OPCODE_NAMES.get(
                    message.opcode, f"unknown-0x{message.opcode:02x}"
                ),
                "response": bool(message.flags & FLAG_RESPONSE),
                "flags": message.flags,
                "status": message.status,
                "sequence": message.sequence,
                "payload_hex": message.payload.hex(),
            }, sort_keys=True))
            observed += 1
    return 0


def next_sequence() -> int:
    return time.monotonic_ns() & 0xFFFFFFFF


def decode_video_status(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(f"video status must be 16 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    source_bank = (value >> 113) & 3
    source_capture_enabled = bool((value >> 112) & 1)
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "completed_toggle": bool((value >> 119) & 1),
        "completed_bank": (value >> 117) & 3,
        "display_bank": (value >> 115) & 3,
        "source_bank": source_bank,
        "source_capture_enabled": source_capture_enabled,
        # Deprecated compatibility aliases.  These bits describe the
        # source-side RGB packer, not the independently pinned management
        # capture bank/state exposed by CAPTURE_INFO.
        "capture_bank": source_bank,
        "capture_enabled": source_capture_enabled,
        "write_fifo_full": bool((value >> 111) & 1),
        "read_fifo_full": bool((value >> 110) & 1),
        "read_fifo_valid": bool((value >> 109) & 1),
        "scan_active": bool((value >> 108) & 1),
        "mig_state": (value >> 105) & 7,
        "underruns": (value >> 89) & 0xFFFF,
        "dropped_frames": (value >> 73) & 0xFFFF,
        "mig_reads": (value >> 57) & 0xFFFF,
        "mig_writes": (value >> 41) & 0xFFFF,
        "source_words": (value >> 25) & 0xFFFF,
        "inflight_banks": (value >> 22) & 7,
        "payload_signature_valid": bool((value >> 18) & 1),
        "payload_low_half_match": bool((value >> 17) & 1),
        "payload_high_half_match": bool((value >> 16) & 1),
        "source_low_half_nonzero": bool((value >> 15) & 1),
        "source_high_half_nonzero": bool((value >> 14) & 1),
        "mig_low_half_nonzero": bool((value >> 13) & 1),
        "mig_high_half_nonzero": bool((value >> 12) & 1),
        "lg1_underrun_seen": bool((value >> 11) & 1),
        "lg1_protocol_error_seen": bool((value >> 10) & 1),
        "payload_signature_generation": (value >> 2) & 0xFF,
    }


def decode_scan_service_status(payload: bytes) -> dict[str, int | bool | str]:
    """Decode one completed active frame's first scan-miss boundary."""
    if len(payload) != 16:
        raise ValueError(
            f"scan service status must be 16 bytes, got {len(payload)}"
        )
    value = int.from_bytes(payload, "big")
    flags = (value >> 40) & 0x7F
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "valid": bool((value >> 111) & 1),
        "epoch": (value >> 103) & 0xFF,
        "misses": (value >> 87) & 0xFFFF,
        "first_miss_index": (value >> 71) & 0xFFFF,
        "first_head_epoch": (value >> 63) & 0xFF,
        "first_head_index": (value >> 47) & 0xFFFF,
        "first_miss_seen": bool(flags & 0x40),
        "head_valid": bool(flags & 0x20),
        "head_exact": bool(flags & 0x10),
        "head_obsolete": bool(flags & 0x08),
        "head_future": bool(flags & 0x04),
        "fifo_empty": bool(flags & 0x02),
        "scan_valid": bool(flags & 0x01),
    }


def decode_scan_latency_status(payload: bytes) -> dict[str, int | bool | str]:
    """Decode one completed MIG scan epoch's service ledger."""
    if len(payload) != 16:
        raise ValueError(
            f"scan latency status must be 16 bytes, got {len(payload)}"
        )
    value = int.from_bytes(payload, "big")
    flags = value & 0x3F
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "epoch": (value >> 104) & 0xFF,
        "commands": (value >> 88) & 0xFFFF,
        "responses": (value >> 72) & 0xFFFF,
        "max_command_wait_cycles": (value >> 56) & 0xFFFF,
        "max_response_latency_cycles": (value >> 40) & 0xFFFF,
        "minimum_resident_words": (value >> 33) & 0x7F,
        "maximum_outstanding_reads": (value >> 30) & 0x07,
        "responses_le_32_cycles": (value >> 22) & 0xFF,
        "responses_33_to_128_cycles": (value >> 14) & 0xFF,
        "responses_gt_128_cycles": (value >> 6) & 0xFF,
        "valid": bool(flags & 0x20),
        "video_record_valid": bool(flags & 0x10),
        "epoch_matches_video": bool(flags & 0x08),
        "commands_match_responses": bool(flags & 0x04),
        "resident_reached_zero": bool(flags & 0x02),
        "slow_response_seen": bool(flags & 0x01),
    }


def decode_gfx_retrace_status(payload: bytes) -> dict[str, int | bool | str]:
    """Decode the retained LG1 raw-retrace through CPU-IRQ boundary."""
    if len(payload) != 16:
        raise ValueError(
            f"graphics retrace status must be 16 bytes, got {len(payload)}"
        )
    flags = payload[3]
    return {
        "signature": f"0x{payload[0]:02x}",
        "port_config": f"0x{payload[1]:02x}",
        "lio1_mask": f"0x{payload[2]:02x}",
        "raw_now": bool(flags & 0x01),
        "pending": bool(flags & 0x02),
        "cpu_irq1": bool(flags & 0x04),
        "armed": bool(flags & 0x08),
        "mask7": bool(flags & 0x10),
        "raw_count": int.from_bytes(payload[4:7], "big"),
        "armed_raw_count": int.from_bytes(payload[7:10], "big"),
        "clear_write_count": int.from_bytes(payload[10:13], "big"),
        "arm_write_count": int.from_bytes(payload[13:16], "big"),
    }


def decode_video_signatures(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(f"video signatures must be 16 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    source_low = (value >> 77) & 0xFFFF
    source_high = (value >> 61) & 0xFFFF
    mig_low = (value >> 45) & 0xFFFF
    mig_high = (value >> 29) & 0xFFFF
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "completed_valid": bool((value >> 111) & 1),
        "completed_bank": (value >> 109) & 3,
        "frame_generation": (value >> 93) & 0xFFFF,
        "source_low": f"0x{source_low:04x}",
        "source_high": f"0x{source_high:04x}",
        "mig_low": f"0x{mig_low:04x}",
        "mig_high": f"0x{mig_high:04x}",
        "low_half_match": source_low == mig_low,
        "high_half_match": source_high == mig_high,
        "signature_generation": (value >> 21) & 0xFF,
    }


def decode_video_stage_signatures(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(
            f"video stage signatures must be 16 bytes, got {len(payload)}"
        )
    value = int.from_bytes(payload, "big")
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "completed_valid": bool((value >> 111) & 1),
        "completed_bank": (value >> 109) & 3,
        "frame_generation": (value >> 93) & 0xFFFF,
        "raw_low": f"0x{(value >> 77) & 0xFFFF:04x}",
        "raw_high": f"0x{(value >> 61) & 0xFFFF:04x}",
        "fetch_low": f"0x{(value >> 45) & 0xFFFF:04x}",
        "fetch_high": f"0x{(value >> 29) & 0xFFFF:04x}",
        "signature_generation": (value >> 21) & 0xFF,
        "fetch_valid": bool((value >> 20) & 1),
        "fetch_generation": (value >> 12) & 0xFF,
    }


def decode_visual_stage_status(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(
            f"visual stage status must be 16 bytes, got {len(payload)}"
        )
    value = int.from_bytes(payload, "big")
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "completed_valid": bool((value >> 111) & 1),
        "completed_bank": (value >> 109) & 3,
        "frame_generation": (value >> 93) & 0xFFFF,
        "did_signature": f"0x{(value >> 77) & 0xFFFF:04x}",
        "xmap_mode_signature": f"0x{(value >> 61) & 0xFFFF:04x}",
        "route_signature": f"0x{(value >> 45) & 0xFFFF:04x}",
        "display_index_signature": f"0x{(value >> 29) & 0xFFFF:04x}",
    }


def decode_resolver_status(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(f"resolver status must be 16 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    completed_valid = bool((value >> 111) & 1)
    resolver_valid = bool((value >> 92) & 1)
    cfg_stable = bool((value >> 91) & 1)
    display_enable = bool((value >> 90) & 1)
    window_seen = bool((value >> 88) & 1)
    rdmask = (value >> 80) & 0xFF
    overlay = (value >> 60) & 0xFFFFF
    flood = (value >> 40) & 0xFFFFF
    cursor = (value >> 20) & 0xFFFFF
    front_nonzero = value & 0xFFFFF
    noncursor_override = overlay != 0 or flood != 0 or window_seen
    if not completed_valid:
        classification = "no-completed-frame"
    elif not resolver_valid:
        classification = "resolver-record-unavailable"
    elif not cfg_stable:
        classification = "configuration-changed-during-frame"
    elif not display_enable:
        classification = "vc1-global-blank"
    elif overlay + cursor == 1024 * 768:
        classification = "full-screen-overlay"
    elif flood + cursor == 1024 * 768:
        classification = "full-screen-window-flood"
    elif not noncursor_override and cursor < 1024 * 768 and rdmask == 0:
        classification = "zero-readmask"
    elif (not noncursor_override and cursor < 1024 * 768 and
          rdmask != 0 and front_nonzero == 0):
        classification = "did-xmap-palette-black"
    elif front_nonzero != 0:
        classification = "front-palette-active"
    else:
        classification = "mixed-or-unclassified"
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "completed_valid": completed_valid,
        "completed_bank": (value >> 109) & 3,
        "frame_generation": (value >> 93) & 0xFFFF,
        "resolver_valid": resolver_valid,
        "cfg_stable": cfg_stable,
        "display_enable": display_enable,
        "window_mode": bool((value >> 89) & 1),
        "window_seen": window_seen,
        "rdmask": f"0x{rdmask:02x}",
        "overlay_winners": overlay,
        "flood_winners": flood,
        "cursor_winners": cursor,
        "front_nonzero_winners": front_nonzero,
        "classification": classification,
    }


def validate_resolver_watch_status(resolver: dict[str, object]) -> None:
    """Reject a descriptor that cannot support frame-continuity claims."""
    if resolver.get("signature") != "0x52" or resolver.get("version") != 1:
        raise ValueError(f"unsupported resolver descriptor {resolver}")
    if not resolver.get("completed_valid"):
        raise ValueError("resolver descriptor has no completed frame")
    if not resolver.get("resolver_valid"):
        raise ValueError("resolver descriptor has no valid resolver record")


def classify_resolver_generation_transition(
    previous: int, current: int
) -> dict[str, int | str]:
    """Classify a bounded 16-bit completed-frame generation transition."""
    step = (current - previous) & 0xFFFF
    if step == 0:
        classification = "duplicate"
        missed = 0
    elif step == 1:
        classification = "contiguous"
        missed = 0
    elif step <= 0x7FFF:
        classification = "missed"
        missed = step - 1
    else:
        # A 16-bit generation alone cannot distinguish a counter reset from a
        # stale/backward sample.  In a bounded watch either invalidates the
        # continuity claim, while 0xffff -> 0 remains the step-one case above.
        classification = "backward-or-reset"
        missed = 0
    return {
        "classification": classification,
        "step": step,
        "missed_completed_frames": missed,
    }


def decode_resolver_detail_status(payload: bytes) -> dict[str, int | bool | str]:
    if len(payload) != 16:
        raise ValueError(
            f"resolver detail status must be 16 bytes, got {len(payload)}"
        )
    value = int.from_bytes(payload, "big")
    completed_valid = bool((value >> 111) & 1)
    resolver_valid = bool((value >> 92) & 1)
    index_nonzero = (value >> 80) & 0xFFF
    if not completed_valid:
        classification = "no-completed-frame"
    elif not resolver_valid:
        classification = "resolver-record-unavailable"
    elif index_nonzero == 0:
        classification = "front-index-zero-or-no-front"
    else:
        classification = "front-index-nonzero-observed"
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "completed_valid": completed_valid,
        "completed_bank": (value >> 109) & 3,
        "frame_generation": (value >> 93) & 0xFFFF,
        "resolver_valid": resolver_valid,
        "front_index_nonzero_count": index_nonzero,
        "front_index_nonzero_count_saturated": index_nonzero == 0xFFF,
        "front_did0_winners": (value >> 60) & 0xFFFFF,
        "front_bank0_winners": (value >> 40) & 0xFFFFF,
        "front_bank1_winners": (value >> 20) & 0xFFFFF,
        "front_bank2_winners": value & 0xFFFFF,
        "classification": classification,
    }


def correlate_resolver_detail(
    resolver: dict[str, object], detail: dict[str, object]
) -> dict[str, object]:
    if resolver.get("signature") != "0x52" or resolver.get("version") != 1:
        raise ValueError(f"unsupported resolver descriptor {resolver}")
    if detail.get("signature") != "0x44" or detail.get("version") != 1:
        raise ValueError(f"unsupported resolver detail descriptor {detail}")
    if (
        resolver.get("completed_bank") != detail.get("completed_bank")
        or resolver.get("frame_generation") != detail.get("frame_generation")
    ):
        raise ValueError(
            "resolver detail descriptor does not match resolver frame: "
            f"resolver={resolver} detail={detail}"
        )
    if resolver.get("completed_valid") != detail.get("completed_valid"):
        raise ValueError("resolver pages disagree about completed-frame validity")
    if resolver.get("resolver_valid") != detail.get("resolver_valid"):
        raise ValueError("resolver pages disagree about resolver-record validity")

    result: dict[str, object] = {
        "classification": resolver.get("classification"),
        "front_winners": None,
        "front_bank3_winners": None,
        "front_all_did0": None,
        "front_single_bank": None,
    }
    if not resolver.get("completed_valid") or not resolver.get("resolver_valid"):
        return result
    if (
        not resolver.get("cfg_stable")
        or not resolver.get("display_enable")
        or resolver.get("window_seen")
    ):
        # Window-palette winners are not included in either the FRONT or flood
        # counts, so the FRONT total and bank 3 cannot be derived in this mode.
        return result

    frame_pixels = 1024 * 768
    front_winners = frame_pixels - sum(
        int(resolver[name])
        for name in ("overlay_winners", "flood_winners", "cursor_winners")
    )
    bank012 = sum(
        int(detail[name])
        for name in (
            "front_bank0_winners",
            "front_bank1_winners",
            "front_bank2_winners",
        )
    )
    did0 = int(detail["front_did0_winners"])
    if front_winners < 0 or bank012 > front_winners or did0 > front_winners:
        raise ValueError(
            "resolver detail counters are inconsistent with resolver winners"
        )
    bank3 = front_winners - bank012
    banks = [
        int(detail["front_bank0_winners"]),
        int(detail["front_bank1_winners"]),
        int(detail["front_bank2_winners"]),
        bank3,
    ]
    single_banks = [
        bank for bank, count in enumerate(banks)
        if front_winners != 0 and count == front_winners
    ]
    result.update({
        "front_winners": front_winners,
        "front_bank3_winners": bank3,
        "front_all_did0": front_winners != 0 and did0 == front_winners,
        "front_single_bank": single_banks[0] if len(single_banks) == 1 else None,
    })
    if resolver.get("classification") == "did-xmap-palette-black":
        if int(detail["front_index_nonzero_count"]) == 0:
            result["classification"] = "front-index-zero"
        else:
            result["classification"] = "front-palette-black"
    return result


def decode_board_status(payload: bytes) -> dict[str, object]:
    if len(payload) != 16:
        raise ValueError(f"board status must be 16 bytes, got {len(payload)}")
    hdmi = int.from_bytes(payload[0:4], "big")
    return {
        "adv7511": {
            "signature": f"0x{(hdmi >> 24) & 0xFF:02x}",
            "i2c_state": (hdmi >> 20) & 0xF,
            "config_index": (hdmi >> 14) & 0x3F,
            "done": bool((hdmi >> 11) & 1),
            "nack_seen": bool((hdmi >> 10) & 1),
            "scl": bool((hdmi >> 9) & 1),
            "sda": bool((hdmi >> 8) & 1),
            "nack_count": hdmi & 0xFF,
        },
        "soc_status": f"0x{int.from_bytes(payload[4:8], 'big'):08x}",
        "video_head": f"0x{int.from_bytes(payload[8:12], 'big'):08x}",
        "scc_access_queued": payload[12],
        "scc_access_overflow_low": payload[13],
        "fan_duty": {0: 50, 1: 25, 2: 75, 3: 100}[payload[14] & 3],
        "leds": f"0x{payload[15]:02x}",
    }


def decode_hdmi_link_status(payload: bytes) -> dict[str, object]:
    if len(payload) != 16:
        raise ValueError(f"HDMI link status must be 16 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "i2c_state": (value >> 116) & 0xF,
        "config_index": (value >> 110) & 0x3F,
        "config_done": bool((value >> 107) & 1),
        "nack_seen": bool((value >> 106) & 1),
        "scl": bool((value >> 105) & 1),
        "sda": bool((value >> 104) & 1),
        "nack_count": (value >> 96) & 0xFF,
        "readback_valid": bool((value >> 95) & 1),
        "hpd": bool((value >> 94) & 1),
        "monitor_sense": bool((value >> 93) & 1),
        "pll_locked": bool((value >> 92) & 1),
        "power_down": bool((value >> 91) & 1),
        "registers": {
            "0x42": f"0x{(value >> 83) & 0xFF:02x}",
            "0x9e": f"0x{(value >> 75) & 0xFF:02x}",
            "0x41": f"0x{(value >> 67) & 0xFF:02x}",
            "0x15": f"0x{(value >> 59) & 0xFF:02x}",
            "0x16": f"0x{(value >> 51) & 0xFF:02x}",
            "0x48": f"0x{(value >> 43) & 0xFF:02x}",
        },
        "readback_generation": (value >> 27) & 0xFFFF,
    }


def decode_capture_info(payload: bytes) -> dict[str, object]:
    if len(payload) != 16:
        raise ValueError(f"capture info must be 16 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    return {
        "signature": f"0x{(value >> 120) & 0xFF:02x}",
        "version": (value >> 112) & 0xFF,
        "width": (value >> 96) & 0xFFFF,
        "height": (value >> 80) & 0xFFFF,
        "words_per_frame": (value >> 64) & 0xFFFF,
        "bank": (value >> 62) & 3,
        "pinned": bool((value >> 61) & 1),
        "completed_valid": bool((value >> 60) & 1),
        "frame_generation": (value >> 44) & 0xFFFF,
        "cached_word": (value >> 28) & 0xFFFF,
        "cache_valid": bool((value >> 27) & 1),
        "error": bool((value >> 26) & 1),
    }


def validate_pinned_video_signatures(
    capture_info: dict[str, object], signatures: dict[str, object]
) -> None:
    if signatures.get("signature") != "0x53" or signatures.get("version") != 1:
        raise ValueError(f"unsupported video signature descriptor {signatures}")
    if not signatures.get("completed_valid"):
        raise ValueError("video signature descriptor has no completed frame")
    if (
        signatures.get("completed_bank") != capture_info.get("bank")
        or signatures.get("frame_generation") != capture_info.get("frame_generation")
    ):
        raise ValueError(
            "video signature descriptor does not match pinned capture: "
            f"capture={capture_info} signatures={signatures}"
        )


def validate_pinned_video_stage_signatures(
    capture_info: dict[str, object], signatures: dict[str, object]
) -> None:
    if signatures.get("signature") != "0x46" or signatures.get("version") != 1:
        raise ValueError(f"unsupported video stage descriptor {signatures}")
    if not signatures.get("completed_valid"):
        raise ValueError("video stage descriptor has no completed frame")
    if (
        signatures.get("completed_bank") != capture_info.get("bank")
        or signatures.get("frame_generation") != capture_info.get("frame_generation")
    ):
        raise ValueError(
            "video stage descriptor does not match pinned capture: "
            f"capture={capture_info} signatures={signatures}"
        )


def validate_pinned_visual_stage_status(
    capture_info: dict[str, object], stages: dict[str, object]
) -> None:
    if stages.get("signature") != "0x50" or stages.get("version") != 1:
        raise ValueError(f"unsupported visual stage descriptor {stages}")
    if not stages.get("completed_valid"):
        raise ValueError("visual stage descriptor has no completed frame")
    if (
        stages.get("completed_bank") != capture_info.get("bank")
        or stages.get("frame_generation") != capture_info.get("frame_generation")
    ):
        raise ValueError(
            "visual stage descriptor does not match pinned capture: "
            f"capture={capture_info} stages={stages}"
        )


def validate_pinned_resolver_status(
    capture_info: dict[str, object], resolver: dict[str, object]
) -> None:
    if resolver.get("signature") != "0x52" or resolver.get("version") != 1:
        raise ValueError(f"unsupported resolver descriptor {resolver}")
    if not resolver.get("completed_valid"):
        raise ValueError("resolver descriptor has no completed frame")
    if (
        resolver.get("completed_bank") != capture_info.get("bank")
        or resolver.get("frame_generation") != capture_info.get("frame_generation")
    ):
        raise ValueError(
            "resolver descriptor does not match pinned capture: "
            f"capture={capture_info} resolver={resolver}"
        )


def validate_pinned_resolver_detail_status(
    capture_info: dict[str, object], detail: dict[str, object]
) -> None:
    if detail.get("signature") != "0x44" or detail.get("version") != 1:
        raise ValueError(f"unsupported resolver detail descriptor {detail}")
    if not detail.get("completed_valid"):
        raise ValueError("resolver detail descriptor has no completed frame")
    if (
        detail.get("completed_bank") != capture_info.get("bank")
        or detail.get("frame_generation") != capture_info.get("frame_generation")
    ):
        raise ValueError(
            "resolver detail descriptor does not match pinned capture: "
            f"capture={capture_info} detail={detail}"
        )


def encode_png_rgb(width: int, height: int, rgb: bytes) -> bytes:
    if len(rgb) != width * height * 3:
        raise ValueError("RGB byte count does not match image dimensions")

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack("!I", len(data))
            + kind
            + data
            + struct.pack("!I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    stride = width * 3
    scanlines = b"".join(
        b"\x00" + rgb[offset : offset + stride]
        for offset in range(0, len(rgb), stride)
    )
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(scanlines, 6))
        + chunk(b"IEND", b"")
    )


def decode_scc_access_events(payload: bytes) -> list[dict[str, object]]:
    if len(payload) % 4:
        raise ValueError(f"SCC access trace must be a multiple of 4 bytes, got {len(payload)}")
    events = []
    for offset in range(0, len(payload), 4):
        flags, address_register, data, rr0 = payload[offset : offset + 4]
        events.append({
            "operation": "write" if flags & 0x80 else "read",
            "channel": "keyboard" if flags & 0x40 else "mouse",
            "port": "control" if flags & 0x20 else "data",
            "reset_blocked": bool(flags & 0x10),
            "irq": bool(flags & 0x08),
            "mie": bool(flags & 0x04),
            "rx_ready": bool(flags & 0x02),
            "tx_pending": bool(flags & 0x01),
            "address": address_register >> 4,
            "register": address_register & 0x0F,
            "data": f"0x{data:02x}",
            "rr0": f"0x{rr0:02x}",
        })
    return events


def decode_lg1_trace_status(payload: bytes) -> dict[str, object]:
    if len(payload) != 16:
        raise ValueError(f"LG1 trace status must be 16 bytes, got {len(payload)}")
    if payload[:2] != b"LG" or payload[2] != 1 or payload[3] != 12:
        raise ValueError(f"unsupported LG1 trace descriptor {payload[:4].hex()}")
    flags = payload[4]
    address_width = payload[5]
    if not 1 <= address_width <= 16:
        raise ValueError(f"invalid LG1 trace address width {address_width}")
    return {
        "signature": "LG",
        "version": payload[2],
        "record_size": payload[3],
        "armed": bool(flags & 0x80),
        "frozen": bool(flags & 0x40),
        "wrapped": bool(flags & 0x20),
        "command_pending": bool(flags & 0x10),
        "timing_saturated": bool(flags & 0x08),
        "writes_only": bool(flags & 0x04),
        "rex_interesting": bool(flags & 0x02),
        "address_width": address_width,
        "depth": 1 << address_width,
        "retained_count": int.from_bytes(payload[6:8], "big"),
        "oldest_pointer": int.from_bytes(payload[8:10], "big"),
        "write_pointer": int.from_bytes(payload[10:12], "big"),
        "overwrite_count": int.from_bytes(payload[12:14], "big"),
        "timing_saturation_count": int.from_bytes(payload[14:16], "big"),
    }


def decode_lg1_trace_record(payload: bytes) -> dict[str, object]:
    if len(payload) != 12:
        raise ValueError(f"LG1 trace record must be 12 bytes, got {len(payload)}")
    value = int.from_bytes(payload, "big")
    record_type = (value >> 94) & 3
    if record_type == 0:
        address = (value >> 47) & 0x7FFF
        source = (value >> 9) & 3
        return {
            "type": "access",
            "pc": (value >> 62) & 0xFFFFFFFF,
            "address": address,
            "go": bool(address & 0x0800),
            "word": (value >> 15) & 0xFFFFFFFF,
            "lane_mask": (value >> 11) & 0xF,
            "source": source,
            "source_name": ("cpu", "vdma", "capture-bist", "reserved")[source],
            "write": bool((value >> 8) & 1),
            "cycle_delta": value & 0xFF,
            "timing_saturated": (value & 0xFF) == 0xFF,
        }
    if record_type == 1:
        tag = (value >> 86) & 0xFF
        return {
            "type": "marker",
            "tag": tag,
            "tag_name": LG1_TRACE_MARKER_NAMES.get(tag, f"0x{tag:02x}"),
            "cycle": (value >> 22) & 0xFFFFFFFFFFFFFFFF,
            "configsel": (value >> 19) & 7,
            "sequence": value & 0x7FFFF,
        }
    return {"type": "reserved", "record_type": record_type,
            "raw": payload.hex()}


def expand_lg1_trace_records(records: list[bytes]) -> list[str]:
    """Expand raw board records into the existing LG1_ACCESS replay syntax."""
    lines: list[str] = []
    cycle = 0
    access_number = 0
    for ordinal, payload in enumerate(records):
        decoded = decode_lg1_trace_record(payload)
        if decoded["type"] == "marker":
            cycle = int(decoded["cycle"])
            lines.append(
                f"# LG1_TRACE_MARK ordinal={ordinal} tag={decoded['tag_name']} "
                f"cycle={cycle} cfgsel={decoded['configsel']} "
                f"sequence={decoded['sequence']}"
            )
            continue
        if decoded["type"] != "access":
            lines.append(
                f"# LG1_TRACE_RESERVED ordinal={ordinal} raw={payload.hex()}"
            )
            continue
        cycle += int(decoded["cycle_delta"])
        word = int(decoded["word"])
        is_write = bool(decoded["write"])
        address = int(decoded["address"])
        is_config = ((address >> 12) & 0x7) == 0x4
        rdata = 0 if is_write else (word << 32) | word
        wdata = word if is_write else 0
        lines.append(
            f"LG1_ACCESS @{cycle} n={access_number} "
            f"src={decoded['source']} pc={int(decoded['pc']):08x} "
            f"write={int(is_write)} go={int(bool(decoded['go']))} "
            f"cfg={int(is_config)} addr={address:04x} "
            f"wmask={int(decoded['lane_mask']):02x} "
            f"wdata={wdata:016x} rdata={rdata:016x}"
        )
        access_number += 1
    return lines


def send_operation(args: argparse.Namespace, opcode: int, payload: bytes = b"") -> int:
    request = MgmtMessage(opcode, next_sequence(), payload)
    try:
        response = transact(args.interface, request, args.timeout)
    except PermissionError:
        print("raw Ethernet access requires root or CAP_NET_RAW", file=sys.stderr)
        return 2
    except (OSError, TimeoutError, ValueError) as exc:
        print(f"cyanctl: {exc}", file=sys.stderr)
        return 1
    result = {
        "opcode": OPCODE_NAMES.get(response.opcode, f"0x{response.opcode:02x}"),
        "sequence": response.sequence,
        "status": response.status,
        "payload_hex": response.payload.hex(),
    }
    if response.status == 0 and response.opcode == OP_BOARD_STATUS:
        result["board"] = decode_board_status(response.payload)
    if response.status == 0 and response.opcode == OP_HDMI_LINK_STATUS:
        result["hdmi"] = decode_hdmi_link_status(response.payload)
    if response.status == 0 and response.opcode == OP_VIDEO_STATUS:
        result["video"] = decode_video_status(response.payload)
    if response.status == 0 and response.opcode == OP_SCAN_SERVICE_STATUS:
        result["scan_service"] = decode_scan_service_status(response.payload)
    if response.status == 0 and response.opcode == OP_SCAN_LATENCY_STATUS:
        result["scan_latency"] = decode_scan_latency_status(response.payload)
    if response.status == 0 and response.opcode == OP_GFX_RETRACE_STATUS:
        result["gfx_retrace"] = decode_gfx_retrace_status(response.payload)
    if response.status == 0 and response.opcode == OP_VIDEO_SIGNATURES:
        result["video_signatures"] = decode_video_signatures(response.payload)
    if response.status == 0 and response.opcode == OP_VIDEO_STAGE_SIGNATURES:
        result["video_stage_signatures"] = decode_video_stage_signatures(
            response.payload
        )
    if response.status == 0 and response.opcode == OP_RESOLVER_STATUS:
        result["resolver"] = decode_resolver_status(response.payload)
    if response.status == 0 and response.opcode == OP_RESOLVER_DETAIL_STATUS:
        result["resolver_detail"] = decode_resolver_detail_status(response.payload)
    if response.status == 0 and response.opcode == OP_VISUAL_STAGE_STATUS:
        result["visual_stages"] = decode_visual_stage_status(response.payload)
    if response.status == 0 and response.opcode == OP_LG1_TRACE_STATUS:
        result["lg1_trace"] = decode_lg1_trace_status(response.payload)
    if response.status == 0 and response.opcode == OP_LG1_TRACE_READ:
        result["lg1_trace_record"] = decode_lg1_trace_record(response.payload)
    if response.status == 0 and response.opcode == OP_SCC_ACCESS_EVENTS:
        result["events"] = decode_scc_access_events(response.payload)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if response.status == 0 else 1


def watch_resolver_frames(args: argparse.Namespace) -> int:
    """Emit a strictly contiguous stream of completed resolver frames."""
    sequence = next_sequence()
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(args.duration * 1_000_000_000)
    last_generation: int | None = None
    first_generation: int | None = None
    sample_index = 0
    query_count = 0
    valid_query_count = 0
    timeout_count = 0
    duplicate_count = 0
    gap_count = 0
    missed_completed_frames = 0
    backward_or_reset_count = 0
    last_activity_ns = start_ns
    failure_class: str | None = None
    failure_message: str | None = None
    duration_reached = False
    return_code = 1

    try:
        source_mac = read_interface_mac(args.interface)
        with socket.socket(
            RAW_PACKET_FAMILY, socket.SOCK_RAW, socket.htons(MGMT_ETHERTYPE)
        ) as sock:
            sock.bind((args.interface, 0))
            while time.monotonic_ns() < deadline_ns:
                query_start_ns = time.monotonic_ns()
                last_activity_ns = query_start_ns
                request = MgmtMessage(OP_RESOLVER_STATUS, sequence)
                # A timed-out response may still be in flight.  Never reuse
                # its sequence when diagnostic timeout continuation is
                # enabled, so transact_on_socket cannot attribute a late
                # response to a later observation.
                sequence = (sequence + 1) & 0xFFFFFFFF
                query_count += 1
                try:
                    response = transact_on_socket(
                        sock, source_mac, request, args.timeout
                    )
                except (TimeoutError, socket.timeout):
                    query_end_ns = time.monotonic_ns()
                    last_activity_ns = query_end_ns
                    timeout_count += 1
                    if not getattr(args, "continue_timeouts", False):
                        raise
                    print(json.dumps({
                        "type": "resolver-query-timeout",
                        "query_index": query_count - 1,
                        "sequence": request.sequence,
                        "elapsed_us": (query_end_ns - start_ns) // 1_000,
                        "query_us":
                            (query_end_ns - query_start_ns) // 1_000,
                    }, sort_keys=True), flush=True)
                    if args.interval:
                        time.sleep(args.interval)
                    continue
                query_end_ns = time.monotonic_ns()
                last_activity_ns = query_end_ns
                if response.status != 0:
                    failure_class = "endpoint-status"
                    raise RuntimeError(
                        f"resolver-status returned status {response.status}"
                    )
                resolver = decode_resolver_status(response.payload)
                try:
                    validate_resolver_watch_status(resolver)
                except ValueError:
                    failure_class = "invalid-descriptor"
                    raise
                valid_query_count += 1
                generation = int(resolver["frame_generation"])
                if last_generation is None:
                    transition = {
                        "classification": "baseline",
                        "step": 0,
                        "missed_completed_frames": 0,
                    }
                    first_generation = generation
                else:
                    transition = classify_resolver_generation_transition(
                        last_generation, generation
                    )

                transition_class = str(transition["classification"])
                if transition_class == "duplicate":
                    duplicate_count += 1
                elif transition_class == "missed":
                    gap_count += 1
                    missed_completed_frames += int(
                        transition["missed_completed_frames"]
                    )
                elif transition_class == "backward-or-reset":
                    gap_count += 1
                    backward_or_reset_count += 1

                if transition_class in ("missed", "backward-or-reset"):
                    gap_record = {
                        "type": "resolver-generation-gap",
                        "classification": transition_class,
                        "previous_generation": last_generation,
                        "frame_generation": generation,
                        "generation_step": int(transition["step"]),
                        "missed_completed_frames": int(
                            transition["missed_completed_frames"]
                        ),
                        "elapsed_us": (query_end_ns - start_ns) // 1_000,
                    }
                    print(json.dumps(gap_record, sort_keys=True), flush=True)
                    failure_class = transition_class
                    raise RuntimeError(
                        "resolver generation continuity failure: "
                        f"{transition_class} previous={last_generation} "
                        f"current={generation} step={transition['step']}"
                    )

                if args.all_samples or transition_class != "duplicate":
                    print(json.dumps({
                        "type": "resolver-frame",
                        "sample_index": sample_index,
                        "elapsed_us": (query_end_ns - start_ns) // 1_000,
                        "query_us": (query_end_ns - query_start_ns) // 1_000,
                        "generation_transition": transition_class,
                        "generation_step": int(transition["step"]),
                        "resolver": resolver,
                    }, sort_keys=True), flush=True)
                    sample_index += 1
                if transition_class != "duplicate":
                    last_generation = generation
                if args.interval:
                    time.sleep(args.interval)
            duration_reached = True
        if first_generation is None:
            failure_class = "no-baseline"
            raise RuntimeError("no valid completed resolver frame observed")
        if timeout_count:
            failure_class = "timeout"
            failure_message = (
                f"{timeout_count} management request(s) timed out; "
                "retained records may still be generation-contiguous"
            )
            print(f"cyanctl resolver watch: {failure_message}", file=sys.stderr)
            return_code = 1
        else:
            return_code = 0
    except PermissionError as exc:
        failure_class = failure_class or "permission"
        failure_message = str(exc)
        print("raw Ethernet access requires root or CAP_NET_RAW", file=sys.stderr)
        return_code = 2
    except (TimeoutError, socket.timeout) as exc:
        failure_class = failure_class or "timeout"
        failure_message = str(exc)
        print(f"cyanctl resolver watch: {exc}", file=sys.stderr)
        return_code = 1
    except (OSError, ValueError, RuntimeError) as exc:
        failure_class = failure_class or "error"
        failure_message = str(exc)
        print(f"cyanctl resolver watch: {exc}", file=sys.stderr)
        return_code = 1
    finally:
        summary = {
            "type": "resolver-watch-summary",
            "complete": return_code == 0,
            "duration_reached": duration_reached,
            "requested_duration_s": args.duration,
            "elapsed_us": max(0, last_activity_ns - start_ns) // 1_000,
            "queries": query_count,
            "valid_queries": valid_query_count,
            "timeout_queries": timeout_count,
            "records_emitted": sample_index,
            "first_generation": first_generation,
            "last_generation": last_generation,
            "duplicate_queries": duplicate_count,
            "generation_gaps": gap_count,
            "missed_completed_frames": missed_completed_frames,
            "backward_or_reset_transitions": backward_or_reset_count,
            "failure_class": failure_class,
            "failure": failure_message,
        }
        print(json.dumps(summary, sort_keys=True), flush=True)
    return return_code


def sample_rgb_points(
    points: list[tuple[int, int]],
    width: int,
    height: int,
    read_rgb_word,
) -> list[dict[str, object]]:
    """Read unique 16-pixel words and describe requested point samples."""
    words_per_line = width // 16
    word_cache: dict[int, bytes] = {}
    samples: list[dict[str, object]] = []
    for x, y in points:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise RuntimeError(
                f"capture point {x},{y} is outside {width}x{height}"
            )
        word_index = y * words_per_line + (x // 16)
        if word_index not in word_cache:
            word_cache[word_index] = read_rgb_word(word_index)
        word_rgb = word_cache[word_index]
        if len(word_rgb) not in (48, 64):
            raise RuntimeError(
                f"capture word {word_index} returned {len(word_rgb)} bytes"
            )
        pixel_offset = (x % 16) * 3
        rgb_bytes = word_rgb[:48]
        raw_indices = word_rgb[48:] if len(word_rgb) == 64 else None
        pixel = rgb_bytes[pixel_offset : pixel_offset + 3]
        samples.append({
            "x": x,
            "y": y,
            "word_index": word_index,
            "pixel_rgb": pixel.hex(),
            "raw_index": (
                f"0x{raw_indices[x % 16]:02x}"
                if raw_indices is not None else None
            ),
            "word_nonblack_pixels": sum(
                rgb_bytes[offset : offset + 3] != b"\x00\x00\x00"
                for offset in range(0, len(rgb_bytes), 3)
            ),
            "word_rgb_sha256": hashlib.sha256(rgb_bytes).hexdigest(),
            "word_raw_index_sha256": (
                hashlib.sha256(raw_indices).hexdigest()
                if raw_indices is not None else None
            ),
        })
    return samples


def read_capture_word_chunks(
    read_chunk,
    word_index: int,
    *,
    include_raw_indices: bool,
) -> bytes:
    """Read RGB chunks plus optional packed indices from old or new RTL."""
    word_data = bytearray()
    for chunk_index in range(3):
        word_data.extend(read_chunk(word_index, chunk_index))
    if include_raw_indices:
        try:
            word_data.extend(read_chunk(word_index, 3))
        except RuntimeError as exc:
            # Capture descriptor version 1 predates the optional fourth
            # chunk.  An admitted older endpoint reports INVALID (status 2).
            if str(exc) != "capture-read returned status 2":
                raise
    return bytes(word_data)


def watch_presentation_frames(args: argparse.Namespace) -> int:
    """Emit coherent pinned telemetry for each newly completed frame.

    This watcher intentionally has no target-action or trace-control surface.
    It establishes an initial generation without pinning, then uses one
    persistent raw socket to pin and inspect each subsequently observed frame.
    It never issues CAPTURE_READ.
    """
    sequence = next_sequence()
    start_ns = time.monotonic_ns()
    deadline_ns = start_ns + int(args.duration * 1_000_000_000)
    last_generation: int | None = None
    sample_index = 0
    output_stream = None
    points = getattr(args, "point", None) or []

    try:
        output_path = getattr(args, "output", None)
        if output_path is not None:
            output_stream = output_path.open("x", encoding="utf-8")
        stream = output_stream if output_stream is not None else sys.stdout
        source_mac = read_interface_mac(args.interface)
        with socket.socket(
            RAW_PACKET_FAMILY, socket.SOCK_RAW, socket.htons(MGMT_ETHERTYPE)
        ) as sock:
            sock.bind((args.interface, 0))

            def operation(opcode: int, payload: bytes = b"") -> MgmtMessage:
                nonlocal sequence
                if not presentation_watch_request_allowed(
                    opcode,
                    payload,
                    allow_rgb_word_reads=bool(points),
                ):
                    raise RuntimeError(
                        "presentation-watch rejected non-observational request "
                        f"opcode={opcode:#04x} payload={payload.hex()}"
                    )
                attempts = (
                    getattr(args, "read_retries", 3)
                    if presentation_watch_request_retryable(opcode, payload)
                    else 1
                )
                for attempt in range(attempts):
                    request = MgmtMessage(opcode, sequence, payload)
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    try:
                        request_timeout = args.timeout
                        if opcode == OP_CAPTURE_INFO and payload == b"\x01":
                            request_timeout = getattr(args, "pin_timeout", None)
                            if request_timeout is None:
                                request_timeout = args.timeout
                        elif opcode == OP_CAPTURE_READ:
                            request_timeout = getattr(args, "read_timeout", None)
                            if request_timeout is None:
                                request_timeout = args.timeout
                        response = transact_on_socket(
                            sock, source_mac, request, request_timeout
                        )
                        break
                    except OSError as exc:
                        if attempt + 1 == attempts:
                            raise RuntimeError(
                                f"{OPCODE_NAMES.get(opcode, hex(opcode))} "
                                f"transport failure after {attempts} attempts: "
                                f"{type(exc).__name__}: {exc}"
                            ) from exc
                        retry_delay = getattr(args, "retry_delay", 0.01)
                        if retry_delay:
                            time.sleep(retry_delay)
                if response.status != 0:
                    raise RuntimeError(
                        f"{OPCODE_NAMES.get(opcode, hex(opcode))} returned "
                        f"status {response.status}"
                    )
                return response

            while time.monotonic_ns() < deadline_ns:
                poll_start_ns = time.monotonic_ns()
                resolver_poll = decode_resolver_status(
                    operation(OP_RESOLVER_STATUS).payload
                )
                poll_end_ns = time.monotonic_ns()
                if (
                    resolver_poll.get("signature") != "0x52"
                    or resolver_poll.get("version") != 1
                ):
                    raise RuntimeError(
                        f"unsupported resolver descriptor {resolver_poll}"
                    )
                if not resolver_poll.get("completed_valid"):
                    if args.interval:
                        time.sleep(args.interval)
                    continue

                polled_generation = int(resolver_poll["frame_generation"])
                if last_generation is None:
                    # Establish a baseline.  The first record emitted is a
                    # frame proven to have completed after the watch began.
                    last_generation = polled_generation
                    if args.interval:
                        time.sleep(args.interval)
                    continue
                if polled_generation == last_generation:
                    if args.interval:
                        time.sleep(args.interval)
                    continue

                snapshot_start_ns = time.monotonic_ns()
                pin_attempted = False
                record: dict[str, object] | None = None
                try:
                    # A lost reply does not prove the pin command was lost, so
                    # arrange a release attempt before transmitting the pin.
                    pin_attempted = True
                    capture_info = decode_capture_info(
                        operation(OP_CAPTURE_INFO, b"\x01").payload
                    )
                    if (
                        capture_info.get("signature") != "0x43"
                        or capture_info.get("version") != 1
                        or not capture_info.get("pinned")
                        or not capture_info.get("completed_valid")
                        or capture_info.get("error")
                    ):
                        raise RuntimeError(
                            f"no stable completed presentation frame: {capture_info}"
                        )

                    signatures = decode_video_signatures(
                        operation(OP_VIDEO_SIGNATURES).payload
                    )
                    validate_pinned_video_signatures(capture_info, signatures)
                    stage_signatures = decode_video_stage_signatures(
                        operation(OP_VIDEO_STAGE_SIGNATURES).payload
                    )
                    validate_pinned_video_stage_signatures(
                        capture_info, stage_signatures
                    )
                    resolver = decode_resolver_status(
                        operation(OP_RESOLVER_STATUS).payload
                    )
                    validate_pinned_resolver_status(capture_info, resolver)
                    resolver_detail = decode_resolver_detail_status(
                        operation(OP_RESOLVER_DETAIL_STATUS).payload
                    )
                    validate_pinned_resolver_detail_status(
                        capture_info, resolver_detail
                    )
                    resolver_correlation = correlate_resolver_detail(
                        resolver, resolver_detail
                    )
                    visual_stages = decode_visual_stage_status(
                        operation(OP_VISUAL_STAGE_STATUS).payload
                    )
                    validate_pinned_visual_stage_status(
                        capture_info, visual_stages
                    )
                    point_samples: list[dict[str, object]] = []
                    if points:
                        width = int(capture_info["width"])
                        height = int(capture_info["height"])

                        def read_rgb_word(word_index: int) -> bytes:
                            def read_chunk(_word_index: int, chunk_index: int) -> bytes:
                                index = _word_index.to_bytes(2, "big")
                                response = operation(
                                    OP_CAPTURE_READ,
                                    index + bytes((chunk_index,)),
                                )
                                if len(response.payload) != 16:
                                    raise RuntimeError(
                                        f"capture word {word_index} chunk "
                                        f"{chunk_index} returned "
                                        f"{len(response.payload)} bytes"
                                    )
                                return response.payload

                            return read_capture_word_chunks(
                                read_chunk,
                                word_index,
                                include_raw_indices=True,
                            )

                        point_samples = sample_rgb_points(
                            points, width, height, read_rgb_word
                        )
                    pinned_generation = int(capture_info["frame_generation"])
                    record = {
                        "type": (
                            "presentation-point-frame" if points
                            else "presentation-frame"
                        ),
                        "sample_index": sample_index,
                        "elapsed_us": (poll_end_ns - start_ns) // 1_000,
                        "poll_query_us":
                            (poll_end_ns - poll_start_ns) // 1_000,
                        "polled_frame_generation": polled_generation,
                        "pin_raced_to_newer_generation":
                            pinned_generation != polled_generation,
                        "capture_info": capture_info,
                        "video_signatures": signatures,
                        "video_stage_signatures": stage_signatures,
                        "stage_correlation": {
                            "source_mig_low_match":
                                bool(signatures["low_half_match"]),
                            "source_mig_high_match":
                                bool(signatures["high_half_match"]),
                            "raw_fetch_low_match":
                                stage_signatures["raw_low"] ==
                                stage_signatures["fetch_low"],
                            "raw_fetch_high_match":
                                stage_signatures["raw_high"] ==
                                stage_signatures["fetch_high"],
                        },
                        "resolver": resolver,
                        "resolver_detail": resolver_detail,
                        "resolver_correlation": resolver_correlation,
                        "visual_stages": visual_stages,
                    }
                    if points:
                        record["points"] = point_samples
                    last_generation = pinned_generation
                finally:
                    if pin_attempted:
                        operation(OP_CAPTURE_INFO, b"\x02")

                assert record is not None
                snapshot_end_ns = time.monotonic_ns()
                record["snapshot_us"] = (
                    snapshot_end_ns - snapshot_start_ns
                ) // 1_000
                print(
                    json.dumps(record, sort_keys=True),
                    file=stream,
                    flush=True,
                )
                sample_index += 1
                if args.interval:
                    time.sleep(args.interval)
        return 0
    except PermissionError:
        print("raw Ethernet access requires root or CAP_NET_RAW", file=sys.stderr)
        return 2
    except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
        print(f"cyanctl presentation watch: {exc}", file=sys.stderr)
        return 1
    finally:
        if output_stream is not None:
            output_stream.close()


def dump_lg1_config_trace(args: argparse.Namespace) -> int:
    sequence = next_sequence()
    try:
        source_mac = read_interface_mac(args.interface)
        with socket.socket(
            RAW_PACKET_FAMILY, socket.SOCK_RAW, socket.htons(MGMT_ETHERTYPE)
        ) as sock:
            sock.bind((args.interface, 0))

            def operation(opcode: int, payload: bytes = b"") -> MgmtMessage:
                nonlocal sequence
                for attempt in range(args.read_retries):
                    request = MgmtMessage(opcode, sequence, payload)
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    try:
                        response = transact_on_socket(
                            sock, source_mac, request, args.timeout
                        )
                        break
                    except OSError as exc:
                        if attempt + 1 == args.read_retries:
                            raise RuntimeError(
                                f"{OPCODE_NAMES.get(opcode, hex(opcode))} "
                                f"transport failure after {args.read_retries} "
                                f"attempts: {type(exc).__name__}: {exc}"
                            ) from exc
                        if args.retry_delay:
                            time.sleep(args.retry_delay)
                if response.status != 0:
                    raise RuntimeError(
                        f"{OPCODE_NAMES.get(opcode, hex(opcode))} failed "
                        f"with status {response.status}"
                    )
                return response

            status = decode_lg1_trace_status(
                operation(OP_LG1_TRACE_STATUS).payload
            )
            if args.freeze and (status["armed"] or not status["frozen"]):
                operation(OP_LG1_TRACE_FREEZE)
                status = decode_lg1_trace_status(
                    operation(OP_LG1_TRACE_STATUS).payload
                )
            if status["armed"] or not status["frozen"]:
                raise RuntimeError(
                    "LG1 trace is live; freeze it first or pass --freeze"
                )
            count = int(status["retained_count"])
            records: list[bytes] = []
            for ordinal in range(count):
                response = operation(
                    OP_LG1_TRACE_READ, ordinal.to_bytes(2, "big")
                )
                if len(response.payload) != 12:
                    raise RuntimeError(
                        f"LG1 trace record {ordinal} has "
                        f"{len(response.payload)} bytes"
                    )
                records.append(response.payload)
                if args.progress and (ordinal + 1) % 256 == 0:
                    print(
                        f"read {ordinal + 1}/{count} LG1 config records",
                        file=sys.stderr,
                    )

        lines = [
            "# PROJECT CYAN board LG1 configuration trace v1",
            "# status=" + json.dumps(status, sort_keys=True),
        ]
        lines.extend(expand_lg1_trace_records(records))
        document = ("\n".join(lines) + "\n").encode("ascii")
        output = Path(args.output)
        output.write_bytes(document)
        result = {
            "output": str(output),
            "records": len(records),
            "sha256": hashlib.sha256(document).hexdigest(),
            "status": status,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except PermissionError:
        print("raw Ethernet access requires root or CAP_NET_RAW", file=sys.stderr)
        return 2
    except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
        print(f"cyanctl LG1 trace dump: {exc}", file=sys.stderr)
        return 1


def capture_rgb(args: argparse.Namespace) -> int:
    sequence = next_sequence()
    pinned = False
    release_error: Exception | None = None

    try:
        source_mac = read_interface_mac(args.interface)
        with socket.socket(
            RAW_PACKET_FAMILY, socket.SOCK_RAW, socket.htons(MGMT_ETHERTYPE)
        ) as sock:
            sock.bind((args.interface, 0))

            def operation(
                opcode: int,
                payload: bytes = b"",
                *,
                context: str = "",
            ) -> MgmtMessage:
                nonlocal sequence
                # A full 1024x768 RGB frame requires 147,456 read requests.
                # Retrying a lost CAPTURE_READ reply is safe because the
                # operation is read-only and addresses an already pinned bank.
                # A pin is the only non-idempotent operation in this command:
                # replaying it could select a newer completed bank.  Reads,
                # status queries, and release are safe to retry.  Release is
                # explicitly idempotent in the resampler (command 2 always
                # clears the pin), so a lost release reply must not discard an
                # otherwise complete frame or leave ownership ambiguous.
                pin_command = opcode == OP_CAPTURE_INFO and payload == b"\x01"
                attempts = 1 if pin_command else args.read_retries
                request_timeout = args.timeout
                if opcode == OP_CAPTURE_READ:
                    request_timeout = getattr(args, "read_timeout", None)
                    if request_timeout is None:
                        request_timeout = args.timeout
                for attempt in range(attempts):
                    request = MgmtMessage(opcode, sequence, payload)
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    try:
                        response = transact_on_socket(
                            sock, source_mac, request, request_timeout
                        )
                        break
                    except OSError as exc:
                        if attempt + 1 == attempts:
                            detail = f" ({context})" if context else ""
                            raise RuntimeError(
                                f"{OPCODE_NAMES[opcode]} transport failure{detail} "
                                f"after {attempts} attempts: "
                                f"{type(exc).__name__}: {exc}"
                            ) from exc
                        if args.retry_delay:
                            time.sleep(args.retry_delay)
                if response.status != 0:
                    raise RuntimeError(
                        f"{OPCODE_NAMES[opcode]} returned status {response.status}"
                    )
                return response

            try:
                pinned = True
                info = decode_capture_info(operation(OP_CAPTURE_INFO, b"\x01").payload)
                if info["signature"] != "0x43" or info["version"] != 1:
                    raise RuntimeError(f"unsupported capture descriptor {info}")
                if not info["pinned"] or not info["completed_valid"] or info["error"]:
                    raise RuntimeError(f"no stable completed RGB frame: {info}")
                width = int(info["width"])
                height = int(info["height"])
                words = int(info["words_per_frame"])
                if width == 0 or height == 0 or width % 16 != 0:
                    raise RuntimeError(f"invalid capture dimensions {width}x{height}")
                if words != (width // 16) * height:
                    raise RuntimeError("capture word count does not match dimensions")

                signatures = decode_video_signatures(
                    operation(OP_VIDEO_SIGNATURES).payload
                )
                validate_pinned_video_signatures(info, signatures)
                stage_signatures = decode_video_stage_signatures(
                    operation(OP_VIDEO_STAGE_SIGNATURES).payload
                )
                validate_pinned_video_stage_signatures(info, stage_signatures)
                resolver = decode_resolver_status(
                    operation(OP_RESOLVER_STATUS).payload
                )
                validate_pinned_resolver_status(info, resolver)
                resolver_detail = decode_resolver_detail_status(
                    operation(OP_RESOLVER_DETAIL_STATUS).payload
                )
                validate_pinned_resolver_detail_status(info, resolver_detail)
                resolver_correlation = correlate_resolver_detail(
                    resolver, resolver_detail
                )

                words_per_line = width // 16
                points = getattr(args, "point", None) or []

                def read_capture_word(word_index: int) -> bytes:
                    def read_chunk(_word_index: int, chunk_index: int) -> bytes:
                        index = _word_index.to_bytes(2, "big")
                        response = operation(
                            OP_CAPTURE_READ,
                            index + bytes((chunk_index,)),
                            context=(
                                f"word {_word_index}/{words} chunk {chunk_index}"
                            ),
                        )
                        if len(response.payload) != 16:
                            raise RuntimeError(
                                f"capture word {_word_index} chunk {chunk_index} "
                                f"returned {len(response.payload)} bytes"
                            )
                        return response.payload

                    return read_capture_word_chunks(
                        read_chunk,
                        word_index,
                        include_raw_indices=bool(points),
                    )

                point_samples: list[dict[str, object]] = []
                if points:
                    point_samples = sample_rgb_points(
                        points, width, height, read_capture_word
                    )
                else:
                    rgb = bytearray()
                    for word_index in range(words):
                        rgb.extend(read_capture_word(word_index))
                        if (args.progress and
                                word_index % (words_per_line * 64) == 0):
                            print(
                                f"captured row "
                                f"{word_index // words_per_line}/{height}",
                                file=sys.stderr,
                            )
            finally:
                if pinned:
                    try:
                        operation(OP_CAPTURE_INFO, b"\x02")
                    except Exception as exc:
                        release_error = exc

        if release_error is not None:
            raise RuntimeError(f"frame captured but bank release failed: {release_error}")
        result: dict[str, object] = {
            "width": width,
            "height": height,
            "frame_generation": info["frame_generation"],
            "bank": info["bank"],
            "video_signatures": signatures,
            "video_stage_signatures": stage_signatures,
            "resolver": resolver,
            "resolver_detail": resolver_detail,
            "resolver_correlation": resolver_correlation,
        }
        if points:
            result["points"] = point_samples
        else:
            raw = bytes(rgb)
            png = encode_png_rgb(width, height, raw)
            output = Path(args.rgb).expanduser()
            output.write_bytes(png)
            result.update({
                "output": str(output),
                "rgb_sha256": hashlib.sha256(raw).hexdigest(),
                "png_sha256": hashlib.sha256(png).hexdigest(),
            })
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except PermissionError:
        print("raw Ethernet access requires root or CAP_NET_RAW", file=sys.stderr)
        return 2
    except (OSError, TimeoutError, ValueError, RuntimeError) as exc:
        print(f"cyanctl capture: {exc}", file=sys.stderr)
        return 1


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must not be negative")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_capture_point(value: str) -> tuple[int, int]:
    try:
        x_text, y_text = value.split(",", 1)
        x = int(x_text, 10)
        y = int(y_text, 10)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("must be X,Y") from exc
    if x < 0 or y < 0:
        raise argparse.ArgumentTypeError("coordinates must not be negative")
    return x, y


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control a KCU105 CYAN target")
    subparsers = parser.add_subparsers(dest="command", required=True)

    status_parser = subparsers.add_parser("status", help="show local control-plane status")
    status_parser.add_argument("--json", action="store_true")

    subparsers.add_parser("ports", help="list KCU105 CP2105 serial interfaces")

    ether = subparsers.add_parser("ether-status", help="query the FPGA management endpoint")
    ether.add_argument("--interface", default="ens18")
    ether.add_argument("--timeout", type=float, default=1.0)

    snoop = subparsers.add_parser(
        "network-snoop",
        help="passively decode management frames without transmitting",
    )
    snoop.add_argument("--interface", default="ens18")
    snoop.add_argument(
        "--count", type=positive_int, default=1,
        help="number of valid management frames to emit (default: 1)",
    )
    snoop.add_argument(
        "--timeout", type=positive_float, default=5.0,
        help="stop after this many seconds without a valid frame",
    )

    for command, help_text in (
        ("board-status", "query live board status"),
        ("hdmi-status", "query ADV7511 link and retained-register status"),
        ("video-status", "query the full HDMI DDR-resampler status"),
        ("scan-service-status", "query the completed-frame scan miss record"),
        ("scan-latency-status", "query the completed-epoch DDR service ledger"),
        ("video-signatures", "query raw source and MIG payload signatures"),
        ("video-stage-signatures", "query framebuffer-fetch and raw signatures"),
        ("resolver-status", "classify the completed physical resolver frame"),
        ("resolver-detail-status", "query FRONT DID, XMAP-bank, and index counts"),
        ("visual-stage-status", "query DID, XMAP, route, and display-index signatures"),
        ("gfx-retrace-status", "query retained LG1 retrace/INT2 lifecycle state"),
    ):
        status_query = subparsers.add_parser(command, help=help_text)
        status_query.add_argument("--interface", default="ens18")
        status_query.add_argument("--timeout", type=float, default=1.0)

    resolver_watch = subparsers.add_parser(
        "resolver-watch",
        help="stream one physical resolver record per completed frame",
    )
    resolver_watch.add_argument("--interface", default="ens18")
    resolver_watch.add_argument("--timeout", type=float, default=1.0)
    resolver_watch.add_argument("--duration", type=nonnegative_float, default=5.0)
    resolver_watch.add_argument(
        "--interval", type=nonnegative_float, default=0.0,
        help="host delay between queries in seconds (default: none)",
    )
    resolver_watch.add_argument(
        "--all-samples", action="store_true",
        help="emit duplicate queries as well as new frame generations",
    )
    resolver_watch.add_argument(
        "--continue-timeouts", action="store_true",
        help=(
            "retain observations after explicit query-timeout records; "
            "the final summary and exit status remain fail-closed"
        ),
    )

    presentation_watch = subparsers.add_parser(
        "presentation-watch",
        help="stream coherent pinned telemetry for newly completed frames",
    )
    presentation_watch.add_argument("--interface", default="ens18")
    presentation_watch.add_argument("--timeout", type=float, default=1.0)
    presentation_watch.add_argument(
        "--pin-timeout", type=positive_float, default=None,
        help="single-shot capture-pin reply timeout (default: use --timeout)",
    )
    presentation_watch.add_argument(
        "--duration", type=nonnegative_float, default=5.0
    )
    presentation_watch.add_argument(
        "--interval", type=nonnegative_float, default=0.0,
        help="host delay between frame-generation polls (default: none)",
    )
    presentation_watch.add_argument(
        "--output", type=Path,
        help="write JSONL here, refusing to overwrite an existing file",
    )

    presentation_point_watch = subparsers.add_parser(
        "presentation-point-watch",
        help="stream coherent pinned RGB words for selected points per frame",
    )
    presentation_point_watch.add_argument("--interface", default="ens18")
    presentation_point_watch.add_argument("--timeout", type=float, default=1.0)
    presentation_point_watch.add_argument(
        "--pin-timeout", type=positive_float, default=None,
        help="single-shot capture-pin reply timeout (default: use --timeout)",
    )
    presentation_point_watch.add_argument(
        "--duration", type=nonnegative_float, default=5.0
    )
    presentation_point_watch.add_argument(
        "--interval", type=nonnegative_float, default=0.0,
        help="host delay between frame-generation polls (default: none)",
    )
    presentation_point_watch.add_argument(
        "--point", action="append", type=parse_capture_point,
        metavar="X,Y", required=True,
        help="sample this pixel and containing 16-pixel RGB word; repeatable",
    )
    presentation_point_watch.add_argument(
        "--read-retries", type=positive_int, default=20,
        help="retry a timed-out read of the same pinned RGB word (default: 20)",
    )
    presentation_point_watch.add_argument(
        "--read-timeout", type=positive_float, default=None,
        help=(
            "seconds to wait for each pinned RGB read reply; control requests "
            "still use --timeout (default: use --timeout)"
        ),
    )
    presentation_point_watch.add_argument(
        "--retry-delay", type=nonnegative_float, default=0.01,
        help="seconds between pinned RGB read retries (default: 0.01)",
    )
    presentation_point_watch.add_argument(
        "--output", type=Path,
        help="write JSONL here, refusing to overwrite an existing file",
    )

    fan_parser = subparsers.add_parser("fan", help="set the FPGA fan PWM duty")
    fan_parser.add_argument("duty", type=int, choices=(25, 50, 75, 100))
    fan_parser.add_argument("--interface", default="ens18")
    fan_parser.add_argument("--timeout", type=float, default=1.0)

    input_parser = subparsers.add_parser("input", help="inject real SCC keyboard/mouse bytes")
    input_parser.add_argument("--interface", default="ens18")
    input_parser.add_argument("--timeout", type=float, default=1.0)
    input_subparsers = input_parser.add_subparsers(dest="input_kind", required=True)

    key_parser = input_subparsers.add_parser("key")
    key_parser.add_argument("key")
    key_parser.add_argument("--release", action="store_true")

    text_parser = input_subparsers.add_parser("text")
    text_parser.add_argument("text")

    mouse_parser = input_subparsers.add_parser("mouse")
    mouse_parser.add_argument("--buttons", type=int, default=0)
    mouse_parser.add_argument("--dx", type=int, default=0)
    mouse_parser.add_argument("--dy", type=int, default=0)

    trace_parser = subparsers.add_parser(
        "scc-trace", help="drain captured guest-to-keyboard/mouse SCC bytes"
    )
    trace_parser.add_argument(
        "stream",
        choices=("keyboard-tx", "mouse-tx", "keyboard-rx", "mouse-rx"),
    )
    trace_parser.add_argument("--interface", default="ens18")
    trace_parser.add_argument("--timeout", type=float, default=1.0)

    access_trace = subparsers.add_parser(
        "scc-access-trace", help="drain captured SCC0 CPU register accesses"
    )
    access_trace.add_argument("--interface", default="ens18")
    access_trace.add_argument("--timeout", type=float, default=1.0)
    trace_control = access_trace.add_mutually_exclusive_group()
    trace_control.add_argument("--arm-input", action="store_true")
    trace_control.add_argument("--clear", action="store_true")
    trace_control.add_argument("--disarm", action="store_true")

    lg1_trace = subparsers.add_parser(
        "lg1-config-trace",
        help="control or dump the passive accepted-LG1 configuration trace",
    )
    lg1_trace_sub = lg1_trace.add_subparsers(
        dest="lg1_trace_action", required=True
    )
    for action in ("arm", "arm-writes", "arm-rex", "freeze", "status"):
        action_parser = lg1_trace_sub.add_parser(action)
        action_parser.add_argument("--interface", default="ens18")
        action_parser.add_argument("--timeout", type=float, default=1.0)
    mark_parser = lg1_trace_sub.add_parser("mark")
    mark_parser.add_argument("marker", choices=tuple(LG1_TRACE_MARKERS))
    mark_parser.add_argument("--interface", default="ens18")
    mark_parser.add_argument("--timeout", type=float, default=1.0)
    dump_parser = lg1_trace_sub.add_parser("dump")
    dump_parser.add_argument("--output", required=True)
    dump_parser.add_argument("--freeze", action="store_true")
    dump_parser.add_argument("--progress", action="store_true")
    dump_parser.add_argument("--read-retries", type=positive_int, default=20)
    dump_parser.add_argument(
        "--retry-delay", type=nonnegative_float, default=0.01
    )
    dump_parser.add_argument("--interface", default="ens18")
    dump_parser.add_argument("--timeout", type=float, default=1.0)

    capture = subparsers.add_parser(
        "capture", help="capture a pinned final resolved-RGB frame"
    )
    capture_output = capture.add_mutually_exclusive_group(required=True)
    capture_output.add_argument("--rgb", metavar="OUTPUT.png")
    capture_output.add_argument(
        "--point",
        action="append",
        type=parse_capture_point,
        metavar="X,Y",
        help=(
            "sample one pixel and its containing 16-pixel RGB word from the "
            "pinned frame; repeat for multiple points instead of --rgb"
        ),
    )
    capture.add_argument("--interface", default="ens18")
    capture.add_argument("--timeout", type=float, default=1.0)
    capture.add_argument(
        "--read-retries",
        type=positive_int,
        default=20,
        help="retry a timed-out read of the same pinned RGB word (default: 20)",
    )
    capture.add_argument(
        "--read-timeout",
        type=positive_float,
        default=None,
        help=(
            "seconds to wait for each pinned RGB read reply; control requests "
            "still use --timeout (default: use --timeout)"
        ),
    )
    capture.add_argument(
        "--retry-delay",
        type=nonnegative_float,
        default=0.01,
        help="seconds between pinned RGB read retries (default: 0.01)",
    )
    capture.add_argument("--progress", action="store_true")
    return parser


def main(argv: list[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "ports":
        for port in discover_cp2105_ports():
            owners = console_owners(port["device"])
            suffix = f" owners={','.join(owners)}" if owners else " free"
            print(
                f"{port['device']} interface={port['interface']} "
                f"role={port['role']}{suffix}"
            )
        return 0

    if args.command == "status":
        document = status_document()
        if args.json:
            print(json.dumps(document, indent=2, sort_keys=True))
        else:
            print(
                f"source {document['source']['commit']} "
                f"tracked_dirty={document['source']['tracked_dirty']}"
            )
            for port in document["serial"]:
                owners = port["owners"]
                suffix = f" owners={','.join(owners)}" if owners else " free"
                print(f"{port['device']} {port['role']}{suffix}")
            mgmt = document["management"]
            print(
                f"management mac={mgmt['destination_mac']} "
                f"ethertype={mgmt['ethertype']} rtl={mgmt['rtl_endpoint']}"
            )
        return 0

    if args.command == "ether-status":
        return send_operation(args, OP_STATUS)

    if args.command == "network-snoop":
        return snoop_management_frames(args)

    if args.command == "board-status":
        return send_operation(args, OP_BOARD_STATUS)

    if args.command == "hdmi-status":
        return send_operation(args, OP_HDMI_LINK_STATUS)

    if args.command == "video-status":
        return send_operation(args, OP_VIDEO_STATUS)

    if args.command == "scan-service-status":
        return send_operation(args, OP_SCAN_SERVICE_STATUS)

    if args.command == "scan-latency-status":
        return send_operation(args, OP_SCAN_LATENCY_STATUS)

    if args.command == "gfx-retrace-status":
        return send_operation(args, OP_GFX_RETRACE_STATUS)

    if args.command == "video-signatures":
        return send_operation(args, OP_VIDEO_SIGNATURES)

    if args.command == "video-stage-signatures":
        return send_operation(args, OP_VIDEO_STAGE_SIGNATURES)

    if args.command == "resolver-status":
        return send_operation(args, OP_RESOLVER_STATUS)

    if args.command == "resolver-detail-status":
        return send_operation(args, OP_RESOLVER_DETAIL_STATUS)

    if args.command == "visual-stage-status":
        return send_operation(args, OP_VISUAL_STAGE_STATUS)

    if args.command == "resolver-watch":
        return watch_resolver_frames(args)

    if args.command in ("presentation-watch", "presentation-point-watch"):
        if not hasattr(args, "read_retries"):
            args.read_retries = 3
            args.retry_delay = 0.01
        return watch_presentation_frames(args)

    if args.command == "fan":
        return send_operation(args, OP_FAN, bytes((args.duty,)))

    if args.command == "input":
        try:
            if args.input_kind == "key":
                payload = encode_key(args.key, args.release)
                opcode = OP_KBD_BYTES
            elif args.input_kind == "text":
                payload = encode_text(args.text)
                opcode = OP_KBD_BYTES
            else:
                payload = encode_mouse(args.buttons, args.dx, args.dy)
                opcode = OP_MOUSE_BYTES
        except ValueError as exc:
            parser.error(str(exc))
        return send_operation(args, opcode, payload)

    if args.command == "scc-trace":
        opcode = {
            "keyboard-tx": OP_KBD_TX_EVENTS,
            "mouse-tx": OP_MOUSE_TX_EVENTS,
            "keyboard-rx": OP_KBD_RX_EVENTS,
            "mouse-rx": OP_MOUSE_RX_EVENTS,
        }[args.stream]
        return send_operation(args, opcode)

    if args.command == "scc-access-trace":
        if args.arm_input:
            return send_operation(args, OP_SCC_TRACE_CONTROL, b"\x01")
        if args.clear:
            return send_operation(args, OP_SCC_TRACE_CONTROL, b"\x02")
        if args.disarm:
            return send_operation(args, OP_SCC_TRACE_CONTROL, b"\x00")
        return send_operation(args, OP_SCC_ACCESS_EVENTS)

    if args.command == "lg1-config-trace":
        if args.lg1_trace_action == "arm":
            return send_operation(args, OP_LG1_TRACE_ARM)
        if args.lg1_trace_action == "arm-writes":
            return send_operation(args, OP_LG1_TRACE_ARM, b"\x01")
        if args.lg1_trace_action == "arm-rex":
            return send_operation(args, OP_LG1_TRACE_ARM, b"\x02")
        if args.lg1_trace_action == "mark":
            return send_operation(
                args,
                OP_LG1_TRACE_MARK,
                bytes((LG1_TRACE_MARKERS[args.marker],)),
            )
        if args.lg1_trace_action == "freeze":
            return send_operation(args, OP_LG1_TRACE_FREEZE)
        if args.lg1_trace_action == "status":
            return send_operation(args, OP_LG1_TRACE_STATUS)
        if args.lg1_trace_action == "dump":
            return dump_lg1_config_trace(args)

    if args.command == "capture":
        return capture_rgb(args)

    parser.error(f"unhandled command {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
