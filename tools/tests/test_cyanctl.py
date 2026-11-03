#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "cyanctl.py"
SPEC = importlib.util.spec_from_file_location("cyanctl", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
cyanctl = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = cyanctl
SPEC.loader.exec_module(cyanctl)


class MessageTests(unittest.TestCase):
    def test_message_round_trip(self):
        original = cyanctl.MgmtMessage(
            cyanctl.OP_KBD_BYTES,
            0x12345678,
            b"keyboard",
            cyanctl.FLAG_RESPONSE,
            7,
        )
        self.assertEqual(cyanctl.MgmtMessage.decode(original.encode()), original)

    def test_checksum_rejects_corruption(self):
        encoded = bytearray(cyanctl.MgmtMessage(cyanctl.OP_STATUS, 3, b"abc").encode())
        encoded[-1] ^= 0x80
        with self.assertRaisesRegex(ValueError, "checksum"):
            cyanctl.MgmtMessage.decode(encoded)

    def test_ethernet_frame_round_trip(self):
        source = bytes.fromhex("408d5cbfe96c")
        message = cyanctl.MgmtMessage(cyanctl.OP_STATUS, 99)
        frame = cyanctl.encode_ethernet_frame(source, message)
        destination, decoded_source, decoded = cyanctl.decode_ethernet_frame(
            frame, cyanctl.MGMT_MAC
        )
        self.assertEqual(destination, cyanctl.MGMT_MAC)
        self.assertEqual(decoded_source, source)
        self.assertEqual(decoded, message)
        self.assertGreaterEqual(len(frame), 60)

    def test_response_targets_requesting_host(self):
        host = bytes.fromhex("408d5cbfe96c")
        message = cyanctl.MgmtMessage(
            cyanctl.OP_STATUS, 99, flags=cyanctl.FLAG_RESPONSE
        )
        payload = message.encode()
        payload += bytes(max(0, cyanctl.ETH_MIN_PAYLOAD - len(payload)))
        frame = cyanctl.ETH_HEADER.pack(
            host, cyanctl.MGMT_MAC, cyanctl.MGMT_ETHERTYPE
        ) + payload
        destination, source, decoded = cyanctl.decode_ethernet_frame(frame, host)
        self.assertEqual(destination, host)
        self.assertEqual(source, cyanctl.MGMT_MAC)
        self.assertEqual(decoded, message)

    def test_wrong_response_destination_is_rejected(self):
        host = bytes.fromhex("408d5cbfe96c")
        frame = cyanctl.encode_ethernet_frame(
            host, cyanctl.MgmtMessage(cyanctl.OP_STATUS, 99)
        )
        with self.assertRaisesRegex(ValueError, "destination"):
            cyanctl.decode_ethernet_frame(frame, host)

    def test_network_snoop_is_receive_only(self):
        host = bytes.fromhex("408d5cbfe96c")
        frame = cyanctl.encode_ethernet_frame(
            host, cyanctl.MgmtMessage(cyanctl.OP_BOARD_STATUS, 0x12345678)
        )

        class FakeSocket:
            def __init__(self):
                self.bound = None
                self.timeout = None
                self.frames = [frame]

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, address):
                self.bound = address

            def settimeout(self, timeout):
                self.timeout = timeout

            def recv(self, _size):
                return self.frames.pop(0)

        fake = FakeSocket()
        args = SimpleNamespace(interface="test0", timeout=0.5, count=1)
        with mock.patch.object(cyanctl.socket, "socket", return_value=fake), \
                mock.patch("sys.stdout", new_callable=io.StringIO) as stdout:
            self.assertEqual(cyanctl.snoop_management_frames(args), 0)

        record = json.loads(stdout.getvalue())
        self.assertEqual(fake.bound, ("test0", 0))
        self.assertEqual(record["opcode_name"], "board-status")
        self.assertEqual(record["sequence"], 0x12345678)
        self.assertFalse(record["response"])
        self.assertEqual(record["payload_hex"], "")

    def test_network_snoop_parser_defaults(self):
        args = cyanctl.build_parser().parse_args(("network-snoop",))
        self.assertEqual(args.interface, "ens18")
        self.assertEqual(args.count, 1)
        self.assertEqual(args.timeout, 5.0)


class InputEncodingTests(unittest.TestCase):
    def test_text_matches_cosim_encoding(self):
        self.assertEqual(
            cyanctl.encode_text("Az 0\n"),
            bytes((5, 10, 0x8A, 0x85, 19, 0x93, 82, 0xD2, 45, 0xAD, 50, 0xB2)),
        )

    def test_named_key_press_and_release(self):
        self.assertEqual(cyanctl.encode_key("left"), bytes((72,)))
        self.assertEqual(cyanctl.encode_key("left", release=True), bytes((0xC8,)))

    def test_sgi_modifier_keycodes(self):
        self.assertEqual(cyanctl.encode_key("leftshift"), b"\x05")
        self.assertEqual(cyanctl.encode_key("leftshift", release=True), b"\x85")
        self.assertEqual(cyanctl.encode_key("rightshift"), b"\x04")
        self.assertEqual(cyanctl.encode_key("rightshift", release=True), b"\x84")
        self.assertEqual(cyanctl.encode_key("capslock"), b"\x03")

    def test_controller_shift_x_sequence(self):
        self.assertEqual(
            b"".join((
                cyanctl.encode_key("leftshift"),
                cyanctl.encode_key("x"),
                cyanctl.encode_key("x", release=True),
                cyanctl.encode_key("leftshift", release=True),
            )),
            b"\x05\x14\x94\x85",
        )

    def test_controller_question_mark_sequence(self):
        self.assertEqual(
            b"".join((
                cyanctl.encode_key("leftshift"),
                cyanctl.encode_key("/"),
                cyanctl.encode_key("/", release=True),
                cyanctl.encode_key("leftshift", release=True),
            )),
            b"\x05\x34\xb4\x85",
        )

    def test_text_encoder_shifts_question_mark(self):
        self.assertEqual(
            cyanctl.encode_text("?"),
            b"\x05\x34\xb4\x85",
        )

    def test_text_encoder_shifts_us_ascii_symbols(self):
        shifted = "~!@#$%^&*()_+{}|:\"<>?"
        bases = "`1234567890-=[]\\;',./"
        expected = bytearray()
        for base in bases:
            code = cyanctl.scancode_for_key(base)
            expected.extend((0x05, code, code | 0x80, 0x85))
        self.assertEqual(cyanctl.encode_text(shifted), bytes(expected))

    def test_mouse_packet_matches_cosim_encoding(self):
        self.assertEqual(
            cyanctl.encode_mouse(1, 12, -9),
            bytes((0x83, 12, 0xF7, 0, 0)),
        )

    def test_large_mouse_delta_is_split(self):
        encoded = cyanctl.encode_mouse(0, 128, -129)
        self.assertEqual(encoded[:5], bytes((0x87, 127, 0x80, 0, 0)))
        self.assertEqual(encoded[5:], bytes((0x87, 1, 0xFF, 0, 0)))


class TraceCommandTests(unittest.TestCase):
    def test_resolver_watch_parser_defaults_and_bounds(self):
        args = cyanctl.build_parser().parse_args([
            "resolver-watch", "--duration", "1.25", "--interval", "0.002",
        ])
        self.assertEqual(args.command, "resolver-watch")
        self.assertEqual(args.interface, "ens18")
        self.assertEqual(args.duration, 1.25)
        self.assertEqual(args.interval, 0.002)
        self.assertFalse(args.all_samples)
        self.assertFalse(args.continue_timeouts)
        self.assertTrue(cyanctl.build_parser().parse_args([
            "resolver-watch", "--continue-timeouts",
        ]).continue_timeouts)
        with self.assertRaises(SystemExit):
            cyanctl.build_parser().parse_args([
                "resolver-watch", "--duration", "-1",
            ])

    def test_presentation_watch_parser_is_observe_only(self):
        args = cyanctl.build_parser().parse_args([
            "presentation-watch", "--duration", "1.25",
            "--interval", "0.002", "--interface", "test0",
        ])
        self.assertEqual(args.command, "presentation-watch")
        self.assertEqual(args.interface, "test0")
        self.assertEqual(args.duration, 1.25)
        self.assertEqual(args.interval, 0.002)
        self.assertIsNone(args.output)
        self.assertFalse(hasattr(args, "action"))
        with self.assertRaises(SystemExit):
            cyanctl.build_parser().parse_args([
                "presentation-watch", "--duration", "-1",
            ])

    def test_presentation_watch_request_allowlist_is_read_only(self):
        self.assertEqual(
            cyanctl.PRESENTATION_WATCH_ALLOWED_REQUESTS,
            frozenset({
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_CAPTURE_INFO, b"\x01"),
                (cyanctl.OP_CAPTURE_INFO, b"\x02"),
                (cyanctl.OP_VIDEO_SIGNATURES, b""),
                (cyanctl.OP_VIDEO_STAGE_SIGNATURES, b""),
                (cyanctl.OP_RESOLVER_DETAIL_STATUS, b""),
                (cyanctl.OP_VISUAL_STAGE_STATUS, b""),
            }),
        )
        dangerous = {
            cyanctl.OP_RESET,
            cyanctl.OP_FAN,
            cyanctl.OP_KBD_BYTES,
            cyanctl.OP_MOUSE_BYTES,
            cyanctl.OP_KBD_TX_EVENTS,
            cyanctl.OP_MOUSE_TX_EVENTS,
            cyanctl.OP_KBD_RX_EVENTS,
            cyanctl.OP_MOUSE_RX_EVENTS,
            cyanctl.OP_SCC_ACCESS_EVENTS,
            cyanctl.OP_SCC_TRACE_CONTROL,
            cyanctl.OP_LG1_TRACE_ARM,
            cyanctl.OP_LG1_TRACE_MARK,
            cyanctl.OP_LG1_TRACE_FREEZE,
            cyanctl.OP_LG1_TRACE_STATUS,
            cyanctl.OP_LG1_TRACE_READ,
            cyanctl.OP_CAPTURE_READ,
        }
        self.assertTrue(dangerous.isdisjoint(
            opcode for opcode, _payload in
            cyanctl.PRESENTATION_WATCH_ALLOWED_REQUESTS
        ))

    def test_presentation_point_watch_parser_and_request_boundary(self):
        args = cyanctl.build_parser().parse_args([
            "presentation-point-watch", "--duration", "3",
            "--point", "608,350", "--point", "736,400",
            "--read-retries", "7", "--read-timeout", "0.15",
            "--pin-timeout", "1.0",
        ])
        self.assertEqual(args.command, "presentation-point-watch")
        self.assertEqual(args.point, [(608, 350), (736, 400)])
        self.assertEqual(args.read_retries, 7)
        self.assertEqual(args.read_timeout, 0.15)
        self.assertEqual(args.pin_timeout, 1.0)
        self.assertTrue(cyanctl.presentation_watch_request_allowed(
            cyanctl.OP_CAPTURE_READ, bytes.fromhex("123403"),
            allow_rgb_word_reads=True,
        ))
        self.assertFalse(cyanctl.presentation_watch_request_allowed(
            cyanctl.OP_CAPTURE_READ, bytes.fromhex("123404"),
            allow_rgb_word_reads=True,
        ))
        self.assertFalse(cyanctl.presentation_watch_request_retryable(
            cyanctl.OP_CAPTURE_INFO, b"\x01"
        ))
        self.assertTrue(cyanctl.presentation_watch_request_retryable(
            cyanctl.OP_CAPTURE_INFO, b"\x02"
        ))
        self.assertTrue(cyanctl.presentation_watch_request_retryable(
            cyanctl.OP_CAPTURE_READ, bytes.fromhex("123402")
        ))
        self.assertTrue(cyanctl.presentation_watch_request_retryable(
            cyanctl.OP_RESOLVER_STATUS, b""
        ))
        for opcode in (
            cyanctl.OP_RESET,
            cyanctl.OP_FAN,
            cyanctl.OP_KBD_BYTES,
            cyanctl.OP_MOUSE_BYTES,
            cyanctl.OP_LG1_TRACE_ARM,
            cyanctl.OP_LG1_TRACE_FREEZE,
        ):
            self.assertFalse(cyanctl.presentation_watch_request_allowed(
                opcode, b"", allow_rgb_word_reads=True,
            ))
        with self.assertRaises(SystemExit):
            cyanctl.build_parser().parse_args(["presentation-point-watch"])

    def test_lg1_trace_status_decode(self):
        payload = bytes.fromhex(
            "4c47010c680e12340002000600070008"
        )
        decoded = cyanctl.decode_lg1_trace_status(payload)
        self.assertEqual(decoded["signature"], "LG")
        self.assertEqual(decoded["record_size"], 12)
        self.assertFalse(decoded["armed"])
        self.assertTrue(decoded["frozen"])
        self.assertTrue(decoded["wrapped"])
        self.assertTrue(decoded["timing_saturated"])
        self.assertFalse(decoded["writes_only"])
        self.assertEqual(decoded["depth"], 16384)
        self.assertEqual(decoded["retained_count"], 0x1234)
        self.assertEqual(decoded["oldest_pointer"], 2)
        self.assertEqual(decoded["write_pointer"], 6)
        self.assertEqual(decoded["overwrite_count"], 7)
        self.assertEqual(decoded["timing_saturation_count"], 8)

        write_only = cyanctl.decode_lg1_trace_status(bytes.fromhex(
            "4c47010c440e00030000000300000000"
        ))
        self.assertTrue(write_only["frozen"])
        self.assertTrue(write_only["writes_only"])

        rex_interesting = cyanctl.decode_lg1_trace_status(bytes.fromhex(
            "4c47010c420e00040000000400000000"
        ))
        self.assertTrue(rex_interesting["frozen"])
        self.assertFalse(rex_interesting["writes_only"])
        self.assertTrue(rex_interesting["rex_interesting"])

    def test_lg1_trace_access_and_marker_decode(self):
        access_value = (
            (0x5E307084 << 62)
            | (0x4FE8 << 47)
            | (0x00000082 << 15)
            | (0xF << 11)
            | (1 << 8)
            | 7
        )
        access = cyanctl.decode_lg1_trace_record(
            access_value.to_bytes(12, "big")
        )
        self.assertEqual(access["type"], "access")
        self.assertEqual(access["pc"], 0x5E307084)
        self.assertEqual(access["address"], 0x4FE8)
        self.assertTrue(access["go"])
        self.assertTrue(access["write"])
        self.assertEqual(access["word"], 0x82)
        self.assertEqual(access["lane_mask"], 0xF)
        self.assertEqual(access["cycle_delta"], 7)

        marker_value = (
            (1 << 94) | (3 << 86) | (0x123456789ABCDEF0 << 22)
            | (5 << 19) | 9
        )
        marker = cyanctl.decode_lg1_trace_record(
            marker_value.to_bytes(12, "big")
        )
        self.assertEqual(marker["type"], "marker")
        self.assertEqual(marker["tag_name"], "twilight-active")
        self.assertEqual(marker["cycle"], 0x123456789ABCDEF0)
        self.assertEqual(marker["configsel"], 5)
        self.assertEqual(marker["sequence"], 9)

    def test_lg1_trace_expands_existing_replay_syntax(self):
        marker = (
            (1 << 94) | (2 << 86) | (1000 << 22) | (1 << 19)
        ).to_bytes(12, "big")
        access = (
            (0x5E307084 << 62) | (0x47E8 << 47)
            | (0x82 << 15) | (0xF << 11) | (1 << 8) | 3
        ).to_bytes(12, "big")
        rex_access = (
            (0x5E307088 << 62) | (0x003C << 47)
            | (0x12345 << 15) | (0xF << 11) | (1 << 8) | 4
        ).to_bytes(12, "big")
        lines = cyanctl.expand_lg1_trace_records(
            [marker, access, rex_access]
        )
        self.assertIn("tag=twilight-pre-xwd", lines[0])
        self.assertIn("LG1_ACCESS @1003 n=0", lines[1])
        self.assertIn("pc=5e307084", lines[1])
        self.assertIn("cfg=1", lines[1])
        self.assertIn("addr=47e8", lines[1])
        self.assertIn("wmask=0f", lines[1])
        self.assertIn("wdata=0000000000000082", lines[1])
        self.assertIn("LG1_ACCESS @1007 n=1", lines[2])
        self.assertIn("pc=5e307088", lines[2])
        self.assertIn("cfg=0", lines[2])
        self.assertIn("addr=003c", lines[2])

    def test_lg1_trace_rejects_bad_lengths(self):
        with self.assertRaisesRegex(ValueError, "16 bytes"):
            cyanctl.decode_lg1_trace_status(b"short")
        with self.assertRaisesRegex(ValueError, "12 bytes"):
            cyanctl.decode_lg1_trace_record(b"short")

    def test_video_status_decode(self):
        value = (
            (0x56 << 120)
            | (1 << 119)
            | (2 << 117)
            | (1 << 115)
            | (2 << 113)
            | (1 << 112)
            | (3 << 105)
            | (7 << 89)
            | (8 << 73)
            | (9 << 57)
            | (10 << 41)
            | (49152 << 25)
            | (5 << 22)
            | (1 << 18)
            | (1 << 17)
            | (1 << 16)
            | (1 << 15)
            | (1 << 14)
            | (1 << 13)
            | (1 << 12)
            | (0x5A << 2)
        )
        decoded = cyanctl.decode_video_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x56")
        self.assertEqual(decoded["display_bank"], 1)
        self.assertEqual(decoded["source_bank"], 2)
        self.assertTrue(decoded["source_capture_enabled"])
        # Retain the old, misleading names only as compatibility aliases.
        self.assertEqual(decoded["capture_bank"], decoded["source_bank"])
        self.assertEqual(
            decoded["capture_enabled"], decoded["source_capture_enabled"]
        )
        self.assertEqual(decoded["mig_state"], 3)
        self.assertEqual(decoded["underruns"], 7)
        self.assertEqual(decoded["mig_writes"], 10)
        self.assertEqual(decoded["source_words"], 49152)
        self.assertEqual(decoded["inflight_banks"], 5)
        self.assertTrue(decoded["payload_signature_valid"])
        self.assertTrue(decoded["payload_low_half_match"])
        self.assertTrue(decoded["payload_high_half_match"])
        self.assertFalse(decoded["lg1_underrun_seen"])
        self.assertFalse(decoded["lg1_protocol_error_seen"])
        self.assertEqual(decoded["payload_signature_generation"], 0x5A)

    def test_scan_service_status_decode(self):
        flags = 0x40 | 0x20 | 0x08 | 0x02 | 0x01
        value = (
            (0x51 << 120)
            | (1 << 112)
            | (1 << 111)
            | (0xA5 << 103)
            | (954 << 87)
            | (92 << 71)
            | (0xA4 << 63)
            | (91 << 47)
            | (flags << 40)
        )
        decoded = cyanctl.decode_scan_service_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x51")
        self.assertEqual(decoded["epoch"], 0xA5)
        self.assertEqual(decoded["misses"], 954)
        self.assertEqual(decoded["first_miss_index"], 92)
        self.assertEqual(decoded["first_head_epoch"], 0xA4)
        self.assertEqual(decoded["first_head_index"], 91)
        self.assertTrue(decoded["first_miss_seen"])
        self.assertTrue(decoded["head_valid"])
        self.assertFalse(decoded["head_exact"])
        self.assertTrue(decoded["head_obsolete"])
        self.assertFalse(decoded["head_future"])
        self.assertTrue(decoded["fifo_empty"])
        self.assertTrue(decoded["scan_valid"])

    def test_scan_latency_status_decode(self):
        flags = 0x20 | 0x10 | 0x08 | 0x04 | 0x02 | 0x01
        value = (
            (0x4C << 120)
            | (1 << 112)
            | (0xA5 << 104)
            | (49152 << 88)
            | (49152 << 72)
            | (5500 << 56)
            | (144 << 40)
            | (3 << 30)
            | (255 << 22)
            | (17 << 14)
            | (2 << 6)
            | flags
        )
        decoded = cyanctl.decode_scan_latency_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x4c")
        self.assertEqual(decoded["epoch"], 0xA5)
        self.assertEqual(decoded["commands"], 49152)
        self.assertEqual(decoded["responses"], 49152)
        self.assertEqual(decoded["max_command_wait_cycles"], 5500)
        self.assertEqual(decoded["max_response_latency_cycles"], 144)
        self.assertEqual(decoded["minimum_resident_words"], 0)
        self.assertEqual(decoded["maximum_outstanding_reads"], 3)
        self.assertEqual(decoded["responses_le_32_cycles"], 255)
        self.assertEqual(decoded["responses_33_to_128_cycles"], 17)
        self.assertEqual(decoded["responses_gt_128_cycles"], 2)
        self.assertTrue(decoded["valid"])
        self.assertTrue(decoded["epoch_matches_video"])
        self.assertTrue(decoded["commands_match_responses"])
        self.assertTrue(decoded["resident_reached_zero"])
        self.assertTrue(decoded["slow_response_seen"])

    def test_gfx_retrace_status_decode(self):
        payload = bytes.fromhex(
            "52 1e 80 1e 00 01 23 00 01 20 00 00 fe 00 00 ff"
        )
        decoded = cyanctl.decode_gfx_retrace_status(payload)
        self.assertEqual(decoded["signature"], "0x52")
        self.assertEqual(decoded["port_config"], "0x1e")
        self.assertEqual(decoded["lio1_mask"], "0x80")
        self.assertFalse(decoded["raw_now"])
        self.assertTrue(decoded["pending"])
        self.assertTrue(decoded["cpu_irq1"])
        self.assertTrue(decoded["armed"])
        self.assertTrue(decoded["mask7"])
        self.assertEqual(decoded["raw_count"], 0x123)
        self.assertEqual(decoded["armed_raw_count"], 0x120)
        self.assertEqual(decoded["clear_write_count"], 0xFE)
        self.assertEqual(decoded["arm_write_count"], 0xFF)

        with self.assertRaisesRegex(ValueError, "must be 16 bytes"):
            cyanctl.decode_gfx_retrace_status(b"short")

    def test_video_signature_status_decode(self):
        value = (
            (0x53 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (0x796A << 77)
            | (0xFDC8 << 61)
            | (0x3A88 << 45)
            | (0xBC82 << 29)
            | (0x5A << 21)
        )
        decoded = cyanctl.decode_video_signatures(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x53")
        self.assertEqual(decoded["version"], 1)
        self.assertTrue(decoded["completed_valid"])
        self.assertEqual(decoded["completed_bank"], 2)
        self.assertEqual(decoded["frame_generation"], 0x1234)
        self.assertEqual(decoded["source_low"], "0x796a")
        self.assertEqual(decoded["source_high"], "0xfdc8")
        self.assertEqual(decoded["mig_low"], "0x3a88")
        self.assertEqual(decoded["mig_high"], "0xbc82")
        self.assertEqual(decoded["signature_generation"], 0x5A)
        self.assertFalse(decoded["low_half_match"])
        self.assertFalse(decoded["high_half_match"])

    def test_video_stage_signature_status_decode(self):
        value = (
            (0x46 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (0x796A << 77)
            | (0xFDC8 << 61)
            | (0x3A88 << 45)
            | (0xBC82 << 29)
            | (0x5A << 21)
            | (1 << 20)
            | (0x9A << 12)
        )
        decoded = cyanctl.decode_video_stage_signatures(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x46")
        self.assertEqual(decoded["version"], 1)
        self.assertTrue(decoded["completed_valid"])
        self.assertEqual(decoded["completed_bank"], 2)
        self.assertEqual(decoded["frame_generation"], 0x1234)
        self.assertEqual(decoded["raw_low"], "0x796a")
        self.assertEqual(decoded["raw_high"], "0xfdc8")
        self.assertEqual(decoded["fetch_low"], "0x3a88")
        self.assertEqual(decoded["fetch_high"], "0xbc82")
        self.assertEqual(decoded["signature_generation"], 0x5A)
        self.assertTrue(decoded["fetch_valid"])
        self.assertEqual(decoded["fetch_generation"], 0x9A)
        capture = {"bank": 2, "frame_generation": 0x1234}
        cyanctl.validate_pinned_video_stage_signatures(capture, decoded)
        with self.assertRaises(ValueError):
            cyanctl.validate_pinned_video_stage_signatures(
                {"bank": 1, "frame_generation": 0x1234}, decoded
            )

    def test_visual_stage_status_decode_and_pinned_validation(self):
        value = (
            (0x50 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (0x1111 << 77)
            | (0x2222 << 61)
            | (0x3333 << 45)
            | (0x4444 << 29)
        )
        decoded = cyanctl.decode_visual_stage_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x50")
        self.assertEqual(decoded["version"], 1)
        self.assertTrue(decoded["completed_valid"])
        self.assertEqual(decoded["completed_bank"], 2)
        self.assertEqual(decoded["frame_generation"], 0x1234)
        self.assertEqual(decoded["did_signature"], "0x1111")
        self.assertEqual(decoded["xmap_mode_signature"], "0x2222")
        self.assertEqual(decoded["route_signature"], "0x3333")
        self.assertEqual(decoded["display_index_signature"], "0x4444")
        cyanctl.validate_pinned_visual_stage_status(
            {"bank": 2, "frame_generation": 0x1234}, decoded
        )
        with self.assertRaises(ValueError):
            cyanctl.validate_pinned_visual_stage_status(
                {"bank": 1, "frame_generation": 0x1234}, decoded
            )

    def test_resolver_status_decode_and_classification(self):
        value = (
            (0x52 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (1 << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
            | 0x45678
        )
        decoded = cyanctl.decode_resolver_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x52")
        self.assertEqual(decoded["version"], 1)
        self.assertTrue(decoded["completed_valid"])
        self.assertEqual(decoded["completed_bank"], 2)
        self.assertEqual(decoded["frame_generation"], 0x1234)
        self.assertTrue(decoded["resolver_valid"])
        self.assertTrue(decoded["cfg_stable"])
        self.assertEqual(decoded["rdmask"], "0xff")
        self.assertEqual(decoded["front_nonzero_winners"], 0x45678)
        self.assertEqual(decoded["classification"], "front-palette-active")

        black = value & ~0xFFFFF
        black |= 94 << 20
        decoded = cyanctl.decode_resolver_status(black.to_bytes(16, "big"))
        self.assertEqual(decoded["classification"], "did-xmap-palette-black")

        zero_mask = black & ~(0xFF << 80)
        decoded = cyanctl.decode_resolver_status(zero_mask.to_bytes(16, "big"))
        self.assertEqual(decoded["classification"], "zero-readmask")

    def test_resolver_status_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            cyanctl.decode_resolver_status(b"short")

    def test_resolver_detail_decode_and_correlation(self):
        frame_pixels = 1024 * 768
        cursor = 94
        front = frame_pixels - cursor
        resolver_value = (
            (0x52 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (1 << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
            | (cursor << 20)
        )
        detail_value = (
            (0x44 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (0x1234 << 93)
            | (1 << 92)
            | (front << 60)
            | (front << 40)
        )
        resolver = cyanctl.decode_resolver_status(
            resolver_value.to_bytes(16, "big")
        )
        detail = cyanctl.decode_resolver_detail_status(
            detail_value.to_bytes(16, "big")
        )
        self.assertEqual(detail["signature"], "0x44")
        self.assertEqual(detail["version"], 1)
        self.assertTrue(detail["completed_valid"])
        self.assertEqual(detail["completed_bank"], 2)
        self.assertEqual(detail["frame_generation"], 0x1234)
        self.assertEqual(detail["front_index_nonzero_count"], 0)
        self.assertEqual(detail["front_did0_winners"], front)
        self.assertEqual(detail["front_bank0_winners"], front)
        correlation = cyanctl.correlate_resolver_detail(resolver, detail)
        self.assertEqual(correlation["classification"], "front-index-zero")
        self.assertEqual(correlation["front_winners"], front)
        self.assertEqual(correlation["front_bank3_winners"], 0)
        self.assertTrue(correlation["front_all_did0"])
        self.assertEqual(correlation["front_single_bank"], 0)

        capture = {"bank": 2, "frame_generation": 0x1234}
        cyanctl.validate_pinned_resolver_status(capture, resolver)
        cyanctl.validate_pinned_resolver_detail_status(capture, detail)
        with self.assertRaisesRegex(ValueError, "does not match pinned capture"):
            cyanctl.validate_pinned_resolver_detail_status(
                {"bank": 1, "frame_generation": 0x1234}, detail
            )

    def test_resolver_detail_distinguishes_nonzero_index_black_palette(self):
        frame_pixels = 1024 * 768
        resolver_value = (
            (0x52 << 120)
            | (1 << 112)
            | (1 << 111)
            | (1 << 109)
            | (7 << 93)
            | (1 << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
        )
        detail_value = (
            (0x44 << 120)
            | (1 << 112)
            | (1 << 111)
            | (1 << 109)
            | (7 << 93)
            | (1 << 92)
            | (0xFFF << 80)
            | (frame_pixels << 20)
        )
        resolver = cyanctl.decode_resolver_status(
            resolver_value.to_bytes(16, "big")
        )
        detail = cyanctl.decode_resolver_detail_status(
            detail_value.to_bytes(16, "big")
        )
        self.assertTrue(detail["front_index_nonzero_count_saturated"])
        correlation = cyanctl.correlate_resolver_detail(resolver, detail)
        self.assertEqual(correlation["classification"], "front-palette-black")
        self.assertEqual(correlation["front_single_bank"], 1)

    def test_resolver_detail_rejects_wrong_length_and_mismatched_frame(self):
        with self.assertRaises(ValueError):
            cyanctl.decode_resolver_detail_status(b"short")
        resolver = {
            "signature": "0x52", "version": 1, "completed_valid": True,
            "completed_bank": 0, "frame_generation": 1,
            "resolver_valid": True,
        }
        detail = {
            "signature": "0x44", "version": 1, "completed_valid": True,
            "completed_bank": 0, "frame_generation": 2,
            "resolver_valid": True,
        }
        with self.assertRaisesRegex(ValueError, "does not match resolver frame"):
            cyanctl.correlate_resolver_detail(resolver, detail)

    def test_board_status_decode(self):
        payload = bytes.fromhex("48a5ce03 12345678 56f00000 000000a5")
        decoded = cyanctl.decode_board_status(payload)
        self.assertEqual(decoded["adv7511"]["signature"], "0x48")
        self.assertEqual(decoded["adv7511"]["nack_count"], 3)
        self.assertEqual(decoded["soc_status"], "0x12345678")
        self.assertEqual(decoded["scc_access_queued"], 0)
        self.assertEqual(decoded["leds"], "0xa5")

    def test_hdmi_link_status_decode(self):
        value = (
            (0x48 << 120)
            | (10 << 116)
            | (42 << 110)
            | (1 << 107)
            | (1 << 95)
            | (1 << 94)
            | (1 << 93)
            | (1 << 92)
            | (0x60 << 83)
            | (0x10 << 75)
            | (0x10 << 67)
            | (0x01 << 59)
            | (0x34 << 51)
            | (0x08 << 43)
            | (7 << 27)
        )
        decoded = cyanctl.decode_hdmi_link_status(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x48")
        self.assertTrue(decoded["config_done"])
        self.assertTrue(decoded["readback_valid"])
        self.assertTrue(decoded["hpd"])
        self.assertTrue(decoded["monitor_sense"])
        self.assertTrue(decoded["pll_locked"])
        self.assertFalse(decoded["power_down"])
        self.assertEqual(decoded["registers"]["0x16"], "0x34")
        self.assertEqual(decoded["readback_generation"], 7)

    def test_capture_info_and_png(self):
        value = (
            (0x43 << 120)
            | (1 << 112)
            | (16 << 96)
            | (2 << 80)
            | (2 << 64)
            | (2 << 62)
            | (1 << 61)
            | (1 << 60)
            | (9 << 44)
        )
        decoded = cyanctl.decode_capture_info(value.to_bytes(16, "big"))
        self.assertEqual(decoded["signature"], "0x43")
        self.assertEqual(decoded["width"], 16)
        self.assertEqual(decoded["height"], 2)
        self.assertEqual(decoded["words_per_frame"], 2)
        self.assertEqual(decoded["bank"], 2)
        self.assertTrue(decoded["pinned"])
        self.assertEqual(decoded["frame_generation"], 9)
        rgb = bytes(range(16 * 2 * 3))
        png = cyanctl.encode_png_rgb(16, 2, rgb)
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")

        signatures = {
            "signature": "0x53",
            "version": 1,
            "completed_valid": True,
            "completed_bank": 2,
            "frame_generation": 9,
        }
        cyanctl.validate_pinned_video_signatures(decoded, signatures)
        signatures["frame_generation"] = 10
        with self.assertRaisesRegex(ValueError, "does not match pinned capture"):
            cyanctl.validate_pinned_video_signatures(decoded, signatures)

    def test_capture_queries_both_resolver_pages_while_pinned(self):
        capture_info = (
            (0x43 << 120)
            | (1 << 112)
            | (16 << 96)
            | (1 << 80)
            | (1 << 64)
            | (2 << 62)
            | (1 << 61)
            | (1 << 60)
            | (9 << 44)
        ).to_bytes(16, "big")
        common = (1 << 111) | (2 << 109) | (9 << 93)
        video_signatures = (
            (0x53 << 120) | (1 << 112) | common
        ).to_bytes(16, "big")
        stage_signatures = (
            (0x46 << 120) | (1 << 112) | common
        ).to_bytes(16, "big")
        resolver = (
            (0x52 << 120)
            | (1 << 112)
            | common
            | (1 << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
            | 1
        ).to_bytes(16, "big")
        resolver_detail = (
            (0x44 << 120)
            | (1 << 112)
            | common
            | (1 << 92)
            | (1 << 80)
            | ((1024 * 768) << 60)
            | ((1024 * 768) << 40)
        ).to_bytes(16, "big")
        operations = []
        release_timeouts = 0
        chunks = iter((bytes(range(16)), bytes(range(16, 32)), bytes(range(32, 48))))

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            nonlocal release_timeouts
            operations.append((request.opcode, request.payload))
            if request.opcode == cyanctl.OP_CAPTURE_INFO:
                if request.payload == b"\x02" and release_timeouts == 0:
                    release_timeouts += 1
                    raise TimeoutError("dropped release reply")
                payload = capture_info
            elif request.opcode == cyanctl.OP_VIDEO_SIGNATURES:
                payload = video_signatures
            elif request.opcode == cyanctl.OP_VIDEO_STAGE_SIGNATURES:
                payload = stage_signatures
            elif request.opcode == cyanctl.OP_RESOLVER_STATUS:
                payload = resolver
            elif request.opcode == cyanctl.OP_RESOLVER_DETAIL_STATUS:
                payload = resolver_detail
            elif request.opcode == cyanctl.OP_CAPTURE_READ:
                payload = next(chunks)
            else:
                self.fail(f"unexpected capture opcode {request.opcode:#x}")
            return cyanctl.MgmtMessage(
                request.opcode,
                request.sequence,
                payload,
                cyanctl.FLAG_RESPONSE,
                0,
            )

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "frame.png"
            args = SimpleNamespace(
                interface="test0",
                timeout=1.0,
                read_retries=2,
                retry_delay=0.0,
                progress=False,
                rgb=str(output),
            )
            stdout = io.StringIO()
            with mock.patch.object(
                cyanctl, "read_interface_mac", return_value=b"\0" * 6
            ), mock.patch.object(
                cyanctl.socket, "socket", return_value=FakeSocket()
            ), mock.patch.object(
                cyanctl, "transact_on_socket", side_effect=transact
            ), mock.patch("sys.stdout", stdout):
                self.assertEqual(cyanctl.capture_rgb(args), 0)
            report = json.loads(stdout.getvalue())
            self.assertEqual(report["resolver"]["frame_generation"], 9)
            self.assertEqual(report["resolver_detail"]["frame_generation"], 9)
            self.assertTrue(output.is_file())

        self.assertEqual(
            operations,
            [
                (cyanctl.OP_CAPTURE_INFO, b"\x01"),
                (cyanctl.OP_VIDEO_SIGNATURES, b""),
                (cyanctl.OP_VIDEO_STAGE_SIGNATURES, b""),
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_RESOLVER_DETAIL_STATUS, b""),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x00"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x01"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x02"),
                (cyanctl.OP_CAPTURE_INFO, b"\x02"),
                (cyanctl.OP_CAPTURE_INFO, b"\x02"),
            ],
        )

    def test_capture_points_share_word_reads_and_extract_exact_pixels(self):
        capture_info = (
            (0x43 << 120)
            | (1 << 112)
            | (32 << 96)
            | (1 << 80)
            | (2 << 64)
            | (1 << 62)
            | (1 << 61)
            | (1 << 60)
            | (23 << 44)
        ).to_bytes(16, "big")
        common = (1 << 111) | (1 << 109) | (23 << 93)
        video_signatures = ((0x53 << 120) | (1 << 112) | common).to_bytes(
            16, "big"
        )
        stage_signatures = ((0x46 << 120) | (1 << 112) | common).to_bytes(
            16, "big"
        )
        resolver = (
            (0x52 << 120)
            | (1 << 112)
            | common
            | (1 << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
            | 1
        ).to_bytes(16, "big")
        resolver_detail = (
            (0x44 << 120)
            | (1 << 112)
            | common
            | (1 << 92)
            | (1 << 80)
            | (32 << 60)
            | (32 << 40)
        ).to_bytes(16, "big")
        word_data = {
            0: bytes(range(48)) + bytes(range(0xa0, 0xb0)),
            1: bytes(range(48, 96)) + bytes(range(0xb0, 0xc0)),
        }
        operations = []

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            operations.append((request.opcode, request.payload))
            if request.opcode == cyanctl.OP_CAPTURE_INFO:
                payload = capture_info
            elif request.opcode == cyanctl.OP_VIDEO_SIGNATURES:
                payload = video_signatures
            elif request.opcode == cyanctl.OP_VIDEO_STAGE_SIGNATURES:
                payload = stage_signatures
            elif request.opcode == cyanctl.OP_RESOLVER_STATUS:
                payload = resolver
            elif request.opcode == cyanctl.OP_RESOLVER_DETAIL_STATUS:
                payload = resolver_detail
            elif request.opcode == cyanctl.OP_CAPTURE_READ:
                word_index = int.from_bytes(request.payload[:2], "big")
                chunk_index = request.payload[2]
                start = chunk_index * 16
                payload = word_data[word_index][start : start + 16]
            else:
                self.fail(f"unexpected capture opcode {request.opcode:#x}")
            return cyanctl.MgmtMessage(
                request.opcode,
                request.sequence,
                payload,
                cyanctl.FLAG_RESPONSE,
                0,
            )

        args = SimpleNamespace(
            interface="test0",
            timeout=1.0,
            read_timeout=0.1,
            read_retries=1,
            retry_delay=0.0,
            progress=False,
            rgb=None,
            point=[(1, 0), (15, 0), (16, 0)],
        )
        stdout = io.StringIO()
        with mock.patch.object(
            cyanctl, "read_interface_mac", return_value=b"\0" * 6
        ), mock.patch.object(
            cyanctl.socket, "socket", return_value=FakeSocket()
        ), mock.patch.object(
            cyanctl, "transact_on_socket", side_effect=transact
        ), mock.patch("sys.stdout", stdout):
            self.assertEqual(cyanctl.capture_rgb(args), 0)

        report = json.loads(stdout.getvalue())
        self.assertNotIn("output", report)
        self.assertEqual(
            [sample["pixel_rgb"] for sample in report["points"]],
            ["030405", "2d2e2f", "303132"],
        )
        self.assertEqual(
            [sample["word_index"] for sample in report["points"]],
            [0, 0, 1],
        )
        self.assertEqual(
            [sample["raw_index"] for sample in report["points"]],
            ["0xa1", "0xaf", "0xb0"],
        )
        self.assertEqual(
            [operation for operation in operations
             if operation[0] == cyanctl.OP_CAPTURE_READ],
            [
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x00"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x01"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x02"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x00\x03"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x01\x00"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x01\x01"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x01\x02"),
                (cyanctl.OP_CAPTURE_READ, b"\x00\x01\x03"),
            ],
        )

    def test_point_word_read_falls_back_to_legacy_rgb_only_endpoint(self):
        chunks = []

        def read_chunk(_word_index, chunk_index):
            chunks.append(chunk_index)
            if chunk_index == 3:
                raise RuntimeError("capture-read returned status 2")
            return bytes(range(chunk_index * 16, (chunk_index + 1) * 16))

        word = cyanctl.read_capture_word_chunks(
            read_chunk, 7, include_raw_indices=True
        )
        samples = cyanctl.sample_rgb_points([(1, 0)], 32, 1, lambda _index: word)

        self.assertEqual(chunks, [0, 1, 2, 3])
        self.assertEqual(len(word), 48)
        self.assertEqual(samples[0]["pixel_rgb"], "030405")
        self.assertIsNone(samples[0]["raw_index"])
        self.assertIsNone(samples[0]["word_raw_index_sha256"])

    def test_presentation_watch_pins_coherent_new_frame_without_rgb_reads(self):
        def resolver_payload(generation, bank=2):
            value = (
                (0x52 << 120)
                | (1 << 112)
                | (1 << 111)
                | (bank << 109)
                | (generation << 93)
                | (1 << 92)
                | (1 << 91)
                | (1 << 90)
                | (0xFF << 80)
                | 1
            )
            return value.to_bytes(16, "big")

        def capture_payload(generation, bank=2):
            value = (
                (0x43 << 120)
                | (1 << 112)
                | (1024 << 96)
                | (768 << 80)
                | (49152 << 64)
                | (bank << 62)
                | (1 << 61)
                | (1 << 60)
                | (generation << 44)
            )
            return value.to_bytes(16, "big")

        def signatures_payload(generation, bank=2):
            common = (1 << 111) | (bank << 109) | (generation << 93)
            value = (
                (0x53 << 120)
                | (1 << 112)
                | common
                | (0x1234 << 77)
                | (0x5678 << 61)
                | (0x1234 << 45)
                | (0x5678 << 29)
            )
            return value.to_bytes(16, "big")

        def stage_payload(generation, bank=2):
            common = (1 << 111) | (bank << 109) | (generation << 93)
            value = (
                (0x46 << 120)
                | (1 << 112)
                | common
                | (0x9ABC << 77)
                | (0xDEF0 << 61)
                | (0x9ABC << 45)
                | (0xDEF0 << 29)
                | (1 << 20)
            )
            return value.to_bytes(16, "big")

        def detail_payload(generation, bank=2):
            value = (
                (0x44 << 120)
                | (1 << 112)
                | (1 << 111)
                | (bank << 109)
                | (generation << 93)
                | (1 << 92)
                | (1 << 80)
            )
            return value.to_bytes(16, "big")

        def visual_stage_payload(generation, bank=2):
            value = (
                (0x50 << 120)
                | (1 << 112)
                | (1 << 111)
                | (bank << 109)
                | (generation << 93)
                | (0x1111 << 77)
                | (0x2222 << 61)
                | (0x3333 << 45)
                | (0x4444 << 29)
            )
            return value.to_bytes(16, "big")

        operations = []
        resolver_responses = iter((
            resolver_payload(9),
            resolver_payload(10),
            resolver_payload(11),
        ))

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            operations.append((request.opcode, request.payload))
            if request.opcode == cyanctl.OP_RESOLVER_STATUS:
                payload = next(resolver_responses)
            elif request.opcode == cyanctl.OP_CAPTURE_INFO:
                payload = capture_payload(11)
            elif request.opcode == cyanctl.OP_VIDEO_SIGNATURES:
                payload = signatures_payload(11)
            elif request.opcode == cyanctl.OP_VIDEO_STAGE_SIGNATURES:
                payload = stage_payload(11)
            elif request.opcode == cyanctl.OP_RESOLVER_DETAIL_STATUS:
                payload = detail_payload(11)
            elif request.opcode == cyanctl.OP_VISUAL_STAGE_STATUS:
                payload = visual_stage_payload(11)
            else:
                self.fail(f"unsafe presentation-watch opcode {request.opcode:#x}")
            return cyanctl.MgmtMessage(
                request.opcode, request.sequence, payload,
                cyanctl.FLAG_RESPONSE, 0,
            )

        args = SimpleNamespace(
            interface="test0", timeout=1.0, duration=0.9, interval=0.0,
        )
        # next_sequence() also consumes one monotonic tick before the watch
        # establishes its duration deadline.
        ticks = iter(range(20))
        stdout = io.StringIO()
        with mock.patch.object(
            cyanctl, "read_interface_mac", return_value=b"\0" * 6
        ), mock.patch.object(
            cyanctl.socket, "socket", return_value=FakeSocket()
        ), mock.patch.object(
            cyanctl, "transact_on_socket", side_effect=transact
        ), mock.patch.object(
            cyanctl.time, "monotonic_ns",
            side_effect=lambda: next(ticks) * 100_000_000,
        ), mock.patch("sys.stdout", stdout):
            self.assertEqual(cyanctl.watch_presentation_frames(args), 0)

        report = json.loads(stdout.getvalue())
        self.assertEqual(report["type"], "presentation-frame")
        self.assertEqual(report["polled_frame_generation"], 10)
        self.assertEqual(report["capture_info"]["frame_generation"], 11)
        self.assertTrue(report["pin_raced_to_newer_generation"])
        self.assertTrue(report["stage_correlation"]["source_mig_low_match"])
        self.assertTrue(report["stage_correlation"]["source_mig_high_match"])
        self.assertTrue(report["stage_correlation"]["raw_fetch_low_match"])
        self.assertTrue(report["stage_correlation"]["raw_fetch_high_match"])
        self.assertEqual(
            operations,
            [
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_CAPTURE_INFO, b"\x01"),
                (cyanctl.OP_VIDEO_SIGNATURES, b""),
                (cyanctl.OP_VIDEO_STAGE_SIGNATURES, b""),
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_RESOLVER_DETAIL_STATUS, b""),
                (cyanctl.OP_VISUAL_STAGE_STATUS, b""),
                (cyanctl.OP_CAPTURE_INFO, b"\x02"),
            ],
        )
        self.assertNotIn(
            cyanctl.OP_CAPTURE_READ,
            [opcode for opcode, _payload in operations],
        )

    def test_presentation_watch_releases_pin_after_validation_failure(self):
        def resolver_payload(generation):
            return (
                (0x52 << 120)
                | (1 << 112)
                | (1 << 111)
                | (2 << 109)
                | (generation << 93)
            ).to_bytes(16, "big")

        capture_info = (
            (0x43 << 120)
            | (1 << 112)
            | (1024 << 96)
            | (768 << 80)
            | (49152 << 64)
            | (2 << 62)
            | (1 << 61)
            | (1 << 60)
            | (10 << 44)
        ).to_bytes(16, "big")
        mismatched_signatures = (
            (0x53 << 120)
            | (1 << 112)
            | (1 << 111)
            | (2 << 109)
            | (11 << 93)
        ).to_bytes(16, "big")
        resolver_responses = iter((resolver_payload(9), resolver_payload(10)))
        operations = []

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            operations.append((request.opcode, request.payload))
            if request.opcode == cyanctl.OP_RESOLVER_STATUS:
                payload = next(resolver_responses)
            elif request.opcode == cyanctl.OP_CAPTURE_INFO:
                payload = capture_info
            elif request.opcode == cyanctl.OP_VIDEO_SIGNATURES:
                payload = mismatched_signatures
            else:
                self.fail(f"unexpected opcode {request.opcode:#x}")
            return cyanctl.MgmtMessage(
                request.opcode, request.sequence, payload,
                cyanctl.FLAG_RESPONSE, 0,
            )

        args = SimpleNamespace(
            interface="test0", timeout=1.0, duration=1.0, interval=0.0,
        )
        ticks = iter(range(20))
        stderr = io.StringIO()
        with mock.patch.object(
            cyanctl, "read_interface_mac", return_value=b"\0" * 6
        ), mock.patch.object(
            cyanctl.socket, "socket", return_value=FakeSocket()
        ), mock.patch.object(
            cyanctl, "transact_on_socket", side_effect=transact
        ), mock.patch.object(
            cyanctl.time, "monotonic_ns",
            side_effect=lambda: next(ticks) * 100_000_000,
        ), mock.patch("sys.stderr", stderr):
            self.assertEqual(cyanctl.watch_presentation_frames(args), 1)

        self.assertIn("does not match pinned capture", stderr.getvalue())
        self.assertEqual(
            operations[-1], (cyanctl.OP_CAPTURE_INFO, b"\x02")
        )
        self.assertNotIn(
            cyanctl.OP_CAPTURE_READ,
            [opcode for opcode, _payload in operations],
        )

    def test_presentation_watch_releases_after_lost_pin_reply(self):
        def resolver_payload(generation):
            return (
                (0x52 << 120)
                | (1 << 112)
                | (1 << 111)
                | (2 << 109)
                | (generation << 93)
            ).to_bytes(16, "big")

        resolver_responses = iter((resolver_payload(9), resolver_payload(10)))
        operations = []

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            operations.append((request.opcode, request.payload))
            if request.opcode == cyanctl.OP_RESOLVER_STATUS:
                payload = next(resolver_responses)
            elif request.payload == b"\x01":
                raise TimeoutError("lost pin response")
            elif request.payload == b"\x02":
                payload = b""
            else:
                self.fail(f"unexpected opcode {request.opcode:#x}")
            return cyanctl.MgmtMessage(
                request.opcode, request.sequence, payload,
                cyanctl.FLAG_RESPONSE, 0,
            )

        args = SimpleNamespace(
            interface="test0", timeout=1.0, duration=1.0, interval=0.0,
        )
        ticks = iter(range(20))
        stderr = io.StringIO()
        with mock.patch.object(
            cyanctl, "read_interface_mac", return_value=b"\0" * 6
        ), mock.patch.object(
            cyanctl.socket, "socket", return_value=FakeSocket()
        ), mock.patch.object(
            cyanctl, "transact_on_socket", side_effect=transact
        ), mock.patch.object(
            cyanctl.time, "monotonic_ns",
            side_effect=lambda: next(ticks) * 100_000_000,
        ), mock.patch("sys.stderr", stderr):
            self.assertEqual(cyanctl.watch_presentation_frames(args), 1)

        self.assertIn("lost pin response", stderr.getvalue())
        self.assertEqual(
            operations,
            [
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_RESOLVER_STATUS, b""),
                (cyanctl.OP_CAPTURE_INFO, b"\x01"),
                (cyanctl.OP_CAPTURE_INFO, b"\x02"),
            ],
        )

    def test_scc_access_decode(self):
        decoded = cyanctl.decode_scc_access_events(bytes.fromhex("e2bf1065"))
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0]["operation"], "write")
        self.assertEqual(decoded[0]["channel"], "keyboard")
        self.assertEqual(decoded[0]["port"], "control")
        self.assertTrue(decoded[0]["rx_ready"])
        self.assertEqual(decoded[0]["address"], 0xB)
        self.assertEqual(decoded[0]["register"], 0xF)
        self.assertEqual(decoded[0]["data"], "0x10")
        self.assertEqual(decoded[0]["rr0"], "0x65")

    def test_status_query_opcodes_are_distinct_and_named(self):
        expected = {
            cyanctl.OP_BOARD_STATUS: "board-status",
            cyanctl.OP_HDMI_LINK_STATUS: "hdmi-status",
            cyanctl.OP_VIDEO_STATUS: "video-status",
            cyanctl.OP_VIDEO_SIGNATURES: "video-signatures",
            cyanctl.OP_RESOLVER_STATUS: "resolver-status",
            cyanctl.OP_RESOLVER_DETAIL_STATUS: "resolver-detail-status",
            cyanctl.OP_VISUAL_STAGE_STATUS: "visual-stage-status",
            cyanctl.OP_SCAN_SERVICE_STATUS: "scan-service-status",
            cyanctl.OP_SCAN_LATENCY_STATUS: "scan-latency-status",
        }
        self.assertEqual(
            {opcode: cyanctl.OPCODE_NAMES[opcode] for opcode in expected},
            expected,
        )

    def test_status_query_parsers_accept_interface(self):
        parser = cyanctl.build_parser()
        for command in (
            "board-status", "hdmi-status", "video-status", "video-signatures",
            "resolver-status", "resolver-detail-status",
            "visual-stage-status", "scan-service-status",
            "scan-latency-status",
        ):
            args = parser.parse_args((command, "--interface", "test0"))
            self.assertEqual(args.command, command)
            self.assertEqual(args.interface, "test0")

    def test_fan_parser_accepts_only_supported_duties(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args(("fan", "25", "--interface", "test0"))
        self.assertEqual(args.duty, 25)
        self.assertEqual(args.interface, "test0")

    def test_trace_opcodes_are_distinct_and_named(self):
        expected = {
            cyanctl.OP_KBD_TX_EVENTS: "kbd-tx-events",
            cyanctl.OP_MOUSE_TX_EVENTS: "mouse-tx-events",
            cyanctl.OP_KBD_RX_EVENTS: "kbd-rx-events",
            cyanctl.OP_MOUSE_RX_EVENTS: "mouse-rx-events",
        }
        self.assertEqual(
            {opcode: cyanctl.OPCODE_NAMES[opcode] for opcode in expected},
            expected,
        )

    def test_trace_parser_accepts_all_streams(self):
        parser = cyanctl.build_parser()
        for stream in ("keyboard-tx", "mouse-tx", "keyboard-rx", "mouse-rx"):
            args = parser.parse_args(("scc-trace", stream))
            self.assertEqual(args.command, "scc-trace")
            self.assertEqual(args.stream, stream)

    def test_scc_access_trace_control_parser(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args(("scc-access-trace", "--arm-input"))
        self.assertTrue(args.arm_input)
        self.assertFalse(args.clear)

    def test_lg1_config_trace_parser(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args((
            "lg1-config-trace", "arm-writes", "--interface", "test0",
        ))
        self.assertEqual(args.lg1_trace_action, "arm-writes")
        self.assertEqual(args.interface, "test0")
        args = parser.parse_args((
            "lg1-config-trace", "arm-rex", "--interface", "test0",
        ))
        self.assertEqual(args.lg1_trace_action, "arm-rex")
        self.assertEqual(args.interface, "test0")
        args = parser.parse_args((
            "lg1-config-trace", "mark", "twilight-active",
            "--interface", "test0",
        ))
        self.assertEqual(args.lg1_trace_action, "mark")
        self.assertEqual(args.marker, "twilight-active")
        self.assertEqual(args.interface, "test0")
        args = parser.parse_args((
            "lg1-config-trace", "dump", "--output", "trace.log",
            "--freeze",
        ))
        self.assertEqual(args.output, "trace.log")
        self.assertTrue(args.freeze)
        self.assertEqual(args.read_retries, 20)
        self.assertEqual(args.retry_delay, 0.01)

    def test_lg1_trace_dump_freezes_only_a_live_trace(self):
        frozen_status = bytes.fromhex(
            "4c47010c400e00010000000100000000"
        )
        live_status = bytes.fromhex(
            "4c47010c800e00010000000100000000"
        )
        record = ((1 << 94) | (0xFF << 86)).to_bytes(12, "big")

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        for initially_live in (False, True):
            with self.subTest(initially_live=initially_live):
                operations = []
                status_responses = iter(
                    (live_status, frozen_status) if initially_live else
                    (frozen_status,)
                )

                def transact(_sock, _source_mac, request, _timeout):
                    operations.append((request.opcode, request.payload))
                    if request.opcode == cyanctl.OP_LG1_TRACE_STATUS:
                        payload = next(status_responses)
                    elif request.opcode == cyanctl.OP_LG1_TRACE_FREEZE:
                        payload = b""
                    elif request.opcode == cyanctl.OP_LG1_TRACE_READ:
                        payload = record
                    else:
                        self.fail(
                            f"unexpected trace-dump opcode {request.opcode:#x}"
                        )
                    return cyanctl.MgmtMessage(
                        request.opcode,
                        request.sequence,
                        payload,
                        cyanctl.FLAG_RESPONSE,
                        0,
                    )

                with tempfile.TemporaryDirectory() as temporary:
                    args = SimpleNamespace(
                        interface="test0",
                        timeout=1.0,
                        read_retries=1,
                        retry_delay=0.0,
                        progress=False,
                        freeze=True,
                        output=str(Path(temporary) / "trace.log"),
                    )
                    with mock.patch.object(
                        cyanctl, "read_interface_mac", return_value=b"\0" * 6
                    ), mock.patch.object(
                        cyanctl.socket, "socket", return_value=FakeSocket()
                    ), mock.patch.object(
                        cyanctl, "transact_on_socket", side_effect=transact
                    ), mock.patch("sys.stdout", io.StringIO()):
                        self.assertEqual(cyanctl.dump_lg1_config_trace(args), 0)

                expected = [(cyanctl.OP_LG1_TRACE_STATUS, b"")]
                if initially_live:
                    expected.extend((
                        (cyanctl.OP_LG1_TRACE_FREEZE, b""),
                        (cyanctl.OP_LG1_TRACE_STATUS, b""),
                    ))
                expected.append((cyanctl.OP_LG1_TRACE_READ, b"\x00\x00"))
                self.assertEqual(operations, expected)

    def test_capture_parser_requires_output(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args(("capture", "--rgb", "frame.png"))
        self.assertEqual(args.rgb, "frame.png")
        self.assertEqual(args.read_retries, 20)
        self.assertIsNone(args.read_timeout)
        self.assertEqual(args.retry_delay, 0.01)

    def test_capture_parser_accepts_retry_controls(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args((
            "capture", "--rgb", "frame.png",
            "--read-retries", "32", "--read-timeout", "0.02",
            "--retry-delay", "0.05",
        ))
        self.assertEqual(args.read_retries, 32)
        self.assertEqual(args.read_timeout, 0.02)
        self.assertEqual(args.retry_delay, 0.05)

    def test_capture_parser_accepts_repeated_points(self):
        parser = cyanctl.build_parser()
        args = parser.parse_args((
            "capture", "--point", "608,220", "--point", "736,260",
        ))
        self.assertIsNone(args.rgb)
        self.assertEqual(args.point, [(608, 220), (736, 260)])

    def test_capture_parser_rejects_invalid_retry_controls(self):
        parser = cyanctl.build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args((
                "capture", "--rgb", "frame.png", "--read-retries", "0",
            ))
        with self.assertRaises(SystemExit):
            parser.parse_args((
                "capture", "--rgb", "frame.png", "--retry-delay", "-0.1",
            ))
        with self.assertRaises(SystemExit):
            parser.parse_args((
                "capture", "--rgb", "frame.png", "--read-timeout", "0",
            ))
        with self.assertRaises(SystemExit):
            parser.parse_args(("capture", "--point", "-1,2"))
        with self.assertRaises(SystemExit):
            parser.parse_args(("capture", "--point", "not-a-point"))


class ResolverWatchTests(unittest.TestCase):
    @staticmethod
    def resolver_payload(
        generation: int,
        *,
        signature: int = 0x52,
        version: int = 1,
        completed_valid: bool = True,
        resolver_valid: bool = True,
    ) -> bytes:
        value = (
            (signature << 120)
            | (version << 112)
            | (int(completed_valid) << 111)
            | (2 << 109)
            | (generation << 93)
            | (int(resolver_valid) << 92)
            | (1 << 91)
            | (1 << 90)
            | (0xFF << 80)
            | (94 << 20)
            | 562285
        )
        return value.to_bytes(16, "big")

    def run_watch(self, responses, *, all_samples=False,
                  continue_timeouts=False):
        response_iterator = iter(responses)
        self.seen_watch_sequences = []

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def bind(self, _address):
                pass

        def transact(_sock, _source_mac, request, _timeout):
            self.seen_watch_sequences.append(request.sequence)
            response = next(response_iterator)
            if isinstance(response, BaseException):
                raise response
            payload, status = response
            return cyanctl.MgmtMessage(
                request.opcode,
                request.sequence,
                payload,
                cyanctl.FLAG_RESPONSE,
                status,
            )

        args = SimpleNamespace(
            interface="test0",
            timeout=1.0,
            duration=1.0,
            interval=0.0,
            all_samples=all_samples,
            continue_timeouts=continue_timeouts,
        )
        # next_sequence(), start, then exactly three complete queries before
        # the fourth loop-condition timestamp reaches the deadline.
        ticks = iter(range(20))
        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch.object(
            cyanctl, "read_interface_mac", return_value=b"\0" * 6
        ), mock.patch.object(
            cyanctl.socket, "socket", return_value=FakeSocket()
        ), mock.patch.object(
            cyanctl, "transact_on_socket", side_effect=transact
        ), mock.patch.object(
            cyanctl.time, "monotonic_ns",
            side_effect=lambda: next(ticks) * 100_000_000,
        ), mock.patch("sys.stdout", stdout), mock.patch("sys.stderr", stderr):
            result = cyanctl.watch_resolver_frames(args)
        records = [json.loads(line) for line in stdout.getvalue().splitlines()]
        return result, records, stderr.getvalue()

    def test_generation_transition_classification(self):
        self.assertEqual(
            cyanctl.classify_resolver_generation_transition(7, 7),
            {
                "classification": "duplicate",
                "step": 0,
                "missed_completed_frames": 0,
            },
        )
        self.assertEqual(
            cyanctl.classify_resolver_generation_transition(0xFFFF, 0)[
                "classification"
            ],
            "contiguous",
        )
        missed = cyanctl.classify_resolver_generation_transition(10, 13)
        self.assertEqual(missed["classification"], "missed")
        self.assertEqual(missed["missed_completed_frames"], 2)
        self.assertEqual(
            cyanctl.classify_resolver_generation_transition(100, 99)[
                "classification"
            ],
            "backward-or-reset",
        )

    def test_descriptor_validation(self):
        cyanctl.validate_resolver_watch_status(cyanctl.decode_resolver_status(
            self.resolver_payload(1)
        ))
        invalid = (
            self.resolver_payload(1, signature=0x51),
            self.resolver_payload(1, version=2),
            self.resolver_payload(1, completed_valid=False),
            self.resolver_payload(1, resolver_valid=False),
        )
        for payload in invalid:
            with self.subTest(payload=payload.hex()), self.assertRaises(ValueError):
                cyanctl.validate_resolver_watch_status(
                    cyanctl.decode_resolver_status(payload)
                )

    def test_contiguous_wrap_and_duplicate_emit_complete_summary(self):
        result, records, stderr = self.run_watch((
            (self.resolver_payload(0xFFFF), 0),
            (self.resolver_payload(0), 0),
            (self.resolver_payload(0), 0),
        ))
        self.assertEqual(result, 0)
        self.assertEqual(stderr, "")
        self.assertEqual(
            [record["type"] for record in records],
            ["resolver-frame", "resolver-frame", "resolver-watch-summary"],
        )
        self.assertEqual(records[0]["generation_transition"], "baseline")
        self.assertEqual(records[1]["generation_transition"], "contiguous")
        self.assertEqual(records[1]["generation_step"], 1)
        summary = records[-1]
        self.assertTrue(summary["complete"])
        self.assertEqual(summary["first_generation"], 0xFFFF)
        self.assertEqual(summary["last_generation"], 0)
        self.assertEqual(summary["queries"], 3)
        self.assertEqual(summary["valid_queries"], 3)
        self.assertEqual(summary["duplicate_queries"], 1)
        self.assertEqual(summary["generation_gaps"], 0)

    def test_missed_generation_emits_gap_and_incomplete_summary(self):
        result, records, stderr = self.run_watch((
            (self.resolver_payload(10), 0),
            (self.resolver_payload(12), 0),
        ))
        self.assertEqual(result, 1)
        self.assertEqual(
            [record["type"] for record in records],
            [
                "resolver-frame",
                "resolver-generation-gap",
                "resolver-watch-summary",
            ],
        )
        gap = records[1]
        self.assertEqual(gap["classification"], "missed")
        self.assertEqual(gap["generation_step"], 2)
        self.assertEqual(gap["missed_completed_frames"], 1)
        summary = records[-1]
        self.assertFalse(summary["complete"])
        self.assertEqual(summary["failure_class"], "missed")
        self.assertEqual(summary["generation_gaps"], 1)
        self.assertEqual(summary["missed_completed_frames"], 1)
        self.assertIn("continuity failure", stderr)

    def test_backward_or_reset_is_an_explicit_failure(self):
        result, records, _stderr = self.run_watch((
            (self.resolver_payload(100), 0),
            (self.resolver_payload(99), 0),
        ))
        self.assertEqual(result, 1)
        self.assertEqual(records[1]["classification"], "backward-or-reset")
        summary = records[-1]
        self.assertEqual(summary["failure_class"], "backward-or-reset")
        self.assertEqual(summary["backward_or_reset_transitions"], 1)

    def test_invalid_descriptor_timeout_and_endpoint_error_fail_with_summary(self):
        cases = (
            (
                ((self.resolver_payload(1, resolver_valid=False), 0),),
                "invalid-descriptor",
            ),
            ((TimeoutError("timed out"),), "timeout"),
            (((self.resolver_payload(1), 3),), "endpoint-status"),
        )
        for responses, expected_failure in cases:
            with self.subTest(expected_failure=expected_failure):
                result, records, _stderr = self.run_watch(responses)
                self.assertEqual(result, 1)
                self.assertEqual(records[-1]["type"], "resolver-watch-summary")
                self.assertFalse(records[-1]["complete"])
                self.assertEqual(
                    records[-1]["failure_class"], expected_failure
                )

    def test_continue_timeouts_retains_contiguous_frames_but_fails_closed(self):
        result, records, stderr = self.run_watch((
            (self.resolver_payload(10), 0),
            cyanctl.socket.timeout("timed out"),
            (self.resolver_payload(11), 0),
        ), continue_timeouts=True)
        self.assertEqual(result, 1)
        self.assertEqual(
            [record["type"] for record in records],
            [
                "resolver-frame",
                "resolver-query-timeout",
                "resolver-frame",
                "resolver-watch-summary",
            ],
        )
        self.assertEqual(records[2]["generation_transition"], "contiguous")
        timeout = records[1]
        self.assertEqual(timeout["query_index"], 1)
        self.assertEqual(timeout["sequence"], self.seen_watch_sequences[1])
        self.assertEqual(
            self.seen_watch_sequences,
            [
                self.seen_watch_sequences[0],
                (self.seen_watch_sequences[0] + 1) & 0xFFFFFFFF,
                (self.seen_watch_sequences[0] + 2) & 0xFFFFFFFF,
            ],
        )
        summary = records[-1]
        self.assertFalse(summary["complete"])
        self.assertTrue(summary["duration_reached"])
        self.assertEqual(summary["queries"], 3)
        self.assertEqual(summary["valid_queries"], 2)
        self.assertEqual(summary["timeout_queries"], 1)
        self.assertEqual(summary["generation_gaps"], 0)
        self.assertEqual(summary["first_generation"], 10)
        self.assertEqual(summary["last_generation"], 11)
        self.assertIn("1 management request(s) timed out", stderr)


class DiscoveryTests(unittest.TestCase):
    def test_cp2105_roles_are_stable(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            usb = root / "usb"
            driver = root / "cp210x"
            driver.mkdir()
            for number in (0, 1):
                interface = usb / f"1-1:1.{number}"
                device = interface / f"ttyUSB{number}"
                device.mkdir(parents=True)
                (interface / "bInterfaceNumber").write_text(f"0{number}\n")
                (device / "driver").symlink_to(driver, target_is_directory=True)
                class_entry = root / f"ttyUSB{number}"
                class_entry.mkdir()
                (class_entry / "device").symlink_to(device, target_is_directory=True)

            ports = cyanctl.discover_cp2105_ports(root)
            self.assertEqual(
                [(port["interface"], port["role"]) for port in ports],
                [("00", "system-controller"), ("01", "sgi-scc-b")],
            )


if __name__ == "__main__":
    unittest.main()
