# CYAN management protocol

CYAN exposes a versioned management endpoint over a reserved raw-Ethernet
frame. The endpoint is independent of guest Ethernet/IP and remains available
while the emulated machine is not running. `tools/cyanctl.py` is the reference
host codec and command-line implementation.

## Transport

- Destination MAC: `02:43:59:41:4e:01`
- EtherType: `0x88B5`
- Byte order: network/big-endian
- Minimum Ethernet payload: 46 bytes; padding after the declared message is
  ignored
- Host requests use the host interface MAC as source. Replies reverse the
  source and destination addresses.

The management message header is 20 bytes:

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII `CYAN` |
| 4 | 1 | version | Protocol version, currently `1` |
| 5 | 1 | opcode | Operation identifier |
| 6 | 1 | flags | Bit 0 marks a response |
| 7 | 1 | status | Zero on success; nonzero values reject the request |
| 8 | 4 | sequence | Host-selected request identifier |
| 12 | 2 | payload length | Bytes following the header, excluding padding |
| 14 | 2 | reserved | Must be zero |
| 16 | 4 | payload CRC-32 | Standard CRC-32 of the declared payload |

Replies must match the request's opcode and sequence. A client ignores frames
with the wrong EtherType, destination, source, magic, version, checksum,
opcode, sequence, or response flag. Stateful implementations must make a
repeated sequence idempotent so a lost reply cannot duplicate an operation.

## Operations

The current public codec defines these operation numbers:

| Opcode | Name | Class |
|---:|---|---|
| `0x01` | `STATUS` | Observation |
| `0x02` | `RESET` | Reserved mutating operation; not exposed by the CLI |
| `0x03` | `FAN` | Mutating |
| `0x04` | `BOARD_STATUS` | Observation |
| `0x05` | `HDMI_LINK_STATUS` | Observation |
| `0x10` | `KBD_BYTES` | Input injection |
| `0x11` | `MOUSE_BYTES` | Input injection |
| `0x20`–`0x23` | SCC byte-event streams | Destructive trace drain |
| `0x24` | `VIDEO_STATUS` | Observation |
| `0x25` | `SCC_ACCESS_EVENTS` | Destructive trace drain |
| `0x26` | `SCC_TRACE_CONTROL` | Trace mutation |
| `0x27` | `VIDEO_SIGNATURES` | Observation |
| `0x28` | `VIDEO_STAGE_SIGNATURES` | Observation |
| `0x29` | `RESOLVER_STATUS` | Observation |
| `0x2A` | `RESOLVER_DETAIL_STATUS` | Observation |
| `0x2B`–`0x2F` | LG1 configuration trace | Trace mutation/status/read |
| `0x30` | `CAPTURE_INFO` | Capture pin/query/release |
| `0x31` | `CAPTURE_READ` | Read pinned RGB data |
| `0x32` | `GFX_RETRACE_STATUS` | Observation |
| `0x33` | `VISUAL_STAGE_STATUS` | Observation |
| `0x34` | `SCAN_SERVICE_STATUS` | Observation |
| `0x35` | `SCAN_LATENCY_STATUS` | Observation |

Payload layouts are decoded by named functions in `tools/cyanctl.py`. Decoders
require the exact payload length and return structured JSON-compatible fields;
they do not guess across protocol revisions.

## Passive observation

`network-snoop` opens an `AF_PACKET` raw socket filtered to EtherType `0x88B5`,
receives frames, validates the complete Ethernet/message envelope, and emits
one JSON object per valid request or response. It never calls `send` and does
not issue a status request:

```sh
sudo python3 tools/cyanctl.py network-snoop \
  --interface IFACE --count 10 --timeout 5
```

Each record contains source and destination MACs, numeric and symbolic opcode,
response state, flags, status, sequence, and the exact payload as hexadecimal.

The `presentation-watch` path is also observation-oriented but actively sends
read requests. It has a literal allowlist of status and capture operations and
cannot issue reset, fan, input, SCC drain, or trace-control commands.

## Security and safety

The protocol has integrity checks but no authentication, confidentiality, or
replay protection beyond request sequencing. Use it only on a trusted,
preferably point-to-point Layer-2 network. Raw sockets normally require root or
the Linux `CAP_NET_RAW` capability.

Commands are not equally passive. Status queries and `network-snoop` are
observations; fan selection, input injection, capture pinning, trace drains,
and trace controls change endpoint or machine-visible state. Automation should
use an explicit operation allowlist appropriate to its task.

## Development contract

Changes to the wire format require a protocol-version decision, unit tests for
round-trip and rejection behavior, and matching RTL endpoint changes. New
observers must remain fail-closed on malformed or mismatched frames. A command
must not be described as passive merely because it does not alter guest RAM;
draining a trace or pinning a capture is stateful.
