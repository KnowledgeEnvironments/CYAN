#!/usr/bin/env python3
"""Convert a big-endian 32-bit binary image to 32-bit memory init words."""

from __future__ import annotations

import argparse
from pathlib import Path


def write_words(output: Path, words: list[int], output_format: str) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii") as f:
        if output_format == "mif":
            depth = len(words)
            addr_digits = max(1, ((depth - 1).bit_length() + 3) >> 2)
            f.write("WIDTH=32;\n")
            f.write(f"DEPTH={depth};\n\n")
            f.write("ADDRESS_RADIX=HEX;\n")
            f.write("DATA_RADIX=HEX;\n\n")
            f.write("CONTENT BEGIN\n")
            for addr, word in enumerate(words):
                f.write(f"    {addr:0{addr_digits}x} : {word:08x};\n")
            f.write("END;\n")
        else:
            for word in words:
                f.write(f"{word:08x}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--depth", type=int, default=None)
    parser.add_argument(
        "--format",
        choices=("memh", "mif"),
        default="memh",
        help="output format: Verilog readmemh words or Quartus MIF",
    )
    parser.add_argument(
        "--bank-count",
        type=int,
        default=1,
        help="split output into N contiguous banks; output path is a format string",
    )
    args = parser.parse_args()

    data = args.input.read_bytes()
    if len(data) % 4:
        data += bytes(4 - (len(data) % 4))

    words = [int.from_bytes(data[i : i + 4], "big") for i in range(0, len(data), 4)]
    if args.depth is not None:
        if len(words) > args.depth:
            raise SystemExit(f"{args.input} has {len(words)} words, depth is {args.depth}")
        words.extend([0] * (args.depth - len(words)))

    if args.bank_count < 1:
        raise SystemExit("--bank-count must be at least 1")
    if args.bank_count == 1:
        write_words(args.output, words, args.format)
    else:
        if len(words) % args.bank_count:
            raise SystemExit(f"{len(words)} words does not divide into {args.bank_count} banks")
        bank_depth = len(words) // args.bank_count
        output_pattern = str(args.output)
        for bank in range(args.bank_count):
            output = Path(output_pattern.format(bank=bank, bank_hex=f"{bank:02x}"))
            start = bank * bank_depth
            write_words(output, words[start : start + bank_depth], args.format)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
